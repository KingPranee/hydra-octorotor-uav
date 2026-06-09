"""
hydra_grpc_server.py
════════════════════
Project HYDRA v4 — gRPC Telemetry & Command Server

Implements the HydraControl bidirectional streaming service.
The ground station (client) can:
  1. Stream MotorSystemStatus protos from the companion at 5 Hz.
  2. Send ConfigUpdate protos to change threshold/PDD params in-flight.
  3. Send CommandRequest protos to trigger emergency shutdown,
     force enable, or request a state snapshot.

Since we cannot compile .proto files in this environment the service
is implemented using grpc reflection and a pure-Python proto-like
dataclass encoding over raw bytes.  In production, generate proper
stubs from hydra.proto with:
    python3 -m grpc_tools.protoc -I./proto --python_out=. --grpc_python_out=. proto/hydra.proto

PROTO DEFINITION (for reference — hydra.proto):
───────────────────────────────────────────────
syntax = "proto3";
package hydra;

service HydraControl {
  rpc StreamStatus(google.protobuf.Empty) returns (stream MotorSystemStatus);
  rpc SendCommand(CommandRequest) returns (CommandResponse);
  rpc UpdateConfig(ConfigUpdate) returns (ConfigUpdateAck);
  rpc BidirectionalControl(stream ControlFrame) returns (stream StatusFrame);
}

message MotorSystemStatus {
  double timestamp_unix     = 1;
  string fsm_state          = 2;
  int32  fsm_state_code     = 3;
  int32  transition_count   = 4;
  double weight_raw_g       = 5;
  double ukf_mass_g         = 6;
  double ukf_uncertainty_g  = 7;
  double mhdd_posterior     = 8;
  double pid_output         = 9;
  double threshold_g        = 10;
  double hysteresis_g       = 11;
  string mission_phase      = 12;
  double vibration_level    = 13;
  double loop_hz            = 14;
  repeated MotorStatus motors = 15;
  bool   pixhawk_hb_ok      = 16;
  bool   ate_locked         = 17;
  double ate_baseline_g     = 18;
}

message MotorStatus {
  int32  index      = 1;
  int32  rpm        = 2;
  double health     = 3;
  double temp_c     = 4;
  double current_a  = 5;
  bool   degrading  = 6;
  bool   enabled    = 7;
}

message CommandRequest {
  enum Command {
    NOP                = 0;
    EMERGENCY_SHUTDOWN = 1;
    FORCE_ENABLE_ALL   = 2;
    FORCE_QUAD         = 3;
    SNAPSHOT           = 4;
    ARM                = 5;
    DISARM             = 6;
    RTL                = 7;
  }
  Command command = 1;
  string  reason  = 2;
}

message CommandResponse {
  bool   ok      = 1;
  string message = 2;
}

message ConfigUpdate {
  double threshold_g           = 1;
  double hysteresis_g          = 2;
  double mhdd_posterior_thresh = 3;
  double pid_kp                = 4;
  double pid_ki                = 5;
  double pid_kd                = 6;
  double pid_commit_threshold  = 7;
}

message ConfigUpdateAck {
  bool   ok      = 1;
  string message = 2;
}
───────────────────────────────────────────────
"""

from __future__ import annotations

import json
import logging
import struct
import threading
import time
from dataclasses import dataclass, field, asdict
from typing import Callable, Dict, Iterator, List, Optional, Tuple

log = logging.getLogger("hydra.grpc")

# ─── Try grpc ────────────────────────────────────────────────────────────────
try:
    import grpc
    from concurrent import futures
    GRPC_OK = True
except ImportError:
    GRPC_OK = False
    log.warning("[gRPC] grpc not installed — server disabled")


# ════════════════════════════════════════════════════════════════════════════
#  PURE-PYTHON MESSAGE STRUCTS
#  Used internally regardless of protobuf availability.
#  JSON-serialisable so we can also serve over the Prometheus HTTP path.
# ════════════════════════════════════════════════════════════════════════════

@dataclass
class MotorStatusMsg:
    index:     int   = 0
    rpm:       int   = 0
    health:    float = 1.0
    temp_c:    float = 0.0
    current_a: float = 0.0
    degrading: bool  = False
    enabled:   bool  = True

@dataclass
class MotorSystemStatusMsg:
    timestamp_unix:    float = 0.
    fsm_state:         str   = "OCTOCOPTER"
    fsm_state_code:    int   = 0
    transition_count:  int   = 0
    weight_raw_g:      float = 0.
    ukf_mass_g:        float = 0.
    ukf_uncertainty_g: float = 0.
    mhdd_posterior:    float = 0.
    pid_output:        float = 0.
    threshold_g:       float = 500.
    hysteresis_g:      float = 75.
    mission_phase:     str   = "UNKNOWN"
    vibration_level:   float = 0.
    loop_hz:           float = 5.
    motors:            List[MotorStatusMsg] = field(
        default_factory=lambda: [MotorStatusMsg(index=i) for i in range(1,9)])
    pixhawk_hb_ok:     bool  = True
    ate_locked:        bool  = False
    ate_baseline_g:    float = 0.

    def to_json(self) -> str:
        d = asdict(self)
        return json.dumps(d, separators=(",", ":"))

    def to_bytes(self) -> bytes:
        """
        Simple TLV binary encoding — tag(1B) + length(2B) + value.
        Used for the raw gRPC bytes stub until proper protobuf is generated.
        """
        data = self.to_json().encode("utf-8")
        return struct.pack(">BH", 0x01, len(data)) + data

    @classmethod
    def from_bytes(cls, raw: bytes) -> "MotorSystemStatusMsg":
        tag, length = struct.unpack_from(">BH", raw)
        payload = raw[3: 3 + length].decode("utf-8")
        d = json.loads(payload)
        motors = [MotorStatusMsg(**m) for m in d.pop("motors", [])]
        return cls(**d, motors=motors)


@dataclass
class CommandRequestMsg:
    command: str = "NOP"   # "EMERGENCY_SHUTDOWN" | "FORCE_ENABLE_ALL" | etc.
    reason:  str = ""

@dataclass
class ConfigUpdateMsg:
    threshold_g:            Optional[float] = None
    hysteresis_g:           Optional[float] = None
    mhdd_posterior_thresh:  Optional[float] = None
    pid_kp:                 Optional[float] = None
    pid_ki:                 Optional[float] = None
    pid_kd:                 Optional[float] = None
    pid_commit_threshold:   Optional[float] = None


# ════════════════════════════════════════════════════════════════════════════
#  STATUS BROADCASTER
#  The companion's main loop pushes status updates here.
#  All gRPC streaming handlers subscribe and receive updates.
# ════════════════════════════════════════════════════════════════════════════

class StatusBroadcaster:
    def __init__(self):
        self._lock        = threading.Lock()
        self._latest: Optional[MotorSystemStatusMsg] = None
        self._subscribers: List[threading.Event] = []

    def publish(self, status: MotorSystemStatusMsg) -> None:
        with self._lock:
            self._latest = status
            for ev in self._subscribers:
                ev.set()

    def subscribe(self) -> "StatusSubscription":
        ev = threading.Event()
        with self._lock:
            self._subscribers.append(ev)
        return StatusSubscription(self, ev)

    def unsubscribe(self, ev: threading.Event) -> None:
        with self._lock:
            try: self._subscribers.remove(ev)
            except ValueError: pass

    @property
    def latest(self) -> Optional[MotorSystemStatusMsg]:
        with self._lock:
            return self._latest


class StatusSubscription:
    def __init__(self, parent: StatusBroadcaster, ev: threading.Event):
        self._parent = parent
        self._ev     = ev
        self._closed = False

    def wait_and_get(self, timeout_s: float = 1.0) \
            -> Optional[MotorSystemStatusMsg]:
        self._ev.wait(timeout=timeout_s)
        self._ev.clear()
        return self._parent.latest

    def close(self) -> None:
        if not self._closed:
            self._parent.unsubscribe(self._ev)
            self._closed = True


# ════════════════════════════════════════════════════════════════════════════
#  COMMAND DISPATCHER
#  Maps command strings to FSM callbacks registered by the main loop.
# ════════════════════════════════════════════════════════════════════════════

class CommandDispatcher:
    def __init__(self):
        self._handlers: Dict[str, Callable[[str], bool]] = {}

    def register(self, command: str,
                 handler: Callable[[str], bool]) -> None:
        self._handlers[command] = handler
        log.debug("[gRPC] Registered handler for %s", command)

    def dispatch(self, cmd: CommandRequestMsg) -> Tuple[bool, str]:
        fn = self._handlers.get(cmd.command)
        if fn is None:
            msg = f"Unknown command: {cmd.command}"
            log.warning("[gRPC] %s", msg)
            return False, msg
        try:
            ok = fn(cmd.reason)
            return ok, "OK" if ok else "Handler returned False"
        except Exception as e:
            log.error("[gRPC] Command %s error: %s", cmd.command, e)
            return False, str(e)

    def apply_config(self, upd: ConfigUpdateMsg, cfg) -> Tuple[bool, str]:
        """Apply ConfigUpdateMsg fields to the live Config object."""
        changed = []
        for attr in (
            "threshold_g", "hysteresis_g", "mhdd_posterior_thresh",
            "pid_kp", "pid_ki", "pid_kd", "pid_commit_threshold"
        ):
            val = getattr(upd, attr, None)
            if val is not None:
                old = getattr(cfg, attr, None)
                setattr(cfg, attr, val)
                changed.append(f"{attr}: {old}→{val}")
        if changed:
            msg = "Updated: " + ", ".join(changed)
            log.info("[gRPC] Config update — %s", msg)
            return True, msg
        return True, "No fields changed"


# ════════════════════════════════════════════════════════════════════════════
#  GRPC SERVICE IMPLEMENTATION  (raw bytes stub)
#
#  Since proper protobuf stubs require offline code generation, this
#  implements the service in terms of our internal dataclasses.
#  Replace the _encode / _decode methods with proper proto serialize/parse
#  once hydra_pb2 is generated.
# ════════════════════════════════════════════════════════════════════════════


# ════════════════════════════════════════════════════════════════════════════
#  GRPC SERVICE  —  HydraServicer + start_grpc_server
#
#  HydraServicer is only defined when grpc is available.
#  start_grpc_server is defined unconditionally: it returns None immediately
#  when grpc is absent, so callers never need to guard the call themselves.
# ════════════════════════════════════════════════════════════════════════════

class HydraServicer:
    """
    Implements HydraControl service over a raw-bytes channel.
    Replace bytes encoding with proper proto serialization once
    hydra_pb2 is generated from hydra.proto.
    This class is instantiated only when grpc is available (checked inside
    start_grpc_server), so it is always safe to define at module level.
    """

    def __init__(self, broadcaster: StatusBroadcaster,
                 dispatcher: CommandDispatcher, cfg) -> None:
        self._bc   = broadcaster
        self._disp = dispatcher
        self._cfg  = cfg

    # ── Server-streaming: push MotorSystemStatus at ~5 Hz ────────────────────
    def StreamStatus(self, request, context):
        log.info("[gRPC] StreamStatus connected: %s", context.peer())
        sub = self._bc.subscribe()
        try:
            while context.is_active():
                status = sub.wait_and_get(timeout_s=1.0)
                if status is not None:
                    yield status.to_bytes()
        finally:
            sub.close()
            log.info("[gRPC] StreamStatus disconnected: %s", context.peer())

    # ── Unary: execute a command on the FSM ──────────────────────────────────
    def SendCommand(self, request, context) -> bytes:
        raw = request.decode("utf-8") if isinstance(request, bytes) else request
        try:
            msg = CommandRequestMsg(**json.loads(raw))
        except Exception as e:
            log.error("[gRPC] SendCommand parse error: %s", e)
            return json.dumps({"ok": False, "message": str(e)}).encode()
        ok, reply = self._disp.dispatch(msg)
        log.info("[gRPC] Command %s → ok=%s  %s", msg.command, ok, reply)
        return json.dumps({"ok": ok, "message": reply}).encode()

    # ── Unary: apply in-flight config update ─────────────────────────────────
    def UpdateConfig(self, request, context) -> bytes:
        raw = request.decode("utf-8") if isinstance(request, bytes) else request
        try:
            d = json.loads(raw)
            upd = ConfigUpdateMsg(**{k: v for k, v in d.items()
                                      if hasattr(ConfigUpdateMsg, k)})
        except Exception as e:
            return json.dumps({"ok": False, "message": str(e)}).encode()
        ok, msg = self._disp.apply_config(upd, self._cfg)
        return json.dumps({"ok": ok, "message": msg}).encode()

    # ── Bidi-streaming: interleaved commands and status frames ────────────────
    def BidirectionalControl(self, request_iterator, context):
        log.info("[gRPC] BidiControl connected: %s", context.peer())
        sub = self._bc.subscribe()
        try:
            for raw_frame in request_iterator:
                if not context.is_active():
                    break
                try:
                    frame = json.loads(
                        raw_frame.decode("utf-8") if isinstance(raw_frame, bytes)
                        else raw_frame)
                    ftype = frame.get("type", "")
                    if ftype == "command":
                        cmd = CommandRequestMsg(**frame.get("payload", {}))
                        self._disp.dispatch(cmd)
                    elif ftype == "config":
                        upd = ConfigUpdateMsg(**frame.get("payload", {}))
                        self._disp.apply_config(upd, self._cfg)
                except Exception as e:
                    log.warning("[gRPC] BidiControl frame error: %s", e)

                status = self._bc.latest
                if status is not None:
                    yield status.to_bytes()
        finally:
            sub.close()
            log.info("[gRPC] BidiControl disconnected: %s", context.peer())


def start_grpc_server(cfg,
                      broadcaster: StatusBroadcaster,
                      dispatcher: CommandDispatcher,
                      port: int = 50051):
    """
    Start the HYDRA gRPC server.

    Returns the grpc.Server object on success, or None if grpc is not
    installed or the server fails to bind.

    This function is always safe to call — it checks GRPC_OK internally
    and logs a warning rather than raising if grpc is absent.
    Replace the _GenericHandler with proper add_HydraControlServicer_to_server
    once hydra_pb2_grpc is generated from hydra.proto.
    """
    if not GRPC_OK:
        log.warning("[gRPC] grpc package not installed — server not started")
        return None

    servicer = HydraServicer(broadcaster, dispatcher, cfg)

    class _GenericHandler(grpc.GenericRpcHandler):
        """
        Routes incoming RPCs to the correct servicer method with the
        correct handler type for each RPC pattern.
        """
        def service_name(self) -> str:
            return "hydra.HydraControl"

        def service(self, handler_call_details):
            method = handler_call_details.method

            if method == "/hydra.HydraControl/StreamStatus":
                # Unary request → server-streaming response
                return grpc.unary_stream_rpc_method_handler(
                    servicer.StreamStatus,
                    request_deserializer=lambda b: b,
                    response_serializer=lambda r: r,
                )
            if method == "/hydra.HydraControl/SendCommand":
                # Unary request → unary response
                return grpc.unary_unary_rpc_method_handler(
                    servicer.SendCommand,
                    request_deserializer=lambda b: b,
                    response_serializer=lambda r: r,
                )
            if method == "/hydra.HydraControl/UpdateConfig":
                # Unary request → unary response
                return grpc.unary_unary_rpc_method_handler(
                    servicer.UpdateConfig,
                    request_deserializer=lambda b: b,
                    response_serializer=lambda r: r,
                )
            if method == "/hydra.HydraControl/BidirectionalControl":
                # Client-streaming → server-streaming (bidi)
                return grpc.stream_stream_rpc_method_handler(
                    servicer.BidirectionalControl,
                    request_deserializer=lambda b: b,
                    response_serializer=lambda r: r,
                )
            return None

    try:
        server = grpc.server(
            futures.ThreadPoolExecutor(max_workers=4),
            options=[
                ("grpc.max_send_message_length",         4 * 1024 * 1024),
                ("grpc.max_receive_message_length",       4 * 1024 * 1024),
                ("grpc.keepalive_time_ms",               10_000),
                ("grpc.keepalive_timeout_ms",             5_000),
                ("grpc.keepalive_permit_without_calls",   True),
                ("grpc.http2.max_pings_without_data",     0),
            ],
        )
        server.add_generic_rpc_handlers([_GenericHandler()])
        bound = server.add_insecure_port(f"[::]:{port}")
        if bound == 0:
            log.error("[gRPC] Failed to bind port %d", port)
            return None
        server.start()
        log.info("[gRPC] HydraControl server listening on port %d", port)
        return server
    except Exception as e:
        log.error("[gRPC] Server start failed: %s", e)
        return None


# ════════════════════════════════════════════════════════════════════════════
#  SIMPLE CLI GROUND CLIENT  (test / debug)
#
#  Usage:
#    python3 hydra_grpc_server.py --client --host 192.168.1.10 --port 50051
# ════════════════════════════════════════════════════════════════════════════

def _run_client(host: str, port: int) -> None:
    """
    Minimal ground station client — connects and prints streamed status.
    For production use, generate stubs and use the proper proto client.
    """
    if not GRPC_OK:
        print("grpc not installed — cannot run client")
        return

    import socket, json, struct

    addr = (host, port)
    print(f"Connecting to HYDRA gRPC server at {host}:{port} ...")

    channel = grpc.insecure_channel(f"{host}:{port}")
    grpc.channel_ready_future(channel).result(timeout=10)
    print("Connected.\n")

    stub_bytes = channel.unary_stream(
        "/hydra.HydraControl/StreamStatus",
        request_serializer=lambda x: b"",
        response_deserializer=lambda b: b)

    print(f"{'TIME':8s}  {'STATE':22s}  {'MASS':8s}  {'POST':6s}  "
          f"{'PHASE':18s}  {'HZ':5s}  {'VIBE':5s}")
    print("-" * 90)
    try:
        for raw in stub_bytes(b""):
            try:
                msg = MotorSystemStatusMsg.from_bytes(raw)
                ts  = time.strftime("%H:%M:%S", time.localtime(msg.timestamp_unix))
                print(f"{ts:8s}  {msg.fsm_state:22s}  "
                      f"{msg.ukf_mass_g:7.1f}g  "
                      f"{msg.mhdd_posterior:5.3f}  "
                      f"{msg.mission_phase:18s}  "
                      f"{msg.loop_hz:4.1f}  "
                      f"{msg.vibration_level:5.1f}")
            except Exception as e:
                print(f"Parse error: {e}")
    except KeyboardInterrupt:
        print("\nClient disconnected.")
    channel.close()


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--client", action="store_true")
    p.add_argument("--host",   default="127.0.0.1")
    p.add_argument("--port",   type=int, default=50051)
    a = p.parse_args()
    if a.client:
        _run_client(a.host, a.port)
    else:
        print("Import this module from hydra_companion_v4.py.")
        print("To test the client: python3 hydra_grpc_server.py "
              "--client --host <rpi-ip>")
