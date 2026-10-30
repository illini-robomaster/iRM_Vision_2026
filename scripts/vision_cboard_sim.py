#!/usr/bin/env python3
"""CBoard (MCU) simulator for the vision <-> CBoard v1 relative-angle link.

Two modes, both restricted to a **verified vcan** interface (never physical CAN):

  acceptance (default)
      Scripted end-to-end run against ``build/relative_aim`` (spawned unless
      ``--no-spawn``): periodic HELLO + READY ACK, CONTROL enable/disable/colour/
      session changes, CONTROL-heartbeat loss, malformed CONTROL, one frame per
      observation, independent stop while image acquisition is throttled, wire/
      CSV order agreement, and the SIGINT shutdown stop.

  --interactive
      Manual console against an already running vision process. Commands:
      ``hello``, ``enable``, ``disable``, ``color red|blue``, ``session new|0x...``,
      ``hb on|off``, ``status``, ``send <id> <hex>``, ``quit``. Every received
      angle frame is decoded and printed with its receive time.

Only the Python standard library is used. The four CAN IDs, the interface and the
two timeouts are read from the vision yaml, so the peer cannot drift from the
configuration. Candidate IDs in ``configs/relative_current.yaml`` are NOT hardware
assignments; this tool never satisfies ``vision_can_ids_confirmed`` for a real bus.

Typical isolated run (new user+network namespace, nothing touches can0)::

    unshare -rn bash -c 'ip link add dev vcan_test type vcan &&
        ip link set up vcan_test &&
        exec python3 scripts/vision_cboard_sim.py'

Attach to an already running ``relative_aim`` instead of spawning one::

    python3 scripts/vision_cboard_sim.py --no-spawn --interactive
"""

import argparse
import csv
import json
import pathlib
import random
import select
import signal
import socket
import struct
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]

# struct can_frame (classic CAN): id u32, dlc u8, 3 pad bytes, 8 data bytes.
FRAME = struct.Struct("=IB3x8s")

MSG_HELLO = 0x01
MSG_ACK = 0x02
MSG_CONTROL = 0x03
VERSION = 1
STATUS_READY = 0
STATUS_NOT_READY = 1
STATUS_VERSION_MISMATCH = 2
COLOR_CODES = {"red": 1, "blue": 2}

DEFAULT_CONFIG = "configs/relative_vcan.yaml"
DEFAULT_SESSION = 0x12345678


# --------------------------------------------------------------------------- #
# Configuration: top-level `key: value` scalars only (no PyYAML dependency).
# --------------------------------------------------------------------------- #


def read_yaml_scalars(path):
    """Return the first top-level ``key: value`` scalar for each key."""
    values = {}
    for raw in pathlib.Path(path).read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or ":" not in line or line.startswith("- "):
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key and value and key not in values:
            values[key] = value
    return values


def parse_int(value):
    if isinstance(value, int):
        return value
    text = str(value).strip()
    try:
        return int(text, 0)  # handles 0x1FF and decimal
    except ValueError:
        raise SystemExit("not an integer: {!r}".format(text))


def load_config(path, interface_override=None):
    """Read the IDs/timeouts the peer needs; missing keys are a hard error."""
    scalars = read_yaml_scalars(path)
    ids = {}
    for name in ("relative_canid", "vision_handshake_request_canid",
                 "vision_handshake_response_canid", "vision_control_canid"):
        if name not in scalars:
            raise SystemExit("{}: missing '{}'".format(path, name))
        can_id = parse_int(scalars[name])
        if not 0 <= can_id <= 0x7FF:
            raise SystemExit("{}: '{}' = {} is not a standard 11-bit ID".format(
                path, name, can_id))
        ids[name] = can_id
    if len(set(ids.values())) != 4:
        raise SystemExit("{}: the four ID keys must be distinct: {}".format(path, ids))

    return {
        "path": str(path),
        "interface": interface_override or scalars.get("can_interface") or "vcan_test",
        "relative": ids["relative_canid"],
        "hello": ids["vision_handshake_request_canid"],
        "ack": ids["vision_handshake_response_canid"],
        "control": ids["vision_control_canid"],
        "control_timeout_ms": float(scalars.get("vision_control_timeout_ms", 500)),
        "frame_timeout_ms": float(scalars.get("relative_frame_timeout_ms", 100)),
        "can_command_mode": scalars.get("can_command_mode", ""),
        "vision_protocol": scalars.get("vision_protocol", "v1"),
        "ids_confirmed": scalars.get("vision_can_ids_confirmed", "false"),
    }


def require_vcan(interface):
    """`ip -j -details` must report linkinfo.info_kind == vcan for this interface."""
    try:
        info = json.loads(subprocess.check_output(
            ["ip", "-j", "-details", "link", "show", "dev", interface], text=True))
    except (subprocess.CalledProcessError, FileNotFoundError) as exc:
        raise SystemExit("cannot inspect {}: {}".format(interface, exc))
    if not info or info[0].get("linkinfo", {}).get("info_kind") != "vcan":
        raise SystemExit("{} is not a verified vcan interface; refusing to use it"
                         .format(interface))


# --------------------------------------------------------------------------- #
# MCU peer
# --------------------------------------------------------------------------- #


class Peer:
    """Minimal CBoard stand-in: HELLO, CONTROL heartbeat, angle-frame decoder."""

    def __init__(self, config, session=DEFAULT_SESSION, color="blue", verbose=False):
        self.config = config
        self.session = session & 0xFFFFFFFF
        self.enable = False
        self.color = COLOR_CODES[color]
        self.heartbeat = True
        self.heartbeat_s = 0.08
        self.verbose = verbose
        self.seq = 0
        self.t0 = time.monotonic()
        self.records = []
        self.last_relative_t = None

        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((config["interface"],))
        self.sock.settimeout(0.02)

    # ---- transmit -----------------------------------------------------------

    def _send(self, can_id, payload):
        if len(payload) != 8:
            raise ValueError("CAN payload must be 8 bytes, got {}".format(len(payload)))
        self.sock.send(FRAME.pack(can_id, 8, payload))

    def hello(self, session=None):
        if session is not None:
            self.session = session & 0xFFFFFFFF
        self._send(self.config["hello"],
                   struct.pack(">BBIBB", MSG_HELLO, VERSION, self.session, 0, 0))

    def control(self, enable=None, color=None):
        if enable is not None:
            self.enable = bool(enable)
        if color is not None:
            self.color = COLOR_CODES[color]
        self.seq = (self.seq + 1) & 0xFF
        flags = int(self.enable) | (self.color << 1)
        self._send(self.config["control"],
                   struct.pack(">BBIBB", MSG_CONTROL, VERSION, self.session, flags, self.seq))

    def malformed_control(self, flags=0x85, sequence=100):
        """Structurally illegal CONTROL: reserved bits set -> vision must fail closed."""
        self._send(self.config["control"],
                   struct.pack(">BBIBB", MSG_CONTROL, VERSION, self.session, flags, sequence))

    def send_raw(self, can_id, payload):
        self._send(can_id, payload)

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass

    # ---- receive ------------------------------------------------------------

    @staticmethod
    def decode_relative(payload):
        control = bool(payload[0] & 0x01)
        dyaw, dpitch = struct.unpack(">hh", payload[2:6])
        return control, dyaw / 1e4, dpitch / 1e4

    def _handle(self, raw):
        can_id, dlc, data = FRAME.unpack(raw)
        now = time.monotonic()
        row = {"t": now - self.t0, "id": can_id, "dlc": dlc, "hex": data.hex()}
        if can_id == self.config["relative"]:
            row["kind"] = "relative"
            control, dyaw, dpitch = self.decode_relative(data)
            row["control"] = control
            row["dyaw"] = dyaw
            row["dpitch"] = dpitch
            gap = 0.0 if self.last_relative_t is None else (now - self.last_relative_t) * 1000.0
            self.last_relative_t = now
            self.records.append(row)
            if self.verbose:
                if control:
                    print("[{:8.3f}s | +{:7.1f}ms] 0x{:03X} dyaw={:+.4f} rad ({:+.2f} deg) "
                          "dpitch={:+.4f} rad ({:+.2f} deg)".format(
                              row["t"], gap, can_id, dyaw, dyaw * 57.29577951308232,
                              dpitch, dpitch * 57.29577951308232), flush=True)
                else:
                    print("[{:8.3f}s | +{:7.1f}ms] 0x{:03X} stop (control=0, all-zero)".format(
                        row["t"], gap, can_id), flush=True)
        else:
            row["kind"] = "ack" if can_id == self.config["ack"] else "other"
            self.records.append(row)
            if self.verbose:
                print("[{:8.3f}s] 0x{:03X} {} {}".format(row["t"], can_id, row["kind"],
                                                         row["hex"]), flush=True)
        return row

    def pump(self, duration):
        """Collect frames for `duration` seconds, sending the CONTROL heartbeat."""
        rows = []
        deadline = time.monotonic() + duration
        next_control = 0.0
        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            if self.heartbeat and now >= next_control:
                self.control()
                next_control = now + self.heartbeat_s
            try:
                raw = self.sock.recv(FRAME.size)
            except socket.timeout:
                continue
            rows.append(self._handle(raw))
        return rows


# --------------------------------------------------------------------------- #
# Row predicates
# --------------------------------------------------------------------------- #


def is_ready_ack(row, config, session):
    if row["id"] != config["ack"]:
        return False
    data = bytes.fromhex(row["hex"])
    return (data[0] == MSG_ACK and data[1] == VERSION and
            struct.unpack(">I", data[2:6])[0] == session and
            data[6] == STATUS_READY and data[7] == 0)


def ack_rows(rows, config):
    return [row for row in rows if row["id"] == config["ack"]]


def valid_rows(rows, config):
    return [row for row in rows
            if row["id"] == config["relative"] and bytes.fromhex(row["hex"])[0] & 0x01]


def stop_rows(rows, config):
    return [row for row in rows
            if row["id"] == config["relative"] and bytes.fromhex(row["hex"]) == bytes(8)]


def rel_rows(rows, config):
    return [row for row in rows if row["id"] == config["relative"]]


# --------------------------------------------------------------------------- #
# Scripted acceptance
# --------------------------------------------------------------------------- #


def step(text):
    print("PASS " + text, flush=True)


def spawn_vision(config_path, prefix):
    log_path = pathlib.Path(str(prefix) + ".log")
    log = open(log_path, "w", encoding="utf-8")
    proc = subprocess.Popen(
        [str(ROOT / "build/relative_aim"), "-c=" + str(config_path),
         "-csv=" + str(prefix) + ".csv"],
        cwd=str(ROOT), stdout=log, stderr=subprocess.STDOUT)
    return proc, log, log_path


def close_vision(proc, log):
    if proc.poll() is None:
        proc.kill()
        proc.wait()
    if log is not None:
        log.close()


def shutdown_vision(proc, peer, timeout_s=5.0):
    """SIGINT must produce a final stop frame, then a clean exit.

    The wait is bounded by a deadline rather than a fixed window: at a throttled
    image rate the main loop may sit in `camera.read_for()` across the signal.
    """
    started = time.monotonic()
    proc.send_signal(signal.SIGINT)
    latency = None
    deadline = started + timeout_s
    while time.monotonic() < deadline:
        fresh = peer.pump(0.05)
        if stop_rows(fresh, peer.config):
            latency = (time.monotonic() - started) * 1000.0
            break
        if proc.poll() is not None:
            break
    if latency is None:
        raise AssertionError("missing SIGINT shutdown stop frame within {:.0f}s"
                             .format(timeout_s))
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        raise AssertionError("normal SIGINT shutdown exceeded 15s")
    if proc.returncode != 0:
        raise AssertionError("relative_aim exited with {}".format(proc.returncode))
    return latency


def handshake(peer, timeout_s, session=None):
    """Periodic HELLO until READY; model construction can take seconds.

    No CONTROL is sent here on purpose: the first scenario must show that a
    handshaken-but-not-enabled session produces no valid angle frame.
    """
    heartbeat, peer.heartbeat = peer.heartbeat, False
    try:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            peer.hello(session)
            rows = peer.pump(0.1)
            if any(is_ready_ack(row, peer.config, peer.session) for row in rows):
                return True
        return False
    finally:
        peer.heartbeat = heartbeat


def check_csv_order(prefix, config, records):
    """Every valid wire frame must appear, in order, among the published CSV rows."""
    with open(str(prefix) + ".csv", newline="") as source:
        published = [(int(row["tx_yaw_1e4"]), int(row["tx_pitch_1e4"]))
                     for row in csv.DictReader(source) if row["publish_ok"] == "1"]
    wire = valid_rows(records, config)
    if not wire:
        raise AssertionError("no valid wire frame to compare with the CSV")
    cursor = 0
    for row in wire:
        angles = struct.unpack(">hh", bytes.fromhex(row["hex"])[2:6])
        while cursor < len(published) and published[cursor] != angles:
            cursor += 1
        if cursor >= len(published):
            raise AssertionError("wire frame {} absent/out of order in the published CSV"
                                 .format(row["hex"]))
        cursor += 1
    step("wire int16 angles match the published CSV order ({} frames)".format(len(wire)))


def derive_slow_config(base_path, prefix, fps):
    """Copy the config with `video_frame_rate: <fps>` so images arrive slowly."""
    lines = pathlib.Path(base_path).read_text(encoding="utf-8").splitlines()
    out, found = [], False
    for line in lines:
        key = line.split("#", 1)[0].split(":", 1)[0].strip()
        if key == "video_frame_rate":
            out.append("video_frame_rate: {}".format(fps))
            found = True
        else:
            out.append(line)
    if not found:
        out.append("video_frame_rate: {}".format(fps))
    path = pathlib.Path(str(prefix) + ".slow.yaml")
    path.write_text("\n".join(out) + "\n", encoding="utf-8")
    return path



def run_acceptance(peer, args):
    config = peer.config

    if not handshake(peer, args.ready_timeout):
        raise AssertionError("READY not received (inspect the vision log)")
    step("handshake: READY ACK echoes session 0x{:08X}".format(peer.session))

    rows = peer.pump(0.15)
    if valid_rows(rows, config):
        raise AssertionError("valid angle frame before CONTROL/enable")
    step("gating: no valid angle frame before CONTROL (handshake only)")

    peer.heartbeat = True
    peer.control(enable=True, color="blue")
    active = peer.pump(1.5)
    if not valid_rows(active, config):
        raise AssertionError("no valid frame after enable blue")
    step("enable blue: {} valid frames, {} stops".format(
        len(valid_rows(active, config)), len(stop_rows(active, config))))

    peer.control(enable=False, color="blue")
    disabled = peer.pump(0.3)
    stops = stop_rows(disabled, config)
    if not stops:
        raise AssertionError("disable did not emit a stop frame")
    after = disabled[disabled.index(stops[0]):]
    if valid_rows(after, config):
        raise AssertionError("valid frame after the disable stop (in-flight observation leaked)")
    step("disable: immediate stop, in-flight observation dropped")

    peer.control(enable=True, color="red")
    red = peer.pump(0.5)
    if valid_rows(red, config):
        raise AssertionError("blue demo produced a valid frame after switching to red")
    peer.control(enable=True, color="blue")
    blue = peer.pump(0.7)
    if not valid_rows(blue, config):
        raise AssertionError("blue did not resume after the colour switch")
    step("colour switch red->blue: old-colour observations gated, blue resumes")

    # CONTROL silence -> control timeout; the old session must be dead afterwards.
    peer.heartbeat = False
    quiet_ms = config["control_timeout_ms"] + 350.0
    quiet = peer.pump(quiet_ms / 1000.0)
    if not stop_rows(quiet, config):
        raise AssertionError("CONTROL timeout did not emit a stop")
    peer.heartbeat = True  # heartbeat resumes with the OLD session
    stale = peer.pump(0.4)
    if valid_rows(stale, config):
        raise AssertionError("old session produced valid frames after the timeout")
    step("control timeout: stop emitted, old session rejected")

    if not handshake(peer, args.ready_timeout, session=peer.session + 1):
        raise AssertionError("re-handshake with a new session failed")
    peer.heartbeat = True
    resumed = peer.pump(0.9)
    if not valid_rows(resumed, config):
        raise AssertionError("new session did not resume valid output")
    step("new session 0x{:08X}: re-handshake and valid output".format(peer.session))

    peer.malformed_control()
    malformed = peer.pump(0.2)
    if not stop_rows(malformed, config):
        raise AssertionError("malformed CONTROL did not stop immediately")
    if valid_rows(malformed, config):
        raise AssertionError("malformed CONTROL still produced valid frames")
    step("malformed CONTROL: immediate fail-closed stop (no timeout wait)")


def run_slow_image_acceptance(peer, fps, seconds):
    """Images arrive far slower than relative_frame_timeout_ms.

    Proves two contract points on the wire:
      * one observation -> at most one valid frame (no repeating stream);
      * a stop appears when the observation expires, without waiting for the next
        image (transmission is independent of acquisition).
    """
    config = peer.config
    if not handshake(peer, 25.0):
        raise AssertionError("slow-image: READY not received")
    peer.heartbeat = True
    peer.control(enable=True, color="blue")
    rows = peer.pump(seconds)
    valids, stops = valid_rows(rows, config), stop_rows(rows, config)
    if not valids:
        raise AssertionError("slow-image ({} fps): no valid frame".format(fps))
    if len(stops) < len(valids):
        raise AssertionError(
            "slow-image: expected one stop per expired observation ({} valid, {} stops)"
            .format(len(valids), len(stops)))
    period = 1.0 / fps
    for prev, nxt in zip(valids, valids[1:]):
        if nxt["t"] - prev["t"] < period * 0.5:
            raise AssertionError(
                "slow-image: valid frames {:.3f}s apart at {} fps -> an observation was "
                "transmitted more than once".format(nxt["t"] - prev["t"], fps))
    step("slow image ({} fps): {} valid, {} stops, one frame per observation".format(
        fps, len(valids), len(stops)))



# --------------------------------------------------------------------------- #
# Interactive console
# --------------------------------------------------------------------------- #

INTERACTIVE_HELP = """commands:
  hello [session]       send HELLO (default: current session; new session otherwise)
  enable | disable      CONTROL with the current colour
  color red|blue        switch the requested target colour
  session new|0x...     change session (sends HELLO), resets the CONTROL sequence
  hb on|off             CONTROL heartbeat (80 ms) on/off
  status                peer state, counters, last decoded angle
  send <id> <hex>       raw frame (<=16 hex chars, zero padded) on any standard ID
  quit                  leave
"""


def summarise(peer):
    config = peer.config
    valids, stops = valid_rows(peer.records, config), stop_rows(peer.records, config)
    lines = ["session=0x{:08X} enable={} color={} heartbeat={} seq={}".format(
        peer.session, int(peer.enable), peer.color, int(peer.heartbeat), peer.seq),
        "rx: ack={} valid={} stop={}".format(
            len(ack_rows(peer.records, config)), len(valids), len(stops))]
    if valids:
        last = valids[-1]
        lines.append("last valid: t={:.3f}s dyaw={:+.4f} rad ({:+.2f} deg) "
                     "dpitch={:+.4f} rad ({:+.2f} deg)".format(
                         last["t"], last["dyaw"], last["dyaw"] * 57.29577951308232,
                         last["dpitch"], last["dpitch"] * 57.29577951308232))
    return "\n".join(lines)


def handle_command(peer, line):
    """Return False to leave the console."""
    parts = line.split()
    if not parts:
        return True
    cmd = parts[0].lower()

    if cmd in ("quit", "exit"):
        return False
    if cmd == "help":
        print(INTERACTIVE_HELP, flush=True)
    elif cmd == "hello":
        peer.hello(parse_int(parts[1]) if len(parts) > 1 else None)
        print("HELLO session=0x{:08X}".format(peer.session), flush=True)
    elif cmd == "enable":
        peer.control(enable=True)
        print("CONTROL enable=1 color={}".format(peer.color), flush=True)
    elif cmd == "disable":
        peer.control(enable=False)
        print("CONTROL enable=0 color={}".format(peer.color), flush=True)
    elif cmd == "color":
        if len(parts) < 2 or parts[1].lower() not in COLOR_CODES:
            print("usage: color red|blue", flush=True)
        else:
            peer.control(color=parts[1].lower())
            print("CONTROL color={}".format(peer.color), flush=True)
    elif cmd == "session":
        if len(parts) < 2:
            print("usage: session new|0x12345678", flush=True)
        else:
            value = random.getrandbits(32) if parts[1] == "new" else parse_int(parts[1])
            peer.seq = 0
            peer.hello(value)
            print("HELLO session=0x{:08X} (sequence reset)".format(peer.session), flush=True)
    elif cmd == "hb":
        if len(parts) < 2 or parts[1].lower() not in ("on", "off"):
            print("usage: hb on|off", flush=True)
        else:
            peer.heartbeat = parts[1].lower() == "on"
            print("heartbeat={}".format("on" if peer.heartbeat else "off"), flush=True)
    elif cmd == "status":
        print(summarise(peer), flush=True)
    elif cmd == "send":
        if len(parts) < 3:
            print("usage: send <id> <hex>", flush=True)
        else:
            can_id = parse_int(parts[1])
            payload = bytes.fromhex(parts[2].ljust(16, "0"))
            peer.send_raw(can_id, payload)
            print("sent 0x{:03X} {}".format(can_id, payload.hex()), flush=True)
    else:
        print("unknown command; type help", flush=True)
    return True


def run_interactive(peer):
    print(INTERACTIVE_HELP, flush=True)
    while True:
        ready, _, _ = select.select([sys.stdin], [], [], 0.05)
        if ready:
            line = sys.stdin.readline()
            if not line or not handle_command(peer, line.strip()):
                break
        peer.pump(0.05)
    print("console closed", flush=True)



# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #


def run_slow_scenario(prefix, args, config):
    slow_config = derive_slow_config(config["path"], prefix, args.slow_image_fps)
    slow = load_config(slow_config, config["interface"])
    peer = Peer(slow, session=parse_int(args.session), color="blue", verbose=args.verbose)
    proc, log, _ = spawn_vision(slow_config, str(prefix) + ".slow")
    try:
        run_slow_image_acceptance(peer, args.slow_image_fps, args.slow_seconds)
    finally:
        try:
            if proc.poll() is None:
                step("slow-image SIGINT shutdown: stop frame in {:.0f} ms".format(
                    shutdown_vision(proc, peer)))
        finally:
            close_vision(proc, log)
            peer.close()


def main():
    parser = argparse.ArgumentParser(
        description="CBoard (MCU) simulator for the vision<->CBoard v1 link (vcan only)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument("--config", default=DEFAULT_CONFIG,
                        help="vision yaml to read the IDs/timeouts from (default: %(default)s)")
    parser.add_argument("--interface", default=None,
                        help="override can_interface from the yaml")
    parser.add_argument("--session", default="0x12345678", help="initial session id")
    parser.add_argument("--color", choices=sorted(COLOR_CODES), default="blue",
                        help="initial target colour (default: %(default)s)")
    parser.add_argument("--output", default="/tmp/vision_cboard_vcan",
                        help="prefix for .log / .csv / .trace.json (default: %(default)s)")
    parser.add_argument("--interactive", action="store_true",
                        help="manual console instead of the scripted acceptance")
    parser.add_argument("--spawn", action="store_true",
                        help="also spawn build/relative_aim in --interactive mode")
    parser.add_argument("--no-spawn", action="store_true",
                        help="attach to an already running relative_aim")
    parser.add_argument("--verbose", action="store_true",
                        help="print every received frame (decoded angle + receive time)")
    parser.add_argument("--ready-timeout", type=float, default=25.0,
                        help="seconds to keep retrying HELLO before READY")
    parser.add_argument("--slow-image-fps", type=float, default=1.0,
                        help="image rate for the stall scenario, 0 disables it")
    parser.add_argument("--slow-seconds", type=float, default=4.5,
                        help="duration of the stall scenario")
    args = parser.parse_args()

    config = load_config(args.config, args.interface)
    require_vcan(config["interface"])

    print("MCU simulator on {}: relative=0x{:03X} hello=0x{:03X} ack=0x{:03X} "
          "control=0x{:03X}".format(config["interface"], config["relative"],
                                    config["hello"], config["ack"], config["control"]),
          flush=True)
    print("config={} mode={} protocol={} control_timeout={:.0f}ms frame_timeout={:.0f}ms "
          "ids_confirmed={}".format(config["path"], config["can_command_mode"],
                                    config["vision_protocol"], config["control_timeout_ms"],
                                    config["frame_timeout_ms"], config["ids_confirmed"]),
          flush=True)
    if config["vision_protocol"] != "v1":
        raise SystemExit("this peer implements the v1 handshake; vision_protocol is {!r}"
                         .format(config["vision_protocol"]))
    if config["can_command_mode"] != "relative":
        raise SystemExit("can_command_mode is {!r}; the relative link needs 'relative'"
                         .format(config["can_command_mode"]))
    if config["ids_confirmed"] == "true":
        print("NOTE: vision_can_ids_confirmed=true; these IDs are treated as approved. "
              "Only do this on a real bus after MCU sign-off.", flush=True)

    spawn = args.spawn if args.interactive else not args.no_spawn
    if spawn and not (ROOT / "build/relative_aim").exists():
        raise SystemExit("build/relative_aim missing: build it or pass --no-spawn")

    prefix = pathlib.Path(args.output)
    peer = Peer(config, session=parse_int(args.session), color=args.color, verbose=args.verbose)
    proc = log = None
    try:
        if spawn:
            proc, log, log_path = spawn_vision(config["path"], prefix)
            print("spawned relative_aim (log: {})".format(log_path), flush=True)
        if args.interactive:
            run_interactive(peer)
            return 0
        run_acceptance(peer, args)
        if proc is not None:
            latency = shutdown_vision(proc, peer)
            step("SIGINT shutdown: stop frame in {:.0f} ms, clean exit".format(latency))
            check_csv_order(prefix, config, peer.records)
        if args.slow_image_fps > 0 and spawn:
            run_slow_scenario(prefix, args, config)
        print("VCAN PASS; trace/log/CSV prefix: " + str(prefix), flush=True)
        return 0
    finally:
        if proc is not None:
            close_vision(proc, log)
        with open(str(prefix) + ".trace.json", "w", encoding="utf-8") as trace:
            json.dump(peer.records, trace, indent=2)
        peer.close()


if __name__ == "__main__":
    sys.exit(main())

