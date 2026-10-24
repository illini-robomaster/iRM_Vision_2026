#!/usr/bin/env python3
"""Isolated vcan acceptance: real relative_aim + independent CBoard peer, stdlib only.

Run inside a fresh user/network namespace; never accepts physical CAN interfaces.
"""
import argparse
import csv
import json
import pathlib
import signal
import socket
import struct
import subprocess
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
FRAME = struct.Struct("=IB3x8s")
SESSION = 0x12345678


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="/tmp/vision_cboard_vcan")
    args = parser.parse_args()
    # ip's JSON kind is a second, independent protection in addition to the C++ check.
    info = json.loads(subprocess.check_output(
        ["ip", "-j", "-details", "link", "show", "vcan_test"], text=True))
    if info[0].get("linkinfo", {}).get("info_kind") != "vcan":
        raise RuntimeError("vcan_test is not a verified vcan interface")
    prefix = pathlib.Path(args.output)
    peer = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    peer.bind(("vcan_test",))
    peer.settimeout(0.02)
    events = []
    seq = 0

    def send(can_id, payload):
        peer.send(FRAME.pack(can_id, 8, payload))

    def hello(session=SESSION):
        send(0x301, struct.pack(">BBIBB", 1, 1, session, 0, 0))

    def control(enable, color=2, session=SESSION):
        nonlocal seq
        seq = (seq + 1) & 255
        send(0x303, struct.pack(">BBIBB", 3, 1, session, int(enable) | (color << 1), seq))

    def collect(duration, heartbeat=None):
        result = []
        deadline = time.monotonic() + duration
        next_control = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if heartbeat and now >= next_control:
                heartbeat()
                next_control = now + 0.08
            try:
                raw = peer.recv(FRAME.size)
            except socket.timeout:
                continue
            can_id, dlc, data = FRAME.unpack(raw)
            event = {"time": time.monotonic(), "id": can_id, "dlc": dlc, "hex": data.hex()}
            events.append(event)
            result.append(event)
            if can_id == 0x300:
                assert dlc == 8 and data[0] in (0, 1)
                assert data[1] == 0 and data[6:] == b"\x00\x00"
                if data[0] == 0:
                    assert data == bytes(8), "stop contains stale angles"
        return result

    def valid(rows):
        return [row for row in rows if row["id"] == 0x300 and row["hex"].startswith("01")]

    def stopped(rows):
        return any(row["id"] == 0x300 and row["hex"] == "00" * 8 for row in rows)

    with open(str(prefix) + ".log", "w") as log:
        proc = subprocess.Popen(
            [str(ROOT / "build/relative_aim"), "-c=" + str(ROOT / "configs/relative_vcan.yaml"),
             "-csv=" + str(prefix) + ".csv"], cwd=str(ROOT), stdout=log, stderr=subprocess.STDOUT)
        try:
            # Model construction may take seconds. HELLO does not keep a session alive.
            deadline = time.monotonic() + 25
            ready = False
            while time.monotonic() < deadline and proc.poll() is None:
                hello()
                rows = collect(0.1)
                ready = any(row["id"] == 0x302 and
                            row["hex"] == "0201123456780000" for row in rows)
                if ready:
                    break
            assert ready, "READY not received (inspect model/backend log)"
            assert not valid(collect(0.12)), "valid before CONTROL"
            active = collect(1.5, lambda: control(True))
            assert valid(active), "no blue target output"
            print("PASS handshake + enable blue: {} valid frames".format(len(valid(active))))
            disabled = collect(0.3, lambda: control(False))
            assert stopped(disabled), "disable did not stop"
            stop_index = next(i for i, row in enumerate(disabled)
                              if row["id"] == 0x300 and row["hex"] == "00" * 8)
            assert not valid(disabled[stop_index:]), "valid frame after disable stop"
            print("PASS disable + zero angles")
            red = collect(0.5, lambda: control(True, 1))
            assert not valid(red), "blue demo incorrectly accepted after switch to red"
            blue = collect(0.7, lambda: control(True))
            assert valid(blue), "blue did not resume"
            print("PASS color red filters blue + blue resumes")
            timeout_rows = collect(0.8)
            assert stopped(timeout_rows), "missing timeout stop"
            assert not valid(collect(0.3, lambda: control(True))), "old session survived timeout"
            print("PASS timeout + old-session rejection")
            hello(SESSION + 1)
            collect(0.1)
            resumed = collect(0.8, lambda: control(True, session=SESSION + 1))
            assert valid(resumed), "new session failed"
            # Invalid reserved bits must fail closed, without waiting 500 ms.
            send(0x303, struct.pack(">BBIBB", 3, 1, SESSION + 1, 0x85, 100))
            malformed = collect(0.15)
            assert stopped(malformed), "malformed control did not stop immediately"
            print("PASS re-handshake + malformed fail-closed")
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                shutdown = collect(0.25)
                assert stopped(shutdown), "missing shutdown stop frame"
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
                raise RuntimeError("normal SIGINT shutdown exceeded 10s")
            with open(str(prefix) + ".json", "w") as trace:
                json.dump(events, trace, indent=2)
            peer.close()
        assert proc.returncode == 0, "relative_aim failed: {}".format(proc.returncode)
    # Every actual valid frame must occur in publication order in the independent CSV.
    with open(str(prefix) + ".csv", newline="") as source:
        published = [(int(row["tx_yaw_1e4"]), int(row["tx_pitch_1e4"]))
                     for row in csv.DictReader(source) if row["publish_ok"] == "1"]
    cursor = 0
    for row in valid(events):
        angles = struct.unpack(">hh", bytes.fromhex(row["hex"])[2:6])
        while cursor < len(published) and published[cursor] != angles:
            cursor += 1
        assert cursor < len(published), "wire frame absent/out of order in published CSV"
        cursor += 1
    print("PASS wire int16 angles match published CSV order + SIGINT shutdown stop")
    print("VCAN PASS; trace/log/CSV prefix: " + str(prefix))


if __name__ == "__main__":
    main()