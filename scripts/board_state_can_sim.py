#!/usr/bin/env python3
"""Independent CBoard peer for board-state CAN v1 on a verified vcan interface.

Uses only Python's standard library and deliberately refuses physical CAN
interfaces. Pair with ./build/board_state_can_test -c=<vcan yaml>.
"""

import argparse
import json
import math
import socket
import struct
import subprocess
import time

FRAME = struct.Struct("=IB3x8s")
START = 1
ACK_COLOR = 2
FINAL_ACK = 3
VERSION = 1
READY = 0
NOT_READY = 1
VERSION_MISMATCH = 2
BLUE = 2


def require_vcan(interface):
    info = json.loads(subprocess.check_output(
        ["ip", "-j", "-details", "link", "show", "dev", interface], text=True))
    if not info or info[0].get("linkinfo", {}).get("info_kind") != "vcan":
        raise RuntimeError("{} is not a verified vcan interface".format(interface))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="vcan42")
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--not-ready-ms", type=int, default=0)
    parser.add_argument("--stall-after-ms", type=int, default=0,
                        help="pause telemetry after this many milliseconds (0 disables)")
    parser.add_argument("--stall-ms", type=int, default=1000)
    args = parser.parse_args()
    require_vcan(args.interface)

    peer = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    peer.bind((args.interface,))
    peer.settimeout(0.001)
    started = time.monotonic()
    deadline = started + args.seconds
    ready_after = started + args.not_ready_ms / 1000.0
    session = None
    response_due = None
    final_received = False
    sequence = 0
    next_sample = 0.0

    def send(can_id, data):
        assert len(data) == 8
        peer.send(FRAME.pack(can_id, 8, data))

    def send_response(now):
        if session is None:
            return
        status = READY if now >= ready_after else NOT_READY
        if session[1] != VERSION:
            status = VERSION_MISMATCH
        send(0x321, struct.pack(">BBIBB", ACK_COLOR, VERSION, session[0], status, BLUE))

    def send_sample(now):
        nonlocal sequence
        angle = 0.2 * math.sin(now)
        payload = struct.pack(
            "<5fB", math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2), 15.0, 1)
        assert len(payload) == 21
        for fragment in range(4):
            part = payload[fragment * 6:fragment * 6 + 6]
            header = bytes([(fragment << 5) | (sequence & 0x1F), session[0] & 0xFF])
            send(0x322, (header + part).ljust(8, b"\x00"))
        sequence = (sequence + 1) & 0x1F

    print("CBoard simulator listening on {}".format(args.interface), flush=True)
    while time.monotonic() < deadline:
        now = time.monotonic()
        try:
            raw = peer.recv(FRAME.size)
            can_id, dlc, data = FRAME.unpack(raw)
        except socket.timeout:
            can_id = None
            dlc = 0
            data = b""

        if can_id == 0x320 and dlc == 8:
            message_type = data[0]
            if message_type == START and data[6:] == b"\x00\x00":
                candidate = struct.unpack(">I", data[2:6])[0]
                version = data[1]
                if session is None or candidate != session[0]:
                    session = (candidate, version)
                    final_received = False
                    sequence = 0
                    print("START session=0x{:08x} version={}".format(candidate, version), flush=True)
                else:
                    session = (candidate, version)
                response_due = now
                send_response(now)
                response_due = None if final_received else now + 0.1
            elif (message_type == FINAL_ACK and session is not None and
                  data[1] == VERSION and data[6:] == b"\x00\x00" and
                  struct.unpack(">I", data[2:6])[0] == session[0] and
                  session[1] == VERSION and now >= ready_after):
                if not final_received:
                    print("final ACK session=0x{:08x}; streaming".format(session[0]), flush=True)
                final_received = True
                response_due = None
                next_sample = now

        if response_due is not None and now >= response_due:
            send_response(now)
            response_due = now + 0.1
        stalled = (args.stall_after_ms > 0 and
                   now >= started + args.stall_after_ms / 1000.0 and
                   now < started + (args.stall_after_ms + args.stall_ms) / 1000.0)
        if final_received and not stalled and now >= next_sample:
            send_sample(now)
            next_sample = now + 0.01

    peer.close()
    if session is None:
        raise RuntimeError("no START received")
    if not final_received:
        raise RuntimeError("handshake did not complete")
    print("CBoard simulator finished; {} samples sent".format(sequence), flush=True)


if __name__ == "__main__":
    main()
