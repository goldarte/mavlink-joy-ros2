#!/usr/bin/env python3
"""Stand-in for the mavlink-joystick app without a phone.

Waits for the ROS node's MAVLink HEARTBEAT, then streams MANUAL_CONTROL
message 69 at 50 Hz with varying stick positions, exactly like the app does.
"""

import math
import socket
import struct
import sys
import time

STX_V2 = 0xFD
MANUAL_CONTROL_ID = 69
MANUAL_CONTROL_CRC_EXTRA = 243
SYSTEM_ID = 255
COMPONENT_ID = 190


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def frame(message_id: int, payload: bytes, sequence: int, crc_extra: int) -> bytes:
    header = struct.pack(
        "<BBBBBB", len(payload), 0, 0, sequence, SYSTEM_ID, COMPONENT_ID
    ) + message_id.to_bytes(3, "little")
    checksum = crc16(header + payload + bytes([crc_extra]))
    return bytes([STX_V2]) + header + payload + struct.pack("<H", checksum)


def manual_control(x: int, y: int, z: int, r: int, buttons: int = 0) -> bytes:
    return struct.pack("<hhhhHB", x, y, z, r, buttons, 1)


def main() -> None:
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 14555
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", port))
    sock.settimeout(10)
    print(f"fake app listening on {sock.getsockname()}", flush=True)

    data, node = sock.recvfrom(2048)
    print(f"got {len(data)} bytes from {node}: {data[:20].hex()}...", flush=True)

    deadline = time.monotonic() + seconds
    sequence = 0
    while time.monotonic() < deadline:
        phase = (deadline - time.monotonic()) * 4.0
        payload = manual_control(
            x=int(-700 * math.sin(phase)),   # pitch
            y=int(1000 * math.cos(phase)),   # roll
            z=500,                           # throttle
            r=int(300 * math.sin(phase)),    # yaw
            buttons=0b101,
        )
        sock.sendto(frame(MANUAL_CONTROL_ID, payload, sequence, MANUAL_CONTROL_CRC_EXTRA), node)
        sequence = (sequence + 1) % 256
        if sequence % 25 == 0:
            print(f"sent MANUAL_CONTROL #{sequence}", flush=True)
        time.sleep(0.02)


if __name__ == "__main__":
    main()
