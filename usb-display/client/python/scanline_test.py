#!/usr/bin/env -S uv run --script --
# /// script
# requires-python = "==3.12.*"
# dependencies = ["pyserial"]
# ///
"""
Send a scanning line pattern to the HUB75 display over USB serial.
Same pattern as crossfade_test.rs but driven from the host, so you
can compare USB throughput vs on-device generation.

Usage:
    ./host/scanline_test.py --width 64 --height 32
    ./host/scanline_test.py --width 64 --height 64 /dev/ttyACM0
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
from hub75_client import Hub75Client


def make_hline(y, r, g, b, width, height):
    buf = bytearray(width * height * 3)
    for row in range(height):
        for col in range(width):
            off = (row * width + col) * 3
            if row == y:
                buf[off] = r
                buf[off + 1] = g
                buf[off + 2] = b
            else:
                buf[off] = 0
                buf[off + 1] = 0
                buf[off + 2] = 0
    return bytes(buf)


def make_vline(x, r, g, b, width, height):
    buf = bytearray(width * height * 3)
    for row in range(height):
        for col in range(width):
            off = (row * width + col) * 3
            if col == x:
                buf[off] = r
                buf[off + 1] = g
                buf[off + 2] = b
            else:
                buf[off] = 0
                buf[off + 1] = 0
                buf[off + 2] = 0
    return bytes(buf)


def main():
    parser = argparse.ArgumentParser(description="Scanning line test over USB")
    parser.add_argument("port", nargs="?", help="Serial port (auto-detected if omitted)")
    parser.add_argument("--width", type=int, required=True, help="Panel width in pixels")
    parser.add_argument("--height", type=int, required=True, help="Panel height in pixels")
    parser.add_argument("--fps", type=float, default=30, help="Target fps (default: 30)")
    args = parser.parse_args()

    with Hub75Client(width=args.width, height=args.height, port=args.port) as client:
        print(f"Connected to {client.width}×{client.height}. Sending scanning lines at {args.fps} fps target. Ctrl+C to stop.")

        interval = 1.0 / args.fps
        horizontal = True
        position = 0
        sent = 0
        t0 = time.monotonic()

        try:
            while True:
                if horizontal:
                    frame = make_hline(position, 255, 0, 0, client.width, client.height)
                else:
                    frame = make_vline(position, 0, 128, 255, client.width, client.height)

                client.send_frame(frame)
                sent += 1

                position += 1
                limit = client.height if horizontal else client.width
                if position >= limit:
                    position = 0
                    horizontal = not horizontal

                # Print actual fps every second
                elapsed = time.monotonic() - t0
                if elapsed >= 1.0:
                    print(f"  {sent / elapsed:.1f} fps actual")
                    sent = 0
                    t0 = time.monotonic()

                time.sleep(interval)
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    if not os.environ.get("VIRTUAL_ENV"):
        print(
            "Error: no virtual environment detected. Run this script via "
            "'./host/scanline_test.py' (requires uv), or activate a virtual "
            "environment first."
        )
        sys.exit(100)
    main()
