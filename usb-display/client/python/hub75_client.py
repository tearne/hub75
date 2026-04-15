#!/usr/bin/env -S uv run --script --
# /// script
# requires-python = "==3.12.*"
# dependencies = ["pyserial"]
# ///
"""
Host library for sending frames to the HUB75 display over USB serial.

Usage as a library:
    from hub75_client import Hub75Client

    client = Hub75Client()                  # auto-detect port
    client = Hub75Client("/dev/ttyACM0")    # explicit port

    # Send a solid red frame
    frame = bytes([255, 0, 0] * 64 * 64)
    client.send_frame(frame)

    # Or build from tuples
    pixels = [(r, g, b) for ...]
    client.send_frame(Hub75Client.pack_pixels(pixels))

Usage as a script (sends test patterns):
    ./host/hub75_client.py                  # auto-detect
    ./host/hub75_client.py /dev/ttyACM0     # explicit port
"""

import argparse
import math
import os
import sys
import time

import serial
import serial.tools.list_ports

WIDTH = 64
HEIGHT = 64
FRAME_MAGIC = b"HB75"
FRAME_PIXEL_BYTES = WIDTH * HEIGHT * 3


class Hub75Client:
    """Send frames to the HUB75 display firmware over USB serial."""

    def __init__(self, port=None, baudrate=115200):
        port = port or _find_port()
        if port is None:
            raise RuntimeError(
                "Could not find HUB75 Display device. "
                "Available ports: "
                + ", ".join(f"{p.device} ({p.description})" for p in serial.tools.list_ports.comports())
            )
        self._ser = serial.Serial(port, baudrate=baudrate, timeout=1)
        self._seq = 0
        # Give the device a moment to initialise after connection
        time.sleep(0.5)

    def close(self):
        self._ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def send_frame(self, pixel_bytes: bytes):
        """Send a frame of raw RGB pixel data.

        pixel_bytes must be exactly 12288 bytes (64*64*3), row-major,
        top-left origin, R/G/B order.
        """
        if len(pixel_bytes) != FRAME_PIXEL_BYTES:
            raise ValueError(
                f"Expected {FRAME_PIXEL_BYTES} bytes, got {len(pixel_bytes)}"
            )
        header = FRAME_MAGIC + bytes([self._seq])
        self._ser.write(header + pixel_bytes)
        self._seq = (self._seq + 1) & 0xFF

    @staticmethod
    def pack_pixels(pixels) -> bytes:
        """Pack an iterable of (r, g, b) tuples into raw frame bytes."""
        buf = bytearray(FRAME_PIXEL_BYTES)
        for i, (r, g, b) in enumerate(pixels):
            off = i * 3
            buf[off] = r
            buf[off + 1] = g
            buf[off + 2] = b
        return bytes(buf)


def _find_port():
    """Find the USB serial port for the HUB75 display by manufacturer + product."""
    for port in serial.tools.list_ports.comports():
        if port.manufacturer == "tearne" and port.product == "hub75":
            return port.device
    return None


# ── Test patterns (used when run as a script) ────────────────────────

def _solid(r, g, b):
    return bytes([r, g, b] * WIDTH * HEIGHT)


def _gradient():
    """Red-to-blue horizontal gradient."""
    buf = bytearray(FRAME_PIXEL_BYTES)
    for y in range(HEIGHT):
        for x in range(WIDTH):
            off = (y * WIDTH + x) * 3
            t = x * 255 // (WIDTH - 1)
            buf[off] = 255 - t      # R
            buf[off + 1] = 0        # G
            buf[off + 2] = t        # B
    return bytes(buf)


def _rainbow(phase=0.0):
    """Slowly shifting rainbow pattern."""
    buf = bytearray(FRAME_PIXEL_BYTES)
    for y in range(HEIGHT):
        for x in range(WIDTH):
            off = (y * WIDTH + x) * 3
            hue = (x / WIDTH + y / HEIGHT + phase) % 1.0
            r, g, b = _hsv_to_rgb(hue, 1.0, 1.0)
            buf[off] = r
            buf[off + 1] = g
            buf[off + 2] = b
    return bytes(buf)


def _hsv_to_rgb(h, s, v):
    """Convert HSV (0-1 floats) to (r, g, b) as 0-255 ints."""
    if s == 0.0:
        c = int(v * 255)
        return c, c, c
    i = int(h * 6.0)
    f = h * 6.0 - i
    p = int(v * (1.0 - s) * 255)
    q = int(v * (1.0 - s * f) * 255)
    t = int(v * (1.0 - s * (1.0 - f)) * 255)
    v = int(v * 255)
    i %= 6
    if i == 0: return v, t, p
    if i == 1: return q, v, p
    if i == 2: return p, v, t
    if i == 3: return p, q, v
    if i == 4: return t, p, v
    return v, p, q


def main():
    parser = argparse.ArgumentParser(description="Send test patterns to HUB75 display")
    parser.add_argument("port", nargs="?", help="Serial port (auto-detected if omitted)")
    parser.add_argument(
        "--pattern", choices=["solid-red", "solid-green", "solid-blue", "gradient", "rainbow"],
        default="rainbow", help="Test pattern to display (default: rainbow)",
    )
    parser.add_argument("--fps", type=float, default=10, help="Frames per second (default: 10)")
    args = parser.parse_args()

    with Hub75Client(args.port) as client:
        print(f"Connected. Sending '{args.pattern}' at {args.fps} fps. Ctrl+C to stop.")

        if args.pattern == "solid-red":
            frame = _solid(255, 0, 0)
            client.send_frame(frame)
            _wait_forever()
        elif args.pattern == "solid-green":
            frame = _solid(0, 255, 0)
            client.send_frame(frame)
            _wait_forever()
        elif args.pattern == "solid-blue":
            frame = _solid(0, 0, 255)
            client.send_frame(frame)
            _wait_forever()
        elif args.pattern == "gradient":
            frame = _gradient()
            client.send_frame(frame)
            _wait_forever()
        elif args.pattern == "rainbow":
            phase = 0.0
            interval = 1.0 / args.fps
            while True:
                frame = _rainbow(phase)
                client.send_frame(frame)
                phase += 0.02
                time.sleep(interval)


def _wait_forever():
    """Block until Ctrl+C."""
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    if not os.environ.get("VIRTUAL_ENV"):
        print(
            "Error: no virtual environment detected. Run this script via "
            "'./host/hub75_client.py' (requires uv), or activate a virtual "
            "environment first."
        )
        sys.exit(100)
    main()
