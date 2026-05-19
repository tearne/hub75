#!/usr/bin/env -S uv run --script --
# /// script
# requires-python = "==3.12.*"
# dependencies = ["pyserial"]
# ///
"""
Host library for sending frames to the HUB75 display over USB CDC ACM.

Opens the firmware's CDC ACM serial port (``/dev/ttyACM*`` on Linux,
``/dev/cu.usbmodem*`` on macOS, ``COMx`` on Windows via the in-box
``usbser.sys`` driver — no admin install needed). Slower than the
vendor-class transport, but works on locked-down Windows hosts.

Sends are capped at 15 fps client-side as a conservative ceiling
until measured on the target host.

The panel size is required and must match the firmware's compile-time
panel-WxH feature; pass it explicitly to ``Hub75Client``.

Usage as a library:
    from hub75_client_cdc import Hub75Client

    client = Hub75Client(width=64, height=32)               # first match
    client = Hub75Client(width=64, height=32, serial="living-room")

    frame = bytes([255, 0, 0] * client.width * client.height)
    client.send_frame(frame)

Usage as a script (sends test patterns):
    ./hub75_client_cdc.py --width 64 --height 32

The firmware must be built with ``--features usb-class-cdc``.
"""

import argparse
import os
import sys
import time

import serial
import serial.tools.list_ports

USB_VID = 0x1209
USB_PID = 0x7575

FRAME_MAGIC = b"HB75"

# Conservative CDC frame-rate cap (see module docstring). Raise this
# once a target host has been measured.
MAX_FPS = 15
MIN_FRAME_INTERVAL = 1.0 / MAX_FPS


class Hub75Client:
    """Send frames to the HUB75 display firmware over CDC ACM."""

    def __init__(self, width: int, height: int, serial: str | None = None, timeout: float = 1.0):
        self.width = width
        self.height = height
        self.frame_pixel_bytes = width * height * 3

        port_name = _find_port(serial)
        if port_name is None:
            extra = f" with serial {serial!r}" if serial else ""
            raise RuntimeError(
                f"Could not find HUB75 CDC panel{extra} "
                f"(looking for VID {USB_VID:#06x} / PID {USB_PID:#06x}). "
                "Is the firmware built with --features usb-class-cdc?"
            )
        # pyserial's `serial` module shadows the parameter name; use the
        # fully-qualified `Serial` class.
        import serial as _serial_mod
        self._port = _serial_mod.Serial(port_name, 115_200, timeout=timeout, write_timeout=timeout)
        self._seq = 0
        self._last_send = 0.0

    def close(self):
        self._port.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def send_frame(self, pixel_bytes: bytes):
        if len(pixel_bytes) != self.frame_pixel_bytes:
            raise ValueError(
                f"Expected {self.frame_pixel_bytes} bytes, got {len(pixel_bytes)}"
            )
        # Rate cap, enforced here so callers don't have to think about
        # pacing. Sleep up to MIN_FRAME_INTERVAL since the last send.
        elapsed = time.monotonic() - self._last_send
        if elapsed < MIN_FRAME_INTERVAL:
            time.sleep(MIN_FRAME_INTERVAL - elapsed)
        header = FRAME_MAGIC + bytes([self._seq])
        self._port.write(header + pixel_bytes)
        self._last_send = time.monotonic()
        self._seq = (self._seq + 1) & 0xFF

    def pack_pixels(self, pixels) -> bytes:
        buf = bytearray(self.frame_pixel_bytes)
        for i, (r, g, b) in enumerate(pixels):
            off = i * 3
            buf[off] = r
            buf[off + 1] = g
            buf[off + 2] = b
        return bytes(buf)


def _find_port(serial_arg: str | None):
    for info in serial.tools.list_ports.comports():
        if info.vid != USB_VID or info.pid != USB_PID:
            continue
        if serial_arg is not None and (info.serial_number or "") != serial_arg:
            continue
        return info.device
    return None


def list_panels():
    """Return a list of {'serial': str, 'available': bool} dicts for attached CDC panels."""
    panels = []
    for info in serial.tools.list_ports.comports():
        if info.vid != USB_VID or info.pid != USB_PID:
            continue
        panels.append({
            "serial": info.serial_number or "",
            # CDC enumeration doesn't tell us whether another process
            # holds the port; assume available.
            "available": True,
        })
    return panels


def _solid(r, g, b, width, height):
    return bytes([r, g, b] * width * height)


def _gradient(width, height):
    buf = bytearray(width * height * 3)
    for y in range(height):
        for x in range(width):
            off = (y * width + x) * 3
            t = x * 255 // (width - 1)
            buf[off] = 255 - t
            buf[off + 1] = 0
            buf[off + 2] = t
    return bytes(buf)


def main():
    parser = argparse.ArgumentParser(description="Send test patterns to HUB75 CDC display")
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument(
        "--pattern", choices=["solid-red", "solid-green", "solid-blue", "gradient"],
        default="gradient",
    )
    parser.add_argument("--serial", help="Target a specific panel by USB serial number")
    args = parser.parse_args()

    with Hub75Client(width=args.width, height=args.height, serial=args.serial) as client:
        print(f"Connected to {client.width}×{client.height} CDC panel. "
              f"Sending '{args.pattern}' (capped at {MAX_FPS} fps). Ctrl+C to stop.")
        if args.pattern == "solid-red":
            frame = _solid(255, 0, 0, client.width, client.height)
        elif args.pattern == "solid-green":
            frame = _solid(0, 255, 0, client.width, client.height)
        elif args.pattern == "solid-blue":
            frame = _solid(0, 0, 255, client.width, client.height)
        else:
            frame = _gradient(client.width, client.height)
        try:
            while True:
                client.send_frame(frame)
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    if not os.environ.get("VIRTUAL_ENV"):
        print(
            "Error: no virtual environment detected. Run this script via "
            "'./hub75_client_cdc.py' (requires uv), or activate a virtual "
            "environment first."
        )
        sys.exit(100)
    main()
