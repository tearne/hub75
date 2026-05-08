#!/usr/bin/env -S uv run --script --
# /// script
# requires-python = "==3.12.*"
# dependencies = ["pyusb"]
# ///
"""
Host library for sending frames to the HUB75 display over USB.

Talks to the firmware's vendor-class bulk endpoint via libusb (through
pyusb), bypassing the kernel TTY/line-discipline machinery that CDC ACM
otherwise pulls in. The firmware advertises VID 0x1209 (pid.codes) and
PID 0x7575; we double-check the manufacturer/product strings as a
sanity match.

The panel size is required and must match the firmware's compile-time
panel-WxH feature; pass it explicitly to ``Hub75Client``.

Usage as a library:
    from hub75_client import Hub75Client

    client = Hub75Client(width=64, height=32)               # first match
    client = Hub75Client(width=64, height=32, serial="living-room")  # specific panel

    # Send a solid red frame
    frame = bytes([255, 0, 0] * client.width * client.height)
    client.send_frame(frame)

    # Or build from tuples
    pixels = [(r, g, b) for ...]
    client.send_frame(client.pack_pixels(pixels))

Usage as a script (sends test patterns):
    ./hub75_client.py --width 64 --height 32

Requires libusb-1.0 on the host. On Linux, the device must be writable
by the calling user — see SETUP.md for the udev rule.
"""

import argparse
import os
import sys
import time

import usb.core
import usb.util

USB_VID = 0x1209
USB_PID = 0x7575
USB_MANUFACTURER = "tearne"
USB_PRODUCT = "hub75"

# First bulk OUT endpoint on the firmware's vendor-class interface.
BULK_OUT_ENDPOINT = 0x01

FRAME_MAGIC = b"HB75"


class Hub75Client:
    """Send frames to the HUB75 display firmware over USB bulk."""

    def __init__(self, width: int, height: int, serial: str | None = None, timeout_ms: int = 1000):
        self.width = width
        self.height = height
        self.frame_pixel_bytes = width * height * 3
        self._timeout_ms = timeout_ms

        self._dev = _find_device(serial)
        if self._dev is None:
            extra = f" with serial {serial!r}" if serial else ""
            raise RuntimeError(
                f"Could not find HUB75 display{extra} "
                f"(looking for VID {USB_VID:#06x} / PID {USB_PID:#06x}, "
                f"manufacturer={USB_MANUFACTURER!r}, product={USB_PRODUCT!r})"
            )
        # Detach any kernel driver that might have grabbed the interface
        # (rare for vendor-class, but cheap to be safe).
        if self._dev.is_kernel_driver_active(0):
            self._dev.detach_kernel_driver(0)
        self._dev.set_configuration()
        usb.util.claim_interface(self._dev, 0)
        self._seq = 0

    def close(self):
        usb.util.release_interface(self._dev, 0)
        usb.util.dispose_resources(self._dev)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def send_frame(self, pixel_bytes: bytes):
        """Send a frame of raw RGB pixel data.

        ``pixel_bytes`` must be exactly ``width * height * 3`` bytes,
        row-major, top-left origin, R/G/B order.
        """
        if len(pixel_bytes) != self.frame_pixel_bytes:
            raise ValueError(
                f"Expected {self.frame_pixel_bytes} bytes, got {len(pixel_bytes)}"
            )
        header = FRAME_MAGIC + bytes([self._seq])
        self._dev.write(BULK_OUT_ENDPOINT, header + pixel_bytes, self._timeout_ms)
        self._seq = (self._seq + 1) & 0xFF

    def pack_pixels(self, pixels) -> bytes:
        """Pack an iterable of (r, g, b) tuples into raw frame bytes."""
        buf = bytearray(self.frame_pixel_bytes)
        for i, (r, g, b) in enumerate(pixels):
            off = i * 3
            buf[off] = r
            buf[off + 1] = g
            buf[off + 2] = b
        return bytes(buf)


def _find_device(serial: str | None = None):
    for dev in usb.core.find(find_all=True, idVendor=USB_VID, idProduct=USB_PID):
        try:
            manufacturer = usb.util.get_string(dev, dev.iManufacturer) or ""
            product = usb.util.get_string(dev, dev.iProduct) or ""
        except usb.core.USBError:
            continue
        if manufacturer != USB_MANUFACTURER or product != USB_PRODUCT:
            continue
        if serial is not None:
            try:
                got = usb.util.get_string(dev, dev.iSerialNumber) or ""
            except usb.core.USBError:
                continue
            if got != serial:
                continue
        return dev
    return None


def list_panels():
    """Return a list of {'serial': str, 'available': bool} dicts for attached panels."""
    panels = []
    for dev in usb.core.find(find_all=True, idVendor=USB_VID, idProduct=USB_PID):
        try:
            manufacturer = usb.util.get_string(dev, dev.iManufacturer) or ""
            product = usb.util.get_string(dev, dev.iProduct) or ""
        except usb.core.USBError:
            continue
        if manufacturer != USB_MANUFACTURER or product != USB_PRODUCT:
            continue
        try:
            serial = usb.util.get_string(dev, dev.iSerialNumber) or ""
        except usb.core.USBError:
            serial = ""
        try:
            usb.util.claim_interface(dev, 0)
            available = True
            usb.util.release_interface(dev, 0)
        except usb.core.USBError:
            available = False
        panels.append({"serial": serial, "available": available})
    return panels


# ── Test patterns (used when run as a script) ────────────────────────

def _solid(r, g, b, width, height):
    return bytes([r, g, b] * width * height)


def _gradient(width, height):
    """Red-to-blue horizontal gradient."""
    buf = bytearray(width * height * 3)
    for y in range(height):
        for x in range(width):
            off = (y * width + x) * 3
            t = x * 255 // (width - 1)
            buf[off] = 255 - t      # R
            buf[off + 1] = 0        # G
            buf[off + 2] = t        # B
    return bytes(buf)


def _rainbow(width, height, phase=0.0):
    """Slowly shifting rainbow pattern."""
    buf = bytearray(width * height * 3)
    for y in range(height):
        for x in range(width):
            off = (y * width + x) * 3
            hue = (x / width + y / height + phase) % 1.0
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
    parser.add_argument("--width", type=int, required=True, help="Panel width in pixels (must match firmware)")
    parser.add_argument("--height", type=int, required=True, help="Panel height in pixels (must match firmware)")
    parser.add_argument(
        "--pattern", choices=["solid-red", "solid-green", "solid-blue", "gradient", "rainbow"],
        default="rainbow", help="Test pattern to display (default: rainbow)",
    )
    parser.add_argument("--fps", type=float, default=10, help="Frames per second (default: 10)")
    parser.add_argument("--serial", help="Target a specific panel by USB serial number (default: first match)")
    args = parser.parse_args()

    with Hub75Client(width=args.width, height=args.height, serial=args.serial) as client:
        print(f"Connected to {client.width}×{client.height} panel. Sending '{args.pattern}' at {args.fps} fps. Ctrl+C to stop.")

        if args.pattern == "solid-red":
            frame = _solid(255, 0, 0, client.width, client.height)
            client.send_frame(frame)
            _wait_forever()
        elif args.pattern == "solid-green":
            frame = _solid(0, 255, 0, client.width, client.height)
            client.send_frame(frame)
            _wait_forever()
        elif args.pattern == "solid-blue":
            frame = _solid(0, 0, 255, client.width, client.height)
            client.send_frame(frame)
            _wait_forever()
        elif args.pattern == "gradient":
            frame = _gradient(client.width, client.height)
            client.send_frame(frame)
            _wait_forever()
        elif args.pattern == "rainbow":
            phase = 0.0
            interval = 1.0 / args.fps
            while True:
                frame = _rainbow(client.width, client.height, phase)
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
            "'./hub75_client.py' (requires uv), or activate a virtual "
            "environment first."
        )
        sys.exit(100)
    main()
