"""Generate a .H75 test frame for the usb-drop firmware.

Usage: python3 gen.py [pattern] > out.H75

Patterns:
  gradient (default) — red/green ramp; good for orientation checks
  solid              — solid red; "did anything render?"
  origin             — single white pixel at (0,0); origin sanity
  blocks             — 8x8 colour-block grid; channel + geometry
"""
import json
import sys

W, H = 64, 64


def gradient(x, y):
    return [(x * 4) % 256, (y * 4) % 256, 0]


def solid(x, y):
    return [255, 0, 0]


def origin(x, y):
    return [255, 255, 255] if (x, y) == (0, 0) else [0, 0, 0]


def blocks(x, y):
    bx, by = x // 8, y // 8
    palette = [
        [255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0],
        [0, 255, 255], [255, 0, 255], [255, 255, 255], [128, 128, 128],
    ]
    return palette[(bx + by) % len(palette)]


PATTERNS = {"gradient": gradient, "solid": solid, "origin": origin, "blocks": blocks}


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "gradient"
    fn = PATTERNS[name]
    pixels = [fn(x, y) for y in range(H) for x in range(W)]
    print(json.dumps({"version": 1, "width": W, "height": H, "pixels": pixels}))


if __name__ == "__main__":
    main()
