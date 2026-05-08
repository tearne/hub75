#!/usr/bin/env -S uv run --script --
# /// script
# requires-python = "==3.12.*"
# dependencies = ["pyusb"]
# ///
"""
List attached HUB75 panels and whether they're free for this process to open.

Usage:
    ./list_panels.py
"""

import argparse
import os
import sys

from hub75_client import list_panels

VERSION = "1.0.0"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--version", action="version", version=f"%(prog)s {VERSION}")
    parser.parse_args()

    panels = list_panels()
    if not panels:
        print("No HUB75 panels found.")
        return
    width = max(len(p["serial"]) for p in panels + [{"serial": "serial"}])
    print(f"{'serial':<{width}}  available")
    print(f"{'-' * width}  ---------")
    for p in panels:
        print(f"{p['serial']:<{width}}  {'yes' if p['available'] else 'no (claimed)'}")


if __name__ == "__main__":
    if not os.environ.get("VIRTUAL_ENV"):
        print(
            "Error: no virtual environment detected. Run this script via "
            "'./list_panels.py' (requires uv), or activate a virtual "
            "environment first."
        )
        sys.exit(100)
    main()
