#!/usr/bin/env bash
# Build the sysmon .deb package. Run from this directory.
#
#   ./build.sh
#
# Produces:  target/aarch64-unknown-linux-gnu/debian/sysmon_<version>_arm64.deb
#
# Requires `cargo-deb` — install with `cargo install cargo-deb` if missing.
set -euo pipefail
cd "$(dirname "$0")"

if ! command -v cargo-deb >/dev/null 2>&1; then
    echo "error: cargo-deb is not installed."
    echo "install it with:  cargo install cargo-deb"
    exit 1
fi

cargo deb

deb_path=$(find target -name 'sysmon_*_arm64.deb' -printf '%T@ %p\n' 2>/dev/null \
    | sort -rn | head -1 | cut -d' ' -f2-)
if [ -z "$deb_path" ]; then
    echo "error: no .deb produced under target/"
    exit 1
fi

echo
echo "Built: $deb_path"
echo "Install with:  sudo apt install ./$deb_path"
