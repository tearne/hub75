#!/usr/bin/env bash
# Run the sysmon_linux example, overriding the crate's pinned x86_64
# target with whatever host this machine actually is.
#
# Usage:  ./run-sysmon.sh                       # default 1 Hz update
#         ./run-sysmon.sh -u 5000               # 5 s update interval
#         ./run-sysmon.sh /dev/ttyACM1          # specific port
#         ./run-sysmon.sh -u 200 /dev/ttyACM1   # both
set -euo pipefail
cd "$(dirname "$0")"

host_target="$(rustc -vV | awk '/^host:/ {print $2}')"

exec cargo run --release \
    --target "$host_target" \
    --example sysmon_linux \
    --features panel-64x32 \
    -- "$@"
