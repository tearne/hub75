# Profiling sysmon

How to capture a CPU flamegraph of sysmon while it talks to a live Pico. Use when investigating where host CPU goes — e.g. before deciding on a render-pipeline change. The current baseline (post `reduce-usb-write-cost`, fast mode) is roughly 0.5% CPU at ~337M weighted samples over 30 s.

## Prerequisites

- Pico flashed with `usb` firmware (vendor-class build) and plugged in (sysmon's normal runtime state).
- Running on the host where sysmon is installed (e.g. the Raspberry Pi).

## Install tools

```bash
sudo apt install linux-perf            # pulls in `perf` for the running kernel
cargo install flamegraph               # provides `cargo flamegraph` (build + perf record)
cargo install inferno                  # provides `inferno-collapse-perf` for folding stacks
cargo install flamelens                # TUI flamegraph viewer
```

Verify: `perf --version`, `cargo flamegraph --version`, `inferno-collapse-perf --version`, and `flamelens --version` all work.

`perf` needs unprivileged kernel sampling enabled. Check:

```bash
cat /proc/sys/kernel/perf_event_paranoid
```

If it's `>= 2`, lower it for the session (resets on reboot):

```bash
sudo sysctl kernel.perf_event_paranoid=1
```

## Stop the systemd service

The installed service runs sysmon in prod mode (1 s cycle) — too idle to profile. Stop it before launching a foreground instance:

```bash
sudo systemctl stop sysmon
```

(Restart at the end with `sudo systemctl start sysmon`.)

## Run 1 — fast mode (50 ms cycle)

From `sysmon/`:

```bash
cargo flamegraph -- -f
# Ctrl+C after ~30 seconds
perf script | inferno-collapse-perf > stacks-fast.folded
flamelens stacks-fast.folded
```

`cargo flamegraph` builds with debuginfo, launches the binary under `perf record`, and leaves `perf.data` in the current directory. The SVG it also writes (`flamegraph.svg`) is a side product — `flamelens` is fed from the folded stacks we extract from `perf.data`.

`flamelens` keys: `q` quits, `/` searches, arrow keys navigate, Enter zooms in, Esc zooms out.

## Run 2 — amplified-bottleneck mode

Default `-f` is 50 ms. To make the dominant cost more obvious, push the cycle below the system's sustainable rate. Easiest way: temporarily lower the `FAST` constant in `sysmon/src/main.rs`:

```rust
const FAST: Mode = Mode {
    name: "fast",
    sampling_rate: Duration::from_millis(20),   // was 50
};
```

Then repeat the run with a different output file:

```bash
cargo flamegraph -- -f
# Ctrl+C after ~30 seconds
perf script | inferno-collapse-perf > stacks-overdrive.folded
flamelens stacks-overdrive.folded
```

Revert the constant before committing.

## What to capture for the discussion

For each run, share:

1. The folded-stacks file (`stacks-*.folded`) or a screenshot of the top hotspots from `flamelens`.
2. The console output sysmon prints on startup (mode + sampling rate).
3. Approximate observed frame rate / dropped-frame messages (firmware logs over `defmt` if a probe is attached, otherwise skip).
4. Host load — `top`/`htop` snapshot of the sysmon process's CPU% during the run.

## Tear-down

```bash
sudo systemctl start sysmon
```

Revert `Cargo.toml` and `main.rs` edits if you made them.
