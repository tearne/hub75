# sysmon example for usb-display

## Intent

Add a new `usb-display` Rust client example that turns the 64×32 panel (rotated 90° clockwise into a 32×64 portrait canvas) into an at-a-glance system monitor for the host machine.

Four metrics, stacked vertically, each rendered as a small sparkline-style mini graph:

- **CPU** — overall utilisation over time.
- **RAM** — composition shown as a stacked bar (used / buffers / cached / free), plus a sparkline of total usage over time.
- **Disk I/O** — read and write throughput over time.
- **Network** — upload and download throughput over time.

Metrics with no fixed upper bound (disk, network) auto-rescale against the maximum observed during the running session.

The example is Linux-only — it reads `/proc` directly — but cross-architecture (Pi, x86 desktop, etc.). The example's filename signals the OS constraint.

## Approach

### Filename and OS gating

Example file: `usb-display/client/rust/examples/sysmon_linux.rs`. The `_linux` suffix advertises the OS constraint at the call site. The example body is wrapped in `#[cfg(target_os = "linux")]`; on other OSes the binary still compiles but its `main` prints a message and exits, so a blanket `cargo build --examples` doesn't break.

### Portrait-mode rendering

All drawing happens in a logical 32×64 (W×H) portrait buffer. A single `pack_rotated_cw` step maps logical `(x, y)` to physical `(WIDTH - 1 - y, x)` when emitting the wire frame. This keeps the drawing code orientation-agnostic and confines the rotation to one place.

### Vertical layout — four equal strips

The 64-row canvas is divided into four 16-row strips, one per metric, in this order top-to-bottom: CPU, RAM, Disk, Network. Equal heights keep the layout legible and avoid bias between metrics. Each strip uses its full 32-pixel width.

### Per-strip composition

Within each 16-row strip:

- A 1-pixel coloured stripe down the left edge identifies the metric in the strip's primary colour.
- The remainder is the sparkline area (~30 px wide × ~14 px tall after a 1 px frame margin).

This gives identification without any glyphs, satisfying the graphics-only constraint.

### Strip colours

- **CPU** — cyan family. The sparkline is a per-column stack of usage types: user (bright cyan), system (deeper cyan/teal), iowait (a bluer shade). Total stack height is overall non-idle %.
- **RAM** — magenta family. Stacked composition bar uses four magenta shades for used / buffers / cached / free; the sparkline uses a single magenta.
- **Disk** — amber for both read (above centreline) and write (below), with read slightly brighter to distinguish.
- **Network** — green family for contrast with amber and magenta. Down (above centreline) is bright green, up (below) is a yellower/limier green.

### Sparkline rendering

Sparklines are filled area plots: each column is a vertical bar from the strip baseline up to the sample's normalised height. The newest sample is on the right; older samples scroll left. History length matches sparkline width (one sample per column).

### RAM strip — composition + history

The RAM strip splits into two side-by-side regions:

- Left ~10 px: a vertical stacked bar showing current composition — used / buffers / cached / free, from bottom to top, each in its own colour.
- Right ~20 px: the usual scrolling sparkline of total used (used + buffers + cached) as a fraction of total.

This shows both the instantaneous breakdown and the trend in the same strip.

### Disk and network — dual values

Both metrics carry two values (read/write, up/down). They render as a *mirrored* sparkline: read/down rises from the strip's vertical centre upward; write/up falls from the centre downward. The two share the same auto-rescaled maximum so relative magnitudes are comparable.

Disk sums across all real block devices, skipping `loop*`, `ram*`, and `dm-*`. Network sums across all interfaces except loopback (`lo`).

### Auto-rescaling

CPU is fixed 0–100 %. RAM is fixed against `MemTotal`. Disk and network track a session-max-so-far per channel pair (one max for disk, one for network) and normalise against that. The max never decays, so spikes leave a permanent ceiling for the rest of the session — accepted because the user asked for session-max behaviour explicitly.

### Sampling cadence

`/proc/stat`, `/proc/meminfo`, `/proc/diskstats`, `/proc/net/dev` are read once per second on a background thread. Rates are computed as deltas across that interval. The render loop runs faster (~15 fps) and reads the latest sampled values; sparkline columns advance once per new sample, not per frame.

### Dependencies

Pure `std` for `/proc` parsing — no `sysinfo` or `procfs` crate. The parsing surface is small (four files, simple line formats), and avoiding new dependencies keeps the example self-contained and matches the style of the existing examples.

## Plan

Version bump: `hub75-client` patch — **0.2.1 → 0.2.2**. The public library API is unchanged; only a new example ships.

- [x] Bump `usb-display/client/rust/Cargo.toml` version to `0.2.2`
- [x] Create `examples/sysmon_linux.rs` skeleton: `#[cfg(target_os = "linux")]` gate, stub `main`, fallback `main` for non-Linux
- [x] Add `pack_rotated_cw` helper: map 32×64 logical buffer → 64×32 physical wire frame (`px = 63 - ly`, `py = lx`)
- [x] Add `Sparkline` ring buffer type with session-max auto-rescale and column-fill rendering
- [x] Implement `/proc/stat` parser: per-interval user / system / iowait / total deltas
- [x] Implement `/proc/meminfo` parser: total / free / buffers / cached (KiB)
- [x] Implement `/proc/diskstats` parser: sum read/write sectors across real block devices (skip `loop*`, `ram*`, `dm-*`)
- [x] Implement `/proc/net/dev` parser: sum rx/tx bytes across all interfaces except `lo`
- [x] Implement 1 Hz sampler thread populating a shared `Snapshot` via `Arc<Mutex<...>>`
- [x] Render CPU strip: stacked cyan-family sparkline (user / system / iowait), bounded 0–100 %
- [x] Render RAM strip: magenta stacked composition bar (used / buffers / cached / free) on the left, magenta total-used sparkline on the right
- [x] Render disk strip: mirrored amber sparkline (read above centreline, write below), shared session-max rescale
- [x] Render network strip: mirrored green-family sparkline (down above centreline, up below), shared session-max rescale
- [x] Wire ~15 fps render loop: clear, draw the four strips, pack-rotate, `send_frame_rgb`
- [x] Add a `sysmon_linux` row to `usb-display/README.md`'s rust client example table
- [x] `cargo build --example sysmon_linux --features panel-64x32` succeeds; user runs and visually verifies on the panel

## Log

- `usb-display/client/rust/.cargo/config.toml` pins `target = "x86_64-unknown-linux-gnu"`. Build verification on the Pi (aarch64) requires `--target aarch64-unknown-linux-gnu`. Did not change the config — it reflects the project's deliberate "host-side x86 by default" choice. User will likely build/run on their x86 host.
- Composition bar drops the `free` segment fraction and instead fills the remainder of the bar with the `RAM_FREE` colour. This avoids cumulative rounding leaving a gap or overshoot at the top.
- Sparkline RAM area uses `LW - RAM_BAR_W - RAM_GAP = 26` columns; CPU/disk/net use the full 32 columns. History buffer length is 32 throughout; the RAM sparkline takes the most-recent 26 samples. Strip-bottom row is left blank as a separator, leaving 15 drawing rows per strip; for mirrored strips the centreline is row 7 of the strip.
- Added `usb-display/client/rust/run-sysmon.sh` (out-of-plan, at user request after they hit the x86_64-target friction running on the Pi). Auto-detects host target via `rustc -vV` and runs the `sysmon_linux` example specifically — passes through any extra args (port, `-u <ms>`).
- Added `-u`/`--update <ms>` flag to `sysmon_linux` (out-of-plan, user request). Single knob controls both sampler interval and render-loop interval — they tick together, so the panel updates exactly when data changes. Range clamped to 33–10000 ms (30 fps to 1 frame per 10 s); default 1000 ms preserves prior behaviour.
- CPU strip switched from stacked-bar height-encoding to density-of-dots encoding (out-of-plan, user request). Each column lights `(user+system+iowait) × strip_rows` random rows seeded by a per-sample u32 (`next_seed()` golden-ratio counter), so the position pattern is stable across renders for the same sample. Colours are interleaved throughout the column (priority: user → system → iowait when budget is exhausted).
- RAM and Disk merged into a single horizontal strip (rows 16–30): two 15-column stacked-bar histories side-by-side with a 2-column black separator. One column = 20 seconds, so each bar holds 5 minutes of history. RAM column stacks used/buffers/cached/free against `total`. Disk column stacks read/write against the session-max of `(read + write)`. A second history deque (`Snapshot.slow`) is updated once per `SLOW_PERIOD = 20 s`; at first valid sample the deque is pre-filled with 15 copies of the current values so the bars are populated from startup ("project current usage back in history").
- Network strip stretched to double height (rows 32–62) to use the strip freed by the RAM/Disk merge, giving the most-volatile metric the most vertical resolution.
- Strip-drawing functions now take explicit `(top, base)` pairs instead of a strip index — the previous `strip_bounds(idx)` helper was incompatible with non-uniform strip heights.
- Slow-history machinery and side-by-side bars dropped after disk visibility issues exposed a session-max scaling problem: a transient cargo-build spike at startup pegged `disk_max` so high that idle activity rounded to zero rows. Rather than patch with a warm-up or rolling-window fix, restructured to all-dots, all-fast: four equal 16-row strips (CPU, RAM, Disk, Network) each with one sample per render interval, all using the same density-of-dots encoding (`paint_dot_column`). Disk and network use a fixed log scale (1 KB/s → 1 GB/s, 6 decades over 15 rows ≈ 2.5 rows per decade) so quiet idle still shows and bursts don't peg out. RAM uses linear scale and intentionally drops the `free` colour — coloured dots are used / buffers / cached, unlit dots are everything else. User accepted the loss of precise composition reading in exchange for whole-panel synchronised motion.
- Diagonal-dot bug at high refresh rates: `next_seed()` originally stepped the counter by `0x9e3779b9`, which is the same constant `mix32` uses internally as its `b`-multiplier. Result: `mix32(seed + k·G, r)` collapses to `mix32(seed, r + k)`, so consecutive samples produced the *same* shuffled permutation rotated by a fixed offset — visible as dots marching diagonally at fast refresh. Fixed by hashing a plain incrementing counter through `mix32` instead, which decorrelates consecutive seeds.
- Per-metric scroll speeds (user feedback: whole-panel sync at fast refresh felt dizzy). CPU still updates every tick; RAM and Network push every 6 ticks; Disk every 20 ticks. Sampler tracks per-metric "tick of last push" and computes the elapsed-time basis for rate calculations from that, so disk/net rates are averaged over the full inter-push window. Multipliers scale with `--update`: at default 1 s tick, CPU=1 s, RAM/Net=6 s, Disk=20 s per column. With 32 columns of history, slower metrics show much longer windows (CPU 32 s, RAM/Net 3 min, Disk ~10 min).
- Network split into two layers: download in the upper 7 rows of its strip, upload in the lower 7, with a 1-row gap separator at row 7 of the strip. Each half scales independently against the same log endpoints, and gets its own derived seed (`sample.seed.wrapping_add(constant)`) so the two halves don't share a permutation. `IoPalette` enum dropped — disk and net are now rendered by separate strip-specific functions instead of a generic `draw_io_strip`.
- Disk colours brought into the warm spectrum like network's green pair (read = bright gold-yellow, write = orange-red).
- Disk also given the layered-split treatment (read on top, write on bottom). Disk and net renderers now share `draw_layered_strip` — `LAYER_HALF_ROWS`/`LAYER_LOWER_OFFSET` constants generalised from the previous `NET_*` names. The combined-density `split_dots` helper went away with this change since both layered metrics now scale their halves independently.
- Phase offsets per metric so the slow strips never push on the same tick. RAM at tick % 6 == 0, NET at tick % 6 == 4, DISK at tick % 20 == 11. Chosen so RAM and NET (both period 6) sit 4 ticks apart inside their cycle, and DISK's three firings per LCM(6,20)=60-tick cycle land on residues mod 6 that neither RAM nor NET use. CPU still ticks every step (the constant background rhythm).

## Conclusion

Shipped a working `sysmon_linux` example, plus `run-sysmon.sh` and a bumped `hub75-client` 0.2.2. Implementation diverged substantially from the original Plan during build:

- The Plan called for traditional sparklines and a stacked composition bar. The final design is all-dots, all-strips: density encodes the metric, colour encodes the type/direction, and each strip scrolls at its own pace with phased pushes so the panel doesn't lurch as a single block.
- Disk visibility issue led to dropping session-max scaling in favour of fixed log-scale endpoints (1 KB/s → 1 GB/s).
- Two CLI niceties added at user request and not in the Plan: `-u/--update <ms>` flag, and the `run-sysmon.sh` host-target-detecting wrapper script.
- One real bug found and fixed: `next_seed()`'s step constant collided with `mix32`'s internal multiplier, producing diagonal dot drift at fast refresh.

The project has no map.md and no changelog, so nothing else to update.

A follow-up change is queued: smooth dot fading via subframes, with ~2-dot blur to avoid wiggle between fading pairs and singletons.



