# Rotate panel + per-core CPU sub-bands

## Intent

Rotate the sysmon panel 90° counter-clockwise (instead of the current clockwise) so the strips run as four vertical columns, with the newest data streaming in from the top and falling downward. This gives roughly double the visible time history per metric (64 rows vs 32 cols) and reads more like a tape-feed than a horizontal sparkline.

Strip order left-to-right: **Disk | CPU | RAM | Net**, with widths **6 | 14 | 6 | 6**. CPU gets the widest column because it now also contains a per-core breakdown — four horizontal sub-bands stacked vertically within the CPU column, one per core, so a single-threaded workload pinning one core is immediately visible. Each core's sub-band shows the same user / system / iowait dot encoding the CPU strip already uses, just for that core's load.

Disk and Net keep their two-layer split (read/write, down/up) but lose the 1-pixel separator between layers — colour difference alone distinguishes them, freeing those pixels for density resolution.

The sliding direction flips with the rotation: the smooth-fade work continues to apply, but the slide is now vertical (downward) instead of horizontal (leftward).

## Approach

### Rotation: flip the pack mapping

Replace the current logical→physical mapping (`px = WIDTH-1-ly, py = lx`) with the mirrored CCW form (`px = ly, py = HEIGHT-1-lx`). Same canvas dimensions (32 × 64 logical, 64 × 32 physical) but the "top" of the logical view now corresponds to the opposite physical edge.

### Strip layout: vertical columns

Strips become vertical columns running the full 64-row height. Replace `_TOP` constants with `_LEFT` and `_RIGHT` ranges:

- DISK: cols 0–5  (6 wide)
- CPU:  cols 6–19 (14 wide)
- RAM:  cols 20–25 (6 wide)
- NET:  cols 26–31 (6 wide)

No inter-strip gaps. Strips are distinguished entirely by the colour palette of their dots.

### Time axis: vertical, top-down

`HIST_LEN` becomes 64 (was 32). Newest sample at row 0; samples slide downward over their period; oldest at the bottom of the buffer slides off row 63.

### Per-column rendering: transposed

The shuffle / splat / fade-in machinery transposes:

- `paint_dot_column` becomes `paint_dot_row` — picks pseudo-random *columns* across the strip width and splats at a sub-pixel *y* position.
- `splat_gaussian` axis flips: kernel runs vertically (along *y*) rather than horizontally.
- Sub-pixel position: sample at history index `i` is rendered at logical `y = i + t` (newest pushes downward as `t` grows).
- Fade-in still applies to the newest (i = 0, top of strip): alpha ramps from 0 to 1 over the first `FADE_IN_FRACTION` of the metric's period.

### CPU: per-core sub-bands

Hardcode 4 cores (Pi 5). Read `cpu0` through `cpu3` lines from `/proc/stat`; ignore the aggregate `cpu` line and any cpuN with N ≥ 4. The geometry stacks 4 horizontal sub-bands in the 64-row CPU column at 16 rows each, no separator gap.

`CpuSample` carries per-core `[CoreSample; 4]` instead of a single `(user, system, iowait, seed)` tuple. One seed per sample (shared across cores) so the random row pattern is coherent within a sample frame.

CPU strip renderer iterates the four 16-row sub-bands; each sub-band is rendered like the existing CPU strip in miniature — same user/system/iowait colours, same dot encoding, just with `rows = 16` and the appropriate vertical offset.

### Disk and Net: horizontal layered split

The two-channel layered split also transposes. With the strip now a vertical column, the layers become **horizontal halves**: left half = upper channel (read / down), right half = lower channel (write / up). For 6-wide strips this is 3 + 3 with no separator gap; colour difference distinguishes them.

`LAYER_HALF_ROWS` retires; new `LAYER_HALF_COLS = 3`. The renderer paints the upper-channel half at the strip's left columns and the lower-channel half at the right columns, each with its own derived seed.

### Periods, phases, sampler thread: unchanged

`PERIOD_RAM/NET/DISK`, `PHASE_RAM/NET/DISK`, the per-metric `_at` timestamps, the sampler's tick logic, and the smooth-fade render rate all stay as-is. Only the rendering code transposes.

### Defaults and constants tweaks

- `LAYER_HALF_ROWS` → `LAYER_HALF_COLS` (= 3).
- New strip-position constants: `DISK_LEFT`, `CPU_LEFT`, `CPU_WIDTH`, `RAM_LEFT`, `NET_LEFT` (and analogous widths).
- `STRIP_ROWS` retires; per-strip width replaces it as the "candidate dot positions" parameter passed to `dots_for`.
- `BLUR_HALF_PIX` unchanged (still ±2, just along *y* now).

## Plan

Version bump: `hub75-client` patch — **0.2.3 → 0.2.4**. Public API unchanged; internal example restructure.

- [x] Bump `usb-display/client/rust/Cargo.toml` to `0.2.4`
- [x] Replace pack mapping: `px = ly, py = HEIGHT - 1 - lx` (rename `pack_rotated_cw` → `pack_rotated_ccw`); update its doc comment
- [x] Replace `_TOP` strip constants with `_LEFT`/`_WIDTH` pairs
- [x] Drop `LAYER_HALF_ROWS` / `LAYER_LOWER_OFFSET`; add `LAYER_HALF_COLS = 3`
- [x] Add `CORE_COUNT = 4`, `CORE_HEIGHT = 16`
- [x] History depth now per-metric: `CPU_HIST_LEN = 16` (one core sub-band's height), `OTHER_HIST_LEN = 64` (full canvas)
- [x] Replace `CpuSample` with per-core `[CoreSample; CORE_COUNT]` plus a single shared `seed`
- [x] Update `read_cpu_times` → `read_cpu_times_all`
- [x] Update `sample_cpu` to compute per-core deltas
- [x] Transpose `splat_gaussian`: kernel runs vertically along *y*, also takes `y_min`/`y_max` to clip to the strip or sub-band
- [x] Rename `paint_dot_column` → `paint_dot_row`
- [x] Use `y = i as f32 + t` (newest at i=0)
- [x] Rewrite `draw_cpu_strip` with 4-core sub-band loop
- [x] Rewrite `draw_ram_strip` for vertical column
- [x] Rewrite `draw_disk_strip` / `draw_net_strip` via shared `draw_layered_strip` with horizontal layered split
- [x] Newest-sample fade-in still applies to `i = 0`
- [x] Sampler uses `push_front` with cap from the back
- [x] Update `usb-display/README.md` `sysmon_linux` row
- [x] `cargo build --example sysmon_linux --features panel-64x32` succeeds; user runs and visually confirms

## Log

- `HIST_LEN` originally specified as a single global of 64 in the Plan turned out to be wrong: per-core sub-bands are only 16 rows tall, so the CPU buffer must cap at `CORE_HEIGHT = 16`, not `LH = 64`. Split into `CPU_HIST_LEN = 16` and `OTHER_HIST_LEN = 64`. Trade-off acknowledged: per-core history is shorter (16 samples) than the other strips' 64 samples, but the per-core visibility is the point.
- `splat_gaussian` grew `y_min`/`y_max` parameters to clip its kernel to a region — needed because a sample at the bottom of one core's sub-band would otherwise have its blur tail bleed into the next core's sub-band. Non-CPU strips just pass `0, LH-1`.
- Per-core seeds derived as `mix32(sample.seed, core_idx)` so the four cores' random patterns don't overlay identically (which would create vertical stripes across all 4 sub-bands at once).
- `DISK_WIDTH` / `NET_WIDTH` constants were initially unused (the layered renderer hardcodes `LAYER_HALF_COLS * 2`). Added compile-time `assert!`s tying them to `LAYER_HALF_COLS * 2` and verifying total width sums to `LW` — gives the constants a load-bearing role and catches future layout typos at compile time.
- After first run on the panel, two issues surfaced. (1) Flow direction was inverted — dots moved bottom-to-top because the new "CCW" pack mapping placed the logical top edge on a physical edge that, after the user's physical mounting, ended up at the bottom of the user-visible view. Reverted the pack to the original CW mapping (`px = WIDTH-1-ly, py = lx`); this is a 180° rotation of the strict CCW form and is what the user's panel mounting actually needs. The function name went back to `pack_rotated_cw`.  (2) The "4 horizontal sub-bands stacked vertically" CPU layout in the Approach was misread — the user wanted 4 thin **vertical** sub-strips, each running the full canvas height, so single-threaded workloads pinning one core are immediately visible against three quiet neighbours. Restructured: dropped `CORE_HEIGHT`, added `CORE_WIDTH = 3`. CPU column shrunk from 14 to 12 (= 4 × 3) and the freed 2 cols went to Disk (now 8 wide, layered as 4+4). Net stays 6 wide (layered 3+3). Layered renderer now takes `half_cols` per call so the two strips can have different layer widths. CPU history depth grew from 16 (one sub-band's height) back to 64 (the full canvas), giving each core full time history at the cost of fewer dot positions per sample (3 vs 14).
- User refined the layout: Disk back to 6 wide (3+3 layered, symmetric with Net), the freed 2 cols went to RAM (now 8 wide). Final widths: **DISK 6 | CPU 12 | RAM 8 | NET 6**. Single shared `LAYER_HALF_COLS = 3` constant again.
- Layered-split orientation: input on the inner edge of each strip (toward canvas centre), output on the outer. So Disk has write on the left (outer) and read on the right (inner); Net has download on the left (inner) and upload on the right (outer). `draw_layered_strip` parameterised with two `impl Fn(&IoSample) -> f32` picks so each call independently chooses which sample field feeds which side.
- Reclaimable-RAM colours pushed dim and shifted off the magenta hue: Buffers → dim violet `[30, 20, 60]`, Cached → very dim slate `[10, 8, 22]` (near the LED noise floor). Visually reads as "background, not committed", solving the previous problem of all RAM-coloured rows looking equally "used".
- Top-edge "dark pulse" while a core was pegged: traced to the alpha fade-in of the newest sample (`alpha = 0` at sample-time, ramping over the first 30 % of period) — visible as a periodic dim pulse at the top of the strip. Two-phase fix.  (i) Wider Gaussian + fixed kernel range — tried first, didn't feel right.  (ii) Reverted, then introduced `TOP_SLIDE_OFFSET = -1.0` so the newest sample lives at logical `y = -1` (one row off the top) just after sampling and slides into `y = 0` as `t` reaches 1. The Gaussian kernel naturally tapers as the dot crosses the edge — no alpha fade-in needed. Symmetric treatment for the bottom: `HIST_LEN` bumped from `LH = 64` to `LH + 2 = 66` so the oldest sample lives in an off-canvas row at `y = LH` and slides out smoothly.
- Dot kernel switched from Gaussian (σ = 0.7) to a **raised-cosine (Hann) window** at the same ±2 px support. Same total width, but flatter near the centre (G(0.5) = 0.85 vs Gaussian's 0.78) and exactly zero at the boundary. Less judder from sub-pixel sliding because small offsets cause smaller centre brightness changes; and the centre-rounded kernel range no longer creates visible jumps at half-integer crossings (boundary pixels have weight 0 anyway). `BLUR_SIGMA` constant dropped — kernel is now parameterised only by `BLUR_HALF_PIX`.
- CPU palette pushed apart for better contrast between the three categories. Final: user `[0, 170, 120]` teal, system `[50, 230, 230]` bright cyan, iowait `[80, 100, 255]` saturated blue. (User and system swapped from initial assignment because user-mode time is the most common during typical workloads, and bright cyan is the most attention-grabbing of the three.)

## Conclusion

Shipped a rotated, smooth, all-dots sysmon: **DISK 6 | CPU 12 | RAM 8 | NET 6** vertical strips on a 32 × 64 portrait canvas, dots flowing top-down at 20 fps with raised-cosine splatting and off-canvas sample rows above and below the visible area for smooth edge entry/exit.

Significant deviations beyond the Plan, all captured in the Log:

- CPU per-core ended up as **vertical** sub-strips (4 × 3 cols, full height) rather than the horizontal sub-bands the Approach had described. The user clarified after seeing the first build; restructure was self-contained inside `draw_cpu_strip`.
- Pack mapping reverted from `pack_rotated_ccw` to the original `pack_rotated_cw` (a 180° flip) once mounted on the panel.
- Layered-split orientation parameterised so input/output align with inner/outer edges of the canvas.
- Alpha fade-in dropped entirely; replaced by spatial slide-in via off-canvas imaginary rows. Required `HIST_LEN = LH + 2` and a `TOP_SLIDE_OFFSET = -1.0`.
- Gaussian kernel replaced by raised-cosine for less centre-judder at the same support width.

`hub75-client` bumped to **0.2.4**. README updated. No project changelog, no map. No further changes queued.

