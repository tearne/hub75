# Banded dots

## Intent

Replace sysmon2's continuous time-compressed splat rendering with a banded scheme: the panel is split into a small number of horizontal bands (starting with **4 bands × 16 rows = 64**), each covering one aggregation timescale. Within a band the rendering is a standard scrolling display of integer rows; deeper bands aggregate from the band above with a fixed factor (starting **factor 4**), so rows age into progressively coarser timescales. All metrics — CPU, RAM, Disk, Net — switch to this scheme, with **random-dot density encoding** uniformly. Bar fill is removed.

This is stage 1: hard band boundaries, no smoothing between them, no sub-pixel slide. Each band ticks at its own rate; commits are discrete row shifts. A future change will explore sub-pixel slide or seam smoothing if the visible boundaries warrant it.

The shift is motivated by the per-row cascade approach hitting fundamental friction: every variant either washed to mush at the bottom (independent random per sample) or collapsed to fixed columns (inherited per sample) or showed visible jumps at row boundaries (per-row stateful with rate cascade). Bands sidestep the mush problem entirely by collapsing many-samples-per-row into one aggregate per row *before* the dot pattern is drawn.

## Approach

### Band geometry

Four bands stacked top-to-bottom on the panel, each 16 rows tall. Per metric, each band's rows live within the metric's column slice (no cross-metric layout change).

### Flow between bands

Each band has an **aggregation accumulator** — an invisible buffer that collects values from the band above. When it has accumulated `factor` (= 4) values, it computes the mean and commits one new row at the top of its own band, clearing the accumulator. The band-above's bottom row's data flows into the next band's accumulator on its way out.

Band 0 is fed directly from the metric's per-tick sample (no upstream band). Bands 1, 2, 3 are fed from the band above.

### Per-row rendering

Each row holds:
- An aggregated value `v`
- A random dot pattern (`n = max(1, round(v × W))` columns lit, chosen by a stable seed at commit time)
- An intensity `v` for per-row brightness

When a new row commits at the top of a band, the band's existing rows shift down by one and the oldest falls off the bottom (potentially feeding the next band's accumulator on its way).

### Master sampling rate and effective per-band rates

The master sampling rate is unchanged. Per-metric multipliers are unchanged (CPU 1, RAM 6, Net 6, Disk 20). Within a metric, each band's tick is a multiple of the metric's sample period:

- Band 0: every metric sample (period 1 × metric sample period)
- Band 1: every 4 metric samples
- Band 2: every 16
- Band 3: every 64

In production with master = 1 s and CPU multiplier = 1: bands 0/1/2/3 commit every 1 s / 4 s / 16 s / 64 s respectively. Bottom of CPU's deepest band reaches ~16 min back.

### What goes away

- Bar fill (centred, intensity-scaled) for the bar metrics — replaced by random dots per row.
- Continuous time-compression splat with sub-pixel y — replaced by integer-row rendering per band.
- The geometric window scheme `ceil(1.10^(r-1))` and the ring length derived from it — replaced by 4 × 16 fixed-size band rings per metric.
- All RAM-specific machinery: `RamStream`, per-panel-row state, inheritance accumulator, `inherited` mask, cascading shifts. RAM uses the same band scheme as the others.
- Brightness compensation `α = 0.5 / N(r)^0.65` — no longer needed since each row holds one aggregate, not many overlapping splats.

### What stays

- Per-metric colours and the column layout (CPU 4-wide cores, Disk/Net halves 3-wide, RAM 4-wide).
- Modes (`-f` for fast).
- LED floor `MIN_LED = 8` with proportional channel scaling.
- Display rotation.
- Per-metric multipliers and the master sampling rate concept.

### Map impact

Substantial: Time Compression node is no longer apt; the panel buffer concept goes away (or becomes the per-band ring); a new node describes bands and flow. Both Slate and the rendering nodes need rework. We'll do this after the code lands.

## Plan

Build it incrementally:

- [ ] Define `Band` data structure (per-band ring of `(value, pattern)` rows + aggregation accumulator).
- [ ] Define `MetricBands` — 4 stacked bands per metric stream.
- [ ] Implement band commit: when accumulator full, compute mean → commit new row → cascade evicted oldest into next band's accumulator.
- [ ] Implement row rendering: at each render frame, paint each band's rows at their fixed integer panel-row positions with the metric's colour scaled by row's value.
- [ ] Wire all metrics through band rendering. Remove bar fill, splat-with-windows, RAM-specific code paths.
- [ ] Update CLI banner / parameters where helpful.
- [ ] Update map.

Not in this change (stage 2): sub-pixel slide within bands, smoothing across band boundaries.

Cadence agile.

## Conclusion

What shipped:

- New `bands.rs` module: `Band` (per-band scrolling row buffer + aggregation accumulator) and `MetricBands` (one stack of 4 bands per metric).
- Slate slimmed to nine `MetricBands` (CPU × 4 cores, RAM, Disk read/write, Net down/up). All `Ring`, `RamStream`, `RamRow`, `RamPattern` types removed.
- Projection collapsed to ~80 lines: render every band's rows at integer panel y, intensity = `value × colour`. No splat, no Gaussian, no sub-pixel math, no window curve, no brightness compensation.
- Presentation: edge rows dropped (`TOTAL_ROWS = LOGICAL_HEIGHT = 64`), since hard-edge bands need no margin.
- Main loop unchanged in shape; `push_X_sample` simply calls `bands.push_sample(value)` on the metric's stack.
- All sample-rate multipliers set to 1 — every metric samples on every master tick. Tested briefly with non-uniform multipliers; uniform looked cleaner so it stuck.

Properties confirmed:

- All metrics' bands commit in lockstep at the "magic" sample numbers (4, 16, 64) — guaranteed because all metrics' band accumulators initialise identically and tick in sync under uniform multipliers.
- Compute drop is roughly two orders of magnitude (576 row paintings per render vs ~30,000 splats previously).

Map is now significantly stale — Time Compression, Slate's value-ring framing, splat machinery, RAM's whole subtree all describe code that no longer exists. Map update deferred to a follow-up change.

Stage 2 (sub-pixel slide / seam smoothing) was deemed unnecessary on visual inspection of stage 1 — the panel reads cleanly with hard band boundaries.
