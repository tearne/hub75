# Continuous time compression

## Intent

Retry the falling-dots-slow-down idea with a continuous time compression instead of discrete stages. Maintain a virtual sample buffer much longer than the visible canvas, and assign each canvas row its own window into that buffer — windows progressively wider as rows go down. Top rows cover one or two raw samples (fast motion); bottom rows cover many (slow motion, more averaging). No hard boundaries, so the slow-down reads as a smooth gradient rather than a series of jumps.

Brightness compensation is part of the picture — averaging more samples per row dilutes individual activity, so lower rows need either an alpha or amplification step to read at comparable brightness to upper rows.

The "filling up" effect of the previous attempt (panel populating from top down as samples accumulate) is preserved automatically: each row's window only has data once enough samples have arrived, so lower rows stay dark during warm-up and gradually come online.

This is again experimental — willingness to abandon if it doesn't read well is part of the framing.

## Approach

### Single buffer per metric, much longer than the canvas

Replace each metric's display-sized buffer with a long ring buffer holding the last `BUFFER_LEN` raw samples — newest at front, oldest at back, capped via push-front / pop-back. `BUFFER_LEN` sums the window sizes of every canvas row plus two imaginary rows (one above, one below) for slide-in / slide-out continuity at the canvas edges.

### Window sizes grow geometrically with row

Each row's window size is `ceil(b^r)` for some base `b`. Base 1.04 gives `N(0)=1` at the top and `N(63) ≈ 12` at the bottom, summing to ~290 samples plus 2 imaginary rows ≈ **292 samples per metric**. The geometric growth gives an exponentially-curving slowdown — top rows turnover at full speed, bottom rows ~12× slower — without any visible breakpoints.

`WINDOW_SIZES`, `WINDOW_STARTS` (cumulative offsets into the buffer) and `BUFFER_LEN` are precomputed at compile time via `const fn` so the renderer just indexes into the tables.

### Render each individual sample at its own sub-pixel y

For each sample at buffer index `i`, find its `(row, w)` position — `row` is whichever row's window covers index `i`, and `w` is the offset within that window. Render the sample at logical y:

```
y = (row - 1) + (w + t) / N(row)
```

The `-1` shift puts the imaginary-above row off the top of the canvas. As `t` runs 0 → 1 within a sample period, every sample slides forward by `1/N(row)` of a pixel — fast in row 0 (1 px/period), slow in row 63 (~1/12 px/period). At each push, indices shift by 1 and the formula collapses cleanly across row boundaries (sample exiting row r, w=N(r)-1, t=1 lands at row r+1, w=0, t=0 with the same y), so motion stays continuous across the entire panel even though the speed varies with row.

### Brightness compensation: alpha = 1 / N(row)

Each sample is splatted at alpha `1/N(row)`. Because row r contains `N(r)` samples, the total brightness contribution of a row at activity level `f` works out to roughly `f × dot_count × N(r) × (1/N(r)) = f × dot_count` — the same as the top row would show for the same activity. Result: rows look equally bright at the same activity level regardless of how compressed they are.

A by-product of this is a nice character gradient down the panel — top rows show *discrete dots* (one sample per row, sharp-looking), middle rows show *speckle* (a few overlapping dim splats per row), and bottom rows show *smooth bars* (many overlapping dim splats blurring together). The visual texture shifts smoothly with the time-compression.

### Imaginary rows above and below

One imaginary row of `N=1` sits at logical row 0 (canvas y range [-1, 0]) so the newest sample slides into view smoothly. One imaginary row of `N=1` sits at logical row `LH+1` (canvas y range [LH, LH+1]) so the oldest visible sample slides off the bottom smoothly before being popped from the buffer. Same pattern as the previous version, generalised to the new window scheme.

### Sample seeds stay per-sample

Each sample keeps its own random `seed`. Within a row, the `N(row)` samples have different seeds → different lit columns — that's what gives the gradient from "discrete" up top to "smooth" at the bottom. No change needed.

### Sampler unchanged; cascade machinery removed

Sampler still pushes to the metric's single buffer at the metric's tick rate. The aggregation cascade (`accum_2`, `accum_3`) and the three-stage `MetricStages` struct retire — the staged design's complexity was carried by the cascade, and the new single-buffer / per-sample model has none of that.

### `_at` timestamp drives the sub-pixel `t`

Each metric still tracks `last_at` (Instant of latest push). Renderer computes `t = (now - last_at) / sample_period` clamped to `[0, 1]`. Same as the original (pre-staging) model.

## Plan

Going with `b = 1.04` (middle of the proposed range — visible slowdown without overstuffing the bottom) and `alpha = 1/N(r)` (cleanest math; iterate later if bursts feel too quiet at the bottom).

Version bump: `hub75-client` patch — **0.2.5 → 0.2.6**.

- [x] Bump `usb-display/client/rust/Cargo.toml` to `0.2.6`
- [x] Add `const fn` helpers to compute `WINDOW_SIZES: [usize; LH + 2]`, `WINDOW_STARTS: [usize; LH + 2]` and `BUFFER_LEN: usize`
- [x] Replace each metric's `VecDeque<T>` cap from `HIST_LEN` to `BUFFER_LEN`. Drop the `HIST_LEN` constant
- [x] Replace `TOP_SLIDE_OFFSET` with the new `window_y(r, w, t)` formula
- [x] Rewrite each `draw_*_strip` to use `for_each_window_sample` iteration
- [x] Thread per-sample `window_alpha(r)` through `paint_dot_row` calls (existing alpha parameter — just stop hardcoding `1.0`)
- [x] `cargo build --example sysmon_linux --features panel-64x32` succeeds; user runs and visually confirms continuous slowdown, no boundary discontinuities, and that brightness reads consistently across rows

## Log

- First run: bright spots visible at the top and bottom edges. Diagnosis: imaginary-row contribution + first visible row's contribution both landed at full alpha at the canvas edges, producing two full-alpha samples in the top/bottom row's kernel reach versus the more averaged (many low-alpha) middle. With per-sample seeded shuffles at sparse activity, this read as concentrated bright dots at the edges.
- Halved imag-row alpha (`0.5/N`). Helped a little but the edges still stood out.
- Dropped imag alpha further to `0.25/N`. Surprisingly made things worse — with imag damped almost to nothing, the visible top row dominated alone and the contrast between top "spots" and middle "smooth glow" became more pronounced. Also revealed that the brightness ramp visibly *darkened* going downward at sparse activity.
- Switched the base alpha rule from `1/N(r)` to `1/sqrt(N(r))` to flatten the perceived gradient. Lifted the lower rows but went too far — bottom became uniformly bright, gradient flipped to "brighter going down".
- Settled on `alpha = 0.5 / N(r)^0.7` for visible rows — sits between `1/N` and `1/sqrt(N)`. Per-column expected at the bottom roughly matches the top's peak brightness, so neither end dominates.
- Bumped the geometric base from `1.04` to `1.06` for a longer total time span (~12 min CPU history vs ~5 min) and a steeper visible slowdown (top-to-bottom rate ratio 39× vs 12×). Buffer grows to ~700 samples per metric (~85 KB total) — still trivial.
- RAM strip looked like a solid magenta lump because RAM has more dots per sample (sum of used/buffers/cached) than the others (4–5 dots over 8 cols vs CPU's 0–1 per core), so heavier overlap. Narrowed the blur kernel from σ=1.0 to σ=0.7 — adjacent-pixel contribution roughly halves, so dots read as more distinct.
- Final pass on the top edge: imaginary-above contribution still produced a faint bright glow at canvas row 0. Reduced its alpha multiplier all the way to `0.1 / N(r)^0.7` (vs `0.5` for visible rows) so it just barely contributes — enough to keep the edge slide smooth without dominating row 0.

## Conclusion

Shipped continuous time compression. Each metric keeps a single ring buffer of ~700 raw samples; the renderer iterates each sample at a sub-pixel y position derived from a per-row geometric window table (`ceil(1.06^(r-1))`). The full slow-down arc from N=1 at the top of each strip to N≈39 at the bottom reads as a continuous gradient — no discrete bands, no boundary discontinuities.

Final tuned constants after iteration:

- Geometric base **1.06** (window ratio top-to-bottom ≈ 39×, ~12 min CPU history visible).
- Visible-row alpha **`0.5 / N(r)^0.7`** — between strict `1/N` (uniform expected) and `1/sqrt(N)` (over-correction). Settled on the exponent that puts bottom uniform brightness at roughly top peak brightness.
- Imaginary-row alpha **`0.1 / N(r)^0.7`** — much dampened so the edge slide-in/out doesn't pile up bright contributions at the canvas top/bottom.
- Gaussian blur **σ = 0.7** (was σ=1.0 from the previous change) — narrower kernel keeps dots distinct, prevents the RAM strip's higher dot count from smearing into a solid lump.

All four are single-constant retunes if behaviour drifts later.

`hub75-client` bumped to **0.2.6**. No project changelog, no map. README unchanged (visual difference is in feel; previous descriptions still read correctly).

## Conclusion

Shipped continuous time compression. Each metric now keeps a single ring buffer of ~316 raw samples, and the renderer iterates each sample at a sub-pixel y position derived from its window-table lookup (`y = (r − 1) + (w + t) / N(r)`) with brightness compensation `alpha = 1 / N(r)`. Window sizes grow geometrically (`ceil(1.04^(r-1))`) from N=1 at the top of the canvas to N≈12 at the bottom — slowdown is smooth and continuous, no discrete bands.

The earlier staged-cascade machinery (`MetricStages`, accumulators, per-stage timestamps, `Aggregable` trait) is gone. The retained pieces from before that change still apply: the imaginary-row-above-and-below trick for smooth edge entry/exit, per-sample seeded shuffles, the 2-D Gaussian dot splat.

`hub75-client` bumped to **0.2.6**. No project changelog, no map. README unchanged (the visual difference is in feel; the description from the previous change still reads correctly).

