# HSL palette

## Intent

The OKLCh perceptually-uniform palette didn't work out visually — colours felt muddy and the deep-blue gamut squeeze made CPU dull. Revert to a plainer HSL approach.

Six colours at equal 60° hue spacing, with a 30° rotation offset from the standard primaries so no metric is rendered at a pure primary or secondary — everything is a mix, rooted in **orange (30°)**. Hue set: `{30°, 90°, 150°, 210°, 270°, 330°}`.

Per-metric assignments fixed by the user:

- **CPU** at **210°** (deepest blue).
- **Disk read** at **150°** (green-cyan).
- **RAM** at **330°** (pink-magenta).

The remaining slots (30°, 90°, 270°) are to be assigned to Disk write, Net down, Net up.

For deviation expression, **above-mean rows render at full saturation; below-mean rows drop saturation**. Same hue and lightness, just less colourful. Replaces the OKLCh "warm = max chroma, cool = uniform chroma" scheme.

## Approach

### Per-metric hue assignments

| Metric     | Hue   | Identity (HSL) |
|------------|-------|----------------|
| Disk write | 30°   | orange         |
| Disk read  | 150°  | green-cyan     |
| Net down   | 90°   | yellow-green   |
| CPU        | 210°  | deep blue      |
| Net up     | 270°  | purple         |
| RAM        | 330°  | pink-magenta   |

Hues hard-coded in `projection.rs` as `f32` constants. No runtime computation.

### HSL → RGB at fixed lightness

Single global `L = 0.5` (maximally saturated point in HSL). Saturation has two values: `S_HIGH = 1.0` (above-mean rows), `S_LOW = 0.4` (below-mean rows). At paint time, the metric's hue plus the row's `above_mean` flag select the saturation; HSL→RGB conversion produces the painted pixel colour.

Conversion is a small standard formula. Already lives in the codebase (the original `hsl_to_rgb` from before the OKLCh switch was removed in the perceptual-colour-uniformity change but is straightforward to reintroduce).

### `oklch.rs` stays — for now

The `oklch.rs` module is no longer used by `projection.rs` after this change but isn't deleted. Removing it can be a separate cleanup pass; keeping it during build-eyeball makes reverting easy if HSL also disappoints.

## Plan

- [x] In `projection.rs`, replace `METRIC_HUES: [f32; 6]` with the new hue table. Update the per-metric index assignments so each metric points at its new hue slot.
- [x] Replace the `optimal_lc()` / `variants_for()` / `WARM_CHROMA_FACTOR` machinery with a small `hsl_to_rgb(h, s, l) -> Pixel` helper plus a `variants_for(metric_idx) -> ColourPair` that returns `(low_sat_rgb, high_sat_rgb)` for that metric's hue.
- [x] Add `S_HIGH = 1.0`, `S_LOW = 0.4`, `L_FIXED = 0.5` constants. Drop the now-unused `optimal_lc` and OKLCh imports from `projection.rs`.
- [x] Update doc comments at the top of `projection.rs` to describe the HSL scheme.
- [x] `cargo build --release` and eyeball.