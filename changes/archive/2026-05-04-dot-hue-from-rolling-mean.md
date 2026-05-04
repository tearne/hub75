# Dot hue from rolling mean

## Intent

In `sysmon2`, encode local deviation from recent activity in **hue** rather than brightness. A row's dots are rendered in the metric's colour rotated **+30°** (warmer) when the row's value sat above its band's pre-commit rolling mean, and **−30°** (cooler) when below. Brightness becomes uniform — every dot is fully lit; the deviation signal lives entirely in colour.

The intent is to make local deviations — bursts and lulls relative to recent activity — visually salient at every timescale, while preserving the metric's colour identity (a ±30° rotation stays unmistakeably "the same metric"). A row's hue reads as "this sample, against what the band looked like just before it arrived" — a stable verdict, fixed at commit and carried through the band.

This is an experiment in `sysmon2` only.

## Approach

### Rolling mean = band's visible rows at commit time

Each band already holds its 8 most-recent committed rows (`band.rows`). When a new row is about to be committed, take the mean of those 8 existing values — the rows currently on screen, before the oldest is shifted off. That is the comparator for the incoming row's value. No extra state, no separate window.

### Brightness is fixed at commit, stored on the row

The three-level step (`BASE − STEP` / `BASE` / `BASE + STEP`) is computed once, at the moment the new row is committed, against the pre-commit band mean. The result is stored on `BandRow` itself and travels with the row through every subsequent frame. Painting then becomes a straight read — no per-frame mean computation, no drift as later rows arrive.

This is the key shift from the first attempt, where brightness was recomputed every frame from the band's *current* mean. Under that earlier rule, a row's brightness would change as new rows arrived and shifted the mean. Now the verdict is final at entry.

### Two-level hue shift

At commit time the row records a single bit: did its value sit at or above the band's pre-commit mean (`above_mean = true`) or below (`false`). Painting picks one of two pre-shifted colour variants per metric:

- `above_mean` → metric colour with hue rotated **+30°** (warmer).
- otherwise → metric colour with hue rotated **−30°** (cooler).

The shifted variants are computed once per frame from each metric's base RGB via an HSL round-trip, and passed into the metric's render pass. No per-pixel hue maths.

### Brightness becomes uniform

With deviation expressed in hue, dot brightness is no longer doing work. Every lit dot paints at full intensity (1.0). The previous `BASE`/`STEP`/`intensity` machinery is removed.

### Dot placement and count unchanged

Pattern logic in `bands.rs` (dot count from `value`, basis-and-flow pattern derivation) is untouched. Only `BandRow` gains an intensity field; `Band::push` populates it; `paint_row` reads it.

## Plan

- [x] Simplify `commit_intensity` in `bands.rs` to the two-level rule (`>=` chooses `BASE + STEP`, else `BASE - STEP`). Remove the `K` constant.
- [x] Update doc comments to match.
- [x] `cargo build --release` and eyeball.
- [x] Replace `intensity: f32` field on `BandRow` with `above_mean: bool`. Drop `BASE` and `STEP` constants from `bands.rs`.
- [x] In `Band::push`, set `above_mean = (value >= prior_mean)`.
- [x] Add a small `hue_shift(rgb, degrees) -> Pixel` helper (RGB→HSL→shift→RGB) in `projection.rs`.
- [x] In `render_canvas`, compute the warm/cool variant pair for each metric once per frame and pass into `render_metric` instead of a single colour. Constant `HUE_SHIFT_DEGREES = 30.0`.
- [x] In `paint_row`, pick the colour from the variant pair using `row.above_mean`; paint at fixed intensity 1.0.
- [x] Update doc comments at the top of both files to describe the hue rule.
- [x] `cargo build --release` and eyeball.

`sysmon2` is experimental and not yet released, so no version bump.

## Conclusion

Iterated through four rules in build before settling on the final shape: three-level brightness step → continuous tanh brightness → two-level brightness → ±30° HSL hue rotation with uniform brightness. The change document was rewritten between iterations (no log preserved) at the user's direction.

One follow-up proposal spawned during build, captured in `changes/open/`:

- `perceptual-colour-uniformity.md` — replace HSL with OKLCh for the warm/cool variants, and brightness-match the six base metric colours.

Map: `Row Rendering` node in `usb-display/client/sysmon2/map.md` still describes intensity as `value × colour` — wrong post-change, but deliberately left for a later catch-up.

File renamed on archive to `dot-hue-from-rolling-mean.md` to reflect the final shape (started as a brightness experiment, ended as a hue experiment).
