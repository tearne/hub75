# Three bands, factor-9 aggregation

## Intent

Experimental change to sysmon2's banding scheme. Reduce from 8 bands to 3, with an aggregation factor of 9 at each band boundary instead of 2. Keeps a similar total bottom-of-panel timescale but in larger, fewer aggregation steps — giving each band more vertical real estate and a wider gap between adjacent bands' timescales.

## Approach

### Band layout

Three bands totalling the full 64-row panel. Band 0 gets the extra row to favour the most-active timescale: `[22, 21, 21]`.

### Aggregation factors

`[1, 9, 9]` — band 0 takes raw samples, bands 1 and 2 each aggregate 9 evicted rows from the band above into one of their own. Cumulative samples per row from top to bottom: 1, 9, 81.

### Cadence

At the 500ms production rate, bottom of panel reaches `0.5 × (1×22 + 9×21 + 81×21) = 0.5 × 1912 = 956 s ≈ 16 minutes`. Comparable to the current 17-minute reach.

### Code shape

Most of `bands.rs` stays intact. The fixed `BAND_HEIGHT` constant becomes per-band, since bands no longer have uniform height. Two minimal-friction options for the storage:

- Keep `rows: [BandRow; MAX_BAND_HEIGHT]` on each `Band`, plus a `height: usize` field. Some array slots in the shorter bands are unused but the storage stays on the stack.
- Switch to `rows: Vec<BandRow>`. Heap-allocates per-band but cleaner.

Defaulting to the first (stack array, used-prefix). Less change to the surrounding code's lifetime story.

`MAX_FACTOR` rises from 2 to 9 — the accumulator becomes a 9-slot array on each band. Storage cost: 9 metrics × 3 bands × 9 slots × `BandRow` (~9 bytes) ≈ 2KB. Trivial.

`projection.rs` iterates bands using each band's height instead of the constant `BAND_HEIGHT`.

## Plan

- [x] In `bands.rs`: change `BAND_COUNT` to 3, replace `BAND_HEIGHT` const with `BAND_HEIGHTS: [usize; BAND_COUNT] = [22, 21, 21]` and `MAX_BAND_HEIGHT: usize = 22`. Change `AGGREGATION_FACTORS` to `[1, 9, 9]`. Bump `MAX_FACTOR` to 9.
- [x] Add `height: usize` field on `Band`; initialise from `BAND_HEIGHTS[band_idx]` in `MetricBands::new`.
- [x] Change `Band::push` to use `self.height` instead of `BAND_HEIGHT` for the row-shift bound and eviction index.
- [x] In `projection.rs::render_metric`, iterate `0..band.height` instead of `0..BAND_HEIGHT` and use `band_idx` cumulative offset for `band_top` (sum of preceding heights).
- [x] Update doc comments at the top of `bands.rs` to describe the new scheme.
- [x] `cargo build --release` and eyeball.

## Conclusion

Landed factor **12**, not 9 as the title suggested — the user bumped it after the initial eyeball at 9. Cumulative samples per row: 1, 12, 144. At 500ms production rate the bottom of the panel now reaches ~27.5 min of history (vs ~17 min in the previous 8-band scheme).

File renamed on archive to `three-bands-factor-twelve.md` to reflect what shipped.
