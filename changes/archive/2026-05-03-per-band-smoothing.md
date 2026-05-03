# Per-band smoothing

## Intent

Try Gaussian smoothing inside each band, with sub-pixel vertical sliding between commits — both clipped to the band the row belongs to (no bleed across boundaries). Decoupled render rate stays.

How it works:

- Each band tracks its last commit time.
- At render time, each band has a fractional offset `t ∈ [0, 1)` = `elapsed_since_last_commit / commit_cycle`. As real time elapses between commits, `t` advances 0 → 1.
- Each band-row is splatted at sub-pixel `y = band_top + row_idx + t` with a vertical Gaussian envelope (σ ≈ 0.7).
- The Gaussian is clipped to the band's panel-row range — tails that would extend into the band above or below are cut. Hard boundaries between bands stay hard.

When `t = 1` and the next commit fires, the band's rows shift down by 1 in the data, `t` resets to 0. The visible position is continuous: row 0 at `t = 1` is at `band_top + 1`, exactly where row 1 (which inherits row 0's data after the shift) renders at `t = 0`.

Smoothing comes from the Gaussian: each row's contribution bleeds slightly into adjacent integer rows within its band, softening the integer-step look while sliding.

Cadence agile.

## Conclusion

Tried, reverted. Visual didn't earn its complexity. Code restored to integer-aligned per-row band rendering; `last_commit_at`, `CUMULATIVE_FACTORS`, sub-pixel sliding, and the clipped Gaussian splat all removed. The render-once-per-sample idea (still in `changes/open/`) remains live.
