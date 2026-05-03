# RAM streaks

## Intent

Re-enable time compression for RAM (back to `RAM_MULTIPLIER = 6`, dropping the linear-time mapping introduced in the previous change), but preserve the dot character without descending into uniform-mush at the bottom of the panel. The mush comes from random per-sample column choice averaging out under aggregation; the fix is **per-panel-row dot patterns that evolve over samples**, not over individual ring entries.

Algorithm: stateful buffer of one dot pattern per panel row (+ edges). On each RAM sample arrival, shift the buffer down by one row, regenerate the top with fresh random columns, and adjust each row's pattern (add/remove dots) to match its currently-aggregated value's target dot count. The result is dot streaks flowing downward — a dot lit at the top tends to persist down the panel as long as the metric stays at or above the level that lit it.

Between samples, the buffer is static and rendered with the existing Gaussian splat at sub-pixel y for smooth sliding.

Cadence agile.

## Conclusion

What shipped — the streak algorithm went through three iterations:

1. **First version**: stateful per-panel-row buffer that shifted by exactly one row per RAM sample regardless of panel depth. Easy to reason about but lost the time-compression deceleration — dots fell at constant speed instead of slowing toward the bottom.

2. **Second version**: per-ring-entry `(value, pattern)` store, patterns inherited between consecutive samples, splatted with the same time-compressed scheme as the bar metrics. Decel was correct, but under steady metric all entries had identical patterns and the whole RAM column collapsed into a single solid line of dots — no "travelling randomness".

3. **Final version (kept)**: stateful per-panel-row buffer with **per-row fractional accumulators** that increment at `1/N(r)` per RAM sample. Each row shifts independently when its accumulator crosses 1, inheriting from the snapshotted pre-shift pattern of the row above. Top reseeds with fresh independent randomness every sample. Sub-pixel sliding via `y = (r-1) + frac_offset + t/N(r)` keeps motion continuous through shifts (row r's y at end of period equals row r+1's y at start of period). Result: distinct random snapshots descending the panel, decelerating at the same curve as the bar metrics.

Identified during this change but **not** addressed (deferred to follow-up):

- Conceptual conflation in the code: `RamStream` bundles a sample-side ring (data-space) and a per-panel-row buffer (panel-space) under one name. The map describes both as "the slate" which obscures that they're at different layers.
- Need terms for the two layers (working terms: "sample slate" and "panel slate"). Map needs reorganisation around the distinction.

The follow-up change picks up these unresolved items: clarify the rendering, refine naming, possibly refactor, then update the map.
