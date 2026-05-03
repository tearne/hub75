# Bar rendering

## Intent

Replace sysmon2's per-metric density encoding (binary on/off pixels with random column placement, min-1-pixel rule) with a deterministic bar where the metric's value is encoded as continuous illumination across the column slice. Treated as an experiment: in production mode the existing scheme produces uniformly-lit bottom rows even at low CPU activity, because random column placement averages to all columns at the bottom of the panel. A bar is the simplest fix that addresses both root causes (random placement, always-1 floor).

Side benefit: ring entries can now hold a scalar value rather than a pre-laid pixel row, since the column pattern is a deterministic function of the value alone.

Cadence remains agile, as for the sysmon2 build session.

## Conclusion

What shipped:

- Each ring entry is now a scalar `f32` value instead of a pre-laid pixel row. Row rendering is just `ring.push(value)`; the pixel pattern is computed deterministically from the value at vertical-mapping time.
- Vertical mapping paints each entry as a uniform-intensity bar across the metric's column slice (every column in the slice lights at intensity = value × colour). This replaces the random-column-placement density scheme, which created uniformly-lit bottom rows even at low load.
- A first attempt used a length-varying bar (`coverage = clamp(L − col, 0, 1)`); abandoned because at typical CPU loads only the leftmost column lit, looking like single dots rather than bars.
- Splat accumulator switched from `u32` to `f32` to stop tiny per-entry contributions truncating to zero individually at the bottom rows; floats sum properly and convert to `u8` only at output time.
- Added an LED hardware floor `MIN_LED = 8`: sub-threshold-but-non-zero pixels are scaled so their dominant channel reaches the floor, with other channels scaling proportionally. Preserves colour ratio (avoids dim cyan washing to grey) while ensuring sub-threshold contributions actually drive the LEDs.

Map updates: Slate, Row Rendering, and Vertical Mapping nodes were rewritten to describe the new scheme. The window scheme survives unchanged.

No version bump (sysmon2 still v0.1.0; the crate hasn't been released).
