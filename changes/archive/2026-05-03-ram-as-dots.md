# RAM as dots

## Intent

Take RAM out of the unified time-compressed bar pipeline and give it its own visual treatment, distinct from the other metrics:

- **Random dot density**, not a bar. Each sample paints a small number of randomly-placed binary pixels across RAM's 4-column slice — sysmon1's density encoding, kept stable per sample by a deterministic seed.
- **No time compression.** The ring is linear: one ring entry per panel row, sliding straight down at one row per sample. The slowdown that other metrics get from the geometric window curve doesn't apply to RAM.
- **Slower sampling.** RAM's multiplier becomes 10 (so RAM samples every 10 master sample periods — 10 s in production, 500 ms in fast). With 64 visible rows and one entry per row, an entry traverses the panel in ~10 min wall-clock at production rate.

The combination gives RAM a distinct "rain falling at constant speed, peppered with dots" look that contrasts with the smooth slowdown of the other metrics — visually marks RAM as a different kind of thing.

Cadence agile.

## Conclusion

What shipped:

- New `RamRing` in `slate.rs` storing pre-laid pixel rows (one per panel row + edges, 66 total). Distinct from the other metrics' scalar-value `Ring`.
- `RAM_MULTIPLIER` 6 → 10 (RAM samples every 10 master periods).
- New rendering path in `projection.rs`: `splat_ram_ring` and `render_ram_row` — random-dot density encoding (`n = max(1, round(v × W))` columns lit, deterministic by seed) with each lit pixel's intensity scaled by `value` so both dot *count* and per-dot *brightness* rise with the metric.
- RAM splat uses linear time mapping: ring entry at index `i` is at sub-pixel `y = (i − 1) + t`. No window-aggregation curve.
- Map's RAM and Slate nodes updated to describe RAM's separate pipeline.

Constraint discovered (motivates the next change): without time compression, RAM only reaches ~10 min back at the bottom of the panel — much shallower than CPU's ~55 min. The next change explores re-enabling time compression for RAM while preserving dot character.
