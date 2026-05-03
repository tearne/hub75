# Inter-band pattern flow

## Intent

Make patterns flow visually across band boundaries. Currently each band commit picks a fresh independent random dot pattern, so the seam between (say) the bottom of band 0 and the top of band 1 shows two unrelated patterns. This change derives the new commit's pattern from the most recent non-blank upstream value's pattern (the row that just evicted from the band above), adjusting up or down to match the new aggregated value's target dot count. Brightness still comes from the aggregated value.

The accumulator becomes a small ring of `BandRow` (value + pattern) entries — the same machinery the visible-buffers experiment used invisibly, but its purpose now is pattern continuity at commit time, not display.

If the latest accumulator entry is blank (zero pattern), walk backward to the next-most-recent non-blank one. If all are blank (metric idle), fall back to a fresh random pattern.

Cadence agile.

## Conclusion

What shipped:

- Each band's accumulator now stores `BandRow` (value + pattern), not just running sum + count. The accumulator capacity is `MAX_FACTOR = 4`, which covers all current bands.
- At commit time, the new band row's pattern is derived from `latest_non_blank_pattern(accumulator)` rather than freshly random. If `target_n` matches the basis's dot count, the pattern flows through unchanged — no randomness used.
- When `target_n` differs from the basis's count, the choice of which dots to add or remove is biased by **column frequency across the full accumulator**: adds prefer columns lit by *other* accumulator entries (consensus), removes prefer columns lit *only* by the basis (outliers). Tie-breaks by hash.
- Random fallback for `basis = 0` removed — it's redundant. `flow_pattern(0, ..., target_n, seed)` with `target_n > 0` already picks `target_n` columns by frequency-then-hash, which matches the previous fresh-random behaviour for empty accumulators (band 0's case, every commit). Code is uniform.

Visual effect: at band boundaries, the new top row's lit columns track the accumulator's history. Steady metric → pattern preserved exactly. Changing metric → only the differing dots get adjusted, with the new column biased toward what the recent history was already showing.

Map remains stale.
