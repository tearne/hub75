# Eight bands

## Intent

Try 8 bands × 8 rows = 64. Each band doubles its predecessor's cumulative aggregation: band 0 = raw samples (factor 1), bands 1–7 each have factor 2 from the band above. Cumulative samples per band: 1, 2, 4, 8, 16, 32, 64, 128.

Cadence agile.

## Conclusion

`BAND_COUNT` 4 → 8, `BAND_HEIGHT` 16 → 8, `AGGREGATION_FACTORS` `[1, 4, 4, 4]` → `[1, 2, 2, 2, 2, 2, 2, 2]`, `MAX_FACTOR` 4 → 2. No other code changes. 64 panel rows fit exactly. Bottom-of-panel wall-clock depth ~17 min at production (500 ms master), ~1.7 min at fast (50 ms).
