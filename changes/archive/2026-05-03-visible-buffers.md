# Visible buffers

## Intent

Make the inter-band aggregation accumulators visible. Currently each accumulator collects 4 upstream values silently before committing one row to its band; this change renders those pending values as a small "buffer" area between bands, so you can see data flowing in and "filling up" before the next band-row commits. When the 4th value arrives the buffer flushes (visibly clears) and the new row appears at the top of the band below.

Also: drop the production master sampling rate from 1000 ms to 500 ms.

Stage 1 (banded dots) committed quietly with no inter-band motion. This change adds a small temporal animation between bands that should make the structure of the rendering more legible.

## Approach

### Panel layout

| Region    | Panel rows | Height |
|-----------|------------|--------|
| Band 0    | 0–13       | 14     |
| Buffer 1  | 14–16      | 3      |
| Band 1    | 17–30      | 14     |
| Buffer 2  | 31–33      | 3      |
| Band 2    | 34–47      | 14     |
| Buffer 3  | 48–50      | 3      |
| Band 3    | 51–63      | 13     |

3 inter-band buffers × 3 rows + 4 bands (14+14+14+13) = 64. Bands shrink slightly from 16 to 14/13 to make room.

### Buffer mechanics

Each accumulator (for bands 1, 2, 3) is replaced by a small fixed-size buffer of `(value, pattern)` entries — capacity `factor - 1` = 3. On each push:

- New entry inserted at index 0 (most recent at bottom of buffer when rendered).
- If at capacity (`factor - 1`) and a 4th value arrives: compute the mean of all 4 (3 buffered + this one), commit one row to the band below, clear the buffer.

Each buffer entry has its own dot pattern, computed at insertion time from its individual upstream value (not the eventual mean). So the buffer shows individual pre-aggregation samples; the band shows aggregated commits. The visual hand-off: 3 buffer rows clear, 1 band row appears.

### Master sampling rate

Production: `1000 ms` → `500 ms`. Faster scroll than today; CPU still very low. Bottom of panel reaches half the wall-clock depth (~10 min for the deepest band).

Cadence agile.

## Conclusion

What landed:

- **Production master sampling rate dropped from 1000 ms to 500 ms.** Bottom of panel reaches half the wall-clock depth (~10 min for the deepest band) but visual scroll is twice as lively.

What was tried and reverted:

- **Visible buffers between bands.** First as a fill-up-and-flush pattern (where the buffer accumulated upstream values and cleared on commit). Looked correct on band 0→1 (buffer fills in ~2 s) but the deeper buffers spent most of their lives black (buffer 3 dark for ~24 s out of every ~32 s cycle). Pivoted to a rolling-last-N variant (buffer always shows the most recent 3 upstream values, never clears). Looked sensible — until we noticed it wasn't visually distinct from just extending each band by 3 rows. The rolling variant gave no new information per row; it was just a different slice of where bands ended and started. So the buffer experiment as a whole didn't add value and was reverted.
- Layout returned to the stage-1 plain-bands shape: 4 bands × 16 rows = 64, no inter-band gaps, no buffers.

Lesson for any future attempt at inter-band visualisation: a meaningful buffer needs to show something the bands don't already show — either a clear "filling and flushing" temporal phase (with ugly dead space at deep bands) or a fundamentally different visual element (separator line, accumulator progress dot, etc.). Neither was worth pursuing further at this point.

Map remains stale (Time Compression / panel buffer / RAM streaks all still describe code that no longer exists). Map update remains the next change.
