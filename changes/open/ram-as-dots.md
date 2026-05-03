# RAM as dots

## Intent

Take RAM out of the unified time-compressed bar pipeline and give it its own visual treatment, distinct from the other metrics:

- **Random dot density**, not a bar. Each sample paints a small number of randomly-placed binary pixels across RAM's 4-column slice — sysmon1's density encoding, kept stable per sample by a deterministic seed.
- **No time compression.** The ring is linear: one ring entry per panel row, sliding straight down at one row per sample. The slowdown that other metrics get from the geometric window curve doesn't apply to RAM.
- **Slower sampling.** RAM's multiplier becomes 10 (so RAM samples every 10 master sample periods — 10 s in production, 500 ms in fast). With 64 visible rows and one entry per row, an entry traverses the panel in ~10 min wall-clock at production rate.

The combination gives RAM a distinct "rain falling at constant speed, peppered with dots" look that contrasts with the smooth slowdown of the other metrics — visually marks RAM as a different kind of thing.

Cadence agile.
