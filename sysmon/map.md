# sysmon

[Down](#devices)
[Down](#metric)
[Down](#display)
[Down](#modes)
[Down](#a-b-palette-mode)

Captures host metrics and renders them as a banded waterfall on the LED panel. Each metric occupies a vertical column slice, with rows aggregated at increasing timescales down the panel.

```
sysmon
├ Devices
│ ├ CPU
│ ├ RAM
│ ├ Disk
│ └ Network
├ Metric
├ Display
│ ├ Projection
│ │ ├ Row Rendering
│ │ └ Bands
│ └ Presentation
├ Modes
└ A/B Palette Mode
```

# Devices

[Up](#sysmon)
[Down](#cpu)
[Down](#ram)
[Down](#disk)
[Down](#network)

The four host resource categories rendered on the panel: CPU, RAM, Disk, Network. Each device contributes one or more **metrics** — a metric is a single vertical column slice of the display. Across the four devices, sysmon renders **nine metrics** in total. Every metric is a single value (one fraction in [0, 1] per sample); finer breakdowns become more metrics, not dimensions of one metric.

**See also**

- [Metric](#metric) — defines what a metric is and how each is normalised.

# CPU

[Up](#devices)

Four metrics, one per Pi-5 core. Each metric is one-dimensional: a single "total busy" fraction in [0, 1] of the sample interval, summed from the kernel's user (application work), system (kernel work), and iowait (waiting on disk) categories. The remaining kernel categories (nice, irq, softirq, steal) fold into idle and don't contribute. If finer detail is wanted later — distinguishing user from system, etc. — we'll split those into separate metrics rather than re-introducing per-metric dimensions.

**Detail**

Source: `/proc/stat`. Each `cpuN` line gives cumulative jiffies per category since boot; per-sample fractions come from differencing against the previous reading. Sampled at the sampling rate. Core count discovered at startup.

**See also**

- [Modes](#modes) — owns the sampling rate.

# RAM

[Up](#devices)

One metric, one-dimensional: committed memory — memory in use that isn't reclaimable cache or buffers. A fraction in [0, 1] of total system RAM.

**Detail**

Source: `/proc/meminfo`. Computed as `(MemTotal − MemAvailable) / MemTotal`. `MemAvailable` is the kernel's estimate of memory available for new processes, accounting for reclaimable cache; subtracting it from total gives the working set (matches `btop` and `free`'s "used" reading). Sampled at the sampling rate.

**See also**

- [Modes](#modes) — owns the sampling rate.

# Disk

[Up](#devices)

Two metrics, each one-dimensional: read (bytes/sec) and write (bytes/sec). Each is sampled at the sampling rate by differencing cumulative byte counters against the previous reading and dividing by the elapsed interval.

Read and write each carry their own scale, log-normalised between a near-zero floor and a per-channel running peak. A read-heavy workload therefore doesn't dampen the visibility of write activity, and vice versa.

**Detail**

Source: per-block-device byte counters from `/proc/diskstats`, summed across whole block devices (loop, ram, dm-* virtuals and partitions are skipped — partitions detected by `/sys/class/block/<name>/partition`).

Log-scale endpoints: floor `MIN_BPS = 1` (1 B/s — anything below normalises to 0; effectively any non-trivial activity registers). Starting upper end `MAX_FLOOR = 10,000` (10 KB/s) per channel; each channel's running peak only ever grows from there, so the scale stretches as activity bursts but doesn't shrink back during quiet periods.

**See also**

- [Metric](#metric) — defines per-metric normalisation.
- [Modes](#modes) — owns the sampling rate.

# Network

[Up](#devices)

Two metrics, each one-dimensional: down (bytes/sec received) and up (bytes/sec sent). Each is sampled at the sampling rate by differencing cumulative byte counters against the previous reading and dividing by the elapsed interval.

Down and up share a single log scale, normalised between a 10 KB/s floor and one running peak across both directions — so the bands are directly comparable: a taller band genuinely means more bytes/sec. Disk diverges by keeping per-channel peaks.

**Detail**

Source: per-interface byte counters from `/proc/net/dev`, summed across non-loopback interfaces, differenced per sample.

Log-scale endpoints: floor at 10 KB/s, starting upper end at 200 KB/s. The shared peak grows from there as bursts arrive in either direction.

**See also**

- [Metric](#metric) — defines per-metric normalisation.
- [Modes](#modes) — owns the sampling rate.

# Metric

[Up](#sysmon)

A single vertical column slice of the display, fed by one stream from a device. Sampled at the sampling rate. Each metric carries one value (a fraction in [0, 1]) per sample; sysmon keeps metrics one-dimensional by construction — finer-grained breakdowns are modelled as additional metrics rather than as dimensions of one.

Each metric carries its own scale. Every sample is normalised into [0, 1] before it enters the rendering pipeline, so a metric's panel intensity is independent of underlying units. CPU and RAM normalise implicitly (already fractions of an interval, or of total memory). Disk read, Disk write, Network down, and Network up each carry an explicit per-metric scale that floats with the maximum value observed for that metric since the application started.

**See also**

- [Devices](#devices) — the four resource categories that contribute the nine metrics.
- [Modes](#modes) — owns the sampling rate.

# Display

[Up](#sysmon)
[Down](#projection)
[Down](#presentation)

A 64×32 HUB75 LED matrix, mounted in portrait — sysmon renders into a 32-wide × 64-tall logical canvas, with a single 90° rotation applied at output time to match the panel's native orientation. All rendering work upstream of that final rotation works in the portrait frame.

Driven over a USB-display protocol via the `hub75-client` crate (shared with the existing `sysmon`). The host emits u8 RGB per pixel; brightness and gamma handling live downstream in the firmware, so values written into the canvas reach the panel without further shaping.

**Detail**

A new frame is produced each time a sample is taken — Projection runs once per sampling cycle, immediately after the sample is pushed into the bands.

Rotation: output packing rotates the 32×64 portrait buffer 90° clockwise into the panel's 64×32 native landscape frame. The user's top-of-view edge maps to the panel's right edge; the user's left edge maps to the panel's top. Concretely, logical pixel `(x, y)` in the portrait buffer is written to native pixel `(63 − y, x)` in the wire frame.

# Projection

[Up](#display)
[Down](#row-rendering)
[Down](#bands)

Maps metric data into panel-coordinate space. **Row rendering** turns a band-row's value and pattern into the panel pixels for its column slice. **Bands** organise each metric's history into a stack of scrolling-displays at increasing aggregation timescales, with patterns flowing across band seams.

# Row Rendering

[Up](#projection)

When a band commits a row, its value and dot pattern are produced by the band-aggregation logic — see [Bands](#bands). Row rendering's job is to turn that `(value, pattern)` pair into pixels in the metric's column slice on the panel: each lit column in the pattern is painted at intensity `value × colour`, integer-aligned to the panel row; unlit columns stay dark.

**Detail**

Per-metric colours (carried from sysmon's tuned spectrum):

| Metric     | Colour (RGB)       | Hue          |
|------------|--------------------|--------------|
| Disk write | `[240,  50,  50]`  | red          |
| Disk read  | `[255, 175,  40]`  | amber-gold   |
| CPU 0–3    | `[ 50, 230, 230]`  | cyan         |
| RAM        | `[230,  60, 170]`  | magenta      |
| Net down   | `[ 40, 220,  90]`  | green        |
| Net up     | `[200, 240,  40]`  | yellow-lime  |

The CPU colour is shared across all four cores; cores are visually distinguished by spatial position, not colour. Going around the wheel: red → amber → cyan → magenta → green → lime.

**See also**

- [Bands](#bands) — produces the `(value, pattern)` pairs that this node renders.

# Bands

[Up](#projection)

Each metric's column slice is split into 8 stacked **bands**, each at a different aggregation timescale. Band 0 (top) holds raw samples; deeper bands aggregate 2 evicted rows from the band above into one of their own. Cumulative sample count per row doubles down the panel: 1, 2, 4, 8, 16, 32, 64, 128 at bands 0–7.

Within a band, rendering is a standard scrolling display — 8 rows, newest at top, oldest evicted off the bottom. Each band has an aggregation accumulator that collects upstream values until the band's factor (2 for bands 1–7; 1 for band 0) is reached.

At commit, the new row's value is the mean of the accumulated values. Its dot pattern starts from the **latest non-blank** entry in the accumulator (the basis), and is adjusted to match the new value's target dot count:

- When the count matches the basis, the pattern flows through unchanged — no randomness.
- When dots need to be added, the new columns are drawn from elsewhere in the accumulator's history, biased toward the columns most frequently lit there (consensus across recent rows).
- When dots need to be removed, the dropped column is biased toward those least frequently lit (outliers from the basis).

So **patterns stay continuous at band seams** for steady metrics; under change, basis columns are preserved where possible and additions/removals are guided by the whole accumulator, not just the latest entry.

**Detail**

Aggregation factors `[1, 2, 2, 2, 2, 2, 2, 2]`. At production master rate 500 ms, bottom-of-panel reaches ~17 min back; at fast (50 ms) ~1.7 min.

**See also**

- [Row Rendering](#row-rendering) — how each band-row's value and pattern become panel pixels.

# Presentation

[Up](#display)

Decides which columns each metric occupies on the panel and in what order. CPU cores are interleaved between the other metrics, left to right across the 32-column logical canvas: Disk write, CPU0, Disk read, CPU1, Net down, CPU2, Net up, CPU3, RAM. Each CPU core acts as a visual separator between the non-CPU metrics, and CPU activity is visible across the whole width of the panel rather than clustered. RAM sits at the end so that under the hourly screen-burn shift the wrap-adjacency is RAM↔Disk write rather than Net↔Disk.

**Detail**

Column widths per metric, left to right (CPU cores interleaved between non-CPU metrics):

| Metric     | Cols  | Width |
|------------|-------|-------|
| Disk write | 0–2   | 3     |
| CPU0       | 3–6   | 4     |
| Disk read  | 7–9   | 3     |
| CPU1       | 10–13 | 4     |
| Net down   | 14–16 | 3     |
| CPU2       | 17–20 | 4     |
| Net up     | 21–23 | 3     |
| CPU3       | 24–27 | 4     |
| RAM        | 28–31 | 4     |

CPU cores at width 4 (more visual presence per core); Disk and Net halves at width 3; RAM at width 4. Sums to 32. CPU cores carry more emphasis since on a desktop workload the four cores are the most active and most differentiated metrics.

# Modes

[Up](#sysmon)

sysmon runs in one of two modes selected at startup. The pipeline is identical between them — same data sources, same rendering — only the sampling rate differs. Sampling and rendering happen in lockstep: one frame is produced per sample.

- **Production (default)** — slow: sampling rate 500 ms. Bottom of panel reaches ~17 min back.
- **Fast** — for development: sampling rate 50 ms. Bottom of panel reaches ~1.7 min back. Quicker visual feedback when tuning. Uses more CPU.

Selected via CLI flag `-f` for fast; absence of the flag selects production. Production is the default so the systemd-installed binary runs at low CPU without further flags.

**Detail**

| Mode                 | Sampling rate | Bottom-of-panel history |
|----------------------|---------------|-------------------------|
| production (default) | 500 ms        | ~17 min                 |
| fast (`-f`)          | 50 ms         | ~1.7 min                |

Bottom-of-panel depth = `sampling_rate × Σᵢ (band_heightᵢ × cumulative_factorᵢ)` = `sampling_rate × 8 × (1 + 2 + 4 + … + 128)` = `sampling_rate × 2040`.

# A/B Palette Mode

[Up](#sysmon)

A development aid for tuning the metric palette. Defines two compile-time `[Pixel; 6]` constants — `PALETTE_A` (production) and `PALETTE_B` (experimental candidate). When enabled, the active palette flips every 5 seconds of wall-clock and a small `A`/`B` glyph in the bottom-right of the panel says which is currently rendering. By alternating, side-by-side colour comparisons become possible without rebuilding twice.

The mechanism is committed but disabled by default. Three toggle points sit in `main.rs` as commented-out blocks: the palette/label selection, the screen-burn-shift override (usually disabled during comparison so palette positions stay still), and a `synthetic_fraction` injection that gives Disk read pseudo-random activity when the host has no real disk I/O. A glyph helper `draw_label` in `projection.rs` paints the A/B marker in white at the bottom-right corner of the logical canvas pre-rotation.

