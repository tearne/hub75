# sysmon2

[Down](#devices)
[Down](#metric)
[Down](#display)
[Down](#modes)

A fresh take on the HUB75 panel system monitor. Captures host metrics and renders them as a waterfall-with-slowdown on the LED panel. Built from first principles alongside the existing `sysmon` crate.

```
sysmon2
├ Devices
│ ├ CPU
│ ├ RAM
│ ├ Disk
│ └ Network
├ Metric
├ Display
│ ├ Projection
│ │ ├ Row Rendering
│ │ ├ Slate
│ │ └ Time Compression
│ └ Presentation
└ Modes
```

# Devices

[Up](#sysmon2)
[Down](#cpu)
[Down](#ram)
[Down](#disk)
[Down](#network)

The four host resource categories rendered on the panel: CPU, RAM, Disk, Network. Each device contributes one or more **metrics** — a metric is a single vertical column slice of the display. Across the four devices, sysmon2 renders **nine metrics** in total. Every metric is a single value (one fraction in [0, 1] per sample); finer breakdowns become more metrics, not dimensions of one metric.

**See also**

- [Metric](#metric) — defines what a metric is and how each is normalised.

# CPU

[Up](#devices)

Four metrics, one per Pi-5 core. Each metric is one-dimensional: a single "total busy" fraction in [0, 1] of the sample interval, summed from the kernel's user (application work), system (kernel work), and iowait (waiting on disk) categories. The remaining kernel categories (nice, irq, softirq, steal) fold into idle and don't contribute. If finer detail is wanted later — distinguishing user from system, etc. — we'll split those into separate metrics rather than re-introducing per-metric dimensions.

**Detail**

Source: `/proc/stat`. Each `cpuN` line gives cumulative jiffies per category since boot; per-sample fractions come from differencing against the previous reading. Sampled at the master sampling rate. Core count discovered at startup.

**See also**

- [Slate](#slate) — owns the master sampling rate.

# RAM

[Up](#devices)

One metric, one-dimensional: committed memory — i.e. memory in use that isn't reclaimable cache or buffers. A fraction in [0, 1] of total system RAM.

RAM is rendered differently from the other metrics: random-dot density encoding (sysmon1-style) rather than centred bar fill, and **no time compression** — the ring is linear, one entry per panel row sliding straight down at one row per sample. To populate the panel meaningfully under linear time, RAM samples slower than the others (multiplier 10): an entry traverses the panel in 64 × 10 master sample periods of wall-clock.

**Detail**

Source: `/proc/meminfo`. Computed as `(MemTotal − MemAvailable) / MemTotal`. `MemAvailable` is the kernel's estimate of memory available for new processes, accounting for reclaimable cache; subtracting it from total gives the working set (matches `btop` and `free`'s "used" reading).

Render: each sample produces a width-4 pixel row with `n = max(1, round(value × 4))` random columns lit (column choice deterministic per sample by seed). Each lit pixel's intensity is also scaled by `value`, so dot *count* and dot *brightness* both rise with the metric. Row is pushed onto a 66-entry ring (64 visible + 2 edge). At render time, ring entry at index `i` splats at sub-pixel `y = (i − 1) + t`, with `t` the elapsed-fraction since the last RAM sample.

Multiplier 10: in production (master = 1 s) RAM samples every 10 s; in fast mode (50 ms) every 500 ms. Bottom of panel reaches ~10 min back in production.

**See also**

- [Slate](#slate) — owns the master sampling rate and per-metric multipliers; also describes RAM's separate ring type.
- [Time Compression](#time-compression) — applies to all metrics *except* RAM.

# Disk

[Up](#devices)

Two metrics, each one-dimensional: read (bytes/sec) and write (bytes/sec). Each is sampled at the master sampling rate by differencing cumulative byte counters against the previous reading and dividing by the elapsed interval.

Read and write each carry their own scale, log-normalised between a near-zero floor and a per-channel running peak. A read-heavy workload therefore doesn't dampen the visibility of write activity, and vice versa.

**Detail**

Source: per-block-device byte counters from `/proc/diskstats`, summed across whole block devices (loop, ram, dm-* virtuals and partitions are skipped — partitions detected by `/sys/class/block/<name>/partition`).

Log-scale endpoints: floor `MIN_BPS = 1` (1 B/s — anything below normalises to 0; effectively any non-trivial activity registers). Starting upper end `MAX_FLOOR = 10,000` (10 KB/s) per channel; each channel's running peak only ever grows from there, so the scale stretches as activity bursts but doesn't shrink back during quiet periods.

**See also**

- [Metric](#metric) — defines per-metric normalisation.
- [Slate](#slate) — owns the master sampling rate.

# Network

[Up](#devices)

Two metrics, each one-dimensional: down (bytes/sec received) and up (bytes/sec sent). Each is sampled at the master sampling rate by differencing cumulative byte counters against the previous reading and dividing by the elapsed interval.

Down and up each carry their own scale, log-normalised between a near-zero floor and a per-channel running peak — same scheme as Disk.

**Detail**

Source: per-interface byte counters from `/proc/net/dev`, summed across non-loopback interfaces, differenced per sample.

Log-scale endpoints: floor `MIN_BPS = 1` (1 B/s). Starting upper end `MAX_FLOOR = 200,000` (200 KB/s) per channel; each channel's running peak grows from there as bursts arrive.

**See also**

- [Metric](#metric) — defines per-metric normalisation.
- [Slate](#slate) — owns the master sampling rate.

# Metric

[Up](#sysmon2)

A single vertical column slice of the display, fed by one stream from a device. Sampled at the master sampling rate. Each metric carries one value (a fraction in [0, 1]) per sample; sysmon2 keeps metrics one-dimensional by construction — finer-grained breakdowns are modelled as additional metrics rather than as dimensions of one.

Each metric carries its own scale. Every sample is normalised into [0, 1] before it enters the rendering pipeline, so a metric's panel intensity is independent of underlying units. CPU and RAM normalise implicitly (already fractions of an interval, or of total memory). Disk read, Disk write, Network down, and Network up each carry an explicit per-metric scale that floats with the maximum value observed for that metric since the application started.

**See also**

- [Devices](#devices) — the four resource categories that contribute the nine metrics.
- [Slate](#slate) — owns the master sampling rate.

# Display

[Up](#sysmon2)
[Down](#projection)
[Down](#presentation)

A 64×32 HUB75 LED matrix, mounted in portrait — sysmon2 renders into a 32-wide × 64-tall logical canvas, with a single 90° rotation applied at output time to match the panel's native orientation. All rendering work upstream of that final rotation works in the portrait frame.

Driven over a USB-display protocol via the `hub75-client` crate (shared with the existing `sysmon`). The host emits u8 RGB per pixel; brightness and gamma handling live downstream in the firmware, so values written into the canvas reach the panel without further shaping.

**Detail**

Render cadence: configurable per Mode, decoupled from the master sampling rate. Each render interval, Projection runs against the current slate to produce one frame buffer.

Rotation: output packing rotates the 32×64 portrait buffer 90° clockwise into the panel's 64×32 native landscape frame. The user's top-of-view edge maps to the panel's right edge; the user's left edge maps to the panel's top. Concretely, logical pixel `(x, y)` in the portrait buffer is written to native pixel `(63 − y, x)` in the wire frame.

# Projection

[Up](#display)
[Down](#row-rendering)
[Down](#slate)
[Down](#time-compression)

Maps metric data into panel-coordinate space, in two stages around the slate. **Row rendering** runs once per metric sample and writes a fresh pixel row into that metric's ring on the slate. **Vertical mapping** runs once per render frame, reads every ring entry, and splats it at a continuous sub-pixel `y` position with a vertical-Gaussian kernel into the visible canvas. The slate decouples writes from reads: writes happen at metric cadence, reads at render cadence.

# Row Rendering

[Up](#projection)

When a metric is sampled, its current value (a fraction in [0, 1]) is pushed onto its ring on the slate. That's the entire write path — no pixel row is built at this stage. The deterministic conversion from value to pixels happens later, during time compression.

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

- [Slate](#slate) — receives the value.

# Time Compression

[Up](#projection)

Once per render frame, every ring entry is splatted onto the visible canvas at a continuous sub-pixel `y` with a vertical-Gaussian kernel. Each entry slides downward by `1/N(r)` of a row per sample period when in window `r` — newer entries (shallow windows) slide fast, older ones (deep windows) slide slow, producing the slowdown.

For an entry at ring index `a`, in window `r` with position `w = a − WINDOW_STARTS[r]`:

```
y = (r − 1) + (w + t) / WINDOW_SIZES[r]
```

`t` is the elapsed-fraction since the last sample (0→1 between samples; resets as `w` increments, so `y` is continuous through window transitions).

**Bar fill.** For value `v` in a slice of width `W`, the bar is centred at the slice's middle with length `L = v × W`. Per-column intensity is `overlap × v × colour` — both spatially shorter *and* dimmer at low values.

Brightness compensation `α = 0.5 / N(r)^0.65` keeps bottom rows readable; many entries pile up there.

**Detail**

Vertical Gaussian σ ≈ 0.7, kernel half-width 1 panel row.

LED floor: sub-threshold pixels scaled so dominant channel reaches `MIN_LED = 8` (preserves colour ratio).

**See also**

- [Slate](#slate) — provides the rings being splatted.

# Slate

[Up](#projection)

Nine per-metric ring buffers. Eight (CPU × 4, Disk read/write, Net down/up) are the same length and store scalar values; their pixel pattern is computed during time compression. The ninth — RAM — is special: it stores pre-laid pixel rows (random dots) and has length 66 (one per panel row + edges) for its linear-time mapping. The slate isn't one rectangular thing — it's a per-metric assembly.

Fast metrics push new entries often; slow metrics rarely. So a slow metric's ring covers more wall-clock at the bottom of the panel.

> [!IMPORTANT] Time aligns at the top (every newest entry is "now") but diverges down the panel: at the bottom, CPU reaches ~30 s back while Disk reaches several minutes. Slow metrics gain visual weight by scrolling slowly — same shape as `sysmon`.

**Detail**

Master sampling rate: set by [Modes](#modes) (production = 1 s, fast = 50 ms). CPU samples at this rate; slower metrics sample at integer multiples.

Per-metric multipliers: CPU 1, Net 6, Disk 20, RAM 10. Each metric's effective sample period is `master_rate × multiplier`. RAM is slower than its underlying-source allows because its linear-time mapping (one ring entry per panel row) needs samples spread over a panel-traversal-worth of wall-clock to read meaningfully.

Ring length: ~3,340 entries each — the sum of `ceil(1.10^(r-1))` across 66 rows (64 visible + 2 edge). Same length for every metric regardless of sample period.

**See also**

- [Row Rendering](#row-rendering) — writes new entries into the rings.
- [Time Compression](#time-compression) — reads the rings and splats them onto the visible canvas.

# Presentation

[Up](#display)

Applies after each metric has been projected into panel-coordinate space. Two responsibilities:

- **Layout.** Decides which columns each metric occupies on the panel and in what order. CPU cores are interleaved between the other metrics, left to right across the 32-column logical canvas: Disk write, CPU0, Disk read, CPU1, RAM, CPU2, Net down, CPU3, Net up. Each CPU core acts as a visual separator between the non-CPU metrics, and CPU activity is visible across the whole width of the panel rather than clustered.
- **Edge rows.** Maintains hidden rows above the top and below the bottom of the visible canvas so blending operations have well-defined neighbours at the edges. Without these, samples entering or leaving view would cause the topmost and bottommost visible rows to pulsate as their blend partners appeared and disappeared.

**Detail**

Column widths per metric, left to right (CPU cores interleaved between non-CPU metrics):

| Metric     | Cols  | Width |
|------------|-------|-------|
| Disk write | 0–2   | 3     |
| CPU0       | 3–6   | 4     |
| Disk read  | 7–9   | 3     |
| CPU1       | 10–13 | 4     |
| RAM        | 14–17 | 4     |
| CPU2       | 18–21 | 4     |
| Net down   | 22–24 | 3     |
| CPU3       | 25–28 | 4     |
| Net up     | 29–31 | 3     |

CPU cores at width 4 (more visual presence per core); Disk and Net halves at width 3; RAM at width 4. Sums to 32. CPU cores carry more emphasis since on a desktop workload the four cores are the most active and most differentiated metrics.

Edge rows: 1 above and 1 below the visible 64 rows (matches `sysmon`). They are conceptual targets in the splat coordinate space — entries with `y` outside `[0, 64)` still get computed but their Gaussian contributions to visible rows fall off naturally, so content sliding into and out of view doesn't pulsate.

# Modes

[Up](#sysmon2)

sysmon2 runs in one of two modes selected at startup. The pipeline is identical between them — same data sources, same rendering, same ring length — only the cadence differs.

- **Production (default)** — slow: master sampling rate 1 s, render 5 fps. Top of panel moves a row every second; bottom of CPU column reaches ~20 min back. Roughly a quarter of fast's CPU because per-frame work scales with render rate, not sampling rate. Top motion stays smooth because 5 frames span one row of motion.

- **Fast** — for development: master sampling rate 50 ms, render 20 fps. Top moves a row every 50 ms; bottom of CPU column reaches ~30 s back. Quick visual feedback when tuning or chasing artefacts. Uses the most CPU.

Selected via CLI flag `-f` for fast; absence of the flag selects production. Production is the default so the systemd-installed binary runs at low CPU without further flags.

**Detail**

| Mode | Master sampling rate | Render rate | Render interval | Bottom-of-CPU history |
|------|-------------|-------------|------------------|-----------------------|
| production (default) | 1 s | 5 fps  | 200 ms | ~20 min |
| fast (`-f`)          | 50 ms | 20 fps | 50 ms | ~30 s   |

Why this works: the ring length is fixed by the window scheme and stays the same in both modes. Slowing the master sampling rate stretches each ring entry's wall-clock coverage; slowing the render rate drops per-second rendering work. Sub-pixel sliding stays smooth because the elapsed-fraction `t` advances the same 0→1 over one sample period regardless of rate, so each render frame interpolates to a sensible intermediate position.
