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
│ │ └ Vertical Mapping
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

**Detail**

Source: `/proc/meminfo`. Computed as `(MemTotal − MemAvailable) / MemTotal`. `MemAvailable` is the kernel's estimate of memory available for new processes, accounting for reclaimable cache; subtracting it from total gives the working set (matches `btop` and `free`'s "used" reading). Sampled at the master sampling rate.

**See also**

- [Slate](#slate) — owns the master sampling rate.

# Disk

[Up](#devices)

Two metrics, each one-dimensional: read (bytes/sec) and write (bytes/sec). Each is sampled at the master sampling rate by differencing cumulative byte counters against the previous reading and dividing by the elapsed interval.

Read and write each carry their own scale, independently normalised against the largest value observed for that metric since the application started. A read-heavy workload therefore doesn't dampen the visibility of write activity, and vice versa.

**Detail**

Source: per-block-device byte counters from `/proc/diskstats`, summed across devices, differenced per sample.

**See also**

- [Metric](#metric) — defines per-metric normalisation.
- [Slate](#slate) — owns the master sampling rate.

# Network

[Up](#devices)

Two metrics, each one-dimensional: down (bytes/sec received) and up (bytes/sec sent). Each is sampled at the master sampling rate by differencing cumulative byte counters against the previous reading and dividing by the elapsed interval.

Down and up each carry their own scale, independently normalised against the largest value observed for that metric since the application started.

**Detail**

Source: per-interface byte counters from `/proc/net/dev`, summed across non-loopback interfaces, differenced per sample.

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
[Down](#vertical-mapping)

Maps metric data into panel-coordinate space, in two stages around the slate. **Row rendering** runs once per metric sample and writes a fresh pixel row into that metric's ring on the slate. **Vertical mapping** runs once per render frame, reads every ring entry, and splats it at a continuous sub-pixel `y` position with a vertical-Gaussian kernel into the visible canvas. The slate decouples writes from reads: writes happen at metric cadence, reads at render cadence.

# Row Rendering

[Up](#projection)

When a metric is sampled, its current value is rendered as a horizontal pixel row of width W matching its column slice. Pixels are binary on/off; the metric's value is encoded by *how many* pixels light, not by their brightness. Lit pixels sit at random columns within the slice (deterministic per sample, seeded by the sample number, so the same sample renders the same pattern across frames). All lit pixels of a metric share that metric's colour. The rendered row is pushed onto that metric's ring on the slate.

**Detail**

Pixel count from a fraction `v`: `n = 0` if `v = 0`, otherwise `n = max(1, round(v × W))`. The `max(1)` lower threshold ensures any non-zero value lights at least one pixel rather than rounding to zero. Transition boundaries between counts fall at `(k + 0.5) / W` for `k = 1..W−1`.

Per-metric colours (carried from sysmon's tuned spectrum):

| Metric     | Colour (RGB)       | Hue          |
|------------|--------------------|--------------|
| Disk write | `[240,  50,  50]`  | red          |
| Disk read  | `[255, 175,  40]`  | amber-gold   |
| CPU 0–3    | `[ 50, 230, 230]`  | cyan         |
| RAM        | `[230,  60, 170]`  | magenta      |
| Net down   | `[ 40, 220,  90]`  | green        |
| Net up     | `[200, 240,  40]`  | yellow-lime  |

The CPU colour is shared across all four cores; visual differentiation between cores comes from their independent per-sample random patterns, not from colour. Going around the wheel: red → amber → cyan → magenta → green → lime.

**See also**

- [Slate](#slate) — receives the rendered row.

# Vertical Mapping

[Up](#projection)

Once per render frame, every ring entry is splatted onto the visible canvas at a continuous sub-pixel `y` position with a vertical-Gaussian kernel. The continuous `y` is what makes the slowdown read smoothly: between samples, every entry slides downward by a fraction of a row whose size depends on its window depth. There is no separate aggregation or smoothing step — the splatting itself produces both the slowdown and the inter-row blending.

The slate's window scheme tells us each entry's `y`. An entry at age `a` (its index in the ring) lies in window `r` where `WINDOW_STARTS[r] ≤ a < WINDOW_STARTS[r] + WINDOW_SIZES[r]`. Its position within that window is `w = a − WINDOW_STARTS[r]`. The continuous panel-row position is:

```
y = (r − 1) + (w + t) / WINDOW_SIZES[r]
```

where `t ∈ [0, 1]` is the elapsed-fraction since the metric's last sample. Between samples `t` advances 0→1, so `y` advances by `1/N(r)` of a row over one sample period — fast in shallow windows (top of panel), slow in deep windows (bottom). When the next sample arrives, `w` increments by 1 and `t` resets to 0, so `(w + t)` is continuous through the transition: no jump.

> [!IMPORTANT] The window scheme survives but means a different thing now. It no longer aggregates entries — it tells us where on the panel each entry sits, with `WINDOW_SIZES[r]` controlling how much an entry slides per sample period while it's in window `r`.

Each entry's pre-laid pixel row is splatted at its `y` with a vertical Gaussian: centre row at full weight, ±1 panel rows at ~36%. Splats from many entries that land near the same panel rows accumulate additively. Per-entry brightness is scaled by `α = scale / N(r)^exponent` to keep the panel readable: bottom rows have many entries piling up over a small `y` range, so each entry's contribution must be dimmer than at the top where one entry per row stands alone.

**Detail**

Vertical Gaussian σ ≈ 0.7, kernel half-width 1 panel row. Centre weight 1.0; ±1 row weight ≈ 0.36.

Brightness compensation: `α = 0.5 / N(r)^0.65`. The `0.5` is a global headroom factor; the `0.65` exponent sits between mean-uniform (`1/N`) and variance-uniform (`1/√N`) — sysmon1's last-tuned perceptual compromise.

**See also**

- [Slate](#slate) — provides the rings being splatted.

# Slate

[Up](#projection)

Nine per-metric ring buffers, all the same length. Each entry is a pre-laid pixel row matching that metric's column slice (width 3 for CPU cores and Disk/Net halves, 8 for RAM). The slate isn't one rectangular thing — it's a per-metric assembly.

Fast metrics push new entries often; slow metrics rarely. So a slow metric's ring covers more wall-clock at the bottom of the panel.

> [!IMPORTANT] Time aligns at the top (every newest entry is "now") but diverges down the panel: at the bottom, CPU reaches ~30 s back while Disk reaches several minutes. Slow metrics gain visual weight by scrolling slowly — same shape as `sysmon`.

**Detail**

Master sampling rate: set by [Modes](#modes) (production = 1 s, fast = 50 ms). CPU samples at this rate; slower metrics sample at integer multiples.

Per-metric multipliers (carried from `sysmon`): CPU 1, RAM 6, Net 6, Disk 20. Each metric's effective sample period is `master_rate × multiplier`.

Ring length: ~1,185 entries each — the sum of `ceil(1.07^(r-1))` across 66 rows (64 visible + 2 edge). Same length for every metric regardless of sample period.

**See also**

- [Row Rendering](#row-rendering) — writes new entries into the rings.
- [Vertical Mapping](#vertical-mapping) — reads the rings and splats them onto the visible canvas.

# Presentation

[Up](#display)

Applies after each metric has been projected into panel-coordinate space. Two responsibilities:

- **Layout.** Decides which columns each metric occupies on the panel and in what order. Starting layout, left to right across the 32-column logical canvas: Disk write, Disk read, CPU0, CPU1, CPU2, CPU3, RAM, Network down, Network up.
- **Edge rows.** Maintains hidden rows above the top and below the bottom of the visible canvas so blending operations have well-defined neighbours at the edges. Without these, samples entering or leaving view would cause the topmost and bottommost visible rows to pulsate as their blend partners appeared and disappeared.

**Detail**

Column widths per metric, left to right (matches the existing `sysmon` layout):

| Metric     | Cols  | Width |
|------------|-------|-------|
| Disk write | 0–2   | 3     |
| Disk read  | 3–5   | 3     |
| CPU0       | 6–8   | 3     |
| CPU1       | 9–11  | 3     |
| CPU2       | 12–14 | 3     |
| CPU3       | 15–17 | 3     |
| RAM        | 18–25 | 8     |
| Net down   | 26–28 | 3     |
| Net up     | 29–31 | 3     |

Eight metrics at width 3 plus RAM at 8 sum to the 32-column canvas. RAM's wider slot is inherited from `sysmon`; it absorbed the slack the uniform-3 layout would otherwise leave.

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
