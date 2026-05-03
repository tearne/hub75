# Remaining metrics

## Intent

Extend sysmon2 from the CPU vertical slice to the full nine-metric pipeline by adding RAM, Disk read, Disk write, Network down, and Network up. The architecture set during the sysmon2 build session is parametric; this change is concrete fill-in. Per-metric sample-rate multipliers (RAM/Net every 6 master samples, Disk every 20) come along, plus log-normalisation against running peaks for the unbounded throughput metrics.

Cadence remains agile.

## Conclusion

What shipped:

- **RAM** sampler at `src/ram.rs` — `(MemTotal − MemAvailable) / MemTotal`, matches `btop`'s "used" reading.
- **Disk and Net** share a generic `ThroughputSampler` at `src/throughput.rs` that tracks per-channel running peaks and log-normalises bytes/sec into [0, 1] between a 1 KB/s floor and the running max. Disk reads `/proc/diskstats` (real block devices only; loop/ram/dm-* skipped). Net reads `/proc/net/dev` (non-loopback interfaces).
- **Slate** extended to nine rings (CPU × 4, RAM, Disk read/write, Net down/up) plus per-device `last_*_sample_at` timestamps. Sample multipliers (`CPU_MULTIPLIER` 1, `RAM_MULTIPLIER` 6, `NET_MULTIPLIER` 6, `DISK_MULTIPLIER` 20) live here.
- **Projection** updated: `render_canvas` now takes the master sampling rate and current frame `Instant`, computes a per-metric elapsed-fraction `t` against each metric's own period, and splats every metric onto its column slice.
- **Layout** rebalanced for visual evenness — RAM dropped from width 8 to width 4. All five non-CPU metrics now at width 4; CPU cores at width 3. Sums to 32 with no gaps; all metrics within ±1 column of each other.
- **Bar fill style** briefly switched to uniform-intensity-across-slice while exploring; reverted to length-varying (`coverage = clamp(L − c, 0, 1)` per column), since the variable widths plus length-varying bar give RAM-at-15% a thin partial column rather than a full bright slice (avoiding the original "RAM too bright" issue without needing a per-metric gamma).

Not changed:
- Window scheme, ring length, splat parameters, LED floor — all carried over from the bar-rendering change.
- Modes — production default, `-f` for fast.

No version bump; sysmon2 still v0.1.0.
