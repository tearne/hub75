# Sysmon metric fidelity

**Mode:** Explore

## Intent

Two visual symptoms suggest the rendered metrics may not accurately reflect what's happening on the host:

- **Network down and up bands look identical** — same magnitude, same dot arrangement. Real rx and tx traffic shouldn't track each other that closely.
- **CPU cores often look very similar to each other** — the four core columns frequently show near-identical dot patterns, where some per-core variation would be expected.

Open question whether the per-cell randomness used in rendering is genuinely random, or whether something upstream (sampling, normalisation, the dithering/placement step) is producing correlated outputs across metrics.

Goals: find out which of (a) sampling is wrong, (b) normalisation collapses signals together, or (c) the dot-placement randomness is correlated/seeded identically across metrics — and report findings, with fixes scoped separately.

## Approach

### Primary suspect: shared seed across metrics

`bands.rs:109` derives the column-choice seed as `mix32(commit_count, band_idx)`. Metric identity is not in the seed. Every metric is pushed in lockstep each cycle, so all nine metrics share the same `commit_count` value and therefore the same seed for a given `band_idx`. When two metrics produce the same `target_n` and the same `basis` pattern, `flow_pattern` will pick identical columns — *by construction*. This is the leading hypothesis for "identical dot arrangement" across both Net rx/tx and across CPU cores.

The effect is amplified for narrow metrics: Net halves are 3 columns wide, so only a handful of pattern bits exist; with the seed shared, coincidence is the rule rather than the exception.

### Net sampling: spot-check, not a code dive

`read_net_bytes` reads field 0 (rx_bytes) and field 8 (tx_bytes) from `/proc/net/dev`, sums across non-loopback interfaces. The code looks right; verify by tailing `/proc/net/dev` against expected traffic and confirming rx and tx counters move independently with deliberate one-direction load (e.g. a download vs. an upload).

### Net normalisation: explains "same quantity"

`ThroughputSampler` tracks `max_a_bps` and `max_b_bps` independently and log-scales each direction against its own monotonically-growing peak. On a host that has at some point seen comparable rx and tx peaks, both scales settle similarly, and modest current activity in either direction produces similar fractions. This is design-as-intended — but it's worth confirming that the user's "identical quantity" observation is the running peaks coincidentally aligning, not a sampling defect.

### Pattern derivation at low target_n

Band 0 has `factor = 1`, so its accumulator holds one entry whose `pattern` is always 0 (the raw upstream row carries no pattern). That makes `freq = [0; 8]` for band 0's commit, which collapses `flow_pattern`'s sort to pure seed-based tiebreak on column index. Combined with the shared seed above, this is exactly where pattern coincidence is strongest.

### Topics

- Confirm seed-shared-across-metrics is the cause of identical dot arrangement (read `bands.rs:101–112` and `slate.rs`; reason about what varies between two metric instances at the same cycle).
- Verify Net rx/tx are sampled independently and accurately (manual `/proc/net/dev` comparison under directed load).
- Characterise Net normalisation: compare the running peaks for rx and tx on this host; confirm whether quantity convergence is normalisation artefact, not sampling.
- Identify whether other metrics (CPU cores, Disk read/write) hit the same shared-seed coincidence and to what degree.

### Net spot-check before synthetic load

Start by reading the sampling path with the four topics in mind; only fall back to driving a synthetic load test if reading isn't conclusive.

### Fixes ride along

Findings and fixes belong in the same change rather than splitting Explore findings from a follow-up Formal fix change. The Plan below covers both.

## Plan

### Topics

- Confirm seed sharing causes identical dot arrangement: trace what varies between two metric instances at the same cycle through `MetricBands::push_sample` and `flow_pattern`. Add a stable per-metric identity (passed in at `MetricBands::new`) and mix it into the seed.

- Spot-check Net sampling: re-read `read_net_bytes` against `/proc/net/dev` field layout; confirm rx and tx counters are read independently. If anything is unclear from reading, generate directed one-way traffic and observe.

- Characterise Net per-channel normalisation: read `ThroughputSampler` to confirm `max_a_bps` and `max_b_bps` are independent; reason about whether running-peak convergence is the cause of "same quantity" appearance. Decide whether to leave it (design-as-intended) or change it.

- Sweep the other metrics (CPU cores, Disk read/write, RAM) for the same shared-seed coincidence; the metric-identity fix above should cover them, but verify nothing else in the rendering path correlates them.

### Done when

- Each topic has a written conclusion in the **Log** with cause confirmed or ruled out.
- The seed-sharing fix (per-metric identity in the mix) is implemented and the visual symptoms (Net rx/tx and CPU cores looking identical) are gone or noticeably reduced.
- Any further fix decided during the topics is implemented or explicitly deferred with a note.

## Conclusion

Two real defects, one design tweak. The dot-placement seed has been per-metric since this change — `MetricBands` now carries a `metric_id: u32` mixed in as `mix32(mix32(metric_id, commit_count), band_idx)`, with distinct ids assigned in `Slate::new`. Net rx and tx now share a single monotonic peak (`PeakMode::Shared` on `ThroughputSampler`); Disk keeps independent peaks. Net log floor raised from 1 B/s to 10 KB/s so a dominant direction stands above ACK return traffic at width 3.

Net sampling itself was correct (no bug); the apparent fidelity issue was entirely in the rendering and normalisation layers.

Final version: sysmon 0.6.8 (patch-level bump confirmed; scope stayed within Plan Topic 3's "decide whether to change it" hook).

**Documentation impact:** `sysmon/map.md` Network node is stale on two points — it says rx/tx normalise per-channel "same scheme as Disk" (no longer true: shared peak), and it gives the floor as 1 B/s (now 10 KB/s). To catch up per-node when the user is ready.

## Log

- **Topic 1 — seed sharing, confirmed and fixed.** `MetricBands::push_sample` built the dot-placement seed from `(commit_count, band_idx)` only. With nine metrics pushed in lockstep each cycle, every metric saw the same `commit_count` and therefore the same seed for a given band. Whenever two metrics arrived at the same `target_n` and `basis`, `flow_pattern` picked identical columns by construction. Fixed by adding a stable `metric_id: u32` field to `MetricBands`, mixed into the seed as `mix32(mix32(metric_id, commit_count), band_idx)`. Distinct ids assigned in `Slate::new` (CPU 0x01–0x04, RAM 0x10, Disk r/w 0x20/0x21, Net down/up 0x30/0x31).

- **Topic 2 — Net sampling, no bug found.** `read_net_bytes` parses `/proc/net/dev`: per non-loopback interface, splits on `:`, parses the right side as whitespace-separated u64s. The format is 8 rx fields then 8 tx fields, so `nums[0]` is rx_bytes and `nums[8]` is tx_bytes — `nums.first()` and `nums.get(8)` are correct. Counters are independent per direction. No synthetic load run needed.

- **Topic 3 — Net normalisation, design-as-intended.** `ThroughputSampler` keeps `max_a_bps` and `max_b_bps` independently and log-scales each direction against its own monotonic peak. On a host where rx and tx have at some point seen comparable peaks (typical for desktop / small-server traffic), modest activity in either direction produces similar fractions. This explains the "same quantity" observation without it being a bug. Left unchanged.

- **Topic 4 — Sweep of other metrics.** The CPU cores, Disk r/w and RAM all went through the same shared-seed code path, so the metric-identity fix lifts all of them simultaneously. Nothing else in the rendering path correlates metrics: each `MetricBands` has its own band/accumulator state, and the Renderer composes columns independently.

- Bumped sysmon to 0.6.6. Release build clean.
- **Topic 3, follow-up:** user opted to switch Net to a shared peak so rx/tx bands are directly comparable. Added `PeakMode { Independent, Shared }` to `ThroughputSampler`; `net_sampler()` uses `Shared`, `disk_sampler()` keeps `Independent`. In `Shared`, both `max_a_bps` and `max_b_bps` track a single monotonic peak across both channels and current bps. **Map impact:** `sysmon/map.md` Network node says "Down and up each carry their own scale … same scheme as Disk" — now stale; needs catch-up to describe shared-peak normalisation for Network and the divergence from Disk. Bumped to 0.6.7.
- **Topic 3, follow-up 2 — log-floor raise.** With width-3 columns and a 1 B/s log floor, ACK return-traffic during a download (~3% of payload) was rendering at fraction ≈0.8, only one dot below rx. Raised `NET_LOG_MIN_BPS` from `1` to `10_000` (10 KB/s). Idle sub-floor traffic now reads as zero, but a dominant direction stands clearly above ACK return traffic without overpowering. Map impact (already flagged) extends to this: the Network node also needs the new floor reflected when caught up. Bumped to 0.6.8.
