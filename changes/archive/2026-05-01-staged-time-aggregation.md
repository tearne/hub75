# Staged time aggregation in the falling stream

## Intent

Make the dots appear to **slow down as they fall**. The top portion of each strip stays at the current full-resolution time scale (one row per sample). Lower portions compress more time per row by aggregating samples — so the further down a dot has fallen, the more seconds of activity that row represents and the slower it slides.

The intent is twofold: cover much more time history than the current ~64 seconds in the same physical strip, and visually convey "now" vs "a while ago" through the slide speed itself rather than only through position.

A staged version is the simplest realisation — split the strip into bands (say three thirds of its height) where each successive band aggregates samples by an integer factor (e.g. 1×, 3×, 9×). How many stages, what aggregation factor, and what aggregation method (mean / max / latest) are open design choices to settle in the Approach.

This is an experimental change — willingness to throw it away if it doesn't read well visually is part of the framing.

## Approach

### Three equal stages with geometric aggregation

Split each strip's 64-row height into three equal bands of 21 rows each (with one row left over — discussed below). Aggregation factors 1× / 3× / 9× — geometric so each band shows 3× the time depth of the one above.

### Per-stage buffers, cascading aggregation

Each metric grows from one buffer to three: `stage1[H1]`, `stage2[H2]`, `stage3[H3]`. When a sample arrives it pushes to the front of `stage1`. When `stage1` overflows, the oldest 3 samples (`F2 = 3`) are popped, averaged, and the result pushes to the front of `stage2`. Same cascade from `stage2` to `stage3` with `F3/F2 = 3`. Total samples retained per metric: `H1 + F2·H2 + F3·H3 = 21 + 63 + 189 = 273` per metric. Trivial memory.

### Mean aggregation

Aggregation is the mean of the F samples being collapsed. For CPU: per-component mean of `(user, system, iowait)` separately across the three samples. For RAM: mean of each composition counter. For IO: mean of `a_bps`, `b_bps`. Each aggregated sample also gets a fresh `seed` so the dot pattern looks new (rather than re-shuffling identically to the source samples).

### Slide rate per stage

Stage 1 keeps the existing smooth sub-pixel slide (one row per metric sample period; sub-pixel during the period). Stages 2 and 3 advance **stepwise** — when their internal counter (samples-since-last-stage-tick) reaches the aggregation threshold, the entire stage shifts down by one row. No sub-pixel slide between steps in lower stages. The discrete advancement is part of the desired "slowing down" reading.

### Visible boundary handling

Hard boundaries — no smoothing across stage 1 → stage 2 → stage 3 transitions. The transition reads visually as "suddenly older" which fits the framing. The 64-row canvas with three 21-row bands leaves 1 row spare; allocate it as a black gap row at one of the boundaries (say between stage 2 and stage 3) to make the boundary deliberate. Or leave the boundary unmarked; either reads.

### Apply to all four metrics

Same staging applies to CPU, RAM, Disk, Net. Because each metric's sample period differs, the absolute time depth of stage 3 differs per metric (CPU bottom band ≈ 3.3 min, RAM/Net ≈ 20 min, Disk ≈ 66 min). That's intentional — slow-changing metrics benefit from extra-deep history.

### Sampler stays as-is

The sampler thread continues to push at the existing per-metric cadence. Cascade logic lives in `push_history` (or its replacement): pushing a stage-1 sample triggers the overflow check, which can in turn trigger stage-2 push, which can trigger stage-3 push.

## Plan

Version bump: `hub75-client` patch — **0.2.5 → 0.2.6**.

Heights `[22, 21, 21]`, factors `[1, 3, 9]`, no gap rows, stages 2 and 3 stepped (no sub-pixel slide).

- [x] Bump `usb-display/client/rust/Cargo.toml` to `0.2.6`
- [x] Add stage constants: `STAGE_HEIGHTS: [usize; 3] = [22, 21, 21]`, `STAGE_FACTORS: [u32; 3] = [1, 3, 9]`, derived `STAGE_TOPS: [usize; 3] = [0, 22, 43]`
- [x] Add `mean(samples: &[T]) -> T` (via `Aggregable` trait) for `CpuSample`, `RamSample`, `IoSample`
- [x] Replace each metric's single `VecDeque` in `Snapshot` with a `MetricStages<T>` struct holding three deques (`stage1`, `stage2`, `stage3`) plus two small accumulator `Vec`s for the cascade between stages
- [x] Rewrite `push_history` as `push_cascading`: push to stage1.front; if stage1 over capacity, pop oldest into stage2's accumulator; when accumulator hits `F2 = 3`, mean it and push to stage2; same cascade from stage2 → stage3 with `F3/F2 = 3`
- [x] Stage 1 keeps the existing smooth-slide rendering (sample i at y = i + t − 1, with one imaginary row above and below). Stages 2 and 3 render stepwise: sample i at integer y = `STAGE_TOPS[k] + i`, no sub-pixel offset
- [x] Wire up rendering for all four strips via shared `for_each_staged_sample` helper plus per-metric per-sample render functions
- [x] `cargo build --example sysmon_linux --features panel-64x32` succeeds; user runs and visually confirms the slow-down effect, the stepwise motion in lower stages, and that stage transitions don't introduce render glitches

## Log

- After first run: stepped motion in stages 2 and 3 felt like the dots "disappeared most of the time" — long static gaps between cascade events broke the continuous flow established by stage 1's smooth slide. Tried reverting the stepped-motion design choice and gave all three stages smooth sub-pixel slides at their respective rates (1×, 1/3×, 1/9× of the metric's sample period).
- This required adding per-stage `_at` timestamps to `MetricStages` (touched only on each stage's push), one imaginary row above and one below each stage's visible band (buffers are H+2 for all three), and a per-stage `t` computation in `for_each_staged_sample`. Snapshot's outer `_at` fields became redundant and were removed.
- User decided the change as a whole did not work and asked to abort. Reverted all code changes back to the post–two-axis-dot-blur state (`hub75-client 0.2.5`).

## Feedback

**Status:** not implemented.

**Notes:** The staged time-aggregation idea didn't read well visually. Both implementations attempted (stepped lower stages, then smooth-but-slower lower stages) felt disconnected from stage 1's flow rather than conveying "slow down as they fall". The aggregation method (mean) also dilutes brief activity bursts, which compounded the perception that the lower stages were empty or disappearing.

If revisited later, worth exploring:
- Aggregation with `max` instead of mean — preserves activity peaks instead of smoothing them away.
- Continuous compression rather than hard stages — e.g. an exponential time-axis where each successive row covers slightly more time than the one above. Avoids visible boundaries entirely.
- Or a different visual metaphor entirely for "deeper history" — e.g. dim older samples but keep the time axis linear.

**Documentation impact:** none — no project docs were touched in this change since the code never shipped.

