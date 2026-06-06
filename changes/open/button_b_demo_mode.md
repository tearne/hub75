# Button B demo mode

**Mode:** Formal

## Intent

Button B on the panel is currently unused. Pressing it should kick off a 30-second demo that drives the visualisation with synthetic, broadly realistic metric activity instead of the real host samplers — varying CPU load across individual cores, network up and down, disk read and write, and RAM rising and falling. The point is to show the panel doing something visibly interesting on demand (for demos, photos, or sanity-checking the visuals) without having to actually load the host. The demo runs for 30 seconds then hands back to the live samplers; pressing B again during the demo is a reasonable way to restart or cancel it (to be settled in Approach).

The activity should look *plausible* rather than uniformly noisy — cores ramping at different rates, network and disk bursts, RAM drifting — so it reads as "the host got busy", not as a test pattern.

## Approach

### Inject at the `push_sample` boundary

All four samplers feed [Bands](sysmon/map.md#bands) as fractions in `[0, 1]`. When demo mode is active, `sample_all` pushes synthetic fractions at the same boundary; Bands, Projection, and rendering see no difference.

### Keep real samplers ticking, discard their output

CPU and throughput samplers carry differencing state. Skipping `.sample()` for 30 s would make the first post-demo reading one giant delta. Keep calling, throw away.

### `peak_busy` follows synthetic CPU during demo

Otherwise the adaptive Hz curve idles while the visuals simulate a hot host.

### Per-metric character

A new `demo` module produces the nine fractions per cycle:

- **CPU cores** — independent low-passed walks, different time constants, occasional spikes.
- **RAM** — slow drift in a plausible band.
- **Disk / Net** — bursty: near-zero baseline, Bernoulli-triggered bursts, decay.

Uniform noise reads as a test pattern; per-metric character makes it read as a busy host.

### Activation, cancellation, duration

A B-press while inactive starts demo mode (fresh `DemoEngine`, start time recorded). A B-press during demo cancels and returns to live immediately. 30 s is a `const` in `main.rs` alongside the other tuning constants. Console prints on entry and exit.

### PRNG

`rand::thread_rng`, unseeded — fresh variety on each press. `rand` becomes a new dependency.

### Demo indicator

A single white pixel in the bottom-left corner of the logical canvas while demo is active, painted after the projection pass (same hook as the A/B label glyph). Makes it unambiguous on photos/video that the activity is synthetic.

### Map impact

[Modes](sysmon/map.md#modes) gains a demo entry. Per-node catch-up after Build.

## Plan

- [x] Add `rand` to `sysmon/Cargo.toml`.
- [x] New `demo` module with `DemoEngine`: per-cycle nine fractions (CPU per-core walks, RAM drift, Disk/Net bursts) plus a synthetic `peak_busy`.
- [x] Wire Button B in `main.rs` to toggle demo state and auto-end at the 30 s const.
- [x] In `sample_all`, when demo is active, call real samplers and discard, push `DemoEngine` fractions, drive `peak_busy` from the synthetic CPU.
- [x] Paint a white pixel in the bottom-left of the logical canvas while demo is active.
- [x] Build-verify sysmon.

## Log

- Build clean on the first try (`cargo build` from `sysmon/`).
- First hardware impression: too "mid-level uniform", not bursty enough. Adjusted demo synthesis in 0.8.1: CPU walks now idle near zero with occasional spike retargets to [0.7, 1.0] (spike_prob 20%), rather than uniformly retargeting across [0, 1]. Disk/Net burst frequencies cut roughly in half and decay taus tightened, so the baseline is quiet and the bursts stand out. RAM kept as slow drift (matches its real character).
- 0.8.2: per-core spike probability raised 0.2 → 0.45, spike range tightened to [0.85, 1.0]. Cores now hit near-100% noticeably more often.
- 0.8.3: still too uniform — cores spent too much time *sweeping through* the middle. Snappier taus (0.15–0.5 s, was 0.4–2.0 s) and slower retarget rates (0.2–0.35/s, was 1/s) so each core dwells at idle or spike and only briefly transits the middle.
- 0.8.4: demo duration 30 s → 60 s.
- 0.8.5: added a slow `intensity` modulator (`Walk`, tau 2.5 s, retargets ~6 s) that scales every core's `spike_prob`. When intensity dips, all four cores tend to idle together → `peak_busy` falls → adaptive frame rate slows. When it rises, cores spike often → rate ramps up. Gives the panel a visible speed-up / slow-down rhythm across the demo.
- 0.8.6: demo duration 60 s → 600 s (10 min).

## Conclusion

Shipped at **0.8.6** (minor 0.7.9 → 0.8.0 confirmed at entry; six patch iterations during hardware testing). Build itself was straightforward; the bulk of the work was tuning the synthesis to *read* as a busy host rather than uniform noise — captured in the Log as 0.8.1→0.8.6.

Key tuning outcomes worth noting beyond the Log:

- The synthesis ended up in two layers: a slow `intensity` walk gates per-core `spike_prob`, so cores idle and spike *together* rather than independently. This gives the panel a visible speed-up / slow-down rhythm (via the adaptive frame rate) that single-layer noise didn't.
- CPU walks settled at very snappy taus (0.15–0.5 s) plus slow retargets (~3–5 s dwell), so cores sit at idle or near-max and only briefly transit the middle.
- Demo duration grew from the 30 s in the Intent to 60 s and finally 10 min during testing — closer to a screensaver than a "press for a short showpiece".

**Documentation impact** — [Modes](sysmon/map.md#modes) currently lists only production and fast. A demo entry (button-triggered, time-bounded, synthetic source) needs catching up via the per-node negotiation rule. Not started yet.


