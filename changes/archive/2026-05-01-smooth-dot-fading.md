# Smooth dot fading on the sysmon panel

## Intent

Replace the current snap-on-tick rendering of `sysmon_linux` with smooth motion: between sample ticks, the dot field should slide and fade gradually rather than jump.

Render rate is decoupled from sample rate. Sampling stays per-metric (CPU every tick, RAM/Net every 6 ticks, Disk every 20). Renders happen continuously at a higher fixed rate (10 fps), with each render reading the snapshot and the elapsed time since the last sample for each metric, then drawing the dot field at fractional column positions.

Each dot is rendered with a horizontal blur (roughly 2 pixels wide) — both because the motion is horizontal and because blurring lets fading transitions combine cleanly without the visual "wiggle" that single-pixel dots produce when an adjacent pair fades to a singleton. Where multiple blurred dots overlap, the brightness should add naturally so the whole display reads as a soft, continuously-moving field rather than discrete pixels.

## Approach

### Render and sampler split into separate threads

The sampler stays as it is — ticks at `--update`, pushes per-metric samples on their phased schedules. A new render thread runs at a fixed `RENDER_FPS = 10` independent of `--update`. Render never modifies the snapshot; sampler never touches the canvas. Lock the snapshot mutex once per render to clone, then unlock and draw.

### `--update` keeps current meaning

`--update` controls the *sample* tick interval (and therefore per-metric scroll cadence). Render rate is separate, set by a `RENDER_FPS` constant for now. Existing user-facing behaviour for `-u <ms>` is unchanged.

### Per-metric "last sampled at" timestamps in the snapshot

The snapshot grows fields recording when each metric was last pushed (`Instant`). The renderer subtracts current time and divides by the metric's period (in seconds) to get `t ∈ [0, 1)` — fractional progress to the next sample. CPU's `t` is meaningful between consecutive renders even though it samples every tick, because render rate exceeds sample rate.

### Sub-pixel column positions

Buffer index `i` is rendered at logical column `x = i - t`. Columns slide leftwards continuously over the period; when a new sample is pushed, indices shift by one and the freshly-pushed sample appears at the rightmost slot, so visually no sample jumps — each one moves smoothly off the left and the new one enters smoothly from the right.

### Horizontal Gaussian blur per dot

Each lit dot at sub-pixel column `x` is splatted into the canvas via a 1-D Gaussian kernel (vertical extent: just the row; horizontal extent: ±2 pixels around `x`). Sigma ≈ 0.7 gives an effective ~2 px width, enough to soften single-vs-pair wiggle without smearing detail.

### Additive saturating blend in u16, then clamp to u8

The canvas accumulator becomes `[[u16; 3]; LW * LH]`. Each Gaussian splat adds `colour_channel × weight` to the accumulator. After all dots are drawn, clamp each channel to `255` and pack into `[u8; 3]` for the wire frame. Additive over linear-RGB looks good on LED panels and makes overlapping blurred dots brighten naturally without alpha-blending complexity.

### Newest sample fades in to avoid edge pop-in

When a new sample arrives, it appears at logical x = LW-1 (rightmost). To stop it popping in, the renderer ramps its overall intensity from 0 to 1 over the first `FADE_IN_FRACTION` of its period (proposed 0.3). This applies only to the current rightmost-slot sample; older samples render at full intensity throughout.

### Leftmost samples cull when fully off-screen

A sample at `x < 0` is considered off-screen and skipped in the inner loop. Some Gaussian tail bleed may bleed through the leftmost canvas pixel briefly during the slide-out — accepted as a small artefact in exchange for a tighter inner loop.

## Plan



Version bump: `hub75-client` patch — **0.2.2 → 0.2.3**. Public library API unchanged; only the example.

- [x] Bump `usb-display/client/rust/Cargo.toml` to `0.2.3`
- [x] Add `RENDER_FPS: u64 = 10` constant; render loop sleeps `1000 / RENDER_FPS` ms between frames instead of `interval`
- [x] Add per-metric `last_at: Instant` fields to `Snapshot` (one per CPU / RAM / Net / Disk); default to `Instant::now()` at sampler start
- [x] Sampler updates the matching `last_at` whenever it pushes a sample for that metric
- [x] Switch canvas accumulator type from `[[u8; 3]; LW * LH]` to `[[u16; 3]; LW * LH]`; saturate-add into `u16` then clamp to `u8` when packing the wire frame
- [x] Add `splat_gaussian(canvas, x_subpix, row, colour, alpha)` helper — splats a 1-D Gaussian (sigma 0.7, ±2 px horizontal extent) at fractional `x_subpix` into the given row, scaled by `alpha`
- [x] Pass `interval` through to render so per-metric `period_secs = period_ticks * interval.as_secs_f32()` can be computed; compute `t = ((now - last_at).as_secs_f32() / period_secs).clamp(0.0, 1.0)` per metric
- [x] Update each strip renderer to draw column index `i` at sub-pixel `x = i as f32 - t` and call `splat_gaussian` instead of `put`
- [x] Apply fade-in alpha to the rightmost (newest) column: `alpha = (t / FADE_IN_FRACTION).min(1.0)` where `FADE_IN_FRACTION = 0.3`
- [x] Skip drawing any column whose sub-pixel `x < 0.0` (off-screen on the left)
- [x] `cargo build --example sysmon_linux --features panel-64x32` succeeds; user runs and visually confirms smooth scrolling and absence of wiggle

## Log

- Dropped explicit `x < 0` cull. With column positions `LW - n + i - t`, the leftmost (i=0) at full buffer slides from x=0 to x=-1 over a period; culling at x<0 made it invisible for ~99% of the period — defeats the smooth slide. Replaced with implicit cull via `splat_gaussian`'s in-canvas bounds check on each pixel of its kernel; samples that have slid fully off (x ≤ -BLUR_HALF_PIX) contribute no on-canvas pixels naturally. Approach text says "x<0" but the user-facing intent ("tiny tail OK") is preserved by this cleaner implementation.
- Post-build tweaks (small): bumped `RENDER_FPS` from 10 to 20 for visibly smoother slide; dimmed RAM `Buffers` and `Cached` palette so the visually-dominant rows no longer overstate "fullness" (these categories are reclaimable, so they shouldn't read as committed); pushed the CPU palette further apart for type contrast (user → bright cyan, system → medium teal, iowait → brighter saturated blue).
- Discussed adding more CPU categories (split softirq, nice, etc.) and per-core CPU sub-bands. Both deferred: extra categories not pursued (kept user/system/iowait); per-core sub-bands folded into the upcoming rotation change since both touch CPU-area layout.

## Conclusion

Smooth fading shipped. Every strip slides continuously between samples at `RENDER_FPS = 20`, with dots blurred via a 1-D horizontal Gaussian (sigma 0.7, ±2 px) and the newest column fading in over the first 30 % of its period. Disk visibility issue from the previous change is fully resolved — the log-scale density combined with the smooth motion makes idle activity readable while bursts still pop.

Out-of-plan deviations beyond what's already in the Log were minor: palette adjustments (RAM dimming, CPU contrast, IOWAIT brightness) and the FPS bump.

Bumped `hub75-client` to **0.2.3**. No project changelog, no map.

Per-core CPU sub-bands and the panel rotation (90 CCW with vertical strips, time flowing top-down) are queued as the next change.

