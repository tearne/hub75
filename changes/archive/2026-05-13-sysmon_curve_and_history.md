# Sysmon load-curve and history across layouts

**Mode:** Formal

## Intent

Two adjustments to sysmon's behaviour:

**1. Reshape the CPU-load → frame-rate curve.** Currently the cycle interpolates linearly between `ADAPTIVE_MIN_HZ` and `ADAPTIVE_MAX_HZ` against peak-core busy. At "normal" idle-ish loads (single-digit %), this already runs noticeably faster than the floor. The user wants the panel to sit much closer to `ADAPTIVE_MIN_HZ` under normal low levels of CPU activity, and ramp up sharply only when the host is genuinely busy. Same bounds (0.2–30 Hz); curve shape, not endpoints.

**2. Preserve band history across layout toggles.** Pressing button A currently re-creates the `Slate`, so the new layout starts with empty bands and has to refill from scratch. With both layouts seen often, this is jarring. The fix: keep band history live for both layouts simultaneously (or migrate it on switch) so toggling shows the same data, just re-projected.

## Approach

### Curve: power law with exponent `p > 1`

Replace the linear interpolation in `adaptive_cycle` with `hz = min + (max - min) * busy^p` and start at `p = 3.0`. At idle-ish loads (e.g. 10 % peak-core busy) the cycle now lands close to `ADAPTIVE_MIN_HZ` rather than ~10 % of the way up the range, and the rate climbs sharply only when peak-busy approaches 1.0. One knob, easy to retune from the panel after watching it run.

### History: keep both layouts live in parallel

Maintain two `Slate` instances — one with `LAYOUT_BANDED`, one with `LAYOUT_FAST_ONLY` — and push every sample to both each cycle. The renderer reads from whichever the active layout selects. Switching layouts becomes a pointer flip; both histories stay correct because both have been continuously fed.

Reason: simplest and most accurate. The alternatives — keeping a raw-sample ring and rebuilding bands on switch, or projecting one layout's rows into the other — are either lossy or significantly fiddlier. The memory cost is trivial (the band data is small) and the per-sample CPU cost is doubled on the band path only, which itself is already cheap compared with the renderer.

A small wrapper owns the two `Slate`s and exposes "push to all" and "active slate to render" — the rest of the code reads from one slate as today.

## Plan

- [x] `main.rs`: change `adaptive_cycle` to `hz = min + (max - min) * busy^3.0`.
- [x] Add a `Slates` wrapper holding banded + fast-only `Slate` instances and an active selector; expose push and active-render accessors.
- [x] Update `sample_all` and the render call to use `Slates`.
- [x] Button A flips `Slates::active` instead of re-creating the slate.
- [x] Verify on board: ramp feels closer to min at idle, toggle preserves both views' history.

## Log

- `adaptive_cycle` now maps `busy` through `weight = busy.powf(ADAPTIVE_CURVE)` (3.0) before the linear interpolation. At 10% peak-busy the weight is 0.001 → cycle sits within ~0.2 % of `ADAPTIVE_MIN_HZ`; at 50 % busy it's 0.125 → ~3.9 Hz; at 100 % it's 1.0 → max. Sharp ramp near the top, flat near the floor.
- `slate.rs` gained `LayoutMode` enum and a `Slates` wrapper owning both banded and fast-only `Slate`s plus an active marker. `each_mut()` returns `[&mut Slate; 2]` for fan-out pushes; `active()` returns the read-only slate the renderer uses; `toggle()` flips active and returns the new mode for logging.
- `sample_all` now iterates `slates.each_mut()` for each metric, so both layouts receive every sample. CPU peak detection moved out of the band-push loop into its own pass.
- Button A handling collapses to a single `slates.toggle()` call — no re-creation, no history loss.
- Build clean.
- Raised `ADAPTIVE_MIN_HZ` from 0.2 to 0.5 Hz (2 s at idle). Bumped to 0.7.4.
- Raised `ADAPTIVE_MAX_HZ` from 30 to 50 Hz. Bumped to 0.7.5.

## Conclusion

Both items landed. Curve change is one line plus a tuned constant: `weight = busy^ADAPTIVE_CURVE` (3.0) before the linear interpolation, so the panel sits near `ADAPTIVE_MIN_HZ` at routine idle-ish loads and only ramps up when peak-core busy approaches 100 %. History preservation is a new `Slates` wrapper in `slate.rs` that owns both layouts and fans `push_sample` out to each; the renderer reads the active one. Button A is a single `toggle()` call. Final bounds 0.5–50 Hz after live tuning.

Final version: sysmon 0.7.5.
