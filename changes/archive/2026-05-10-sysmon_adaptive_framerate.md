# Sysmon adaptive frame rate

**Mode:** Wander

## Intent

Make sysmon's frame rate vary with CPU usage rather than running at a fixed rate. Higher CPU load → faster updates (more responsive when something is happening); low load → slower updates (less self-induced load when idle).

Bounds: min 1 fps, max 20 fps.

## Conclusion

Sysmon's main loop now derives its cycle duration from CPU load rather than running fixed. `parse_cycle_arg()` returns `Option<Duration>`; `None` means adaptive. Each cycle, `sample_all` exposes the per-sample max-core busy fraction, which feeds an EMA (`tau = 3 s`, alpha derived from actual elapsed-since-last-sample so it adapts to the variable cycle). The smoothed signal is linearly mapped to Hz between `ADAPTIVE_MIN_HZ` and `ADAPTIVE_MAX_HZ`. `-f` still forces a fixed rate and skips the adaptive path entirely.

Final bounds after iteration: 0.2–30 Hz. Shipped as sysmon 0.6.5.

No documentation impact — `sysmon/map.md` describes devices/metrics/display but not the loop cadence, which is implementation detail. The startup banner now reports adaptive vs. fixed mode.

## Log

- Implemented in `main.rs`: `parse_cycle_arg()` now returns `Option<Duration>`; `None` means adaptive. Each loop iteration calls `adaptive_cycle(peak_busy)`, where `peak_busy` is the max across cores from the latest CPU sample. Linear interpolation: `hz = 1 + 19 * peak_busy`. `-f` override unchanged.
- Bumped sysmon to 0.6.0. Release build clean.
- Lowered min to 0.1 Hz (10 s cycle) per user. Note: at idle the CPU sampler differences against the previous reading, so a brief CPU spike during a 10 s gap will be averaged across the full interval — adaptive ramp-up may feel sluggish on short bursts. Bumped to 0.6.1.
- Added symmetric inertia: EMA on the driving busy signal with `tau = 3 s`, `alpha = 1 - exp(-dt/tau)` per cycle (`dt` is actual elapsed since last sample, so it adapts to the variable cycle). Bumped to 0.6.2.
- Raised min back to 0.5 Hz (2 s cycle at idle). Bumped to 0.6.3.
- Min set to 0.2 Hz (5 s cycle at idle). Bumped to 0.6.4.
- Max raised to 30 Hz. Bumped to 0.6.5.
