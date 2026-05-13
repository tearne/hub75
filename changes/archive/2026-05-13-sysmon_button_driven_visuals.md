# Sysmon button-driven visuals

**Mode:** Wander

## Intent

Have sysmon consume the button events from the firmware (the bulk IN channel proved out in the previous change) and react to them on the panel. Two starting behaviours:

- **Button A** — toggle layout: current 3-band configuration ↔ "just the fastest band, full panel" (the most responsive view, no aggregation rows).
- **Button B** — A/B mode toggle, starting as a switch between two configurations of frame-rate ramp parameters. Future presses will swap in other A/B experiments (palette, scroll behaviour, …) as we explore.

This is an open-ended Wander — these two behaviours are the starting point; the work may grow new directions while running with the panel in front of us.

## Log

- Bumped sysmon to 0.7.0. Main loop now drains pending button events via `Hub75Client::recv_event` (1 ms timeout = effectively non-blocking) each cycle, tracks last packed state, and detects press edges per bit.
- Button A toggles `layout` between `LAYOUT_BANDED` (the existing `[32, 20, 12]` heights with `[1, 12, 12]` aggregation) and `LAYOUT_FAST_ONLY` (`[64, 0, 0]` heights, `[1, 1, 1]` factors — full panel, raw samples only). On press, the `Slate` is recreated with the new layout (band-row history is dropped; sampler state — CPU prev counters, throughput running peaks — is preserved).
- Lifted `MAX_BAND_HEIGHT` from 32 to 64 so storage fits the fast-only layout. `BAND_HEIGHTS` / `AGGREGATION_FACTORS` consts replaced by a `BandLayout` struct that `MetricBands::new` and `Slate::new` take by reference. `MetricBands::push_sample` now breaks the band cascade when it hits a `height == 0` band (otherwise `Band::push` would index empty row storage).
- Button B toggles between `RAMP_A` (existing values: 0.2–30 Hz, τ=3 s) and `RAMP_B` (1–60 Hz, τ=0.5 s) — more aggressive, snappier inertia. Reusing the existing label-drawing path: mode A = no label, mode B = the 'B' glyph on the right edge, so the live mode is visible on the panel.
- Removed the compile-time `AB_PALETTE_MODE` constant — palette A/B comparison is now superseded by button-B-driven ramp A/B (palette experiments can be re-bound to a button later if wanted). The `PALETTE_B` const stays in `projection.rs` for future palette experiments.
- Release build clean. Ready for hands-on testing.
- Startup pause noticed: at idle CPU the EMA begins at 0 → adaptive cycle picks the slowest end (~5 s/frame at `min_hz = 0.2`), so the first row appears then visibly sits before the next. Fixed by initialising `smoothed_busy = 1.0` — the ramp now opens at max Hz and eases toward idle. Bumped to 0.7.1.
- User decided the Wander was done. Reset to a clean shipping state: default layout is `LAYOUT_FAST_ONLY` (button A still toggles to banded and back); button B handling and the `RampParams` A/B infrastructure removed (ramp constants restored to plain `ADAPTIVE_*` consts). Button B is reserved (still emitted by firmware) for future experiments. Bumped to 0.7.2.

## Conclusion

Sysmon now reads button events. The infrastructure: `drain_button_events` pulls all pending bytes from `Hub75Client::recv_event` per cycle, returning the latest packed state and a bitmask of newly-pressed buttons since the previous state. Each cycle starts with that drain, then runs the existing sample → render → sleep loop.

The button-A behaviour landed as a layout toggle between `LAYOUT_BANDED` (the original 3-band 32/20/12 view) and `LAYOUT_FAST_ONLY` (band 0 alone at full panel height, no aggregation). Implementation note: `bands.rs` gained a `BandLayout` struct passed to `MetricBands::new` and `Slate::new`, `MAX_BAND_HEIGHT` rose from 32 to 64 to hold the full-panel layout, and the push cascade now stops at `height == 0` bands so the empty downstream bands don't index nonexistent row storage. Switching layout re-creates the `Slate` — band-row history is dropped, but sampler-side state (CPU jiffy counters, throughput running peaks) survives.

Default layout chosen as fast-only; banded reachable via button A. Button B is wired in the firmware but not consumed by sysmon — reserved for the next experiment.

Adaptive-ramp startup pause fixed (init `smoothed_busy = 1.0`).

**Documentation impact:** `sysmon/map.md` describes the 3-band layout as the layout. With a runtime-selectable layout now in play, the Bands and possibly Display nodes warrant a catch-up — flag for per-node negotiation when ready.

Final version: sysmon 0.7.2.
