# Panel rendering options

## Intent

Hardware testing turned up two distinct per-panel rendering quirks: G/B channels swapped on the 64×32 shift panel, and a horizontal mirror on the 128×64 S-PWM panel (`clock` runs backwards). Both look like physical-panel-vendor wiring differences. The 64×64 shift panel hadn't been tested yet.

Separately, the user prefers the channel-swapped palette `life` was producing on the mis-wired 64×32 and wants that look as the example's deliberate default — independent of which panel is plugged in.

This change diagnoses how each panel actually renders, lands per-panel wiring corrections, and applies the swapped palette to `life`.

## Approach

### Decouple firmware wiring fixes from `life`'s palette choice

Wiring fixes go in `hub75` (`pack.rs` per family, cfg-gated by a per-panel feature) so the host always sees a true-RGB pixel grid. The `life` palette swap goes in `life.rs`'s `hsv()`, independent of any panel.

### Diagnostic ships first as a permanent tool

`hub75/examples/test_pattern.rs` cycles through solid R/G/B/W, solid Y/C/M, an asymmetric "sunset" scene (sun in upper-left, sky on top, dark ground), and a bouncing ball (gravity = down). Each solid fill renders the colour name in tiny black text on the panel itself, so the example characterises a panel's wiring even without a debug probe. One file, cfg-gated by panel feature.

### Wiring features live on `hub75`, named after the panel

`hub75` features `panel-shift-64x64`, `panel-shift-64x32`, `panel-spwm-128x64` — these select the panel for the test-pattern example *and* gate the wiring-correction edits in `pack.rs`. `usb-display/firmware` declares matching features that forward to the `hub75` ones, so picking the firmware panel auto-enables the wiring corrections.

### Rotation deferred

Considered including R0/R180/R90/R270 rotation for user-mounting orientation; deferred to a follow-up change. R90/R270 swap logical W/H from physical W/H and need an API-shape decision; an R0/R180-only half-implementation doesn't help the user's real 90° CW mounting need.

### Quirk table (filled in during build)

| Panel | Channel order | Column direction | Row direction |
|---|---|---|---|
| shift 64×32 | G/B swapped | correct | correct |
| shift 64×64 | correct | correct | correct |
| spwm 128×64 | correct | reversed | correct |

## Plan

- [x] Add `panel-shift-64x64` / `panel-shift-64x32` / `panel-spwm-128x64` features to `hub75/Cargo.toml`. Bump `hub75` to `0.3.0`.
- [x] Add `hub75/examples/test_pattern.rs` cycling through R/G/B/W → Y/C/M → sunset → bounce, with a tiny on-panel text label naming each solid fill.
- [x] Run the test pattern on each known panel and fill in the quirk table.
- [x] Cfg-gate the wiring corrections in `hub75/src/<family>/pack.rs`: G/B swap when `panel-shift-64x32` is enabled (in `shift/pack.rs`); read from mirrored column when `panel-spwm-128x64` is enabled (in `dp3364s/pack.rs`).
- [x] Forward `usb-display/firmware`'s panel features to `hub75`'s.
- [x] Swap G and B in `usb-display/client/rust/examples/life.rs`'s `hsv()` for the deliberate palette.
- [x] Update `hub75/README.md` and `usb-display/README.md` for the new features and the diagnostic workflow.
- [x] Build verify across all panel features; hardware re-verify each panel.

## Log

- Replaced the Approach's original "named-by-behaviour" wiring features (`shift-gb-swap`, `spwm-cols-reversed`) with panel-named features (`panel-shift-64x32`, etc.) that double as both example selector and wiring-correction gate. One axis instead of two; matches what `usb-display/firmware` already had.
- The test-pattern example evolved during build — started as ramps + corner marker, became sunset for spatial recognisability, gained a bouncing ball for unambiguous "down", then on-panel text labels for probe-less channel identification.
- Added an R0/R180 rotation constructor parameter, then reverted entirely: the user's actual need is 90° CW (not 180°), so the half-feature wasn't useful today. Full rotation work moves to a follow-up.

## Conclusion

Completed. Wiring corrections in place for both physical panels with quirks; clean panel passes the test pattern unchanged. `test_pattern` is now a permanent diagnostic tool. `life`'s palette swap shipped independently.
