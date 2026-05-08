# Client should target a specific panel when multiple are attached

**Mode:** Formal

## Intent

When more than one HUB75 panel is plugged into the host, the Rust client's `Hub75Client::open_auto()` grabs the first device matching the VID/PID/manufacturer/product strings. There's no way to say "talk to *that* panel". Today, working on a second panel requires shutting down `sysmon` and unplugging the panel it owns — friction that discourages multi-panel work and risks the kind of mishap that motivated `flash_targets_specific_panel`.

Consume the panel-identity convention established by `changes/archive/2026-05-07-flash_targets_specific_panel.md` and expose it through the client so a caller can select a specific panel. Examples (`life`, `clock`) and `sysmon` should be updatable to target a chosen panel without unplugging others.

## Approach

### Unified `open` constructor

Replace `Hub75Client::open_auto()` with `Hub75Client::open(serial: Option<&str>) -> Result<Self>`. `None` keeps today's behaviour (first matching panel); `Some(s)` filters on USB `iSerial`. One entry point makes the "do you care which panel?" question explicit at every callsite, and the existing callers all live in this repo.

### Discovery via `list_panels`

Add `Hub75Client::list_panels() -> Result<Vec<PanelInfo>>` returning a small struct per attached panel: serial, plus an availability flag (free vs already-claimed by another process — derived from a non-destructive open attempt). Cheap given the iteration code already exists; lets users see what's attached and which are usable without trying each blindly.

### Conflict avoidance is OS-level

`Hub75Client::open(None)` keeps today's first-match behaviour — when multiple panels are attached and the caller doesn't pick one, the OS-level USB interface lock is the safety net: if the chosen panel is already claimed (e.g. by `sysmon`), `claim_interface` fails with `Busy`. Loud, immediate, recoverable.

### Selector convention per consumer

Examples (`life`, `clock`) accept the serial as an optional positional CLI arg (`cargo run --example life -- living-room`); `None` falls back to first-match. `sysmon` is hardcoded to `Hub75Client::open(Some("sysmon"))` — it only ever attaches to a panel flashed with `PANEL_NAME=sysmon`. No env var, no fallback. Errors clearly if no such panel is attached. This makes sysmon's target an explicit deployment convention rather than a runtime guess, removing the failure mode where it grabs the wrong panel.

### Python client

The Python client (`usb-serial/client/python/hub75_client.py`) gets the same treatment: replace `_find_device` with a serial-aware constructor, add a listing helper, and consume `HUB75_PANEL` / a CLI arg in any scripts that use it.

### Documentation

`SETUP.md`'s existing "Targeting a specific panel" section gains runtime selector instructions: example CLI usage, `HUB75_PANEL` for sysmon.

## Plan

- [x] Rust lib: replace `Hub75Client::open_auto` with `open(serial: Option<&str>)`. Filter logic uses USB `iSerial`; descriptive error when a requested serial isn't found.
- [x] Rust lib: add `PanelInfo` struct (serial + availability flag) and `Hub75Client::list_panels() -> Result<Vec<PanelInfo>>`.
- [x] Update `usb-serial/client/rust/examples/life.rs` and `clock.rs` to take an optional positional CLI arg as the serial.
- [x] Update `sysmon/src/main.rs` to call `Hub75Client::open(Some("sysmon"))` — hardcoded target.
- [x] Update `sysmon/README.md` to document the `PANEL_NAME=sysmon` flashing requirement.
- [x] Migration step (operator action, not code): re-flash the existing sysmon panel with `PANEL_NAME=sysmon` before the new sysmon binary lands.
- [x] Python client (`usb-serial/client/python/hub75_client.py`): replace `_find_device` with a serial-aware open, add a `list_panels` helper. Update `scanline_test.py` to take an optional CLI arg.
- [x] Update `SETUP.md`: extend the "Targeting a specific panel" section with runtime selector usage (CLI arg for examples; sysmon's `PANEL_NAME=sysmon` requirement).
- [x] Smoke-test on hardware: with two panels would be ideal; with one, verify `list_panels` reports it, `open(Some("…"))` works for matching/non-matching serials, `open(None)` still works.
- [x] Bump `hub75-client` version 0.3.0 → 0.4.0 (breaking API change: `open_auto` removed).
- [x] Bump `sysmon` 0.4.2 → 0.5.0 (deployment requirement changes — `.deb` upgrade now needs a re-flashed panel).

## Conclusion

Client now consumes the panel-identity convention end-to-end. `Hub75Client::open` (Rust) and `Hub75Client(serial=…)` (Python) target by USB `iSerial`; `list_panels` enumerates with availability. Examples take an optional CLI arg; `sysmon` is hardcoded to `Hub75Client::open(Some("sysmon"))` and refuses to start without a `PANEL_NAME=sysmon` panel. `SETUP.md` and `sysmon/README.md` document the runtime selectors and the flashing requirement.

Bumps: `hub75-client` 0.3.0 → 0.4.0 (breaking — `open_auto` removed); `sysmon` 0.4.2 → 0.5.0 (deployment requires re-flash). Migration done: existing sysmon panel re-flashed with `PANEL_NAME=sysmon`; new `.deb` installed and service running.

A new POS-style `usb-serial/client/python/list_panels.py` was added — small utility for runtime panel discovery, follows the project's POS conventions.

Surprise during smoke-test: opening `via-probe` panel via `life` returned `USB transfer status 1` on the first frame and knocked the whole bus offline until power-cycled. Doesn't relate to the targeting changes themselves (open succeeded; failure was on `send_frame_rgb`). Same family of symptom as the earlier "panel corruption after clock" observation, so the parked proposal was renamed to `panel_corruption_during_client_transitions.md` and updated with the broader evidence.

Documentation impact: covered. No `map.md` to catch up.
