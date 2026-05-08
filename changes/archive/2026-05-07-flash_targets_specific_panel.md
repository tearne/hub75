# Flashing should target a specific panel when multiple are attached

**Mode:** Formal

## Intent

When more than one HUB75 panel is plugged into the host, flashing currently has no safe way to target a chosen panel — the flashing tool picks one of the attached devices, which may not be the intended one. This already caused real damage: a USB cable failed mid-test, a different panel was reflashed by accident, and because the panels are physically different types the wrong firmware caused cascading problems.

Establish the convention this project will use to identify panels (e.g. a serial number, a user-set name, a slot tag — TBD), and use it to make flashing target a specific panel deterministically. This change is the one that decides the identity scheme; later changes (notably client-side targeting) will consume it.

## Approach

### Identity

Each Pico carries two distinct unique IDs: the SPI flash chip's unique ID (visible to `picotool info -a` and used by `picotool load --ser`), and the RP2350 silicon's OTP chip ID (readable from firmware via `embassy_rp::otp::get_chipid`). The firmware uses the OTP chip ID as its USB `serial_number`, emitting the low 32 bits as 8 hex chars — readable in `lsusb`, collision risk negligible at this project's scale. Picotool keeps using the flash unique ID for `--ser`; the two IDs identify the same board but are different values. Acceptable because they're consulted in non-overlapping situations: picotool's ID matters only when multiple boards are in BOOTSEL (firmware not running), the firmware's ID matters only at runtime (BOOTSEL not in play).

Reason for not unifying: reading the SPI flash unique ID from firmware on RP2350 isn't supported by `embassy-rp` 0.10 (rp2040-only) and would require non-trivial unsafe XIP/ROM plumbing for a property that isn't user-visible in normal use.

### Friendly-name override

A `PANEL_NAME` env var, read by the firmware's existing `build.rs` and baked into the binary as a `&'static str`, replaces the chip-ID default for the USB `serial_number`. Single-panel users build with no env var and get a working unique ID for free; users juggling panels bake names in per-flash.

### Flash-time targeting

For the BOOTSEL+picotool flashing path, `picotool load --ser <flash-id>` (already supported) is the targeting mechanism. SETUP.md gains guidance: discover IDs via `picotool info -a`, target with `--ser`. The debug-probe+probe-rs path already targets physically (one probe, one wired board) and needs no change.

### Picotool is not a new requirement

The convention is path-agnostic. Default single-panel use needs nothing: firmware emits its chip ID as `serial_number`, host picks up whatever's attached. Discovering a chip ID can also be done via `lsusb -v` (once firmware is running) or probe-rs over SWD — picotool is just one option. Picotool is only unavoidable in the narrow case of "multiple boards in BOOTSEL, deterministic targeting wanted" — and that workflow already uses picotool.

## Plan

- [x] In `usb-serial/firmware/build.rs`, read `PANEL_NAME` env var; emit a `cargo:rerun-if-env-changed` directive and a `cargo:rustc-env=PANEL_NAME=...` (or unset) so the firmware can read it via `option_env!`.
- [x] In `usb-serial/firmware/src/main.rs`, replace the hard-coded `serial_number = Some("001")` with: if `option_env!("PANEL_NAME")` is set, use that; else read RP2350 chip ID, format last 8 hex chars into a `StaticCell<heapless::String<…>>` (or equivalent `&'static str`), use that.
- [x] Verify the firmware builds for each panel feature, with and without `PANEL_NAME` set.
- [x] Smoke-test on hardware: confirm `lsusb -v` shows the new `serial_number` (chip ID by default, override when `PANEL_NAME` was set at build).
- [x] Update `SETUP.md`: add a "Targeting a specific panel" section covering `picotool info -a` discovery, `picotool load --ser <id>`, and the `PANEL_NAME=foo` build override.
- [x] Bump `usb-serial-firmware` version 0.5.1 → 0.6.0 (added retroactively; missed in initial Plan).

## Conclusion

Convention established: each panel advertises a unique USB `serial_number` — the low 32 bits of the RP2350 OTP chip ID by default, or a `PANEL_NAME=…` build-time override. `SETUP.md` documents both the BOOTSEL `picotool --ser` and probe-rs flows. Verified on hardware via both flashing paths: default emitted `56c8c555`; `PANEL_NAME=living-room` and `PANEL_NAME=via-probe` overrides both surfaced correctly in `lsusb`.

Deviations:
- Approach was reframed mid-build from "one physical fact, two views" to two distinct IDs (SPI flash unique ID for picotool, OTP chip ID for firmware); details in the Log.
- Version-bump task added retroactively (missed in initial Plan).

Documentation impact: none beyond the `SETUP.md` edit shipped here. No `map.md` to catch up.

## Log

- Approach revised mid-build: original framing said "one physical fact, two views". RP2350 has two distinct unique IDs (SPI flash chip's, used by picotool; OTP chip ID, used by firmware) and `embassy-rp` 0.10 only exposes the flash unique ID for rp2040, not RP2350. Reading it on RP2350 needs custom unsafe XIP/ROM plumbing. Switched to OTP chip ID (`embassy_rp::otp::get_chipid`) — Approach updated. Two IDs per board accepted because they're consulted in non-overlapping situations.

