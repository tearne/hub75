# Consider panel-identity convention for `usb-drop` firmware

**Mode:** Formal

## Intent

`flash_targets_specific_panel` establishes a panel-identity convention for the `usb-serial` firmware: each panel advertises a unique USB `serial_number` (default: last 8 hex chars of RP2350 chip ID; override: `PANEL_NAME` env var). `usb-drop` firmware was kept out of scope to keep that change focused.

Decide whether `usb-drop` should adopt the same convention, a divergent one, or none. Considerations: does anyone juggle multiple `usb-drop` boards; does `usb-drop` have its own host-side discovery story; does flash-time disambiguation matter for it.

## Approach

### Adopt the convention

`usb-drop` joins `usb-serial` in advertising a unique USB `serial_number` per board. Rationale: as a Mass Storage Class device, multiple boards on the same host with identical serials would clash at OS mount time (`/dev/disk/by-id/...` paths embed the serial). The motivation is stronger for `usb-drop` than for `usb-serial` because the OS itself depends on serial uniqueness.

### Chip ID only — no `PANEL_NAME` override

`usb-drop` doesn't need the build-time friendly-name override that `usb-serial` carries. Mass Storage mounts surface the serial in disk-by-id paths automatically, so chip-ID-as-serial gives a stable, unique identifier without the host having to remember a chosen name. Skipping the override removes a build-time variable, a `build.rs` change, and a code path. Reconsider later if a use case emerges.

### Implementation

- Identity: `rp235x-hal::rom_data::sys_info_api::chip_info().device_id` — a `u32`, formatted as 8 lowercase hex chars. No `embassy-rp` OTP API in this stack.
- `&'static str` materialisation: via `static_cell::StaticCell`, added as a new dependency.

### Documentation

`SETUP.md`'s "Targeting a specific panel" section gains a one-line note that `usb-drop` follows the same chip-ID-as-serial convention without a build-time override.

## Plan

- [x] Add `static_cell` to `usb-drop/firmware/Cargo.toml`.
- [x] In `usb-drop/firmware/src/main.rs`, add a `panel_serial_number() -> &'static str` helper: read `chip_info().device_id`, format low 32 bits as 8 lowercase hex chars into a `StaticCell`-backed buffer.
- [x] Replace the hard-coded `.serial_number("001")` in the USB device builder with the helper.
- [x] Verify the firmware builds.
- [x] Smoke-test on hardware: confirm `lsusb -v` shows the new chip-ID serial; confirm a stable mount path under `/dev/disk/by-id/`.
- [x] Update `SETUP.md`'s "Targeting a specific panel" section with a one-line note that `usb-drop` follows the same chip-ID-as-serial convention without a build-time override.
- [x] Bump `usb-drop-firmware` 0.1.0 → 0.2.0 (observable USB descriptor change).

## Conclusion

`usb-drop` adopts the chip-ID side of the panel-identity convention: the firmware now advertises the RP2350 `device_id` as 8 lowercase hex chars in its USB `serial_number`. Verified on hardware: `lsusb` shows `iSerial 3 a666bdeb`; `/dev/disk/by-id/usb-tearne_hub75-drop_a666bdeb-0:0` provides a stable mount path. No `PANEL_NAME` override per the simplified scope. `SETUP.md` documents the firmware's behaviour. Bumped `usb-drop-firmware` 0.1.0 → 0.2.0.

Surprise during the bonus mount-and-render test: the firmware doesn't render dropped `.H75` files. The drive mounts and files reach disk fine, but the panel stays blank. Out of scope for this change (the serial work is unrelated to the render path); parked as `changes/open/usb_drop_not_rendering.md` for focused investigation with `probe-rs`/`defmt`.

Documentation impact: covered. No `map.md` to catch up.

## Log

- `chip_info()` returns `Result<Option<ChipInfo>, BootRomApiErrorCode>` rather than the bare `ChipInfo` I'd assumed. Handled with `.ok().flatten().map(...).unwrap_or(0)` so a ROM call failure shows up as serial `00000000` rather than panicking on boot.
