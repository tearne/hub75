# Dedicated firmware build for sysmon

**Mode:** Formal

## Intent

The firmware currently embeds its USB serial number via the `PANEL_NAME` env var at build time. Sysmon opens its panel by serial `"sysmon"`, so any rebuild that omits `PANEL_NAME=sysmon` produces firmware that sysmon can't find — a footgun that has just bitten in practice.

Provide a more ergonomic way to build the sysmon firmware so this can't be accidentally dropped: e.g. a `cargo build` alias, a feature flag, a wrapper script, or a per-product Cargo profile. Decide on the cheapest approach that makes the right command obvious and the wrong command harder.

## Approach

### New library crate at `usb-serial/firmware-lib/`

Extract the bulk of `usb-serial/firmware/src/main.rs` and its sibling `display.rs` into a new library crate. The library carries everything product-independent: USB descriptor setup, bulk OUT framing (`FrameReceiver`/`ReceiveBuffer`), the panel task, the button polling task, the constants (`USB_VID`, `USB_PID`, `BULK_MAX_PACKET`, etc.), and a configuration surface for the binary to fill in. Name: `usb-serial-firmware-lib` — sits next to the protocol it implements; can be promoted to a top-level crate later if multiple firmwares accumulate.

### Library API

A `FirmwareConfig` struct passed in by the binary, carrying the identity bits: `serial_number: &'static str`, and a function returning the firmware's `embassy` runtime entry point (`pub async fn run<P: Panel>(spawner, config, panel, usb_periph, button_a, button_b)`). Panel construction stays in the binary because the panel type depends on the panel-feature gate, which is binary-specific.

### Two firmware binaries

- **`usb-serial/firmware/`** keeps its location and continues to exist as the default firmware. `main.rs` shrinks to ~30 lines: `embassy_executor::main`, construct panel from feature-gated type, call `firmware_lib::run` with a config that uses the existing `PANEL_NAME` env var → chip-ID fallback. Existing `cargo build --features panel-…` invocation still works for `life`/`clock` examples, generic boards, etc.

- **`sysmon/firmware/`** new crate. Owns its own `Cargo.toml`, `.cargo/config.toml` (embedded target), `memory.x`, `build.rs`. `main.rs` is the same ~30-line shape as the default firmware but hard-codes `serial_number: "sysmon"` and picks a panel feature (initially `panel-shift-64x32`, the panel currently driving sysmon). Built with `cargo build --release` from `sysmon/firmware/` — no env vars, no feature flags to remember.

### Documentation

`usb-serial/README.md` gains a short "Firmware variants" section explaining the lib + per-product binary split. `sysmon/README.md` (or a new `sysmon/firmware/README.md`) describes the sysmon firmware build and flash. `SETUP.md` updated to point at the right binary.

## Plan

- [x] Create `usb-serial/firmware-lib/` crate (Cargo.toml, no_std, embassy/embassy-rp/embassy-usb dependencies mirrored from current firmware).
- [x] Move `display.rs` (FrameReceiver, ReceiveBuffer, WIDTH/HEIGHT cfg) into the lib.
- [x] Move USB descriptor constants, `panel_task`, `usb_task`, button polling task, and `Debouncer` from firmware `main.rs` into the lib; expose them as a `run(...)` entry function plus a `FirmwareConfig { serial_number }` surface.
- [x] Shrink `usb-serial/firmware/src/main.rs` to the embassy-main + panel-construction + `lib::run` shape; identity continues via `PANEL_NAME` env var → chip-ID fallback.
- [x] Create `sysmon/firmware/` (Cargo.toml, `.cargo/config.toml` for `thumbv8m.main-none-eabihf`, `memory.x`, `build.rs`, `src/main.rs` hard-coding `serial_number: "sysmon"` and the `panel-shift-64x32` feature).
- [x] Both firmwares build clean.
- [x] Update `usb-serial/README.md` (firmware variants section), add `sysmon/firmware/README.md`, update `SETUP.md` flashing pointer.
- [x] Flash sysmon firmware and confirm `cargo run --release` in `sysmon/` finds the panel by serial.

## Log

- `usb-serial/firmware-lib/` new crate at 0.1.0. Owns `display.rs` (`FrameReceiver`, `ReceiveBuffer`, panel `WIDTH`/`HEIGHT`), USB descriptor constants (`USB_VID`, `USB_PID`, `VENDOR_CLASS`, `BULK_MAX_PACKET`), the cross-task `FRAME_READY` signal + `RX_BUF` static, `Debouncer`, and two top-level async functions:
  - `run_panel<P: Panel<Frame = [[Rgb; WIDTH]; HEIGHT]>>(panel)` — feeds completed frames from `RX_BUF` into the panel.
  - `run_usb_and_buttons<D: embassy_usb::driver::Driver<'static>>(driver, pin_14, pin_15, &config)` — builds the USB descriptor, runs the OUT receiver and IN button emitter, joined.
- Binaries combine them with `embassy_futures::join::join(run_panel, run_usb_and_buttons)` inside `#[embassy_executor::main]`. The Panel trait's associated `Frame` is unsized in general, so `run_panel` constrains it to the concrete `[[Rgb; WIDTH]; HEIGHT]` from the lib's feature gate — both shift and SPWM panels match.
- `usb-serial/firmware/` at 0.8.0: same shape and behaviour as before from the outside; main.rs is now ~85 lines (was ~325). `display.rs` deleted from this crate. `PANEL_NAME` env var → chip-ID fallback retained.
- `sysmon/firmware/` at 0.1.0: own `.cargo/config.toml`, `memory.x`, `build.rs`. `main.rs` is the same shape as the default firmware's main, with `PANEL_SERIAL = "sysmon"` hard-coded and only the shift-panel branch — no feature gates needed.
- The shift-only-panel sysmon variant means its `Cargo.toml` has `default = ["usb-serial-firmware-lib/panel-shift-64x32"]` so a plain `cargo build --release` produces the right binary.
- Both firmwares build green; lib + default firmware via `cargo build --features panel-shift-64x32`, sysmon firmware via plain `cargo build`.
- Docs updated: `usb-serial/README.md` gained a "Firmware variants" section, `SETUP.md`'s sysmon paragraph rewritten to point at the dedicated crate first, new `sysmon/firmware/README.md`, top-level `README.md` structure block updated to mention the sysmon firmware home, `sysmon/README.md` flashing instructions rewritten to use `cd firmware && cargo build --release` (no `PANEL_NAME` env var). `hub75/README.md` is panel-driver-only and didn't need touching.

## Conclusion

The footgun is gone: sysmon's firmware is now a dedicated `sysmon/firmware/` crate with `"sysmon"` and the 64×32 shift panel hard-coded. Build with plain `cargo build --release` from that directory — no env var, no feature flags. Identity is structural; you can't build the sysmon firmware without sysmon identity.

The bulk of firmware logic (USB descriptors, frame protocol, button polling, debouncer) extracted to `usb-serial/firmware-lib/`. Both the default firmware (`usb-serial/firmware/`) and the sysmon firmware are ~30-line shells that construct a panel, build the USB driver, and `join` two library async functions — `run_panel` and `run_usb_and_buttons`. The default firmware preserves the `PANEL_NAME` env var workflow for ad-hoc boards and the `life`/`clock` examples.

The `Panel` trait's `Frame` associated type is `?Sized` in general; the library bounds `run_panel` to `Frame = [[Rgb; WIDTH]; HEIGHT]` from its own feature gate, which both panel families satisfy.

Versions: `usb-serial-firmware-lib` 0.1.0, `usb-serial-firmware` 0.7.0 → 0.8.0, `sysmon-firmware` 0.1.0.
