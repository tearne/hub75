# usb-display panel size configurability

## Intent

`usb-display/firmware` currently hardcodes the panel as 64×64 (in `display.rs` and the `ShiftPanel<64, 64>` instantiation). When a different panel is plugged in (a 64×32 was tested today), the display partially works but with addressing glitches because the firmware is sized wrong for the hardware.

The shift-register code in `hub75` is already const-generic over `<W, H>`, and the host client's protocol could just as easily carry frames at any size — the bottleneck is purely that the firmware's W/H is fixed at compile time. A user wanting to drive a 64×32 panel today has to edit the source.

The change should make it easy to build the firmware for a different panel size — and keep host and firmware agreed on what size frame is being sent.

## Approach

### Mutually-exclusive Cargo features for panel size

Each crate that needs a size at compile time defines a `panel-WxH` feature for each supported physical size. **No default feature** — user must pick one. A `compile_error!` cfg block fires if zero features are selected, naming the available choices. If two are accidentally enabled, a second `compile_error!` catches the conflict.

Build invocation:

```sh
cargo build --release --features panel-64x32
```

Discoverable via `cargo build --help`, IDE tooling, and rust-analyzer. New panel sizes are added by extending the Cargo.toml feature list and the cfg blocks.

### Firmware and Rust client share the feature contract

Both `usb-display/firmware` and `usb-display/client/rust` declare the same `panel-WxH` feature names. Build both with the same feature and they agree on frame size by construction.

### Python client takes dimensions as constructor args

Python has no compile step; the `Hub75Client` constructor takes `width` / `height` arguments. Caller passes the size matching the running firmware.

### Spwm side stays single-size for now

`Dp3364sPanel` is currently hardcoded 64×128 (not const-generic) — chip count, scan lines, the pack algorithm's chip-major loop, and `CONFIG_REGS` all bake in those numbers. Making it size-generic is a non-trivial refactor that we shouldn't do without a second physical spwm panel to validate against. This change therefore touches only the shift-register / `usb-display` side. When a second spwm panel size shows up, that refactor can land then and the same Cargo-feature pattern can be applied to its consumer.

### No runtime USB handshake

Out of scope. The frame protocol stays as-is; host and firmware agree on size by external coordination (whoever builds them picks matching features / args).

### Documentation

`usb-display/README.md` gets a "Panel size" section listing the available `panel-WxH` features and the matching Python client argument, with example invocations.

## Plan

- [x] Declare `panel-64x64`, `panel-64x32` features (no `default`) in `usb-display/firmware/Cargo.toml`. Bump version to `0.2.0` (breaking — no default panel size).
- [x] In `usb-display/firmware/src/display.rs`: replace the hardcoded `WIDTH`/`HEIGHT` consts with `#[cfg(feature = "panel-WxH")]` blocks for each size; add a `compile_error!` cfg block that fires when zero features are selected (listing the available ones) and another for when two or more are enabled.
- [x] Confirm `display::WIDTH` / `display::HEIGHT` flow through to `ShiftPanel<WIDTH, HEIGHT>` in `main.rs` cleanly (no further changes needed there since it already references `display::{WIDTH, HEIGHT}`).
- [x] Mirror the feature set in `usb-display/client/rust/Cargo.toml` (same names; no default; bump version to `0.2.0`) and apply the same cfg blocks + compile_error guards to `WIDTH`/`HEIGHT` in `usb-display/client/rust/src/lib.rs`.
- [x] Update `usb-display/client/rust/examples/` (`clock.rs`, `life.rs`) if they reference `WIDTH`/`HEIGHT` directly — they should pick them up via the lib's cfg-selected consts unchanged, but verify.
- [x] Update `usb-display/client/python/hub75_client.py` so `Hub75Client` takes `width` and `height` constructor args (no default — caller must pass), and the helper functions in the file thread them through.
- [x] Update example/test scripts that use the Python client (`scanline_test.py` and the `if __name__ == "__main__"` block in `hub75_client.py`) to pass dimensions explicitly.
- [x] Add a "Panel size" section to `usb-display/README.md` documenting the available feature names, the build invocation (`cargo build --features panel-WxH`), the Python `Hub75Client(width=, height=)` argument, and the requirement that firmware and host agree.
- [x] Verify firmware builds for both `panel-64x64` and `panel-64x32` features.
- [x] Verify the Rust client builds for both `panel-64x64` and `panel-64x32` features.
- [x] Hardware verify: build the firmware with `--features panel-64x32`, flash it, and confirm the USB-driven display works correctly on the 64×32 panel.

## Conclusion

Completed as planned. Both `usb-display/firmware` and `usb-display/client/rust` are now feature-selected (`panel-64x64`, `panel-64x32`; no default; mutually-exclusive with `compile_error!` guards); `usb-display/client/python`'s `Hub75Client` takes `width`/`height` constructor args. Both Rust crates bumped to `0.2.0`. Documented in `usb-display/README.md`.

Project does not maintain a changelog; no entry proposed.

