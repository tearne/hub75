# Remove RGB primer from the `life` example

**Mode:** Formal

## Intent

The Rust `life` example currently runs a 2-second yellow → green "RGB sanity check" before starting Game of Life. We added it earlier as an ad-hoc channel-order diagnostic, but the project now has a proper [`test_pattern`](../hub75/examples/test_pattern.rs) example for that. The primer in `life` is redundant clutter at the start of every run; remove it so `life` jumps straight into the simulation.

## Approach

Delete the primer block (the `yellow` and `green` array literals, the two `send_frame_rgb` + `sleep` calls, and the `println!` announcing it). Nothing depends on it; nothing else needs to change.

## Plan

- [x] Remove the RGB sanity-check block from `usb-serial/client/rust/examples/life.rs`.
- [x] Fix stale `Hub75Client::open(&p)` call in `life.rs` (library API moved to `open_auto` in commit `3cbc886`; the example was missed).
- [x] Verify `life` still builds for each panel size feature.

## Conclusion

Primer removed; scope grew to fix a stale `Hub75Client::open(&p)` call left over from the libusb migration in `3cbc886` (was blocking all builds of `life`). Verified on hardware against the 128x64 panel.

Follow-ups surfaced — to be drafted as separate changes:

- `clock.rs` carries the same stale `open(&p)` call.
- Running examples while `sysmon` holds the USB interface fails with `Busy`; worth a docs note or a friendlier error.

## Log

- Pre-existing build failure on `main`: `life.rs:39` called `Hub75Client::open(&p)`, but the library only exposes `open_auto` (since `3cbc886`, when the lib moved to libusb-based device discovery). User approved fixing it as part of this change.
- `clock.rs` has the same stale call — out of scope here, will need its own change.
