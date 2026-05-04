# Remove RGB primer from the `life` example

## Intent

The Rust `life` example currently runs a 2-second yellow → green "RGB sanity check" before starting Game of Life. We added it earlier as an ad-hoc channel-order diagnostic, but the project now has a proper [`test_pattern`](../hub75/examples/test_pattern.rs) example for that. The primer in `life` is redundant clutter at the start of every run; remove it so `life` jumps straight into the simulation.

## Approach

Delete the primer block (the `yellow` and `green` array literals, the two `send_frame_rgb` + `sleep` calls, and the `println!` announcing it). Nothing depends on it; nothing else needs to change.

## Plan

- [ ] Remove the RGB sanity-check block from `usb-display/client/rust/examples/life.rs`.
- [ ] Verify `life` still builds for each panel size feature.
