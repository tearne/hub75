# Shift-register panel

## Intent

The `hub75` crate currently supports only one panel family — DP3364S, which has on-chip SRAM and PWM. The other family in widespread use is the generic shift-register HUB75 panel (74HC595-style column drivers; the host computes BCM timing and clocks bits out per bitplane). The `usb-display/firmware` already drives one of these directly; that work hasn't been factored into the crate.

To use these panels through `hub75`, a second concrete panel type is needed alongside `Dp3364sPanel`. Two physical sizes must be supported at minimum: 64×64 and 64×32. Adding the second family also serves as the first real test of the `Panel` trait abstraction — until now there's been only one implementor.

`usb-display/firmware` already implements this panel family inline (its `display.rs` has the pixel pack and BCM timing; its `main.rs` has the PIO + DMA setup). Once the new panel type lands in `hub75`, that firmware should be refactored to consume it, removing the duplicate implementation.

## Approach

### Separate module from `dp3364s`; no internals shared

The shift-register architecture differs fundamentally from DP3364S: two PIO state machines (data + address with IRQ handshake) instead of one unified SM, no on-chip SRAM, host-side BCM timing instead of chip-side PWM, no boot flush / sync phase. The new panel lives in `src/shift/` (struct: `ShiftPanel`) and shares only `Rgb`, the gamma LUT (probably), and the `Panel` trait with `dp3364s/`. Trying to share lower-level pieces would just couple two unrelated drivers.

### Const-generic dimensions

`<const W: usize, const H: usize>` on the panel struct so that 64×64 and 64×32 are different concrete types instantiated by the caller. Address-pin count is a function of `H` (scan-lines = H/2 → 5 address bits when H ≥ 64, 4 when H = 32). The PIO program writes 5 bits regardless; for 1/16 scan the top bit is always 0, which is harmless.

### Pack runs synchronously in `commit()`

A 64×64 BCM pack is small (~8 K u32 words, ≪ 100 µs at 150 MHz). Running it on the calling task is fast enough that core 1 isn't needed. This means the constructor takes no `Spawner` and no `CORE1` / stack — strictly fewer parameters than `Dp3364sPanel::new`. For larger / heavier shift-register panels that emerge later, core-1 packing can be added then.

### Steady-state scan is fully autonomous

Like `usb-display/firmware`'s current setup: four DMA channels (data + timing chains, each a 2-channel self-feeding pair) run indefinitely once started. No DMA IRQ in steady state, no scan-loop task. The constructor sets up the chains and the PIO state machines, then returns. After that the only CPU work is pack-on-commit.

### Reuse `InterstatePins`

Same 14 GPIOs as `Dp3364sPanel`. The pin struct is already defined; share it.

### Migrate `usb-display/firmware` in this change

The firmware's `display.rs` (pixel pack + BCM timing) and the inline PIO/DMA setup in `main.rs` are exactly what `ShiftPanel` will provide. Bundling the migration means no duplicate code lands at change-end. The firmware's USB protocol bits (`FrameReceiver`, `ReceiveBuffer`) are application-specific and stay where they are.

### README catch-up

`hub75/README.md` currently only mentions DP3364S. As part of this change, update it to explain the two panel families (DP3364S and shift-register), why the crate names them as it does (`Dp3364sPanel`, `ShiftPanel`), and which physical panels each one drives.

## Plan

- [x] Scaffold `src/shift/` with submodules (`pack`, `pio`, `dma`, `timing`) and add `pub mod shift;` to `lib.rs`. Re-export `ShiftPanel` and panel-specific public types.
- [x] Add public types: `ShiftPanel<const W: usize, const H: usize>` struct + `new()` signature consuming PIO0, `InterstatePins`, four DMA channel peripherals, and a PIO0_IRQ_0 binding.
- [x] Port the pack pipeline (8-bit→8-bit gamma LUT, RGB → bitplane DMA-buffer pack).
- [x] Port the PIO program (Data SM + Address SM with IRQ handshake; BCM timing words).
- [x] Port the DMA chain (4 channels: data buf → SM0 TX with reload, timing buf → SM1 TX with reload — both autonomous).
- [x] Wire `ShiftPanel::new` (configure pins, install PIO programs, start DMA chain, return). Implement `Panel::frame_mut` / `commit` (commit packs synchronously into the spare buffer and updates the active-buffer pointer for the data DMA chain).
- [x] Add example `hub75/examples/shift_rainbow.rs` — 64×64, R/G/B/W solids (3 s each) followed by scrolling rainbow, modelled on the existing `rainbow.rs`.
- [x] Rename `hub75/examples/rainbow.rs` → `hub75/examples/spwm_rainbow.rs` for symmetry with `shift_rainbow.rs` and the `learning-examples/spwm_*` convention; update `hub75/README.md` references accordingly.
- [x] Add a "without a debug probe" section to `learning-examples/README.md` (BOOTSEL + picotool, mirroring the existing block in the project root README).
- [x] Migrate `usb-display/firmware`: drop the panel-specific contents of `display.rs` (`DmaBuffer`, `generate_timing_buffer`, `estimate_refresh_rate`, `OE` constant), drop the inline PIO + DMA setup from `main.rs`, instantiate `ShiftPanel<64, 64>` in main, and feed received frames through `frame_mut`/`commit`. Keep `Rgb` (use `hub75::Rgb`), `FrameReceiver`, and `ReceiveBuffer` — those are USB-protocol bits.
- [x] Update `hub75/README.md` to introduce both panel families (DP3364S vs shift-register), explain the naming (`Dp3364sPanel` vs `ShiftPanel`), and add a one-liner on which physical panels each one drives.
- [x] Bump `hub75` crate version to `0.2.0` (minor — new public panel type added).
- [x] Verify `ShiftPanel` example on hardware: `cargo run --release --example shift_rainbow` from `hub75/`, confirm correct output on a 64×64 panel. (Verified on a 64×32 panel after the example was switched to `<64, 32>`.)
- [x] Verify migrated `usb-display/firmware` on hardware: `cargo run --release` from `usb-display/firmware`, confirm USB-driven display still works. (Verified on a 64×32 panel; minor glitches due to panel-size mismatch with the hardcoded `<64, 64>` in firmware — addressed by a follow-up change.)

## Conclusion

Completed as planned, plus two scope additions during build (renamed `rainbow.rs` → `spwm_rainbow.rs` for symmetry with `shift_rainbow.rs` and the `learning-examples/spwm_*` convention; added a probe-less flashing section to `learning-examples/README.md`). `hub75` now exposes both panel families through the `Panel` trait; `usb-display/firmware` consumes `ShiftPanel<64, 64>` and the inline PIO/DMA setup is gone. `Dp3364sPanel` remains chip-specific; `ShiftPanel` is const-generic over W/H.

Project does not maintain a changelog; no entry proposed.

