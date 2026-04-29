# SPWM Embassy Port

## Intent

The `spwm_3_autonomous` example is the minimum-viable architecture for driving DP3364S panels — 128 Hz refresh, 17% dark gap, core 0 free. It works and the architecture is settled.

It lives as a bare-metal `rp235x-hal` example. The display application that will use it is built on `embassy-rp`. To consume the architecture from an application, it needs to become a library on the application's HAL.

## Approach

### New workspace member; `learning-examples` unchanged

The port produces a new workspace member crate at `./hub75/` (alongside `learning-examples` and `usb-display`). The `spwm_3_autonomous` example stays as the canonical reference architecture — the source we port from, not something we abandon.

### Public surface: `Panel` trait + concrete `Dp3364sPanel`

A single trait `Panel` carries only what is portable across HUB75 panel families: geometry (width/height as associated constants) and a `frame_mut()` / `commit()` pair. The DP3364S-specific concrete type owns its own driver state and exposes inherent methods for the things only it has — `set_scan_cycles`, brightness — matching the established constraint that brightness is panel-specific.

### Panel owns the RGB buffer

The panel holds the RGB pixel buffer internally. Caller obtains `&mut [[Rgb; W]; H]` via `frame_mut()`, writes pixels in place, then calls `commit()` to release the borrow and trigger the pack on core 1. `frame_mut()` blocks until the prior pack completes, so the borrow is always exclusive of the packer. One 24 KB RGB buffer in RAM, no per-frame memcpy.

### Application owns peripherals; panel takes them by value

Pins (GPIO 0..13), PIO0, two DMA channels, and core 1 are passed to the panel constructor. Embassy idiom: the application owns peripherals and hands them in. The crate does not steal or assume ownership of board-wide resources.

### Packing stays on core 1

The architecture's premise is that core 0 is ~99% free for application work. Moving the pack onto core 0 as an Embassy task competes with USB and other application I/O. Core 1 is otherwise idle. The panel constructor takes core 1 by value.

### Raw-register access for what embassy-rp doesn't expose

DMA ring-mode, PIO INSTR/PINCTRL/EXECCTRL phase swaps, and the DMA IRQ handler all need register-level access. The crate uses embassy-rp's PAC where the higher-level abstractions don't fit — same pattern `usb-display/firmware` already follows.

### Tunables exposed as inherent setters

`Dp3364sPanel::set_scan_cycles(u32)` is an inherent method, applied at the next frame boundary. The constructor takes no scan-cycles argument; the default matches `spwm_3_autonomous`'s current operating point (20 cycles → ~128 Hz / ~34 % dark, the refresh-rate sweet spot).

### Smoke-test example: solid colour cycle, then scrolling rainbow

A single example: solid R / G / B / W fills, three seconds each, followed by a scrolling diagonal rainbow. Validates `frame_mut` / `commit` plus animated content, end to end.

## Plan

- [x] Scaffold the crate: add `hub75/` to root `Cargo.toml` workspace members; create `hub75/{Cargo.toml (version 0.1.0, embassy-rp deps mirroring usb-display/firmware), build.rs, memory.x, .cargo/config.toml, src/lib.rs}`.
- [x] Add public types: `Rgb`, gamma LUT, `Panel` trait (associated `WIDTH`/`HEIGHT`, `frame_mut`, `commit`), and `Dp3364sPanel` struct + `new()` signature consuming the required peripherals plus an embassy `Spawner` and a core-1 stack reference.
- [x] Port the pack pipeline (frame-buffer layout constants, `CONFIG_REGS`, `init_frame_headers`, `pack_pixels` chip-major scatter). Pure data, no peripherals.
- [x] Port the PIO program (two-entry assembly: `data_cmd` / `scan_entry`) onto embassy-rp's `Pio` API; raw-register access for the `INSTR` / `EXECCTRL` / `PINCTRL` phase swaps.
- [x] Port DMA: ch0 one-shot data, ch1 ring-mode scan, `DMA_IRQ_0` handler bridged into an `embassy_sync::signal::Signal` for async wait.
- [x] Spawn the pack worker on core 1 via `embassy_rp::multicore::spawn_core1` from inside `new()`; SIO-FIFO or atomic-based handshake for buffer-ready signalling.
- [x] Spawn the scan loop on core 0 as an embassy task: boot flush (write `CONFIG_REGS`) → ~1 s scan-pointer sync → main loop (data phase, scan phase, buffer swap on core-1-done).
- [x] Implement `Panel::frame_mut` (awaits prior pack done) and `Panel::commit` (kicks pack on core 1); add inherent `Dp3364sPanel::set_scan_cycles(u32)` applied at the next frame boundary.
- [x] Add the smoke-test example `hub75/examples/rainbow.rs`: solid R / G / B / W fills (3 s each) followed by a scrolling diagonal rainbow.
- [x] Verify on hardware: `cargo build --release --example rainbow` succeeds, then flash and visually confirm correct panel output.

## Log

- DMA_IRQ_0 wired through embassy's `bind_interrupts!` pattern rather than a raw `#[cortex_m_rt::interrupt]` handler. The crate exposes `DmaIrqHandler` and the application binds `DMA_IRQ_0 => hub75::DmaIrqHandler;` (alongside the `PIO0_IRQ_0` binding `Pio::new` already requires). Both bindings are passed to `Dp3364sPanel::new` as a single `impl Copy + Binding<...> + Binding<...>` parameter — the unit-struct generated by `bind_interrupts!` is `Copy`, so one `Irqs` instance flows to both `Pio::new` and the internal `DmaEngine::new`.
- Initially scaffolded the crate at `hub75/hub75/` (three levels of `hub75` from filesystem root). Confused the "tokio/tokio" idiom — there the outer is the git repo and the inner is the crate, so the analog here is `~/CODE/hub75/` (the git repo) + `hub75/` (the crate). One level inside the repo, not two. Moved everything up to `hub75/` and updated the workspace `members` entry.
- First hardware run: scan loop iterated correctly but the panel stayed dark (FRAME_BUF[0] kept its all-zero pixel data because no pack ever completed). Cause: `embassy_rp::multicore::spawn_core1` enables `SIO_IRQ_FIFO` on core 1 for its pause functionality. With no handler bound, the first FIFO write from core 0 fires an unhandled IRQ that re-fires forever, wedging core 1 in the cortex-m-rt default handler. The reference architecture used raw multicore (no embassy) and didn't enable that IRQ. Fix: `cortex_m::peripheral::NVIC::mask(pac::Interrupt::SIO_IRQ_FIFO)` at the top of `pack_worker_loop`. WFE in the FIFO read loop still wakes on the SEV core 0 sends after each fifo_write, so we don't lose the wake-up path.

## Conclusion

Completed as planned. The `hub75` crate at `./hub75/` exposes `Dp3364sPanel` with `frame_mut`/`commit`, an inherent `set_scan_cycles`, and the `Panel` trait. Smoke-test example renders solid R/G/B/W (3 s each) followed by a scrolling rainbow on hardware. Reference `learning-examples/spwm_3_autonomous` remains as the canonical pre-port architecture.

Project does not maintain a changelog; no entry proposed.
