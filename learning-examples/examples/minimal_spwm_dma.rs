//! # DP3364S S-PWM HUB75 Driver — PIO + DMA Data Loader
//!
//! Second-stage implementation of the DP3364S driver. Builds directly on
//! `minimal_spwm_cpu.rs` by offloading the hot path (the 512 DATA_LATCH
//! operations per frame) to a PIO state machine fed by DMA.
//!
//! ## What changed vs. `minimal_spwm_cpu`
//!
//! In the CPU version, `data_transfer_simple()` performs on the order of
//! 65,000 SIO writes per frame: 512 latches × 128 pixel-clocks, each
//! pixel doing two GPIO writes (set/clr data, pulse CLK). That is by
//! far the largest single consumer of CPU time in the refresh loop.
//!
//! This example replaces that inner loop with a short PIO program —
//! six instructions, no branches except the per-pixel decrement — and
//! streams the whole 64 KB frame buffer straight to the state machine
//! via DMA. The CPU is idle during the transfer.
//!
//! ## Scope: DATA_LATCH only
//!
//! The S-PWM protocol has four commands distinguished by LAT pulse
//! width (see `minimal_spwm_cpu.rs` for the full table). This
//! implementation handles only DATA_LATCH (1-CLK LAT) in hardware.
//! The three other commands (VSYNC, PRE_ACT, WR_CFG) have different
//! pulse widths and remain CPU-bitbanged, exactly as in the CPU
//! version. Unifying all four commands under PIO requires a more
//! flexible program with a run-time-configurable LAT position — the
//! subject of the next example.
//!
//! ## Pin-function handover
//!
//! The S-PWM protocol multiplexes four commands onto the same CLK,
//! LAT and RGB lines. Since PIO (for DMA) and SIO (for CPU bitbang)
//! cannot drive a pin simultaneously, pins 0–5, 11 and 12 are
//! function-switched twice per frame:
//!
//! ```text
//!   VSYNC + PRE_ACT + WR_CFG   → FUNCSEL = SIO   (CPU bitbang)
//!   DATA_LATCH × 512           → FUNCSEL = PIO0  (DMA stream)
//!   display_loop (CLK + ROW)   → FUNCSEL = SIO   (CPU bitbang)
//! ```
//!
//! Each switch is one 32-bit write per pin to `IO_BANK0_GPIOn_CTRL`.
//! At 60 fps that's ~1 kHz of register writes — negligible compared
//! to the CPU cycles saved on the DATA_LATCH loop.
//!
//! To avoid glitches at the handover, the CPU drives CLK, LAT and
//! the six RGB lines LOW via SIO immediately before flipping funcsel
//! to PIO0. The PIO program is stalled at `out pins, 6 side 0b00`
//! (CLK = LAT = 0) whenever its FIFO is empty, so after the switch
//! the pins simply remain low until DMA starts feeding data.
//!
//! ## PIO program
//!
//! Six instructions. Side-set is 2 bits on GPIO 11–12 (CLK, LAT):
//!
//! ```text
//! .side_set 2
//!     out  y, 32        side 0b00   ; one-time: Y <- pixel-count-1
//! .wrap_target
//!     mov  x, y         side 0b00   ; reload per-latch counter
//! loop:
//!     out  pins, 6      side 0b00   ; CLK low, LAT low, 6 data bits out
//!     out  null, 2      side 0b01   ; CLK high — rising edge shifts bit in
//!     jmp  x--, loop    side 0b00   ; 127 iterations, CLK back to low
//!     out  pins, 6      side 0b10   ; CLK low, LAT high (command framing)
//!     out  null, 2      side 0b11   ; CLK high + LAT high → DATA_LATCH
//! .wrap                              ; next `mov x, y` drops LAT and CLK
//! ```
//!
//! Y is pre-loaded once at startup (CHAIN_BITS − 2 = 126). It persists
//! across `.wrap`, so every latch reuses the same count.
//!
//! ## Data buffer layout
//!
//! Autopull = 32 bits, shift direction right (LSB first). Each 32-bit
//! FIFO word carries four pixel-clocks worth of GPIO state; each
//! "pixel" is 6 RGB bits (GPIO 0–5) followed by 2 bits of padding that
//! the PIO discards:
//!
//! ```text
//!   bits  [5:0]    pixel k     (R0 | G0 | B0 | R1 | G1 | B1)
//!   bits  [7:6]    padding
//!   bits [13:8]    pixel k+1
//!   bits [15:14]   padding
//!   bits [21:16]   pixel k+2
//!   bits [23:22]   padding
//!   bits [29:24]   pixel k+3
//!   bits [31:30]   padding
//! ```
//!
//! One latch = 128 pixels = 32 words. One frame = 512 latches = 16 384
//! words = 64 KB, held in a single `static mut` buffer.
//!
//! ## Run
//!
//! ```sh
//! cargo run --release --example minimal_spwm_dma
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use rp235x_hal as hal;

use hal::dma::{single_buffer, DMAExt};
use hal::pio::PIOExt;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

// ── HUB75 pin mapping (Interstate 75 W) ──────────────────────────────
const R0: u32 = 1 << 0;
const G0: u32 = 1 << 1;
const B0: u32 = 1 << 2;
const R1: u32 = 1 << 3;
const G1: u32 = 1 << 4;
const B1: u32 = 1 << 5;
const ALL_RGB: u32 = R0 | G0 | B0 | R1 | G1 | B1;
const ADDR_MASK: u32 = 0x1F << 6;
const CLK: u32 = 1 << 11;
const LAT: u32 = 1 << 12;
const OE: u32 = 1 << 13;
const ALL_HUB75: u32 = ALL_RGB | ADDR_MASK | CLK | LAT | OE;

// ── Panel geometry (64×128, 1/32 scan) ───────────────────────────────
const PANEL_WIDTH: usize = 128;
const SCAN_LINES: usize = 32;
const CHIPS_PER_CHAIN: usize = PANEL_WIDTH / 16;
const CHAIN_BITS: usize = CHIPS_PER_CHAIN * 16; // 128 bits shifted per latch
const ROW_PERIOD: usize = 128;

// ── DP3264S / DP3364S configuration registers ────────────────────────
//
// See `minimal_spwm_cpu.rs` for a per-register explanation. One is
// written per frame via PRE_ACT + WR_CFG; round-robin through all 13.
const CONFIG_REGS: [u16; 13] = [
    0x1100, 0x021F, 0x033F, 0x043F, 0x0504, 0x0642, 0x0700,
    0x08BF, 0x0960, 0x0ABE, 0x0B8B, 0x0C88, 0x0D12,
];

// ── DMA frame buffer ─────────────────────────────────────────────────
const WORDS_PER_LATCH: usize = CHAIN_BITS / 4; // 32 (4 pixels per u32)
const LATCHES_PER_FRAME: usize = SCAN_LINES * 16; // 512
const FRAME_WORDS: usize = WORDS_PER_LATCH * LATCHES_PER_FRAME; // 16 384

static mut FRAME_BUF: [u32; FRAME_WORDS] = [0u32; FRAME_WORDS];

/// Pack a solid-colour frame into FRAME_BUF.
///
/// All 512 latches receive the same 16-bit greyscale word, with
/// `color_mask` selecting which of the six RGB lines are driven.
/// Every chip in every scan line gets the same pattern — so the panel
/// displays one uniform colour at one brightness.
fn fill_frame(brightness: u16, color_mask: u32) {
    let mask6 = color_mask & ALL_RGB;

    // Build one latch once, then splat it into all 512 slots.
    let mut latch = [0u32; WORDS_PER_LATCH];
    for pix in 0..CHAIN_BITS {
        // Bit 15 of `brightness` is sent first (MSB-first per chip).
        let bit_pos = 15 - (pix % 16);
        let data = if (brightness >> bit_pos) & 1 == 1 { mask6 } else { 0 };
        let w = pix / 4;
        let slot = pix % 4;
        latch[w] |= data << (slot * 8);
    }

    let buf = unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF) };
    for l in 0..LATCHES_PER_FRAME {
        let base = l * WORDS_PER_LATCH;
        buf[base..base + WORDS_PER_LATCH].copy_from_slice(&latch);
    }
}

// ── Pin-function switching ───────────────────────────────────────────
//
// FUNCSEL values per the RP2350 GPIO function table:
//   5 = SIO, 6 = PIO0. A plain 32-bit write of the funcsel to
//   IO_BANK0_GPIOn_CTRL clears all OUTOVER/INOVER/OEOVER/IRQOVER
//   overrides at the same time — exactly what we want.
const IO_BANK0_BASE: u32 = 0x4002_8000;
const FUNC_SIO: u32 = 5;
const FUNC_PIO0: u32 = 6;
const SHARED_PINS: [u32; 8] = [0, 1, 2, 3, 4, 5, 11, 12];

#[inline(always)]
fn set_pin_func(gpio: u32, funcsel: u32) {
    let addr = IO_BANK0_BASE + 0x04 + gpio * 8;
    unsafe { (addr as *mut u32).write_volatile(funcsel) };
}

fn pins_to_sio() {
    for &p in &SHARED_PINS { set_pin_func(p, FUNC_SIO); }
}

fn pins_to_pio0() {
    for &p in &SHARED_PINS { set_pin_func(p, FUNC_PIO0); }
}

// ── SIO bitbang helpers (identical to minimal_spwm_cpu) ──────────────
type Sio = hal::pac::sio::RegisterBlock;

#[inline(always)]
fn clock_pulse(sio: &Sio) {
    sio.gpio_out_set().write(|w| unsafe { w.bits(CLK) });
    cortex_m::asm::nop();
    sio.gpio_out_clr().write(|w| unsafe { w.bits(CLK) });
}

fn send_clocks(sio: &Sio, n: usize) {
    sio.gpio_out_clr().write(|w| unsafe { w.bits(ALL_RGB) });
    for _ in 0..n { clock_pulse(sio); }
}

fn send_latches(sio: &Sio, width: usize) {
    sio.gpio_out_clr().write(|w| unsafe { w.bits(ALL_RGB) });
    sio.gpio_out_set().write(|w| unsafe { w.bits(LAT) });
    for _ in 0..width { clock_pulse(sio); }
    sio.gpio_out_clr().write(|w| unsafe { w.bits(LAT) });
}

fn shift_all_with_lat(sio: &Sio, word: u16, lat_width: usize) {
    let lat_start = CHAIN_BITS - lat_width;
    for i in 0..CHAIN_BITS {
        let bit_pos = 15 - (i % 16);
        if i == lat_start {
            sio.gpio_out_set().write(|w| unsafe { w.bits(LAT) });
        }
        if (word >> bit_pos) & 1 == 1 {
            sio.gpio_out_set().write(|w| unsafe { w.bits(ALL_RGB) });
        } else {
            sio.gpio_out_clr().write(|w| unsafe { w.bits(ALL_RGB) });
        }
        clock_pulse(sio);
    }
    sio.gpio_out_clr().write(|w| unsafe { w.bits(LAT | ALL_RGB) });
}

fn vsync(sio: &Sio) {
    sio.gpio_out_clr().write(|w| unsafe { w.bits(ALL_RGB | CLK) });
    clock_pulse(sio);
    send_latches(sio, 3);
}

fn pre_act(sio: &Sio) { send_latches(sio, 14); }

fn wr_cfg(sio: &Sio, reg_data: u16) { shift_all_with_lat(sio, reg_data, 5); }

fn set_addr(sio: &Sio, row: usize) {
    sio.gpio_out_clr().write(|w| unsafe { w.bits(ADDR_MASK) });
    sio.gpio_out_set().write(|w| unsafe { w.bits((row as u32) << 6) });
}

fn row_pulse(sio: &Sio, width: usize) {
    sio.gpio_out_set().write(|w| unsafe { w.bits(OE) });
    for _ in 0..width { clock_pulse(sio); }
    sio.gpio_out_clr().write(|w| unsafe { w.bits(OE) });
}

fn display_loop(sio: &Sio, cycles: usize) {
    let pre_addr = ROW_PERIOD - 28;
    let mut row: usize = 0;
    for _ in 0..cycles * SCAN_LINES {
        send_clocks(sio, pre_addr);
        set_addr(sio, row);
        send_clocks(sio, 28);
        let w = if row == 0 { 12 } else { 4 };
        row_pulse(sio, w);
        row += 1;
        if row >= SCAN_LINES { row = 0; }
    }
}

// ── Entry point ──────────────────────────────────────────────────────

#[hal::entry]
fn main() -> ! {
    // ── 1. Boot ──────────────────────────────────────────────────────
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let _clocks = hal::clocks::init_clocks_and_plls(
        12_000_000, pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB,
        &mut pac.RESETS, &mut watchdog,
    ).unwrap();

    // ── 2. Pins ──────────────────────────────────────────────────────
    //
    // All 14 HUB75 pins start as SIO push-pull outputs; the eight
    // shared ones (0–5, 11, 12) get re-routed to PIO0 around the DMA
    // burst and back again via set_pin_func().
    let sio_hal = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0, pac.PADS_BANK0, sio_hal.gpio_bank0, &mut pac.RESETS,
    );
    let _pins = (
        pins.gpio0.into_push_pull_output(),  pins.gpio1.into_push_pull_output(),
        pins.gpio2.into_push_pull_output(),  pins.gpio3.into_push_pull_output(),
        pins.gpio4.into_push_pull_output(),  pins.gpio5.into_push_pull_output(),
        pins.gpio6.into_push_pull_output(),  pins.gpio7.into_push_pull_output(),
        pins.gpio8.into_push_pull_output(),  pins.gpio9.into_push_pull_output(),
        pins.gpio10.into_push_pull_output(), pins.gpio11.into_push_pull_output(),
        pins.gpio12.into_push_pull_output(), pins.gpio13.into_push_pull_output(),
    );
    let sio = unsafe { &(*hal::pac::SIO::ptr()) };
    sio.gpio_out_clr().write(|w| unsafe { w.bits(ALL_HUB75) });

    // ── 3. PIO program (six instructions) ────────────────────────────
    let ss = pio::SideSet::new(false, 2, false);
    let mut a = pio::Assembler::new_with_side_set(ss);
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    let mut pixel_loop = a.label();

    // Preamble (runs once): Y <- pixel-count-minus-one
    a.out_with_side_set(pio::OutDestination::Y, 32, 0b00);

    a.bind(&mut wrap_target);
    // Each latch: reload per-latch counter from Y
    a.mov_with_side_set(
        pio::MovDestination::X, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );

    a.bind(&mut pixel_loop);
    // 127 normal pixels
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0b00);
    a.out_with_side_set(pio::OutDestination::NULL, 2, 0b01);
    a.jmp_with_side_set(
        pio::JmpCondition::XDecNonZero, &mut pixel_loop, 0b00,
    );

    // 1 final pixel with LAT asserted → DATA_LATCH command
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0b10);
    a.out_with_side_set(pio::OutDestination::NULL, 2, 0b11);
    a.bind(&mut wrap_source);

    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio0.install(&program).unwrap();

    let (mut sm, _, mut tx) = hal::pio::PIOBuilder::from_installed_program(installed)
        .buffers(hal::pio::Buffers::OnlyTx)
        .out_pins(0, 6)                  // RGB on GPIO 0–5
        .side_set_pin_base(11)           // sideset bit 0 = CLK, bit 1 = LAT
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .autopull(true)
        .pull_threshold(32)
        .clock_divisor_fixed_point(2, 0) // PIO clock = sys / 2 = 75 MHz
        .build(sm0);

    sm.set_pindirs([
        (0,  hal::pio::PinDir::Output), (1,  hal::pio::PinDir::Output),
        (2,  hal::pio::PinDir::Output), (3,  hal::pio::PinDir::Output),
        (4,  hal::pio::PinDir::Output), (5,  hal::pio::PinDir::Output),
        (11, hal::pio::PinDir::Output), (12, hal::pio::PinDir::Output),
    ]);

    // Pre-load Y = CHAIN_BITS - 2 = 126. The pixel loop runs 127 times
    // and the two fall-through OUTs produce the 128th (LAT-asserted)
    // pixel. Y persists across .wrap so this is a one-shot.
    tx.write((CHAIN_BITS - 2) as u32);
    let _sm = sm.start();

    // ── 4. DMA ───────────────────────────────────────────────────────
    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut dma_ch = dma.ch0;
    let mut tx = tx;

    defmt::info!("DP3364S S-PWM (PIO+DMA) starting");

    // ── 5. Main loop ─────────────────────────────────────────────────
    let tests: [(u16, u32, &str); 6] = [
        (0x3FFF, ALL_RGB,  "bright white"),
        (0x00FF, ALL_RGB,  "dim white"),
        (0x0000, ALL_RGB,  "black"),
        (0x3FFF, R0 | R1,  "red"),
        (0x3FFF, G0 | G1,  "green"),
        (0x3FFF, B0 | B1,  "blue"),
    ];

    let mut test_idx: usize = 0;
    let mut reg_idx: usize = 0;

    loop {
        let (brightness, color_mask, name) = tests[test_idx % tests.len()];
        defmt::info!("{}", name);
        fill_frame(brightness, color_mask);

        for _ in 0..300 {
            // ── VSYNC + one config register (CPU bitbang via SIO) ──
            pins_to_sio();
            vsync(sio);
            send_clocks(sio, 16);
            pre_act(sio);
            send_clocks(sio, 8);
            wr_cfg(sio, CONFIG_REGS[reg_idx]);
            send_clocks(sio, 8);
            reg_idx = (reg_idx + 1) % CONFIG_REGS.len();

            // Leave all shared lines low before handing them to PIO
            // so the function switch can't glitch CLK/LAT.
            sio.gpio_out_clr().write(|w| unsafe {
                w.bits(ALL_RGB | CLK | LAT)
            });

            // ── DATA_LATCH × 512 (PIO + DMA) ───────────────────────
            pins_to_pio0();
            let buf = unsafe { &*core::ptr::addr_of!(FRAME_BUF) };
            let cfg = single_buffer::Config::new(dma_ch, buf, tx);
            let xfer = cfg.start();
            let (ch, _, returned_tx) = xfer.wait();
            dma_ch = ch;
            tx = returned_tx;
            // Wait for the PIO FIFO to drain so the last latch has
            // actually been clocked out before we switch pin function.
            while !tx.is_empty() { cortex_m::asm::nop(); }
            cortex_m::asm::delay(50);

            // ── Display (CLK feeds PLL, ROW advances scan) ─────────
            pins_to_sio();
            display_loop(sio, 10);
        }

        test_idx += 1;
    }
}
