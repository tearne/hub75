//! Minimal HUB75 example using PIO+DMA on the Interstate 75 W (RP2350A).
//!
//! Cycles through solid colors on a 64x64 panel.
//! Demonstrates offloading pixel data clocking from CPU to PIO+DMA.
//! Read top to bottom.
//!
//! Run: cargo run --release --example minimal_dma
//!
//! ## How it works
//!
//! The HUB75 scan loop has two parts:
//!
//!   1. **Inner loop** (per pixel): set 6 RGB data bits, pulse CLK.
//!      → Handled by PIO state machine + DMA. Zero CPU involvement.
//!
//!   2. **Outer loop** (per row): blank display, set row address, latch, enable.
//!      → Handled by CPU via direct GPIO writes.
//!
//! Data flow:
//!
//!   ┌──────────────┐      DMA       ┌─────┐  GPIO 0-5  ┌───────┐
//!   │ Display buf  │ ──────────────→│ PIO │────────────→│ Panel │
//!   │ (RAM array)  │   (automatic)  │ SM  │  + CLK(11) │       │
//!   └──────────────┘                └─────┘             └───────┘
//!         ↑                                                 ↑
//!   CPU: convert                               CPU: address, latch, OE
//!   framebuffer                                 (GPIOs 6-13)
//!   to packed bits

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use rp235x_hal as hal;
use hal::dma::{single_buffer, DMAExt};
use hal::gpio::FunctionPio0;
use hal::pio::PIOExt;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

// ── Panel constants ──────────────────────────────────────────────────

const WIDTH: usize = 64;
const HEIGHT: usize = 64;
const ROW_PAIRS: usize = HEIGHT / 2; // 32 row pairs for 64-row panel

// ── GPIO pin masks (CPU-controlled pins only) ────────────────────────
//
// PIO controls: GPIO 0-5 (RGB data) and GPIO 11 (CLK)
// CPU controls: GPIO 6-10 (row address), GPIO 12 (LAT), GPIO 13 (OE)

const ADDR_MASK: u32 = 0x1F << 6; // bits 6-10
const LAT: u32 = 1 << 12;
const OE: u32 = 1 << 13;

// RGB data bit positions (used when building the display buffer)
const R0: u32 = 1 << 0;
const G0: u32 = 1 << 1;
const B0: u32 = 1 << 2;
const R1: u32 = 1 << 3;
const G1: u32 = 1 << 4;
const B1: u32 = 1 << 5;
const ALL_RGB: u32 = R0 | G0 | B0 | R1 | G1 | B1;

// ── Display buffer ───────────────────────────────────────────────────
//
// Pre-packed GPIO data for PIO+DMA. Each u32 holds 4 pixels:
//
//   bits  0-5:  pixel 0 (R0|G0|B0|R1|G1|B1)
//   bits  6-7:  padding (discarded by PIO)
//   bits  8-13: pixel 1
//   bits 14-15: padding
//   bits 16-21: pixel 2
//   bits 22-23: padding
//   bits 24-29: pixel 3
//   bits 30-31: padding
//
// PIO shifts out 6 data bits, then discards 2 padding bits per pixel.
// Autopull at 32 bits refills the OSR after every 4 pixels.

const WORDS_PER_ROW: usize = WIDTH / 4; // 16 words per row

// One row of packed pixel data for DMA.
static mut ROW_BUF: [u32; WORDS_PER_ROW] = [0u32; WORDS_PER_ROW];

/// Pack a solid color into the row buffer.
/// `color` is 6 bits: R0|G0|B0|R1|G1|B1 (same color on both halves).
fn fill_row_buf(color: u32) {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(ROW_BUF) };
    // Pack the same color into all 4 pixel slots of each word
    let word = (color & 0x3F)
        | ((color & 0x3F) << 8)
        | ((color & 0x3F) << 16)
        | ((color & 0x3F) << 24);
    for w in buf.iter_mut() {
        *w = word;
    }
}

// ── Entry point ──────────────────────────────────────────────────────

#[hal::entry]
fn main() -> ! {
    // ── 1. Boot the chip ─────────────────────────────────────────────
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let _clocks = hal::clocks::init_clocks_and_plls(
        12_000_000,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    // ── 2. Configure GPIO pins ───────────────────────────────────────
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Data pins (0-5) and CLK (11) → PIO function
    // PIO will drive these automatically.
    let _data_pins = (
        pins.gpio0.into_function::<FunctionPio0>(),
        pins.gpio1.into_function::<FunctionPio0>(),
        pins.gpio2.into_function::<FunctionPio0>(),
        pins.gpio3.into_function::<FunctionPio0>(),
        pins.gpio4.into_function::<FunctionPio0>(),
        pins.gpio5.into_function::<FunctionPio0>(),
    );
    let _clk_pin = pins.gpio11.into_function::<FunctionPio0>();

    // Address (6-10), LAT (12), OE (13) → SIO outputs (CPU-controlled)
    let _ctrl_pins = (
        pins.gpio6.into_push_pull_output(),
        pins.gpio7.into_push_pull_output(),
        pins.gpio8.into_push_pull_output(),
        pins.gpio9.into_push_pull_output(),
        pins.gpio10.into_push_pull_output(),
        pins.gpio12.into_push_pull_output(),
        pins.gpio13.into_push_pull_output(),
    );

    // SIO registers for fast atomic GPIO control
    let sio_regs = unsafe { &(*hal::pac::SIO::ptr()) };

    // Start with OE disabled (high) and LAT low
    sio_regs.gpio_out_set().write(|w| unsafe { w.bits(OE) });
    sio_regs.gpio_out_clr().write(|w| unsafe { w.bits(LAT) });

    // ── 3. PIO program ──────────────────────────────────────────────
    //
    // Two instructions, executed in a loop:
    //
    //   out pins, 6    side 0    ; shift 6 RGB bits to GPIOs 0-5, CLK low
    //   out null, 2    side 1    ; discard 2 padding bits, CLK high (rising edge)
    //
    // The PIO state machine stalls when the TX FIFO is empty (waiting
    // for DMA to feed more data). When DMA is done, PIO stops on its own.

    let side_set = pio::SideSet::new(false, 1, false);
    let mut a = pio::Assembler::new_with_side_set(side_set);
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();

    a.bind(&mut wrap_target);
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0);
    a.out_with_delay_and_side_set(pio::OutDestination::NULL, 2, 0, 1);
    a.bind(&mut wrap_source);

    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    // ── 4. Install PIO program and configure state machine ──────────
    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio0.install(&program).unwrap();

    let (mut sm, _, tx) = hal::pio::PIOBuilder::from_installed_program(installed)
        .buffers(hal::pio::Buffers::OnlyTx)  // we only send data, never receive
        .out_pins(0, 6)                       // data output on GPIOs 0-5
        .side_set_pin_base(11)                // CLK on GPIO 11
        .out_shift_direction(hal::pio::ShiftDirection::Right) // LSB first
        .autopull(true)                       // auto-refill OSR from FIFO
        .pull_threshold(32)                   // refill after 32 bits (4 pixels)
        .clock_divisor_fixed_point(2, 0)      // PIO clock = sys_clk / 2
        .build(sm0);

    // Tell PIO that data + CLK pins are outputs
    sm.set_pindirs([
        (0, hal::pio::PinDir::Output),
        (1, hal::pio::PinDir::Output),
        (2, hal::pio::PinDir::Output),
        (3, hal::pio::PinDir::Output),
        (4, hal::pio::PinDir::Output),
        (5, hal::pio::PinDir::Output),
        (11, hal::pio::PinDir::Output),
    ]);

    // Start PIO — it immediately stalls waiting for FIFO data
    let _sm = sm.start();

    // ── 5. DMA setup ─────────────────────────────────────────────────
    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut dma_ch = dma.ch0;
    let mut tx = tx;

    defmt::info!("Minimal PIO+DMA HUB75 example");

    // ── 6. Color definitions ─────────────────────────────────────────
    // Each color sets both halves (R0+R1, G0+G1, B0+B1) to the same value.
    let colors: [(u32, &str); 7] = [
        (R0 | R1, "red"),
        (G0 | G1, "green"),
        (B0 | B1, "blue"),
        (R0 | R1 | G0 | G1, "yellow"),
        (R0 | R1 | B0 | B1, "magenta"),
        (G0 | G1 | B0 | B1, "cyan"),
        (ALL_RGB, "white"),
    ];
    let mut color_idx = 0;

    // ── 7. Main loop ─────────────────────────────────────────────────
    loop {
        let (color, name) = colors[color_idx % colors.len()];
        defmt::info!("Color: {}", name);

        // Pack this color into the row buffer (same for every row)
        fill_row_buf(color);

        // Show for ~1500 frames
        for _frame in 0..1500 {
            for row in 0..ROW_PAIRS as u32 {

                // ── a) Blank display while loading new data ──────
                sio_regs.gpio_out_set().write(|w| unsafe { w.bits(OE) });

                // ── b) DMA row data to PIO ───────────────────────
                // DMA reads from ROW_BUF and writes to PIO TX FIFO.
                // PIO automatically clocks out each pixel.
                // CPU is free during this transfer.
                let row_data = unsafe { &*core::ptr::addr_of!(ROW_BUF) };
                let config = single_buffer::Config::new(dma_ch, row_data, tx);
                let xfer = config.start();
                let (ch, _, returned_tx) = xfer.wait();
                dma_ch = ch;
                tx = returned_tx;

                // Wait for PIO to finish clocking out the last pixels
                while !tx.is_empty() { cortex_m::asm::nop(); }
                cortex_m::asm::delay(20);

                // ── c) Set row address (CPU) ─────────────────────
                sio_regs.gpio_out_clr().write(|w| unsafe { w.bits(ADDR_MASK) });
                sio_regs.gpio_out_set().write(|w| unsafe { w.bits(row << 6) });

                // ── d) Latch: transfer shift register → output ───
                sio_regs.gpio_out_set().write(|w| unsafe { w.bits(LAT) });
                cortex_m::asm::nop();
                sio_regs.gpio_out_clr().write(|w| unsafe { w.bits(LAT) });

                // ── e) Enable output (OE low = LEDs on) ──────────
                sio_regs.gpio_out_clr().write(|w| unsafe { w.bits(OE) });

                // ── f) Display time ──────────────────────────────
                cortex_m::asm::delay(3000);
            }
        }

        color_idx += 1;
    }
}
