//! Minimal HUB75 example for a 64x64 panel on the Interstate 75 W (RP2350A).
//!
//! Cycles through 7 solid colors to verify the panel and wiring work.
//! Intended as a learning reference — read top to bottom.
//!
//! Run: cargo run --release --bin minimal

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use rp235x_hal as hal;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

// ── HUB75 pin mapping (Interstate 75 W) ─────────────────────────────
//
// The HUB75 connector has 6 data pins, 5 address pins, and 3 control pins,
// mapped to consecutive GPIOs 0–13:
//
//   Data (accent color for two row halves):
//     GPIO 0 = R0   (red,   upper half)
//     GPIO 1 = G0   (green, upper half)
//     GPIO 2 = B0   (blue,  upper half)
//     GPIO 3 = R1   (red,   lower half)
//     GPIO 4 = G1   (green, lower half)
//     GPIO 5 = B1   (blue,  lower half)
//
//   Row address (selects which row pair is active):
//     GPIO 6–10 = A, B, C, D, E   (5 lines → 32 row pairs for 64-row panel)
//
//   Control:
//     GPIO 11 = CLK   (rising edge clocks one pixel of RGB data in)
//     GPIO 12 = LAT   (latch: transfers shift register → output drivers)
//     GPIO 13 = OE    (output enable, active LOW — low = LEDs on)
//
const R0: u32 = 1 << 0;
const G0: u32 = 1 << 1;
const B0: u32 = 1 << 2;
const R1: u32 = 1 << 3;
const G1: u32 = 1 << 4;
const B1: u32 = 1 << 5;
const ALL_RGB: u32 = R0 | G0 | B0 | R1 | G1 | B1;
const ADDR_MASK: u32 = 0x1F << 6; // bits 6–10
const CLK: u32 = 1 << 11;
const LAT: u32 = 1 << 12;
const OE: u32 = 1 << 13;
const ALL_HUB75: u32 = ALL_RGB | ADDR_MASK | CLK | LAT | OE;

const PANEL_WIDTH: usize = 64;
const ROW_PAIRS: usize = 32; // 64 rows ÷ 2 halves

// ── Entry point ──────────────────────────────────────────────────────

#[hal::entry]
fn main() -> ! {
    // ── 1. Boot the chip ─────────────────────────────────────────────
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let _clocks = hal::clocks::init_clocks_and_plls(
        12_000_000, // external crystal frequency
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    // ── 2. Configure GPIOs 0–13 as outputs ───────────────────────────
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // Each call sets the pin function to SIO and direction to output.
    // We don't use the HAL pin objects after this — we drive GPIOs
    // directly through the SIO registers for speed.
    let _pins = (
        pins.gpio0.into_push_pull_output(),
        pins.gpio1.into_push_pull_output(),
        pins.gpio2.into_push_pull_output(),
        pins.gpio3.into_push_pull_output(),
        pins.gpio4.into_push_pull_output(),
        pins.gpio5.into_push_pull_output(),
        pins.gpio6.into_push_pull_output(),
        pins.gpio7.into_push_pull_output(),
        pins.gpio8.into_push_pull_output(),
        pins.gpio9.into_push_pull_output(),
        pins.gpio10.into_push_pull_output(),
        pins.gpio11.into_push_pull_output(),
        pins.gpio12.into_push_pull_output(),
        pins.gpio13.into_push_pull_output(),
    );

    // SIO registers let us set/clear individual GPIO bits atomically,
    // without affecting other pins.
    let sio_regs = unsafe { &(*hal::pac::SIO::ptr()) };

    // Start with all HUB75 pins low, then disable output (OE high).
    sio_regs.gpio_out_clr().write(|w| unsafe { w.bits(ALL_HUB75) });
    sio_regs.gpio_out_set().write(|w| unsafe { w.bits(OE) });

    defmt::info!("Minimal HUB75 test: cycling colors");

    // ── 3. Define test colors ────────────────────────────────────────
    // Each color sets bits for both halves (R0+R1, G0+G1, B0+B1)
    // so the upper and lower halves show the same color.
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

    // ── 4. Main loop ─────────────────────────────────────────────────
    loop {
        let (color, name) = colors[color_idx % colors.len()];
        defmt::info!("Color: {}", name);

        // Show each color for ~300 frames.
        for _frame in 0..1500 {
            // Scan all 32 row pairs once per frame.
            for row in 0..ROW_PAIRS as u32 {
                // ── a) Clock in pixel data ────────────────────────
                // The previous row is still displayed (OE low) while
                // we shift in data for the next row. This maximises
                // the time LEDs are lit.
                for _col in 0..PANEL_WIDTH {
                    // Put color on data pins (clear first, then set).
                    sio_regs.gpio_out_clr().write(|w| unsafe { w.bits(ALL_RGB) });
                    sio_regs.gpio_out_set().write(|w| unsafe { w.bits(color) });
                    cortex_m::asm::nop(); // data setup time
                    // Rising clock edge latches one pixel.
                    sio_regs.gpio_out_set().write(|w| unsafe { w.bits(CLK) });
                    cortex_m::asm::nop();
                    sio_regs.gpio_out_clr().write(|w| unsafe { w.bits(CLK) });
                }

                // ── b) Blank → address → latch → enable ─────────
                // Disable output while we switch rows.
                sio_regs.gpio_out_set().write(|w| unsafe { w.bits(OE) });
                // Set the 5-bit row address (A–E on GPIOs 6–10).
                sio_regs.gpio_out_clr().write(|w| unsafe { w.bits(ADDR_MASK) });
                sio_regs.gpio_out_set().write(|w| unsafe { w.bits(row << 6) });
                // Pulse latch to transfer shift register to outputs.
                sio_regs.gpio_out_set().write(|w| unsafe { w.bits(LAT) });
                cortex_m::asm::nop();
                sio_regs.gpio_out_clr().write(|w| unsafe { w.bits(LAT) });
                // Re-enable output (OE low = LEDs on).
                sio_regs.gpio_out_clr().write(|w| unsafe { w.bits(OE) });

                // ── c) Display time ──────────────────────────────
                // Hold this row lit. Longer = brighter but slower
                // scan rate. At 150 MHz, 3000 cycles ≈ 20 µs.
                cortex_m::asm::delay(3000);
            }
        }

        color_idx += 1;
    }
}
