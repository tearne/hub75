//! # DP3364S S-PWM HUB75 Driver — Minimal Working Example
//!
//! Drives a 64×128 HUB75 LED panel using DP3364S S-PWM driver chips on the
//! Pimoroni Interstate 75 W (RP2350A). Cycles through solid colours and
//! brightness levels to demonstrate full RGB + greyscale control.
//!
//! ## Background
//!
//! The DP3364S (by Shenzhen Developer Microelectronics / Depuw) is an S-PWM
//! (Scrambled PWM) constant-current LED driver. Unlike traditional HUB75
//! shift-register drivers (FM6124, ICN2037, MBI5124, etc.) which require
//! the host controller to implement Binary Code Modulation (BCM) via OE
//! timing, the DP3364S contains:
//!
//!   - Internal SRAM for storing 14-bit greyscale data per output
//!   - A PLL that multiplies the data clock (DCLK) to generate an internal
//!     greyscale clock (GCLK)
//!   - An autonomous PWM generator — brightness is handled entirely on-chip
//!   - 13 configuration registers that must be initialised before display
//!
//! The DP3364S is closely related to the DP3264S (same manufacturer, same
//! register map). The DP3264S datasheet was used as the primary protocol
//! reference, supplemented by the DMD_STM32 open-source library which has
//! a working DP3264 driver.
//!
//! At time of writing (April 2026), no open-source project has published a
//! working DP3364S driver. The register values and protocol documented here
//! were validated empirically on real hardware.
//!
//! ## Protocol Overview
//!
//! The S-PWM protocol uses the same HUB75 physical pins but with different
//! semantics:
//!
//! ### Command encoding via LAT pulse width
//!
//! Commands are encoded by the number of CLK pulses that occur while LAT
//! is held HIGH during a data shift:
//!
//! | LAT width | Command     | Description                              |
//! |-----------|-------------|------------------------------------------|
//! | 1 CLK     | DATA_LATCH  | Store 16-bit greyscale data into SRAM    |
//! | 3 CLK     | VSYNC       | Frame sync — commit data, start new frame|
//! | 5 CLK     | WR_CFG      | Write one configuration register         |
//! | 14 CLK    | PRE_ACT     | Enable register writing (precedes WR_CFG)|
//!
//! **Critical:** LAT must go HIGH during the LAST N CLK cycles of the data
//! shift, not as a separate pulse afterwards. Extra CLK pulses after the
//! shift would push data further through the daisy chain, corrupting the
//! values in each chip's shift register.
//!
//! ### OE pin repurposed as ROW signal
//!
//! The OE pin does not function as an output enable. Instead it serves as
//! a ROW signal that advances the chip's internal scan-line counter:
//!
//!   - W12: OE HIGH for 12 CLK cycles — resets to row 0 (start of group)
//!   - W4:  OE HIGH for 4 CLK cycles — advance to next row
//!   - Minimum ~128 CLK between ROW pulses
//!   - ABCDE address lines must be set before each ROW pulse to select
//!     which physical row of LEDs is powered
//!
//! ### Frame structure
//!
//! Each refresh frame follows this sequence:
//!
//! ```text
//! VSYNC → PRE_ACT → WR_CFG (one register) → DATA_LATCH × 512 → display
//! ```
//!
//! **Critical:** There must be exactly ONE VSYNC per frame. Using separate
//! VSYNCs for config and data (e.g., VSYNC→config→VSYNC→data) breaks data
//! loading — the second VSYNC commits an incomplete frame.
//!
//! **Critical:** The display cannot be sustained by CLK + ROW alone. The
//! chip requires continuous VSYNC + data reload cycles. Without fresh data
//! frames, the display fades within approximately one frame period.
//!
//! ### Configuration registers
//!
//! 13 registers must be written, one per frame (one WR_CFG per VSYNC
//! cycle). No separate initialisation phase is needed — registers can be
//! written interleaved with normal data refresh from cold start. Format
//! is `{register_address[15:8], value[7:0]}`.
//!
//! ### Data transfer
//!
//! For a 128-pixel-wide panel with 1/32 scan (32 scan lines):
//!
//!   - 8 DP3364S chips per colour chain (128 pixels ÷ 16 outputs per chip)
//!   - 6 parallel chains: R0, G0, B0 (upper half), R1, G1, B1 (lower half)
//!   - Each DATA_LATCH shifts 128 bits (8 chips × 16 bits), MSB first
//!   - 512 DATA_LATCH operations per frame (32 scan lines × 16 channels)
//!   - 16-bit greyscale per pixel; only lower 14 bits used (max 0x3FFF)
//!
//! ## Hardware
//!
//!   - Board: Pimoroni Interstate 75 W (RP2350A, ARM Cortex-M33, 150 MHz)
//!   - Panel: 64×128 HUB75 with DP3364S drivers, 1/32 scan
//!   - Debug: Raspberry Pi Debug Probe (CMSIS-DAP SWD)
//!
//! ## References
//!
//!   - DP3264S datasheet (translated):
//!     <https://cognigraph.com/6502/datasheet-DP3264S-google-translate.pdf>
//!   - DMD_STM32 S-PWM driver (working DP3264 code):
//!     <https://github.com/board707/DMD_STM32>
//!     Key files: DMD_SPWM_Driver.h, DMD_SPWM_Driver_RP.h, DMD_RGB.cpp
//!   - hzeller/rpi-rgb-led-matrix S-PWM discussion:
//!     <https://github.com/hzeller/rpi-rgb-led-matrix/issues/1866>
//!   - DMD_STM32 DP3364 tracking issues:
//!     <https://github.com/board707/DMD_STM32/issues/127>
//!     <https://github.com/board707/DMD_STM32/discussions/174>
//!   - Depuw (chip manufacturer): <http://www.depuw.com/>
//!
//! ## Run
//!
//! ```sh
//! cargo run --release --example minimal_spwm_cpu
//! ```

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
// The Interstate 75 W maps the HUB75 connector to GPIOs 0–13:
//
//   Data (accent colour for upper/lower panel halves):
//     GPIO 0 = R0   GPIO 3 = R1
//     GPIO 1 = G0   GPIO 4 = G1
//     GPIO 2 = B0   GPIO 5 = B1
//
//   Row address (active row selection via external decoder/MOSFETs):
//     GPIO 6–10 = A, B, C, D, E   (5 bits → 32 rows for 1/32 scan)
//
//   Control:
//     GPIO 11 = CLK  (serial data clock; also feeds the DP3364S's PLL)
//     GPIO 12 = LAT  (latch; pulse width encodes the S-PWM command)
//     GPIO 13 = OE   (repurposed as ROW signal — not output enable)
//
const R0: u32 = 1 << 0;
const G0: u32 = 1 << 1;
const B0: u32 = 1 << 2;
const R1: u32 = 1 << 3;
const G1: u32 = 1 << 4;
const B1: u32 = 1 << 5;
const ALL_RGB: u32 = R0 | G0 | B0 | R1 | G1 | B1;
const ADDR_MASK: u32 = 0x1F << 6; // Bits 6–10
const CLK: u32 = 1 << 11;
const LAT: u32 = 1 << 12;
const OE: u32 = 1 << 13;
const ALL_HUB75: u32 = ALL_RGB | ADDR_MASK | CLK | LAT | OE;

// ── Panel geometry ───────────────────────────────────────────────────
const PANEL_WIDTH: usize = 128; // Pixels per row
const SCAN_LINES: usize = 32; // 1/32 scan (64 rows ÷ 2 halves)
const CHIPS_PER_CHAIN: usize = PANEL_WIDTH / 16; // 8 (each DP3364S has 16 outputs)
const CHAIN_BITS: usize = CHIPS_PER_CHAIN * 16; // 128 bits shifted per DATA_LATCH

// Minimum CLK cycles between ROW pulses. The DP3264S datasheet specifies
// 128 as the minimum for correct open-circuit detection timing. The
// DMD_STM32 driver uses a 28+4+96 = 128 cycle pattern (pre-OE delay +
// OE pulse + post-OE display time).
const ROW_PERIOD: usize = 128;

// ── DP3264S / DP3364S configuration registers ────────────────────────
//
// These values are taken from the DMD_STM32 DP3264 driver and confirmed
// to work unmodified with the DP3364S. Format: {addr[15:8], value[7:0]}.
//
// One register is written per VSYNC frame via the PRE_ACT → WR_CFG
// sequence. All 13 are cycled through round-robin during normal operation.
//
const CONFIG_REGS: [u16; 13] = [
    0x1100, // reg 0x11 = 0x00  Reserved (included per DMD_STM32 reference)
    0x021F, // reg 0x02 = 0x1F  LINE_SET = 31 → 32 scan lines
    0x033F, // reg 0x03 = 0x3F  GROUP_SET = 63 → 64 display groups
    0x043F, // reg 0x04 = 0x3F  PWM_WIDTH = 63 → max greyscale depth
    0x0504, // reg 0x05 = 0x04  DISSHD_TIME (shadow/disappearance timing)
    0x0642, // reg 0x06 = 0x42  PLL_DIV = 2 → GCLK = DCLK × 3
    0x0700, // reg 0x07 = 0x00  Gamma disabled
    0x08BF, // reg 0x08 = 0xBF  IGAIN (output current gain)
    0x0960, // reg 0x09 = 0x60  DECOUP_1 (coupling optimisation)
    0x0ABE, // reg 0x0A = 0xBE  Feature enables (shadow, coupling, low-grey)
    0x0B8B, // reg 0x0B = 0x8B  CORNER + DISSHD_LEVEL
    0x0C88, // reg 0x0C = 0x88  SYNC_MODE = 2 (high-grey independent refresh)
    0x0D12, // reg 0x0D = 0x12  DECOUP_LEVEL
];

type Sio = hal::pac::sio::RegisterBlock;

// ── Low-level signal helpers ─────────────────────────────────────────

/// One CLK cycle: rising edge then falling edge.
#[inline(always)]
fn clock_pulse(sio: &Sio) {
    sio.gpio_out_set().write(|w| unsafe { w.bits(CLK) });
    cortex_m::asm::nop(); // Data setup time
    sio.gpio_out_clr().write(|w| unsafe { w.bits(CLK) });
}

/// Generate `n` CLK pulses with all data lines held low.
fn send_clocks(sio: &Sio, n: usize) {
    sio.gpio_out_clr().write(|w| unsafe { w.bits(ALL_RGB) });
    for _ in 0..n {
        clock_pulse(sio);
    }
}

/// Pulse LAT for `width` CLK cycles (command-only; shift register data
/// does not matter). Used for PRE_ACT (width=14).
fn send_latches(sio: &Sio, width: usize) {
    sio.gpio_out_clr().write(|w| unsafe { w.bits(ALL_RGB) });
    sio.gpio_out_set().write(|w| unsafe { w.bits(LAT) });
    for _ in 0..width {
        clock_pulse(sio);
    }
    sio.gpio_out_clr().write(|w| unsafe { w.bits(LAT) });
}

// ── S-PWM command functions ──────────────────────────────────────────

/// Shift a 16-bit word to all chips in all colour chains, with LAT
/// asserted during the last `lat_width` CLK cycles.
///
/// The 16-bit `word` is repeated for each of the 8 chips in the chain
/// (128 CLK total). All 6 RGB data lines carry the same bit pattern so
/// every colour chain receives identical data.
///
/// This implements the same protocol as DMD_STM32's `send_to_allRGB()`:
/// LAT goes HIGH at CLK `(CHAIN_BITS - lat_width)` and stays HIGH for
/// the remainder of the shift. The LAT pulse width seen by the chip
/// encodes the command type (5 = WR_CFG, 1 = DATA_LATCH, etc.).
fn shift_all_with_lat(sio: &Sio, word: u16, lat_width: usize) {
    let lat_start = CHAIN_BITS - lat_width;
    for i in 0..CHAIN_BITS {
        let bit_pos = 15 - (i % 16); // MSB first, repeating per chip

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
    sio.gpio_out_clr()
        .write(|w| unsafe { w.bits(LAT | ALL_RGB) });
}

/// VSYNC (LAT width = 3): commit previous frame, start new frame.
///
/// Per DMD_STM32: 1 bare CLK pulse followed by 3 CLK with LAT HIGH.
fn vsync(sio: &Sio) {
    sio.gpio_out_clr()
        .write(|w| unsafe { w.bits(ALL_RGB | CLK) });
    clock_pulse(sio); // 1 bare CLK
    send_latches(sio, 3); // 3 CLK with LAT
}

/// PRE_ACT (LAT width = 14): enable register writing. Must be issued
/// immediately before WR_CFG within the same frame.
fn pre_act(sio: &Sio) {
    send_latches(sio, 14);
}

/// WR_CFG (LAT width = 5): write one configuration register.
///
/// `reg_data` format: `{register_address[15:8], value[7:0]}`.
/// The 16-bit word is shifted to all chips (all get the same config).
fn wr_cfg(sio: &Sio, reg_data: u16) {
    shift_all_with_lat(sio, reg_data, 5);
}

/// Set the 5-bit row address (ABCDE) on GPIOs 6–10.
fn set_addr(sio: &Sio, row: usize) {
    sio.gpio_out_clr()
        .write(|w| unsafe { w.bits(ADDR_MASK) });
    sio.gpio_out_set()
        .write(|w| unsafe { w.bits((row as u32) << 6) });
}

/// Generate a ROW pulse (OE pin HIGH) for `width` CLK cycles.
fn row_pulse(sio: &Sio, width: usize) {
    sio.gpio_out_set().write(|w| unsafe { w.bits(OE) });
    for _ in 0..width {
        clock_pulse(sio);
    }
    sio.gpio_out_clr().write(|w| unsafe { w.bits(OE) });
}

// ── Display loop ─────────────────────────────────────────────────────

/// Run `cycles` complete scans through all SCAN_LINES rows.
///
/// This provides the two signals the DP3364S needs to display:
///   a) Continuous CLK — feeds the PLL that generates the internal GCLK
///   b) Periodic ROW pulses — advance through scan lines
///
/// The ABCDE address is set 28 CLK before each ROW pulse (matching
/// the DMD_STM32 timing of d1=28 pre-OE delay).
fn display_loop(sio: &Sio, cycles: usize) {
    let pre_addr = ROW_PERIOD - 28; // CLK before address change
    let mut row: usize = 0;
    for _ in 0..cycles * SCAN_LINES {
        send_clocks(sio, pre_addr);
        set_addr(sio, row);
        send_clocks(sio, 28); // Address setup time
        let w = if row == 0 { 12 } else { 4 }; // W12 resets scan; W4 advances
        row_pulse(sio, w);
        row += 1;
        if row >= SCAN_LINES {
            row = 0;
        }
    }
}

// ── Data transfer ────────────────────────────────────────────────────

/// Load greyscale data into the DP3364S SRAM for all scan lines and
/// output channels.
///
/// `brightness`: 14-bit greyscale value (0x0000–0x3FFF) for the background.
/// `color_mask`: which of the 6 RGB data lines receive the brightness
///               value. Unselected lines are held low (black on that
///               channel). For example:
///                 - `R0 | R1`  → red only
///                 - `G0 | G1`  → green only
///                 - `ALL_RGB`  → white
/// `highlight_col`: column (0..PANEL_WIDTH-1) to draw as a bright white
///                  vertical line, or `usize::MAX` to disable.
///
/// ## Data addressing
///
/// The 512 DATA_LATCH operations are structured as:
///   - Outer: 32 scan lines (each drives a row pair: upper + lower half)
///   - Inner: 16 channels per chip (OUT15 first, OUT0 last)
///
/// Within each channel latch, 128 bits are shifted through the chain.
/// The first bits shifted end up at the last chip (farthest from the
/// connector); the last bits shifted stay at the first chip (nearest).
///
/// To address a specific column, we need to determine which chip and
/// which output channel it maps to, and send the highlight brightness
/// only for that combination.
/// Shift one latch (128 bits) of uniform data. This is the fast path
/// used for 511 of the 512 latches per frame.
#[inline(always)]
fn latch_uniform(sio: &Sio, brightness: u16, color_mask: u32) {
    let lat_at = CHAIN_BITS - 1;
    for i in 0..CHAIN_BITS {
        let bit_pos = 15 - (i % 16);
        if i == lat_at {
            sio.gpio_out_set()
                .write(|w| unsafe { w.bits(LAT) });
        }
        sio.gpio_out_clr()
            .write(|w| unsafe { w.bits(ALL_RGB) });
        if (brightness >> bit_pos) & 1 == 1 {
            sio.gpio_out_set()
                .write(|w| unsafe { w.bits(color_mask) });
        }
        clock_pulse(sio);
    }
    sio.gpio_out_clr()
        .write(|w| unsafe { w.bits(LAT | ALL_RGB) });
}

/// Shift one latch with a single chip highlighted (different brightness/colour).
#[inline(always)]
fn latch_with_highlight(
    sio: &Sio,
    brightness: u16,
    color_mask: u32,
    hl_shift_pos: usize,
) {
    let lat_at = CHAIN_BITS - 1;
    for i in 0..CHAIN_BITS {
        let bit_pos = 15 - (i % 16);
        let (b, m) = if (i >> 4) == hl_shift_pos {
            (0x3FFF_u16, ALL_RGB)
        } else {
            (brightness, color_mask)
        };
        if i == lat_at {
            sio.gpio_out_set()
                .write(|w| unsafe { w.bits(LAT) });
        }
        sio.gpio_out_clr()
            .write(|w| unsafe { w.bits(ALL_RGB) });
        if (b >> bit_pos) & 1 == 1 {
            sio.gpio_out_set()
                .write(|w| unsafe { w.bits(m) });
        }
        clock_pulse(sio);
    }
    sio.gpio_out_clr()
        .write(|w| unsafe { w.bits(LAT | ALL_RGB) });
}

fn data_transfer_simple(
    sio: &Sio,
    brightness: u16,
    color_mask: u32,
    highlight_col: usize,
) {
    let hl_chip_phys = highlight_col / 16;
    let hl_output = highlight_col % 16;
    let hl_channel = 15 - hl_output;
    let hl_shift_pos = (CHIPS_PER_CHAIN - 1) - hl_chip_phys;

    for _scan_line in 0..SCAN_LINES {
        for channel in 0..16usize {
            if highlight_col < PANEL_WIDTH && channel == hl_channel {
                latch_with_highlight(sio, brightness, color_mask, hl_shift_pos);
            } else {
                latch_uniform(sio, brightness, color_mask);
            }
        }
    }
}

// ── Entry point ──────────────────────────────────────────────────────

#[hal::entry]
fn main() -> ! {
    // ── 1. Initialise the RP2350 ─────────────────────────────────────
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let _clocks = hal::clocks::init_clocks_and_plls(
        12_000_000, // External crystal frequency
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    // ── 2. Configure GPIOs 0–13 as SIO outputs ──────────────────────
    let sio_hal = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio_hal.gpio_bank0,
        &mut pac.RESETS,
    );
    let _pins = (
        pins.gpio0.into_push_pull_output(),  // R0
        pins.gpio1.into_push_pull_output(),  // G0
        pins.gpio2.into_push_pull_output(),  // B0
        pins.gpio3.into_push_pull_output(),  // R1
        pins.gpio4.into_push_pull_output(),  // G1
        pins.gpio5.into_push_pull_output(),  // B1
        pins.gpio6.into_push_pull_output(),  // A
        pins.gpio7.into_push_pull_output(),  // B
        pins.gpio8.into_push_pull_output(),  // C
        pins.gpio9.into_push_pull_output(),  // D
        pins.gpio10.into_push_pull_output(), // E
        pins.gpio11.into_push_pull_output(), // CLK
        pins.gpio12.into_push_pull_output(), // LAT
        pins.gpio13.into_push_pull_output(), // OE (ROW)
    );

    // Drive GPIOs via the SIO register block for speed.
    let sio = unsafe { &(*hal::pac::SIO::ptr()) };
    sio.gpio_out_clr()
        .write(|w| unsafe { w.bits(ALL_HUB75) });

    defmt::info!("DP3364S S-PWM driver starting");

    // ── 3. Main refresh loop ─────────────────────────────────────────
    //
    // Each iteration of the outer loop displays one test pattern for
    // approximately 4–5 seconds (300 refresh frames).
    //
    // Each refresh frame:
    //   1. VSYNC (starts the frame)
    //   2. PRE_ACT + WR_CFG (writes one config register, round-robin)
    //   3. Data transfer (512 DATA_LATCH operations)
    //   4. Display loop (CLK + ROW cycling)
    //
    // No separate initialisation phase is needed. The config registers
    // are populated during the first 13 refresh frames while the panel
    // simultaneously receives data and begins displaying.

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
    let mut scan_col: usize = 0; // Column position of the scanning line
    let mut frame_count: usize = 0; // Frame counter for slowing the scan

    loop {
        let (brightness, color_mask, name) = tests[test_idx % tests.len()];
        defmt::info!("{}", name);

        for _ in 0..300 {
            // ── Frame start ──────────────────────────────────────
            vsync(sio);
            send_clocks(sio, 16);

            // ── Config register (one per frame, round-robin) ─────
            pre_act(sio);
            send_clocks(sio, 8);
            wr_cfg(sio, CONFIG_REGS[reg_idx]);
            send_clocks(sio, 8);
            reg_idx += 1;
            if reg_idx >= CONFIG_REGS.len() {
                reg_idx = 0;
            }

            // ── Pixel data with scanning vertical line ───────────
            data_transfer_simple(sio, brightness, color_mask, scan_col);

            // Advance column every 4 frames for visible scanning.
            frame_count += 1;
            if frame_count >= 4 {
                frame_count = 0;
                scan_col += 1;
                if scan_col >= PANEL_WIDTH {
                    scan_col = 0;
                }
            }

            // ── Display (CLK feeds PLL, ROW advances scan) ───────
            display_loop(sio, 10);
        }

        test_idx += 1;
    }
}
