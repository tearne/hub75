//! # DP3364S S-PWM — Step 5: Unified State Machine
//!
//! Fifth in the SPWM progression. Builds on `spwm_4_dual_core.rs`.
//!
//! **What this step adds:** a single PIO state machine handles both data
//! loading and scan/display, eliminating the SM handover dark gap that
//! plagued step 4.
//!
//! **Architecture:** each DMA word starts with a 1-bit type flag. The PIO
//! program reads this bit and branches: type=0 dispatches to the data
//! path (shift RGB bits with CLK/LAT), type=1 dispatches to the scan
//! path (display period, address latch, OE pulse). No SM disable/enable,
//! no pindir handover, no dark gap.
//!
//! ```text
//!   Core 0: DMA data → SM → DMA scan (loop) → core 1 done? → DMA new data
//!           ↑                                    no → keep scanning
//!           └──────────────────────────────────────────────────────────
//!
//!   Core 1: wait for signal → fill pixels → pack → signal done
//! ```
//!
//! Communication via SIO FIFO (hardware mailbox between cores).
//!
//! ## Run
//!
//! ```sh
//! cargo run --release --example spwm_5_unified_sm
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

// ── Panel geometry ───────────────────────────────────────────────────
const SCAN_LINES: usize = 32;

// ── DP3264S / DP3364S configuration registers ────────────────────────
const CONFIG_REGS: [u16; 13] = [
    0x1100, 0x021F, 0x037F, 0x043F, 0x0504, 0x0642, 0x0700,
    0x08BF, 0x0960, 0x0ABE, 0x0B8B, 0x0C88, 0x0D12,
];

// ── Per-command CLK counts ───────────────────────────────────────────
const VSYNC_PRE: u32 = 1;    const VSYNC_LAT: u32 = 3;
const VSYNC_WORDS: usize = 1;

const PRE_ACT_PRE: u32 = 2;  const PRE_ACT_LAT: u32 = 14;
const PRE_ACT_WORDS: usize = 4;

const WR_CFG_PRE: u32 = 123; const WR_CFG_LAT: u32 = 5;
const WR_CFG_WORDS: usize = 32;

const DATA_LATCH_PRE: u32 = 127; const DATA_LATCH_LAT: u32 = 1;
const DATA_LATCH_WORDS: usize = 32;

const VSYNC_OFFSET:      usize = 0;
const PRE_ACT_OFFSET:    usize = VSYNC_OFFSET + 1 + VSYNC_WORDS;      // 2
const WR_CFG_OFFSET:     usize = PRE_ACT_OFFSET + 1 + PRE_ACT_WORDS;  // 7
const DATA_LATCH_OFFSET: usize = WR_CFG_OFFSET + 1 + WR_CFG_WORDS;    // 40

const LATCHES_PER_FRAME: usize = SCAN_LINES * 16; // 512
const DATA_LATCH_STRIDE: usize = 1 + DATA_LATCH_WORDS; // 33

const FRAME_WORDS: usize =
    DATA_LATCH_OFFSET + LATCHES_PER_FRAME * DATA_LATCH_STRIDE; // 16 936

// ── Double-buffered frame data ───────────────────────────────────────
static mut FRAME_BUF_A: [u32; FRAME_WORDS] = [0u32; FRAME_WORDS];
static mut FRAME_BUF_B: [u32; FRAME_WORDS] = [0u32; FRAME_WORDS];

// ── Core 1 stack ─────────────────────────────────────────────────────
static mut CORE1_STACK: hal::multicore::Stack<4096> = hal::multicore::Stack::new();

// ── Scan parameters ──────────────────────────────────────────────────
const DISPLAY_CLK: u32 = 100;
const SETUP_CLK: u32 = 28;
const OE_CLK_W4: u32 = 4;
const OE_CLK_W12: u32 = 12;

static mut SCAN_BUF: [u32; SCAN_LINES] = [0u32; SCAN_LINES];

// ── Pixel type and framebuffer ───────────────────────────────────────

#[derive(Copy, Clone)]
struct Rgb { r: u8, g: u8, b: u8 }
impl Rgb {
    const fn new(r: u8, g: u8, b: u8) -> Self { Self { r, g, b } }
    const BLACK: Rgb = Rgb::new(0, 0, 0);
}

static mut PIXELS: [[Rgb; 128]; 64] = [[Rgb::BLACK; 128]; 64];

// ── Gamma LUT: 8-bit -> 14-bit greyscale ─────────────────────────────

static GAMMA14: [u16; 256] = {
    let mut lut = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        let v = (i as u32 * i as u32 * 0x3FFF) / (255 * 255);
        lut[i] = v as u16;
        i += 1;
    }
    lut
};

// ── HSV helper ───────────────────────────────────────────────────────

fn hsv(hue: u8) -> Rgb {
    let h = hue as u16;
    let region = h / 43;
    let remainder = (h - region * 43) * 6;
    let q = (255 - remainder) as u8;
    let t = remainder as u8;
    match region {
        0 => Rgb::new(255, t, 0),  1 => Rgb::new(q, 255, 0),
        2 => Rgb::new(0, 255, t),  3 => Rgb::new(0, q, 255),
        4 => Rgb::new(t, 0, 255),  _ => Rgb::new(255, 0, q),
    }
}

// ── Data header with type bit ───────────────────────────────────────

const fn data_header(n_pre: u32, n_lat: u32) -> u32 {
    // bit 0 = 0 (data type)
    ((n_lat - 1) << 16) | ((n_pre - 1) << 1)
}

// ── Scan word with type bit ─────────────────────────────────────────

fn scan_word(display: u32, addr: u32, setup: u32, oe: u32) -> u32 {
    1                           // bit 0 = 1 (scan type)
    | ((display - 1) << 1)     // bits 7:1
    // bits 13:8: 0 (RGB zeros)
    | (addr << 14)             // bits 18:14 → GPIO 6-10
    | ((setup - 1) << 19)      // bits 23:19
    | ((oe - 1) << 24)         // bits 27:24
}

// ── Scan buffer builder ─────────────────────────────────────────────

fn build_scan_buf() {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(SCAN_BUF) };
    for row in 0..SCAN_LINES {
        let oe = if row == 0 { OE_CLK_W12 } else { OE_CLK_W4 };
        buf[row] = scan_word(DISPLAY_CLK, row as u32, SETUP_CLK, oe);
    }
}

// ── Frame-buffer packing ─────────────────────────────────────────────

fn pack_shift128(out: &mut [u32], word: u16, color_mask: u32) {
    let mask6 = color_mask & ALL_RGB;
    for w in 0..32 {
        let mut acc: u32 = 0;
        for slot in 0..4 {
            let pix = w * 4 + slot;
            let bit_pos = 15 - (pix % 16);
            let data = if (word >> bit_pos) & 1 == 1 { mask6 } else { 0 };
            acc |= data << (slot * 8);
        }
        out[w] = acc;
    }
}

/// Write the static command headers (VSYNC, PRE_ACT, WR_CFG) into
/// the given frame buffer. Called once at startup for each buffer;
/// DATA_LATCH headers are written by `pack_pixels()`.
fn init_frame_headers(buf: &mut [u32; FRAME_WORDS]) {
    buf[VSYNC_OFFSET] = data_header(VSYNC_PRE, VSYNC_LAT);
    buf[VSYNC_OFFSET + 1] = 0;
    buf[PRE_ACT_OFFSET] = data_header(PRE_ACT_PRE, PRE_ACT_LAT);
    for i in 0..PRE_ACT_WORDS {
        buf[PRE_ACT_OFFSET + 1 + i] = 0;
    }
    buf[WR_CFG_OFFSET] = data_header(WR_CFG_PRE, WR_CFG_LAT);
    for l in 0..LATCHES_PER_FRAME {
        buf[DATA_LATCH_OFFSET + l * DATA_LATCH_STRIDE] =
            data_header(DATA_LATCH_PRE, DATA_LATCH_LAT);
    }
}

/// Pack the PIXELS framebuffer into the given frame buffer's
/// DATA_LATCH regions.
///
/// Gamma-corrected greyscale is precomputed per column (8 chips x
/// 6 channels = 48 lookups per latch) rather than per pixel-clock
/// (128 x 6 = 768), cutting gamma lookups from 393 K to 24 K.
fn pack_pixels(buf: &mut [u32; FRAME_WORDS]) {
    let pixels = unsafe { &*core::ptr::addr_of!(PIXELS) };

    for scan_line in 0..SCAN_LINES {
        let upper_row = scan_line;
        let lower_row = scan_line + 32;

        for channel in 0..16usize {
            let latch_idx = scan_line * 16 + channel;
            let base = DATA_LATCH_OFFSET + latch_idx * DATA_LATCH_STRIDE + 1;
            let output = 15 - channel;

            let mut ur = [0u16; 8];
            let mut ug = [0u16; 8];
            let mut ub = [0u16; 8];
            let mut lr = [0u16; 8];
            let mut lg = [0u16; 8];
            let mut lb = [0u16; 8];
            for chip in 0..8usize {
                let col = chip * 16 + output;
                let u = pixels[upper_row][col];
                let l = pixels[lower_row][col];
                ur[chip] = GAMMA14[u.r as usize];
                ug[chip] = GAMMA14[u.g as usize];
                ub[chip] = GAMMA14[u.b as usize];
                lr[chip] = GAMMA14[l.r as usize];
                lg[chip] = GAMMA14[l.g as usize];
                lb[chip] = GAMMA14[l.b as usize];
            }

            for w in 0..32usize {
                let mut acc: u32 = 0;
                for slot in 0..4usize {
                    let pix = w * 4 + slot;
                    let chip = 7 - (pix >> 4);
                    let bit_pos = 15 - (pix & 15);
                    let shift = slot * 8;
                    acc |= (((ur[chip] >> bit_pos) & 1) as u32) << shift;
                    acc |= (((ug[chip] >> bit_pos) & 1) as u32) << (shift + 1);
                    acc |= (((ub[chip] >> bit_pos) & 1) as u32) << (shift + 2);
                    acc |= (((lr[chip] >> bit_pos) & 1) as u32) << (shift + 3);
                    acc |= (((lg[chip] >> bit_pos) & 1) as u32) << (shift + 4);
                    acc |= (((lb[chip] >> bit_pos) & 1) as u32) << (shift + 5);
                }
                buf[base + w] = acc;
            }
        }
    }
}

fn update_wr_cfg(buf: &mut [u32; FRAME_WORDS], reg_idx: usize) {
    let reg = CONFIG_REGS[reg_idx];
    pack_shift128(
        &mut buf[WR_CFG_OFFSET + 1 .. WR_CFG_OFFSET + 1 + WR_CFG_WORDS],
        reg, ALL_RGB,
    );
}

// ── Fill pixels helper ──────────────────────────────────────────────

fn fill_pixels(offset: u8) {
    let pixels = unsafe { &mut *core::ptr::addr_of_mut!(PIXELS) };
    for row in 0..64usize {
        for col in 0..128usize {
            let hue = (col as u32 * 255 / 127) as u8;
            let row_shift = (row as u32 * 255 / 63) as u8;
            pixels[row][col] = hsv(hue.wrapping_add(offset).wrapping_add(row_shift));
        }
    }
}

// ── SIO FIFO raw register access ────────────────────────────────────

fn fifo_write(val: u32) {
    unsafe {
        while (0xD000_0050 as *const u32).read_volatile() & 2 == 0 {}
        (0xD000_0054 as *mut u32).write_volatile(val);
        cortex_m::asm::sev();
    }
}

fn fifo_try_read() -> Option<u32> {
    unsafe {
        if (0xD000_0050 as *const u32).read_volatile() & 1 != 0 {
            Some((0xD000_0058 as *const u32).read_volatile())
        } else {
            None
        }
    }
}

fn fifo_read() -> u32 {
    unsafe {
        loop {
            if (0xD000_0050 as *const u32).read_volatile() & 1 != 0 {
                return (0xD000_0058 as *const u32).read_volatile();
            }
            cortex_m::asm::wfe(); // sleep until event
        }
    }
}

// ── Core 1 task ─────────────────────────────────────────────────────

fn core1_task() {
    loop {
        let msg = fifo_read();
        let offset = (msg & 0xFF) as u8;
        let buf_idx = ((msg >> 8) & 1) as u8;

        let buf = if buf_idx == 0 {
            unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) }
        } else {
            unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_B) }
        };

        fill_pixels(offset);
        pack_pixels(buf);
        update_wr_cfg(buf, (offset as usize / 2) % 13);

        fifo_write(1);
    }
}

// ── PIO + DMA helpers ───────────────────────────────────────────────

const PIO0_BASE: u32 = 0x5020_0000;
const PIO0_FDEBUG: u32 = PIO0_BASE + 0x008;
const SM0_CLKDIV: u32 = PIO0_BASE + 0x0C8;
const SM0_TXSTALL: u32 = 1 << 24;

fn set_clkdiv(int_div: u32) {
    unsafe { (SM0_CLKDIV as *mut u32).write_volatile(int_div << 16) };
}

fn wait_txstall(sm_stall_bit: u32) {
    unsafe {
        (PIO0_FDEBUG as *mut u32).write_volatile(sm_stall_bit);
        while (PIO0_FDEBUG as *const u32).read_volatile() & sm_stall_bit == 0 {
            cortex_m::asm::nop();
        }
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
    let mut sio_hal = hal::Sio::new(pac.SIO);
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
    let sio_regs = unsafe { &(*hal::pac::SIO::ptr()) };
    sio_regs.gpio_out_clr().write(|w| unsafe {
        w.bits(ALL_RGB | (0x1F << 6) | (1 << 11) | (1 << 12) | (1 << 13))
    });
    const IO_BANK0_BASE: u32 = 0x4002_8000;
    for p in 0..14u32 {
        let addr = IO_BANK0_BASE + 0x04 + p * 8;
        unsafe { (addr as *mut u32).write_volatile(6) };
    }

    // Pull-down on LAT (GPIO 12) for safety during startup
    const PADS_BANK0: u32 = 0x4003_8000;
    const GPIO12_PAD: u32 = PADS_BANK0 + 0x04 + 12 * 4;
    unsafe {
        let val = (GPIO12_PAD as *const u32).read_volatile();
        (GPIO12_PAD as *mut u32).write_volatile(val | (1 << 2));
    }

    // ── 3. Pre-build the scan buffer ─────────────────────────────────
    build_scan_buf();

    // ── 4. Install unified PIO program ───────────────────────────────
    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let ss = pio::SideSet::new(false, 2, false);
    let mut a = pio::Assembler::new_with_side_set(ss);

    // Labels
    let mut dispatch    = a.label();
    let mut data_cmd    = a.label();
    let mut display_lp  = a.label();
    let mut setup_lp    = a.label();
    let mut oe_lp       = a.label();
    let mut pre_loop    = a.label();
    let mut lat_loop    = a.label();
    let mut wrap_source = a.label();

    // ── DISPATCH (wrap_target) ───────────────────────────────────
    a.bind(&mut dispatch);
    // 0: out x, 1 side 0b00            ; read type bit
    a.out_with_side_set(pio::OutDestination::X, 1, 0b00);
    // 1: jmp !x data_cmd side 0b00     ; type=0 → data
    a.jmp_with_side_set(pio::JmpCondition::XIsZero, &mut data_cmd, 0b00);

    // ── SCAN (type=1) ────────────────────────────────────────────
    // 2: out y, 7 side 0b00            ; display_count-1
    a.out_with_side_set(pio::OutDestination::Y, 7, 0b00);
    // 3: display loop start
    a.bind(&mut display_lp);
    // 3: mov y, y side 0b00            ; NOP
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );
    // 4: mov y, y side 0b00            ; NOP
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );
    // 5: jmp y-- display_lp side 0b01  ; CLK high
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut display_lp, 0b01);

    // 6: out pins, 11 side 0b00        ; ADDR to GPIO 6-10 (zeros to RGB 0-5)
    a.out_with_side_set(pio::OutDestination::PINS, 11, 0b00);
    // 7: out y, 5 side 0b00            ; setup_count-1
    a.out_with_side_set(pio::OutDestination::Y, 5, 0b00);

    // 8: setup loop
    a.bind(&mut setup_lp);
    // 8: mov y, y side 0b00            ; NOP
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );
    // 9: mov y, y side 0b00            ; NOP
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );
    // 10: jmp y-- setup_lp side 0b01   ; CLK high
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut setup_lp, 0b01);

    // 11: out y, 4 side 0b00           ; oe_count-1
    a.out_with_side_set(pio::OutDestination::Y, 4, 0b00);
    // 12: set pins, 1 side 0b00        ; OE high
    a.set_with_side_set(pio::SetDestination::PINS, 1, 0b00);

    // 13: OE loop
    a.bind(&mut oe_lp);
    // 13: mov y, y side 0b00           ; NOP
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );
    // 14: mov y, y side 0b00           ; NOP
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );
    // 15: jmp y-- oe_lp side 0b01      ; CLK high
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut oe_lp, 0b01);

    // 16: set pins, 0 side 0b00        ; OE low
    a.set_with_side_set(pio::SetDestination::PINS, 0, 0b00);
    // 17: out null, 4 side 0b00        ; discard remaining bits
    a.out_with_side_set(pio::OutDestination::NULL, 4, 0b00);
    // 18: jmp dispatch side 0b00       ; back to dispatch
    a.jmp_with_side_set(pio::JmpCondition::Always, &mut dispatch, 0b00);

    // ── DATA (type=0) ────────────────────────────────────────────
    a.bind(&mut data_cmd);
    // 19: out x, 15 side 0b00          ; N_pre-1 (15 bits since 1 bit was type)
    a.out_with_side_set(pio::OutDestination::X, 15, 0b00);
    // 20: out y, 16 side 0b00          ; N_lat-1
    a.out_with_side_set(pio::OutDestination::Y, 16, 0b00);

    // 21: pre loop
    a.bind(&mut pre_loop);
    // 21: out pins, 6 side 0b00        ; RGB, CLK low
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0b00);
    // 22: out null, 2 side 0b01        ; discard, CLK high
    a.out_with_side_set(pio::OutDestination::NULL, 2, 0b01);
    // 23: jmp x-- pre_loop side 0b00   ; pre loop
    a.jmp_with_side_set(pio::JmpCondition::XDecNonZero, &mut pre_loop, 0b00);

    // 24: lat loop
    a.bind(&mut lat_loop);
    // 24: out pins, 6 side 0b10        ; RGB, LAT high, CLK low
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0b10);
    // 25: out null, 2 side 0b11        ; discard, CLK+LAT high
    a.out_with_side_set(pio::OutDestination::NULL, 2, 0b11);
    // 26: jmp y-- lat_loop side 0b10   ; lat loop
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut lat_loop, 0b10);
    a.bind(&mut wrap_source);

    let prog = a.assemble_with_wrap(wrap_source, dispatch);
    let installed = pio0.install(&prog).unwrap();

    // ── SM0 setup: unified data+scan ─────────────────────────────
    let (mut sm, _, tx) =
        hal::pio::PIOBuilder::from_installed_program(installed)
            .buffers(hal::pio::Buffers::OnlyTx)
            .out_pins(0, 11)         // GPIO 0-10: RGB + ADDR
            .set_pins(13, 1)         // GPIO 13: OE
            .side_set_pin_base(11)   // GPIO 11: CLK, GPIO 12: LAT
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .autopull(true)
            .pull_threshold(32)
            .clock_divisor_fixed_point(2, 0)
            .build(sm0);

    sm.set_pindirs([
        (0,  hal::pio::PinDir::Output), (1,  hal::pio::PinDir::Output),
        (2,  hal::pio::PinDir::Output), (3,  hal::pio::PinDir::Output),
        (4,  hal::pio::PinDir::Output), (5,  hal::pio::PinDir::Output),
        (6,  hal::pio::PinDir::Output), (7,  hal::pio::PinDir::Output),
        (8,  hal::pio::PinDir::Output), (9,  hal::pio::PinDir::Output),
        (10, hal::pio::PinDir::Output), (11, hal::pio::PinDir::Output),
        (12, hal::pio::PinDir::Output), (13, hal::pio::PinDir::Output),
    ]);
    let _sm = sm.start();
    let mut tx = tx;

    // ── 5. DMA — single channel feeds unified SM ─────────────────
    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut ch = dma.ch0;

    defmt::info!("DP3364S S-PWM (unified SM, no dark gap) starting");

    // ── 6. Init frame headers + initial pixel fill ───────────────
    init_frame_headers(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) });
    init_frame_headers(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_B) });

    fill_pixels(0);
    pack_pixels(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) });

    // ── 7. Spawn core 1 ──────────────────────────────────────────
    let mut mc = hal::multicore::Multicore::new(
        &mut pac.PSM, &mut pac.PPB, &mut sio_hal.fifo,
    );
    let cores = mc.cores();
    cores[1].spawn(unsafe { CORE1_STACK.take().unwrap() }, core1_task).unwrap();

    // ── 8. Startup flush using buffer A ──────────────────────────
    for flush in 0..14u32 {
        if (flush as usize) < CONFIG_REGS.len() {
            update_wr_cfg(
                unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) },
                flush as usize,
            );
        }
        let buf = unsafe { &*core::ptr::addr_of!(FRAME_BUF_A) };
        let cfg = single_buffer::Config::new(ch, buf, tx);
        let xfer = cfg.start();
        let (new_ch, _, new_tx) = xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);

        // Scan a few passes after each flush frame
        let scan = unsafe { &*core::ptr::addr_of!(SCAN_BUF) };
        let scan_cfg = single_buffer::Config::new(ch, scan, tx);
        let scan_xfer = scan_cfg.start();
        let (new_ch, _, new_tx) = scan_xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);
    }
    defmt::info!("flush complete");

    // ── 9. Main loop — unified SM, no dark gap ───────────────────
    let mut active_buf: u8 = 0;
    let mut offset: u8 = 0;

    const TIMER0_BASE: u32 = 0x400B_0000;
    const TIMELR: *const u32 = (TIMER0_BASE + 0x0C) as *const u32;
    fn timer_us() -> u32 {
        unsafe { TIMELR.read_volatile() }
    }
    let mut frame_count: u32 = 0;
    let mut last_report = timer_us();

    // Kick off core 1 with the first pack job
    offset = offset.wrapping_add(2);
    let mut core1_buf: u8 = 1 - active_buf;
    fifo_write(offset as u32 | ((core1_buf as u32) << 8));

    // Initial data load
    {
        let buf = if active_buf == 0 {
            unsafe { &*core::ptr::addr_of!(FRAME_BUF_A) }
        } else {
            unsafe { &*core::ptr::addr_of!(FRAME_BUF_B) }
        };
        let data_cfg = single_buffer::Config::new(ch, buf, tx);
        let data_xfer = data_cfg.start();
        let (new_ch, _, new_tx) = data_xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);
    }

    set_clkdiv(3);

    loop {
        // ── Scan continuously until core 1 done ──────────────────
        loop {
            let scan = unsafe { &*core::ptr::addr_of!(SCAN_BUF) };
            let scan_cfg = single_buffer::Config::new(ch, scan, tx);
            let scan_xfer = scan_cfg.start();
            let (new_ch, _, new_tx) = scan_xfer.wait();
            ch = new_ch;
            tx = new_tx;
            wait_txstall(SM0_TXSTALL);

            if fifo_try_read().is_some() { break }
        }

        frame_count += 1;

        // ── New frame ready — DMA it (no SM stop/start!) ─────────
        set_clkdiv(2);
        active_buf = core1_buf;
        let buf = if active_buf == 0 {
            unsafe { &*core::ptr::addr_of!(FRAME_BUF_A) }
        } else {
            unsafe { &*core::ptr::addr_of!(FRAME_BUF_B) }
        };
        let data_cfg = single_buffer::Config::new(ch, buf, tx);
        let data_xfer = data_cfg.start();
        let (new_ch, _, new_tx) = data_xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);
        set_clkdiv(3);

        // ── Kick off next pack job ───────────────────────────────
        offset = offset.wrapping_add(2);
        core1_buf = 1 - active_buf;
        fifo_write(offset as u32 | ((core1_buf as u32) << 8));

        if frame_count & 0xFF == 0 {
            let now = timer_us();
            let elapsed_us = now.wrapping_sub(last_report);
            if elapsed_us > 0 {
                let fps_x10 = 2_560_000_000u32 / elapsed_us;
                defmt::info!("FPS: {}.{}", fps_x10 / 10, fps_x10 % 10);
            }
            last_report = now;
        }
    }
}
