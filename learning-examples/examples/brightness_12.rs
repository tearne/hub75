//! # Thread A brightness probe — spwm_12 (split-DMA, unified-program) variant
//!
//! Purpose: verify the hypothesis that spwm_12's older architecture
//! flickered less than brightness_14/15 because it **re-displays each
//! frame at a fixed ~65 Hz** (data + 50 scan cycles per loop iteration),
//! regardless of core-1 pack rate. brightness_14/15 only trigger a data
//! phase when core 1 signals a fresh frame, so their data-phase rate
//! collapses to the ~39 Hz pack rate — well below human flicker fusion.
//!
//! ## Prediction (to be confirmed by this probe)
//!
//! With POST_SCAN_CYCLES=50 at 253 µs/cycle, plus 2.7 ms data:
//!
//!   - Frame (cycle) rate = 1 / (2.7 + 50 × 0.253) ms ≈ **65 Hz**
//!   - Dark fraction       = 2.7 / 15.4 ≈ **17.5 %**
//!
//! If the measurements land there, spwm_12's flicker advantage is
//! confirmed as **higher data-phase frequency** — above fusion for
//! most observers — even though its dark fraction is *larger* than
//! brightness_14/15's.
//!
//! ## Caveat — echo is present
//!
//! spwm_12's architecture has the scan_line 31 → scan_line 0 SRAM
//! echo (the whole subject of the E21–E25 investigation, see
//! `spwm_12_split_dma.rs`). On a uniform grey flood this doesn't
//! matter — every row is the same value, so the echo is invisible
//! (grey echoes grey = grey). On the rainbow pattern (`MOVING=true`)
//! the echo will show as smearing on rows 0 / 32, but that's
//! orthogonal to the flicker measurement.
//!
//! ## Fill mode (compile-time flag)
//!
//! Match `brightness_14`'s and `brightness_15`'s `MOVING` and
//! `GREY_LEVEL` for a valid cross-architecture comparison.
//!
//! ## Architecture — verbatim from spwm_12 except:
//!
//!   - `fill_pixels` is the Thread A grey/rainbow toggle.
//!   - DWT cycle-counter timing instrumentation replaces the
//!     non-functional `timer_us`/FPS report.
//!   - The `INTER_DMA_DELAY_CYCLES` / `INTER_DMA_INJECT` /
//!     `INTER_DMA_HIGHZ_CYCLES` experiment toggles (all no-ops at
//!     spwm_12's current baseline) are removed.
//!
//! Startup flush and sync phases are kept verbatim — bootstrapping
//! the DP3364S on this panel is delicate.
//!
//! ```sh
//! cargo run --release --example brightness_12
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

// ── Thread A controls ────────────────────────────────────────────────
/// `false` = solid grey flood (brightness probe).
/// `true`  = diagonal rainbow scroll (flicker/echo probe).
const MOVING: bool = false;
/// Per-channel level for the solid-grey probe. Keep identical to the
/// value in `brightness_14` / `brightness_15` for a valid comparison.
const GREY_LEVEL: u8 = 128;

// ── Timing instrumentation (DWT cycle counter) ──────────────────────
const SYS_CLK_MHZ: u32 = 150;
const DEMCR: u32     = 0xE000_EDFC;
const DWT_CTRL: u32  = 0xE000_1000;
const DWT_CYCCNT: u32 = 0xE000_1004;

fn enable_cycle_counter() {
    unsafe {
        let demcr = (DEMCR as *const u32).read_volatile();
        (DEMCR as *mut u32).write_volatile(demcr | (1 << 24)); // TRCENA
        (DWT_CYCCNT as *mut u32).write_volatile(0);
        let ctrl = (DWT_CTRL as *const u32).read_volatile();
        (DWT_CTRL as *mut u32).write_volatile(ctrl | 1); // CYCCNTENA
    }
}

fn now_cycles() -> u32 {
    unsafe { (DWT_CYCCNT as *const u32).read_volatile() }
}

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
// GROUP_SET (0x03) FIRST — see spwm_12 for rationale.
const CONFIG_REGS: [u16; 13] = [
    0x037F, 0x1100, 0x021F, 0x043F, 0x0504, 0x0642, 0x0700,
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

// ── Frame layout ────────────────────────────────────────────────────
const VSYNC_OFFSET:   usize = 0;
const PRE_ACT_OFFSET: usize = VSYNC_OFFSET + 1 + VSYNC_WORDS;     // 2
const WR_CFG_OFFSET:  usize = PRE_ACT_OFFSET + 1 + PRE_ACT_WORDS; // 7
const HEADER_WORDS:   usize = WR_CFG_OFFSET + 1 + WR_CFG_WORDS;   // 40

const DATA_LATCH_STRIDE: usize = 1 + DATA_LATCH_WORDS;            // 33
const LATCHES_PER_LINE:  usize = 16;
const LATCHES_PER_FRAME: usize = SCAN_LINES * LATCHES_PER_LINE;   // 512

const DATA_OFFSET: usize = HEADER_WORDS;                          // 40
const DATA_END:    usize = DATA_OFFSET + LATCHES_PER_FRAME * DATA_LATCH_STRIDE; // 16 936

/// Cycles of full-frame scan after each data load. 50 is spwm_12's
/// baseline — yields ~65 Hz display refresh with the current data
/// phase duration.
const POST_SCAN_CYCLES: usize = 50;

const SCAN_OFFSET: usize = DATA_END;
const POST_SCAN_WORDS: usize = POST_SCAN_CYCLES * SCAN_LINES;

const FRAME_WORDS: usize = SCAN_OFFSET + POST_SCAN_WORDS; // 18 536

// ── Double-buffered frame data ───────────────────────────────────────
static mut FRAME_BUF_A: [u32; FRAME_WORDS] = [0u32; FRAME_WORDS];
static mut FRAME_BUF_B: [u32; FRAME_WORDS] = [0u32; FRAME_WORDS];

// ── Scan buffer for ring-mode DMA (sync-phase only) ──────────────────
#[repr(C, align(128))]
struct ScanBufAligned([u32; SCAN_LINES]);
static mut SCAN_BUF: ScanBufAligned = ScanBufAligned([0u32; SCAN_LINES]);

// ── Core 1 stack ─────────────────────────────────────────────────────
static mut CORE1_STACK: hal::multicore::Stack<4096> = hal::multicore::Stack::new();

// ── Scan parameters ──────────────────────────────────────────────────
const DISPLAY_CLK: u32 = 100;
const SETUP_CLK: u32 = 28;
const OE_CLK_W4: u32 = 4;
const OE_CLK_W12: u32 = 12;

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
    1
    | ((display - 1) << 1)
    | (addr << 14)
    | ((setup - 1) << 19)
    | ((oe - 1) << 24)
}

fn scan_word_for_sync(scan_line: usize) -> u32 {
    let oe = if scan_line == 0 { OE_CLK_W12 } else { OE_CLK_W4 };
    scan_word(DISPLAY_CLK, scan_line as u32, SETUP_CLK, oe)
}

// ── Offset helpers ──────────────────────────────────────────────────

const fn latch_header_offset(scan_line: usize, channel: usize) -> usize {
    let latch_idx = scan_line * LATCHES_PER_LINE + channel;
    DATA_OFFSET + latch_idx * DATA_LATCH_STRIDE
}

const fn post_scan_offset(cycle: usize, scan_line: usize) -> usize {
    SCAN_OFFSET + cycle * SCAN_LINES + scan_line
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

fn init_frame_headers(buf: &mut [u32; FRAME_WORDS]) {
    buf[VSYNC_OFFSET] = data_header(VSYNC_PRE, VSYNC_LAT);
    buf[VSYNC_OFFSET + 1] = 0;

    buf[PRE_ACT_OFFSET] = data_header(PRE_ACT_PRE, PRE_ACT_LAT);
    for i in 0..PRE_ACT_WORDS {
        buf[PRE_ACT_OFFSET + 1 + i] = 0;
    }

    buf[WR_CFG_OFFSET] = data_header(WR_CFG_PRE, WR_CFG_LAT);

    for scan_line in 0..SCAN_LINES {
        for channel in 0..LATCHES_PER_LINE {
            buf[latch_header_offset(scan_line, channel)] =
                data_header(DATA_LATCH_PRE, DATA_LATCH_LAT);
        }
    }

    for cycle in 0..POST_SCAN_CYCLES {
        for slot in 0..SCAN_LINES {
            let oe = if slot == 0 { OE_CLK_W12 } else { OE_CLK_W4 };
            buf[post_scan_offset(cycle, slot)] =
                scan_word(DISPLAY_CLK, slot as u32, SETUP_CLK, oe);
        }
    }
}

const PACK_SHIFT: isize = 0;

fn pack_pixels(buf: &mut [u32; FRAME_WORDS]) {
    let pixels = unsafe { &*core::ptr::addr_of!(PIXELS) };

    for scan_line in 0..SCAN_LINES {
        let source_line =
            (scan_line as isize + PACK_SHIFT).rem_euclid(SCAN_LINES as isize) as usize;
        let upper_row = source_line;
        let lower_row = source_line + 32;

        for channel in 0..LATCHES_PER_LINE {
            let base = latch_header_offset(scan_line, channel) + 1;
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

// ── Fill pixels — Thread A probe ────────────────────────────────────

fn fill_pixels(offset: u8) {
    let pixels = unsafe { &mut *core::ptr::addr_of_mut!(PIXELS) };
    for row in 0..64usize {
        for col in 0..128usize {
            pixels[row][col] = if MOVING {
                let hue = (row as u16 + col as u16 + offset as u16) as u8;
                hsv(hue)
            } else {
                Rgb::new(GREY_LEVEL, GREY_LEVEL, GREY_LEVEL)
            };
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
            cortex_m::asm::wfe();
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
        fifo_write(1);
    }
}

// ── PIO helpers ─────────────────────────────────────────────────────

const PIO0_BASE: u32 = 0x5020_0000;
const PIO0_CTRL_SET: u32 = PIO0_BASE + 0x2000;
const PIO0_CTRL_CLR: u32 = PIO0_BASE + 0x3000;
const PIO0_FDEBUG: u32 = PIO0_BASE + 0x008;
const SM0_CLKDIV: u32 = PIO0_BASE + 0x0C8;
const SM0_EN: u32 = 1 << 0;
const SM0_TXSTALL: u32 = 1 << 24;

fn enable_sm(mask: u32) {
    unsafe { (PIO0_CTRL_SET as *mut u32).write_volatile(mask) };
}
fn disable_sm(mask: u32) {
    unsafe { (PIO0_CTRL_CLR as *mut u32).write_volatile(mask) };
}

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

// ── DMA ring-mode scan (ch1, raw register access, sync-phase only) ──
const DMA_BASE: u32 = 0x5000_0000;
const CH1_READ_ADDR:  *mut u32 = (DMA_BASE + 0x040) as *mut u32;
const CH1_WRITE_ADDR: *mut u32 = (DMA_BASE + 0x044) as *mut u32;
const CH1_AL1_CTRL:   *mut u32 = (DMA_BASE + 0x050) as *mut u32;
const CH1_AL1_TRANS_COUNT_TRIG: *mut u32 = (DMA_BASE + 0x05C) as *mut u32;
const PIO0_TXF0: u32 = PIO0_BASE + 0x010;

const CH1_CTRL_VAL: u32 =
    (1 << 0)
    | (2 << 2)
    | (1 << 4)
    | (7 << 8)
    | (1 << 13)
    | (0 << 17);

fn start_ring_scan() {
    let scan_addr = core::ptr::addr_of!(SCAN_BUF) as u32;
    unsafe {
        CH1_AL1_CTRL.write_volatile(CH1_CTRL_VAL);
        CH1_READ_ADDR.write_volatile(scan_addr);
        CH1_WRITE_ADDR.write_volatile(PIO0_TXF0);
        CH1_AL1_TRANS_COUNT_TRIG.write_volatile((15 << 28) | 32);
    }
}

fn stop_ring_scan() {
    unsafe {
        const CHAN_ABORT: *mut u32 = (DMA_BASE + 0x464) as *mut u32;
        CHAN_ABORT.write_volatile(1 << 1);
        while CHAN_ABORT.read_volatile() & (1 << 1) != 0 {
            cortex_m::asm::nop();
        }
    }
}

fn build_scan_buf() {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(SCAN_BUF) };
    for row in 0..SCAN_LINES {
        buf.0[row] = scan_word_for_sync(row);
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

    enable_cycle_counter();

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

    const PADS_BANK0: u32 = 0x4003_8000;
    const GPIO12_PAD: u32 = PADS_BANK0 + 0x04 + 12 * 4;
    unsafe {
        let val = (GPIO12_PAD as *const u32).read_volatile();
        (GPIO12_PAD as *mut u32).write_volatile(val | (1 << 2));
    }

    // ── 3. Install unified PIO program (from spwm_12) ────────────────
    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let ss = pio::SideSet::new(false, 2, false);
    let mut a = pio::Assembler::new_with_side_set(ss);

    let mut dispatch    = a.label();
    let mut data_cmd    = a.label();
    let mut display_lp  = a.label();
    let mut setup_lp    = a.label();
    let mut oe_lp       = a.label();
    let mut pre_loop    = a.label();
    let mut lat_loop    = a.label();
    let mut wrap_source = a.label();

    a.bind(&mut dispatch);
    a.out_with_side_set(pio::OutDestination::X, 1, 0b00);
    a.jmp_with_side_set(pio::JmpCondition::XIsZero, &mut data_cmd, 0b00);

    a.out_with_side_set(pio::OutDestination::Y, 7, 0b00);
    a.bind(&mut display_lp);
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut display_lp, 0b01);

    a.out_with_side_set(pio::OutDestination::PINS, 11, 0b00);
    a.out_with_side_set(pio::OutDestination::Y, 5, 0b00);

    a.bind(&mut setup_lp);
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut setup_lp, 0b01);

    a.out_with_side_set(pio::OutDestination::Y, 4, 0b00);
    a.set_with_side_set(pio::SetDestination::PINS, 1, 0b00);

    a.bind(&mut oe_lp);
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut oe_lp, 0b01);

    a.set_with_side_set(pio::SetDestination::PINS, 0, 0b00);
    a.out_with_side_set(pio::OutDestination::NULL, 4, 0b00);
    a.jmp_with_side_set(pio::JmpCondition::Always, &mut dispatch, 0b00);

    a.bind(&mut data_cmd);
    a.out_with_side_set(pio::OutDestination::X, 15, 0b00);
    a.out_with_side_set(pio::OutDestination::Y, 16, 0b00);

    a.bind(&mut pre_loop);
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0b00);
    a.out_with_side_set(pio::OutDestination::NULL, 2, 0b01);
    a.jmp_with_side_set(pio::JmpCondition::XDecNonZero, &mut pre_loop, 0b00);

    a.bind(&mut lat_loop);
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0b10);
    a.out_with_side_set(pio::OutDestination::NULL, 2, 0b11);
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut lat_loop, 0b10);
    a.bind(&mut wrap_source);

    let prog = a.assemble_with_wrap(wrap_source, dispatch);
    let installed = pio0.install(&prog).unwrap();

    let (mut sm, _, tx) =
        hal::pio::PIOBuilder::from_installed_program(installed)
            .buffers(hal::pio::Buffers::OnlyTx)
            .out_pins(0, 11)
            .set_pins(13, 1)
            .side_set_pin_base(11)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .autopull(true)
            .pull_threshold(32)
            .clock_divisor_fixed_point(3, 0)
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

    // ── 4. DMA — single channel ────────────────────────────────────
    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut ch = dma.ch0;

    defmt::info!(
        "brightness_12 (split-DMA, unified) — MOVING={=bool} GREY_LEVEL={=u8}",
        MOVING, GREY_LEVEL,
    );

    // ── 5. Init frame headers ──────────────────────────────────────
    init_frame_headers(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) });
    init_frame_headers(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_B) });

    // ── 6. Startup flush (verbatim from spwm_12) ────────────────────
    const FLUSH_FRAMES: u32 = 26;
    for flush in 0..FLUSH_FRAMES {
        if (flush as usize) < CONFIG_REGS.len() {
            update_wr_cfg(
                unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) },
                flush as usize,
            );
        }

        set_clkdiv(2);
        let data_slice = unsafe {
            core::slice::from_raw_parts(
                core::ptr::addr_of!(FRAME_BUF_A) as *const u32,
                DATA_END,
            )
        };
        let cfg = single_buffer::Config::new(ch, data_slice, tx);
        let xfer = cfg.start();
        let (new_ch, _, new_tx) = xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);

        set_clkdiv(3);
        let scan_slice = unsafe {
            let base = (core::ptr::addr_of!(FRAME_BUF_A) as *const u32).add(SCAN_OFFSET);
            core::slice::from_raw_parts(base, SCAN_LINES)
        };
        let scan_cfg = single_buffer::Config::new(ch, scan_slice, tx);
        let scan_xfer = scan_cfg.start();
        let (new_ch2, _, new_tx2) = scan_xfer.wait();
        ch = new_ch2;
        tx = new_tx2;
        wait_txstall(SM0_TXSTALL);
    }

    defmt::info!("flush complete");

    // ── 6b. Sync phase (verbatim from spwm_12) ──────────────────────
    build_scan_buf();
    let _ch1 = dma.ch1;

    set_clkdiv(3);
    start_ring_scan();

    const SYNC_FRAMES: u32 = 60;
    for _ in 0..SYNC_FRAMES {
        cortex_m::asm::delay(150_000 * 10);

        stop_ring_scan();
        wait_txstall(SM0_TXSTALL);

        set_clkdiv(2);
        let data_slice = unsafe {
            core::slice::from_raw_parts(
                core::ptr::addr_of!(FRAME_BUF_A) as *const u32,
                DATA_END,
            )
        };
        let cfg = single_buffer::Config::new(ch, data_slice, tx);
        let xfer = cfg.start();
        let (new_ch, _, new_tx) = xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);
        set_clkdiv(3);

        start_ring_scan();
    }

    stop_ring_scan();
    wait_txstall(SM0_TXSTALL);
    defmt::info!("sync phase complete");

    // Seed FRAME_BUF_B's WR_CFG payload to match A's last value.
    update_wr_cfg(
        unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_B) },
        CONFIG_REGS.len() - 1,
    );

    // ── 7. Pack first real frame and spawn core 1 ───────────────────
    fill_pixels(0);
    pack_pixels(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) });

    let mut mc = hal::multicore::Multicore::new(
        &mut pac.PSM, &mut pac.PPB, &mut sio_hal.fifo,
    );
    let cores = mc.cores();
    cores[1].spawn(unsafe { CORE1_STACK.take().unwrap() }, core1_task).unwrap();

    let mut active_buf: u8 = 0;
    let mut core1_buf: u8 = 1;
    let mut offset: u8 = 0;

    offset = offset.wrapping_add(2);
    fifo_write(offset as u32 | ((core1_buf as u32) << 8));

    // Timing accumulators (cycles).
    let mut scan_cy_total: u64 = 0;
    let mut data_cy_total: u64 = 0;
    let mut scan_count: u32 = 0;
    let mut data_count: u32 = 0;
    let mut last_report = now_cycles();

    // Split-DMA main loop. Each iteration:
    //   DMA 1: header + DATA_LATCHes  (~2.7 ms dark)
    //   DMA 2: POST_SCAN_CYCLES × 32 scan words  (~12.6 ms bright)
    loop {
        let buf_ptr = if active_buf == 0 {
            core::ptr::addr_of!(FRAME_BUF_A) as *const u32
        } else {
            core::ptr::addr_of!(FRAME_BUF_B) as *const u32
        };

        // DMA 1: data phase.
        let t_data0 = now_cycles();
        set_clkdiv(2);
        enable_sm(SM0_EN);
        let data_slice = unsafe { core::slice::from_raw_parts(buf_ptr, DATA_END) };
        let cfg = single_buffer::Config::new(ch, data_slice, tx);
        let xfer = cfg.start();
        let (new_ch, _, new_tx) = xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);
        disable_sm(SM0_EN);
        data_cy_total += now_cycles().wrapping_sub(t_data0) as u64;
        data_count += 1;

        // DMA 2: scan phase, POST_SCAN_CYCLES back-to-back 32-word DMAs.
        set_clkdiv(3);
        enable_sm(SM0_EN);
        for cycle in 0..POST_SCAN_CYCLES {
            let t_scan0 = now_cycles();
            let scan_slice = unsafe {
                core::slice::from_raw_parts(
                    buf_ptr.add(SCAN_OFFSET + cycle * SCAN_LINES),
                    SCAN_LINES,
                )
            };
            let cfg = single_buffer::Config::new(ch, scan_slice, tx);
            let xfer = cfg.start();
            let (new_ch, _, new_tx) = xfer.wait();
            ch = new_ch;
            tx = new_tx;
            wait_txstall(SM0_TXSTALL);
            scan_cy_total += now_cycles().wrapping_sub(t_scan0) as u64;
            scan_count += 1;
        }
        disable_sm(SM0_EN);

        if fifo_try_read().is_some() {
            active_buf = core1_buf;
            core1_buf = 1 - active_buf;
            offset = offset.wrapping_add(2);
            fifo_write(offset as u32 | ((core1_buf as u32) << 8));
        }

        let now = now_cycles();
        if now.wrapping_sub(last_report) >= SYS_CLK_MHZ * 1_000_000 {
            let scan_avg_us = if scan_count > 0 {
                (scan_cy_total / (SYS_CLK_MHZ as u64 * scan_count as u64)) as u32
            } else { 0 };
            let data_avg_us = if data_count > 0 {
                (data_cy_total / (SYS_CLK_MHZ as u64 * data_count as u64)) as u32
            } else { 0 };
            let total = scan_cy_total + data_cy_total;
            let dark_permille = if total > 0 {
                (data_cy_total * 1000 / total) as u32
            } else { 0 };
            defmt::info!(
                "scan: {=u32}×{=u32}µs  data: {=u32}×{=u32}µs  dark: {=u32}‰",
                scan_count, scan_avg_us, data_count, data_avg_us, dark_permille,
            );
            scan_cy_total = 0;
            data_cy_total = 0;
            scan_count = 0;
            data_count = 0;
            last_report = now;
        }
    }
}
