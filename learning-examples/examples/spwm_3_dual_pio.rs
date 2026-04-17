//! # DP3364S S-PWM — Step 3: Dual PIO with Overlapped Packing
//!
//! Third in the three-part progression. Builds on `spwm_2_pio_cpu.rs`.
//!
//! **What this step adds:** a second PIO state machine (SM1) drives the
//! scan loop — CLK + ROW (OE) pulses — eliminating the CPU bit-bang
//! scan and pin-function switching from step 2. SM0 and SM1 share
//! GPIO 11 (CLK), so a pindir handover protocol transfers ownership
//! between them each frame. A startup flush sequence sends 14 black
//! frames (with WR_CFG) to initialise the driver chips cleanly.
//!
//! The key benefit: packing the next frame overlaps with scanning the
//! current one. In step 2, the display goes dark while the CPU packs
//! because packing and scanning are sequential on the same core. Here,
//! SM1 keeps the display lit while the CPU fills the pixel buffer and
//! packs the DMA buffer — resulting in a brighter image and less flicker.
//!
//! ## Run
//!
//! ```sh
//! cargo run --release --example spwm_3_dual_pio
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

static mut FRAME_BUF: [u32; FRAME_WORDS] = [0u32; FRAME_WORDS];

// ── Scan parameters ──────────────────────────────────────────────────
const DISPLAY_CYCLES: usize = 10;
const SCAN_ITERS: usize = DISPLAY_CYCLES * SCAN_LINES;

const DISPLAY_CLK: u32 = 100;
const SETUP_CLK: u32 = 28;
const OE_CLK_W4: u32 = 4;
const OE_CLK_W12: u32 = 12;

static mut SCAN_BUF: [u32; SCAN_ITERS] = [0u32; SCAN_ITERS];

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

// ── Scan buffer builder ─────────────────────────────────────────────

fn build_scan_buf() {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(SCAN_BUF) };
    for i in 0..SCAN_ITERS {
        let row = (i % SCAN_LINES) as u32;
        let oe = if row == 0 { OE_CLK_W12 } else { OE_CLK_W4 };
        buf[i] = (DISPLAY_CLK - 1)
            | (row << 7)
            | ((SETUP_CLK - 1) << 12)
            | ((oe - 1) << 17);
    }
}

// ── Frame-buffer packing ─────────────────────────────────────────────

const fn header(n_pre: u32, n_lat: u32) -> u32 {
    ((n_lat - 1) << 16) | (n_pre - 1)
}

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
/// FRAME_BUF. Called once at startup; DATA_LATCH headers are written
/// by `pack_pixels()`.
fn init_frame_headers() {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF) };
    buf[VSYNC_OFFSET] = header(VSYNC_PRE, VSYNC_LAT);
    buf[VSYNC_OFFSET + 1] = 0;
    buf[PRE_ACT_OFFSET] = header(PRE_ACT_PRE, PRE_ACT_LAT);
    for i in 0..PRE_ACT_WORDS {
        buf[PRE_ACT_OFFSET + 1 + i] = 0;
    }
    buf[WR_CFG_OFFSET] = header(WR_CFG_PRE, WR_CFG_LAT);
    for l in 0..LATCHES_PER_FRAME {
        buf[DATA_LATCH_OFFSET + l * DATA_LATCH_STRIDE] =
            header(DATA_LATCH_PRE, DATA_LATCH_LAT);
    }
}

/// Pack the PIXELS framebuffer into FRAME_BUF's DATA_LATCH regions.
///
/// Gamma-corrected greyscale is precomputed per column (8 chips x
/// 6 channels = 48 lookups per latch) rather than per pixel-clock
/// (128 x 6 = 768), cutting gamma lookups from 393 K to 24 K.
fn pack_pixels() {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF) };
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

                    let bits = ((ur[chip] >> bit_pos) & 1) as u32
                        | (((ug[chip] >> bit_pos) & 1) as u32) << 1
                        | (((ub[chip] >> bit_pos) & 1) as u32) << 2
                        | (((lr[chip] >> bit_pos) & 1) as u32) << 3
                        | (((lg[chip] >> bit_pos) & 1) as u32) << 4
                        | (((lb[chip] >> bit_pos) & 1) as u32) << 5;
                    acc |= bits << (slot * 8);
                }
                buf[base + w] = acc;
            }
        }
    }
}

fn update_wr_cfg(reg_idx: usize) {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF) };
    let reg = CONFIG_REGS[reg_idx];
    pack_shift128(
        &mut buf[WR_CFG_OFFSET + 1 .. WR_CFG_OFFSET + 1 + WR_CFG_WORDS],
        reg, ALL_RGB,
    );
}

// ── SM enable/disable + pindir handover ─────────────────────────────

const PIO0_BASE: u32 = 0x5020_0000;
const PIO0_CTRL_SET: u32 = PIO0_BASE + 0x2000;
const PIO0_CTRL_CLR: u32 = PIO0_BASE + 0x3000;
const PIO0_FDEBUG: u32 = PIO0_BASE + 0x008;
const SM0_PINCTRL: u32 = PIO0_BASE + 0x0DC;
const SM1_PINCTRL: u32 = PIO0_BASE + 0x0F4;
const SM0_INSTR: u32 = PIO0_BASE + 0x0D8;
const SM1_INSTR: u32 = PIO0_BASE + 0x0F0;
const SM0_EN: u32 = 1 << 0;
const SM1_EN: u32 = 1 << 1;
const SM0_TXSTALL: u32 = 1 << 24;
const SM1_TXSTALL: u32 = 1 << 25;

const SET_PINDIRS_0: u32 = 0xE080;
const SET_PINDIRS_1: u32 = 0xE081;
const SET_PINDIRS_3: u32 = 0xE083;

const PINCTRL_SET_COUNT_MASK: u32 = 0x7 << 26;
const PINCTRL_SET_BASE_MASK: u32 = 0x1F << 5;

fn enable_sm(mask: u32) {
    unsafe { (PIO0_CTRL_SET as *mut u32).write_volatile(mask) };
}
fn disable_sm(mask: u32) {
    unsafe { (PIO0_CTRL_CLR as *mut u32).write_volatile(mask) };
}

fn wait_txstall(sm_stall_bit: u32) {
    unsafe {
        (PIO0_FDEBUG as *mut u32).write_volatile(sm_stall_bit);
        while (PIO0_FDEBUG as *const u32).read_volatile() & sm_stall_bit == 0 {
            cortex_m::asm::nop();
        }
    }
}

fn force_set_pindirs(pinctrl_addr: u32, instr_addr: u32, base: u32, count: u32, instr: u32) {
    unsafe {
        let saved = (pinctrl_addr as *const u32).read_volatile();
        let modified = (saved & !(PINCTRL_SET_COUNT_MASK | PINCTRL_SET_BASE_MASK))
            | (count << 26) | (base << 5);
        (pinctrl_addr as *mut u32).write_volatile(modified);
        (instr_addr as *mut u32).write_volatile(instr);
        (pinctrl_addr as *mut u32).write_volatile(saved);
    }
}

fn release_sm0_clk_lat() {
    force_set_pindirs(SM0_PINCTRL, SM0_INSTR, 11, 2, SET_PINDIRS_0);
}
fn claim_sm0_clk_lat() {
    force_set_pindirs(SM0_PINCTRL, SM0_INSTR, 11, 2, SET_PINDIRS_3);
}
fn release_sm1_clk() {
    force_set_pindirs(SM1_PINCTRL, SM1_INSTR, 11, 1, SET_PINDIRS_0);
}
fn claim_sm1_clk() {
    force_set_pindirs(SM1_PINCTRL, SM1_INSTR, 11, 1, SET_PINDIRS_1);
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

    // ── 3. Pre-build the scan buffer ─────────────────────────────────
    build_scan_buf();

    // ── 4. Install both PIO programs ─────────────────────────────────
    let (mut pio0, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // ── SM0: data path ───────────────────────────────────────────────
    let data_ss = pio::SideSet::new(false, 2, false);
    let mut da = pio::Assembler::new_with_side_set(data_ss);
    let mut d_wrap_target = da.label();
    let mut d_wrap_source = da.label();
    let mut d_pre_loop = da.label();
    let mut d_lat_loop = da.label();

    da.bind(&mut d_wrap_target);
    da.out_with_side_set(pio::OutDestination::X, 16, 0b00);
    da.out_with_side_set(pio::OutDestination::Y, 16, 0b00);
    da.bind(&mut d_pre_loop);
    da.out_with_side_set(pio::OutDestination::PINS, 6, 0b00);
    da.out_with_side_set(pio::OutDestination::NULL, 2, 0b01);
    da.jmp_with_side_set(
        pio::JmpCondition::XDecNonZero, &mut d_pre_loop, 0b00,
    );
    da.bind(&mut d_lat_loop);
    da.out_with_side_set(pio::OutDestination::PINS, 6, 0b10);
    da.out_with_side_set(pio::OutDestination::NULL, 2, 0b11);
    da.jmp_with_side_set(
        pio::JmpCondition::YDecNonZero, &mut d_lat_loop, 0b10,
    );
    da.bind(&mut d_wrap_source);
    let data_prog = da.assemble_with_wrap(d_wrap_source, d_wrap_target);
    let data_installed = pio0.install(&data_prog).unwrap();

    let (mut data_sm, _, data_tx) =
        hal::pio::PIOBuilder::from_installed_program(data_installed)
            .buffers(hal::pio::Buffers::OnlyTx)
            .out_pins(0, 6)
            .side_set_pin_base(11)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .autopull(true)
            .pull_threshold(32)
            .clock_divisor_fixed_point(2, 0)
            .build(sm0);

    data_sm.set_pindirs([
        (0,  hal::pio::PinDir::Output), (1,  hal::pio::PinDir::Output),
        (2,  hal::pio::PinDir::Output), (3,  hal::pio::PinDir::Output),
        (4,  hal::pio::PinDir::Output), (5,  hal::pio::PinDir::Output),
        (11, hal::pio::PinDir::Output), (12, hal::pio::PinDir::Output),
    ]);
    let _data_sm = data_sm.start();

    // ── SM1: scan path ───────────────────────────────────────────────
    let scan_ss = pio::SideSet::new(false, 1, false);
    let mut sa = pio::Assembler::new_with_side_set(scan_ss);
    let mut s_wrap_target = sa.label();
    let mut s_wrap_source = sa.label();
    let mut s_display = sa.label();
    let mut s_setup = sa.label();
    let mut s_oe = sa.label();

    sa.bind(&mut s_wrap_target);
    sa.out_with_side_set(pio::OutDestination::Y, 7, 0);
    sa.bind(&mut s_display);
    sa.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    sa.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    sa.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut s_display, 1);
    sa.out_with_side_set(pio::OutDestination::PINS, 5, 0);
    sa.out_with_side_set(pio::OutDestination::Y, 5, 0);
    sa.bind(&mut s_setup);
    sa.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    sa.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    sa.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut s_setup, 1);
    sa.out_with_side_set(pio::OutDestination::Y, 4, 0);
    sa.set_with_side_set(pio::SetDestination::PINS, 1, 0);
    sa.bind(&mut s_oe);
    sa.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    sa.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    sa.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut s_oe, 1);
    sa.set_with_side_set(pio::SetDestination::PINS, 0, 0);
    sa.out_with_side_set(pio::OutDestination::NULL, 11, 0);
    sa.bind(&mut s_wrap_source);
    let scan_prog = sa.assemble_with_wrap(s_wrap_source, s_wrap_target);
    let scan_installed = pio0.install(&scan_prog).unwrap();

    let (mut scan_sm, _, scan_tx) =
        hal::pio::PIOBuilder::from_installed_program(scan_installed)
            .buffers(hal::pio::Buffers::OnlyTx)
            .out_pins(6, 5)
            .set_pins(13, 1)
            .side_set_pin_base(11)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .autopull(true)
            .pull_threshold(32)
            .clock_divisor_fixed_point(3, 0)
            .build(sm1);

    scan_sm.set_pindirs([
        (6,  hal::pio::PinDir::Output), (7,  hal::pio::PinDir::Output),
        (8,  hal::pio::PinDir::Output), (9,  hal::pio::PinDir::Output),
        (10, hal::pio::PinDir::Output), (11, hal::pio::PinDir::Output),
        (13, hal::pio::PinDir::Output),
    ]);
    let _scan_sm = scan_sm.start();
    disable_sm(SM1_EN);

    // ── 5. DMA — ch0 feeds SM0, ch1 feeds SM1 ────────────────────────
    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut data_ch = dma.ch0;
    let mut scan_ch = dma.ch1;
    let mut data_tx = data_tx;
    let mut scan_tx = scan_tx;

    defmt::info!("DP3364S S-PWM (dual PIO, per-pixel colour) starting");

    // ── 6. Init frame headers + startup flush ──────────────────────
    init_frame_headers();
    // PIXELS is all-black from static init; pack it for flush frames.
    pack_pixels();
    for flush in 0..14u32 {
        if (flush as usize) < CONFIG_REGS.len() {
            update_wr_cfg(flush as usize);
        }
        let buf = unsafe { &*core::ptr::addr_of!(FRAME_BUF) };
        let cfg = single_buffer::Config::new(data_ch, buf, data_tx);
        let xfer = cfg.start();
        let (ch, _, new_tx) = xfer.wait();
        data_ch = ch;
        data_tx = new_tx;
        wait_txstall(SM0_TXSTALL);

        release_sm0_clk_lat();
        disable_sm(SM0_EN);
        claim_sm1_clk();
        let scan = unsafe { &*core::ptr::addr_of!(SCAN_BUF) };
        let scan_cfg = single_buffer::Config::new(scan_ch, scan, scan_tx);
        let scan_xfer = scan_cfg.start();
        enable_sm(SM1_EN);
        let (ch, _, new_tx) = scan_xfer.wait();
        scan_ch = ch;
        scan_tx = new_tx;
        wait_txstall(SM1_TXSTALL);
        release_sm1_clk();
        disable_sm(SM1_EN);
        claim_sm0_clk_lat();
        enable_sm(SM0_EN);
    }
    defmt::info!("flush complete");

    // ── 7. Main loop — scrolling HSV rainbow ─────────────────────
    //
    // Each iteration:
    //   1. Data phase: DMA the packed frame to the chip (SM0)
    //   2. Handover CLK to SM1
    //   3. Start scan DMA (SM1 begins displaying)
    //   4. WHILE SM1 scans, CPU fills + packs the NEXT frame
    //   5. Wait for scan to finish, hand CLK back to SM0
    //
    // Packing overlaps with scanning, so the display stays lit
    // during the CPU-intensive pack step. No blanking.
    let mut offset: u8 = 0;

    // Scrolling HSV rainbow.
    {
        let pixels = unsafe { &mut *core::ptr::addr_of_mut!(PIXELS) };
        for row in 0..64usize {
            for col in 0..128usize {
                let hue = (col as u32 * 255 / 127) as u8;
                pixels[row][col] = hsv(hue);
            }
        }
    }
    pack_pixels();

    loop {
        // ── Data phase (full buffer including WR_CFG) ────────────
        let buf = unsafe { &*core::ptr::addr_of!(FRAME_BUF) };
        let cfg1 = single_buffer::Config::new(data_ch, buf, data_tx);
        let xfer1 = cfg1.start();
        let (ch, _, new_tx) = xfer1.wait();
        data_ch = ch;
        data_tx = new_tx;
        wait_txstall(SM0_TXSTALL);

        // ── Handover to scan ─────────────────────────────────────
        release_sm0_clk_lat();
        disable_sm(SM0_EN);
        claim_sm1_clk();
        let scan = unsafe { &*core::ptr::addr_of!(SCAN_BUF) };
        let scan_cfg = single_buffer::Config::new(scan_ch, scan, scan_tx);
        let scan_xfer = scan_cfg.start();
        enable_sm(SM1_EN);

        // ── CPU work WHILE SM1 scans: fill + pack next frame ─────
        offset = offset.wrapping_add(2);
        {
            let pixels = unsafe { &mut *core::ptr::addr_of_mut!(PIXELS) };
            for row in 0..64usize {
                for col in 0..128usize {
                    let hue = (col as u32 * 255 / 127) as u8;
                    pixels[row][col] = hsv(hue.wrapping_add(offset));
                }
            }
        }
        pack_pixels();

        // ── Wait for scan to finish ──────────────────────────────
        let (ch, _, new_tx) = scan_xfer.wait();
        scan_ch = ch;
        scan_tx = new_tx;
        wait_txstall(SM1_TXSTALL);

        // ── Handover back to data ────────────────────────────────
        release_sm1_clk();
        disable_sm(SM1_EN);
        claim_sm0_clk_lat();
        enable_sm(SM0_EN);
    }
}
