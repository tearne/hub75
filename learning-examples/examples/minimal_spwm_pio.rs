//! # DP3364S S-PWM — Unified PIO + DMA for All Four Commands
//!
//! Third-stage implementation of the DP3364S driver. Where
//! `minimal_spwm_dma.rs` put only DATA_LATCH under PIO and kept
//! VSYNC / PRE_ACT / WR_CFG on the CPU, this example unifies all
//! four S-PWM commands under a single PIO program. A whole frame is
//! issued as one DMA transfer; the CPU only runs `display_loop`
//! (ROW + CLK pulses) between frames.
//!
//! ## What changed vs. `minimal_spwm_dma`
//!
//! The DATA_LATCH-only PIO program hard-coded LAT high on the 128th
//! CLK. The other S-PWM commands (LAT widths 3, 5, 14) could not go
//! through it, so they stayed CPU-bitbanged and the RGB+CLK+LAT pins
//! had to be switched between PIO and SIO twice per frame.
//!
//! Here the PIO program takes a 32-bit control header at the start of
//! every command describing how many CLKs to emit with LAT low and how
//! many with LAT high. A single stream of headers + data drives all
//! four commands at line rate, and LAT stays under PIO for the entire
//! program lifetime. Only CLK and RGB pins still flip to SIO — and
//! only once per frame, for the display phase.
//!
//! ## PIO program (8 instructions)
//!
//! 2-bit sideset on GPIO 11–12 (CLK, LAT). Autopull = 32, LSB-first.
//!
//! ```text
//! .side_set 2
//! .wrap_target
//!     out   x, 16        side 0b00   ; X <- N_pre  - 1   (low 16 bits of header)
//!     out   y, 16        side 0b00   ; Y <- N_lat  - 1   (high 16 bits of header)
//! pre_loop:
//!     out   pins, 6      side 0b00   ; LAT low — shift a data pixel
//!     out   null, 2      side 0b01   ; CLK rising edge, LAT still low
//!     jmp   x--, pre_loop side 0b00
//! lat_loop:
//!     out   pins, 6      side 0b10   ; LAT high — shift a data pixel
//!     out   null, 2      side 0b11   ; CLK rising edge + LAT high → command
//!     jmp   y--, lat_loop side 0b10
//! .wrap                               ; next `out x, 16 side 0b00` drops LAT
//! ```
//!
//! The fall-through from pre_loop into lat_loop is the LAT rising
//! edge; the wrap from lat_loop back to the header `out`s is the LAT
//! falling edge. Between LAT-phase iterations, sideset stays at 0b10
//! (LAT high) so the chip sees one continuous LAT pulse of the
//! specified width.
//!
//! ## Command parameters
//!
//! All four S-PWM commands fit in the same program. `N_pre` ≥ 1 and
//! `N_lat` ≥ 1 are required (the loop structure can't handle zero
//! iterations in either phase).
//!
//! ```text
//!                  N_pre  N_lat  Total CLKs  Data words (32 bits each)
//!   VSYNC             1      3        4            1
//!   PRE_ACT           2     14       16            4
//!   WR_CFG          123      5      128           32
//!   DATA_LATCH      127      1      128           32
//! ```
//!
//! Every command's total CLK count is a multiple of 4 (= 32 bits), so
//! the data portion fits in whole 32-bit words and autopull stays
//! aligned with the header reads at the start of the next command.
//!
//! PRE_ACT in `minimal_spwm_cpu` uses 0 LAT-low CLKs before its 14 LAT
//! CLKs — we split that as 2 LAT-low + 14 LAT-high here, framed by the
//! LAT-low idle time of the preceding and following commands. From the
//! chip's point of view LAT is still high for exactly 14 consecutive
//! CLK rising edges, which is all the command decoder looks at.
//!
//! ## Frame buffer
//!
//! One u32 header + N u32 data words per command, 512 DATA_LATCHes per
//! frame. Layout:
//!
//! ```text
//!   [VSYNC: 1 hdr + 1 data]           2 words
//!   [PRE_ACT: 1 hdr + 4 data]         5 words
//!   [WR_CFG: 1 hdr + 32 data]        33 words
//!   [DATA_LATCH × 512: 33 each]   16 896 words
//!   ----------------------------------------
//!                    total          16 936 words ≈ 68 KB
//! ```
//!
//! The 512 DATA_LATCH blocks are filled once per colour change (every
//! 300 frames). Only the WR_CFG block is re-packed per frame, so the
//! amortised CPU work for buffer maintenance is 33 u32 writes/frame.
//!
//! ## Pin handover
//!
//! GPIO 12 (LAT) stays under PIO forever; no CPU code touches it.
//! GPIO 0–5 (RGB) and GPIO 11 (CLK) still flip SIO ↔ PIO0 around
//! `display_loop`, which CPU-bitbangs ROW + CLK pulses.
//!
//! ## Run
//!
//! ```sh
//! cargo run --release --example minimal_spwm_pio
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
// PANEL_WIDTH (128) and CHIPS_PER_CHAIN (8) are implicit in the
// per-command word counts below: one 128-bit shift per DATA_LATCH
// equals 32 u32 words of packed RGB+padding.
const SCAN_LINES: usize = 32;
const ROW_PERIOD: usize = 128;

// ── DP3264S / DP3364S configuration registers ────────────────────────
const CONFIG_REGS: [u16; 13] = [
    0x1100, 0x021F, 0x033F, 0x043F, 0x0504, 0x0642, 0x0700,
    0x08BF, 0x0960, 0x0ABE, 0x0B8B, 0x0C88, 0x0D12,
];

// ── Per-command CLK counts ───────────────────────────────────────────
//
// See module doc for the derivation. `WORDS_PER_COMMAND` counts the
// data (not header); each command is one header u32 plus that many
// data u32s.
const VSYNC_PRE: u32 = 1;    const VSYNC_LAT: u32 = 3;
const VSYNC_WORDS: usize = 1;

const PRE_ACT_PRE: u32 = 2;  const PRE_ACT_LAT: u32 = 14;
const PRE_ACT_WORDS: usize = 4;

const WR_CFG_PRE: u32 = 123; const WR_CFG_LAT: u32 = 5;
const WR_CFG_WORDS: usize = 32;

const DATA_LATCH_PRE: u32 = 127; const DATA_LATCH_LAT: u32 = 1;
const DATA_LATCH_WORDS: usize = 32;

// Byte offsets into FRAME_BUF for each command's start (header).
const VSYNC_OFFSET:      usize = 0;
const PRE_ACT_OFFSET:    usize = VSYNC_OFFSET + 1 + VSYNC_WORDS;      // 2
const WR_CFG_OFFSET:     usize = PRE_ACT_OFFSET + 1 + PRE_ACT_WORDS;  // 7
const DATA_LATCH_OFFSET: usize = WR_CFG_OFFSET + 1 + WR_CFG_WORDS;    // 40

const LATCHES_PER_FRAME: usize = SCAN_LINES * 16; // 512
const DATA_LATCH_STRIDE: usize = 1 + DATA_LATCH_WORDS; // 33

const FRAME_WORDS: usize =
    DATA_LATCH_OFFSET + LATCHES_PER_FRAME * DATA_LATCH_STRIDE; // 16 936

static mut FRAME_BUF: [u32; FRAME_WORDS] = [0u32; FRAME_WORDS];

// ── Frame-buffer packing ─────────────────────────────────────────────

/// Pack a command header: low 16 bits = N_pre − 1, high 16 = N_lat − 1.
const fn header(n_pre: u32, n_lat: u32) -> u32 {
    ((n_lat - 1) << 16) | (n_pre - 1)
}

/// Fill 32 words (= 128 pixels = one 16-bit word shifted through 8
/// chips) with the bit pattern of `word` on the lines named by
/// `color_mask`. MSB of `word` goes out first, repeating per chip.
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

/// Build the full frame buffer for a given colour / brightness. The
/// WR_CFG header+data gets rewritten per frame by `update_wr_cfg`; the
/// rest of the buffer only needs rebuilding on a colour change.
fn build_frame(brightness: u16, color_mask: u32) {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF) };

    // VSYNC — 1 header + 1 data word of zeros.
    buf[VSYNC_OFFSET] = header(VSYNC_PRE, VSYNC_LAT);
    buf[VSYNC_OFFSET + 1] = 0;

    // PRE_ACT — 1 header + 4 data words of zeros.
    buf[PRE_ACT_OFFSET] = header(PRE_ACT_PRE, PRE_ACT_LAT);
    for i in 0..PRE_ACT_WORDS {
        buf[PRE_ACT_OFFSET + 1 + i] = 0;
    }

    // WR_CFG — header written here; data payload re-written each
    // frame by update_wr_cfg() with the current register entry.
    buf[WR_CFG_OFFSET] = header(WR_CFG_PRE, WR_CFG_LAT);

    // DATA_LATCH × 512 — every latch gets the same pattern.
    for l in 0..LATCHES_PER_FRAME {
        let hdr = DATA_LATCH_OFFSET + l * DATA_LATCH_STRIDE;
        buf[hdr] = header(DATA_LATCH_PRE, DATA_LATCH_LAT);
        pack_shift128(
            &mut buf[hdr + 1 .. hdr + 1 + DATA_LATCH_WORDS],
            brightness, color_mask,
        );
    }
}

/// Rewrite only the WR_CFG payload to carry the next register entry.
/// Called once per refresh frame as `reg_idx` advances.
fn update_wr_cfg(reg_idx: usize) {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF) };
    let reg = CONFIG_REGS[reg_idx];
    pack_shift128(
        &mut buf[WR_CFG_OFFSET + 1 .. WR_CFG_OFFSET + 1 + WR_CFG_WORDS],
        reg, ALL_RGB,
    );
}

// ── Pin-function switching ───────────────────────────────────────────
//
// GPIO 12 (LAT) stays under PIO for the whole program; GPIO 0–5 (RGB)
// and GPIO 11 (CLK) flip between SIO (for CPU-bitbanged display_loop)
// and PIO0 (for the DMA-driven command stream).
const IO_BANK0_BASE: u32 = 0x4002_8000;
const FUNC_SIO: u32 = 5;
const FUNC_PIO0: u32 = 6;
const SHARED_PINS: [u32; 7] = [0, 1, 2, 3, 4, 5, 11];

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

// ── SIO bitbang helpers (for display_loop only) ──────────────────────
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
    // All pins start as SIO outputs. GPIO 12 (LAT) is immediately
    // redirected to PIO and left there for the whole program; the
    // seven other shared pins flip back and forth.
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

    // LAT: PIO owns it for the program lifetime.
    set_pin_func(12, FUNC_PIO0);

    // ── 3. PIO program (8 instructions) ──────────────────────────────
    let ss = pio::SideSet::new(false, 2, false);
    let mut a = pio::Assembler::new_with_side_set(ss);
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    let mut pre_loop = a.label();
    let mut lat_loop = a.label();

    a.bind(&mut wrap_target);
    a.out_with_side_set(pio::OutDestination::X, 16, 0b00);   // X = N_pre - 1
    a.out_with_side_set(pio::OutDestination::Y, 16, 0b00);   // Y = N_lat - 1

    a.bind(&mut pre_loop);
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0b00); // CLK low, LAT low
    a.out_with_side_set(pio::OutDestination::NULL, 2, 0b01); // CLK high
    a.jmp_with_side_set(
        pio::JmpCondition::XDecNonZero, &mut pre_loop, 0b00,
    );

    a.bind(&mut lat_loop);
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0b10); // CLK low, LAT high
    a.out_with_side_set(pio::OutDestination::NULL, 2, 0b11); // CLK high, LAT high
    a.jmp_with_side_set(
        pio::JmpCondition::YDecNonZero, &mut lat_loop, 0b10,
    );
    a.bind(&mut wrap_source);

    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio0.install(&program).unwrap();

    let (mut sm, _, tx) = hal::pio::PIOBuilder::from_installed_program(installed)
        .buffers(hal::pio::Buffers::OnlyTx)
        .out_pins(0, 6)
        .side_set_pin_base(11)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .autopull(true)
        .pull_threshold(32)
        .clock_divisor_fixed_point(2, 0)
        .build(sm0);

    sm.set_pindirs([
        (0,  hal::pio::PinDir::Output), (1,  hal::pio::PinDir::Output),
        (2,  hal::pio::PinDir::Output), (3,  hal::pio::PinDir::Output),
        (4,  hal::pio::PinDir::Output), (5,  hal::pio::PinDir::Output),
        (11, hal::pio::PinDir::Output), (12, hal::pio::PinDir::Output),
    ]);

    // No one-time Y preload — every command carries its own X/Y header.
    let _sm = sm.start();

    // ── 4. DMA ───────────────────────────────────────────────────────
    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut dma_ch = dma.ch0;
    let mut tx = tx;

    defmt::info!("DP3364S S-PWM (unified PIO+DMA) starting");

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
    // Config registers are written once each during the first 13
    // frames, then dropped from the stream. `cfg_written` climbs from
    // 0 to 13 and then stays; after that the DMA path skips the
    // WR_CFG block entirely via a two-kick transfer.
    let mut cfg_written: usize = 0;

    loop {
        let (brightness, color_mask, name) = tests[test_idx % tests.len()];
        defmt::info!("{}", name);
        build_frame(brightness, color_mask);

        for _ in 0..300 {
            pins_to_pio0();

            if cfg_written < CONFIG_REGS.len() {
                // Early phase: include a fresh WR_CFG this frame.
                update_wr_cfg(cfg_written);
                cfg_written += 1;

                let buf = unsafe { &*core::ptr::addr_of!(FRAME_BUF) };
                let cfg = single_buffer::Config::new(dma_ch, buf, tx);
                let xfer = cfg.start();
                let (ch, _, new_tx) = xfer.wait();
                dma_ch = ch;
                tx = new_tx;
            } else {
                // Config loaded — skip WR_CFG. Two DMA kicks bracket
                // the hole left in the buffer; PIO just stalls on an
                // empty FIFO in between and picks up where it left off.
                let frame = unsafe { &*core::ptr::addr_of!(FRAME_BUF) };
                let prefix: &[u32] = &frame[..WR_CFG_OFFSET];
                let cfg1 = single_buffer::Config::new(dma_ch, prefix, tx);
                let xfer1 = cfg1.start();
                let (ch, _, new_tx) = xfer1.wait();
                dma_ch = ch;
                tx = new_tx;

                let frame = unsafe { &*core::ptr::addr_of!(FRAME_BUF) };
                let data: &[u32] = &frame[DATA_LATCH_OFFSET..];
                let cfg2 = single_buffer::Config::new(dma_ch, data, tx);
                let xfer2 = cfg2.start();
                let (ch, _, new_tx) = xfer2.wait();
                dma_ch = ch;
                tx = new_tx;
            }

            // Wait for the PIO FIFO to drain so the final DATA_LATCH
            // has actually been clocked out before the function flip.
            while !tx.is_empty() { cortex_m::asm::nop(); }
            cortex_m::asm::delay(50);

            // ── CPU drives ROW + CLK scanning ────────────────────
            pins_to_sio();
            display_loop(sio, 10);
        }

        test_idx += 1;
    }
}
