//! # DP3364S S-PWM — Step 4: Scan via Second PIO State Machine
//!
//! Final step in the four-part progression. Builds on
//! `spwm_03_pio_commands.rs`.
//!
//! **What this step adds:** a second PIO state machine (SM1) drives
//! the display scan loop — CLK, ADDR (row select), and OE (row
//! pulse). The CPU no longer bit-bangs any panel signals. Its only
//! job is packing the frame buffer and kicking two DMA transfers.
//!
//! **Key challenge: shared CLK pin.** SM0 (data) and SM1 (scan)
//! both need GPIO 11 (CLK). The RP2350 PIO retains an SM's
//! output-enable even when the SM is disabled — simply calling
//! `disable_sm()` does NOT release the pin. The idle SM continues
//! to contend on CLK, corrupting the active SM's output.
//!
//! **Fix: explicit pindir handover.** Before disabling an SM, we
//! force-execute `set pindirs, 0` via the SMx_INSTR register to
//! release CLK's output-enable. Before enabling, we force-execute
//! `set pindirs, N` to reclaim it. Since CLK is a sideset pin (not
//! in the SET range), this requires temporarily modifying PINCTRL
//! to redirect SET_BASE, executing the instruction, then restoring.
//!
//! ```text
//!   SM0 (data):   RGB (0–5), CLK (11), LAT (12)
//!   SM1 (scan):   ADDR (6–10), CLK (11), OE (13)
//!                              ▲
//!                     shared — pindir handover required
//! ```
//!
//! ## SM1 scan program
//!
//! Four phases per row, matching the CPU `display_loop` from step 3:
//!
//!   1. DISPLAY: 100 CLKs with previous ADDR (MOSFET discharge time)
//!   2. ADDR: set row address for current iteration
//!   3. SETUP: 28 CLKs with new ADDR stable
//!   4. OE: W12 pulse (row 0) or W4 (others)
//!
//! Each iteration consumes one 32-bit DMA word:
//!
//! ```text
//!   bits  [6:0]    display_clk − 1   (7 bits, `out y, 7`)
//!   bits [11:7]    ADDR              (5 bits, `out pins, 5`)
//!   bits [16:12]   setup_clk − 1     (5 bits, `out y, 5`)
//!   bits [20:17]   oe_clk − 1        (4 bits, `out y, 4`)
//!   bits [31:21]   padding           (11 bits, `out null, 11`)
//! ```
//!
//! ## Scan buffer
//!
//! Built once at startup. `DISPLAY_CYCLES` full passes through all
//! `SCAN_LINES` rows = one scan batch per frame. Per-iteration
//! timings: `pre_clk` = 124 CLKs, `oe_clk` = 4 CLKs → 128 CLKs
//! between OE rises, matching the 128-CLK minimum the chip needs
//! to complete its internal PWM row cycle.
//!
//! ## Run
//!
//! ```sh
//! cargo run --release --example spwm_04_pio_scan
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
    0x1100, 0x021F, 0x033F, 0x043F, 0x0504, 0x0642, 0x0700,
    0x08BF, 0x0960, 0x0ABE, 0x0B8B, 0x0C88, 0x0D12,
];

// ── Per-command CLK counts (unchanged from spwm_03_pio_commands) ─────────
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
//
// One scan batch = `DISPLAY_CYCLES` full passes through all 32 rows.
// 10 matches the CPU version's `display_loop(sio, 10)`; raise it if
// the display looks dim.
const DISPLAY_CYCLES: usize = 10;
const SCAN_ITERS: usize = DISPLAY_CYCLES * SCAN_LINES;

// Per-row CLK counts, matching the CPU `display_loop` pattern:
//
//   1. DISPLAY phase: 100 CLKs with the PREVIOUS row's ADDR still
//      active. This is the main "LED on-time" — the panel's MOSFETs
//      need real time to fully discharge before switching rows.
//      Skipping this causes cross-conduction ghosting.
//   2. ADDR change: set ABCDE to the current row.
//   3. SETUP phase: 28 CLKs with the new ADDR stable before OE.
//   4. OE pulse: W12 (row 0, scan-counter reset) or W4 (others).
//
// Total CLKs per row = 100 + 28 + OE = 132 (W4) or 140 (W12).
const DISPLAY_CLK: u32 = 100;
const SETUP_CLK: u32 = 28;
const OE_CLK_W4: u32 = 4;
const OE_CLK_W12: u32 = 12;

static mut SCAN_BUF: [u32; SCAN_ITERS] = [0u32; SCAN_ITERS];

/// Scan buffer word layout (consumed LSB-first by SM1):
///
///   bits  [6:0]    display_clk − 1   (7 bits, `out y, 7`)
///   bits [11:7]    ADDR              (5 bits, `out pins, 5`)
///   bits [16:12]   setup_clk − 1     (5 bits, `out y, 5`)
///   bits [20:17]   oe_clk − 1        (4 bits, `out y, 4`)
///   bits [31:21]   padding           (11 bits, `out null, 11`)
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

// ── Frame-buffer packing (unchanged from spwm_03_pio_commands) ───────────

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

fn build_frame(brightness: u16, color_mask: u32) {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF) };

    buf[VSYNC_OFFSET] = header(VSYNC_PRE, VSYNC_LAT);
    buf[VSYNC_OFFSET + 1] = 0;

    buf[PRE_ACT_OFFSET] = header(PRE_ACT_PRE, PRE_ACT_LAT);
    for i in 0..PRE_ACT_WORDS {
        buf[PRE_ACT_OFFSET + 1 + i] = 0;
    }

    buf[WR_CFG_OFFSET] = header(WR_CFG_PRE, WR_CFG_LAT);

    for l in 0..LATCHES_PER_FRAME {
        let hdr = DATA_LATCH_OFFSET + l * DATA_LATCH_STRIDE;
        buf[hdr] = header(DATA_LATCH_PRE, DATA_LATCH_LAT);
        pack_shift128(
            &mut buf[hdr + 1 .. hdr + 1 + DATA_LATCH_WORDS],
            brightness, color_mask,
        );
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

// ── SM enable/disable + pindir handover ──────────────────────────────
//
// When a PIO state machine is disabled, its output-enable for ALL
// pins goes to 0 — the pad sees high-Z from that SM. This cleanly
// releases the shared CLK pin so only the *active* SM drives it.
//
// RP2350 atomic SET/CLR aliases for PIO0_CTRL:
// ── PIO register addresses ───────────────────────────────────────────
const PIO0_BASE: u32 = 0x5020_0000;
const PIO0_CTRL_SET: u32 = PIO0_BASE + 0x2000; // atomic SET alias
const PIO0_CTRL_CLR: u32 = PIO0_BASE + 0x3000; // atomic CLR alias
const PIO0_FDEBUG: u32 = PIO0_BASE + 0x008;
const SM0_PINCTRL: u32 = PIO0_BASE + 0x0DC;
const SM1_PINCTRL: u32 = PIO0_BASE + 0x0F4;
const SM0_INSTR: u32 = PIO0_BASE + 0x0D8;
const SM1_INSTR: u32 = PIO0_BASE + 0x0F0;
const SM0_EN: u32 = 1 << 0;
const SM1_EN: u32 = 1 << 1;
const SM0_TXSTALL: u32 = 1 << 24;
const SM1_TXSTALL: u32 = 1 << 25;

// PIO instruction encodings (sideset=0, delay=0 for all).
const SET_PINDIRS_0: u32 = 0xE080; // set pindirs, 0
const SET_PINDIRS_1: u32 = 0xE081; // set pindirs, 1
const SET_PINDIRS_3: u32 = 0xE083; // set pindirs, 0b11

// PINCTRL field masks.
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

/// Force-execute `set pindirs, val` on an SM targeting a temporary
/// SET_BASE/SET_COUNT range, then restore PINCTRL. This is how we
/// release (val=0) or reclaim (val=N) the shared CLK pin's output-
/// enable — the RP2350 retains an SM's OE state even when disabled,
/// so `disable_sm()` alone doesn't release the pin.
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

/// Release SM0's CLK (11) + LAT (12) pindirs → high-Z.
fn release_sm0_clk_lat() {
    force_set_pindirs(SM0_PINCTRL, SM0_INSTR, 11, 2, SET_PINDIRS_0);
}

/// Reclaim SM0's CLK (11) + LAT (12) pindirs → output.
fn claim_sm0_clk_lat() {
    force_set_pindirs(SM0_PINCTRL, SM0_INSTR, 11, 2, SET_PINDIRS_3);
}

/// Release SM1's CLK (11) pindir → high-Z.
fn release_sm1_clk() {
    force_set_pindirs(SM1_PINCTRL, SM1_INSTR, 11, 1, SET_PINDIRS_0);
}

/// Reclaim SM1's CLK (11) pindir → output.
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
    //
    // Configure pads via `into_push_pull_output()` first — this sets
    // drive strength, disables pull-downs, and enables the output
    // driver at the pad level. Then bulk-switch funcsel to PIO0 via
    // raw IO_BANK0 writes. This matches `spwm_03_pio_commands`'s proven
    // pad setup; using `into_function::<FunctionPio0>()` directly
    // left the pads misconfigured (pull-downs enabled, output driver
    // possibly not fully engaged).
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
    // All pins low before handing to PIO.
    let sio_regs = unsafe { &(*hal::pac::SIO::ptr()) };
    sio_regs.gpio_out_clr().write(|w| unsafe {
        w.bits(ALL_RGB | (0x1F << 6) | (1 << 11) | (1 << 12) | (1 << 13))
    });
    // Switch all 14 HUB75 pins to PIO0.
    const IO_BANK0_BASE: u32 = 0x4002_8000;
    for p in 0..14u32 {
        let addr = IO_BANK0_BASE + 0x04 + p * 8;
        unsafe { (addr as *mut u32).write_volatile(6) }; // funcsel = PIO0
    }

    // Enable pull-down on GPIO 12 (LAT). When SM0 is disabled during
    // the scan phase, its output-enable for LAT drops to 0 and the
    // pin goes high-Z. Without a pull-down, LAT can float high and
    // the chip interprets CLK edges as spurious commands (ghost
    // DATA_LATCHes, VSYNCs, etc.) that corrupt the display. The
    // pull-down keeps LAT solidly at 0 whenever no SM drives it.
    const PADS_BANK0: u32 = 0x4003_8000;
    const GPIO12_PAD: u32 = PADS_BANK0 + 0x04 + 12 * 4;
    unsafe {
        let val = (GPIO12_PAD as *const u32).read_volatile();
        (GPIO12_PAD as *mut u32).write_volatile(val | (1 << 2)); // PDE = 1
    }

    // ── 3. Pre-build the scan buffer ─────────────────────────────────
    build_scan_buf();

    // ── 4. Install both PIO programs ─────────────────────────────────
    let (mut pio0, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // ── SM0: data path (same program as spwm_03_pio_commands.rs, plus a
    //         `mov pins, null` cleanup so RGB settles to 0 on stall)
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
            .out_pins(0, 6)                // RGB GPIO 0–5
            .side_set_pin_base(11)         // CLK (11), LAT (12)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .autopull(true)
            .pull_threshold(32)
            .clock_divisor_fixed_point(2, 0) // 75 MHz → 25 MHz DCLK
            .build(sm0);

    data_sm.set_pindirs([
        (0,  hal::pio::PinDir::Output), (1,  hal::pio::PinDir::Output),
        (2,  hal::pio::PinDir::Output), (3,  hal::pio::PinDir::Output),
        (4,  hal::pio::PinDir::Output), (5,  hal::pio::PinDir::Output),
        (11, hal::pio::PinDir::Output), (12, hal::pio::PinDir::Output),
    ]);
    let _data_sm = data_sm.start();
    // SM0 is now enabled; SM1 will be enabled only during scan phase.

    // ── SM1: scan path — ADDR + OE + CLK ───────────────────────────
    //
    // Four phases per row, matching the CPU display_loop exactly:
    //   1. DISPLAY: CLK pulses with PREVIOUS row's ADDR (MOSFET time)
    //   2. ADDR change to current row
    //   3. SETUP: CLK pulses with new ADDR stable
    //   4. OE pulse (W12 row 0, W4 others)
    let scan_ss = pio::SideSet::new(false, 1, false);
    let mut sa = pio::Assembler::new_with_side_set(scan_ss);
    let mut s_wrap_target = sa.label();
    let mut s_wrap_source = sa.label();
    let mut s_display = sa.label();
    let mut s_setup = sa.label();
    let mut s_oe = sa.label();

    sa.bind(&mut s_wrap_target);
    // Phase 1: display time with old ADDR
    sa.out_with_side_set(pio::OutDestination::Y, 7, 0);     // display_clk - 1
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
    // Phase 2: set new ADDR
    sa.out_with_side_set(pio::OutDestination::PINS, 5, 0);   // ADDR
    // Phase 3: setup time with new ADDR
    sa.out_with_side_set(pio::OutDestination::Y, 5, 0);      // setup_clk - 1
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
    // Phase 4: OE pulse
    sa.out_with_side_set(pio::OutDestination::Y, 4, 0);      // oe_clk - 1
    sa.set_with_side_set(pio::SetDestination::PINS, 1, 0);   // OE high
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
    sa.set_with_side_set(pio::SetDestination::PINS, 0, 0);   // OE low
    sa.out_with_side_set(pio::OutDestination::NULL, 11, 0);   // drop padding
    sa.bind(&mut s_wrap_source);
    let scan_prog = sa.assemble_with_wrap(s_wrap_source, s_wrap_target);
    let scan_installed = pio0.install(&scan_prog).unwrap();

    let (mut scan_sm, _, scan_tx) =
        hal::pio::PIOBuilder::from_installed_program(scan_installed)
            .buffers(hal::pio::Buffers::OnlyTx)
            .out_pins(6, 5)                // ADDR GPIO 6–10
            .set_pins(13, 1)               // OE GPIO 13
            .side_set_pin_base(11)         // CLK (shared with SM0)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .autopull(true)
            .pull_threshold(32)
            .clock_divisor_fixed_point(3, 0) // 50 MHz → ~16.7 MHz scan CLK
            .build(sm1);

    scan_sm.set_pindirs([
        (6,  hal::pio::PinDir::Output), (7,  hal::pio::PinDir::Output),
        (8,  hal::pio::PinDir::Output), (9,  hal::pio::PinDir::Output),
        (10, hal::pio::PinDir::Output), (11, hal::pio::PinDir::Output),
        (13, hal::pio::PinDir::Output),
    ]);
    let _scan_sm = scan_sm.start();
    // Immediately disable SM1 — SM0 needs CLK exclusively for the
    // first data phase. SM1 will be enabled when scan starts.
    disable_sm(SM1_EN);

    // ── 5. DMA — ch0 feeds SM0, ch1 feeds SM1 ────────────────────────
    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut data_ch = dma.ch0;
    let mut scan_ch = dma.ch1;
    let mut data_tx = data_tx;
    let mut scan_tx = scan_tx;

    defmt::info!("DP3364S S-PWM (dual SM) starting");

    // ── 6. Startup flush ─────────────────────────────────────────────
    //
    // The DP3364S chips retain SRAM contents across code changes (as
    // long as panel power isn't cycled). Send 14 all-black frames at
    // boot to: (a) write all 13 config registers, (b) fill every
    // SRAM slot with brightness 0, and (c) give the PLL time to lock
    // onto our DCLK. After this the panel is guaranteed dark and in
    // a known state regardless of what ran previously.
    {
        build_frame(0x0000, ALL_RGB);
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

            // Run one scan batch so the chip displays the black data.
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
    }
    // Config registers are now fully loaded; skip WR_CFG in main loop.
    // ── 7. Main loop ─────────────────────────────────────────────────
    //
    // Config registers were fully loaded during the flush. Every
    // frame skips WR_CFG — two DMA kicks bracket the gap in the
    // buffer (VSYNC+PRE_ACT, then DATA_LATCH × 512).
    let tests: [(u16, u32, &str); 6] = [
        (0x3FFF, ALL_RGB,  "bright white"),
        (0x00FF, ALL_RGB,  "dim white"),
        (0x0000, ALL_RGB,  "black"),
        (0x3FFF, R0 | R1,  "red"),
        (0x3FFF, G0 | G1,  "green"),
        (0x3FFF, B0 | B1,  "blue"),
    ];

    let mut test_idx: usize = 0;

    loop {
        let (brightness, color_mask, name) = tests[test_idx % tests.len()];
        defmt::info!("{}", name);
        build_frame(brightness, color_mask);

        for _ in 0..300 {
            // ── Phase 1: SM0 data commands (WR_CFG skipped) ──────
            let frame = unsafe { &*core::ptr::addr_of!(FRAME_BUF) };
            let prefix: &[u32] = &frame[..WR_CFG_OFFSET];
            let cfg1 = single_buffer::Config::new(data_ch, prefix, data_tx);
            let xfer1 = cfg1.start();
            let (ch, _, new_tx) = xfer1.wait();
            data_ch = ch;
            data_tx = new_tx;

            let frame = unsafe { &*core::ptr::addr_of!(FRAME_BUF) };
            let data: &[u32] = &frame[DATA_LATCH_OFFSET..];
            let cfg2 = single_buffer::Config::new(data_ch, data, data_tx);
            let xfer2 = cfg2.start();
            let (ch, _, new_tx) = xfer2.wait();
            data_ch = ch;
            data_tx = new_tx;
            // Wait for SM0 to finish its last command (precise: polls
            // TXSTALL flag rather than a fixed delay).
            wait_txstall(SM0_TXSTALL);

            // ── Handover: data → scan ────────────────────────────
            // SM0 is stalled. Release its CLK+LAT output-enable so
            // it stops contending with SM1 on pin 11, then disable.
            release_sm0_clk_lat();
            disable_sm(SM0_EN);
            // Claim CLK output-enable for SM1, start scan DMA
            // (FIFO pre-fills while SM1 is still disabled), enable.
            claim_sm1_clk();
            let scan = unsafe { &*core::ptr::addr_of!(SCAN_BUF) };
            let scan_cfg = single_buffer::Config::new(scan_ch, scan, scan_tx);
            let scan_xfer = scan_cfg.start();
            enable_sm(SM1_EN);
            let (ch, _, new_tx) = scan_xfer.wait();
            scan_ch = ch;
            scan_tx = new_tx;
            wait_txstall(SM1_TXSTALL);

            // ── Handover: scan → data ────────────────────────────
            release_sm1_clk();
            disable_sm(SM1_EN);
            claim_sm0_clk_lat();
            enable_sm(SM0_EN);

        }

        test_idx += 1;
    }
}
