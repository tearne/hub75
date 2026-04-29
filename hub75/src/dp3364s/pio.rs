//! PIO program for the unified DP3364S scan engine.
//!
//! One PIO state machine runs both the data and scan phases via two
//! entry points (`data_cmd` at PC 0, `scan_entry` at the address logged
//! at runtime). The CPU switches phases by writing three SM registers:
//!
//! - `INSTR` — force a JMP to the other entry point
//! - `EXECCTRL` — swap WRAP_TOP/WRAP_BOTTOM to that phase's loop body
//! - `PINCTRL` — swap OUT_COUNT (6 in data, 11 in scan) — see the echo
//!   note in the file header for `learning-examples/spwm_3_autonomous`
//!
//! The data clkdiv is 2 (PIO at 75 MHz → DP3364S CLK at 25 MHz, the
//! datasheet maximum); the scan clkdiv is 3 (PIO at 50 MHz). Integer
//! divisors only — fractional clkdiv jitter breaks the chip protocol.

use pio::{
    Assembler, JmpCondition, MovDestination, MovOperation, MovSource,
    OutDestination, Program, SetDestination, SideSet,
};

pub const PROGRAM_SIZE: usize = 32;

// ── PIO0 SM0 register addresses ─────────────────────────────────────

const PIO0_BASE: u32 = 0x5020_0000;
const PIO0_CTRL_SET: *mut u32 = (PIO0_BASE + 0x2000) as *mut u32;
const PIO0_CTRL_CLR: *mut u32 = (PIO0_BASE + 0x3000) as *mut u32;
const PIO0_FDEBUG: *mut u32 = (PIO0_BASE + 0x008) as *mut u32;
const SM0_CLKDIV: *mut u32 = (PIO0_BASE + 0x0C8) as *mut u32;
const SM0_EXECCTRL: *mut u32 = (PIO0_BASE + 0x0CC) as *mut u32;
const SM0_INSTR: *mut u32 = (PIO0_BASE + 0x0D8) as *mut u32;
const SM0_PINCTRL: *mut u32 = (PIO0_BASE + 0x0DC) as *mut u32;

const SM0_EN: u32 = 1 << 0;
const SM0_TXSTALL: u32 = 1 << 24;

// ── Assembly ────────────────────────────────────────────────────────

/// PIO program with two entry points and the offsets needed to set up
/// phase swapping at runtime. `*_local` are pre-relocation addresses
/// (origin = 0); add the installed origin to get hardware PCs.
pub struct AssembledProgram {
    pub program: Program<PROGRAM_SIZE>,
    pub data_cmd_local: u32,
    pub data_wrap_top_local: u32,
    pub scan_entry_local: u32,
    pub scan_wrap_top_local: u32,
}

pub fn assemble() -> AssembledProgram {
    let mut a = Assembler::<PROGRAM_SIZE>::new_with_side_set(SideSet::new(false, 1, false));

    let mut data_cmd = a.label();
    let mut scan_entry = a.label();
    let mut display_lp = a.label();
    let mut setup_lp = a.label();
    let mut oe_lp = a.label();
    let mut pre_loop = a.label();
    let mut lat_loop = a.label();
    let mut data_wrap_source = a.label();
    let mut scan_wrap_source = a.label();

    // ── DATA path (PC 0, wrap_target) ────────────────────────────────
    a.bind(&mut data_cmd);
    a.out_with_side_set(OutDestination::X, 15, 0); // n_pre - 1
    a.out_with_side_set(OutDestination::Y, 16, 0); // n_lat - 1
    a.out_with_side_set(OutDestination::NULL, 1, 0); // pad bit 31

    a.bind(&mut pre_loop);
    a.out_with_side_set(OutDestination::PINS, 6, 0);
    a.out_with_side_set(OutDestination::NULL, 2, 1); // CLK rise
    a.jmp_with_side_set(JmpCondition::XDecNonZero, &mut pre_loop, 0);

    a.set_with_side_set(SetDestination::PINS, 0b01, 0); // LAT high

    a.bind(&mut lat_loop);
    a.out_with_side_set(OutDestination::PINS, 6, 0);
    a.out_with_side_set(OutDestination::NULL, 2, 1); // CLK rise
    a.jmp_with_side_set(JmpCondition::YDecNonZero, &mut lat_loop, 0);

    a.set_with_side_set(SetDestination::PINS, 0b00, 0); // LAT low; data wrap fires after
    a.bind(&mut data_wrap_source);

    // ── SCAN path (reached only via forced JMP scan_entry) ───────────
    a.bind(&mut scan_entry);
    a.out_with_side_set(OutDestination::Y, 7, 0);
    a.bind(&mut display_lp);
    a.mov_with_side_set(MovDestination::Y, MovOperation::None, MovSource::Y, 0);
    a.mov_with_side_set(MovDestination::Y, MovOperation::None, MovSource::Y, 0);
    a.jmp_with_side_set(JmpCondition::YDecNonZero, &mut display_lp, 1);

    a.out_with_side_set(OutDestination::PINS, 11, 0);
    a.out_with_side_set(OutDestination::Y, 5, 0);

    a.bind(&mut setup_lp);
    a.mov_with_side_set(MovDestination::Y, MovOperation::None, MovSource::Y, 0);
    a.mov_with_side_set(MovDestination::Y, MovOperation::None, MovSource::Y, 0);
    a.jmp_with_side_set(JmpCondition::YDecNonZero, &mut setup_lp, 1);

    a.out_with_side_set(OutDestination::Y, 4, 0);
    a.set_with_side_set(SetDestination::PINS, 0b10, 0); // OE on

    a.bind(&mut oe_lp);
    a.mov_with_side_set(MovDestination::Y, MovOperation::None, MovSource::Y, 0);
    a.mov_with_side_set(MovDestination::Y, MovOperation::None, MovSource::Y, 0);
    a.jmp_with_side_set(JmpCondition::YDecNonZero, &mut oe_lp, 1);

    a.set_with_side_set(SetDestination::PINS, 0b00, 0); // OE off
    a.out_with_side_set(OutDestination::NULL, 5, 0); // drain
    a.bind(&mut scan_wrap_source);

    let mut program = a.assemble_with_wrap(data_wrap_source, data_cmd);
    program = program.set_origin(Some(0));

    // pio's `bind` labels the NEXT instruction slot; assemble_with_wrap
    // does `source -= 1` internally to convert that to the WRAP_TOP
    // hardware register value. The local PCs below match hardware
    // values (label - 1 for wrap tops).
    AssembledProgram {
        program,
        data_cmd_local: 0,
        data_wrap_top_local: 10,
        scan_entry_local: 11,
        scan_wrap_top_local: 26,
    }
}

// ── Phase-swap utility ──────────────────────────────────────────────

/// Precomputed PINCTRL / EXECCTRL / forced-JMP-target values for each
/// phase. `to_data()` and `to_scan()` write all three SM registers in
/// the order PINCTRL → EXECCTRL → INSTR — INSTR last because the
/// forced JMP consumes it under the new pin/wrap configuration.
pub struct PhaseSwap {
    data_pinctrl: u32,
    scan_pinctrl: u32,
    data_execctrl: u32,
    scan_execctrl: u32,
    data_cmd_pc: u32,
    scan_entry_pc: u32,
}

impl PhaseSwap {
    /// Build from the installed program's runtime origin and the SM's
    /// base EXECCTRL (read from the SM after PIOBuilder has set it).
    pub fn new(prog: &AssembledProgram, prog_offset: u32, base_execctrl: u32) -> Self {
        const EXECCTRL_WRAP_MASK: u32 = (0x1F << 12) | (0x1F << 7);
        let execctrl_no_wrap = base_execctrl & !EXECCTRL_WRAP_MASK;

        let data_execctrl = execctrl_no_wrap
            | (((prog_offset + prog.data_wrap_top_local) & 0x1F) << 12)
            | (((prog_offset + prog.data_cmd_local) & 0x1F) << 7);
        let scan_execctrl = execctrl_no_wrap
            | (((prog_offset + prog.scan_wrap_top_local) & 0x1F) << 12)
            | (((prog_offset + prog.scan_entry_local) & 0x1F) << 7);

        // PINCTRL bit layout (RP2040 & RP2350):
        //   [31:29] SIDESET_COUNT  [28:26] SET_COUNT
        //   [25:20] OUT_COUNT      [19:15] IN_BASE
        //   [14:10] SIDESET_BASE   [9:5]   SET_BASE
        //   [4:0]   OUT_BASE
        //
        // Only OUT_COUNT differs between phases:
        //   Data: 6  — `out PINS, 6` writes pins 0..5 only; pins 6..10
        //              retain their last-driven value (key to echo
        //              suppression).
        //   Scan: 11 — `out PINS, 11` writes RGB + ADDR.
        let data_pinctrl: u32 =
            (1 << 29) | (2 << 26) | (6 << 20) | (0 << 15) | (11 << 10) | (12 << 5);
        let scan_pinctrl: u32 =
            (1 << 29) | (2 << 26) | (11 << 20) | (0 << 15) | (11 << 10) | (12 << 5);

        Self {
            data_pinctrl,
            scan_pinctrl,
            data_execctrl,
            scan_execctrl,
            data_cmd_pc: prog_offset + prog.data_cmd_local,
            scan_entry_pc: prog_offset + prog.scan_entry_local,
        }
    }

    pub fn to_data(&self) {
        unsafe {
            SM0_PINCTRL.write_volatile(self.data_pinctrl);
            SM0_EXECCTRL.write_volatile(self.data_execctrl);
            SM0_INSTR.write_volatile(self.data_cmd_pc & 0x1F);
        }
    }

    pub fn to_scan(&self) {
        unsafe {
            SM0_PINCTRL.write_volatile(self.scan_pinctrl);
            SM0_EXECCTRL.write_volatile(self.scan_execctrl);
            SM0_INSTR.write_volatile(self.scan_entry_pc & 0x1F);
        }
    }
}

// ── SM control ──────────────────────────────────────────────────────

pub fn enable_sm() {
    unsafe { PIO0_CTRL_SET.write_volatile(SM0_EN) };
}

pub fn disable_sm() {
    unsafe { PIO0_CTRL_CLR.write_volatile(SM0_EN) };
}

/// Integer-only clock divisor. Data phase: 2 (PIO at 75 MHz → CLK at
/// 25 MHz). Scan phase: 3 (PIO at 50 MHz). Fractional clkdiv produces
/// period jitter that breaks the chip protocol.
pub fn set_clkdiv(int_div: u32) {
    unsafe { SM0_CLKDIV.write_volatile(int_div << 16) };
}

/// Busy-wait until the SM has stalled with an empty TX FIFO. Used
/// between phase transitions so the pipeline is fully drained before
/// reconfiguring the SM. Microsecond-scale; not worth a `wfi` path.
pub fn wait_txstall() {
    unsafe {
        PIO0_FDEBUG.write_volatile(SM0_TXSTALL);
        while PIO0_FDEBUG.read_volatile() & SM0_TXSTALL == 0 {
            core::hint::spin_loop();
        }
    }
}
