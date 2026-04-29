//! Two PIO state machines for the shift-register family.
//!
//! - **Data SM** clocks 6 RGB bits per pixel through the panel's shift
//!   registers, latches at end of line, and signals the address SM via
//!   PIO IRQ 1.
//! - **Address SM** waits for the latch signal (PIO IRQ 1), drives the
//!   row address pins, and pulses OE for the bitplane's BCM-weighted
//!   on-time. Signals the data SM via PIO IRQ 0 when safe to latch.
//!
//! The address SM uses a downward counter trick: `X` starts at
//! `scan_lines - 1` and the row address is `~X`, so addresses go
//! `0..scan_lines-1` in order. `scan_lines` (= `H / 2`) varies by
//! panel size, so the program is parameterised over `scan_minus_one`.

use pio::{
    Assembler, IrqIndexMode, JmpCondition, MovDestination, MovOperation, MovSource,
    OutDestination, Program, SetDestination, SideSet, WaitSource,
};

pub const PROGRAM_SIZE: usize = 32;

pub struct AssembledPrograms {
    pub data: Program<PROGRAM_SIZE>,
    pub address: Program<PROGRAM_SIZE>,
}

pub fn assemble(scan_minus_one: u8) -> AssembledPrograms {
    AssembledPrograms {
        data: assemble_data(),
        address: assemble_address(scan_minus_one),
    }
}

// ── Data SM ─────────────────────────────────────────────────────────
// Sideset: 2 bits — bit 0 = CLK (GPIO 11), bit 1 = LAT (GPIO 12).
// OUT pins: 6 bits (RGB at GPIO 0..5).

fn assemble_data() -> Program<PROGRAM_SIZE> {
    let side_set = SideSet::new(false, 2, false);
    let mut a = Assembler::<PROGRAM_SIZE>::new_with_side_set(side_set);

    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    let mut write_data = a.label();

    // Load (W - 1) into Y once at startup. Pre-pushed by the host
    // before DMA takes over the FIFO.
    a.out_with_side_set(OutDestination::Y, 32, 0b00);

    a.bind(&mut wrap_target);
    // Reload pixel counter from Y at the start of every row.
    a.mov_with_side_set(MovDestination::X, MovOperation::None, MovSource::Y, 0b00);

    a.bind(&mut write_data);
    // 6 data bits with CLK low, then 2 bits of pad with CLK high,
    // then loop. Each pixel = 3 PIO cycles.
    a.out_with_side_set(OutDestination::PINS, 6, 0b00);
    a.out_with_side_set(OutDestination::NULL, 2, 0b01);
    a.jmp_with_side_set(JmpCondition::XDecNonZero, &mut write_data, 0b00);

    // Wait for address SM to signal "safe to latch" (PIO IRQ 0),
    // then raise LAT via sideset and signal back via PIO IRQ 1.
    a.wait_with_side_set(1, WaitSource::IRQ, 0, false, 0b00);
    a.irq_with_side_set(false, false, 1, IrqIndexMode::DIRECT, 0b10);

    a.bind(&mut wrap_source);
    a.assemble_with_wrap(wrap_source, wrap_target)
}

// ── Address SM ──────────────────────────────────────────────────────
// Sideset: 1 bit — OE (GPIO 13). 1 = OE high (off), 0 = OE low (on).
// OUT pins: log2(scan_lines) bits (address pins, GPIO 6..).

fn assemble_address(scan_minus_one: u8) -> Program<PROGRAM_SIZE> {
    let side_set = SideSet::new(false, 1, false);
    let mut a = Assembler::<PROGRAM_SIZE>::new_with_side_set(side_set);

    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    let mut next_row = a.label();
    let mut initialize = a.label();
    let mut off_before = a.label();
    let mut on_delay = a.label();
    let mut off_after = a.label();

    // First time through, jump to initialize. Subsequent iterations
    // wrap back to wrap_target and drop into the X-- jmp.
    a.jmp_with_side_set(JmpCondition::Always, &mut initialize, 1);

    a.bind(&mut wrap_target);
    a.jmp_with_side_set(JmpCondition::XDecNonZero, &mut next_row, 1);
    // X reached 0 — we've finished a bitplane. Drop the lingering
    // ON-count word from OSR (autopull will refill with the next
    // bitplane's OFF count for the `out ISR, 32` below).
    a.out_with_side_set(OutDestination::NULL, 32, 1);

    a.bind(&mut initialize);
    // Load the next bitplane's OFF count into ISR (it stays there
    // for the whole bitplane; OSR holds the ON count via autopull).
    a.out_with_side_set(OutDestination::ISR, 32, 1);
    a.set_with_side_set(SetDestination::X, scan_minus_one, 1);

    a.bind(&mut next_row);
    // Signal data SM "safe to latch" (PIO IRQ 0).
    a.irq_with_side_set(false, false, 0, IrqIndexMode::DIRECT, 1);
    // Address pins = ~X (downward counter ⇒ addresses 0..scan_lines-1).
    a.mov_with_side_set(
        MovDestination::PINS,
        MovOperation::Invert,
        MovSource::X,
        1,
    );
    // Wait for data SM to finish latching.
    a.wait_with_side_set(1, WaitSource::IRQ, 1, false, 1);

    // OE-off pre-pulse delay, then OE-on for the bitplane's BCM
    // weight, then OE-off post-pulse delay.
    a.mov_with_side_set(MovDestination::Y, MovOperation::None, MovSource::ISR, 1);
    a.bind(&mut off_before);
    a.jmp_with_side_set(JmpCondition::YDecNonZero, &mut off_before, 1);
    a.mov_with_side_set(MovDestination::Y, MovOperation::None, MovSource::OSR, 1);
    a.bind(&mut on_delay);
    a.jmp_with_side_set(JmpCondition::YDecNonZero, &mut on_delay, 0); // OE on
    a.mov_with_side_set(MovDestination::Y, MovOperation::None, MovSource::ISR, 1);
    a.bind(&mut off_after);
    a.jmp_with_side_set(JmpCondition::YDecNonZero, &mut off_after, 1);

    a.bind(&mut wrap_source);
    a.assemble_with_wrap(wrap_source, wrap_target)
}
