//! Scan-phase buffer for ring-mode DMA.
//!
//! 32 scan words, one per scan-line, replayed by the ring-mode DMA
//! channel for the duration of each frame's scan phase. Each word
//! carries the row's address, OE pulse width, and on/off timing for
//! the PIO scan program.

use super::framing::SCAN_LINES;

const DISPLAY_CLK: u32 = 100;
const SETUP_CLK: u32 = 28;

// OE pulse width in PIO cycles. Scan-line 0 needs the wider W12 pulse
// for SRAM-write-pointer alignment at boot; all other scan-lines use
// W4. Uniform W12 breaks the upper/lower-half sync on the DP3364S.
const OE_CLK_W4: u32 = 4;
const OE_CLK_W12: u32 = 12;

/// One 32-bit word per scan-line, read by the PIO `scan_entry`:
///   bits  0..6  : display count - 1  (brightness-modulated on-time)
///   bits  7..12 : 0x3F                 (RGB pins driven HIGH in scan)
///   bits 13..17 : scan address (0..31)
///   bits 18..22 : setup count - 1     (idle between display and OE)
///   bits 23..26 : oe count - 1        (OE pulse width)
///   bits 27..31 : pad
fn scan_word(display: u32, addr: u32, setup: u32, oe: u32) -> u32 {
    (display - 1)
        | (0x3F << 7)
        | (addr << 13)
        | ((setup - 1) << 18)
        | ((oe - 1) << 23)
}

/// Scan word with the wider OE pulse on scan-line 0 (W12) and the
/// shorter W4 pulse elsewhere. The W12 on line 0 aligns the 8 driver
/// chips' internal SRAM write pointers during the boot sync phase;
/// W12 on every line breaks upper/lower-half sync.
fn scan_word_for_sync(scan_line: usize) -> u32 {
    let oe = if scan_line == 0 { OE_CLK_W12 } else { OE_CLK_W4 };
    scan_word(DISPLAY_CLK, scan_line as u32, SETUP_CLK, oe)
}

/// Populate the scan buffer once at boot.
pub fn build_scan_buf(buf: &mut [u32; SCAN_LINES]) {
    for row in 0..SCAN_LINES {
        buf[row] = scan_word_for_sync(row);
    }
}
