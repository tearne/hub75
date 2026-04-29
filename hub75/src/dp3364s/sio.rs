//! SIO inter-core FIFO helpers.
//!
//! The kick (commit → core 1 pack worker) and done (pack worker → core
//! 0 scan loop) messages travel through the SIO FIFOs:
//! - `fifo_write_*` pushes a u32 to the other core, asserts SEV
//! - `fifo_read_blocking` blocks via `wfe` until a word is available
//! - `fifo_try_read` is non-blocking, returns `Option<u32>`

const FIFO_ST: *const u32 = 0xD000_0050 as *const u32;
const FIFO_WR: *mut u32 = 0xD000_0054 as *mut u32;
const FIFO_RD: *const u32 = 0xD000_0058 as *const u32;

const ST_VLD: u32 = 1 << 0;
const ST_RDY: u32 = 1 << 1;

pub fn fifo_write(val: u32) {
    unsafe {
        while FIFO_ST.read_volatile() & ST_RDY == 0 {}
        FIFO_WR.write_volatile(val);
        cortex_m::asm::sev();
    }
}

pub fn fifo_try_read() -> Option<u32> {
    unsafe {
        if FIFO_ST.read_volatile() & ST_VLD != 0 {
            Some(FIFO_RD.read_volatile())
        } else {
            None
        }
    }
}

pub fn fifo_read_blocking() -> u32 {
    unsafe {
        loop {
            if FIFO_ST.read_volatile() & ST_VLD != 0 {
                return FIFO_RD.read_volatile();
            }
            cortex_m::asm::wfe();
        }
    }
}
