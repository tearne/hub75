//! # DP3364S S-PWM — Step 7: Chained DMA (zero-gap scan↔data)
//!
//! Seventh in the SPWM progression. Builds on `spwm_6_ring_dma.rs`.
//!
//! **Goal:** eliminate ALL CPU involvement in the scan↔data transition
//! by chaining DMA transfers: scan→data→scan runs entirely in hardware.
//! The CPU only swaps buffer pointers between frames.
//!
//! **Status:** NOT YET IMPLEMENTED — placeholder.
//!
//! ## Architecture (planned)
//!
//! ```text
//!   DMA ch0 (scan, ring mode) ──chain──→ ch1 (frame data) ──chain──→ ch0
//!   ↑                                                                  │
//!   └──────────────────────────────────────────────────────────────────┘
//!
//!   Core 0: update ch1 source pointer when core 1 signals done
//!   Core 1: fill pixels → pack → signal done
//! ```
//!
//! **Key challenges:**
//! - Ring-mode DMA (ch0) must terminate cleanly to trigger the chain
//!   to ch1. TRANS_COUNT controls when chaining fires, but ring mode
//!   wraps reads independently — need the right TRANS_COUNT to get
//!   N complete scan passes before chaining.
//! - Clock divisor must switch between scan (div 3) and data (div 2).
//!   May need to accept a single divisor, or use PIO-side delays.
//! - Updating ch1's read address (buffer swap) while the chain is
//!   running requires careful synchronization.
//!
//! ## Run
//!
//! ```sh
//! cargo run --release --example spwm_7_chained_dma
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use rp235x_hal as hal;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let _clocks = hal::clocks::init_clocks_and_plls(
        12_000_000, pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB,
        &mut pac.RESETS, &mut watchdog,
    ).unwrap();

    defmt::info!("spwm_7_chained_dma: not yet implemented");
    loop {
        cortex_m::asm::wfe();
    }
}
