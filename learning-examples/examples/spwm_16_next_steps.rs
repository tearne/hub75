//! # DP3364S S-PWM — Step 16: next-steps plan (placeholder)
//!
//! Sixteenth in the SPWM progression. **Not yet implemented** — this
//! file is the plan for what follows spwm_15's success, organised by
//! thread. Each thread is a self-contained investigation; they can
//! run in parallel or in sequence. Use this file as the living plan
//! until the first concrete experiment displaces it (at which point
//! copy relevant bits into new files and leave this as an index).
//!
//! spwm_15 demonstrated that a single SM running two separate PIO
//! programs (swapped at each phase boundary) eliminates the echo and
//! avoids the ~2.6 ms dual-SM handover dark gap. The architecture is
//! viable. What follows is:
//!
//! ## Thread A — Verify brightness and flicker properly
//!
//! **Why.** On E25's four-thin-colour-lines pattern, spwm_15 looked
//! slightly dimmer than spwm_14 to the eye, but the comparison was
//! impressionistic. Thin lines understate brightness differences and
//! eye-memory is unreliable. Before treating spwm_15 as the winning
//! architecture we need a comparison on a realistic load.
//!
//! **Proposed approach.**
//!   1. Build a "brightness probe" variant of each of spwm_14 and
//!      spwm_15 displaying the **same full-frame pattern**: a mid-
//!      grey flood (e.g. RGB(128,128,128)) across the whole panel.
//!      Full-frame, not thin lines.
//!   2. Flash each in turn under identical ambient lighting. Take
//!      phone-camera photos for reference (fixed exposure).
//!   3. Also run with a **moving pattern** (scrolling hue or block)
//!      to surface any flicker that a static frame would hide.
//!   4. Record: comparative brightness, visible flicker presence/
//!      absence, perceived frame rate, any artefacts.
//!
//! **What we learn.** Either spwm_15 matches spwm_14 on brightness
//! (architecture is a clean win) or it doesn't (there's still a
//! flicker-cost we haven't characterised — thread B's mechanism work
//! becomes directly relevant to finding where the brightness loss is).
//!
//! ## Thread B — Understand the mechanism
//!
//! **Why.** We know **what** works (program swap) but not **why**.
//! The chip is responding to some difference between "unified program
//! wrapping through dispatch" and "separate programs with distinct
//! wrap regions". Candidates:
//!
//!   - **CLK-idle during register writes.** The swap takes several µs
//!     of CPU time writing EXECCTRL / PINCTRL / SMx_INSTR. CLK holds
//!     whatever side-set the last instruction asserted (0 in our
//!     programs). That idle period could be the "pipeline reset".
//!     Test: artificially extend CLK-idle by inserting `asm::delay`
//!     between scan cycles in spwm_12; see if echo diminishes.
//!
//!   - **Different CLK/LAT edge pattern at wrap.** spwm_12's unified
//!     wrap goes through 2 extra instructions (`out X, 1` + `jmp`)
//!     before reaching the scan body. spwm_15's scan wraps directly
//!     to `out Y, 7`. The edge timing on CLK may differ subtly.
//!     Test: capture CLK/LAT on a logic analyser at the wrap points
//!     for both architectures; compare edge patterns bit-for-bit.
//!
//!   - **The forced JMP itself.** spwm_15's swap ends with a JMP
//!     executed via SMx_INSTR, which is distinct from a wrap-back.
//!     Possibly the chip notices "PIO paused to receive an override
//!     instruction"? Test: replace wrap-back in spwm_12's unified
//!     program with an explicit per-scan-cycle SMx_INSTR JMP; see if
//!     that alone kills the echo.
//!
//! **What we learn.** Any of these confirming would let us design
//! the reset condition into cheaper architectures (e.g. if the
//! CLK-idle hypothesis holds, we might inject short idles without
//! switching programs at all). Understanding it also informs future
//! work with the same chip family.
//!
//! ## Thread C — Productionise
//!
//! **Why.** Separate the driver from the investigation code. spwm_15
//! is currently a learning-examples program; a production driver
//! wants a cleaner API, documented pinout and timing, a consumer-
//! facing crate structure, and proper tests.
//!
//! **Proposed approach.**
//!   1. Extract the PIO programs, swap helper, frame layout, pack
//!      routine, and startup-flush sequence into a library crate
//!      (maybe under `usb-display/` or a new top-level crate).
//!   2. Provide a clean API: `Display::new(...) -> Display`,
//!      `display.present(&framebuffer)`. Hide the DMA channel,
//!      program-swap, SIO-FIFO details from consumers.
//!   3. Move `panel_reset` and `echo_baseline` semantics into the
//!      library as optional utilities (panel_reset as a method,
//!      echo_baseline as a test fixture).
//!   4. Run as a `usb-display` application on real hardware under
//!      extended uptime (hours) to shake out any drift or sync loss.
//!
//! **What we learn.** Whether spwm_15's architecture holds up under
//! realistic use (not just "tested for seconds on a synthetic
//! pattern"). If it doesn't, that's a finding too, and sends us
//! back to thread B to understand why.
//!
//! ## Suggested order
//!
//! A → B (informed by A) → C. Thread A is the cheapest and the most
//! likely to reveal a loose end we'd want to investigate before
//! productionising. Thread B's mechanism work becomes more targeted
//! after A. Thread C comes last because "clean up for production"
//! on an architecture we don't fully understand would bake in any
//! subtle issue the mechanism work might otherwise surface.
//!
//! ## Status
//!
//! Plan only — no code yet. Stub `main` below exists so this compiles
//! alongside the other examples; running it does nothing useful.
//!
//! ```sh
//! # Do not run — placeholder.
//! # cargo run --release --example spwm_16_next_steps
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
    defmt::info!("spwm_16 is a planning placeholder; see header for next steps.");
    loop {
        cortex_m::asm::wfe();
    }
}
