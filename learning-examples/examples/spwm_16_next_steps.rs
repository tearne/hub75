//! # DP3364S S-PWM — Step 16: next-steps plan (living document)
//!
//! Placeholder example whose sole purpose is to hold the current plan
//! as doc comments. No code; running it does nothing.
//!
//! Last updated: 2026-04-21, end of session.
//!
//! ## Where we are
//!
//! `spwm_17_combined` is **confirmed on-panel to be echo-free and
//! flicker-free** (measured data-phase rate 65 Hz, dark 16.9 %, E25
//! pattern shows no residue on rows 0/32). The original Thread A
//! framing ("verify brightness of spwm_15 vs spwm_14") is complete,
//! and so are its downstream follow-ups.
//!
//! Key findings from the session (all measured, not conjectured):
//!   - Brightness: spwm_14 ≈ spwm_15 ≈ spwm_17 — architecture choice
//!     does not meaningfully change brightness.
//!   - The "~2.6 ms dual-SM handover dark gap" framing in spwm_14's
//!     and spwm_15's headers is **wrong**. It's actually the PIO
//!     transfer time for the 16 936-word frame buffer at clkdiv=2
//!     (12 cycles/word × 13.3 ns = 160 ns/word × 16 936 = 2.71 ms).
//!     Both architectures incur the same gap. Single-SM's architectural
//!     benefit was always echo elimination, never flicker.
//!   - Flicker is **frequency-driven**, not duty-cycle-driven. 64 Hz
//!     with 17 % dark looks flicker-free; 39 Hz with 10 % dark looks
//!     visibly flickery. Clearing the human fusion threshold matters
//!     more than minimising dark time.
//!   - The architectural fix for flicker is **fixed-rate refresh**
//!     (run the data phase at a fixed cadence regardless of core-1
//!     pack rate) — what spwm_12 did before. spwm_14/15 tied data
//!     rate to pack rate, which collapsed it to ~39 Hz.
//!
//! spwm_17 combines spwm_15's program-swap (echo fix) with spwm_12's
//! `POST_SCAN_CYCLES = 50` fixed-rate refresh (flicker fix). First
//! configuration in this project's progression to clear both failure
//! modes.
//!
//! ## Supporting probes (now reference artefacts)
//!
//!   - `brightness_14.rs` — dual-SM probe, 39 Hz, visible flicker.
//!   - `brightness_15.rs` — single-SM program-swap, 37 Hz, visible flicker.
//!   - `brightness_12.rs` — split-DMA unified-program, 64 Hz, no flicker.
//!     All three share the same full-frame grey / scrolling rainbow
//!     fill via `MOVING` and `GREY_LEVEL` consts for valid comparison.
//!
//! ## What is deliberately NOT yet done (pick next session)
//!
//! Nothing in this list is a blocker on the investigation — spwm_17
//! works. These are follow-ups ordered by rough value-to-effort ratio.
//! Review the order against project priorities before committing.
//!
//! ### 1. Doc hygiene (low effort, high clarity payoff)
//!
//! Correct the misleading "~2.6 ms handover gap" framing in:
//!   - `spwm_14_no_pin_handover.rs` header (paragraph about the dark
//!     gap reason — still says handover).
//!   - `spwm_15_single_sm_ported.rs` header (open brightness question
//!     is now closed; "perhaps a little dimmer" text is resolved).
//!   - This file (consider replacing Thread-A section entirely with
//!     the summary above).
//! Rationale: current text will mislead future-us. ~30 minutes.
//!
//! ### 2. POST_SCAN_CYCLES sweep in spwm_17
//!
//! Map the brightness-vs-flicker tradeoff curve for this chip. Run
//! spwm_17 at `POST_SCAN_CYCLES` ∈ {20, 30, 50, 80, 120} and record:
//!   - data freq (Hz) — must stay above ~60 for flicker-free
//!   - dark fraction
//!   - subjective brightness and flicker
//! Serves flicker > brightness > framerate priority. ~1 hr.
//!
//! ### 3. PIO data clkdiv=1 experiment (risky)
//!
//! Halve the data-phase dark gap by doubling PIO data-path clock
//! (75 → 150 MHz → DCLK 37.5 → 75 MHz). If DP3364S accepts the
//! faster CLK, this compounds with (2) — same refresh freq at half
//! dark, or higher refresh freq at same dark. Risk: chip may produce
//! corrupted output above some CLK threshold. Test carefully with
//! E25 pattern. ~1 hr including recovery-from-corruption.
//!
//! ### 4. Thread B — mechanism dive (scientific, not operational)
//!
//! We know *what* works (program swap creates pipeline reset) but not
//! *why* the chip treats the swap as a discontinuity. Candidates from
//! the original Thread B plan:
//!   - CLK-idle during register writes
//!   - Edge-pattern difference at wrap
//!   - The forced JMP itself
//! Not needed for shippable correctness; would deepen understanding
//! of the DP3364S family. ~2-4 hr.
//!
//! ### 5. Extended-uptime run
//!
//! spwm_17 has only been tested for seconds. Leave it running for
//! hours and confirm: no drift, no sync loss, no accumulating echo
//! from some rare event, no thermal effects. Cheap to start.
//!
//! ### 6. `pack_pixels` profiling on core 1
//!
//! Core 1 takes ~26 ms per frame. Adding timing inside `core1_task`
//! would reveal where (gamma LUT? bit-shuffle? fill?). If we can
//! halve pack time, core-1 utilisation drops from ~100 % to ~50 %,
//! freeing it for application work. Independent of display-flicker
//! work. ~1 hr.
//!
//! ### 7. Real-content test
//!
//! We've only tested static grey, static E25, and scrolling rainbow.
//! Feed real images (USB display input, photo data, video frames) and
//! verify no surprises. Required before considering any downstream
//! application.
//!
//! ### 8. Thread C — productionise
//!
//! Extract spwm_17's architecture into a clean library crate with
//! `Display::new` / `display.present(&framebuffer)` API. Hide DMA,
//! program swap, SIO FIFO, startup flush behind the facade. Defer
//! until (5) and (7) pass — productionising an untested architecture
//! is premature.
//!
//! ## Suggested order
//!
//! (1) first — costs nothing, prevents confusion. Then either (2) or
//! (5) depending on appetite: (2) if feeling investigative (more
//! numbers, more curves), (5) if wanting to build confidence (just
//! leave it running). (3), (4), (6), (7), (8) after those.
//!
//! ## Current demo default
//!
//! `spwm_17_combined.rs` ships with `ECHO_TEST=false` (scrolling
//! rainbow). Flip to `true` for the E25 static four-block pattern
//! that exposes echo. Everything else is a `cargo run --release
//! --example <name>` away.
//!
//! ```sh
//! # Do not run this file — it's just the plan.
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
