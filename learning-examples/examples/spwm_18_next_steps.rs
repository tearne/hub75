//! # DP3364S S-PWM — Step 18: next-steps plan (living document)
//!
//! Placeholder example whose sole purpose is to hold the current plan
//! as doc comments. No code; running it does nothing.
//!
//! Last updated: 2026-04-22 (POST_SCAN_CYCLES sweep, clkdiv experiment,
//! datasheet-driven register sweep all complete; operating point
//! rebaselined to in-spec clocking and max current).
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
//! ### ~~1. POST_SCAN_CYCLES sweep in spwm_17~~ — DONE 2026-04-22
//!
//! Results (measured on-panel, full-frame static rainbow):
//!
//! | cycles | data freq | dark % | verdict                       |
//! |-------:|----------:|-------:|-------------------------------|
//! |     20 |   ~129 Hz |  ~37 % | flicker-free, clearly dim     |
//! |     30 |    ~98 Hz |  ~27 % | flicker-free, subtly dim      |
//! |     50 |     65 Hz |  17 %  | **sweet spot — comfortable**  |
//! |     57 |     58 Hz |  15 %  | borderline flicker            |
//! |     65 |    ~51 Hz |   —    | visible flicker               |
//! |     80 |    ~42 Hz |   —    | clear flicker                 |
//!
//! Findings:
//!   - Flicker edge is at ~58 Hz data rate — just above the textbook
//!     ~60 Hz fusion threshold once subjective comfort margin is
//!     accounted for. 50 cycles (65 Hz) has comfortable margin.
//!   - Brightness perception is log-response: 17 % → 27 % dark is
//!     barely visible on still content, 17 % → 37 % is obvious.
//!     Motion masks small brightness differences; A/B with still
//!     pattern exposes them.
//!   - Going above 50 costs flicker with essentially no brightness
//!     gain. Going below 50 costs brightness with flicker margin
//!     you don't need.
//!   - **Operating default fixed at `POST_SCAN_CYCLES = 50`.** The
//!     only remaining lever for more brightness is shrinking the
//!     data phase itself — see the clkdiv experiment below, which
//!     closed that door.
//!
//! ### ~~1. PIO data clkdiv experiment~~ — DONE 2026-04-22
//!
//! Outcome (initial): no practical brightness lever via overclocking.
//!   - `clkdiv=1` (75 MHz): gross corruption, power-cycle required.
//!   - `clkdiv=1.5` (50 MHz avg): intermittent glitching.
//!   - `clkdiv=1.75` (43 MHz avg): *worse* than 1.5.
//!
//! **Fractional clkdivs are a dead end** for HUB75 shift chains:
//! PIO synthesises non-integer rates by interleaving N- and
//! (N+1)-cycle ticks, so instantaneous periods always dip to the
//! floor rate — at any clkdiv ∈ (1,2) the chain intermittently
//! sees the 75 MHz instantaneous rate and fails. Only integer
//! divisors are usable.
//!
//! Outcome (follow-up after datasheet): dropped clkdiv to 3
//! (DCLK = 25 MHz, in-spec per §6.1 max FCLK). Visually
//! **indistinguishable** from clkdiv=2 (37.5 MHz) — overclocking
//! was costing us reliability for ~7 pp of invisible dark fraction.
//! **Operating clkdiv rebaselined to 3.**
//!
//! ### ~~1b. DP3364S register sweep~~ — DONE 2026-04-22
//!
//! Triggered by finding the datasheet (now checked in at
//! `reference/DP3364S/`) plus HUB320 register tables on the hzeller
//! collation thread. Both surfaced that our `CONFIG_REGS` was only
//! writing 13 of the chip's 15 registers, and the one we cared about
//! most — `reg0x08`, the output-current scale — was at an
//! arbitrary-looking `0xBF` (191).
//!
//! **reg 0x08 A/B** (baseline 0xBF vs test value), in-spec clock:
//!   - 0x80 (128): clearly dimmer — knob works.
//!   - 0xDF (223): a bit brighter than 0xBF.
//!   - 0xFF (255): brighter still. No instability observed.
//!
//! **reg 0x0F A/B** (0x10 nominal vs 0x20 HUB320) with reg0x08=0xFF:
//!   - No visible difference. LEDs appear saturated at reg0x08=0xFF
//!     and 0x0F can't push further. Settled at 0x10 (no reason to
//!     pick the nominal-higher-current value for zero gain).
//!
//! **reg 0x0E**: undocumented in our datasheet copy. HUB320 writes
//! per-colour values but no reference tells us what they mean.
//! Decision: leave unwritten (accept chip's power-on default
//! rather than blind-poke an undocumented register).
//!
//! **Operating point rebaselined** (all in CONFIG_REGS):
//!   - clkdiv = 3 (25 MHz, in-spec)
//!   - POST_SCAN_CYCLES = 50 (from earlier sweep)
//!   - reg 0x08 = 0xFF (max current)
//!   - reg 0x0F = 0x10 (nominal, explicit)
//!   - reg 0x0E omitted
//!
//! Net vs start of session: same or brighter, fully in-spec, no
//! undocumented register pokes. Unusual win.
//!
//! ### 1. Thread B — mechanism dive (scientific, not operational)
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
//! ### 2. Extended-uptime run
//!
//! spwm_17 has only been tested for seconds. Leave it running for
//! hours and confirm: no drift, no sync loss, no accumulating echo
//! from some rare event, no thermal effects. Cheap to start.
//!
//! ### 3. `pack_pixels` profiling on core 1
//!
//! Core 1 takes ~26 ms per frame. Adding timing inside `core1_task`
//! would reveal where (gamma LUT? bit-shuffle? fill?). If we can
//! halve pack time, core-1 utilisation drops from ~100 % to ~50 %,
//! freeing it for application work. Independent of display-flicker
//! work. ~1 hr.
//!
//! ### 4. Real-content test
//!
//! We've only tested static grey, static E25, and scrolling rainbow.
//! Feed real images (USB display input, photo data, video frames) and
//! verify no surprises. Required before considering any downstream
//! application.
//!
//! ### 5. Thread C — productionise
//!
//! Extract spwm_17's architecture into a clean library crate with
//! `Display::new` / `display.present(&framebuffer)` API. Hide DMA,
//! program swap, SIO FIFO, startup flush behind the facade. Defer
//! until (2) and (4) pass — productionising an untested architecture
//! is premature.
//!
//! ## Suggested order
//!
//! Either (1) or (2) depending on appetite: (1) if feeling
//! investigative (explain the program-swap mechanism), (2) if wanting
//! to build confidence (just leave spwm_17 running). (3), (4), (5)
//! after those.
//!
//! ## Current demo default
//!
//! `spwm_17_combined.rs` ships with `ECHO_TEST=false` + `MOVING=true`
//! (scrolling rainbow). Flip `ECHO_TEST` to `true` for the E25 static
//! four-block pattern that exposes echo. Flip `MOVING` to `false` to
//! freeze the rainbow for brightness A/B probing — motion masks small
//! brightness differences. Everything else is a `cargo run --release
//! --example <name>` away.
//!
//! ```sh
//! # Do not run this file — it's just the plan.
//! # cargo run --release --example spwm_18_next_steps
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
    defmt::info!("spwm_18 is a planning placeholder; see header for next steps.");
    loop {
        cortex_m::asm::wfe();
    }
}
