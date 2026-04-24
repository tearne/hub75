//! # DP3364S S-PWM — Step 29: SET-only PINCTRL swap (narrowing)
//!
//! ## Purpose
//!
//! spwm_28 (2026-04-24) proved PINCTRL swap at phase boundary is the
//! echo-killing mechanism. It changed TWO fields simultaneously:
//!   - OUT_COUNT: 6 ↔ 11
//!   - SET_BASE/SET_COUNT: 12/2 ↔ 13/1
//!
//! This narrows by keeping OUT_COUNT=11 for both phases and swapping
//! only SET_BASE/SET_COUNT. Prediction: OUT_COUNT is not behaviorally
//! load-bearing (an `out PINS, N` instruction writes exactly N bits
//! regardless of OUT_COUNT when OUT_COUNT ≥ N, so pins 6-10 are
//! retained unchanged in both cases). Therefore SET_BASE/SET_COUNT
//! should be the load-bearing field.
//!
//! Behaviorally, the SET swap means:
//!   - During data, SET targets pins 12-13 (LAT+OE).
//!   - During scan, SET targets only pin 13 (OE); pin 12 (LAT) is
//!     NOT re-asserted by scan SET instructions — it retains its
//!     end-of-data value (LAT=0) via the output latch.
//!
//! In spwm_27 (no PINCTRL swap), scan's `set PINS 0b10` drives LAT=0
//! alongside OE=1 — twice per scan_word. In spwm_29 (like spwm_28),
//! LAT pin is never touched during scan.
//!
//! ## Outcomes
//!
//! - **Echo-free + display correct** → SET_BASE/SET_COUNT swap alone
//!   is sufficient. Mechanism fully localized to SET PINCTRL fields.
//! - **Echo present** → OUT_COUNT swap was somehow contributing too.
//!   Next: spwm_30 (OUT_COUNT-only swap) to isolate further.
//!
//! Delta from spwm_28: data_pinctrl OUT_COUNT changed 6 → 11 (now
//! matches scan_pinctrl OUT_COUNT and base PINCTRL). Everything else
//! is spwm_28 unchanged.
//!
//! ```sh
//! cargo run --release --example spwm_29_pinctrl_set_only
//! ```
//!
//! ## Inherited context from spwm_27
//!
//! spwm_27 (EXECCTRL-only swap) echoes (confirmed 2026-04-24 after
//! WRAP_TOP off-by-one fix). spwm_28 inherits the same WRAP_TOP
//! constants (DATA_WRAP_TOP_LOCAL=10, SCAN_WRAP_TOP_LOCAL=26).
//!
//! ## Thread B next session plan (priority order)
//!
//! ### A. Debug spwm_27 (cheap, closes the last firmware swap test)
//!
//! - Read `SM0_ADDR` (PIO0_BASE + 0x0D4 on RP2350) after
//!   `swap_to_scan` to confirm PC actually landed at scan_entry=11.
//!   One defmt line resolves "did the JMP take effect?".
//! - Try swap_to_scan writing ONLY `SM_INSTR` (skip EXECCTRL) while
//!   keeping the original explicit `JMP scan_entry` at end of scan
//!   (revert that edit). If it works, add EXECCTRL write back and
//!   re-test. Narrows to whether EXECCTRL write specifically or
//!   the wrap-based scan loop is the problem.
//! - Sample `SM0_FDEBUG` at the hang for TX-stall / overflow state.
//!
//! **Outcome value:** confirms or refutes "persistent EXECCTRL-only
//! swap alone is sufficient". If echo-free, firmware simplification
//! achievable; if echoes, "two separately-assembled programs at
//! different PC offsets" is load-bearing and spwm_17 is the minimum.
//!
//! ### B. Unexplored chip registers (cheap, speculative)
//!
//! Datasheet confirms 15 valid register addresses (0x01–0x0F).
//! CONFIG_REGS currently writes 13. Deliberately skipped:
//! **reg 0x01, reg 0x0E**. Reg 0x0F is partially documented
//! (current-gain formula). Reg 0x0E has NO documentation in the
//! datasheet text — purely speculative territory.
//!
//! - Sweep reg 0x0E values {0x00, 0x55, 0xAA, 0xFF}. Cheap; might
//!   suppress the scan-phase latch behavior if that's what 0x0E
//!   controls.
//! - Write reg 0x01 — never written, unknown function. Similar
//!   speculative sweep.
//!
//! ### C. Different PWM display modes via reg 0x0C[7:6] (cheap)
//!
//! Current reg 0x0C = 0x88 sets `[7:6] = 0b10` (NOT the documented
//! "General Frame Sync" mode = 0b00). Four modes per §10.7:
//!   1. General Frame Sync (0b00)
//!   2. High-gray Data Refresh Frame Sync
//!   3. High-gray Data Refresh Async
//!   4. Low-gray High-brush
//!
//! Echo might be specific to mode 0b10. Test 0b00 (which the
//! datasheet walks through in detail). If echo-free in a different
//! mode, alternative architecture found.
//!
//! **Caveat:** different modes may require different command
//! sequences — changing reg 0x0C[7:6] alone might break display.
//! Cross-reference §10.7.1–§10.7.4 for mode-specific requirements.
//!
//! ### D. Tighter RGB=0 trigger characterization (moderate)
//!
//! Proved: RGB=0 causes echo (spwm_21), RGB=1 doesn't (spwm_22) in
//! spwm_17 architecture. Still unclear which bit(s) matter or
//! whether it's the all-zeros pattern specifically.
//!
//! - **Bit-pattern sweep:** drive RGB during scan with {0x00, 0x01,
//!   0x02, 0x04, 0x08, 0x10, 0x20, 0x15, 0x2A, 0x3F}. Narrows
//!   whether a specific pin (R0, B1, etc.) or the all-zero pattern
//!   is the trigger.
//! - **Intermittent RGB=0:** drive RGB=0 only on every Nth scan
//!   word. Tests cumulative vs instantaneous trigger.
//! - **Per-scan-line RGB:** drive RGB=0 only on scan_line 31,
//!   non-zero elsewhere. Tests if scan_line 31 specifically
//!   matters (it's the echo source) or any scan_line with RGB=0.
//!
//! ### E. Logic analyzer session (after A–D exhausted)
//!
//! Capture CLK, LAT, OE, ADDR[4:0], RGB[5:0] around the scan_line
//! 31→0 boundary under spwm_22 (echo-free) vs spwm_23 (echoes).
//! Direct observation is much higher information density than
//! more compile-flash experiments.
//!
//! ## Purpose
//!
//! Last firmware-only discriminator for Thread B. Tests whether
//! giving the scan phase its **own persistent wrap range** (EXECCTRL
//! WRAP_TOP/BOTTOM) — while keeping all other PIO config identical
//! between phases — reproduces spwm_22's echo-free behavior.
//!
//! PINCTRL is deliberately NOT swapped: spwm_26 established that
//! swapping `SIDESET_COUNT` at runtime doesn't work in a single
//! pre-assembled program (PIO reinterprets each instruction's
//! delay/side-set bit split, which breaks the encoded side-set
//! values). So the test here is whether wrap persistence alone is
//! the mechanism.
//!
//! ## Prior state (Thread B so far)
//!
//! - CLK idle pause: ruled out (spwm_19)
//! - Single-register touch (PINCTRL/EXECCTRL/JMP): ruled out (spwm_20)
//! - SM disable/enable: ruled out (prior to spwm_20)
//! - RGB=0 during scan: triggers echo in spwm_22-derived builds;
//!   doesn't alone fix it in unified builds (spwm_22 vs spwm_23)
//! - LAT drive via side-set: ruled out (spwm_24)
//! - Dispatch prelude: ruled out (spwm_25)
//! - Persistent EXECCTRL+PINCTRL swap in unified program: not
//!   cleanly testable (spwm_26, side_set_count reinterpretation bug)
//!
//! If spwm_27 is echo-free: simplification achievable with "one
//! program + one additional register write per phase (EXECCTRL)".
//! If spwm_27 echoes: the final remaining architectural variable is
//! "two separately-assembled programs at different PC offsets",
//! which can't be replicated in a single unified program. spwm_17
//! becomes the minimum viable echo-free architecture.
//!
//! ## Changes from spwm_25
//!
//! 1. Scan path ends at `scan_wrap_source` label (binding, no
//!    explicit JMP scan_entry). Scan loops via PIO wrap.
//! 2. Compute data_execctrl (data wrap) and scan_execctrl (scan
//!    wrap) at runtime, masking to 5 bits.
//! 3. Force install at PIO offset 0 (origin=Some(0)).
//! 4. Phase-boundary helper writes EXECCTRL + SMx_INSTR. PINCTRL
//!    stays fixed at side_set_count=1.
//!
//! ```sh
//! cargo run --release --example spwm_27_execctrl_swap
//! ```
//!
//! ---
//!
//! # Inherited spwm_25 header follows
//!
//! # DP3364S S-PWM — Step 25: unified-program, no dispatch prelude
//!
//! ## Purpose
//!
//! Test whether the type-bit dispatch prelude (`OUT X, 1` +
//! `JMP X==0`) between every word in spwm_23/24 is the remaining
//! trigger that distinguishes them from spwm_22 (echo-free). If
//! yes, we've localized the mechanism and achieved the
//! simplification goal with a minimal "swap" (one SMx_INSTR write
//! per phase transition).
//!
//! ## Prior state
//!
//! | file    | architecture                         | dispatch | echo |
//! | ------- | ------------------------------------ | -------- | ---- |
//! | spwm_22 | swap, 1-bit scan prog, OUT PINS 11   | no       | no   |
//! | spwm_23 | unified, 2-bit side-set              | yes      | echo |
//! | spwm_24 | unified, 1-bit side-set, SET-LAT     | yes      | echo |
//!
//! ## This experiment
//!
//! Remove the dispatch prelude entirely. Have two entry points in
//! one PIO program:
//!   - `data_cmd` — first instruction (PC 0). Reached via program
//!     wrap after lat_loop, or via CPU-forced JMP at phase boundary.
//!   - `scan_entry` — reached only via CPU-forced JMP. Scan path
//!     ends with `JMP scan_entry` (self-loop), so subsequent scan
//!     words within the same phase run without CPU intervention.
//!
//! At each phase boundary (after `wait_txstall`) the CPU writes
//! `SM0_INSTR = JMP <target>` — one register write per phase
//! transition. Much simpler than spwm_17's three-register swap.
//!
//! ## Changes from spwm_24
//!
//! 1. PIO program: remove dispatch prelude. Data path is first
//!    (PC 0). Scan path placed after wrap_source, reachable only
//!    via forced JMP.
//! 2. `data_header`: drop type bit. `(n_pre-1) | ((n_lat-1) << 15)`.
//!    `data_cmd` does `OUT X 15` + `OUT Y 16` + `OUT NULL 1` = 32.
//! 3. `scan_word`: drop type bit. display(7) + RGB=0x3F(6) +
//!    addr(5) + setup(5) + oe(4) + drain(5) = 32. Matches spwm_22.
//! 4. Main loop (and flush/sync): force-JMP `scan_entry` before
//!    scan DMAs; force-JMP `data_cmd` before data DMAs.
//!
//! ## Outcomes
//!
//! - **Echo-free + display correct** → dispatch prelude was the
//!   missing factor. Simplification target met with one register
//!   write per phase (vs spwm_17's three).
//! - **Echo present** → dispatch isn't it. Remaining structural
//!   difference is "two installed programs at different PC
//!   offsets", which is weird; would imply the swap's persistent
//!   EXECCTRL/PINCTRL values matter (contradicting spwm_20's
//!   null since save-restore differs from persistent-change).
//! - **Garbled display** → refactor bug; debug.
//!
//! ```sh
//! cargo run --release --example spwm_25_no_dispatch
//! ```
//!
//! ---
//!
//! # Inherited spwm_24 header follows
//!
//! # DP3364S S-PWM — Step 24: unified-program, LAT undriven during scan
//!
//! ## Purpose
//!
//! Discriminate between the remaining candidates for why spwm_23's
//! unified-program architecture still echoes even with RGB=1 scan
//! words, when spwm_22's scan-only program with the same RGB=1 does
//! not. This file tests whether spwm_17/22's "LAT not driven by the
//! scan program" behaviour is the differentiator.
//!
//! ## Prior state
//!
//! | file    | architecture                       | RGB during scan | echo |
//! | ------- | ---------------------------------- | --------------- | ---- |
//! | spwm_17 | swap, 1-bit side-set scan program  | undriven        | no   |
//! | spwm_22 | swap, 1-bit side-set, OUT PINS 11  | ONE             | no   |
//! | spwm_23 | unified, 2-bit side-set, OUT PINS 11 | ONE             | echo |
//!
//! spwm_22 → spwm_23 is the generalization that failed. Structural
//! differences include: swap vs unified, type-bit dispatch prelude
//! vs none, 1-bit vs 2-bit side-set (→ LAT driven-to-zero every
//! instruction vs LAT left alone during scan).
//!
//! ## This experiment
//!
//! Refactor spwm_23's PIO program to use 1-bit side-set (CLK only),
//! moving LAT control to SET instructions. During data phase two SET
//! writes raise/lower LAT around lat_loop. During scan phase the two
//! existing SETs (OE enable/disable) will incidentally also write
//! LAT=0 alongside OE — matching the chip-visible LAT behaviour of
//! spwm_22 (LAT always 0 during scan), but with two SET-based
//! writes per scan word rather than spwm_22's zero writes. That's
//! a much closer approximation than spwm_23's ~15 side-set-based
//! LAT=0 writes per scan word.
//!
//! ## Changes from spwm_23
//!
//! 1. `SideSet::new(false, 2, false)` → `SideSet::new(false, 1, false)`
//! 2. `set_pins(13, 1)` → `set_pins(12, 2)` — SET now covers LAT + OE
//! 3. All PIO instructions' side-set values drop the LAT bit:
//!    0b00→0, 0b01→1, 0b10→0 (needs preceding SET), 0b11→1
//! 4. Data path: insert `SET PINS 0b01` (LAT=1, OE=0) before lat_loop,
//!    `SET PINS 0b00` (LAT=0, OE=0) after
//! 5. Scan path: existing OE enable changes from `SET PINS 1` →
//!    `SET PINS 0b10` (LAT=0, OE=1); OE disable stays `SET PINS 0`
//!    (now interpreted as both pins = 0).
//!
//! ## Outcomes
//!
//! - **Echo-free, display correct**: side-set-based LAT=0 rewriting
//!   was the missing piece. Unified-program architecture with 1-bit
//!   side-set works. Simplification achieved (one SM, one program,
//!   no swap, though with extra SET instructions).
//! - **Echo still present**: LAT redriving isn't the mechanism
//!   either. Remaining candidates (type-bit dispatch prelude, or the
//!   act of having two installed programs at all) become primary.
//! - **Display garbled**: bit budget or SET timing broke something.
//!   Need to debug before drawing conclusions.
//!
//! ```sh
//! cargo run --release --example spwm_24_unified_lat_idle
//! ```
//!
//! ---
//!
//! # Inherited spwm_23 header follows
//!
//! # DP3364S S-PWM — Step 23: unified-program, RGB=1 during scan
//!
//! ## Purpose
//!
//! The simplification test. spwm_12's unified-PIO-program architecture
//! (one SM, one program, type-bit dispatch, echo-affected) but with
//! one targeted change: scan-word RGB bits are set to ONE instead of
//! ZERO. If Thread B's finding holds (echo triggered by RGB=0 during
//! scan), this should produce:
//!
//!   - one SM
//!   - one PIO program
//!   - no program-swap machinery
//!   - no boundary ceremony
//!   - **and no echo**
//!
//! That is the full simplification goal we set out for Thread B.
//!
//! ## Change from spwm_12
//!
//! One line: `scan_word()` sets bits 8-13 (RGB field) to `0x3F`
//! instead of 0. All callers — inline post-scan words, sync-phase
//! scan buffer — inherit the change automatically.
//!
//! ## Risks / gotchas to watch for
//!
//! - **Sync phase:** the startup sync phase uses W12 on scan_line 0
//!   to align driver chip SRAM pointers. Driving RGB=1 during sync
//!   might interact with that process and fail to bootstrap. If the
//!   panel is blank or scrambled at startup, split the change so
//!   sync uses RGB=0 and main loop uses RGB=1.
//! - **Chip state carryover:** spwm_17's echo-free behaviour is
//!   subtly different from "RGB=1 during scan" — spwm_17 leaves RGB
//!   pins in whatever state the data phase left them. RGB=1 is a
//!   different state than "last-data-bit state". If there's a
//!   value-specific sensitivity other than the one tested in
//!   spwm_22, this could expose it.
//!
//! ## Outcomes
//!
//! - **Display correct, no echo:** simplification achieved. spwm_17
//!   can be retired. This is the target result.
//! - **Echo present:** Thread B's RGB=1 finding doesn't generalize
//!   from the spwm_17-derived spwm_22 architecture back to spwm_12's
//!   unified-program architecture. Something else in the spwm_12 /
//!   spwm_22 difference matters.
//! - **Garbled display:** RGB=1 interacts with something in the
//!   unified program (the type-bit dispatch prelude, or the wider
//!   OUT PINS 11 instruction) that breaks. Fall back to more
//!   targeted tests.
//!
//! ```sh
//! cargo run --release --example spwm_23_unified_rgb_one
//! ```
//!
//! ---
//!
//! # Inherited spwm_12 header follows
//!
//! # DP3364S S-PWM — Step 12: split DMA (echo investigation, HISTORICAL)
//!
//! Twelfth in the SPWM progression. Copied from spwm_11 (unrotated
//! post-scan, `POST_SCAN_CYCLES = 50`) and modified to emit each frame
//! as **two DMAs** with a `wait_txstall` between them:
//!
//! ```text
//!   DMA 1: [VSYNC | PRE_ACT | WR_CFG | DATA_LATCH × 512]   (DATA_END words)
//!   wait_txstall                                           ← clean pipeline gap
//!   DMA 2: [SCAN_WORD × 32 × POST_SCAN_CYCLES]             (post-scan region)
//!   wait_txstall
//! ```
//!
//! ## Role in the investigation (now closed)
//!
//! This file was the diagnostic vehicle for the scan_line-31 → scan_line-0
//! echo. It is the simplest program that reliably reproduces the echo,
//! so E21–E25 were implemented here to probe individual hypotheses. The
//! investigation is now **complete** — see spwm_15 for the resolution.
//! This file is preserved as the historical echo reproducer and for the
//! experiment log in the E21–E25 sections below.
//!
//! **Bottom line:** the echo came from our **unified PIO program with
//! type-bit dispatch**. Replacing that with two separate PIO programs
//! swapped on the same SM at each phase boundary (spwm_15) eliminated
//! the echo entirely. Details in spwm_15's header.
//!
//! ## E19 result (the original purpose of this file)
//!
//! **Echo still visible.** Inserting `wait_txstall` between the data
//! and post-scan DMAs did not eliminate the scan_line 31 → scan_line 0
//! leak. Pipeline-drain at the MCU side is not sufficient. That ruled
//! out the "DATA → SCAN boundary adjacency" hypothesis at the
//! coarse-grained level. Further null results accumulated in E17,
//! E18, E20 and the other toggles already in this file (see the
//! consts `INTER_DMA_DELAY_CYCLES`, `INTER_DMA_INJECT`,
//! `INTER_DMA_HIGHZ_CYCLES`).
//!
//! ## E21–E25 diagnostic sweep
//!
//! Motivation: between spwm_12 (echo) and spwm_14 (clean), the
//! not-yet-individually-isolated variables are PIO state carryover,
//! TX FIFO residue, single vs dual DMA channel, and unified vs
//! separate PIO programs. The spwm_15 refactor targets the last one
//! but is expensive to execute. These five cheaper experiments narrow
//! the hypothesis space first. Each is a small localised change to
//! spwm_12.
//!
//! Record for each: **Status** (not run / run / ongoing) and
//! **Observation** (echo unchanged / reduced / eliminated / other).
//!
//! ### E21 — SM_RESTART between phases
//! Write `PIO0_CTRL.SM_RESTART` bit 4 (SM0) after `wait_txstall` and
//! before the post-scan DMA. Per RP2350 datasheet §12.5, this clears:
//! the input and output shift counters, the ISR contents, the delay
//! counter, the waiting-on-IRQ state, any stalled instruction, any
//! stalled OUT/PULL/IN/PUSH due to full-empty FIFOs, and the status
//! register. It does **not** clear X, Y, OSR contents, the FIFOs
//! themselves, or the PC.
//!
//! - **Prediction:** if the echo is caused by carryover of any of the
//!   things this clears (most likely the output shift counter, since
//!   autopull state could plausibly misalign the scan phase's first
//!   OUT), echo disappears.
//! - **Implementation:** const `E21_SM_RESTART` (0/1); when 1,
//!   `restart_sm(SM0_RESTART)` fires between data-phase `disable_sm`
//!   and the inter-DMA toggles. Compared against `echo_baseline`
//!   (same architecture, no E21) by flashing both in turn.
//! - **Status:** run 2026-04-21 (both modes flashed against each
//!   other, panel reset via `panel_reset` between runs).
//! - **Observation:** **echo still present, no perceptible change
//!   from baseline.** Prediction falsified. The echo is not caused
//!   by carryover of any of the state `SM_RESTART` clears: input/
//!   output shift counters, ISR, delay counter, waiting-on-IRQ
//!   state, stalled instructions, stalled FIFO ops, or the status
//!   register. Remaining PIO-internal suspects are the X/Y scratch
//!   registers and OSR content — exactly what E22 targets.
//!
//! ### E22 — explicit OSR / X / Y scrub via SMx_INSTR
//! Force-execute `MOV OSR, NULL`, `MOV X, NULL`, `MOV Y, NULL` via
//! `SM0_INSTR` at the phase boundary, while the SM is running-but-
//! stalled on autopull (before `disable_sm`). Reaches X, Y, OSR
//! content — the scratch state `SM_RESTART` specifically leaves
//! alone. Writes to `SMx_INSTR` override the stalled instruction
//! in the next PIO cycle; a short `delay(10)` between writes
//! (~67 ns at 150 MHz, ~5 PIO cycles at 75 MHz) ensures each scrub
//! executes before the next overwrites.
//!
//! - **Prediction (initial):** if residue in OSR / X / Y carries the
//!   echo, scrubbing all three eliminates it.
//! - **Status:** abandoned 2026-04-21 — experiment design was flawed.
//! - **Observation:** panel went blank, scan phase visibly broken.
//!   Diagnosis: `MOV OSR, NULL` resets the output shift counter to
//!   0 as a side effect, which un-stalls the SM from its autopull
//!   wait. The SM then executes pending instructions using zero
//!   data as command bytes (consuming 32 zero bits through dispatch
//!   / data_cmd), stuffing X=0 Y=0 and stalling at a different
//!   point in data_cmd's `out Y, 16`. When the scan DMA starts,
//!   scan words get interpreted as data-command payload bits,
//!   producing nonsense on the pins.
//! - **Deeper conclusion:** the experiment was testing state that
//!   can't actually carry the echo. OSR is strictly consumed by
//!   OUT instructions — every bit is output exactly once, and at
//!   stall time the counter is 32 (OSR fully drained). There is
//!   no residue. X and Y are overwritten by `out X, ...` /
//!   `out Y, ...` at the top of the data path before use, so
//!   residue there is also inert. Combined with the E21 null, all
//!   plausible per-SM internal state is now exonerated. Focus
//!   should shift to what happens **outside** the PIO — the FIFO
//!   (E23 sanity check), the chip-side shift registers (E24), or
//!   the spatial nature of the leak (E25).
//!
//! ### E23 — FIFO-empty assertion post-stall (SKIPPED)
//! Would have confirmed our assumption that wait_txstall implies the
//! TX FIFO is genuinely empty. Skipped because E21's null already
//! indirectly confirms the autopull-stall state; E25 then made this
//! irrelevant by showing the leak is chip-side.
//!
//! ### E24 — dummy drain words between phases (SKIPPED)
//! Would have tested chip-side shift-register residue by clocking
//! extra zero DATA_LATCHes between data and scan. Skipped because
//! E25's finding (echo is per-column SRAM-level, preserving colour
//! exactly, no smear) argues strongly against a serial-chain mechanism
//! — bits stuck in a shift register would produce smearing, not
//! 1:1 column-preserved mirroring. The pipeline-continuity hypothesis
//! was then validated architecturally by spwm_15's success.
//!
//! ### E25 — colour-asymmetric / spatial test patterns
//! Replace the uniform-per-row pattern with spatially distinct
//! blocks so the echo's structure is readable. Current E25 pattern:
//!
//! |              | cols 0-63 | cols 64-127 |
//! | ------------ | --------- | ----------- |
//! | row 31       | RED       | BLUE        |
//! | row 63       | GREEN     | YELLOW      |
//!
//! Shape of the echo on rows 0 / 32 narrows the mechanism —
//! per-column SRAM leak, serial shift-register leak, cross-half
//! bleed, column-selective, etc. See `fill_pixels` docstring for
//! the full decision table.
//!
//! - **Implementation:** modified `fill_pixels` in both spwm_12
//!   and echo_baseline to emit the four-block pattern. No other
//!   code changes.
//! - **Status:** run 2026-04-21.
//! - **Observation:** four colours on the source rows; echo on the
//!   target rows **exactly mirrors** the source rows' colour and
//!   column layout. Row 0: left-half faint red, right-half faint
//!   blue, sharp boundary at col 64. Row 32: left-half faint
//!   green, right-half faint yellow, same sharp boundary. No
//!   colour mixing, no smearing, no horizontal shift, no cross-
//!   half bleed, intensity uniform across each half.
//! - **Conclusion:** the echo is a **per-column, SRAM-address-
//!   level leak**. Column N of scan_line 31's SRAM bleeds into
//!   column N of scan_line 0's SRAM (upper half); similarly for
//!   63 → 32 (lower half). Each colour channel is preserved
//!   exactly. The mechanism operates on individual SRAM cells in
//!   1:1 column correspondence.
//!
//!   **Rules out:**
//!     - serial shift-register leak (would smear / shift the
//!       boundary at col 64);
//!     - cross-half bleed (upper source → lower target, etc.);
//!     - colour-channel-specific mechanism (all channels echo
//!       equally);
//!     - column-selective mechanism (all 128 columns affected
//!       uniformly).
//!
//!   **Remaining hypothesis space** is chip-internal, specific to
//!   the scan_line 31 ↔ 0 (and 63 ↔ 32) SRAM address pair. Most
//!   plausible: the DP3364S's address wrap/decode during the
//!   last DATA_LATCH of scan_line 31 does something that causes
//!   scan_line 0's SRAM cells to be written as well. This cannot
//!   be the MCU pipeline: our command stream is data-correct and
//!   per-column, so the chip is receiving correct bits for
//!   scan_line 31 and somehow also writing scan_line 0 from those
//!   same bits.
//!
//! ## Final outcome
//!
//! The cheap diagnostics (E21, E25) ruled MCU-side state in/out and
//! localised the mechanism to a chip-internal per-column SRAM leak at
//! command-stream wrap points. The spwm_15 refactor (single SM with
//! two separate PIO programs, swapped at phase boundaries) was then
//! confirmed on-panel to eliminate the echo entirely, while retaining
//! the flicker-reduction goal (no ~2.6 ms dual-SM dark gap). The
//! distinguishing variable was **unified-program-with-type-bit-dispatch
//! vs separate-programs-with-distinct-wrap-regions** — not single-SM
//! vs dual-SM as originally framed.
//!
//! ## Test pattern (current — E25)
//!
//! Row 31: cols 0-63 red, cols 64-127 blue.
//! Row 63: cols 0-63 green, cols 64-127 yellow.
//! Echo (if any) lands on row 0 (upper target) and row 32 (lower
//! target). `POST_SCAN_CYCLES` stays at 50 for per-row refresh
//! around 3 kHz. See `fill_pixels` docstring for how to read the
//! echo shape.
//!
//! ```sh
//! cargo run --release --example spwm_12_split_dma
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
// GROUP_SET (0x03) is FIRST: it controls SRAM address wrap, and at boot
// the chip's default is 0x3F (64-wide). If we write DATA_LATCHes with
// that stale GROUP_SET, writes miss half the SRAM. Setting it first
// guarantees every subsequent DATA_LATCH addresses the full 128-wide
// SRAM correctly.
const CONFIG_REGS: [u16; 13] = [
    0x037F, 0x1100, 0x021F, 0x043F, 0x0504, 0x0642, 0x0700,
    0x08BF, 0x0960, 0x0ABE, 0x0B8B, 0x0C88, 0x0D12,
];

// ── Per-command CLK counts ───────────────────────────────────────────
// PIO consumes 8 bits per pre/lat loop iteration; header is 32 bits;
// autopull fires every 32 bits consumed. For each command, (PRE+LAT)
// must be a multiple of 4 so commands end on a 32-bit boundary and
// the next dispatch reads a fresh word.
const VSYNC_PRE: u32 = 1;    const VSYNC_LAT: u32 = 3;
const VSYNC_WORDS: usize = 1;

const PRE_ACT_PRE: u32 = 2;  const PRE_ACT_LAT: u32 = 14;
const PRE_ACT_WORDS: usize = 4;

const WR_CFG_PRE: u32 = 123; const WR_CFG_LAT: u32 = 5;
const WR_CFG_WORDS: usize = 32;

const DATA_LATCH_PRE: u32 = 127; const DATA_LATCH_LAT: u32 = 1;
const DATA_LATCH_WORDS: usize = 32;

// ── Frame layout — fixed order VSYNC → PRE_ACT → WR_CFG → DATA ──────
// Matches spwm_6's working command order. Earlier experiments with
// VSYNC at other positions + an extra "Enable All Output" (EAO) LAT-12
// command all failed to bootstrap sync from cold on DP3364S; see the
// Experiment log in the header docs for the null-result evidence.
const VSYNC_OFFSET:   usize = 0;
const PRE_ACT_OFFSET: usize = VSYNC_OFFSET + 1 + VSYNC_WORDS;     // 2
const WR_CFG_OFFSET:  usize = PRE_ACT_OFFSET + 1 + PRE_ACT_WORDS; // 7
const HEADER_WORDS:   usize = WR_CFG_OFFSET + 1 + WR_CFG_WORDS;   // 40

const DATA_LATCH_STRIDE: usize = 1 + DATA_LATCH_WORDS;                // 33
const LATCHES_PER_LINE:  usize = 16;
const LATCHES_PER_FRAME: usize = SCAN_LINES * LATCHES_PER_LINE;       // 512

const DATA_OFFSET: usize = HEADER_WORDS;                              // 40
const DATA_END:    usize = DATA_OFFSET + LATCHES_PER_FRAME * DATA_LATCH_STRIDE; // 16 936

// E17 sweep variable. Each cycle = 32 scan_words × 132 DCLKs per
// scan_word = 4224 DCLKs. At 16.7 MHz → ~253 µs per cycle.
// Wraps-per-frame = POST_SCAN_CYCLES (every cycle boundary is a
// 31 → 0 wrap in the unrotated architecture).
//
// Sweep order: 50 (baseline) → 12 → 6 → 3 → 1, plus intermediates.
const POST_SCAN_CYCLES: usize = 50;

const SCAN_OFFSET: usize = DATA_END;
const POST_SCAN_WORDS: usize = POST_SCAN_CYCLES * SCAN_LINES;

const FRAME_WORDS: usize = SCAN_OFFSET + POST_SCAN_WORDS; // 18 536

// ── Double-buffered frame data ───────────────────────────────────────
static mut FRAME_BUF_A: [u32; FRAME_WORDS] = [0u32; FRAME_WORDS];
static mut FRAME_BUF_B: [u32; FRAME_WORDS] = [0u32; FRAME_WORDS];

// ── Scan buffer for ring-mode DMA (sync-phase only) ──────────────────
// 128-byte aligned so DMA RING_SIZE=7 wraps cleanly every 32 words.
#[repr(C, align(128))]
struct ScanBufAligned([u32; SCAN_LINES]);
static mut SCAN_BUF: ScanBufAligned = ScanBufAligned([0u32; SCAN_LINES]);

// ── Core 1 stack ─────────────────────────────────────────────────────
static mut CORE1_STACK: hal::multicore::Stack<4096> = hal::multicore::Stack::new();

// Extra idle gap between DMA 1 and DMA 2. Tested up to 100_000 cycles
// (~670 µs) → no effect on the echo, so pipeline-drain isn't the cure.
// Left here at 0 as documentation.
const INTER_DMA_DELAY_CYCLES: u32 = 0;

// VSYNC-injection experiment: after DMA 1 (data) finishes, manually
// push a VSYNC header + payload word into the PIO TX FIFO before
// starting DMA 2 (post-scan). If VSYNC is the pipeline-reset signal
// that spwm_4 implicitly benefits from, a mid-frame extra VSYNC
// should kill the echo on scan_line 0.
//   0 = no injection
//   1 = inject VSYNC
//   2 = inject VSYNC + PRE_ACT (like spwm_4's per-frame header)
const INTER_DMA_INJECT: u32 = 0;

// Pin high-Z experiment: mimic what spwm_4 actually does (state-
// machine handover releases CLK/LAT to input between phases). Here
// we force SM0 to set pindirs=0 on CLK (pin 11) and LAT (pin 12)
// between DMA 1 and DMA 2, optionally delay, then reclaim them.
//   0 = disabled
//   N = release, delay N cortex-m cycles (~6.67 ns each), reclaim
const INTER_DMA_HIGHZ_CYCLES: u32 = 0;

// ── Scan parameters ──────────────────────────────────────────────────
const DISPLAY_CLK: u32 = 100;
const SETUP_CLK: u32 = 28;
const OE_CLK_W4: u32 = 4;
const OE_CLK_W12: u32 = 12;

// ── Pixel type and framebuffer ───────────────────────────────────────

#[derive(Copy, Clone)]
struct Rgb { r: u8, g: u8, b: u8 }
impl Rgb {
    const fn new(r: u8, g: u8, b: u8) -> Self { Self { r, g, b } }
    const BLACK: Rgb = Rgb::new(0, 0, 0);
}

static mut PIXELS: [[Rgb; 128]; 64] = [[Rgb::BLACK; 128]; 64];

// ── Gamma LUT: 8-bit -> 14-bit greyscale ─────────────────────────────

static GAMMA14: [u16; 256] = {
    let mut lut = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        let v = (i as u32 * i as u32 * 0x3FFF) / (255 * 255);
        lut[i] = v as u16;
        i += 1;
    }
    lut
};

// ── HSV helper ───────────────────────────────────────────────────────

#[allow(dead_code)]
fn hsv(hue: u8) -> Rgb {
    let h = hue as u16;
    let region = h / 43;
    let remainder = (h - region * 43) * 6;
    let q = (255 - remainder) as u8;
    let t = remainder as u8;
    match region {
        0 => Rgb::new(255, t, 0),  1 => Rgb::new(q, 255, 0),
        2 => Rgb::new(0, 255, t),  3 => Rgb::new(0, q, 255),
        4 => Rgb::new(t, 0, 255),  _ => Rgb::new(255, 0, q),
    }
}

// ── Data header (no type bit — spwm_25 removes dispatch) ─────────────
//   bits  0-14 (15): n_pre - 1      → OUT X, 15
//   bits 15-30 (16): n_lat - 1      → OUT Y, 16
//   bit     31      : 0 (pad)        → OUT NULL, 1

const fn data_header(n_pre: u32, n_lat: u32) -> u32 {
    (n_pre - 1) | ((n_lat - 1) << 15)
}

// ── Scan word (no type bit — spwm_25) ────────────────────────────────
//   bits  0-6  (7): display - 1     → OUT Y, 7
//   bits  7-12 (6): 0x3F (RGB=1)    \ OUT PINS, 11
//   bits 13-17 (5): addr            /
//   bits 18-22 (5): setup - 1       → OUT Y, 5
//   bits 23-26 (4): oe - 1          → OUT Y, 4
//   bits 27-31 (5): 0 (pad)         → OUT NULL, 5

fn scan_word(display: u32, addr: u32, setup: u32, oe: u32) -> u32 {
    (display - 1)              // bits 0-6
    | (0x3F << 7)              // bits 7-12  RGB high
    | (addr << 13)             // bits 13-17 addr pins 6-10
    | ((setup - 1) << 18)      // bits 18-22
    | ((oe - 1) << 23)         // bits 23-26
}

/// Scan word for the main-loop padded-DMA region: uniform W4 OE on
/// every row, so rows 0 and 32 (both driven by scan_line 0) have the
/// same brightness as the rest of the panel in steady state.
fn scan_word_for(scan_line: usize) -> u32 {
    scan_word(DISPLAY_CLK, scan_line as u32, SETUP_CLK, OE_CLK_W4)
}

/// Scan word for the boot sync phase's ring-mode DMA: W12 OE on
/// scan_line 0, W4 elsewhere. The W12 pulse is what actually resets
/// and aligns the 8 driver chips' SRAM write pointers — the only
/// mechanism we've found that bootstraps sync on DP3364S. This is
/// used only during the ~1 s startup sync phase; after that, main
/// loop scans come from the padded region and use W4 uniformly.
fn scan_word_for_sync(scan_line: usize) -> u32 {
    let oe = if scan_line == 0 { OE_CLK_W12 } else { OE_CLK_W4 };
    scan_word(DISPLAY_CLK, scan_line as u32, SETUP_CLK, oe)
}

// ── Offset helpers ──────────────────────────────────────────────────

const fn latch_header_offset(scan_line: usize, channel: usize) -> usize {
    let latch_idx = scan_line * LATCHES_PER_LINE + channel;
    DATA_OFFSET + latch_idx * DATA_LATCH_STRIDE
}

const fn post_scan_offset(cycle: usize, scan_line: usize) -> usize {
    SCAN_OFFSET + cycle * SCAN_LINES + scan_line
}

// ── Frame-buffer packing ─────────────────────────────────────────────

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

/// Write the static portions of a frame buffer: VSYNC / PRE_ACT / WR_CFG
/// headers, DATA_LATCH headers, inline SCAN_WORDs, and the post-scan
/// padding region.
fn init_frame_headers(buf: &mut [u32; FRAME_WORDS]) {
    buf[VSYNC_OFFSET] = data_header(VSYNC_PRE, VSYNC_LAT);
    buf[VSYNC_OFFSET + 1] = 0;

    buf[PRE_ACT_OFFSET] = data_header(PRE_ACT_PRE, PRE_ACT_LAT);
    for i in 0..PRE_ACT_WORDS {
        buf[PRE_ACT_OFFSET + 1 + i] = 0;
    }

    buf[WR_CFG_OFFSET] = data_header(WR_CFG_PRE, WR_CFG_LAT);

    for scan_line in 0..SCAN_LINES {
        for channel in 0..LATCHES_PER_LINE {
            buf[latch_header_offset(scan_line, channel)] =
                data_header(DATA_LATCH_PRE, DATA_LATCH_LAT);
        }
    }

    // Emit W12 on the first scan_line of EVERY cycle (mimicking
    // spwm_4's ring-mode scan buffer which has W12 at scan_line 0 and
    // wraps every 32 scans, giving the chip a W12 per group rather
    // than once per frame). Datasheet §11.5 only prescribes Group 0
    // Line 0, but spwm_4 (echo-free) emits it every group, so test
    // the same.
    for cycle in 0..POST_SCAN_CYCLES {
        for slot in 0..SCAN_LINES {
            let oe = if slot == 0 { OE_CLK_W12 } else { OE_CLK_W4 };
            buf[post_scan_offset(cycle, slot)] =
                scan_word(DISPLAY_CLK, slot as u32, SETUP_CLK, oe);
        }
    }
}

/// Pack-time compensation for the 4-scan_line hardware shift that
/// appears under SYNC_MODE bit 7 = 1 (e.g. reg 0x0C88). Set to 0
/// when bit 7 is clear (0x0C08 or 0x0C48) since the hardware then
/// displays scan_line N at row N with no offset.
const PACK_SHIFT: isize = 0;

/// Pack the PIXELS framebuffer into the given frame buffer's
/// DATA_LATCH regions.
fn pack_pixels(buf: &mut [u32; FRAME_WORDS]) {
    let pixels = unsafe { &*core::ptr::addr_of!(PIXELS) };

    for scan_line in 0..SCAN_LINES {
        let source_line =
            (scan_line as isize + PACK_SHIFT).rem_euclid(SCAN_LINES as isize) as usize;
        let upper_row = source_line;
        let lower_row = source_line + 32;

        for channel in 0..LATCHES_PER_LINE {
            let base = latch_header_offset(scan_line, channel) + 1;
            let output = 15 - channel;

            let mut ur = [0u16; 8];
            let mut ug = [0u16; 8];
            let mut ub = [0u16; 8];
            let mut lr = [0u16; 8];
            let mut lg = [0u16; 8];
            let mut lb = [0u16; 8];
            for chip in 0..8usize {
                let col = chip * 16 + output;
                let u = pixels[upper_row][col];
                let l = pixels[lower_row][col];
                ur[chip] = GAMMA14[u.r as usize];
                ug[chip] = GAMMA14[u.g as usize];
                ub[chip] = GAMMA14[u.b as usize];
                lr[chip] = GAMMA14[l.r as usize];
                lg[chip] = GAMMA14[l.g as usize];
                lb[chip] = GAMMA14[l.b as usize];
            }

            for w in 0..32usize {
                let mut acc: u32 = 0;
                for slot in 0..4usize {
                    let pix = w * 4 + slot;
                    let chip = 7 - (pix >> 4);
                    let bit_pos = 15 - (pix & 15);
                    let shift = slot * 8;
                    acc |= (((ur[chip] >> bit_pos) & 1) as u32) << shift;
                    acc |= (((ug[chip] >> bit_pos) & 1) as u32) << (shift + 1);
                    acc |= (((ub[chip] >> bit_pos) & 1) as u32) << (shift + 2);
                    acc |= (((lr[chip] >> bit_pos) & 1) as u32) << (shift + 3);
                    acc |= (((lg[chip] >> bit_pos) & 1) as u32) << (shift + 4);
                    acc |= (((lb[chip] >> bit_pos) & 1) as u32) << (shift + 5);
                }
                buf[base + w] = acc;
            }
        }
    }
}

fn update_wr_cfg(buf: &mut [u32; FRAME_WORDS], reg_idx: usize) {
    let reg = CONFIG_REGS[reg_idx];
    pack_shift128(
        &mut buf[WR_CFG_OFFSET + 1 .. WR_CFG_OFFSET + 1 + WR_CFG_WORDS],
        reg, ALL_RGB,
    );
}

// ── Fill pixels helper ──────────────────────────────────────────────

/// E25 diagnostic pattern. Four distinct colour-blocks on the two
/// source rows so the echo's spatial and colour structure is
/// readable:
///
/// |            | cols 0-63 | cols 64-127 |
/// | ---------- | --------- | ----------- |
/// | row 31 (upper source) | RED    | BLUE    |
/// | row 63 (lower source) | GREEN  | YELLOW  |
///
/// Read the echo on row 0 (upper target) and row 32 (lower target):
///   - Same colour per half, same column boundary at col 64 →
///     per-column SRAM-address leak.
///   - Colours preserved but column boundary shifted or smeared →
///     serial shift-register leak inside the driver chips.
///   - Wrong colours (e.g. left-half green on row 0) → cross-half
///     bleed or colour-channel-specific mechanism.
///   - Echo strong on some columns, absent on others → column-
///     selective mechanism (investigate which).
fn fill_pixels(_offset: u8) {
    let pixels = unsafe { &mut *core::ptr::addr_of_mut!(PIXELS) };
    for row in 0..64usize {
        for col in 0..128usize {
            pixels[row][col] = if row == 31 {
                if col < 64 { Rgb::new(255, 0, 0) }       // red
                else        { Rgb::new(0, 0, 255) }       // blue
            } else if row == 63 {
                if col < 64 { Rgb::new(0, 255, 0) }       // green
                else        { Rgb::new(255, 255, 0) }     // yellow
            } else {
                Rgb::BLACK
            };
        }
    }
}

/// Diagonal rainbow: `hue = (row + col + offset) mod 256`. Swap in
/// by renaming to `fill_pixels`.
#[allow(dead_code)]
fn fill_pixels_rainbow(offset: u8) {
    let pixels = unsafe { &mut *core::ptr::addr_of_mut!(PIXELS) };
    for row in 0..64usize {
        for col in 0..128usize {
            let hue = (row as u16 + col as u16 + offset as u16) as u8;
            pixels[row][col] = hsv(hue);
        }
    }
}

// ── SIO FIFO raw register access ────────────────────────────────────

fn fifo_write(val: u32) {
    unsafe {
        while (0xD000_0050 as *const u32).read_volatile() & 2 == 0 {}
        (0xD000_0054 as *mut u32).write_volatile(val);
        cortex_m::asm::sev();
    }
}

fn fifo_try_read() -> Option<u32> {
    unsafe {
        if (0xD000_0050 as *const u32).read_volatile() & 1 != 0 {
            Some((0xD000_0058 as *const u32).read_volatile())
        } else {
            None
        }
    }
}

fn fifo_read() -> u32 {
    unsafe {
        loop {
            if (0xD000_0050 as *const u32).read_volatile() & 1 != 0 {
                return (0xD000_0058 as *const u32).read_volatile();
            }
            cortex_m::asm::wfe();
        }
    }
}

// ── Core 1 task ─────────────────────────────────────────────────────

fn core1_task() {
    loop {
        let msg = fifo_read();
        let offset = (msg & 0xFF) as u8;
        let buf_idx = ((msg >> 8) & 1) as u8;

        let buf = if buf_idx == 0 {
            unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) }
        } else {
            unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_B) }
        };

        fill_pixels(offset);
        pack_pixels(buf);
        // Stage D1 (revised): don't rotate WR_CFG content per frame.
        // Flush leaves both buffers with a constant known reg value
        // (whatever the last flush iteration wrote). If the 4-line
        // offset changes, rotation content is implicated. If it
        // persists, the WR_CFG command itself is responsible.

        fifo_write(1);
    }
}

// ── PIO helpers ─────────────────────────────────────────────────────

const PIO0_BASE: u32 = 0x5020_0000;
const PIO0_CTRL_SET: u32 = PIO0_BASE + 0x2000;
const PIO0_CTRL_CLR: u32 = PIO0_BASE + 0x3000;
const PIO0_FDEBUG: u32 = PIO0_BASE + 0x008;
const SM0_CLKDIV: u32 = PIO0_BASE + 0x0C8;
const SM0_EXECCTRL: u32 = PIO0_BASE + 0x0CC;
const SM0_INSTR: u32 = PIO0_BASE + 0x0D8;
const SM0_PINCTRL: u32 = PIO0_BASE + 0x0DC;
const SM0_EN: u32 = 1 << 0;
const SM0_TXSTALL: u32 = 1 << 24;

fn enable_sm(mask: u32) {
    unsafe { (PIO0_CTRL_SET as *mut u32).write_volatile(mask) };
}
fn disable_sm(mask: u32) {
    unsafe { (PIO0_CTRL_CLR as *mut u32).write_volatile(mask) };
}

/// Force SM0 to JMP to the given absolute PC via SMx_INSTR.
/// Instruction encoding: JMP <addr> = 0b000_00000_000_AAAAA (low 5 bits).
/// Call while SM is stalled (or freshly disabled+stalled); the write injects
/// the instruction as the next to execute.
fn force_jmp_sm0(target_pc: u32) {
    unsafe { (SM0_INSTR as *mut u32).write_volatile(target_pc & 0x1F) };
}

// PIO SET instruction encodings (for force_set_pindirs).
//   E080 = SET pindirs, 0   (all released SET pins → input)
//   E083 = SET pindirs, 3   (2 lowest SET pins → output)
const SET_PINDIRS_0: u32 = 0xE080;
const SET_PINDIRS_3: u32 = 0xE083;

// PINCTRL bit-fields (SET base/count) we temporarily hijack.
const PINCTRL_SET_COUNT_MASK: u32 = 0x7 << 26;
const PINCTRL_SET_BASE_MASK: u32 = 0x1F << 5;

fn set_clkdiv(int_div: u32) {
    unsafe { (SM0_CLKDIV as *mut u32).write_volatile(int_div << 16) };
}

fn wait_txstall(sm_stall_bit: u32) {
    unsafe {
        (PIO0_FDEBUG as *mut u32).write_volatile(sm_stall_bit);
        while (PIO0_FDEBUG as *const u32).read_volatile() & sm_stall_bit == 0 {
            cortex_m::asm::nop();
        }
    }
}

/// Briefly reassign SM0's SET base/count to the given pins, force-
/// execute one PIO instruction, then restore PINCTRL. Ported from
/// spwm_4 — safe to call while SM0 is stalled on a TX wait.
fn force_set_pindirs(base: u32, count: u32, instr: u32) {
    unsafe {
        let saved = (SM0_PINCTRL as *const u32).read_volatile();
        let modified = (saved & !(PINCTRL_SET_COUNT_MASK | PINCTRL_SET_BASE_MASK))
            | (count << 26) | (base << 5);
        (SM0_PINCTRL as *mut u32).write_volatile(modified);
        (SM0_INSTR as *mut u32).write_volatile(instr);
        (SM0_PINCTRL as *mut u32).write_volatile(saved);
    }
}

// ── DMA ring-mode scan (ch1, raw register access, sync-phase only) ──
const DMA_BASE: u32 = 0x5000_0000;
const CH1_READ_ADDR:  *mut u32 = (DMA_BASE + 0x040) as *mut u32;
const CH1_WRITE_ADDR: *mut u32 = (DMA_BASE + 0x044) as *mut u32;
const CH1_AL1_CTRL:   *mut u32 = (DMA_BASE + 0x050) as *mut u32;
const CH1_AL1_TRANS_COUNT_TRIG: *mut u32 = (DMA_BASE + 0x05C) as *mut u32;
const PIO0_TXF0: u32 = PIO0_BASE + 0x010;

const CH1_CTRL_VAL: u32 =
    (1 << 0)
    | (2 << 2)
    | (1 << 4)
    | (7 << 8)
    | (1 << 13)
    | (0 << 17);

fn start_ring_scan() {
    let scan_addr = core::ptr::addr_of!(SCAN_BUF) as u32;
    unsafe {
        CH1_AL1_CTRL.write_volatile(CH1_CTRL_VAL);
        CH1_READ_ADDR.write_volatile(scan_addr);
        CH1_WRITE_ADDR.write_volatile(PIO0_TXF0);
        CH1_AL1_TRANS_COUNT_TRIG.write_volatile((15 << 28) | 32);
    }
}

fn stop_ring_scan() {
    unsafe {
        const CHAN_ABORT: *mut u32 = (DMA_BASE + 0x464) as *mut u32;
        CHAN_ABORT.write_volatile(1 << 1);
        while CHAN_ABORT.read_volatile() & (1 << 1) != 0 {
            cortex_m::asm::nop();
        }
    }
}

fn build_scan_buf() {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(SCAN_BUF) };
    for row in 0..SCAN_LINES {
        buf.0[row] = scan_word_for_sync(row);
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
    let mut sio_hal = hal::Sio::new(pac.SIO);
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
    let sio_regs = unsafe { &(*hal::pac::SIO::ptr()) };
    sio_regs.gpio_out_clr().write(|w| unsafe {
        w.bits(ALL_RGB | (0x1F << 6) | (1 << 11) | (1 << 12) | (1 << 13))
    });
    const IO_BANK0_BASE: u32 = 0x4002_8000;
    for p in 0..14u32 {
        let addr = IO_BANK0_BASE + 0x04 + p * 8;
        unsafe { (addr as *mut u32).write_volatile(6) };
    }

    // Pull-down on LAT (GPIO 12) for safety during startup
    const PADS_BANK0: u32 = 0x4003_8000;
    const GPIO12_PAD: u32 = PADS_BANK0 + 0x04 + 12 * 4;
    unsafe {
        let val = (GPIO12_PAD as *const u32).read_volatile();
        (GPIO12_PAD as *mut u32).write_volatile(val | (1 << 2));
    }

    // ── 3. Install unified PIO program (unchanged from spwm_6) ──────
    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // spwm_25: 1-bit side-set (CLK only). LAT via SET instructions.
    // No dispatch prelude — two entry points reached via forced JMP.
    let ss = pio::SideSet::new(false, 1, false);
    let mut a = pio::Assembler::new_with_side_set(ss);

    let mut data_cmd         = a.label();
    let mut scan_entry       = a.label();
    let mut display_lp       = a.label();
    let mut setup_lp         = a.label();
    let mut oe_lp            = a.label();
    let mut pre_loop         = a.label();
    let mut lat_loop         = a.label();
    let mut data_wrap_source = a.label();
    let mut scan_wrap_source = a.label();

    // Side-set: 0 = CLK low, 1 = CLK high.
    // SET PINS (base 12, count 2): bit0→LAT, bit1→OE.
    //   SET PINS 0b00 : LAT=0, OE=0
    //   SET PINS 0b01 : LAT=1, OE=0
    //   SET PINS 0b10 : LAT=0, OE=1

    // ─── DATA path (PC 0, wrap_target) ───────────────────────────────
    a.bind(&mut data_cmd);
    a.out_with_side_set(pio::OutDestination::X, 15, 0);    // n_pre-1
    a.out_with_side_set(pio::OutDestination::Y, 16, 0);    // n_lat-1
    a.out_with_side_set(pio::OutDestination::NULL, 1, 0);  // pad bit 31

    a.bind(&mut pre_loop);
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0);
    a.out_with_side_set(pio::OutDestination::NULL, 2, 1);  // CLK rise
    a.jmp_with_side_set(pio::JmpCondition::XDecNonZero, &mut pre_loop, 0);

    // Raise LAT for lat_loop
    a.set_with_side_set(pio::SetDestination::PINS, 0b01, 0);

    a.bind(&mut lat_loop);
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0);
    a.out_with_side_set(pio::OutDestination::NULL, 2, 1);  // CLK rise
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut lat_loop, 0);

    // Drop LAT; data wrap fires AFTER this instruction (back to data_cmd)
    a.set_with_side_set(pio::SetDestination::PINS, 0b00, 0);
    a.bind(&mut data_wrap_source);

    // ─── SCAN path (reached only via forced JMP scan_entry) ──────────
    a.bind(&mut scan_entry);
    a.out_with_side_set(pio::OutDestination::Y, 7, 0);
    a.bind(&mut display_lp);
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut display_lp, 1);

    a.out_with_side_set(pio::OutDestination::PINS, 11, 0);
    a.out_with_side_set(pio::OutDestination::Y, 5, 0);

    a.bind(&mut setup_lp);
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut setup_lp, 1);

    a.out_with_side_set(pio::OutDestination::Y, 4, 0);
    // spwm_28: OE on via scan_pinctrl SET_BASE=13, SET_COUNT=1 → bit 0 = pin 13.
    // Was 0b10 under data's SET_BASE=12, COUNT=2 (bit 1 = pin 13).
    a.set_with_side_set(pio::SetDestination::PINS, 0b01, 0);  // OE on

    a.bind(&mut oe_lp);
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut oe_lp, 1);

    a.set_with_side_set(pio::SetDestination::PINS, 0b00, 0);  // OE off
    a.out_with_side_set(pio::OutDestination::NULL, 5, 0);     // drain
    // scan_wrap_source: scan-phase wrap fires AFTER this instruction
    // back to scan_entry. No explicit JMP — relies on PIO wrap.
    a.bind(&mut scan_wrap_source);

    // Assemble with data's wrap. Scan's wrap is applied at runtime
    // by overwriting EXECCTRL WRAP_TOP/BOTTOM before scan phase.
    let mut prog = a.assemble_with_wrap(data_wrap_source, data_cmd);
    prog = prog.set_origin(Some(0));
    let installed = pio0.install(&prog).unwrap();

    // Label positions in the program (must stay in sync with assembly).
    // NOTE: pio-0.2.1's assemble_with_wrap does `source -= 1` internally
    // (pio-0.2.1/src/lib.rs:574) because `bind` labels the NEXT
    // instruction slot. So the hardware WRAP_TOP is the label value - 1.
    // We store WRAP_TOP directly here (what EXECCTRL bits 16:12 want).
    //   data_cmd          = 0   (entry; WRAP_BOTTOM for data)
    //   data_wrap_source  = 11  (label), WRAP_TOP = 10 (last data instr)
    //   scan_entry        = 11  (entry; WRAP_BOTTOM for scan)
    //   scan_wrap_source  = 27  (label), WRAP_TOP = 26 (last scan instr)
    const DATA_CMD_LOCAL: u32 = 0;
    const DATA_WRAP_TOP_LOCAL: u32 = 10;
    const SCAN_ENTRY_LOCAL: u32 = 11;
    const SCAN_WRAP_TOP_LOCAL: u32 = 26;

    let prog_offset = installed.offset() as u32;
    let data_cmd_pc = prog_offset + DATA_CMD_LOCAL;
    let scan_entry_pc = prog_offset + SCAN_ENTRY_LOCAL;
    defmt::info!(
        "spwm_29 PIO: data_cmd={=u32} scan_entry={=u32}  data_wrap_top={=u32}  scan_wrap_top={=u32}",
        data_cmd_pc, scan_entry_pc,
        prog_offset + DATA_WRAP_TOP_LOCAL,
        prog_offset + SCAN_WRAP_TOP_LOCAL,
    );

    let (mut sm, _, tx) =
        hal::pio::PIOBuilder::from_installed_program(installed)
            .buffers(hal::pio::Buffers::OnlyTx)
            .out_pins(0, 11)
            .set_pins(12, 2)  // spwm_24: SET covers LAT (12) + OE (13)
            .side_set_pin_base(11)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .autopull(true)
            .pull_threshold(32)
            .clock_divisor_fixed_point(3, 0)
            .build(sm0);

    sm.set_pindirs([
        (0,  hal::pio::PinDir::Output), (1,  hal::pio::PinDir::Output),
        (2,  hal::pio::PinDir::Output), (3,  hal::pio::PinDir::Output),
        (4,  hal::pio::PinDir::Output), (5,  hal::pio::PinDir::Output),
        (6,  hal::pio::PinDir::Output), (7,  hal::pio::PinDir::Output),
        (8,  hal::pio::PinDir::Output), (9,  hal::pio::PinDir::Output),
        (10, hal::pio::PinDir::Output), (11, hal::pio::PinDir::Output),
        (12, hal::pio::PinDir::Output), (13, hal::pio::PinDir::Output),
    ]);
    let _sm = sm.start();
    let mut tx = tx;

    // ── 3b. Compute EXECCTRL swap values ─────────────────────────────
    // Read the EXECCTRL configured by PIOBuilder (data's wrap from
    // assemble_with_wrap). Scan's EXECCTRL is the same except for
    // WRAP_TOP/WRAP_BOTTOM.
    let base_execctrl = unsafe { (SM0_EXECCTRL as *const u32).read_volatile() };
    const EXECCTRL_WRAP_MASK: u32 = (0x1F << 12) | (0x1F << 7);
    let execctrl_no_wrap = base_execctrl & !EXECCTRL_WRAP_MASK;
    let data_execctrl = execctrl_no_wrap
        | (((prog_offset + DATA_WRAP_TOP_LOCAL) & 0x1F) << 12)
        | (((prog_offset + DATA_CMD_LOCAL) & 0x1F) << 7);
    let scan_execctrl = execctrl_no_wrap
        | (((prog_offset + SCAN_WRAP_TOP_LOCAL) & 0x1F) << 12)
        | (((prog_offset + SCAN_ENTRY_LOCAL) & 0x1F) << 7);

    // Sanity check: data_execctrl we just computed should equal
    // base_execctrl (what PIOBuilder wrote from assemble_with_wrap).
    // If these diverge, the WRAP_TOP constant is off.
    defmt::info!(
        "spwm_29 EXECCTRL: base=0x{=u32:08x} data=0x{=u32:08x} scan=0x{=u32:08x}",
        base_execctrl, data_execctrl, scan_execctrl,
    );

    // ── 3c. Compute PINCTRL swap values ──────────────────────────────
    // PINCTRL bit layout (same RP2040 & RP2350):
    //   [31:29] SIDESET_COUNT   [28:26] SET_COUNT
    //   [25:20] OUT_COUNT       [19:15] IN_BASE
    //   [14:10] SIDESET_BASE    [9:5]   SET_BASE
    //   [4:0]   OUT_BASE
    //
    // spwm_29: OUT_COUNT held at 11 for BOTH phases. Only SET_BASE
    // and SET_COUNT differ across the boundary.
    //
    // Data phase: OUT_BASE=0, OUT_COUNT=11, SIDESET=11/1, SET=12/2.
    // Scan phase: OUT_BASE=0, OUT_COUNT=11, SIDESET=11/1, SET=13/1.
    let data_pinctrl: u32 =
        (1 << 29) | (2 << 26) | (11 << 20) | (0 << 15)
        | (11 << 10) | (12 << 5) | 0;
    let scan_pinctrl: u32 =
        (1 << 29) | (1 << 26) | (11 << 20) | (0 << 15)
        | (11 << 10) | (13 << 5) | 0;

    // Sanity: read PINCTRL PIOBuilder configured (out 0..11, set 12..2,
    // sideset 11/1). Log all three so we can diff on-device.
    let base_pinctrl = unsafe { (SM0_PINCTRL as *const u32).read_volatile() };
    defmt::info!(
        "spwm_29 PINCTRL:  base=0x{=u32:08x} data=0x{=u32:08x} scan=0x{=u32:08x}",
        base_pinctrl, data_pinctrl, scan_pinctrl,
    );

    // Phase-swap closures — write PINCTRL + EXECCTRL + force JMP.
    // PINCTRL first so the subsequent forced JMP executes under the
    // new pin config. Order EXECCTRL then INSTR matches spwm_17's
    // swap_sm0_program order.
    let swap_to_data = || unsafe {
        (SM0_PINCTRL  as *mut u32).write_volatile(data_pinctrl);
        (SM0_EXECCTRL as *mut u32).write_volatile(data_execctrl);
        (SM0_INSTR    as *mut u32).write_volatile(data_cmd_pc & 0x1F);
    };
    let swap_to_scan = || unsafe {
        (SM0_PINCTRL  as *mut u32).write_volatile(scan_pinctrl);
        (SM0_EXECCTRL as *mut u32).write_volatile(scan_execctrl);
        (SM0_INSTR    as *mut u32).write_volatile(scan_entry_pc & 0x1F);
    };

    // ── 4. DMA — single channel, continuous frame loop ──────────────
    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut ch = dma.ch0;

    defmt::info!("DP3364S S-PWM (padded-frame, continuous DMA) starting");
    defmt::info!("spwm_29: SET-only PINCTRL swap (OUT_COUNT=11 in both phases)");

    // ── 5. Init frame headers (DATA_LATCH payloads stay at zero) ────
    init_frame_headers(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) });
    init_frame_headers(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_B) });

    // ── 6. Startup flush — matches spwm_6's proven two-phase pattern:
    //       data-only DMA at clkdiv 2, then one scan pass at clkdiv 3,
    //       with wait_txstall between. This sequence is what actually
    //       syncs the 8 driver chips' SRAM write pointers to zero (we
    //       established empirically that spwm_6 does this and the sync
    //       persists across a program switch as long as panel power
    //       stays on).
    //
    //       First frame writes GROUP_SET; subsequent ones cycle through
    //       the remaining config regs. Extra iterations beyond the 13
    //       regs ensure SRAM is fully overwritten with zeros.
    const FLUSH_FRAMES: u32 = 26;
    for flush in 0..FLUSH_FRAMES {
        if (flush as usize) < CONFIG_REGS.len() {
            update_wr_cfg(
                unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) },
                flush as usize,
            );
        }

        // --- Data-only DMA (HEADER + 512 DATA_LATCHes), at clkdiv 2 ---
        set_clkdiv(2);
        swap_to_data();
        if flush == 0 { defmt::info!("flush iter 0 — after swap_to_data"); }
        let data_slice = unsafe {
            core::slice::from_raw_parts(
                core::ptr::addr_of!(FRAME_BUF_A) as *const u32,
                DATA_END,
            )
        };
        let cfg = single_buffer::Config::new(ch, data_slice, tx);
        let xfer = cfg.start();
        let (new_ch, _, new_tx) = xfer.wait();
        ch = new_ch;
        tx = new_tx;
        if flush == 0 { defmt::info!("flush iter 0 — data DMA done"); }
        wait_txstall(SM0_TXSTALL);
        if flush == 0 { defmt::info!("flush iter 0 — data wait_txstall done"); }

        // --- One scan pass (32 scan words), at clkdiv 3 ---
        set_clkdiv(3);
        swap_to_scan();
        if flush == 0 { defmt::info!("flush iter 0 — after swap_to_scan"); }
        let scan_slice = unsafe {
            let base = (core::ptr::addr_of!(FRAME_BUF_A) as *const u32).add(SCAN_OFFSET);
            core::slice::from_raw_parts(base, SCAN_LINES)
        };
        let scan_cfg = single_buffer::Config::new(ch, scan_slice, tx);
        let scan_xfer = scan_cfg.start();
        let (new_ch2, _, new_tx2) = scan_xfer.wait();
        ch = new_ch2;
        tx = new_tx2;
        if flush == 0 { defmt::info!("flush iter 0 — scan DMA done"); }
        wait_txstall(SM0_TXSTALL);
        if flush == 0 { defmt::info!("flush iter 0 — scan wait_txstall done"); }
    }

    defmt::info!("flush complete");

    // ── 6b. Sync phase — match spwm_6's main loop pattern for ~1 s.
    //        Empirically this is what actually locks the 8 driver chips'
    //        SRAM write pointers to a common offset. Once synced, state
    //        persists into the padded-continuous main loop.
    //
    //        Pattern: ring-mode scan (ch1) running continuously, with
    //        brief one-shot data DMAs (ch0) interrupting it. Each cycle
    //        matches spwm_6 exactly: stop-ring → data → restart-ring.
    build_scan_buf();
    // Keep ch1 bound (don't drop immediately) so HAL doesn't reset it.
    // We access ch1 via raw registers for ring-mode; the HAL type just
    // needs to not interfere.
    let _ch1 = dma.ch1;

    set_clkdiv(3);
    swap_to_scan();
    start_ring_scan();

    const SYNC_FRAMES: u32 = 60; // ~1 s at 60 Hz
    for _ in 0..SYNC_FRAMES {
        // Crude delay to let ring-mode run for ~one core-1 pack period
        cortex_m::asm::delay(150_000 * 10); // ~10 ms at 150 MHz

        stop_ring_scan();
        wait_txstall(SM0_TXSTALL);

        set_clkdiv(2);
        swap_to_data();
        let data_slice = unsafe {
            core::slice::from_raw_parts(
                core::ptr::addr_of!(FRAME_BUF_A) as *const u32,
                DATA_END,
            )
        };
        let cfg = single_buffer::Config::new(ch, data_slice, tx);
        let xfer = cfg.start();
        let (new_ch, _, new_tx) = xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);
        set_clkdiv(3);
        swap_to_scan();

        start_ring_scan();
    }

    stop_ring_scan();
    wait_txstall(SM0_TXSTALL);
    defmt::info!("sync phase complete");

    // Stage D1 (revised): seed FRAME_BUF_B's WR_CFG payload so both
    // buffers emit identical constant WR_CFG content in the main loop.
    // Last flushed reg is CONFIG_REGS[12] = 0x0D12.
    update_wr_cfg(
        unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_B) },
        CONFIG_REGS.len() - 1,
    );

    // ── 7. Pack first real frame and spawn core 1 ───────────────────
    fill_pixels(0);
    pack_pixels(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) });

    let mut mc = hal::multicore::Multicore::new(
        &mut pac.PSM, &mut pac.PPB, &mut sio_hal.fifo,
    );
    let cores = mc.cores();
    cores[1].spawn(unsafe { CORE1_STACK.take().unwrap() }, core1_task).unwrap();

    let mut active_buf: u8 = 0;
    let mut core1_buf: u8 = 1;
    let mut offset: u8 = 0;

    const TIMER0_BASE: u32 = 0x400B_0000;
    const TIMELR: *const u32 = (TIMER0_BASE + 0x0C) as *const u32;
    fn timer_us() -> u32 {
        unsafe { TIMELR.read_volatile() }
    }
    let mut frame_count: u32 = 0;
    let mut last_report = timer_us();

    // Kick off core 1 with the first pack job
    offset = offset.wrapping_add(2);
    fifo_write(offset as u32 | ((core1_buf as u32) << 8));

    // Split-DMA loop (E19). Each frame goes out as two DMAs with a
    // wait_txstall between:
    //   DMA 1: header + DATA_LATCHes   [0 .. DATA_END]
    //   gap   (pipeline reset)
    //   DMA 2: post-scan scan words    [SCAN_OFFSET .. FRAME_WORDS]
    loop {
        let buf_ptr = if active_buf == 0 {
            core::ptr::addr_of!(FRAME_BUF_A) as *const u32
        } else {
            core::ptr::addr_of!(FRAME_BUF_B) as *const u32
        };

        // DMA 1: header + DATA_LATCHes. Match spwm_4's data-phase
        // clkdiv of 2 (75 MHz PIO, ~37.5 MHz DCLK).
        set_clkdiv(2);
        enable_sm(SM0_EN);
        swap_to_data();
        let data_slice = unsafe { core::slice::from_raw_parts(buf_ptr, DATA_END) };
        let cfg = single_buffer::Config::new(ch, data_slice, tx);
        let xfer = cfg.start();
        let (new_ch, _, new_tx) = xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);
        disable_sm(SM0_EN);

        if INTER_DMA_DELAY_CYCLES > 0 {
            cortex_m::asm::delay(INTER_DMA_DELAY_CYCLES);
        }

        if INTER_DMA_HIGHZ_CYCLES > 0 {
            // Release CLK/LAT (pins 11-12) → high-Z, delay, reclaim.
            force_set_pindirs(11, 2, SET_PINDIRS_0);
            cortex_m::asm::delay(INTER_DMA_HIGHZ_CYCLES);
            force_set_pindirs(11, 2, SET_PINDIRS_3);
        }

        if INTER_DMA_INJECT >= 1 {
            // Push VSYNC (2 words) directly to PIO TX FIFO. After the
            // wait_txstall above, FIFO is empty; `OnlyTx` mode gives
            // 8-word depth, so up to 7 words fit without overrun.
            unsafe {
                let txf = PIO0_TXF0 as *mut u32;
                txf.write_volatile(data_header(VSYNC_PRE, VSYNC_LAT));
                txf.write_volatile(0);
                if INTER_DMA_INJECT >= 2 {
                    // + PRE_ACT (1 header + 4 payload = 5 words).
                    txf.write_volatile(data_header(PRE_ACT_PRE, PRE_ACT_LAT));
                    for _ in 0..PRE_ACT_WORDS {
                        txf.write_volatile(0);
                    }
                }
            }
            wait_txstall(SM0_TXSTALL);
        }

        // DMA 2: post-scan region, emitted as POST_SCAN_CYCLES back-
        // to-back 32-word DMAs (mimicking spwm_14's per-group DMA
        // pattern which has a small CPU gap between every group).
        set_clkdiv(3);
        enable_sm(SM0_EN);
        swap_to_scan();
        for cycle in 0..POST_SCAN_CYCLES {
            let scan_slice = unsafe {
                core::slice::from_raw_parts(
                    buf_ptr.add(SCAN_OFFSET + cycle * SCAN_LINES),
                    SCAN_LINES,
                )
            };
            let cfg = single_buffer::Config::new(ch, scan_slice, tx);
            let xfer = cfg.start();
            let (new_ch, _, new_tx) = xfer.wait();
            ch = new_ch;
            tx = new_tx;
            wait_txstall(SM0_TXSTALL);
        }
        disable_sm(SM0_EN);

        if fifo_try_read().is_some() {
            frame_count += 1;
            active_buf = core1_buf;
            core1_buf = 1 - active_buf;
            offset = offset.wrapping_add(2);
            fifo_write(offset as u32 | ((core1_buf as u32) << 8));

            if frame_count & 0xFF == 0 {
                let now = timer_us();
                let elapsed_us = now.wrapping_sub(last_report);
                if elapsed_us > 0 {
                    let fps_x10 = 2_560_000_000u32 / elapsed_us;
                    defmt::info!("FPS: {}.{}", fps_x10 / 10, fps_x10 % 10);
                }
                last_report = now;
            }
        }
    }
}
