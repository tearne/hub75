# HUB75 investigation — forward plan

*Updated 2026-04-24 after Steps 1–4 completed.*

Priorities (in order): **flicker > brightness > frames**. Additional
dimensions: **CPU offload**, **code simplicity**.

---

## Current baseline (`spwm_33_autonomous.rs`)

Echo-free, flicker assessed as good on a full-panel rainbow, 128 Hz
content update, core 0 ~99 % free and core 1 ~38 % free for
application work.

| Metric              | Value          |
|---------------------|----------------|
| Frame iteration     |  7 830 µs      |
| Data phase          |  2 662 µs      |
| Scan phase          |  5 168 µs      |
| Dark ratio          | 339 ‰ (33.9 %) |
| Display refresh     | 127.69 Hz      |
| Content update      | 127.19 Hz      |
| Per-line refresh    | 2 540 Hz       |
| Core 0 `wfi` time   | ~99 %          |
| Core 1 pack time    | ~4.9 ms/frame  |
| Core 1 free         | ~38 %          |

### What's been settled

- **Echo:** root cause is `PINCTRL.OUT_COUNT` zero-filling ADDR pins
  during data phase; fix is OUT_COUNT swap (6↔11) at phase boundary.
  See `HUB75_DP3364S_RP2350_NOTES.md` §4 and §5.1.
- **Flicker:** acceptable on full-panel rainbow. An A/B test between
  `POST_SCAN_CYCLES = 50` (64 Hz / 17 % dark) and `= 20` (128 Hz /
  34 % dark) was perceptually indistinguishable. **Current setting:
  20 cycles**, chosen for refresh-rate headroom. Parameter kept as a
  runtime variable so the final API can expose it as tunable.
- **Dark gap:** 2.66 ms is PIO-data-transfer-bound. Can't shrink via
  faster CLK (DP3364S spec is 25 MHz max; we're at spec max). The
  2-cycle-pre_loop idea is infeasible.
- **Autonomy:** core 0 is now ~99.5 % in `wfi`. Display subsystem uses
  ~0.5 % of core 0.
- **Content rate:** now matches the display rate. Pack rewrite
  (chip-major loop, nibble-scatter LUT) brought pack time from
  ~21 ms (naïve) to ~4.9 ms. Verified byte-identical to the naïve
  implementation via a boot-time stress-pattern diff.

---

## Next directions

Pick based on what actually bothers you when you look at the panel. If
nothing bothers you, option 6 is a perfectly good place to stop.

### 1. Brightness

Chip current is already maxed (reg 0x08 = 0xFF). Remaining levers:

- **Pack-side:** revisit `GAMMA14` for perceptual linearity. Shift more
  PWM weight to the bright end of the curve if low values look crushed.
- **Chip-side:** reg 0x0F is partially documented for current gain.
  Unlikely to add much if 0x08 is already max, but cheap to sweep.
- **OE timing:** `OE_CLK_W4` is the per-scan OE-on duration. Longer
  OE = brighter but less PWM resolution. Sweep to find optimum.

Expected value: modest, conditional on whether the panel looks visually
dim as-is. Effort: low.

### 2. Colour depth

Current: 6 bits per channel via `GAMMA14`-expanded 14-bit PWM weighting.
~262 k colours. Options:

- Increase `LATCHES_PER_LINE` from 16 to, say, 24 — more PWM bit planes,
  smoother gradients. Linear growth in data-phase time (2.66 ms → ~4 ms
  for +8 latches), so dark-ratio rises and flicker may regress.
- Revisit gamma curve for perceptual linearity (see option 1).
- Dithering in pack (Floyd–Steinberg or Bayer) — fakes more colour depth
  without changing the data format. Pack-algorithm work only.

Expected value: quality improvement for gradient-heavy content. Effort:
medium to high. Watch flicker budget.

### 3. Content rate (32 Hz → higher)

For smoother animation. Requires optimising `pack_pixels` on core 1
(currently ~30 ms). Options:

- **Bit-plane-parallel rewrite:** process multiple scan-lines per outer
  iteration, sharing GAMMA14 lookups and chip/slot computations.
- **Expanded LUTs:** precompute per-channel, per-bit-plane masks.
- **Precomputed chip/slot/bit-pos tables:** drop the nested for-loop
  indexing arithmetic.

Goal: get core 1 pack to ~15 ms → 64 Hz content rate (matching display).

Expected value: noticeable on motion / animation. Effort: medium. No
display-side risk.

### 4. Core 1 offload

Same work as (3), framed as "free up core 1 for application work"
rather than "go faster". If core 1 pack is 15 ms, it's idle half the
time → ~50 % of core 1 free. Combined with spwm_33's freed core 0,
that's 1.5 of 2 cores available for application work.

Expected value: high for real-world applications where both cores are
wanted. Effort: same as (3).

### 5. Build something on top

The driver subsystem is done. Point it at a real workload: scrolling
text, animations, image loading from flash, USB-driven pixel pushing,
sensor-reactive effects, etc. Uses the freed core 0 for application
logic.

Expected value: depends on what you want to build. Effort: project-
dependent.

### 6. Stop

spwm_33 is a clean, documented baseline. `HUB75_DP3364S_RP2350_NOTES.md`,
`ROADMAP.md`, and the auto-memory all reflect current state. Picking
the project up weeks or months later should require only re-reading
those three sources.

Expected value: banks the wins, frees mental overhead. Zero effort.

---

## Parking lot (not in plan)

- **Chip-register sweeps** (reg 0x0E, 0x01, 0x0C[7:6]) — low expected
  impact on priority axes. Interesting for chip-behaviour understanding
  only.
- **Logic analyzer capture** — requires hardware we don't have.
- **Faster PIO clock** (clkdiv < 2) — unusable (`pio_fractional_clkdiv`
  memory; "can't clock faster" in git log; DP3364S CLK spec max
  reached).
- **Parallel pack across both cores** — rejected; ties up both cores
  for display, which conflicts with real-application needs.
