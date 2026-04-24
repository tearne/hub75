# HUB75 + DP3364S + RP2350 — Implementation Notes

*A distilled reference for anyone writing a driver for this specific combination of panel, column-driver chip, and microcontroller. Assembled from a month of experimentation; intended to save the next person days to weeks. Last updated 2026-04-24.*

The working reference implementation is `learning-examples/examples/spwm_3_autonomous.rs`.

---

## 1. Hardware

- **Board:** Pimoroni Interstate 75 W (RP2350A, dual-core Cortex-M33 at 150 MHz, 520 KB SRAM).
- **Panel:** 64 × 128 HUB75 RGB matrix, 1:32 scan ratio.
- **Column-driver chip:** 8 × DP3364S in series. Each chip drives 16 columns; 8 × 16 = 128 columns per scan-line. 32 upper + 32 lower scan-lines are selected by a 5-bit ADDR bus.

### GPIO pin mapping (this driver)

| GPIO | Signal | Notes |
|-----:|--------|-------|
|  0..5 | R0 G0 B0 R1 G1 B1 | Upper & lower RGB |
|  6..10 | ADDR[0..4] | Scan-line address |
|  11 | CLK | Pixel shift clock; PIO side-set controls this |
|  12 | LAT | Latch (active high) |
|  13 | OE | Output enable (active low) |

PIO OUT group covers pins 0..10 (11 pins). PIO SET group covers pins 12..13 (LAT + OE). PIO side-set covers pin 11 (CLK).

---

## 2. DP3364S chip essentials

### Register set

15 valid addresses (0x01..0x0F). This driver writes 13:

```rust
const CONFIG_REGS: [u16; 13] = [
    0x037F,  // GROUP_SET: 128-wide SRAM (boot default is 0x3F = 64-wide!)
    0x1100,  // unused-as-such; reg 0x11 ≠ 0x01
    0x021F,  //
    0x043F,  //
    0x0504,  //
    0x0642,  //
    0x0700,  //
    0x08BF,  // current (high byte); 0xFF is maximum
    0x0960,  //
    0x0ABE,  //
    0x0B8B,  //
    0x0C88,  // SYNC_MODE — bits 7:6 select PWM sync mode
    0x0D12,  //
];
```

**GROUP_SET (0x03) MUST BE FIRST** in the write sequence. The chip boots with 0x3F (64-wide). If you send DATA_LATCHes before fixing this, half the SRAM is missed and the panel shows random garbage.

### Reg 0x0C — SYNC_MODE

Bits 7:6 of reg 0x0C pick the PWM sync mode. We use 0b10 (0x0C88). The datasheet documents 0b00 "General Frame Sync" in detail; 0b10 works in practice but is less-documented territory.

Bit 7 of 0x0C controls a 4-scan-line display offset; when set, the hardware displays scan-line N at row N+4 (mod 32). `PACK_SHIFT` in the code compensates when it's set. Leave bit 7 clear (0x0C08 or 0x0C48) and keep `PACK_SHIFT = 0` unless you have reason.

### Registers deliberately skipped

- **0x01, 0x0E** — never written. Undocumented or reserved. Briefly considered worth probing to see if they affect the echo artifact; they don't.
- **0x0F** — partially documented (current-gain formula); 0x08 at 0xFF already maxes current, so 0x0F doesn't buy more.

### Maximum pixel clock

**25 MHz.** Datasheet maximum FCLK. We run exactly at spec max (PIO at 75 MHz with a 3-cycle pre-loop → CLK period 40 ns → 25 MHz). Attempts to exceed this fail: the chip's shift register drops bits and the display garbles. This ceiling is the single biggest constraint on how fast you can push data through.

### SRAM write-pointer sync

The DP3364S has an internal SRAM write pointer that must be aligned to a known offset across all 8 chips before the panel displays correctly. The only mechanism that reliably achieves this at cold boot is approximately:

1. Flush loop: ~26 iterations each sending one data-DMA (writing a single CONFIG_REGS entry via WR_CFG) followed by one scan-DMA. Early iterations set GROUP_SET then cycle through the rest of CONFIG_REGS; extra iterations overwrite the SRAM with zeros.
2. Sync phase: ~1 s of ring-mode scan DMA running continuously, interrupted once every ~10 ms by a one-shot data DMA. Each ring cycle of 32 scan-lines contains a wider OE pulse (W12) on scan-line 0, which is what actually does the alignment.

Without this sequence, the panel shows scrambled pixels or double-image artifacts. Once aligned, the state persists for as long as panel power stays on.

---

## 3. Wire protocol

### Command structure

Every command is one 32-bit header word followed by N payload words. The PIO program reads the header and shifts N_PRE pre-latch pulses then N_LAT latched pulses.

**Header word** layout (sent via `OUT X, 15; OUT Y, 16; OUT NULL, 1` in the PIO program):

| Bits | Field | Meaning |
|------|-------|---------|
| 0..14 | n_pre - 1 | Pre-latch CLK pulse count minus one |
| 15..30 | n_lat - 1 | Latched CLK pulse count minus one |
| 31 | pad | 0 |

### Commands sent per frame, in order

| Command | n_pre | n_lat | Payload words | Purpose |
|---------|------:|------:|-------------:|---------|
| VSYNC | 1 | 3 | 1 | Frame sync pulse |
| PRE_ACT | 2 | 14 | 4 | Pre-activation — some chip-internal state machine |
| WR_CFG | 123 | 5 | 32 | Writes one CONFIG_REGS entry per iteration |
| DATA_LATCH × 512 | 127 | 1 | 32 each | 32 scan-lines × 16 bit-planes of pixel data |

**Invariant:** `(n_pre + n_lat) mod 4 == 0`. The payload of each command occupies a whole number of 32-bit words, so the PIO's 32-bit autopull stays aligned with command boundaries.

### Scan-word layout

One 32-bit word per scan-line. Read by the scan path of the PIO program (which is a separate entry point — see §5):

| Bits | Field | Meaning |
|------|-------|---------|
| 0..6 | display - 1 | Idle cycles before OE (brightness-modulated) |
| 7..12 | 0x3F | RGB pins driven HIGH during scan (echo fix, see §4) |
| 13..17 | addr | Scan-line address (0..31) |
| 18..22 | setup - 1 | Idle cycles between display and OE |
| 23..26 | oe - 1 | OE pulse width |
| 27..31 | pad | 0 |

The OE pulse width trade-off: longer = brighter per scan but less PWM bit-depth resolution within the frame. W4 on most lines, W12 on scan-line 0 for SRAM-pointer alignment.

### Bit-plane (LATCHES_PER_LINE)

Each scan-line has 16 DATA_LATCHes (one per bit-plane). With the gamma-expanded 14-bit PWM weighting, this gives visually smooth brightness across the 6-bit RGB input range. More latches = more bit-depth but linear growth in data-phase time (and thus dark ratio).

---

## 4. The echo artifact — *the* chip gotcha

### Symptom

A faint ghost of scan-line 31's pixel content appears on scan-line 0 of the next frame. Most obvious when scan-line 31 has bright content and scan-line 0 is dark.

### Root cause(s)

Two independent triggers exist inside the DP3364S; either is sufficient to produce the echo. The fix for each is different and both must be applied simultaneously.

#### Trigger 1: ADDR pins held at 0 during the data phase

The chip's SRAM address logic is sensitive to the ADDR bus even *during* the data-shift phase. If ADDR is held at 0 while data is being clocked in, the chip mis-addresses the latch and the previous scan-line's data bleeds into scan-line 0.

**Fix:** keep ADDR pins holding their last-driven value from the previous scan-line (typically line 31 = `0b11111`). The RP2350 PIO will try to zero-fill ADDR via the OUT instruction — see §5 for the PINCTRL.OUT_COUNT work-around.

#### Trigger 2: RGB pins held at 0 during the scan phase

Symmetric to trigger 1 but on the scan side: if RGB is held at 0 during every scan word, the chip registers an extra data bit that shifts into scan-line 0's accumulator.

**Fix:** scan-word bits 7..12 = `0x3F` (all RGB pins HIGH every scan word). The scan program's `out PINS, 11` then drives RGB high and ADDR to the scan-line number simultaneously.

### Why both fixes are necessary

We independently discovered each trigger. An earlier driver generation avoided the echo by using two separately-installed PIO programs at different PC offsets; this was originally thought to be the mechanism but was actually coincidental with the PINCTRL configuration that architecture implied. The unified-program driver in `spwm_3_autonomous.rs` explicitly applies both fixes; disabling either one brings back the artifact.

---

## 5. RP2350 PIO gotchas

### 5.1 `out PINS, N` zero-fills when OUT_COUNT > N

**Undocumented-by-name but load-bearing behaviour.** When `PINCTRL.OUT_COUNT = C` and the instruction is `out PINS, N` with `N < C`, the PIO writes N bits from OSR to pins `OUT_BASE..OUT_BASE+N-1` as expected, *and* zero-fills pins `OUT_BASE+N..OUT_BASE+C-1`. Those pins are forced to 0 every execution.

**Diagnostic evidence:** a standalone PIO probe drove pins 6..10 HIGH via `SET PINDIRS`, executed `out PINS, 6` at `OUT_COUNT = 6` then at `OUT_COUNT = 11`, and read the pins back via `IN PINS`. Pins 6..10 read `0b11111` in the first case and `0b00000` in the second. Accounting for the 2-cycle input-pin synchroniser delay (§5.4), the behaviour is unambiguous.

**Consequence:** if your OUT group is wider than your per-instruction bit count, the "extra" pins are actively driven to 0 on every OUT — they aren't left alone as one might assume. For the data phase on this panel, set OUT_COUNT = 6 (exactly the `out PINS, 6` width) so the ADDR pins 6..10 retain their scan-phase value. For the scan phase, set OUT_COUNT = 11 to match `out PINS, 11` (RGB + ADDR together).

**Swapping OUT_COUNT at phase boundary**: PINCTRL is safe to write while SM0 is stalled on empty FIFO. The order in the phase-swap closure is PINCTRL → EXECCTRL → SM_INSTR. The JMP (via SM_INSTR) reads the new PINCTRL on the first instruction after the swap.

### 5.2 `assemble_with_wrap` does `source - 1` internally

In `pio-0.2.1`, `Assembler::bind(&mut label)` labels the *next* instruction slot (= `instructions.len()`, i.e. the position where the *next* `emit()` will go). So if you bind a label after the last instruction of a wrap region, `label_offset` is one past the last instruction. `assemble_with_wrap(source, target)` compensates internally with `source = label_offset(&source) - 1`, so the `Program`'s `wrap.source` field ends up equal to the PC of the last instruction in the wrap region (what EXECCTRL.WRAP_TOP wants).

**Consequence:** if you manually compute EXECCTRL.WRAP_TOP at runtime (for phase-dependent wrap ranges, as this driver does), remember to subtract 1 from `bind`-labelled positions. Using the raw label values directly makes the scan phase wrap to one PC beyond the program, landing on uninitialised PIO memory (garbage instructions) and hanging — it took a full flash-compile cycle to spot.

Verified in `pio-0.2.1/src/lib.rs:574`.

### 5.3 Fractional PIO clkdiv is unusable (for this panel)

The PIO's fractional clkdiv register supports 8.8 fixed-point division. In practice, any non-integer divisor in `(1, 2)` produces a jittery period that behaves like clkdiv=1 on the fast cycles and clkdiv=2 on the slow cycles. For the DP3364S, this intermittently pushes CLK past the 25 MHz spec and the panel misses bits.

**Use only integer clkdivs.** Write `(int_div as u32) << 16` to the CLKDIV register; leave the fraction field at 0.

### 5.4 Input-pin synchroniser: 2-cycle delay on `in pins`

When PIO writes a pin via OUT/SET/side-set, the pin output register updates on the executing cycle, but the input-pin synchroniser (which feeds the PIO's IN instruction) takes ~2 cycles to reflect the new value. Consequence: `out PINS, ... ; in PINS, ...` back-to-back reads the pin state from *before* the OUT.

This is fine for most use cases (the pins are driven output anyway, so you don't care what IN reads) but matters if you're writing a diagnostic that probes pin state from the same SM that wrote it. The simplest work-around: insert one dummy instruction between OUT and IN, or accept that the IN reads the state from the previous iteration (which is often what you want anyway).

### 5.5 SM_INSTR register writes inject an instruction

Writing to `SMx_INSTR` injects a one-shot instruction that the SM executes in preference to the one at its current PC. Used by this driver to force a JMP to the other phase's entry point. Important properties:

- Safe while the SM is stalled (waiting on empty FIFO, for example).
- Doesn't advance the PC on its own; the injected instruction is executed, then the SM continues from wherever that instruction leaves PC. A JMP therefore sets PC and proceeds.
- Does not clear X/Y/OSR/ISR state.
- Only the low 5 bits of the JMP address field matter (PIO programs are ≤ 32 instructions).

### 5.6 DMA CTRL register layout (RP2350 ≠ RP2040)

RP2350's DMA CTRL register bit layout differs from RP2040 in at least the TREQ_SEL position. This driver uses raw register writes for ring-mode scan DMA with:

```rust
const CH1_CTRL_VAL: u32 =
      (1 << 0)     // EN
    | (2 << 2)     // DATA_SIZE = word (32-bit)
    | (1 << 4)     // INCR_READ
    | (7 << 8)     // TREQ_SEL = 7 (DREQ for PIO0 TX0)
    | (1 << 13);   // RING on read side, ring size = 2^5 = 32 words
```

The rp235x-pac module's DMA register accessors will give you the authoritative bit positions; don't assume RP2040 conventions.

### 5.7 DMA IRQ completion

To free core 0 from busy-waiting on DMA, use DMA_IRQ_0:

```rust
// At boot:
ch.enable_irq0();                               // ch0 via HAL helper
unsafe { (DMA_INTE0 as *mut u32).write_volatile(...); }  // ch1 raw
unsafe { NVIC::unmask(Interrupt::DMA_IRQ_0); }

#[interrupt]
fn DMA_IRQ_0() {
    // Clear pending INTS0 bits (W1C), record which channels finished.
}

// Main loop:
let xfer = cfg.start();
wait_dma(1 << 0);       // sleeps on `wfi` until IRQ sets the bit
let (ch, _, tx) = xfer.wait();   // quick — DMA already done
```

The `#[interrupt]` macro comes from `rp235x-hal::pac::interrupt` (re-exports `cortex_m_rt::interrupt`). Each core has its own NVIC; unmask on whichever core needs the wake-up.

### 5.8 DWT cycle counter is per-core

Each Cortex-M33 core has its own DWT. Writing DEMCR/DWT_CTRL on core 0 does *not* enable the counter on core 1 — the core-1 task must call `enable_cycle_counter()` itself before using `now_cycles()` for timing.

---

## 6. Recommended architecture

```
┌──────────────────┐        ┌──────────────────┐
│ Core 0           │        │ Core 1           │
│ • orchestrate    │        │ • pack next      │
│   DMA + phase    │        │   frame into     │
│   swaps          │        │   DATA_LATCHes   │
│ • ~99 % wfi      │        │ • ~60 % busy     │
└──────────────────┘        └──────────────────┘
         │                           │
         │ phase swap (PINCTRL /     │
         │ EXECCTRL / SM_INSTR)      │ SIO FIFO
         ▼                           ▼ handshake
┌─────────────────────────────────────────────┐
│ PIO0 SM0 — unified program, two entries     │
│   data_cmd (PC 0)   ← force JMP             │
│   scan_entry (PC 11) ← force JMP            │
└─────────────────────────────────────────────┘
      ▲                        ▲
      │ ch0 single-shot        │ ch1 ring-mode
      │  (header + 512         │  (SCAN_BUF ×
      │   DATA_LATCHes)        │   N cycles)
   FRAME_BUF_A / _B         SCAN_BUF
   double-buffered          32 scan words
```

**Why one SM and not two:** a single SM + register-swap at phase boundaries is simpler than two SMs with coordinated FIFOs. The two-SM design (earlier iterations) had no measurable benefit and complicated state-machine handover. The unified program keeps both phases' code in one place.

**Why ring-mode scan DMA:** replaces 50 per-frame one-shot scan DMAs with one DMA that replays SCAN_BUF `POST_SCAN_CYCLES` times. Cuts per-frame DMA-kickoff overhead from 51 to 2 and drops IRQ rate from 3264/s to 128/s. Frees core 0 to sleep in `wfi` for the entire scan phase.

**Why IRQ-driven DMA waits:** busy-waiting `xfer.wait()` occupies core 0 for ~14 ms per 15.6 ms frame — ~90 % of the core doing nothing. IRQ + `wfi` frees ~99 % of core 0's cycles for application work.

---

## 7. Pack algorithm

The pack step converts the 64 × 128 RGB pixel array into bit-packed DATA_LATCH words. It's fundamentally a bit transpose across bit-planes, chips, and pixel positions.

### Naive bit-by-bit pack

Loops `(scan_line, channel, word, slot, colour)`, extracting one bit per iteration. ~21 ms for a 64×128 frame at 150 MHz.

### Chip-major + nibble-scatter LUT

~4.9 ms (4× faster). Key ideas:

1. **Iterate chip-major.** For each of the 8 chips, load its 6 gamma-expanded `u16` values once, then produce all 4 output words for that chip directly.
2. **Nibble-scatter LUT.** Each output word contains 4 pixels × 6 colour bits, laid out at byte-aligned positions (bit 0 of each byte). A 16-entry LUT scatters a 4-bit nibble of the gamma value to the 4 byte-0 positions:
   ```rust
   const SCATTER: [u32; 16] = [
       0x00000000, 0x01000000, 0x00010000, 0x01010000,
       0x00000100, 0x01000100, 0x00010100, 0x01010100,
       0x00000001, 0x01000001, 0x00010001, 0x01010001,
       0x00000101, 0x01000101, 0x00010101, 0x01010101,
   ];
   ```
3. **ORed-by-colour.** Shift each colour's scattered nibble by the colour index (0..5) and OR into the output word. Six ORs per output word vs 24 shifts + 24 ORs for the naive pack.

See `pack_pixels` in `spwm_3_autonomous.rs`.

### Gamma table

`GAMMA14[u8]` → `u16`. 8-bit input, 14-bit output. Quadratic weighting (`v = i*i * 0x3FFF / (255*255)`) gives a perceptually reasonable curve. Straightforward to swap for a CIE-lab-based or dithered alternative.

---

## 8. Flicker & brightness tuning

### POST_SCAN_CYCLES trade-off

| Cycles | Data freq | Dark % | Notes |
|-------:|----------:|-------:|-------|
|     50 |     64 Hz |  17 %  | **Brightness sweet spot.** Each frame has more scan time. |
|     30 |    ~96 Hz |  ~27 % | Middle ground. |
|     20 |   ~128 Hz |  ~34 % | **Refresh-rate sweet spot.** Dimmer but visibly smoother motion. |
|     57 |    ~58 Hz |  ~15 % | Borderline flicker — don't run below 58 Hz. |

Flicker is **frequency-driven**: 64 Hz / 17 % dark is perceived better than 39 Hz / 10 % dark. Prioritise refresh rate over dark-gap reduction when tuning.

### The dark gap is PIO-limited

Data-phase time = `DATA_END words × 8 bits / word / 8 Mbits/sec PIO ≈ 2.66 ms`. This is the time to shift 512 DATA_LATCHes × 128 × 6 = 393 216 RGB bits through the chip at the 25 MHz CLK maximum. You cannot reduce this without either (a) a faster CLK (impossible, 25 MHz is the chip spec) or (b) fewer DATA_LATCHes per frame (reduces bit-depth).

### Per-scan-line refresh rate

`display_Hz × POST_SCAN_CYCLES`. Typically 2500-3200 Hz. Well above flicker-fusion; not a variable worth tuning independently.

---

## 9. False assumptions that cost time

Worth knowing what *doesn't* work, so you can skip re-testing:

1. **"Two separately-assembled PIO programs cause the echo fix."** Wrong. It was a correlate of PINCTRL being swapped at phase boundary in that architecture. The mechanism is OUT_COUNT.
2. **"SET_BASE/SET_COUNT swap is the load-bearing PINCTRL change."** Wrong. Verified via A/B sweep: OUT_COUNT swap alone kills the echo, SET swap alone does not.
3. **"The 2.66 ms dark gap can be reduced by faster CLK."** No — DP3364S spec is 25 MHz. Anything above breaks the chip.
4. **"2-cycle PIO pre-loop would halve data time."** Infeasible — would push CLK to 37.5 MHz, over spec.
5. **"POST_SCAN_CYCLES = 50 is the universal sweet spot."** It's the *brightness* sweet spot. Refresh-rate sweet spot is around 20. Choose per application.
6. **"Echo is about scan-line-31 → 0 transition electrically."** It's chip-internal; scan-line 0 reads its data from the same SRAM that line 31 polluted. The trigger (ADDR or RGB being 0) is what causes the pollution; the symptom (visual echo) is a consequence.
7. **"The RP2350 PIO docs say what `out PINS, N` does when OUT_COUNT > N."** They don't, directly. `rp235x-hal`'s `out_pins` doc comment says "up to count pins" which is ambiguous. Treat OUT_COUNT as exactly the bit count you need, not a safety ceiling.
8. **"Chip register 0x0E must do something useful — let's sweep."** We did. It doesn't.
9. **"OE pulse width is a tunable brightness knob."** No — running uniform W12 across every scan-line breaks the upper/lower-half sync. The W12/W4 split (line 0 = W12, lines 1-31 = W4) is load-bearing; shorter pulses on line 0 fail to align the chip SRAM pointers, longer pulses on other lines desync the halves.
10. **"`LATCHES_PER_LINE` is a PWM bit-plane count; increasing it gives smoother gradients."** No — it's chip geometry. Each DP3364S chip has 16 column outputs; `LATCHES_PER_LINE = 16` maps directly to that. Going higher overflows column indices in `pack_pixels`; going lower leaves columns undriven.
11. **"Changing the gamma curve adds dim-end resolution."** No — gamma only redistributes the chip's 14-bit PWM range across the 256 input values; it doesn't add steps. Dim-end stepping is a fundamental 14-bit limit. Dithering in `pack_pixels` (spatial Bayer or temporal) would be the only way to fake more effective resolution.

---

## Companion files

- `examples/spwm_3_autonomous.rs` — working reference implementation.
- `reference/DP3364S/` — the DP3364S datasheet.
