# learning-examples

Self-contained bare-metal examples for the [Pimoroni Interstate 75 W](https://shop.pimoroni.com/products/interstate-75-w) (RP2350A) driving HUB75 LED matrix panels. Each example is a single file with everything inline — no shared libraries.

See the [root README](../README.md) for prerequisites and hardware setup.

## Hardware

- [Pimoroni Interstate 75 W](https://shop.pimoroni.com/products/interstate-75-w) (RP2350A version)
- HUB75 LED matrix panel (see tested panels below)
- Raspberry Pi Debug Probe (or any CMSIS-DAP SWD probe) for flashing

## Run

```bash
cargo run --release --example <name>
```

## Terminology

| Acronym | Meaning |
|---------|---------|
| **PIO** | Programmable I/O — RP2350 peripheral: tiny state machines that drive GPIO pins with cycle-exact timing |
| **DMA** | Direct Memory Access — hardware that copies data between RAM and peripherals without CPU involvement |
| **SM** | State Machine — one of four independent execution units inside a PIO block |
| **BCM** | Binary Code Modulation — brightness control by scanning multiple bitplanes with weighted display times |
| **S-PWM** | Scrambled PWM — on-chip brightness control via internal PWM engine (no host-side BCM needed) |

## All examples at a glance

Each step moves more work off the CPU onto PIO + DMA hardware. Picture
the CPU sitting back further each time — by the end it's doing pixel
math on one core while the other core, two PIO state machines and a
pair of DMA channels keep the panel lit on their own.

| Example | Visually | What the CPU does | What hardware does |
|---------|----------|-------------------|--------------------|
| `shift_1_cpu` | Solid colours | Everything | — |
| `shift_2_pio_colour` | Scrolling rainbow | BCM bitplanes + scan | — |
| `shift_3_dual_pio` | Bouncing square | Draw into framebuffer | Everything (2 SMs + chained DMA) |
| `spwm_2_pio_cpu` | Scrolling rainbow | Scan + pack framebuffer | All 4 S-PWM commands via PIO+DMA |
| `spwm_3_dual_pio` | Scrolling rainbow (faster, less flicker) | Pack framebuffer | Commands + scan (two SMs) |
| `spwm_4_dual_core` | Scrolling rainbow (smoothest) | Display loop only | Packing on core 1, display on core 0 |
| `spwm_5_unified_sm` | Scrolling rainbow (no dark gaps) | Buffer swap only | Single SM for data+scan + ring-mode scan DMA |
| `spwm_7_ddr` | *WIP — blank display* | Buffer swap only | DDR double-edge clocking (halves data phase) |
| `spwm_8_padded_continuous` | Scrolling rainbow (uniform row cadence) | Buffer swap only | One continuous DMA per frame, all scans pre-baked into the stream |

## Two families of HUB75 driver chip

HUB75 panels use the same physical connector but different driver ICs with fundamentally different protocols. The examples are split into two groups by prefix:

### `shift_` — Traditional shift-register drivers (BCM)

For panels using simple shift-register LED drivers such as **FM6124**, **ICN2037**, or **MBI5124**. The host controller is responsible for brightness: it implements [Binary Code Modulation](https://www.batsocks.co.uk/readme/art_bcm_1.htm) (BCM) by cycling through bitplanes with timed OE pulses.

**Tested with:** 64×64 HUB75 panel (FM6124 drivers, 1/32 scan).

| Example | Description |
|---------|-------------|
| `shift_1_cpu` | Cycle solid colours — CPU clocks every pixel |
| `shift_2_pio_colour` | Per-pixel colour via BCM bitplane packing — scrolling HSV rainbow |
| `shift_3_dual_pio` | Fully autonomous: two PIO SMs + four chained DMA channels — bouncing square, zero CPU involvement |

Progression: `shift_1` → `shift_2` → `shift_3`.

### `spwm_` — S-PWM (Scrambled PWM) drivers

For panels using constant-current S-PWM drivers such as **DP3364S** or **DP3264S** (Depuw / Shenzhen Developer Microelectronics). These chips contain internal SRAM + a PLL-driven PWM engine — brightness is handled on-chip. The host loads 14-bit greyscale data per pixel and keeps the scan clock running.

The S-PWM protocol reuses the HUB75 pins with different semantics: commands are encoded by LAT pulse width, and OE is repurposed as a ROW advance signal.

**Tested with:** 64×128 HUB75 panel (DP3364S drivers, 1/32 scan).

| Example | Description |
|---------|-------------|
| `spwm_2_pio_cpu` | Protocol reference + PIO+DMA commands — the CPU bit-bangs the scan loop, but the four S-PWM commands are handed to hardware. Scrolling HSV rainbow with 14-bit greyscale + gamma. |
| `spwm_3_dual_pio` | Second PIO SM handles the scan loop on its own, sharing GPIO 11 with the data SM via `pindir` handover. Pack and scan now overlap, so the display stays lit while the next frame is being packed. |
| `spwm_4_dual_core` | Core 1 fills + packs; core 0 runs the display loop. Still two PIO SMs, but with a short (~2.6 ms) dark gap during the SM handover each frame. |
| `spwm_5_unified_sm` | Single PIO SM with a type-flag bit at the start of every DMA word: type=0 → data path, type=1 → scan path. The handover dark gap disappears. Scan uses ring-mode DMA so the CPU doesn't touch the scan channel at all. |
| `spwm_7_ddr` | *WIP, blank display* — DDR (double data rate) clocking to halve the data phase from ~2.6 ms to ~1.3 ms. Untested. |
| `spwm_8_padded_continuous` | One continuous DMA per frame. All 32 scan_words for the frame are pre-baked into the buffer together with the data, and the buffer is padded with extra scan cycles so each DMA iteration takes ~16.7 ms. Every row sees a uniform ~3 kHz cadence. |

Progression: `spwm_2` → `spwm_3` → `spwm_4` → `spwm_5` → `spwm_7` / `spwm_8` (two orthogonal optimisation branches from step 5).

**Panel recovery:** if the panel's driver chips get desynchronised
during experimentation (most likely while poking at `spwm_7` or
`spwm_8`), flash `spwm_5_unified_sm` and cold-boot the panel to
restore a known-good sync state.

### Other

| Example | Description |
|---------|-------------|
| `onboard_rgb` | Blink the onboard WS2812 NeoPixel LED via PIO (no panel needed) |
