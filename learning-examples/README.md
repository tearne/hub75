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

| Example | Visually | What the CPU does | What hardware does |
|---------|----------|-------------------|--------------------|
| `shift_1_cpu` | Solid colours | Everything | — |
| `shift_2_pio_colour` | Scrolling rainbow | BCM bitplanes + scan | — |
| `shift_3_dual_pio` | Bouncing square | Draw into framebuffer | Everything (2 SMs + chained DMA) |
| `spwm_1_cpu` | Solid fills + scanning line | Everything | — |
| `spwm_2_pio_cpu` | Scrolling rainbow | Scan + pack framebuffer | All 4 S-PWM commands via PIO+DMA |
| `spwm_3_dual_pio` | Scrolling rainbow (faster, less flicker) | Pack framebuffer | Commands + scan (two SMs) |
| `spwm_4_dual_core` | Scrolling rainbow (smoothest) | Display loop only | Packing on core 1, display on core 0 |
| `spwm_5_unified_sm` | Scrolling rainbow (no dark gap) | Display loop only | Single SM for data+scan, dual-core packing |
| `spwm_6_ring_dma` | Scrolling rainbow (gapless scan) | Buffer swap only | Ring-mode DMA for continuous scan |
| `spwm_7_chained_dma` | *Not yet implemented* | — | Chained DMA: scan→data→scan in hardware |

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
| `spwm_1_cpu` | Solid fills with scanning line — CPU bit-bangs everything (protocol reference) |
| `spwm_2_pio_cpu` | PIO+DMA commands, per-pixel packing — scrolling HSV rainbow |
| `spwm_3_dual_pio` | Dual PIO SM with overlapped packing — scrolling rainbow (brighter, smoother) |
| `spwm_4_dual_core` | Core 1 packs framebuffer while core 0 drives display — smoothest rainbow |
| `spwm_5_unified_sm` | Single SM for data+scan (no handover dark gap) — dual-core, constant brightness |
| `spwm_6_ring_dma` | Ring-mode DMA for gapless scan — zero micro-stutter between scan passes |
| `spwm_7_chained_dma` | *Placeholder* — chained DMA: scan→data→scan entirely in hardware |

Progression: `spwm_1` → `spwm_2` → `spwm_3` → `spwm_4` → `spwm_5` → `spwm_6` → `spwm_7`.

### Other

| Example | Description |
|---------|-------------|
| `onboard_rgb` | Blink the onboard WS2812 NeoPixel LED via PIO (no panel needed) |
