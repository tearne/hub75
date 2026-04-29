# learning-examples

Self-contained bare-metal examples driving HUB75 LED matrix panels on the [Pimoroni Interstate 75 W](https://shop.pimoroni.com/products/interstate-75-w) (RP2350A). Each example is a single file with everything inline — no shared libraries. Reads top-to-bottom as a progression from CPU bit-bang to fully-autonomous PIO + DMA.

Setup and flashing: [`SETUP.md`](../SETUP.md), [`FLASHING.md`](../FLASHING.md).

```sh
cd learning-examples
cargo run --release --example <name>            # with a probe
```

For BOOTSEL + `picotool`, see [`FLASHING.md`](../FLASHING.md).

## Terminology

| Acronym | Meaning |
|---------|---------|
| **PIO** | Programmable I/O — RP2350 peripheral: tiny state machines that drive GPIO pins with cycle-exact timing |
| **DMA** | Direct Memory Access — hardware that copies data between RAM and peripherals without CPU involvement |
| **SM** | State Machine — one of four independent execution units inside a PIO block |
| **BCM** | Binary Code Modulation — brightness control by scanning multiple bitplanes with weighted display times |
| **S-PWM** | Scrambled PWM — on-chip brightness control via internal PWM engine (no host-side BCM needed) |

## All examples at a glance

Each step moves more work off the CPU onto PIO + DMA hardware. Read top to bottom — by the end the CPU's doing pixel maths on one core while the other core, a PIO state machine, two DMA channels and a DMA IRQ keep the panel lit on their own.

| Example | Visually | What the CPU does | What hardware does |
|---------|----------|-------------------|--------------------|
| `shift_1_cpu` | Solid colours | Everything | — |
| `shift_2_pio_colour` | Scrolling rainbow | BCM bitplanes + scan | — |
| `shift_3_dual_pio` | Bouncing square | Draw into framebuffer | Everything (2 SMs + chained DMA) |
| `spwm_1_pio_cpu` | Scrolling rainbow | Scan + pack framebuffer | All 4 S-PWM commands via PIO+DMA |
| `spwm_2_unified_sm` | Scrolling rainbow (no dark gaps) | Buffer swap only | Single SM for data+scan + ring-mode scan DMA + packing on core 1 |
| `spwm_3_autonomous` | Animated rainbow (128 Hz content) | Practically nothing (~99 % `wfi`) | Production driver: echo-free, IRQ-driven, optimised pack |

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
| `spwm_1_pio_cpu` | Protocol introduction. The CPU bit-bangs the scan loop, but the four S-PWM commands (VSYNC, PRE_ACT, WR_CFG, DATA_LATCH) are handed to PIO+DMA. Scrolling HSV rainbow with 14-bit greyscale + gamma. |
| `spwm_2_unified_sm` | Architectural milestone. A single PIO SM handles both data and scan phases via a type-flag-dispatch prelude; ring-mode DMA for the scan loop; packing offloaded to core 1. Also serves as the **panel-recovery tool** — flash this and cold-boot the panel if the driver chips end up in a desync state. |
| `spwm_3_autonomous` | Production driver. Adds the echo-fix PINCTRL.OUT_COUNT swap at phase boundary, IRQ-driven `wfi` on DMA completion (core 0 is ~99 % free), and an optimised bit-transpose pack (~4.9 ms per 64×128 frame). 128 Hz display, 128 Hz content update. See [`reference/HUB75_DP3364S_RP2350_NOTES.md`](reference/HUB75_DP3364S_RP2350_NOTES.md) for the full architecture. |

Progression: `spwm_1` → `spwm_2` → `spwm_3`.

**Panel recovery:** if the panel's driver chips get desynchronised during experimentation, flash `spwm_2_unified_sm` or `panel_reset` and cold-boot the panel to restore a known-good sync state.

### Other

| Example | Description |
|---------|-------------|
| `onboard_rgb` | Blink the onboard WS2812 NeoPixel LED via PIO (no panel needed) |
| `panel_reset` | Overwrite the panel's SRAM with zeros and run a known-good sync sequence. Useful after flashing something that leaves the panel in a weird state. |
