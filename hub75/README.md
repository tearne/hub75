# hub75

HUB75 LED matrix panel driver for the [Pimoroni Interstate 75 W](https://shop.pimoroni.com/products/interstate-75-w) (RP2350A), built on [embassy-rp](https://crates.io/crates/embassy-rp).

Setup and flashing: [`SETUP.md`](../SETUP.md), [`FLASHING.md`](../FLASHING.md).

## Panel families

HUB75 panels share a connector but use very different driver chips. The crate provides a concrete type for each family:

- [`Dp3364sPanel`] — **S-PWM family** (Scrambled-PWM column drivers with on-chip SRAM and PWM, e.g. **DP3364S**, DP3264S). The host loads 14-bit greyscale per pixel into chip SRAM; the chip handles brightness internally. Hardware-tied to 64×128.
- [`shift::ShiftPanel<W, H>`] — **shift-register family** (74HC595-style column drivers, e.g. **FM6124**, ICN2037, MBI5124). Host computes BCM bitplanes and clocks them out per-bitplane while pulsing OE for binary-weighted on-time. Const-generic over panel size; tested at 64×64 and 64×32.

Both run autonomously on PIO + DMA after construction. The `Panel` trait gives a common `frame_mut()` / `commit()` surface across the two; family-specific tunables live as inherent methods on each concrete type.

## Quickstart

```sh
cd hub75

# S-PWM family (e.g. DP3364S panel)
cargo run --release --example spwm_rainbow

# Shift-register family (e.g. 64×64 / 64×32 generic panel)
cargo run --release --example shift_rainbow
```

Each example fills the panel with solid R / G / B / W (3 s each) then a scrolling diagonal rainbow.

### `test_pattern` — diagnostic for new panels

When bringing up a new physical panel, use the `test_pattern` example to characterise its wiring:

```sh
cargo run --release --example test_pattern --features panel-shift-64x32
# or panel-shift-64x64, or panel-spwm-128x64
```

Cycles solid R/G/B/W → solid Y/C/M (each labelled in tiny black text on the panel itself, so a probe isn't required) → a "sunset" image with a sun in the upper-left → a bouncing ball that settles on the panel's bottom edge. Discrepancies vs expected reveal what wiring corrections that panel needs.

## Per-panel features

Compile-time features for the physical panels we've characterised. Selecting one enables the wiring corrections for that panel inside the family's `pack` code.

| Feature | Family | Size | Quirks corrected |
|---|---|---|---|
| `panel-shift-64x64` | shift-register | 64×64 | (none — wiring is correct) |
| `panel-shift-64x32` | shift-register | 64×32 | G ↔ B channel swap |
| `panel-spwm-128x64` | S-PWM (DP3364S) | 128×64 | column direction reversed |

## Usage — shift-register panel

```rust
use hub75::shift::{ShiftPanel, ShiftStorage};
use hub75::{InterstatePins, Panel};

embassy_rp::bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<embassy_rp::peripherals::PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    static STORAGE: StaticCell<ShiftStorage<64, 64>> = StaticCell::new();
    let storage = STORAGE.init(ShiftStorage::new());

    let pins = InterstatePins {
        r0: p.PIN_0, g0: p.PIN_1, b0: p.PIN_2,
        r1: p.PIN_3, g1: p.PIN_4, b1: p.PIN_5,
        addr_a: p.PIN_6, addr_b: p.PIN_7, addr_c: p.PIN_8,
        addr_d: p.PIN_9, addr_e: p.PIN_10,
        clk: p.PIN_11, lat: p.PIN_12, oe: p.PIN_13,
    };

    let mut panel: ShiftPanel<64, 64> = ShiftPanel::new(
        storage, p.PIO0, Irqs, pins,
        p.DMA_CH0, p.DMA_CH1, p.DMA_CH2, p.DMA_CH3,
    );

    let frame = panel.frame_mut().await;
    frame[10][20] = hub75::Rgb::new(255, 0, 0);
    panel.commit();
}
```

## Usage — S-PWM (DP3364S) panel

```rust
use hub75::{DmaIrqHandler, Dp3364sPanel, InterstatePins, Panel, Rgb};

embassy_rp::bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<embassy_rp::peripherals::PIO0>;
    DMA_IRQ_0 => DmaIrqHandler;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    static CORE1_STACK: StaticCell<Stack<4096>> = StaticCell::new();
    let core1_stack = CORE1_STACK.init(Stack::new());

    let pins = InterstatePins {
        r0: p.PIN_0, g0: p.PIN_1, b0: p.PIN_2,
        r1: p.PIN_3, g1: p.PIN_4, b1: p.PIN_5,
        addr_a: p.PIN_6, addr_b: p.PIN_7, addr_c: p.PIN_8,
        addr_d: p.PIN_9, addr_e: p.PIN_10,
        clk: p.PIN_11, lat: p.PIN_12, oe: p.PIN_13,
    };

    let mut panel = Dp3364sPanel::new(
        spawner, Irqs, p.PIO0, pins,
        p.DMA_CH0, p.DMA_CH1, p.CORE1, core1_stack,
    );

    let frame = panel.frame_mut().await;
    frame[10][20] = Rgb::new(255, 0, 0);
    panel.commit();
}
```

## API shape

The panel owns the RGB pixel buffer. `frame_mut()` borrows it (awaiting any in-flight pack first), the caller writes pixel values in place, and `commit()` releases the borrow and triggers the pack. The packed frame becomes visible at the next display-frame boundary.

## Tunables

- `Dp3364sPanel::set_scan_cycles(u32)` — refresh-rate / brightness trade-off. Default 20 (~128 Hz, ~34 % dark gap, refresh-rate sweet spot). 50 → ~64 Hz / ~17 % dark (brightness sweet spot). Above ~57 the refresh enters perceptible flicker.
- `ShiftPanel::set_brightness(u8)` — 0–255, applied via the BCM timing buffer at the next address-SM cycle.

## Architecture

The S-PWM driver is the embassy-rp port of [`learning-examples/examples/spwm_3_autonomous.rs`](../learning-examples/examples/spwm_3_autonomous.rs); the shift-register driver is descended from `learning-examples/examples/shift_3_dual_pio.rs`. Both keep their pre-port single-file form there as canonical, fully-annotated references — read those for architectural rationale (unified PIO + `OUT_COUNT` swap for echo suppression on S-PWM; dual-SM IRQ-handshake + four-channel chained DMA on shift-register).
