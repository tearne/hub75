# learning-examples

Self-contained bare-metal examples for the Interstate 75 W + HUB75 panel. Each example is a single file with everything inline — no shared libraries.

See the [root README](../README.md) for prerequisites and hardware setup.

## Examples

Run from this directory:

```bash
cargo run --release --example <name>
```

| Example | Description |
|---------|-------------|
| `onboard_rgb` | Blink the onboard WS2812 NeoPixel LED via PIO |
| `minimal_cpu` | Cycle solid colours on the panel — simplest HUB75 driver (CPU clocks every pixel) |
| `bitplane_cpu` | Per-pixel colour via Binary Code Modulation — animated colour-cycling circle |
| `minimal_dma` | Same as `minimal_cpu` but pixel data is transferred via PIO+DMA |
| `dma_bounce` | Fully autonomous scanning: two PIO state machines + four chained DMA channels drive the display with zero CPU involvement. Bouncing colour-changing square. |

The examples progress in complexity: `onboard_rgb` → `minimal_cpu` → `bitplane_cpu` → `minimal_dma` → `dma_bounce`.
