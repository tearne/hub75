# usb-display

Receives RGB frames from a host computer over USB and displays them on a HUB75 panel. The panel is scanned autonomously by hardware (PIO + DMA) while the CPU handles USB communication via [Embassy](https://embassy.dev/).

The autonomous scan architecture — using two PIO state machines with an IRQ handshake and four chained DMA channels to drive the display with zero CPU involvement — is based on [dgrantpete/Pi-Pico-Hub75-Driver](https://github.com/dgrantpete/Pi-Pico-Hub75-Driver). That project demonstrated how to offload the entire scan loop to hardware, freeing the CPU completely. This implementation ports the approach to Rust/Embassy on the RP2350, adapted for the Interstate 75 W's pin layout.

See the [root README](../README.md) for prerequisites and hardware setup.

## Flash the firmware

```bash
cd firmware
cargo run --release
```

The firmware persists in flash across power cycles. `cargo run` keeps the terminal open for defmt log output; Ctrl+C to disconnect (firmware keeps running).

## Send frames from Python

Requires [uv](https://docs.astral.sh/uv/) (dependencies install automatically):

```bash
./client/python/hub75_client.py                      # animated rainbow (default)
./client/python/hub75_client.py --pattern solid-red   # solid colour
./client/python/hub75_client.py --pattern gradient    # red-to-blue gradient
./client/python/hub75_client.py --fps 15             # faster animation
./client/python/hub75_client.py /dev/ttyACM1         # explicit serial port
```

## Send frames from Rust

```rust
use hub75_client::Hub75Client;

let mut client = Hub75Client::open_auto()?;
let frame = vec![[255, 0, 0]; 64 * 64];  // solid red
client.send_frame_rgb(&frame)?;
```

The Rust client crate is at `client/rust/`. It includes a Game of Life example:

```bash
cd client/rust
cargo run --example life
```

## Protocol

Each frame is a binary packet over USB CDC serial:

| Field | Size | Description |
|-------|------|-------------|
| Magic | 4 bytes | `HB75` (`0x48 0x42 0x37 0x35`) |
| Sequence | 1 byte | Wrapping counter (0-255) for dropped-frame detection |
| Pixels | 12,288 bytes | 64 x 64 x 3 (RGB), row-major, top-left origin |

The client auto-detects the device by USB manufacturer (`tearne`) and product (`hub75`).

## Architecture

The firmware uses two PIO state machines and four chained DMA channels to scan the display continuously in hardware. The CPU is 100% free for USB packet processing, achieving ~18 fps throughput at USB full-speed.

---

## Technical annex

### Autonomous scan engine

The display refreshes at ~145 Hz with zero CPU involvement after setup. Two PIO state machines coordinate via hardware IRQ handshake:

**Data SM (SM0)** — clocks 64 pixels per row into the panel's shift registers, then pulses the latch.
- OUT pins: GPIO 0-5 (R0, G0, B0, R1, G1, B1)
- Sideset: GPIO 11 (CLK), GPIO 12 (LAT) — 2-bit sideset
- 3 PIO instructions per pixel: `out pins 6`, `out null 2` (with CLK pulse), `jmp`

**Address SM (SM1)** — sets the 5-bit row address and controls OE for Binary Code Modulation (BCM).
- OUT pins: GPIO 6-10 (ADDR A-E)
- Sideset: GPIO 13 (OE) — active low
- Cycles through 32 row-pairs per bitplane, 8 bitplanes per frame
- Consumes `[off_cycles, on_cycles]` timing pairs from DMA for BCM weighting

**IRQ handshake** — Address SM signals "safe to latch" (IRQ 0), Data SM latches and signals back (IRQ 1). This ensures the row address is stable before data is committed to the LED drivers.

### DMA chain

Four DMA channels in two self-restarting chains:

```
CH0 ──pixel data──▶ SM0 TX FIFO    (PIO-paced via DREQ)
 └─chains to─▶ CH1 ──reloads CH0 read address──▶ (restarts CH0)

CH2 ──timing data──▶ SM1 TX FIFO   (PIO-paced via DREQ)
 └─chains to─▶ CH3 ──reloads CH2 read address──▶ (restarts CH2)
```

The pixel chain uses pointer indirection for double-buffering: CH1 reads `ACTIVE_BUF_PTR` and writes it to CH0's read address trigger register. To swap frames, the CPU just updates the pointer — DMA picks it up on the next cycle.

### Pixel buffer layout

RGB pixels are packed into bitplane format for BCM scanning. Each pixel becomes 6 bits (R0, G0, B0, R1, G1, B1 for the upper and lower panel halves). Four pixels pack into one u32 word (6 bits + 2 padding each). The buffer is laid out contiguously by bitplane:

```
[bitplane 0: 32 rows × 16 words] [bitplane 1] ... [bitplane 7]
```

Total: 4,096 words (16 KB) per buffer, two buffers for double-buffering.

### Clock divider

The PIO clock divider is set to 4 (sysclk/4 ≈ 37.5 MHz). Divider 2 causes setup/hold violations on the panel's shift registers due to the 3-instruction pixel loop. The Interstate 75 W's pin layout (address pins GPIO 6-10 adjacent to data pins 0-5) prevents using the 2-instruction `out pins, 8` loop that the reference driver uses.

### Key RP2350 findings

- **PIO pin function on drop**: Embassy-rp's GPIO `Output::drop()` resets the pin's funcsel to NULL. Pins must be `core::mem::forget()`'d if they outlive their scope.
- **OE default state**: When `make_pio_pin()` switches OE from GPIO to PIO function, it goes high-impedance. The panel's pull-up drives it HIGH (display off). The Address SM must be running to drive OE low.
- **DMA register access**: Embassy-rp's DMA API doesn't expose `chain_to`. Raw PAC register writes via the `unstable-pac` feature are required for DMA chaining.
