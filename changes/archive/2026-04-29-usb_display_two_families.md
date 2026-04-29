# usb-display: support both panel families + life polish

## Intent

`usb-display/firmware` currently drives only the shift-register HUB75 family (via `ShiftPanel`). The `hub75` crate now also supports the DP3364S S-PWM family — but to actually use a DP3364S panel through USB, a user has to write their own firmware. The firmware should be able to drive either family, picked at compile time, the same way it already picks the size.

The Rust and Python clients should keep working unchanged across all three panel options — same client code, just a different feature flag (Rust) or `--width`/`--height` (Python).

The `usb-display/README.md` then needs to be readable for someone meeting two panel families for the first time: which option to pick, what the difference is, and what each example client (Python pattern script, Rust life, Rust clock) actually does on the panel.

Finally, the Rust `life` example tends to settle into period-2 oscillators (blinkers) within ~50 generations and stay there indefinitely — visually dead. It should detect stagnation and inject some random "particles" so the simulation stays alive over long runs.

## Approach

### Firmware features get a family prefix; client features stay size-only

The firmware genuinely cares about which family — different PIO programs, different DMA channel counts, different IRQ bindings. So firmware features become `panel-shift-64x64`, `panel-shift-64x32`, `panel-spwm-128x64` (rename + add). The client just sends RGB bytes over USB; it only needs the frame size. So the Rust client adds `panel-128x64` to its existing `panel-64x64` / `panel-64x32` (additive, no rename).

The README documents which firmware feature pairs with which client feature.

Versioning: firmware bumps `0.2.0 → 0.3.0` (rename = breaking). Rust client bumps `0.2.0 → 0.2.1` (additive feature only).

### Firmware: cfg-gated construction in `main.rs`

The two families need different things from the constructor (spwm wants `Spawner`, `CORE1`, a stack, `DMA_IRQ_0` binding; shift wants four DMA channels and only `PIO0_IRQ_0`). Rather than introduce a panel-abstraction layer, just cfg-gate the construction block, the `bind_interrupts!` lines, and the panel-task body in `main.rs`. Both paths still feed the same `RX_BUF`/`FRAME_READY` channel and the same USB rx logic.

### Python client is unchanged

Already takes runtime `--width`/`--height`. Works for any panel size.

### `life` stagnation detection: short-period hash match

Maintain a small ring of the last few generations' state hashes (e.g. last 3). If the current generation's hash matches any in the ring, increment a stagnant counter. Reset on novel state. Once the counter exceeds a threshold (~30 generations), inject a handful of randomly-placed live cells with random hues, then resume normal evolution.

This catches static (period-1), blinker (period-2), and the next harmonic (period-3) without per-generation scoring.

### README restructure

The current README has a "Panel size" section. Expand to "Panel options", listing all three with their families and characteristics. Then a new "Client examples" section listing what each Python pattern (`rainbow`, `gradient`, `solid-*`, `scanline_test`) and Rust example (`life`, `clock`) does on the panel.

## Plan

- [x] In `usb-display/firmware/Cargo.toml`: rename `panel-64x64` → `panel-shift-64x64`, `panel-64x32` → `panel-shift-64x32`, add `panel-spwm-128x64`. Bump version to `0.3.0`.
- [x] In `usb-display/firmware/src/display.rs`: update the three `#[cfg]` arms to use the new feature names (and the spwm one yields `WIDTH=128 HEIGHT=64`); update the `compile_error!` lists.
- [x] In `usb-display/firmware/src/main.rs`: cfg-gate the `bind_interrupts!` (`DMA_IRQ_0` line only when spwm), the panel construction block (one per family), and the `panel_task` body / spawn. Both families share `RX_BUF`, `FRAME_READY`, and the USB rx task unchanged.
- [x] In `usb-display/client/rust/Cargo.toml`: add `panel-128x64` to features (additive). Bump version to `0.2.1`.
- [x] In `usb-display/client/rust/src/lib.rs`: add a third cfg arm for `panel-128x64` and update the `compile_error!` lists.
- [x] In `usb-display/client/rust/examples/life.rs`: track the last 3 generations' state hashes; when the current matches one of them, increment a stagnant counter; reset on novelty. When stagnant >~30 generations, scatter a handful of random live cells with random hues, then resume.
- [x] Restructure `usb-display/README.md`:
  - Replace the "Panel size" section with "Panel options", listing all three (firmware feature ↔ client feature ↔ family ↔ physical panel description).
  - Add a "Client examples" section listing what each Python pattern (`rainbow`, `gradient`, `solid-*`, `scanline_test`) and Rust example (`life`, `clock`) does.
  - Update build / run examples in the README to use the new firmware feature names.
- [x] Verify firmware builds for `panel-shift-64x64`, `panel-shift-64x32`, and `panel-spwm-128x64` features.
- [x] Verify Rust client builds for `panel-64x64`, `panel-64x32`, and `panel-128x64`.
- [x] Hardware verify (shift): build firmware with `--features panel-shift-64x32`, flash, drive from a host client, confirm display works on the 64×32 panel.
- [x] Hardware verify (spwm): build firmware with `--features panel-spwm-128x64`, flash, drive from a host client at 128×64, confirm display works on the DP3364S panel.

## Log

- During end-of-change hardware testing, discovered that the small 64×32 shift-register panel renders with G and B channels swapped (yellow → magenta, green → blue per a sanity-check pattern in `life`). Probably HUB75-connector wiring on that physical panel. The user finds the resulting palette pleasant for `life`. Pausing this change to open a separate investigation (does the 64×64 panel show the same swap? what's the right fix?), will resume tying this change off afterwards.
- Resumed and closed: the channel swap is being addressed in `channel_swap_investigation.md`. Hardware verifies pass (firmware delivers USB-streamed frames on both panels); colour fidelity on the 64×32 is the separate concern.
- During build, `life`'s stagnation detector and particle injector were added (`HISTORY_K = 6`, 95 % match threshold over 3 consecutive generations, 12 random particles), plus the hue-cycling tunables (`NEWBORN_HUE_SHIFT = 10`, `SURVIVOR_NUDGE = 1`) — settled empirically with the user. A `mean_hue` print and an RGB-sanity yellow→green primer were also added; both stay in the file as live diagnostics.

## Conclusion

Completed. Firmware now drives both shift-register and S-PWM HUB75 panels at three sizes (64×64, 64×32, 128×64), selected via `panel-shift-WxH` / `panel-spwm-WxH` features. Rust client adds `panel-128x64` (size-only naming) so it stays panel-family-agnostic. README explains the family/size split, the firmware↔client feature pairing, and lists what each Python and Rust client example does. `life`'s pixel-stagnation detector with random-particle injection is in place, and its hue cycling is exposed as named constants for easy tweaking.

Hardware verification: both panels render USB-streamed frames. A G/B channel-swap on the 64×32 surfaced during testing — handled by a separate change (`channel_swap_investigation.md`).

Versions: `usb-display-firmware` `0.3.0` (breaking — feature rename), `hub75-client` `0.2.1` (additive). Project does not maintain a changelog; no entry proposed.
