# Firmware brightness control

## Intent

Extend the USB wire protocol so the host can set the panel's hardware brightness (`hub75::ShiftPanel::set_brightness`) at runtime. Currently the firmware uses a hard-coded `DEFAULT_BRIGHTNESS = 64` (out of 255, ~25%) with no way for the host to change it.

Three components touched:
- **`hub75-client`** crate: add a public method (e.g. `Hub75Client::set_brightness(&mut self, level: u8)`) that sends the new command.
- **Wire protocol**: add a "set brightness" command type the firmware recognises.
- **Firmware** (`usb-display/firmware`): on receiving the command, call `panel.set_brightness(level)`. `set_brightness` is safe to call at any time — takes effect on the next address-SM timing cycle without DMA reconfiguration.

Once landed, sysmon2 can:
- Set brightness at startup via a CLI flag (e.g. `--brightness 16`) — most natural use.
- Or change dynamically over time (day/night, ambient, etc.) — same protocol, just more calls.

Recommended starting brightness for sysmon2 use: probably 16–32 (currently 64 is too bright for the at-a-glance use case).

Cadence agile.
