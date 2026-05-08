# Consider panel-identity convention for `usb-drop` firmware

## Intent

`flash_targets_specific_panel` establishes a panel-identity convention for the `usb-serial` firmware: each panel advertises a unique USB `serial_number` (default: last 8 hex chars of RP2350 chip ID; override: `PANEL_NAME` env var). `usb-drop` firmware was kept out of scope to keep that change focused.

Decide whether `usb-drop` should adopt the same convention, a divergent one, or none. Considerations: does anyone juggle multiple `usb-drop` boards; does `usb-drop` have its own host-side discovery story; does flash-time disambiguation matter for it.
