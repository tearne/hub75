# Client should target a specific panel when multiple are attached

## Intent

When more than one HUB75 panel is plugged into the host, the Rust client's `Hub75Client::open_auto()` grabs the first device matching the VID/PID/manufacturer/product strings. There's no way to say "talk to *that* panel". Today, working on a second panel requires shutting down `sysmon` and unplugging the panel it owns — friction that discourages multi-panel work and risks the kind of mishap that motivated `flash_targets_specific_panel`.

Consume the panel-identity convention established by `changes/open/flash_targets_specific_panel.md` and expose it through the client so a caller can select a specific panel. Examples (`life`, `clock`) and `sysmon` should be updatable to target a chosen panel without unplugging others.
