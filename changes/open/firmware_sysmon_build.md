# Dedicated firmware build for sysmon

**Mode:** Formal

## Intent

The firmware currently embeds its USB serial number via the `PANEL_NAME` env var at build time. Sysmon opens its panel by serial `"sysmon"`, so any rebuild that omits `PANEL_NAME=sysmon` produces firmware that sysmon can't find — a footgun that has just bitten in practice.

Provide a more ergonomic way to build the sysmon firmware so this can't be accidentally dropped: e.g. a `cargo build` alias, a feature flag, a wrapper script, or a per-product Cargo profile. Decide on the cheapest approach that makes the right command obvious and the wrong command harder.
