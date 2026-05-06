# Audit markdown accuracy

## Intent

The flashing-docs consolidation surfaced two factual errors inherited from older docs: a claim that all crates' `.cargo/config.toml` set the build target (only embedded ones do), and a "Path convention" section premised on a Cargo workspace that doesn't exist. Both had survived for some time because nobody read with a sceptical eye.

Sweep every project markdown file and verify its claims against the current code. In scope:

- `README.md`, `SETUP.md` (top-level)
- `hub75/README.md`, `usb-serial/README.md`, `learning-examples/README.md`, `sysmon/README.md`
- `sysmon/map.md`
- `learning-examples/reference/HUB75_DP3364S_RP2350_NOTES.md`, `learning-examples/reference/DP3364S/README.md`

Out of scope: `CLAUDE.md` (process meta), files under `changes/`.

For each file, check: file/path references, command examples, feature flags, crate names, version numbers, hardware claims, protocol details, links (internal anchors and cross-file). Fix what's wrong; flag anything ambiguous to the user rather than guessing.
