# Consolidate flashing docs

## Intent

`SETUP.md` and `FLASHING.md` currently overlap and split the toolchain-and-flashing story across two files. The reader has to bounce between them to figure out which combination of tools they need. Consolidate into a single document — proposed name `SETUP.md` (already a top-level entry point) — restructured around **the choice the reader actually has to make**: which flashing path to use.

New shape:

1. Hardware list.
2. The flashing options at a glance — a short table comparing the two real paths (debug probe + `probe-rs`, or BOOTSEL + `picotool`) with pros/cons. Up front so the reader picks before installing anything.
3. Per-path installation: tools, udev rules, anything specific.
4. Per-path day-to-day flashing usage.
5. Path convention note (workspace `target/` location) and the BOOTSEL-pre-built-ELF aside.

Important pre-existing knowledge to preserve from the current `FLASHING.md`: **`elf2uf2-rs` does not work for RP2350** (wrong UF2 family ID). Earlier in this conversation I suggested it as an option — wrong. The consolidated doc must keep that warning prominent so it doesn't get suggested again.

`README.md` references both files at present; one of those references will become a single pointer to the consolidated doc.
