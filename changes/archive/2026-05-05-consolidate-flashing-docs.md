# Consolidate flashing docs

**Mode:** Formal

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

## Approach

### Target file

Keep `SETUP.md` as the single consolidated doc; delete `FLASHING.md`. `SETUP.md` is already the top-level entry point named in `README.md`, and "setup" is the broader of the two terms.

### Folding the third workflow

The current "build-once-share-ELF" path is just the `picotool load` half of the BOOTSEL workflow. Fold it in as a short subsection under the BOOTSEL path rather than promoting it to a top-level option — it isn't a distinct toolchain choice.

### Per-path sections own their setup

Each path's section carries its own install steps and udev rules (probe-rs install + probe udev under the probe path; picotool install + RP2350 BOOTSEL udev under the BOOTSEL path). The Rust toolchain step stays shared up front since both paths need it.

### Section order

BOOTSEL path first, probe path second — lower barrier for a new reader who hasn't yet acquired a probe.

### Inbound reference updates

Five README files link to `FLASHING.md` (root, `usb-serial`, `learning-examples`, `hub75`, `sysmon`). All become pointers to `SETUP.md`, deep-linked to the relevant section anchor when the context names a specific path (e.g. "for BOOTSEL + picotool, see ...").

## Plan

- [x] Rewrite `SETUP.md`: hardware list, shared Rust toolchain step, paths-at-a-glance comparison table, BOOTSEL path section (install + udev + usage + pre-built ELF aside + `elf2uf2-rs` warning), probe path section (install + udev + usage), path convention note at end
- [x] Delete `FLASHING.md`
- [x] Update `README.md`: collapse the two bullets into one pointer to `SETUP.md`
- [x] Update `usb-serial/README.md`: replace `FLASHING.md` references with `SETUP.md` (deep-linked where path-specific)
- [x] Update `learning-examples/README.md`: same
- [x] Update `hub75/README.md`: same
- [x] Update `sysmon/README.md`: same

## Conclusion

Completed. Beyond the consolidation itself, the work caught two factual errors inherited from the old docs (overclaimed `.cargo/config.toml` coverage and a non-existent "workspace root" `target/`). Those motivated a follow-up change, `audit-markdown-accuracy.md`, to sweep the rest of the project's markdown for similar drift.

## Log

- New `SETUP.md` section anchors used by deep links: `#bootsel--picotool` and `#debug-probe--probe-rs` (GitHub-style, ampersands stripped, double-dash from " + ").
- Renamed those headings to plain words ("Flashing via BOOTSEL", "Flashing with a debug probe") on user feedback that headings should just use words; deep links updated to `#flashing-via-bootsel`. Then unified preposition to "via" for both.
- Original `SETUP.md` claimed all per-crate `.cargo/config.toml` files set the build target — actually only the embedded crates have one (host-side `sysmon`, `usb-serial/client/rust` don't). Reworded to "embedded crates' `.cargo/config.toml`".
- Inherited "Path convention" section from old `FLASHING.md` was wrong: there is no Cargo workspace and each crate has its own `target/`. The chained one-liners would not have found their ELF. Section deleted; BOOTSEL examples rewritten so `cargo build` and `picotool load` both run inside the crate dir using a crate-relative `target/...` path. Same fix applied to `sysmon/README.md`.
