# README audit and restructure

## Intent

Across the four READMEs (project root, `hub75/`, `learning-examples/`, `usb-display/`) there's a real amount of repetition and a few sources of confusion:

- **Setup / prerequisites duplication.** "Rust + thumbv8m target + probe-rs + udev rules" appears in different forms in the root README and is partially restated in the subdir READMEs.
- **Flashing duplication.** Probe-based and BOOTSEL/`picotool` flashing instructions are spelt out in the root README, again in `learning-examples/`, and obliquely in `usb-display/`. The picotool quirks (don't use `elf2uf2-rs`, install command, flags) belong in one place, referenced from the others.
- **Path-from-workspace-root inconsistency.** Build runs from inside a crate (`cd hub75/`), but the resulting binary is at `target/...` from the *workspace root*. Depending on crate nesting depth, `picotool load ../target/...` vs `../../target/...` is what you type. Easy to get wrong; bad first impression.
- **Stale content in `usb-display/README.md`.** The "Architecture" + "Technical annex" sections describe the pre-`hub75`-crate inline firmware. The implementation and the architectural details now live in `hub75/`; those sections of the usb-display README are stale and misleading.
- **No single source of truth for build workflows.** "Build with probe / build for BOOTSEL / build for sharing an ELF" are scattered across READMEs; a reader can't see the alternatives in one place.

The output of this change: tighter, more hierarchical READMEs that delegate setup and flashing to single-source-of-truth docs (e.g. `SETUP.md`, `FLASHING.md`), reduce the directory-relative-path confusion, and remove stale content. Each subdir README focuses on what's specific to its subdir; shared concerns are pulled up.

## Approach

### Two top-level extracted docs

Pull the cross-cutting "how do I get going" and "how do I flash" content into two new files at the project root:

- **`SETUP.md`** — Rust toolchain + ARM target, probe-rs install, Linux udev rules, hardware list (Interstate 75 W board + supported panels). Read once when starting a new machine.
- **`FLASHING.md`** — three workflows side by side: (1) probe via `cargo run` for fast iteration with logs, (2) BOOTSEL + `picotool` for any board without a probe attached, (3) "build once, share the ELF, flash later". Also covers `picotool` install, the `elf2uf2-rs` warning, and the path conventions (see below).

These two files are the single source of truth. Subdir READMEs link to them.

### Workspace-root-relative paths as the canonical convention

The directory-confusion comes from `target/` living at the workspace root while `cargo build` runs inside a crate. We standardise on **always quoting the picotool path from the workspace root** (e.g. `target/thumbv8m.main-none-eabihf/release/examples/<name>`) and tell users to run `picotool` from the workspace root, regardless of which crate they cd'd into to build. Single absolute-from-root path, works for every crate at any nesting depth.

For users who'd rather build *and* flash without changing directories, `FLASHING.md` shows the chained one-liner pattern:

```sh
(cd hub75 && cargo build --release --example test_pattern --features panel-shift-64x32) && \
  picotool load -v -x -t elf target/thumbv8m.main-none-eabihf/release/examples/test_pattern
```

### Subdir READMEs: subdir-specific only

Each of `hub75/README.md`, `learning-examples/README.md`, `usb-display/README.md` keeps only what's specific to that subdir (its purpose, its examples, its public API or feature set, its protocol). Setup and flashing become "see [`SETUP.md`](../SETUP.md)" / "see [`FLASHING.md`](../FLASHING.md)" links.

### Drop stale `usb-display/` "Technical annex"

The Architecture + Technical annex sections of `usb-display/README.md` describe the pre-migration inline firmware. That code now lives in the `hub75` crate (which has its own architectural reference back to `learning-examples/spwm_3_autonomous`). Delete from `usb-display/README.md`; the firmware README points at `hub75/` for architecture.

### Root README: orientation, not a manual

Trim the root README down to: project blurb, subdir map, links to `SETUP.md` / `FLASHING.md`, acknowledgements. Stop the root from being a one-stop-shop.

## Plan

- [x] Create `SETUP.md` at workspace root: Rust toolchain (+ `thumbv8m.main-none-eabihf`), `probe-rs` install, Linux udev rules (debug probe + RP2350 BOOTSEL), Pimoroni Interstate 75 W board, panel families (link to `hub75/README.md` for the per-panel feature table).
- [x] Create `FLASHING.md` at workspace root: three workflows side-by-side — probe (`cargo run` for fast iteration with `defmt` logs), BOOTSEL + `picotool` (no probe required), and "build once, share the ELF" (the BOOTSEL path with the build and flash steps split). Include `picotool` install, the `elf2uf2-rs` warning, and the workspace-root path convention with worked examples for each crate (`hub75`, `learning-examples`, `usb-display/firmware`).
- [x] Trim project root `README.md` to orientation only: project blurb, subdir map, links to `SETUP.md` and `FLASHING.md`, acknowledgements. Remove the inline prerequisites + flashing sections.
- [x] Trim `learning-examples/README.md`: drop the duplicated probe / BOOTSEL flashing instructions; link to `FLASHING.md`. Keep the terminology table, examples-at-a-glance, and the two-families breakdown.
- [x] Trim `hub75/README.md`: drop the inline link to "project root for BOOTSEL", replace with a `FLASHING.md` link. Update the example invocations to the workspace-root-relative path convention. Keep the panel-families intro, quickstart, `test_pattern`, per-panel features table, usage snippets, tunables, and the architecture pointer.
- [x] Trim `usb-display/README.md`: drop the Architecture and Technical annex sections (stale post-migration). Replace with a one-line "See [`hub75/`](../hub75/) for the panel-driver implementations" pointer. Update flash commands to use `FLASHING.md`'s convention. Keep the panel-options table, client examples, and protocol section.
- [x] Sanity-check all relative links between the four READMEs and the new top-level docs resolve correctly.

## Unresolved

(none — extraction targets, workspace-root-path convention, and stale-content removal are all settled.)

## Log

- Originally planned three flashing workflows (probe / BOOTSEL+build / build-once-share-ELF). The third didn't have meaningfully different commands from the second — same `cargo build`, same `picotool load` — so consolidated to two workflows in `FLASHING.md`, with a short "Flashing a pre-built ELF" subsection under Workflow B for the share-an-ELF case.

## Conclusion

Completed. `SETUP.md` and `FLASHING.md` are the single sources of truth for prerequisites and flashing; the four READMEs link to them rather than duplicating. Workspace-root-relative paths are now the canonical convention in all flashing instructions, removing the `../target/` vs `../../target/` confusion. Stale Architecture / Technical annex sections in `usb-display/README.md` deleted; that README now points at `hub75/` for panel-driver details. Root README is orientation only.
