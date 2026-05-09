# `usb-drop` firmware doesn't render dropped files

**Mode:** Explore

## Intent

After flashing the `usb-drop` firmware on a 64x64 shift panel, copying any of the bundled `.H75` files (e.g. `blocks.H75`, `test.H75` — both 64x64, matching the firmware) onto the mounted drive and unmounting does not produce any output on the panel. The panel stays blank from boot. The USB Mass Storage side works (drive mounts, files appear, unmount succeeds) — only the firmware-side render is silent.

Cause unknown. Could be: file write not reaching firmware (MSC commit timing), firmware parse failure, pinout mismatch with this particular panel, render-path bug, or rendering succeeding but panel hardware not driven correctly. Diagnose with `defmt` logs over `probe-rs` (which will require wiring the debug probe to this Pico) so we can see whether the firmware ever sees the file. Discovered while smoke-testing the panel-identity change for `usb-drop` (`2026-05-09-usb_drop_panel_identity_impact.md`); kept out of scope of that change to avoid scope creep.

## Approach

### Investigation, not feature work

Like the panel-corruption investigation: gather evidence, narrow the failure to a stage of the render pipeline, then decide whether to fix here or spin off. The render path already logs at every failure point — `fat: BPB parse failed`, `fat: read_file failed`, `frame: file too large`, `frame: parse error`, `frame: rendered N bytes` — so live `defmt` should reveal the failure within one reproduction.

### Live firmware visibility via probe-rs

Move the debug probe to the `usb-drop` Pico and re-flash through the probe-rs path so logs stream during reproduction. The picotool path used so far is silent on firmware-internal events.

### Failure classes to distinguish

The existing log points map onto distinct failure mechanisms:
- No `WRITE_COMPLETED` ever fires after `cp`/`umount` — host-side MSC write isn't reaching SCSI handler. Look at `process_command` and add probes if needed.
- `fat: BPB parse failed` — the FAT structure on disk is corrupted post-write.
- `fat: read_file failed` — cluster chain walk fails (potential FAT12 indexing issue specific to written files vs the bundled README).
- `frame: parse error` — file reaches firmware but JSON parser rejects it.
- `frame: rendered N bytes` printed but panel still blank — render succeeded; problem is downstream (PIO/DMA, panel wiring, or `set_pixels` not actually committing).

### Outcome

Document the failure stage in this change, then decide: minor fix (e.g. an off-by-one in scan logic) folds into this change; anything bigger spins off.

## Plan

**Topics**

- Wire the debug probe to the `usb-drop` Pico.
- Re-flash via `cargo run --release` from `usb-drop/firmware/`; confirm `defmt` logs stream and show "usb-drop firmware: starting".
- Reproduce: mount the drive, copy `blocks.H75`, unmount. Capture all `defmt` output around the unmount.
- Classify against the failure-class table above.
- If a stage is identified, decide fix-here vs spin-off.
- If no defmt message at all between unmount and panel staying blank — instrument `process_command`'s WRITE handler with a temporary log, re-run.

**Done when**

The failure stage is identified, AND a decision is made on whether the next step lives in this change or a new one.

## Status (paused)

Investigation paused pending hardware that doesn't trip the Pi's USB over-current protection (powered hub or higher-current PSU). Findings so far:

- `WRITE_COMPLETED` fires correctly: temporary instrumentation confirmed the flag is set after each completed Write transaction (lba 39 + FAT/boot-sector writes during cp/umount).
- `scan_and_render` log never reached the host. Either the function isn't being called, or it ran and the log was lost when the probe-rs USB link dropped on the over-current event.
- Apparent panel "render" on umount turned out to be a power-glitch flicker, not a real frame. Confirmed by user — no recognisable image.
- Every cp/umount cycle currently coincides with an over-current event (probe-rs disconnects with `Error in the USB access`). Same family of bus event documented in `2026-05-08-panel_corruption_during_client_transitions.md`.

Resume plan: with capable hardware, re-flash via probe-rs, repeat the reproduction. Either `scan_and_render: invoked` will appear (proving the function runs and the issue is downstream — FAT/JSON/render) or it will not (proving the gating between `WRITE_COMPLETED` and the deadline check is broken). Temporary diagnostic logs were reverted; they can be re-added quickly when resuming.
