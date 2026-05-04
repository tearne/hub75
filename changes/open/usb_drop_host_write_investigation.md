# USB-drop host write investigation

## Intent

`usb-drop/firmware/` renders new files dropped onto its USB drive, but the "drop the same file again, see it re-render" path is unreliable, and the drive fills up because firmware-side cleanup is invisible to the host's filesystem cache. Three speculative dedup strategies failed in succession during the previous change. We don't actually know what the host sends on the wire during the operations we care about — every fix has been a guess.

This change is a deliberate investigation, not a feature. The output is captured SCSI Write traces and a written understanding of host behaviour, so the eventual redesign rests on observation rather than speculation.

We want to know, at minimum, for both Linux and macOS hosts:

- What sectors does a `cp` of a new file write, and in what order?
- What sectors does a `cp` over an existing file write, when the new content differs from the old?
- What sectors does a `cp` over an existing file write, when the new content is identical to the old?
- What sectors does an `rm` write?
- What does an "Eject"/`umount` look like on the wire?
- Does write_time/write_date in the directory entry actually update on every `cp`?

The findings inform a separate later change that picks a sound ergonomic model (fixed-name overwrite, larger disk, CDC + host watcher, or something else) and a sound dedup signal (or no dedup, if the model doesn't need one). No firmware behaviour change ships from this investigation — the firmware stays on the snapshot+mtime dedup we have today, with debug instrumentation added temporarily and then removed.

## Approach

### Instrumentation

Add `defmt::info!` logging to the SCSI Write completion handler in `usb-drop/firmware/src/main.rs`, emitting `lba` and `len` for every completed Write transaction. This is the only event we genuinely don't have visibility into; SCSI Reads are noisier and not load-bearing for the questions we're asking.

The instrumentation is local to this investigation and is reverted at the end — the firmware on `main` stays unchanged once findings are captured. We won't bump the package version.

### Capture method

Flash with the debug probe attached (`cargo run --release` from `usb-drop/firmware/` — Workflow A from `FLASHING.md`), so `defmt-rtt` streams logs over SWD. Redirect probe-rs output to a file per scenario, e.g. `usb-drop/investigation/linux-cp-new.log`. One log per scenario keeps traces interpretable.

### Scenarios

A fixed list, run in this order against a freshly-mounted drive each time (eject + replug between scenarios so the host's cache doesn't blur scenario boundaries):

1. **First cp of a new file.** `cp solid.H75 /mnt/HUB75DROP/`.
2. **cp over existing file, different content.** `cp solid.H75 /mnt/HUB75DROP/`, then `cp blocks.H75 /mnt/HUB75DROP/solid.H75` (same destination name, different source bytes).
3. **cp over existing file, identical content.** `cp solid.H75 /mnt/HUB75DROP/`, then `cp solid.H75 /mnt/HUB75DROP/` again.
4. **rm of an existing file.** `cp solid.H75 /mnt/HUB75DROP/`, then `rm /mnt/HUB75DROP/solid.H75`.
5. **Eject / `umount`.** `cp solid.H75 /mnt/HUB75DROP/`, then `sync && eject` (or `umount`).
6. **Multiple-file accumulation.** `cp` of three differently-named files in succession, no eject between.

`sync` after each `cp` to force flush before reading the trace.

### Hosts

Linux only. Terminal `cp` only — file-manager drag-drop adds a variable and isn't where the previous change struggled. macOS and drag-drop can be revisited if the redesign needs them.

### Deliverable

A single markdown writeup at `usb-drop/investigation/findings.md` summarising:

- Per scenario: which sectors are written and in what order (FAT region vs root-dir region vs data region — annotated against the BPB so the writeup is self-contained).
- Whether `write_time`/`write_date` in the directory entry actually changes between drops.
- Any surprises (e.g. host writing identical content vs eliding it; multi-FAT propagation; sector access ordering).
- A short "implications" section per question from the Intent — answers, not designs.

The raw `.log` files are kept in `usb-drop/investigation/` alongside the writeup, so the analysis is reproducible.

### Cleanup

Once `findings.md` is written, revert the instrumentation in `main.rs`. The `.log` files and `findings.md` stay in the tree as durable artefacts for the redesign change.

## Plan

- [ ] Add `defmt::info!("scsi: write lba={} len={}", lba, len)` to the SCSI Write completion branch in `usb-drop/firmware/src/main.rs` (the `state.storage_offset == (len * BLOCK_SIZE) as usize` arm).
- [ ] Build verify (`cargo build --release` from `usb-drop/firmware/`).
- [ ] Flash via probe with `cargo run --release` from `usb-drop/firmware/` so `defmt-rtt` streams over SWD.
- [ ] Capture scenario 1 (first cp of a new file) → `usb-drop/investigation/01-cp-new.log`. Eject + replug between scenarios.
- [ ] Capture scenario 2 (cp over existing, different content) → `02-cp-overwrite-different.log`.
- [ ] Capture scenario 3 (cp over existing, identical content) → `03-cp-overwrite-identical.log`.
- [ ] Capture scenario 4 (rm of an existing file) → `04-rm.log`.
- [ ] Capture scenario 5 (eject / `umount`) → `05-eject.log`.
- [ ] Capture scenario 6 (three differently-named cps with no eject) → `06-multi-accumulate.log`.
- [ ] Annotate each log against the BPB: classify each `(lba, len)` as FAT region / root-dir region / data region (with the cluster number for data writes). The disk image's BPB is fixed (`build.rs` formats it once); compute the regions once and reuse.
- [ ] Write `usb-drop/investigation/findings.md`: per-scenario summary of writes, whether write_time/write_date changes between drops, surprises, and an "implications" section answering each question listed in Intent.
- [ ] Revert the `defmt::info!` instrumentation in `main.rs`.
- [ ] Build verify after revert.

