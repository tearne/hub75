# USB-file drop-to-render firmware

## Intent

Right now there's exactly one USB-driven display firmware: `usb-display/firmware/`, which exposes a CDC serial endpoint and consumes frames pushed from a host client. That's a developer-friendly path but requires running a host program to feed it.

We want a second firmware variant that takes a much simpler ergonomic path: the Pico mounts as a USB mass-storage drive. The user drags a correctly-formatted file onto that drive in their OS file manager, and the firmware picks up the new file, parses it, and renders the contents on the panel. No CLI, no client-side library — just a file copy.

To make room for a second variant cleanly, **rename the existing `usb-display/` to `usb-serial/`** (the "display" name was always too generic; "serial" describes the USB transport actually being used). The new variant goes in a sibling **`usb-drop/`** (the spike already established `usb-drop/firmware/`).

Initial scope: single-frame files — drop a file, see one image. Future scope (out of this change): multi-frame animations with per-frame duration, repeat / loop / ping-pong options, and probably transitions.

The output of this change: existing CDC firmware lives at `usb-serial/`, all references updated; `usb-drop/firmware/` (currently a spike that just shows up as a drive with a README) is extended to detect dropped JSON frame files and render them on the panel via `hub75`.

## Approach

### Renames

`usb-display/` → `usb-serial/` (whole directory move). The Rust host crate `hub75-client` keeps its name — it's still the host-side library for the CDC protocol. Workspace `Cargo.toml` `members` updated. References in READMEs updated.

### What the spike already settled

- USB stack: `usb-device` + `usbd-storage` v2 on `rp235x-hal`. Sync. No embassy in this firmware.
- Storage backend: 64 KB static `[u8]` RAM disk. Block reads/writes go straight to it.
- Pre-formatted FAT12 disk image: generated at build-time by `build.rs` using `fatfs` (host-side `std`), `include_bytes!`'d into the firmware, copied into the RAM disk on boot.

### File-event detection: minimal in-firmware FAT12 reader

`fatfs` is unusable in firmware (broken `no_std` story on stable). Instead, hand-roll a minimal FAT12 root-directory reader (~100–150 lines). Just enough to enumerate root entries, find files matching `*.hub75`, and read their contents (cluster-chain walk).

Trigger: after every host-side block write (we know writes happen via the `usbd-storage` SCSI Write callback), set a "directory dirty" flag. In the main loop, when the flag is set and ~50 ms has passed without further writes (settle delay so we don't read mid-write), re-scan the root directory. New file matching the pattern → read + parse + render. Same file modified → re-read + re-render.

### File extension: `.H75`

`.HUB75` would be a 5-char extension, which doesn't fit FAT12's 8.3 short-name field. We could parse LFN (long-filename) entries to recover the user's typed name, but it's a couple hundred lines of UCS-2 + checksum logic for no functional gain. `.H75` is 8.3-clean and unambiguous.

### File format — human-readable JSON

```json
{
  "version": 1,
  "width": 64,
  "height": 32,
  "pixels": [
    [255,   0,   0],
    [  0, 255,   0],
    ...
  ]
}
```

Parsed by a hand-rolled streaming parser in `frame.rs`. We initially planned to use `serde-json-core` but a `[[u8; 3]; 4096]` field would put a ~12 KB struct on the stack during deserialisation (default cortex-m-rt stack is 16 KB), and the streaming alternative is ~150 lines. Files matching `*.H75` (any name, that extension; contents JSON) trigger a render. Decode error → log via `defmt`, ignore the file, leave the previous frame on the panel. Future format versions bump the `version` field for animations.

### Rendering: vendor a minimal sync panel driver inside `usb-drop/firmware/`

`usb-drop/firmware/` is sync `rp235x-hal`; `hub75` is async `embassy-rp`. Rather than blur the boundaries, vendor a small sync shift-register panel driver inside `usb-drop/firmware/`, sourced from `learning-examples/shift_3_dual_pio.rs` (already rp235x-hal-based, autonomous PIO+DMA, single-file). No `hub75` dependency from `usb-drop`. Duplication is contained; future refactor can DRY when there's a real burden.

### Initial panel scope: 64×64 shift-register only

Single supported panel size in this change: 64×64 shift-register. The JSON `width` / `height` fields are validated against the firmware's compile-time configured panel; mismatched sizes are rejected with a `defmt` warning.

Other sizes (64×32 shift, 128×64 S-PWM) are follow-up changes once the file-drop / parse / render pipeline is proven.

### Boot behaviour

Panel is black until a valid file appears. No persistence of the last-rendered frame across power cycles in this iteration (RAM-backed disk is reset to the build-time image at every boot).

## Plan

- [x] Rename `usb-display/` → `usb-serial/`. Update workspace `Cargo.toml` `members`. Sweep references in: root `README.md`, `usb-display/README.md` (now `usb-serial/README.md`), `usb-display/firmware/Cargo.toml`'s package name (`usb-display-firmware` → `usb-serial-firmware`), the workspace member list in `Cargo.toml`. Bump firmware version (rename = breaking) to `0.4.0`.
- [x] Add `pio` (assembler; pinned to `0.2` to match `rp235x-hal 0.3`'s transitive dep) to `usb-drop/firmware/Cargo.toml`. `serde` / `serde-json-core` were dropped — see "File format" above.
- [x] Vendor a minimal sync 64×64 shift-register panel driver in `usb-drop/firmware/src/panel.rs`, sourced from `learning-examples/examples/shift_3_dual_pio.rs`. Exposes `Panel::init(...)` (configures GPIO 0..13 to PIO0, releases DMA from reset, builds both PIO programs and the four DMA channels, starts autonomous scan) and `panel.set_pixels(&[[Rgb; 64]; 64])` (packs into the inactive bitplane buffer and swaps the active pointer).
- [x] Implement a minimal FAT12 root-directory reader in `usb-drop/firmware/src/fat.rs`. Parses the BPB, enumerates 32-byte root directory entries (skipping LFN/volume/subdir/deleted), walks 12-bit-packed FAT chains, reads file contents into a buffer. Caller filters via `DirEntry::is_h75()`.
- [x] Define the JSON frame format (`{ version, width, height, pixels: [[r,g,b], ...] }`) and parse it with a hand-rolled streaming parser in `frame.rs`. Validates `version == 1`, `width`/`height` match the panel's compile-time dimensions, exact pixel count, channel range; on any failure → `defmt::warn!` + ignore.
- [x] Wire it together in `main.rs`: the SCSI Write handler sets a `WRITE_COMPLETED` atomic flag when a transaction finishes; the main loop converts that into (or extends) a 50 ms settle deadline using `TIMER0`; once the deadline lapses with no further writes, scans the root directory, picks the first `*.H75` whose `(start_cluster, size)` differs from the last rendered file, reads + parses + `panel.set_pixels(...)`.
- [x] Update the `README.TXT` content in `build.rs` to actually explain the file format and give an example. Bumped disk image to 256 KB to fit the 64×64 JSON worst case.
- [x] Build verify.
- [ ] Hardware verify: flash `usb-drop/firmware/`, plug into a host, drop a hand-crafted `*.H75` JSON file with a small visible pattern, see it render on the panel.

## Unresolved

(none — Path A confirmed, panel scope set to 64×64 shift, rest is implementation.)

## Conclusion

Partially implemented and archived. What shipped:

- Rename `usb-display/` → `usb-serial/` (workspace member, package name, READMEs); `usb-serial-firmware` bumped to `0.4.0`.
- `usb-drop/firmware/` (`0.1.0`): USB MSC drive, FAT12 RAM disk, `*.H75` JSON frame format, hand-rolled FAT12 reader, sync 64×64 shift-register panel driver vendored from `learning-examples/shift_3_dual_pio.rs`.
- New-file drops render reliably; the basic drop-to-render pipeline is proven on hardware.

What didn't, and why it's deferred to a follow-up change rather than further iteration here: re-drops of an existing file are unreliable across three speculative dedup strategies, and the drive fills up because the host's vfat cache makes firmware-side cleanup ineffective. The Feedback section captures the constraints and the recommended first step (instrument SCSI Writes before any new design).

Build state: `usb-drop/firmware/` on the snapshot+mtime dedup; builds clean.

## Feedback

**Status:** partially implemented. New-file drops render reliably; re-drops of an existing file (whether by same name or fresh content) are unreliable across three dedup strategies. The drive accumulates files indefinitely, eventually filling.

**Notes:**

The Approach assumed two things that turned out not to hold:

1. *That re-drops of a file would be detectable from FAT12 directory state alone.* Tried (cluster, size), then a SCSI dirty-sector bitmap, then (cluster, size, mtime). All have failure modes — the most recent (mtime) is unreliable even with deliberate >2-second pacing between `cp` commands. We don't actually know what the host writes on the wire during a `cp` overwrite; every fix has been speculation. **Before redesigning, instrument the SCSI Write completion handler with `defmt` logs (`lba` and `len` per completed transaction) and capture traces of representative `cp` flows.** That data should drive the new Approach.

2. *That the firmware could manage disk space.* It can't: the host's vfat layer caches FAT + root directory on mount and only re-reads on remount/eject. Firmware-side modifications (deleting dir entries, freeing FAT clusters) don't propagate to the host. The host keeps allocating from its cached free-list, so accumulated files fill the drive. The fix isn't a different firmware-side delete — it's a different ergonomic model. Candidates worth weighing during replanning:

   - Fixed destination filename (`HUB75DROP/current.H75`); user always overwrites the same file. Avoids fill-up. Less ergonomic than drag-drop a uniquely-named file.
   - Larger RAM disk (4 MB?) — but RP2350 SRAM is 520 KB and the firmware already uses ~370 KB across DISK, FILE_BUF, FRAME, and stack; meaningful expansion isn't free.
   - User-managed cleanup (`rm` on the host) when full.
   - A non-MSC transport for the "drop a file" experience (CDC + a watcher on the host?). Bigger redesign.

The Approach needs re-opening, not just a Plan revision — both the dedup design and the disk-management model are Approach-level decisions.

**Documentation impact:** none anticipated; this change is contained within `usb-drop/`. The rename of `usb-display/ → usb-serial/` already shipped and was already reflected in READMEs.

## Log

- All build-side plan tasks complete; firmware compiles clean (3 `static_mut_refs` warnings carried over from the vendored panel driver — the source PIO/DMA pattern needs raw pointers, not actionable here).
- `pio` pinned to `0.2` mid-build because `rp235x-hal 0.3` transitively depends on `pio 0.2` and v0.3's `irq` signature/`Program` type don't match. Don't bump back to 0.3 without a HAL upgrade.
- Switched off `serde-json-core` after writing the `frame.rs` skeleton: a `[[u8; 3]; 4096]` field would land on the stack during deserialise (~12 KB), and the cortex-m-rt default stack is 16 KB. Replaced with a streaming parser (~150 lines) — no stack-resident pixel buffer.
- Disk image bumped from 64 KB to 256 KB so a verbose 64×64 JSON frame fits with headroom; `FILE_BUF` in firmware sized at 96 KB.
- File extension is `.H75` not `.hub75` — 8.3 short-name field is 3 chars; LFN parsing ducked.
- **Resume point:** only "Hardware verify" is unticked. Flash `usb-drop/firmware/`, mount the drive on a host, drop a hand-crafted `*.H75` file matching the format in `build.rs`'s `README_CONTENTS`, and confirm a render appears on the panel within ~50 ms of the file copy completing. No code changes expected — if anything fails, log it here and decide whether it's a build-mode adjustment or a replan.
- **2026-04-30 hardware verify, dedup bug:** first file dropped rendered correctly; subsequent files (different names) did not. Root cause in `scan_and_render`: the LAST_RENDERED match branch did `return` (aborting the whole scan) instead of `continue` (skipping just that entry), so a previously-rendered file appearing earlier in root-dir order masked any newer .H75 behind it. Patched to `continue`. The deeper limitation — single-slot LAST_RENDERED can't represent "all files seen so far," so a third file may cause the first to re-render — is a known gap, not addressed here. User opted for the minimal fix; revisit if it bites.
- Added `usb-drop/gen.py` as a small generator for canonical test frames (gradient / solid / origin / blocks). Convenient for hardware verify and likely useful for future iterations.
- **2026-04-30 hardware verify, dedup ping-pong:** `return → continue` patch unblocked two-file drops, but with three (solid → blocks → origin) the panel ping-ponged between the first two and never reached origin. Root cause: single-slot `LAST_RENDERED` can't represent "all .H75 files seen so far," so each scan finds *some* earlier file that's no longer the recorded last and re-renders it. Replaced `LAST_RENDERED: Option<...>` with `KNOWN: [Option<FileId>; 16]`, a snapshot of the .H75 set observed at the most recent scan; on each scan, render the first .H75 not in `KNOWN` and then refresh `KNOWN` from the current root directory. Bound is 16 (disk fits ~4–5 files at most). Deletions naturally drop entries out of the next snapshot, so deleting then re-adding a file re-triggers a render.
- **2026-04-30 hardware verify, in-place rewrite:** snapshot-of-FileId dedup still missed the case "drop solid.H75 again when solid.H75 already exists" — overwrite usually keeps the same start cluster, the same size, and (depending on host) the same mtime, so identity is unchanged. Right signal is the SCSI Write LBAs: if the host wrote bytes since the last scan, *something* was dropped, regardless of metadata. Replaced KNOWN with a 64-byte `DIRTY` sector bitmap, set in the SCSI Write completion handler and cleared at the end of each scan. `scan_and_render` now renders the first .H75 whose first data cluster intersects DIRTY. Promoted `Bpb::cluster_bytes` and `cluster_data_offset` from private to `pub` so `main.rs` can compute sector ranges. Snapshot logic gone — DIRTY is the single source of truth for "this file was just written."
- **2026-04-30 hardware verify, dirty-bitmap also failed:** repeated `cp` of the same file (terminal, with and without `sync`) didn't trigger a render under the dirty-bitmap firmware. Hypothesis: Linux elides the data-sector writes for identical content (only metadata writes — FAT, dir entry mtime — reach the device), so the file's data clusters never appear dirty. User pivoted to a much simpler design: **after rendering, mark the directory entry as deleted** (FAT12 `0xE5` sentinel in the name's first byte), so every drop appears as a fresh creation regardless of host caching. Removed the dirty-bitmap entirely; `scan_and_render` now just finds the first `.H75`, renders, and deletes its dir entry via new `fat::delete_entry_by_cluster`. Cluster chain is left allocated for now (256 KB disk has headroom; host re-mount reclaims). Side effect: dropped files disappear from the host's view of the drive after rendering — acceptable for a drop-to-display device, would be wrong for a drop-to-store one.
- **2026-04-30 hardware verify, delete-after-render also failed:** new copies stopped working after a few drops with "drive is full" in the host's file explorer; even rendered files still appeared on the drive. Two intertwined issues: (a) I left the cluster chain allocated, leaking ~50 KB per drop, (b) more fundamentally — the host's vfat layer caches the FAT and root directory on mount and only re-reads on remount/eject. Modifying device-side bytes from the firmware doesn't propagate to the host's view. The host keeps allocating from its cached free-list, so the firmware-side delete is invisible to the host. Concept "writes in the firmware tell the host anything" is broken; the host owns the filesystem state. Backed out the delete-on-render change. Returned to the snapshot dedup, but extended `FileId` to include the directory entry's `write_time_date` (offsets 22-25). Rationale: cp updates mtime to "now" by default on the destination, which Linux vfat encodes in the dir entry, which the host writes back to the device. The dir-entry sector therefore changes between drops even when content is identical — and FileId reflects that. Added `write_time_date: u32` to `fat::DirEntry`. Removed `fat::delete_entry_by_cluster`. **Important for testing:** because the previous firmware leaked clusters, the host's cached FAT may still show "drive full" even after reflashing — power-cycle + eject/replug to force the host to re-mount and re-read.
- **2026-04-30 mtime snapshot: unreliable, blocker.** First drops of any file work. Repeat drops of the same file are unreliable — sometimes re-render, sometimes don't, even with `sleep 2` between `cp` commands (which should rule out FAT12 2-second mtime granularity). Tested same-target-name workflow (`cp X /mnt/.../current.H75`); same unreliability. Three dedup strategies attempted in sequence — snapshot-of-(cluster,size), dirty-sector bitmap, snapshot-with-(cluster,size,mtime) — none reliable. Underlying issue: we don't know what the host is actually sending on the SCSI wire during a `cp` over an existing file. Every fix has been speculation. Two further fundamental constraints became clear during this session: (a) firmware-side modification of FS structures is invisible to the host's vfat cache, so we can't safely delete or recycle anything; (b) the disk fills up with accumulated drops because of (a). Path forward needs SCSI Write instrumentation (`defmt` logs of every `lba`+`len` arriving at the Write SCSI handler) before any further design change — without that the next iteration is guessing again. Removing `active.md` and handing back. Build state: `usb-drop/firmware/` currently on the snapshot+mtime dedup; builds clean; `usb-drop/gen.py` is in the tree.
