# sysmon replacement

## Intent

Replace the original `sysmon` crate with `sysmon2`, renamed to `sysmon`. The new code is the production version going forward; the old code's distinctive features (time-compressed splat rendering, RAM streaks, etc.) are no longer wanted. The deb package must continue to build and the deployed binary must upgrade cleanly on the Pi via `apt`/`dpkg`.

Bump version from `0.1.1` (last sysmon release) to **0.2.0** — signals a substantial change while staying within the 0.x lineage of an early-stage end-user app. Major-version bump (1.0.0) feels heavier than the situation warrants.

The current `sysmon` directory (`usb-display/client/sysmon/`) and `sysmon2` directory (`usb-display/client/sysmon2/`) consolidate into a single directory at `usb-display/client/sysmon/` containing what's currently in `sysmon2`, with deb packaging carried over from the old `sysmon`. After this change there is no more `sysmon2` directory.

## Approach

### Replace, don't merge

The old `sysmon` source is discarded outright — every file under `sysmon/src/` is replaced with the corresponding file from `sysmon2/src/`. No code is salvaged; the new code is the production version. Nothing in the deb packaging or build script depends on the old source layout.

### Preserve packaging shell

`sysmon/Cargo.toml` keeps its `[package.metadata.deb]` block, `debian/` directory (postinst, postrm, sysmon.service), `build.sh` and `README.md` untouched in shape. The `[package]` block is updated:

- `version`: `0.1.1` → `0.2.0`
- `description` shortened to drop "first-principles rebuild" wording from sysmon2's Cargo.toml — the new version stands on its own.

The binary name stays `sysmon` (driven by `[package].name`), so the systemd unit, `/usr/bin/sysmon` install path, and `apt`/`dpkg` upgrade story all continue to work unchanged.

### Workspace exclude

The repo's top-level `Cargo.toml` excludes `usb-display/client/sysmon` and `usb-display/client/sysmon2`. After deletion of `sysmon2`, drop that entry from the exclude list.

### Map relocates

`sysmon2/map.md` becomes `sysmon/map.md`. Internal references to "sysmon2" in headings and prose update to "sysmon". The map content survives the move intact — what it describes *is* the new sysmon.

### Eyeball, then deploy

Verify `cargo build --release` in the new `sysmon/`. Build the deb with `./build.sh`. The user installs it on the Pi (`sudo apt install ./...deb`) and confirms upgrade works cleanly.

## Plan

- [x] Delete every file under `usb-display/client/sysmon/src/` and copy `usb-display/client/sysmon2/src/*` in its place.
- [x] In the moved source files, replace `sysmon2` → `sysmon`: `src/main.rs:1` (doc comment) and `src/main.rs:54` (println), `src/projection.rs:89` (eprintln). Verify no other `sysmon2` mentions remain in `src/` after the copy.
- [x] In `usb-display/client/sysmon/Cargo.toml`, bump `version` to `0.2.0` and tighten the `description` field.
- [x] Move `usb-display/client/sysmon2/map.md` to `usb-display/client/sysmon/map.md`. Search-and-replace `sysmon2` → `sysmon` in that file (headings, anchors, prose).
- [x] Delete the `usb-display/client/sysmon2/` directory (including `Cargo.toml`, `Cargo.lock`, `target/`, etc.).
- [x] In `Cargo.toml` at the repo root, remove `"usb-display/client/sysmon2"` from the workspace `exclude` list.
- [x] `cargo build --release` from inside `usb-display/client/sysmon/` to confirm the consolidated crate compiles.
- [x] `./build.sh` from inside `usb-display/client/sysmon/` to produce the new deb. Capture the path so the user can install it.

## Conclusion

Replacement landed cleanly. The deb at `target/debian/sysmon_0.2.0-1_arm64.deb` upgraded the deployed binary on the Pi without incident.

Beyond the mechanical rename, the map's root-node prose was negotiated: the original "fresh take ... built from first principles alongside the existing sysmon" framing was replaced with a present-tense description that doesn't appeal to history.

Map debt unchanged elsewhere: the `Row Rendering` and (possibly) `Bands` nodes still describe behaviour from before the recent dot-hue and three-bands changes. These predate this change and remain pending catch-up.