# Flatten repo layout

**Mode:** Formal

## Intent

Sysmon currently lives at `usb-serial/client/sysmon/` — buried two directories deep alongside the Rust client library and beside the Pico firmware that talks to it. Its location has become misleading: sysmon is a host-side application that *consumes* the USB-display stack, not a piece of it. Move it to its own top-level crate at the repo root (e.g. `./sysmon/`) so the layout matches its role as a standalone application, and so the path is easier to navigate, document, and refer to from the outside.

## Approach

### Target layout

```
hub75/                              (repo root — no Cargo.toml)
├ README.md, FLASHING.md, SETUP.md
├ changes/
├ hub75/                             (panel driver lib, unchanged)
├ learning-examples/                 (unchanged)
├ usb-serial/
│ ├ README.md                        (protocol-level, unchanged)
│ ├ firmware/                        (unchanged)
│ └ client/
│   ├ rust/                          (host lib, unchanged)
│   └ python/                        (host scripts, unchanged)
├ usb-drop/                          (unchanged)
└ sysmon/                            (was usb-serial/client/sysmon — promoted)
```

### Drop root workspace

Delete the top-level `Cargo.toml` and don't replace it. Each crate becomes self-contained: its own `Cargo.toml`, `Cargo.lock`, and local `target/`. Reason: the root workspace today exists only to bundle the rp235x crates while excluding host crates; once each crate stands alone, the workspace's job is gone, and `target/` becomes one `..` closer from any crate dir.

### Promote sysmon to top level

Move `usb-serial/client/sysmon/` to `./sysmon/`. Reason (the original Intent): sysmon is a host-side consumer of the protocol, not part of the protocol stack itself.

### `usb-serial/client/` keeps `rust/` and `python/` siblings

The `rust/` directory stays nested under `client/` because `python/` (host scripts) is its sibling — collapsing `rust/` into `client/` would awkwardly mix Rust crate files with a `python/` dir. So `usb-serial/client/rust/` and `usb-serial/client/python/` are unchanged.

### Path dependencies after the move

- `sysmon/Cargo.toml`: `hub75-client = { path = "../usb-serial/client/rust" }` (was `../rust`).
- No other path deps change.

### History-preserving moves

All relocations via `git mv`. No source restructuring inside the moved crates.

### No version bumps

This change rearranges files only — no source, behaviour, or API changes. Versions stay where they are.

### Stale `usb-display/` removed

`usb-display/client/sysmon/target/` is the only remaining content under `usb-display/`, left over from a prior move. Delete the whole `usb-display/` tree as part of this change.

### Preserve release profile in firmware crates

The root workspace's `[profile.release] opt-level = "s", debug = true, lto = true` currently applies to all four member crates. Replicate it into each of `hub75/Cargo.toml`, `learning-examples/Cargo.toml`, `usb-serial/firmware/Cargo.toml`, `usb-drop/firmware/Cargo.toml`. Reason: embedded crates need `opt-level = "s"` and `lto = true` to fit binary size; default release profile would silently regress them.

### Delete root `Cargo.lock`

Remove the root `Cargo.lock` alongside the root `Cargo.toml` — it becomes a dangling artefact otherwise. Each remaining crate generates its own lockfile on first build.

### Accepted trade-offs

- One shared lockfile becomes four. Dep version drift across crates is now possible; `cargo update` in one no longer propagates.
- Editor "open repo root, see all crates as one project" view is gone. rust-analyzer will discover each `Cargo.toml` independently.

## Reference sweep

Files that point at the old layout and need updating during Build:

- `Cargo.toml` (root) — delete entirely.
- `README.md` (root) — replace the Structure table with a file-tree diagram (in the same style as the one in this Approach) and refresh the surrounding text. Keep total length close to current — do not expand significantly.
- `FLASHING.md` — examples reference `usb-serial/firmware/`; still valid, no edit needed (the firmware doesn't move).
- `usb-serial/README.md` line 73 (`### Sysmon — client/sysmon/`), line 75 (link to `client/sysmon/README.md`) — replace the sysmon section with a one-line pointer to `../sysmon/`. The `client/rust/` and `client/python/` references are unchanged.
- `usb-serial/client/sysmon/Cargo.toml` — path dep `../rust` → `../usb-serial/client/rust`. (Re-anchored from the new top-level `sysmon/` location.)
- `usb-serial/client/sysmon/README.md` line 4 (`../../`), lines 18–19 (build instructions still valid — just verify), line 22 (`../../../SETUP.md`, `../../../FLASHING.md`), line 26 (`From inside this directory (usb-serial/client/sysmon/)`) — re-anchor relative links from the new top-level `sysmon/` location.
- `hub75/src/shift/pack.rs:13` — comment `/// usb-serial/firmware implementation` is still accurate, no edit needed.
- `usb-display/` — delete the whole tree (only stale `target/` remains).

**Open changes referencing the old layout** (informational — not edited as part of this change; flag in the Conclusion):

- `changes/open/usb_drop_host_write_investigation.md` — references `usb-drop/firmware/` (still valid after this change, no edit needed).
- `changes/open/life_drop_rgb_primer.md` — references `usb-display/client/rust/examples/life.rs` (already stale; correct path is `usb-serial/client/rust/examples/life.rs`, which is unchanged by this restructure).

## Plan

- [x] `git mv usb-serial/client/sysmon ./sysmon`.
- [x] Update path dep in `sysmon/Cargo.toml`: `hub75-client = { path = "../rust" }` → `path = "../usb-serial/client/rust"`.
- [x] Delete root `Cargo.toml` and root `Cargo.lock`.
- [x] Add `[profile.release] opt-level = "s", debug = true, lto = true` to `hub75/Cargo.toml`.
- [x] Same to `learning-examples/Cargo.toml`.
- [x] Same to `usb-serial/firmware/Cargo.toml`.
- [x] Same to `usb-drop/firmware/Cargo.toml`.
- [x] Replace `README.md` Structure table with a file-tree diagram in the same style as the Approach diagram; refresh prose; keep total length close to current.
- [x] In `usb-serial/README.md`, replace the Sysmon section (lines 73–75) with a one-line pointer to `../sysmon/`.
- [x] In `sysmon/README.md`, re-anchor relative links (`../../`, `../../../SETUP.md`, `../../../FLASHING.md`) and the "From inside this directory (`usb-serial/client/sysmon/`)" line to the new top-level location.
- [x] Delete the `usb-display/` tree.
- [x] Build verify each crate from its own directory: `hub75`, `learning-examples`, `usb-serial/firmware`, `usb-drop/firmware`, `usb-serial/client/rust`, `sysmon`.

## Log

- Title and H1 originally read "Promote sysmon to top-level crate" — the work grew during planning. Renamed file and H1 to `flatten-repo-layout` at Conclusion time.
- `usb-display/` was untracked (the prior change had moved sysmon away but left the empty parent), so `git rm` failed; deleted with `rm -rf` instead. No git history involved.
- Standalone `cargo build` of `usb-serial/client/rust` failed because that crate's pre-existing `.cargo/config.toml` forces `target = "x86_64-unknown-linux-gnu"`, unavailable on this aarch64 host. The comment claimed it "overrides the workspace target", but no root `.cargo/config.toml` ever existed — the override has been silently forcing x86_64 for any direct build of this crate, regardless of host. Deleted the file (and the now-empty `.cargo/` dir) and rebuilt — clean. Hand-folded into this change rather than spinning a separate one.

## Conclusion

Completed. Out-of-scope follow-up to flag: `changes/open/life_drop_rgb_primer.md` references `usb-display/client/rust/examples/life.rs` — already stale before this change (correct path is `usb-serial/client/rust/examples/life.rs`); now also needs a once-over to confirm before that change goes to Build.
