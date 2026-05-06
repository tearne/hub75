# Review and tidy

**Mode:** Exploration

## Intent

Sweep the project after the recent burst of work (sampling-cost fix, vendor-class USB transport, doc consolidation) and clean up loose ends:

- **Defmt logs in firmware** — keep what's load-bearing for diagnostics, trim the rest. Per-second fps prints are useful when a probe is attached during dev; some others may be noise.
- **Documentation consistency and terseness** — cross-file consistency (terms, paths, version references), removal of repetition, tightening of prose. Includes the markdown-accuracy sweep already queued (see overlap note below).
- **Stale code** — dead branches, unused constants, leftover scaffolding, comments that no longer match the code.
- **Performance opportunities** — low-hanging items visible from the recent profiling work that didn't make it into the targeted changes.
- **Carry-over from prior change Logs** — read the recent archived changes' Log sections (`2026-05-05-consolidate-flashing-docs`, `2026-05-05-reduce-sysmon-sampling-cost`, `2026-05-06-reduce-usb-write-cost`) for anything flagged but not actioned.

Out of scope: non-trivial behaviour changes. Tidying means tidying — if a real refactor or perf fix surfaces, capture it as an aside / new proposal rather than absorbing it here.

## Overlap with `audit-markdown-accuracy`

The queued `audit-markdown-accuracy.md` change is the docs-accuracy slice of the doc work above. Two ways to handle:

a) **Fold it in**: this change subsumes the audit; close `audit-markdown-accuracy.md` (delete it from `open/`, no archive — it never built anything).
b) **Keep separate**: doc-accuracy (factual correctness) stays focused; review-and-tidy handles consistency/terseness/repetition.

Recommend (a) — the same files would be touched in the same sweep, and splitting along factual vs. tidiness lines is fragile.

## Approach

Done up front: a sweep of source, docs, defmt usage, and recent change Logs. The findings below seed the Topics list. Each item is a one-line judgement call to make during build (keep, trim, or rewrite). New items discovered along the way get appended to Topics rather than spun off.

## Plan

### Topics

**Stale references (post-vendor-class)**

- `README.md:11` — structure tree says "USB CDC firmware"; should reflect vendor-class bulk.
- `usb-serial/README.md:60` — Python-script example uses `/dev/ttyACM1` and a port arg that no longer exists.
- `usb-serial/README.md:91` — protocol table preamble says "binary packet over USB CDC serial".
- `usb-serial/README.md:99` — "auto-detects … by manufacturer/product" is now VID/PID with strings as a sanity check.
- `sysmon/README.md:7` — "USB CDC support" requirement.
- `sysmon/README.md:67` — `systemctl edit` example uses flags (`-u 500 /dev/ttyACM1`) that aren't recognised by current sysmon.
- `sysmon/README.md:72` — "`sysmon --help`" — there is no help flag; only `-f`.
- `usb-serial/firmware/src/display.rs:4` — module doc still says "USB CDC carries".

**Stale code**

- `sysmon/src/main.rs:70–81` and `:24` — commented-out A/B palette block + import of `PALETTE_B`. Either reactivate as a documented dev knob or remove the dead code and the `PALETTE_B` constant.
- `sysmon/src/projection.rs:33` — `pub const PALETTE_B: [Pixel; 6] = PALETTE_A;` (an alias of A). Hangs on the A/B decision above.
- `usb-serial/client/python/scanline_test.py:8` — references a `crossfade_test.rs` that doesn't exist in `learning-examples/examples/`. Update or drop the comparison note.
- `sysmon/PROFILING.md` — written before the vendor-class change; no longer mentions which scope of profiling produced the latest baseline. Light update to point at "post-vendor-class baseline" rather than describing it as if first-time.

**Defmt log review (firmware)**

- `usb-serial/firmware/src/main.rs:114` "usb-serial firmware running" — keep (startup confirmation).
- `usb-serial/firmware/src/main.rs:193` "USB task started — waiting for frames" — likely redundant with the startup info, consider trimming.
- `usb-serial/firmware/src/main.rs:214` "{} fps received" every second — useful when debugging, noisy in normal use; consider gating behind a feature flag or moving to debug level.
- `usb-serial/firmware/src/display.rs:105` drop warning — keep (now actually meaningful after the seq fix).
- `hub75/src/dp3364s/scan_loop.rs:40` "hub75: scan loop running" — keep (one-shot startup).

**Doc consistency / repetition**

- VID/PID/manufacturer/product info is duplicated between `SETUP.md` ("Driving the panel from a host") and `usb-serial/README.md` ("USB descriptor"). Pick one as canonical, cross-link from the other.
- `picotool` install version `2.2.0-3` is hardcoded in `SETUP.md`; check it's still current and consider linking to the "latest release" page in the same line.

**Performance opportunities (low-hanging)**

- `sysmon/src/throughput.rs` — `read_to_string` per `/sys/block/<dev>/stat` per cycle allocates a String. Pre-allocate a small byte buffer and parse in place — small but cheap.
- `sysmon/src/cpu.rs` — `/proc/stat` still ~22% of CPU. No obvious `/sys` equivalent for per-core busy fractions, but field-targeted parsing (read once, slice the lines we need) might avoid a `Vec<f32>` per call.
- `sysmon/src/main.rs:85–87` — `if let Some(rest) = mode.sampling_rate.checked_sub(...)` silently drops overruns. Add a one-line warn (rate-limited) so we know when fast mode can't keep up.

**Carry-over from prior change Logs**

- pid.codes PID registration PR for `0x7575` — non-blocking task from the vendor-class change. Submit during this sweep.
- Confirm `hub75/Cargo.toml`'s post-fix `[profile.release]` is clean (no second duplicate-debug regression).
- The `display.rs` seq double-increment was fixed last change; spot-check after re-flash that defmt output stays clean.

### Done when

Every Topic above (and any item discovered along the way) has been resolved — fixed, intentionally kept with a one-line justification in code/docs, or explicitly deferred to a new proposal.

## Conclusion

Completed. All upfront Topics resolved with two notable shapes:

- **A/B palette mode** restructured from a comment-out/uncomment block into a single `const AB_PALETTE_MODE: bool` toggle, with `PALETTE_B` no longer hidden behind `#[allow(unused_imports)]`. Re-enabling it is one boolean change. Behaviour identical with `false`.
- **Cycle-overrun warning** added in `sysmon/src/main.rs`: if a fast-mode iteration takes longer than its sampling rate, a one-second-rate-limited warning prints to stderr. Previously silent.

Defmt fps print slowed from per-second to a 10-second average. The redundant "USB task started" startup info was dropped (the earlier "usb-serial firmware running" already serves that purpose). Disk-stat parsing in `throughput.rs` no longer allocates a `Vec<&str>` per cycle; field access goes via `Iterator::nth`.

Eight stale doc/comment references to CDC ACM, `/dev/ttyACM*`, the old serial-port-style identification, or the non-existent `crossfade_test.rs` example were corrected.

**Outstanding (deferred):** the pid.codes registration PR for VID `0x1209` / PID `0x7575`. Non-blocking, no code or behaviour impact, leaving as a manual follow-up.

## Log

- A/B palette block refactored as a `const` toggle rather than removed; user confirmed the mode will be used in future for palette tuning.
- Defmt fps message slowed to 10-second averaging on user direction (between "remove" and "less frequent").
- `Vec<&str>` allocation in disk stat parser swapped for `Iterator::nth`. Trivial saving in absolute terms; tidies the code shape.
- Considered switching `read_to_string` for stack-buffered file reads in `throughput.rs` — declined as too speculative for a tidy pass; not load-bearing at sysmon's <1% CPU.
