# sysmon replacement

## Intent

Replace the original `sysmon` crate with `sysmon2`, renamed to `sysmon`. The new code is the production version going forward; the old code's distinctive features (time-compressed splat rendering, RAM streaks, etc.) are no longer wanted. The deb package must continue to build and the deployed binary must upgrade cleanly on the Pi via `apt`/`dpkg`.

Bump version from `0.1.1` (last sysmon release) to **0.2.0** — signals a substantial change while staying within the 0.x lineage of an early-stage end-user app. Major-version bump (1.0.0) feels heavier than the situation warrants.

The current `sysmon` directory (`usb-display/client/sysmon/`) and `sysmon2` directory (`usb-display/client/sysmon2/`) consolidate into a single directory at `usb-display/client/sysmon/` containing what's currently in `sysmon2`, with deb packaging carried over from the old `sysmon`. After this change there is no more `sysmon2` directory.
