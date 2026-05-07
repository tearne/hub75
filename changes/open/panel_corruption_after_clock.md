# Investigate panel corruption (observed after running `clock`)

## Intent

After running the freshly-fixed `clock` example on the 128x64 panel (during the `clock_fix_stale_open_api` change), the panel display became corrupted. Power-cycling the panel cleared it and `clock` ran cleanly afterward.

Cause unknown. Could be `clock` itself (e.g. it doesn't account for the 128x64 geometry — the constants assume a 64-wide square), could be a driver/firmware issue surfaced by the switch between examples, could be an artefact of something else done in the session (e.g. running `life`, stopping `sysmon`, plugging the panel between uses). Worth investigating before it bites again silently.
