# Sysmon panel-outage resilience

**Mode:** Formal

## Intent

Currently sysmon crashes if the panel disappears mid-run — the firmware's USB device vanishes (cable yanked, board reset, power glitch), `Hub75Client::send_frame_rgb` returns an error, the `?` in `main` propagates it, and the process exits. All band history goes with it, and on restart the panel begins refilling from empty.

Make sysmon survive panel outages: when the panel goes away, sysmon keeps sampling and updating its internal state, periodically retries to reconnect, and resumes rendering when the panel is back — all without losing the slate history. Same for the button-event channel on the way in.

## Approach

### Resilience lives in sysmon, not `hub75-client`

The client crate stays simple: `send_frame_rgb` and `recv_event` still return errors on transport failure. Sysmon catches them and decides what to do. Reason: other consumers (`life`, `clock`, the buttons example) may want different policies — failing fast is a sensible default for those. Sysmon is the long-running service; resilience is its concern.

### Client owned as `Option<Hub75Client>`

When any I/O call fails, sysmon drops the client (sets it to `None`) and stops trying to send or receive until reconnection succeeds. This avoids the 1 s USB write timeout on every subsequent frame during a long outage. Reconnection runs on its own cadence — independent of the main render cycle.

### Sampling continues during outage

The sampling and slate-update path runs unchanged when disconnected — CPU, RAM, Disk, Net all keep being read, both `Slate`s keep receiving samples, the EMA keeps advancing. Only the frame send and button drain are skipped. When the panel returns, what gets rendered is the current, up-to-date slate — exactly as if the panel had been live the whole time.

### Reconnection cadence: independent timer

A `last_reconnect_attempt: Instant` field is consulted each cycle. If we're disconnected and `>= 2 s` have elapsed, attempt `Hub75Client::open(Some("sysmon"))`. On success: store the handle, log reconnection. On failure: update the timestamp, try again later. 2 s is a sweet spot: fast enough to feel responsive on a brief outage, slow enough that a long absence doesn't spam rusb device-enumeration.

### Logging policy

- Log once at the moment of disconnect with the underlying error.
- Log once at the moment of reconnect with the elapsed downtime.
- Don't log individual reconnect-attempt failures; the cadence is regular and observers know to expect quiet during outage.

### Button events on reconnect

After reconnection, the firmware's packed-state byte is the device's *current* state — it doesn't replay anything missed during outage. So `last_button_state` may diverge from the device's first packet, which would register every currently-pressed button as a fresh press edge. Mitigation: on successful reconnect, reset `last_button_state` to `0` and accept that a button held across an outage doesn't fire on reconnect (firmware only writes on change, so we'll see the next genuine transition). Simpler than trying to read-and-discard.

## Plan

- [x] Hold the client as `Option<Hub75Client>`; track `last_reconnect_attempt: Instant`.
- [x] On send/recv error: log once, drop the client, note disconnect time.
- [x] When disconnected and ≥ 2 s since last attempt: try `Hub75Client::open(Some("sysmon"))`; on success log reconnect with downtime, reset `last_button_state = 0`.
- [x] Skip frame send and button drain while disconnected; keep sampling, slate updates, and ramp evolution as-is.
- [x] Verify by yanking the panel cable mid-run and plugging it back in — sysmon stays alive, history survives, panel resumes.

## Log

- Bumped sysmon to 0.7.6. Panel held as `Option<Hub75Client>`; `disconnected_at: Option<Instant>` and `last_reconnect_attempt: Instant` track outage timing.
- Startup is now lenient — if `Hub75Client::open` fails at boot, log it and start in disconnected mode rather than exiting. Same retry path covers it.
- Reconnect attempt at the top of the loop, gated by `RECONNECT_INTERVAL` (2 s). On success: log downtime, store handle, reset `last_button_state = 0` so a button held across the outage doesn't fire on the next transition.
- Send and recv calls now sit behind `if let Some(p) = panel.as_mut()`. Either failure drops the handle and records `disconnected_at = cycle_start`; the next reconnect cadence picks it up.
- Sampling, slate updates, and ramp evolution all run unconditionally — the history-coherence guarantee from the Approach.
- Build clean.
- Disconnect-log messages mentioned the call site ("on button read" / "on frame send"), which was misleading when the user pulled power — the underlying event is the same regardless of which I/O surfaced it. Collapsed both to `panel disconnected: {error}`. Bumped to 0.7.7.

## Conclusion

Sysmon now survives panel outages. The client is `Option<Hub75Client>`; any I/O error drops it and records the disconnect time. Reconnection runs on an independent 2 s cadence — quiet on failure, logs downtime on success. Sampling, slate updates and the EMA all keep running while disconnected, so when the panel returns it shows current data with no rebuild.

Startup now tolerates an absent panel (logs and retries rather than exiting), and held buttons across an outage don't fire on reconnect (firmware sends only on change; `last_button_state = 0` reset on reconnect absorbs the asymmetry).

Final version: sysmon 0.7.7.
