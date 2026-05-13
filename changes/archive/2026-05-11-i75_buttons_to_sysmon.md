# Interstate 75 buttons over USB

**Mode:** Formal

## Intent

Prove that the Interstate 75 board's physical buttons can deliver press events to the host over USB. The USB-serial protocol already reserves a bulk IN endpoint "for future telemetry/buttons (declared but unused)" — wire button GPIOs through the firmware to that endpoint and demonstrate the host receiving events end-to-end.

Scope is the transport only: firmware reads buttons, host receives bytes. A minimal host-side reader (smoke test, not sysmon integration) is enough to prove the path. Sysmon consuming the events to change settings is deliberately out of scope and lives in a follow-up change.

## Approach

### Reader lives in `hub75-client`, not a standalone tool

The host-side reader is added as a `recv_event` (or similar) method on `Hub75Client`. The smoke test is a tiny binary in the client crate's `examples/` that calls it. Reason: the next change is sysmon consuming events, which means calling exactly this API; building it once now avoids redoing it for sysmon, and gives the smoke test a real-API exercise rather than a raw-rusb detour.

### Wire format: single-byte packed-state event

Each event is one byte where each bit is the current state of one button (1 = pressed). The firmware emits one byte every time the packed state changes (i.e. a press or release of any button). Reason: tiny, self-synchronising, no framing needed, and "state" rather than "edge" means the host never gets out of sync with the device after a missed packet or reconnect. Cost: the host has to diff against the previous state to detect press vs release, but that's trivial.

### Buttons: A and B on the Pimoroni-standard GPIOs

Two momentary buttons (A, B) on the GPIO pins Pimoroni uses for both Interstate 75 and Interstate 75 W: GP14 (A) and GP15 (B). Configured as inputs with pull-ups; pressed reads low. Verified empirically by the smoke test — if a pin assumption is wrong, the test will fail visibly and we'll adjust.

### Button polling, not interrupts

A single embassy task polls GP14 and GP15 every 5 ms, runs a 3-sample debouncer (state must match for 3 consecutive polls = 15 ms to register), and compares packed state against the last sent. Reason: GPIO interrupts on the RP2350 work fine but interrupt-driven debouncing is annoying; a 5 ms poll task is dead simple, has negligible cost (two GPIO reads + a compare; the core sleeps via `Timer::after` between polls), and produces clean events. USB writes happen only on state change, not every poll.

### Firmware emit path

The polling task writes the single byte to the existing `bulk_in` endpoint (currently held as `_bulk_in` in `usb_task`). Done via `bulk_in.write(&[byte]).await`. The endpoint is already declared in the descriptor — no descriptor changes, no re-enumeration concerns.

### Client recv path

`Hub75Client` gains a method that performs a single bulk IN read with a configurable timeout, returning `Option<u8>` (`None` on timeout). Synchronous, matching the existing send API's style. Sysmon will call it from its main loop on a short timeout each cycle, alongside `send_frame_rgb`.

### Smoke test

A small binary in `usb-serial/client/rust/examples/buttons.rs` opens a `Hub75Client`, loops calling `recv_event` with a long timeout, and prints each event's packed byte plus the diff from the previous state ("A pressed", "B released", etc.). Pass = pressing every button on the board produces correct events at the host.

## Plan

- [x] Firmware: add embassy task that polls GP14/GP15 every 5 ms, 3-sample debounce, and writes the packed-state byte to `bulk_in` on change.
- [x] Firmware: stop discarding `bulk_in` in `usb_task`; thread it to the new task.
- [x] `hub75-client`: add `recv_event(timeout) -> Result<Option<u8>, _>` reading the IN endpoint (interface 0 is already claimed in `open`).
- [x] Add `examples/buttons.rs` that opens a client, loops on `recv_event`, prints each new packed byte and the per-button transitions.
- [x] Verify on board: press A, press B, press both, release in various orders — host log matches.

## Log

- Firmware: button polling and emit live inside `usb_task` as a third future joined with `usb.run()` and the OUT reader. Tried separating into a dedicated task but the bulk-IN endpoint's concrete type is hard to type-name; keeping it scoped to the task is simpler and the cost is just a single `join3` call.
- Firmware: `Debouncer` is a small struct with `stable / candidate / streak` — straightforward N-sample debouncer. Constants `BUTTON_POLL_MS = 5`, `BUTTON_DEBOUNCE_SAMPLES = 3`.
- Firmware: bulk-IN write is best-effort (`let _ = bulk_in.write(...)`); if the host isn't draining we drop the event rather than block the poll loop. Resync happens on the next state change.
- Client: implemented `recv_event` with rusb's sync `read_bulk` rather than the async transfer dance the OUT path uses. The OUT path went async to avoid per-frame transfer allocation under high frame rates; `recv_event` is called at most once per loop iteration so the same optimisation isn't worth it.
- All three crates build clean.

## Conclusion

Completed. Firmware 0.7.0, hub75-client 0.5.0 shipped. `usb-serial/README.md` updated: the bulk IN endpoint is no longer "reserved/unused" — added a Button events subsection covering the packed-byte format, A/B GPIO assignments, debounce and best-effort semantics, plus a pointer to `recv_event` and the smoke test example.

