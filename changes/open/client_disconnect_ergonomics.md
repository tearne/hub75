# hub75-client: typed disconnect error + in-place reconnect

## Intent

When a panel is temporarily unplugged, a consumer application currently can't handle it gracefully without guesswork. `send_frame_rgb` (and `open`) return `Result<(), Box<dyn std::error::Error>>`, so the only thing an app can inspect is `.is_err()` — a yanked cable, a write timeout, and a genuine caller bug (wrong pixel count) are all the same opaque box. The app is forced into "any failure → reconnect", which both loses flexibility (can't distinguish transient from fatal) and is burdensome (it must construct a whole new `Hub75Client`, re-supplying the serial selector and losing the `seq` counter and frame buffer). This surfaced building the `dots-64x32` example, whose reconnect loop has exactly this limitation.

The library is the right place to fix most of this: it already abstracts two transport backends (rusb for vendor, `serialport` for CDC) that report disconnects in completely different, per-OS ways — that knowledge currently leaks up to every consumer.

Proposed direction, layered so flexibility is preserved rather than replaced:

- **Typed errors** — a `SendError`/error enum with at least a `Disconnected` variant (transient, reconnectable) distinct from `Timeout`, `InvalidFrame` (caller bug, never retry), and generic I/O. The library normalises rusb's `NO_DEVICE`/pipe errors and `serialport`'s broken-pipe/vanished-port errors into one `Disconnected`. Mildly breaking for existing callers — wants a thought-through taxonomy.
- **In-place reconnect** — `client.reconnect()` that re-runs port discovery while preserving the serial selector (not currently stored on the client), the frame buffer, and `seq` continuity; plus an `open_retrying(serial, policy)` to remove the initial-connect loop apps otherwise write. The app keeps deciding *when*; the lib owns the transport-specific *how*.
- **Optional turnkey resilience** — an opt-in `ResilientClient` wrapper (or `send_frame_resilient`) with a reconnect policy (interval/backoff, optional `on_disconnect`/`on_reconnect` callback) for apps that want zero ceremony. Deliberately opt-in: auto-reconnect inside `send_frame` would hide blocking and steal the app's ability to act during an outage.

The `seq`-continuity point is a correctness reason to prefer in-place reconnect over constructing a fresh client.

Captured as an aside during the `dots-64x32` change.
