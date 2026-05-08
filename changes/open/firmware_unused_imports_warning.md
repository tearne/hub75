# Address `HEIGHT`/`WIDTH` unused-imports warning in firmware

**Mode:** Wander

## Intent

Building `usb-serial/firmware` consistently emits:

```
warning: unused imports: `HEIGHT` and `WIDTH`
  --> src/main.rs:43:45
   |
43 | use display::{FrameReceiver, ReceiveBuffer, HEIGHT, WIDTH};
```

The warning is harmless but constant noise on every build. Either remove the unused imports, or — if `HEIGHT`/`WIDTH` are intended to be referenced for some currently-disabled code path — add the appropriate `#[cfg]` gate or use site so the warning goes away.
