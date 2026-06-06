# Owner tag in USB descriptor (firmware build-time)

**Mode:** Formal

## Intent

Bake a free-form owner tag — a contact email, name, asset tag, or any string meaningful to the owner — into the firmware at build time and surface it in the USB product string, so anyone inspecting the device with standard tools (e.g. `lsusb -v`) sees it. It must not impair existing device identification — clients must still recognise the panel, and serial-number targeting (`PANEL_NAME`, chip-ID fallback) must be unchanged.

## Approach

### The tag is a build-time stamp composed centrally in firmware-lib

Add a `build.rs` to `usb/firmware-lib` that reads the `OWNER_TAG` env var and emits `cargo:rustc-env=USB_PRODUCT_STRING` — `hub75 (<tag>)` when the var is set and non-empty, else plain `hub75` — plus `rerun-if-env-changed=OWNER_TAG`. firmware-lib exposes a single `const PRODUCT: &str = env!("USB_PRODUCT_STRING")` and uses it for `usb_config.product` in both the CDC and vendor paths, replacing the two hardcoded `"hub75"` literals.

The tag lives centrally rather than in `FirmwareConfig`: it's a per-owner/per-deployment stamp, identical across whatever firmware is flashed, unlike `serial_number` which is genuine per-product identity. Central means every firmware binary picks it up with zero per-binary wiring. A `build.rs` is required because a conditional `&'static str` can't be composed with `env!`/`concat!` alone (the unset branch fails to compile) and no_std firmware can't format one at runtime.

The value is opaque — `build.rs` wraps it verbatim, so any string works (`OWNER_TAG="joe@example.com"`, `OWNER_TAG="Joe's bench — ext 4421"`).

### Product string keeps the `hub75` prefix; vendor client matches on the prefix

Keep `hub75` as the literal prefix and relax the vendor client's product-string check in `transport_vendor.rs` (`find_device` and `list_panels`) from exact match to `starts_with("hub75")`. This preserves device recognition once the tag is appended. The CDC client matches on VID/PID only and is unaffected; serial/`PANEL_NAME` targeting is untouched.

### Descriptor length

USB string descriptors cap at 126 UTF-16 units; `hub75 (<tag>)` is far short of that, so no enforcement — noted only so an unreasonably long value isn't a silent surprise.

## Plan

- [x] Add `usb/firmware-lib/build.rs`: read `OWNER_TAG`, emit `cargo:rustc-env=USB_PRODUCT_STRING` (`hub75 (<tag>)` if set and non-empty, else `hub75`) and `cargo:rerun-if-env-changed=OWNER_TAG`.
- [x] In firmware-lib, add `const PRODUCT: &str = env!("USB_PRODUCT_STRING")` and use it for `usb_config.product` in `class_cdc.rs` and `class_vendor.rs`, replacing the hardcoded `"hub75"`.
- [x] Relax the vendor client product check in `transport_vendor.rs` (`find_device`, `list_panels`) from exact match to `starts_with(USB_PRODUCT)`.
- [x] Document the `OWNER_TAG` build flag in `usb/README.md` alongside the existing build-flag docs.
- [x] Build-verify: firmware-lib via a firmware binary with and without `OWNER_TAG` set; the vendor client.

## Log

- Versions: `hub75-client` 0.7.2 (patch — relaxed product check, non-breaking), `usb-firmware-lib` 0.2.0 (minor — descriptor behaviour). Firmware binaries (`usb/firmware`, `sysmon/firmware`) unchanged in source; they pick up the tag on rebuild via the central firmware-lib path.
- Binary-level verification (no panel here): firmware ELF carries `hub75` with no tag, `hub75 (joe@example.com)` with `OWNER_TAG` set, and bare `hub75` for a whitespace-only tag. `rerun-if-env-changed=OWNER_TAG` correctly re-triggered firmware-lib on each env change.

## Conclusion

Completed and hardware-confirmed — `OWNER_TAG` shows in the device product string (`lsusb -v`) and the vendor client still recognises the panel with the tag appended. Final versions: `usb-firmware-lib` 0.2.0, `hub75-client` 0.7.2; firmware binaries unchanged in source. Went to plan; no `map.md` or project changelog to update.
