# Owner email in USB descriptor (firmware build-time)

## Intent

Allow an owner email (or other contact string) to be baked into the firmware at build time and surfaced in the USB descriptor, so anyone inspecting the device with standard tools (e.g. `lsusb -v`) sees a clear contact route. The mechanism should not impair existing device identification — clients must continue to recognise the panel and target it by serial number unchanged.

Approach in outline (settled in the Approach stage): expose an `OWNER_EMAIL` env var read at firmware build time; when set, append it to the USB product string (e.g. `hub75 (joe@example.com)`); relax the vendor-class clients' product-string sanity check from exact match to starts-with so the existing prefix still validates the device. CDC clients match on VID/PID only and are unaffected. Serial-number-based targeting (`PANEL_NAME`, chip-ID fallback) is untouched.
