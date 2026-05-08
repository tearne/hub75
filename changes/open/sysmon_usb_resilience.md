# sysmon should survive transient USB errors

## Intent

Currently `sysmon` exits with status 1 on any `libusb_submit_transfer` failure (error `-4` `LIBUSB_ERROR_NO_DEVICE` observed when another panel was hot-plugged onto the same bus). systemd restarts it, but the panel goes blank during each cycle and on lab benches with marginal USB power the loop can persist indefinitely.

Make sysmon resilient to transient USB errors: re-open the device on `NO_DEVICE` / submit failures and resume streaming, with backoff and a give-up threshold for genuinely-disconnected panels. Crashes should be reserved for unrecoverable failures (panel actually gone, not just briefly perturbed).
