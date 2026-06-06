# Clarify sysmon README firmware wording

**Mode:** Wander

## Intent

`sysmon/README.md`'s opening sentence (line 3) says the panel renders "via a Raspberry Pi Pico running the `usb` firmware (vendor-class build)", which reads as if sysmon runs the *default* `usb` firmware binary. It actually runs its own dedicated `sysmon-firmware` binary (built on the `usb` firmware-lib), as line 13 correctly states. The intro's wording could mislead someone into flashing the wrong binary. Reword the intro so it names `sysmon-firmware` and frames the `usb` relationship as "built on the `usb` firmware-lib", removing the internal inconsistency.

Captured as an aside during the `dots-64x32` change.

## Conclusion

Single-sentence rewrite of `sysmon/README.md` line 3: now names `sysmon-firmware` directly and frames `usb` as the firmware-lib it's built on, matching the (correct) wording further down. Patch bump 0.7.8 → 0.7.9.
