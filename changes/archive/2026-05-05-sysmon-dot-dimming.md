# Sysmon dot dimming

**Mode:** Wander

## Intent

In sysmon, each metric is shown as a row (or band) of dots whose count grows with the metric's value — low values show fewer dots. At present every lit dot is rendered at full brightness, so a low value reads as "few dots, but each one shouts". The user wants low values to also look dimmer overall: the dots that are lit when the value is low should be drawn at reduced brightness, scaled in line with the value, so a low metric quietly fades rather than punching at full intensity.

## Conclusion

`paint_row` in `usb-serial/client/sysmon/src/projection.rs` now scales the lit-dot colour by `row.value`, with a floor of `MIN_DOT_BRIGHTNESS = 0.1` so the lone dot at value≈0 stays visible. Existing `scale_pixel` reused.
