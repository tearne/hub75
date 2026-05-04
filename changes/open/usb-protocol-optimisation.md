# USB protocol optimisation

## Intent

Reduce host-side CPU cost of streaming pixel frames to the Pico over USB. Currently the Pico exposes a CDC ACM (virtual serial port) interface, and sysmon `write()`s ~6 KB of RGB bytes per frame to `/dev/ttyACM*`. At fast-mode 20 Hz this becomes 120 KB/s of byte-stream traffic going through the host kernel's TTY/line-discipline layers, which were never meant for binary pixel data.

Three lines of attack to consider, in increasing scope:

1. **Replace CDC ACM with raw USB bulk on a vendor class.** Firmware drops the "I am a modem" pretence and exposes a vendor-defined bulk endpoint. Host opens the device with `libusb`/`rusb` and writes pixel data straight into URBs, bypassing the TTY stack entirely. Same throughput, less per-byte work. Modest firmware change + a thin Rust wrapper inside `hub75-client`.

2. **Move rendering onto the Pico.** Sysmon sends only the ~9 sampled metric values per cycle (≈40 bytes), and the firmware does the banding, pattern-flow, and painting on-device. USB traffic drops by roughly 150×. The firmware grows substantially — most of `bands.rs` / `projection.rs` would migrate.

3. **Frame-buffer compression.** Most pixels are black; RLE or similar would compress a typical frame to a fraction of the current 6 KB. Adds encode CPU on host, decode CPU on firmware. Cheaper architecturally than the move-rendering option but a smaller win.

For the immediate "make fast mode normal" goal, the Rust-side render-pipeline cleanups (separate change) should come first — they're local and cheap. This proposal is for once those have been measured and the USB layer is the next bottleneck to attack.
