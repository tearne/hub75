# sysmon

System monitor for a HUB75 LED panel. Renders host CPU (per-core), RAM, Disk I/O, and Network throughput onto a 64×32 panel via a Raspberry Pi Pico running the [`usb-serial`](../usb-serial/) firmware. Installs as a `systemd` service that auto-starts on boot.

## Hardware compatibility

**Tested only on Raspberry Pi 5** (aarch64). Should work on any aarch64 Debian-based Linux host with `libusb-1.0` available and a Pico flashed with the matching `panel-shift-64x32` firmware.

## Install

### 1. Flash the Pico firmware

The panel needs the `usb-serial` firmware running on a Raspberry Pi Pico (RP2350). One-time setup, then it sits in a USB port forever.

Hold **BOOT** on the Pico while plugging it into USB to enter BOOTSEL mode, then:

```sh
cd usb-serial/firmware
cargo build --release --features panel-shift-64x32 && \
  picotool load -v -x -t elf target/thumbv8m.main-none-eabihf/release/usb-serial-firmware
```

`picotool` install instructions and udev rules for non-root access are in [`SETUP.md`](../SETUP.md#flashing-via-bootsel) at the repo root.

### 2. Build the deb

From inside this directory (`sysmon/`):

```sh
./build.sh
```

The script needs `cargo-deb` — install it once with `cargo install cargo-deb` if missing. The build runs natively on the Pi (arm64). The script prints the path of the produced `.deb`.

### 3. Install the deb

```sh
sudo apt install ./target/debian/sysmon_<version>_arm64.deb
```

(Using `apt install ./path.deb` rather than `dpkg -i` so any missing dependencies are auto-resolved.)

The package's `postinst` enables and starts the `sysmon` systemd service. It will auto-start on every boot from now on.

## Verify

```sh
systemctl status sysmon         # is it running?
journalctl -u sysmon -f         # follow its log output
```

If the service can't find the panel device, check that the Pico is plugged in and shows up under VID/PID `1209:7575` in `lsusb`. The package installs a udev rule giving the `dialout` group access to the device.

## Configure

Sysmon takes a single optional flag:

- `-f <hz>` — set the panel refresh rate, e.g. `-f 20` for 20 Hz. Without it: prod cadence (1 Hz).

To run at a different rate under systemd, drop an override:

```sh
sudo systemctl edit sysmon
```

…and add:

```
[Service]
ExecStart=
ExecStart=/usr/bin/sysmon -f 20
```

Then `sudo systemctl restart sysmon`. If sysmon can't keep up at the requested rate it will print `warn: cycle overran ...` to the journal (via `journalctl -u sysmon`).

### Frame-rate vs. CPU cost

Measured on a Raspberry Pi 5 running this firmware over USB-FS bulk:

| Rate | CPU (one core) |
|---|---|
| 1 Hz (default) | 0.04% |
| 10 Hz | 0.31% |
| 20 Hz | 0.5% |
| 30 Hz | 0.82% |
| 60 Hz | 1.24% |
| 100 Hz | 2.0% |

Above 20 Hz, host CPU scales linearly at roughly **0.025% per Hz**. The hardware ceiling is around **125 Hz** — limited by USB-FS bulk transfer time of a 6 KB frame, not by host CPU. Above ~125 Hz the cycle overruns and sysmon free-runs slower than requested.

Visually, the band scroll is smooth at 20–30 Hz; higher rates are barely distinguishable. The default is 1 Hz to keep idle host load minimal; raise it to taste.

## Uninstall

```sh
sudo apt remove sysmon       # remove (keeps config)
sudo apt purge sysmon        # remove + clean up
```

The package's `postrm` stops and disables the service.
