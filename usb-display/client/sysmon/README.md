# sysmon

System monitor for a HUB75 LED panel. Renders host CPU (per-core), RAM, Disk I/O, and Network throughput onto a 64×32 panel via a Raspberry Pi Pico running the [`usb-display`](../../) firmware. Installs as a `systemd` service that auto-starts on boot.

## Hardware compatibility

**Tested only on Raspberry Pi 5** (aarch64). Should work on any aarch64 Debian-based Linux host with USB CDC support and a Pico flashed with the matching `panel-shift-64x32` firmware.

## Install

### 1. Flash the Pico firmware

The panel needs the `usb-display` firmware running on a Raspberry Pi Pico (RP2350). One-time setup, then it sits in a USB port forever.

Hold **BOOT** on the Pico while plugging it into USB to enter BOOTSEL mode, then from this repository's workspace root:

```sh
(cd usb-display/firmware && cargo build --release --features panel-shift-64x32) && \
  picotool load -v -x -t elf target/thumbv8m.main-none-eabihf/release/usb-display-firmware
```

`picotool` install instructions and udev rules for non-root access are in [`SETUP.md`](../../../SETUP.md) and [`FLASHING.md`](../../../FLASHING.md) at the repo root.

### 2. Build the deb

From inside this directory (`usb-display/client/sysmon/`):

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

If the service can't find the panel device, check that the Pico is plugged in and shows up as `tearne / hub75` in `lsusb`. The service runs as root so no group-membership setup is needed.

## Configure

To pass flags (e.g. a custom update interval or a specific serial port), drop a systemd override:

```sh
sudo systemctl edit sysmon
```

…and add:

```
[Service]
ExecStart=
ExecStart=/usr/bin/sysmon -u 500 /dev/ttyACM1
```

Then `sudo systemctl restart sysmon`.

Run `sysmon --help` for the full flag list.

## Uninstall

```sh
sudo apt remove sysmon       # remove (keeps config)
sudo apt purge sysmon        # remove + clean up
```

The package's `postrm` stops and disables the service.
