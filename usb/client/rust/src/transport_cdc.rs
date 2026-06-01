//! CDC ACM transport via `serialport`. The firmware appears as
//! `/dev/ttyACM*` on Linux/macOS or a COM port on Windows; we filter
//! enumerated ports by the firmware's VID/PID to find ours.
//!
//! Sends are capped at 15 fps as a conservative ceiling — the kernel
//! TTY layer (Linux/macOS) and `usbser.sys` (Windows) impose per-OS
//! overhead we haven't measured. Raise the cap by editing
//! [`MAX_FPS`] once a target host is measured.

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use serialport::{SerialPort, SerialPortType};

use crate::{Error, PanelInfo, USB_PID, USB_VID};

/// Conservative CDC frame-rate ceiling. Set client-side so the
/// firmware just consumes what arrives.
const MAX_FPS: u32 = 15;
const MIN_FRAME_INTERVAL: Duration = Duration::from_millis(1000 / MAX_FPS as u64);

/// Per-write timeout — generous since CDC throughput is host-OS-dependent.
const WRITE_TIMEOUT: Duration = Duration::from_secs(1);

pub(crate) struct Transport {
    port: Box<dyn SerialPort>,
    last_send: Option<Instant>,
}

impl Transport {
    pub(crate) fn open(serial: Option<&str>) -> crate::Result<Self> {
        let port_name = find_port(serial)?;
        // Non-exclusive open so we coexist with anything else briefly
        // touching the port — notably ModemManager on Linux, which
        // probes every new /dev/ttyACM* and would otherwise block us
        // with a transient exclusive lock. pyserial defaults to
        // non-exclusive; match it.
        let port = serialport::new(&port_name, 115_200)
            .timeout(WRITE_TIMEOUT)
            .exclusive(false)
            .open()?;
        Ok(Self {
            port,
            last_send: None,
        })
    }

    pub(crate) fn send_bytes(&mut self, buffer: &[u8]) -> crate::Result<()> {
        // Enforce the rate cap before each send so callers don't have
        // to think about pacing.
        if let Some(last) = self.last_send {
            let elapsed = last.elapsed();
            if elapsed < MIN_FRAME_INTERVAL {
                std::thread::sleep(MIN_FRAME_INTERVAL - elapsed);
            }
        }
        self.port.write_all(buffer)?;
        self.last_send = Some(Instant::now());
        Ok(())
    }

    pub(crate) fn recv_event(&mut self, timeout: Duration) -> crate::Result<Option<u8>> {
        self.port.set_timeout(timeout)?;
        let mut buf = [0u8; 1];
        let result = self.port.read(&mut buf);
        // Restore the longer write/send timeout so the next frame send
        // gets generous slack rather than this caller's recv window.
        let _ = self.port.set_timeout(WRITE_TIMEOUT);
        match result {
            Ok(1) => Ok(Some(buf[0])),
            Ok(_) => Ok(None),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(None),
            Err(e) => Err(e.into()),
        }
    }

    pub(crate) fn list_panels() -> crate::Result<Vec<PanelInfo>> {
        let mut panels = Vec::new();
        for port in serialport::available_ports()? {
            if let SerialPortType::UsbPort(info) = &port.port_type {
                if info.vid == USB_VID && info.pid == USB_PID {
                    panels.push(PanelInfo {
                        serial: info.serial_number.clone().unwrap_or_default(),
                        // CDC enumeration doesn't tell us whether another
                        // process holds the port; assume available.
                        available: true,
                    });
                }
            }
        }
        Ok(panels)
    }
}

fn find_port(serial: Option<&str>) -> crate::Result<String> {
    for port in serialport::available_ports()? {
        if let SerialPortType::UsbPort(info) = &port.port_type {
            if info.vid != USB_VID || info.pid != USB_PID {
                continue;
            }
            if let Some(want) = serial {
                let got = info.serial_number.as_deref().unwrap_or("");
                if got != want {
                    continue;
                }
            }
            return Ok(port.port_name);
        }
    }
    // No matching panel enumerated — a reconnectable condition, not a hard fault.
    Err(Error::Disconnected)
}

/// Map a `std::io::ErrorKind` (from a read/write on the port) onto a client error.
/// `describe` builds the fallback message only when no specific variant fits.
fn map_io_kind(kind: std::io::ErrorKind, describe: impl FnOnce() -> String) -> Error {
    use std::io::ErrorKind::*;
    match kind {
        BrokenPipe | NotFound | NotConnected | UnexpectedEof => Error::Disconnected,
        TimedOut => Error::Timeout,
        PermissionDenied => Error::PermissionDenied,
        _ => Error::Other(describe()),
    }
}

impl From<std::io::Error> for Error {
    fn from(error: std::io::Error) -> Self {
        let kind = error.kind();
        map_io_kind(kind, || format!("io error: {error}"))
    }
}

impl From<serialport::Error> for Error {
    fn from(error: serialport::Error) -> Self {
        match error.kind {
            // serialport reports a disconnected (or busy) device as NoDevice.
            serialport::ErrorKind::NoDevice => Error::Disconnected,
            serialport::ErrorKind::Io(kind) => {
                map_io_kind(kind, || format!("serial io error: {}", error.description))
            }
            _ => Error::Other(format!("serial error: {}", error.description)),
        }
    }
}
