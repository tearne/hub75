//! Host-side client for sending frames to the HUB75 display over USB.
//!
//! The active transport is chosen at compile time by Cargo feature:
//!
//! - `transport-vendor` (default) — talks to the firmware's vendor-class
//!   bulk endpoint via `libusb` (through `rusb`), using a manual async
//!   transfer loop. Fast; needs a WinUSB-style driver association on
//!   Windows (typically requires admin).
//!
//! - `transport-cdc` — talks to the firmware over CDC ACM via a serial
//!   port. Slower (kernel TTY layer on Linux/macOS, `usbser.sys` on
//!   Windows), but works on locked-down Windows hosts without admin
//!   rights. Sends are capped at 15 fps client-side as a conservative
//!   default until measured.
//!
//! # Example
//!
//! ```no_run
//! use hub75_client::Hub75Client;
//!
//! let mut client = Hub75Client::open(None)?;
//! let frame = vec![[255, 0, 0]; 64 * 32];
//! client.send_frame_rgb(&frame)?;
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

#[cfg(not(any(
    feature = "panel-64x64",
    feature = "panel-64x32",
    feature = "panel-128x64",
)))]
compile_error!(
    "Select a panel size: --features panel-64x64 (or panel-64x32 or panel-128x64)"
);

#[cfg(any(
    all(feature = "panel-64x64", feature = "panel-64x32"),
    all(feature = "panel-64x64", feature = "panel-128x64"),
    all(feature = "panel-64x32", feature = "panel-128x64"),
))]
compile_error!("Only one panel size feature may be enabled");

#[cfg(not(any(feature = "transport-vendor", feature = "transport-cdc")))]
compile_error!("Select a transport: --features transport-vendor (default) or transport-cdc");

#[cfg(all(feature = "transport-vendor", feature = "transport-cdc"))]
compile_error!("transport-vendor and transport-cdc are mutually exclusive");

#[cfg(feature = "panel-64x64")]
pub const WIDTH: usize = 64;
#[cfg(feature = "panel-64x64")]
pub const HEIGHT: usize = 64;

#[cfg(feature = "panel-64x32")]
pub const WIDTH: usize = 64;
#[cfg(feature = "panel-64x32")]
pub const HEIGHT: usize = 32;

#[cfg(feature = "panel-128x64")]
pub const WIDTH: usize = 128;
#[cfg(feature = "panel-128x64")]
pub const HEIGHT: usize = 64;

pub const FRAME_MAGIC: &[u8; 4] = b"HB75";
pub const FRAME_PIXEL_BYTES: usize = WIDTH * HEIGHT * 3;
pub const FRAME_BUFFER_BYTES: usize = 5 + FRAME_PIXEL_BYTES;

pub(crate) const USB_VID: u16 = 0x1209;
pub(crate) const USB_PID: u16 = 0x7575;
#[cfg(feature = "transport-vendor")]
pub(crate) const USB_MANUFACTURER: &str = "tearne";
#[cfg(feature = "transport-vendor")]
pub(crate) const USB_PRODUCT: &str = "hub75";

mod error;
pub use error::{Error, Result};

#[cfg(feature = "transport-vendor")]
mod transport_vendor;
#[cfg(feature = "transport-vendor")]
use transport_vendor::Transport;

#[cfg(feature = "transport-cdc")]
mod transport_cdc;
#[cfg(feature = "transport-cdc")]
use transport_cdc::Transport;

/// One attached panel, as reported by [`Hub75Client::list_panels`].
#[derive(Debug, Clone)]
pub struct PanelInfo {
    pub serial: String,
    /// `true` if the panel is currently free for this process to open.
    /// `false` means another process holds it (vendor transport only;
    /// the CDC transport reports `true` for every enumerated port).
    pub available: bool,
}

pub struct Hub75Client {
    transport: Transport,
    buffer: Box<[u8; FRAME_BUFFER_BYTES]>,
    seq: u8,
    /// The serial selector this client was opened with, kept so
    /// [`reconnect`](Self::reconnect) can re-open the same panel.
    serial: Option<String>,
}

impl Hub75Client {
    /// Open a panel.
    ///
    /// `serial` matches the panel's USB serial-number string (the
    /// firmware's chip-ID hex or a `PANEL_NAME` override). `None` opens
    /// the first matching panel — fine when only one is attached.
    pub fn open(serial: Option<&str>) -> Result<Self> {
        Ok(Self {
            transport: Transport::open(serial)?,
            buffer: Box::new([0u8; FRAME_BUFFER_BYTES]),
            seq: 0,
            serial: serial.map(str::to_owned),
        })
    }

    /// Re-open the same panel after an [`Error::Disconnected`], reusing the
    /// serial selector from [`open`](Self::open) and preserving the frame
    /// sequence counter so the firmware sees an unbroken stream.
    ///
    /// One attempt: returns `Ok(())` once reconnected, or the open error
    /// (typically [`Error::Disconnected`] while the panel is still absent).
    /// The caller decides when and how often to retry.
    pub fn reconnect(&mut self) -> Result<()> {
        self.transport = Transport::open(self.serial.as_deref())?;
        Ok(())
    }

    pub fn send_frame(&mut self, pixel_bytes: &[u8]) -> Result<()> {
        if pixel_bytes.len() != FRAME_PIXEL_BYTES {
            return Err(Error::InvalidFrame {
                expected: FRAME_PIXEL_BYTES,
                got: pixel_bytes.len(),
            });
        }
        self.buffer[..4].copy_from_slice(FRAME_MAGIC);
        self.buffer[4] = self.seq;
        self.buffer[5..].copy_from_slice(pixel_bytes);
        self.transport.send_bytes(&self.buffer[..])?;
        self.seq = self.seq.wrapping_add(1);
        Ok(())
    }

    pub fn send_frame_rgb(&mut self, pixels: &[[u8; 3]]) -> Result<()> {
        if pixels.len() != WIDTH * HEIGHT {
            return Err(Error::InvalidFrame {
                expected: WIDTH * HEIGHT,
                got: pixels.len(),
            });
        }
        let bytes: &[u8] =
            unsafe { std::slice::from_raw_parts(pixels.as_ptr() as *const u8, FRAME_PIXEL_BYTES) };
        self.send_frame(bytes)
    }

    /// Read one button-state byte from the firmware, blocking for at
    /// most `timeout`. Bit 0 = button A, bit 1 = button B; 1 = pressed.
    /// Returns `Ok(Some(byte))` on event, `Ok(None)` on timeout.
    pub fn recv_event(&mut self, timeout: std::time::Duration) -> Result<Option<u8>> {
        self.transport.recv_event(timeout)
    }

    /// List all attached HUB75 panels visible to the active transport.
    pub fn list_panels() -> Result<Vec<PanelInfo>> {
        Transport::list_panels()
    }
}
