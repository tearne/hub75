//! Host-side client for sending frames to the HUB75 display over USB.
//!
//! Talks to the firmware's vendor-class bulk endpoint via `libusb`
//! (through `rusb`), bypassing the kernel TTY/line-discipline machinery
//! that CDC ACM otherwise pulls in. The firmware advertises VID
//! `0x1209` (pid.codes) and PID `0x7575`; we double-check the
//! `manufacturer`/`product` strings as a sanity match.
//!
//! Per-frame work uses libusb's async transfer API with a single
//! reusable `libusb_transfer` allocated at startup. `rusb`'s safe
//! synchronous `write_bulk` allocates and frees a transfer per call,
//! which dominated the host-CPU cost of streaming pixels until we
//! switched to this manual-async path.
//!
//! # Example
//!
//! ```no_run
//! use hub75_client::Hub75Client;
//!
//! let mut client = Hub75Client::open(None)?;
//!
//! let frame = vec![[255, 0, 0]; 64 * 32];
//! client.send_frame_rgb(&frame)?;
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

use std::os::raw::{c_int, c_void};
use std::sync::atomic::{AtomicI32, Ordering};

use rusb::ffi as libusb;
use rusb::UsbContext;

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

const USB_VID: u16 = 0x1209;
const USB_PID: u16 = 0x7575;
const USB_MANUFACTURER: &str = "tearne";
const USB_PRODUCT: &str = "hub75";

/// First bulk OUT endpoint on the firmware's vendor-class interface.
const BULK_OUT_ENDPOINT: u8 = 0x01;

/// Per-transfer timeout in milliseconds.
const WRITE_TIMEOUT_MS: u32 = 1000;

pub struct Hub75Client {
    handle: rusb::DeviceHandle<rusb::GlobalContext>,
    transfer: *mut libusb::libusb_transfer,
    buffer: Box<[u8; FRAME_BUFFER_BYTES]>,
    /// Set to 1 by the libusb completion callback. Polled by
    /// `libusb_handle_events_completed` and by us. Boxed so the
    /// pointer is stable across moves of `Hub75Client`.
    completed: Box<AtomicI32>,
    seq: u8,
}

// rusb's DeviceHandle is Send. The raw pointers we hold (transfer,
// boxed buffer/flag) are stable for the lifetime of the Client and
// only accessed through &mut self in send_frame, so the type is safe
// to move between threads.
unsafe impl Send for Hub75Client {}

/// One attached panel, as reported by [`Hub75Client::list_panels`].
#[derive(Debug, Clone)]
pub struct PanelInfo {
    pub serial: String,
    /// `true` if `claim_interface` succeeded during enumeration —
    /// the panel is currently free for this process to open. `false`
    /// means another process (e.g. `sysmon`) holds it.
    pub available: bool,
}

impl Hub75Client {
    /// Open a panel.
    ///
    /// `serial` matches the panel's USB `iSerial` (the chip-ID hex string
    /// or a `PANEL_NAME` override baked into the firmware). `None` opens
    /// the first matching panel — fine when only one is attached.
    pub fn open(serial: Option<&str>) -> Result<Self, Box<dyn std::error::Error>> {
        let device = find_device(serial)?;
        let handle = device.open()?;
        let _ = handle.set_auto_detach_kernel_driver(true);
        handle.claim_interface(0)?;

        let transfer = unsafe { libusb::libusb_alloc_transfer(0) };
        if transfer.is_null() {
            return Err("libusb_alloc_transfer returned null".into());
        }

        Ok(Self {
            handle,
            transfer,
            buffer: Box::new([0u8; FRAME_BUFFER_BYTES]),
            completed: Box::new(AtomicI32::new(0)),
            seq: 0,
        })
    }

    pub fn send_frame(&mut self, pixel_bytes: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
        if pixel_bytes.len() != FRAME_PIXEL_BYTES {
            return Err(format!(
                "Expected {} bytes, got {}",
                FRAME_PIXEL_BYTES,
                pixel_bytes.len()
            )
            .into());
        }

        self.buffer[..4].copy_from_slice(FRAME_MAGIC);
        self.buffer[4] = self.seq;
        self.buffer[5..].copy_from_slice(pixel_bytes);

        // Pointers captured up front so the unsafe FFI call doesn't
        // straddle multiple borrows of self.
        let dev_handle = self.handle.as_raw();
        let ctx = self.handle.context().as_raw();
        let buf_ptr = self.buffer.as_mut_ptr();
        let buf_len = self.buffer.len() as c_int;
        let completed_ptr = &*self.completed as *const AtomicI32 as *mut c_void;
        let completed_int = completed_ptr as *mut c_int;

        self.completed.store(0, Ordering::Relaxed);

        unsafe {
            libusb::libusb_fill_bulk_transfer(
                self.transfer,
                dev_handle,
                BULK_OUT_ENDPOINT,
                buf_ptr,
                buf_len,
                transfer_callback,
                completed_ptr,
                WRITE_TIMEOUT_MS,
            );
            let r = libusb::libusb_submit_transfer(self.transfer);
            if r != 0 {
                return Err(format!("libusb_submit_transfer failed: {r}").into());
            }
            while self.completed.load(Ordering::Acquire) == 0 {
                let r = libusb::libusb_handle_events_completed(ctx, completed_int);
                if r != 0 {
                    return Err(format!("libusb_handle_events_completed failed: {r}").into());
                }
            }
            let status = (*self.transfer).status;
            if status != libusb::constants::LIBUSB_TRANSFER_COMPLETED {
                return Err(format!("USB transfer status {status}").into());
            }
        }
        self.seq = self.seq.wrapping_add(1);
        Ok(())
    }

    pub fn send_frame_rgb(&mut self, pixels: &[[u8; 3]]) -> Result<(), Box<dyn std::error::Error>> {
        if pixels.len() != WIDTH * HEIGHT {
            return Err(format!(
                "Expected {} pixels, got {}",
                WIDTH * HEIGHT,
                pixels.len()
            )
            .into());
        }
        let bytes: &[u8] =
            unsafe { std::slice::from_raw_parts(pixels.as_ptr() as *const u8, FRAME_PIXEL_BYTES) };
        self.send_frame(bytes)
    }
}

impl Drop for Hub75Client {
    fn drop(&mut self) {
        // Safe: send_frame only returns once the transfer has
        // completed, so no transfer is in flight when Drop runs.
        unsafe { libusb::libusb_free_transfer(self.transfer) };
    }
}

extern "system" fn transfer_callback(transfer: *mut libusb::libusb_transfer) {
    // SAFETY: user_data was populated with a pointer to an
    // `AtomicI32` inside a `Box` owned by the Hub75Client, which
    // outlives any in-flight transfer.
    unsafe {
        let completed = (*transfer).user_data as *const AtomicI32;
        (*completed).store(1, Ordering::Release);
    }
}

impl Hub75Client {
    /// List all attached HUB75 panels.
    pub fn list_panels() -> Result<Vec<PanelInfo>, Box<dyn std::error::Error>> {
        let mut panels = Vec::new();
        for device in rusb::devices()?.iter() {
            let desc = match device.device_descriptor() {
                Ok(d) => d,
                Err(_) => continue,
            };
            if desc.vendor_id() != USB_VID || desc.product_id() != USB_PID {
                continue;
            }
            let handle = match device.open() {
                Ok(h) => h,
                Err(_) => continue,
            };
            let manufacturer = handle
                .read_manufacturer_string_ascii(&desc)
                .unwrap_or_default();
            let product = handle.read_product_string_ascii(&desc).unwrap_or_default();
            if manufacturer != USB_MANUFACTURER || product != USB_PRODUCT {
                continue;
            }
            let serial = handle
                .read_serial_number_string_ascii(&desc)
                .unwrap_or_default();
            let _ = handle.set_auto_detach_kernel_driver(true);
            let available = handle.claim_interface(0).is_ok();
            if available {
                let _ = handle.release_interface(0);
            }
            panels.push(PanelInfo { serial, available });
        }
        Ok(panels)
    }
}

fn find_device(
    serial: Option<&str>,
) -> Result<rusb::Device<rusb::GlobalContext>, Box<dyn std::error::Error>> {
    for device in rusb::devices()?.iter() {
        let desc = match device.device_descriptor() {
            Ok(d) => d,
            Err(_) => continue,
        };
        if desc.vendor_id() != USB_VID || desc.product_id() != USB_PID {
            continue;
        }
        let handle = match device.open() {
            Ok(h) => h,
            Err(_) => continue,
        };
        let manufacturer = handle
            .read_manufacturer_string_ascii(&desc)
            .unwrap_or_default();
        let product = handle.read_product_string_ascii(&desc).unwrap_or_default();
        if manufacturer != USB_MANUFACTURER || product != USB_PRODUCT {
            continue;
        }
        if let Some(want) = serial {
            let got = handle
                .read_serial_number_string_ascii(&desc)
                .unwrap_or_default();
            if got != want {
                continue;
            }
        }
        return Ok(device);
    }
    match serial {
        Some(want) => Err(format!(
            "Could not find HUB75 display with serial {want:?} \
             (looking for VID {:04x} / PID {:04x}, manufacturer={USB_MANUFACTURER:?}, \
             product={USB_PRODUCT:?})",
            USB_VID, USB_PID
        )
        .into()),
        None => Err(format!(
            "Could not find HUB75 display (looking for VID {:04x} / PID {:04x}, \
             manufacturer={USB_MANUFACTURER:?}, product={USB_PRODUCT:?})",
            USB_VID, USB_PID
        )
        .into()),
    }
}
