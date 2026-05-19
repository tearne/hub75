//! Vendor-class transport via `libusb` (through `rusb`).
//!
//! Per-frame work uses libusb's async transfer API with a single
//! reusable `libusb_transfer` allocated at startup. `rusb`'s safe
//! synchronous `write_bulk` allocates and frees a transfer per call,
//! which dominated the host-CPU cost of streaming pixels until we
//! switched to this manual-async path.

use std::os::raw::{c_int, c_void};
use std::sync::atomic::{AtomicI32, Ordering};

use rusb::ffi as libusb;
use rusb::UsbContext;

use crate::{PanelInfo, USB_MANUFACTURER, USB_PID, USB_PRODUCT, USB_VID};

const BULK_OUT_ENDPOINT: u8 = 0x01;
const BULK_IN_ENDPOINT: u8 = 0x81;
const WRITE_TIMEOUT_MS: u32 = 1000;

pub(crate) struct Transport {
    handle: rusb::DeviceHandle<rusb::GlobalContext>,
    transfer: *mut libusb::libusb_transfer,
    /// Set to 1 by the libusb completion callback. Polled by
    /// `libusb_handle_events_completed` and by us. Boxed so the
    /// pointer is stable across moves of `Transport`.
    completed: Box<AtomicI32>,
}

// rusb's DeviceHandle is Send. The raw pointers we hold (transfer,
// boxed completion flag) are stable for the lifetime of the Transport
// and only accessed through &mut self, so the type is safe to move
// between threads.
unsafe impl Send for Transport {}

impl Transport {
    pub(crate) fn open(serial: Option<&str>) -> Result<Self, Box<dyn std::error::Error>> {
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
            completed: Box::new(AtomicI32::new(0)),
        })
    }

    pub(crate) fn send_bytes(&mut self, buffer: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
        let dev_handle = self.handle.as_raw();
        let ctx = self.handle.context().as_raw();
        let buf_ptr = buffer.as_ptr() as *mut u8;
        let buf_len = buffer.len() as c_int;
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
        Ok(())
    }

    pub(crate) fn recv_event(
        &mut self,
        timeout: std::time::Duration,
    ) -> Result<Option<u8>, Box<dyn std::error::Error>> {
        let mut buf = [0u8; 1];
        match self.handle.read_bulk(BULK_IN_ENDPOINT, &mut buf, timeout) {
            Ok(n) if n == 1 => Ok(Some(buf[0])),
            Ok(_) => Ok(None),
            Err(rusb::Error::Timeout) => Ok(None),
            Err(e) => Err(Box::new(e)),
        }
    }

    pub(crate) fn list_panels() -> Result<Vec<PanelInfo>, Box<dyn std::error::Error>> {
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

impl Drop for Transport {
    fn drop(&mut self) {
        // Safe: send_bytes only returns once the transfer has completed,
        // so no transfer is in flight when Drop runs.
        unsafe { libusb::libusb_free_transfer(self.transfer) };
    }
}

extern "system" fn transfer_callback(transfer: *mut libusb::libusb_transfer) {
    // SAFETY: user_data was populated with a pointer to an `AtomicI32`
    // inside a `Box` owned by the Transport, which outlives any
    // in-flight transfer.
    unsafe {
        let completed = (*transfer).user_data as *const AtomicI32;
        (*completed).store(1, Ordering::Release);
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
