//! Host-side client for sending frames to the HUB75 display over USB serial.
//!
//! # Example
//!
//! ```no_run
//! use hub75_client::Hub75Client;
//!
//! let mut client = Hub75Client::open_auto()?;
//!
//! // Solid red frame
//! let frame = vec![[255, 0, 0]; 64 * 64];
//! client.send_frame_rgb(&frame)?;
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

use std::io::Write;
use std::time::Duration;

pub const WIDTH: usize = 64;
pub const HEIGHT: usize = 64;
pub const FRAME_MAGIC: &[u8; 4] = b"HB75";
pub const FRAME_PIXEL_BYTES: usize = WIDTH * HEIGHT * 3;

const USB_MANUFACTURER: &str = "tearne";
const USB_PRODUCT: &str = "hub75";

pub struct Hub75Client {
    port: Box<dyn serialport::SerialPort>,
    seq: u8,
}

impl Hub75Client {
    /// Open a specific serial port by path.
    pub fn open(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let port = serialport::new(path, 115_200)
            .timeout(Duration::from_secs(1))
            .open()?;
        // Give the device a moment to initialise after connection
        std::thread::sleep(Duration::from_millis(500));
        Ok(Self { port, seq: 0 })
    }

    /// Auto-detect and open the HUB75 display's USB serial port.
    pub fn open_auto() -> Result<Self, Box<dyn std::error::Error>> {
        let path = find_port().ok_or("Could not find HUB75 Display device")?;
        Self::open(&path)
    }

    /// Send a frame of raw RGB bytes (must be exactly 12288 bytes).
    pub fn send_frame(&mut self, pixel_bytes: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
        if pixel_bytes.len() != FRAME_PIXEL_BYTES {
            return Err(format!(
                "Expected {} bytes, got {}",
                FRAME_PIXEL_BYTES,
                pixel_bytes.len()
            )
            .into());
        }

        let mut header = [0u8; 5];
        header[..4].copy_from_slice(FRAME_MAGIC);
        header[4] = self.seq;

        self.port.write_all(&header)?;
        self.port.write_all(pixel_bytes)?;
        self.port.flush()?;
        self.seq = self.seq.wrapping_add(1);
        Ok(())
    }

    /// Send a frame from a slice of `[r, g, b]` arrays (must be 4096 pixels).
    pub fn send_frame_rgb(&mut self, pixels: &[[u8; 3]]) -> Result<(), Box<dyn std::error::Error>> {
        if pixels.len() != WIDTH * HEIGHT {
            return Err(format!(
                "Expected {} pixels, got {}",
                WIDTH * HEIGHT,
                pixels.len()
            )
            .into());
        }
        // Safe: [u8; 3] has the same layout as 3 contiguous u8s
        let bytes: &[u8] =
            unsafe { std::slice::from_raw_parts(pixels.as_ptr() as *const u8, FRAME_PIXEL_BYTES) };
        self.send_frame(bytes)
    }
}

/// Find the serial port path for the HUB75 display by USB product string.
pub fn find_port() -> Option<String> {
    serialport::available_ports().ok()?.into_iter().find_map(|p| {
        if let serialport::SerialPortType::UsbPort(info) = &p.port_type {
            if info.manufacturer.as_deref() == Some(USB_MANUFACTURER)
                && info.product.as_deref() == Some(USB_PRODUCT)
            {
                return Some(p.port_name);
            }
        }
        None
    })
}
