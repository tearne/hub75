//! The single error type returned across the public API.

use std::fmt;

/// Convenience alias — every fallible client operation returns this.
pub type Result<T> = std::result::Result<T, Error>;

/// What went wrong talking to a panel.
///
/// A closed enum, so callers can match exhaustively and choose a policy —
/// notably telling a transient [`Error::Disconnected`] (worth a
/// [`reconnect`](crate::Hub75Client::reconnect)) apart from a fatal
/// [`Error::InvalidFrame`] (a caller bug). The transport layer maps its
/// own per-OS, per-backend failures onto these variants so consumers never
/// see a raw `libusb` status code or `io::Error`.
#[derive(Debug)]
pub enum Error {
    /// No usable panel is present — none found at open, or the device went
    /// away mid-use. Reconnectable.
    Disconnected,
    /// The operation didn't complete within the transport's timeout.
    Timeout,
    /// The OS refused access: a missing udev rule (vendor transport) or the
    /// user not being in the `dialout` group (CDC transport).
    PermissionDenied,
    /// Another process currently holds the panel.
    Busy,
    /// A wrong-sized frame was passed to a send call. A caller bug — never
    /// worth retrying.
    InvalidFrame { expected: usize, got: usize },
    /// Anything not covered above, preserved as a human-readable message.
    Other(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Disconnected => write!(f, "no panel connected"),
            Error::Timeout => write!(f, "operation timed out"),
            Error::PermissionDenied => write!(
                f,
                "permission denied — check the udev rule (vendor) or that you are \
                 in the `dialout` group (CDC)"
            ),
            Error::Busy => write!(f, "panel is held by another process"),
            Error::InvalidFrame { expected, got } => {
                write!(f, "wrong frame size: expected {expected}, got {got}")
            }
            Error::Other(message) => write!(f, "{message}"),
        }
    }
}

impl std::error::Error for Error {}
