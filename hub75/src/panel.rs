//! Panel abstraction shared across HUB75 panel families.

#[derive(Copy, Clone)]
pub struct Rgb {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Rgb {
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    pub const BLACK: Rgb = Rgb::new(0, 0, 0);
}

/// A HUB75 panel.
///
/// Pixel data lives inside the panel. The caller borrows it mutably via
/// [`frame_mut`], writes pixel values in place, then calls [`commit`] to
/// release the borrow and trigger the pack pipeline. Geometry is exposed
/// via the [`WIDTH`] and [`HEIGHT`] constants and the [`Frame`] type.
#[allow(async_fn_in_trait)]
pub trait Panel {
    /// Concrete frame buffer type — typically `[[Rgb; WIDTH]; HEIGHT]`.
    type Frame: ?Sized;

    const WIDTH: usize;
    const HEIGHT: usize;

    /// Borrow the panel's pixel buffer for in-place edits. Awaits any
    /// pack from a previous [`commit`] before returning, so the borrow
    /// is exclusive of the packer.
    async fn frame_mut(&mut self) -> &mut Self::Frame;

    /// Release the frame borrow and kick the pack pipeline. The newly
    /// packed frame becomes visible at the next display-frame boundary.
    fn commit(&mut self);
}
