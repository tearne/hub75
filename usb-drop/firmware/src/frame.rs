//! JSON frame file parsing — hand-rolled to avoid putting a 12 KB
//! deserialized struct on the stack.
//!
//! Format (version 1):
//! ```json
//! { "version": 1, "width": 64, "height": 64,
//!   "pixels": [[r,g,b], [r,g,b], ...] }
//! ```
//! `pixels` is a flat row-major list of `width * height` triplets.
//! The parser is forgiving about whitespace and key order, but
//! requires all four keys to be present and a triplet count of
//! exactly `width * height`.

use crate::panel::{Rgb, HEIGHT, WIDTH};

#[derive(Debug, defmt::Format)]
pub enum ParseError {
    InvalidJson,
    MissingField,
    UnsupportedVersion,
    SizeMismatch,
    PixelCountMismatch,
    PixelChannelOutOfRange,
}

struct Parser<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Parser<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, pos: 0 }
    }

    fn peek(&self) -> Option<u8> {
        self.bytes.get(self.pos).copied()
    }

    fn bump(&mut self) -> Option<u8> {
        let b = self.peek()?;
        self.pos += 1;
        Some(b)
    }

    fn skip_ws(&mut self) {
        while let Some(b) = self.peek() {
            if matches!(b, b' ' | b'\t' | b'\r' | b'\n') {
                self.pos += 1;
            } else {
                break;
            }
        }
    }

    fn expect(&mut self, c: u8) -> Result<(), ParseError> {
        self.skip_ws();
        if self.bump() == Some(c) {
            Ok(())
        } else {
            Err(ParseError::InvalidJson)
        }
    }

    fn match_byte(&mut self, c: u8) -> bool {
        self.skip_ws();
        if self.peek() == Some(c) {
            self.pos += 1;
            true
        } else {
            false
        }
    }

    fn parse_str_into<'b>(&mut self, out: &'b mut [u8]) -> Result<&'b [u8], ParseError> {
        self.expect(b'"')?;
        let mut n = 0;
        loop {
            let b = self.bump().ok_or(ParseError::InvalidJson)?;
            if b == b'"' {
                return Ok(&out[..n]);
            }
            if b == b'\\' {
                // No escapes expected in our key set; treat as error.
                return Err(ParseError::InvalidJson);
            }
            if n >= out.len() {
                return Err(ParseError::InvalidJson);
            }
            out[n] = b;
            n += 1;
        }
    }

    fn parse_u32(&mut self) -> Result<u32, ParseError> {
        self.skip_ws();
        let mut v: u32 = 0;
        let mut any = false;
        while let Some(b) = self.peek() {
            if b.is_ascii_digit() {
                v = v
                    .checked_mul(10)
                    .and_then(|v| v.checked_add((b - b'0') as u32))
                    .ok_or(ParseError::InvalidJson)?;
                self.pos += 1;
                any = true;
            } else {
                break;
            }
        }
        if any {
            Ok(v)
        } else {
            Err(ParseError::InvalidJson)
        }
    }
}

/// Parse `bytes` and write the pixels into `out` (row-major, top-left
/// origin). On failure, `out` may be partially modified.
pub fn parse_into(bytes: &[u8], out: &mut [[Rgb; WIDTH]; HEIGHT]) -> Result<(), ParseError> {
    let mut p = Parser::new(bytes);
    p.expect(b'{')?;

    let mut version: Option<u32> = None;
    let mut width: Option<u32> = None;
    let mut height: Option<u32> = None;
    let mut pixels_seen = false;

    loop {
        p.skip_ws();
        if p.match_byte(b'}') {
            break;
        }

        let mut key_buf = [0u8; 16];
        let key = p.parse_str_into(&mut key_buf)?;
        p.expect(b':')?;
        p.skip_ws();

        match key {
            b"version" => version = Some(p.parse_u32()?),
            b"width" => width = Some(p.parse_u32()?),
            b"height" => height = Some(p.parse_u32()?),
            b"pixels" => {
                // Defer the pixels array until we've seen width/height,
                // but in JSON the order is arbitrary. Validate before
                // parsing — width/height must already be set.
                let w = width.ok_or(ParseError::MissingField)? as usize;
                let h = height.ok_or(ParseError::MissingField)? as usize;
                if w != WIDTH || h != HEIGHT {
                    return Err(ParseError::SizeMismatch);
                }
                parse_pixel_array(&mut p, out)?;
                pixels_seen = true;
            }
            _ => return Err(ParseError::InvalidJson),
        }

        p.skip_ws();
        if p.match_byte(b',') {
            continue;
        }
        p.expect(b'}')?;
        break;
    }

    let version = version.ok_or(ParseError::MissingField)?;
    if version != 1 {
        return Err(ParseError::UnsupportedVersion);
    }
    let _ = width.ok_or(ParseError::MissingField)?;
    let _ = height.ok_or(ParseError::MissingField)?;
    if !pixels_seen {
        return Err(ParseError::MissingField);
    }
    Ok(())
}

fn parse_pixel_array(
    p: &mut Parser<'_>,
    out: &mut [[Rgb; WIDTH]; HEIGHT],
) -> Result<(), ParseError> {
    p.expect(b'[')?;
    let mut idx: usize = 0;
    let total = WIDTH * HEIGHT;

    loop {
        p.skip_ws();
        if p.match_byte(b']') {
            break;
        }
        if idx >= total {
            return Err(ParseError::PixelCountMismatch);
        }

        p.expect(b'[')?;
        let r = p.parse_u32()?;
        p.expect(b',')?;
        let g = p.parse_u32()?;
        p.expect(b',')?;
        let b = p.parse_u32()?;
        p.expect(b']')?;
        if r > 255 || g > 255 || b > 255 {
            return Err(ParseError::PixelChannelOutOfRange);
        }

        let y = idx / WIDTH;
        let x = idx % WIDTH;
        out[y][x] = Rgb::new(r as u8, g as u8, b as u8);
        idx += 1;

        p.skip_ws();
        if p.match_byte(b',') {
            continue;
        }
        p.expect(b']')?;
        break;
    }

    if idx != WIDTH * HEIGHT {
        return Err(ParseError::PixelCountMismatch);
    }
    Ok(())
}
