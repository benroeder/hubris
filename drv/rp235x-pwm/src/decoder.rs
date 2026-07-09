// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Streaming audio decoders for the PWM player.
//!
//! Two small traits keep the player decoupled from both the byte transport and
//! the container format:
//!
//! - `ByteSource` yields raw file bytes (the SD-backed source lives in `sd.rs`).
//! - `Decoder` turns those bytes into a stream of mono i16 PCM samples.
//!
//! `WavDecoder` implements `Decoder` over a canonical 44-byte PCM WAV. It is
//! `no_std`, allocation-free, and holds only a small fixed read buffer, so a
//! future compressed decoder (e.g. an MP3 frame decoder) can slot in behind the
//! same `Decoder` trait without touching the player.

/// Errors returned while parsing a stream header.
#[derive(Copy, Clone, Debug)]
pub enum DecodeError {
    /// The stream ended before the full header could be read.
    Truncated,
    /// A required magic field (RIFF/WAVE/fmt/data) did not match.
    BadMagic,
    /// The stream is not PCM (format tag != 1) or not 16-bit, or has an
    /// unsupported channel count (only mono/stereo are accepted).
    Unsupported,
}

/// A source of raw bytes, pulled on demand. `read` fills as much of `out` as it
/// can and returns the count; a return of 0 means end-of-stream.
pub trait ByteSource {
    /// Read up to `out.len()` bytes into `out`. Returns the number of bytes
    /// read; 0 signals EOF.
    fn read(&mut self, out: &mut [u8]) -> usize;
}

/// A streaming PCM decoder: pulls bytes from an underlying `ByteSource` and
/// yields mono i16 samples.
pub trait Decoder {
    /// Sample rate of the decoded stream, in Hz.
    fn sample_rate(&self) -> u32;
    /// Channel count of the SOURCE stream (1 = mono, 2 = stereo). The decoder
    /// always downmixes to mono in `next_pcm`; this reports what it read. Part
    /// of the trait contract for future decoders; the player does not consume it
    /// today (it only needs the sample rate), hence the allow.
    #[allow(dead_code)]
    fn channels(&self) -> u8;
    /// Fill `out` with the next mono i16 samples. Returns the count written; a
    /// return of 0 means the stream is exhausted.
    fn next_pcm(&mut self, out: &mut [i16]) -> usize;
}

/// Size of the decoder's internal file-read buffer, in bytes. One 512-byte SD
/// block; kept small because the player only decodes a ring half at a time.
const READ_BUF_LEN: usize = 512;

/// Read exactly `out.len()` bytes from `src`, looping until the buffer is full
/// or the source hits EOF. Returns true only if every byte was read (a short
/// read means a truncated header). Used only for the fixed-size header parse.
fn read_exact<S: ByteSource>(src: &mut S, out: &mut [u8]) -> bool {
    let mut filled = 0;
    while filled < out.len() {
        let n = src.read(&mut out[filled..]);
        if n == 0 {
            return false;
        }
        filled += n;
    }
    true
}

/// Little-endian u16 from a 2-byte slice.
fn le_u16(b: &[u8]) -> u16 {
    u16::from_le_bytes([b[0], b[1]])
}

/// Little-endian u32 from a 4-byte slice.
fn le_u32(b: &[u8]) -> u32 {
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

/// Streaming decoder for a canonical 44-byte PCM WAV (the exact layout `wavgen`
/// writes: RIFF/WAVE, one `fmt ` chunk of 16 bytes, PCM tag 1, 16-bit, mono or
/// stereo, followed immediately by the `data` chunk). Downmixes stereo to mono
/// by averaging the two channels so the player is always fed a mono stream.
pub struct WavDecoder<S: ByteSource> {
    src: S,
    sample_rate: u32,
    channels: u8,
    /// Bytes of PCM `data` still to consume (decrements as samples are pulled).
    data_remaining: u32,
    /// Fixed read buffer holding the most recent chunk pulled from `src`.
    buf: [u8; READ_BUF_LEN],
    /// Number of valid bytes currently in `buf`.
    buf_len: usize,
    /// Read cursor into the valid region of `buf`.
    buf_pos: usize,
}

impl<S: ByteSource> WavDecoder<S> {
    /// Parse the 44-byte header off `src` and capture the format. Returns an
    /// error for a truncated header, a bad magic field, or an unsupported
    /// (non-PCM, non-16-bit, non-mono/stereo) format.
    pub fn new(mut src: S) -> Result<Self, DecodeError> {
        let mut hdr = [0u8; 44];
        if !read_exact(&mut src, &mut hdr) {
            return Err(DecodeError::Truncated);
        }
        if &hdr[0..4] != b"RIFF"
            || &hdr[8..12] != b"WAVE"
            || &hdr[12..16] != b"fmt "
        {
            return Err(DecodeError::BadMagic);
        }
        // fmt chunk: tag (PCM = 1), channels, sample rate, and bits/sample.
        let fmt_tag = le_u16(&hdr[20..22]);
        let channels = le_u16(&hdr[22..24]);
        let sample_rate = le_u32(&hdr[24..28]);
        let bits = le_u16(&hdr[34..36]);
        if fmt_tag != 1
            || bits != 16
            || !(channels == 1 || channels == 2)
        {
            return Err(DecodeError::Unsupported);
        }
        if &hdr[36..40] != b"data" {
            return Err(DecodeError::BadMagic);
        }
        let data_remaining = le_u32(&hdr[40..44]);

        Ok(Self {
            src,
            sample_rate,
            channels: channels as u8,
            data_remaining,
            buf: [0u8; READ_BUF_LEN],
            buf_len: 0,
            buf_pos: 0,
        })
    }

    /// Return the next raw byte from the buffered stream, refilling from `src`
    /// when the buffer is drained. `None` at end-of-stream.
    fn next_byte(&mut self) -> Option<u8> {
        if self.buf_pos == self.buf_len {
            self.buf_len = self.src.read(&mut self.buf);
            self.buf_pos = 0;
            if self.buf_len == 0 {
                return None;
            }
        }
        let b = self.buf[self.buf_pos];
        self.buf_pos += 1;
        Some(b)
    }

    /// Pull one little-endian i16 sample from the stream. `None` if the stream
    /// ends mid-sample.
    fn next_i16(&mut self) -> Option<i16> {
        let lo = self.next_byte()?;
        let hi = self.next_byte()?;
        Some(i16::from_le_bytes([lo, hi]))
    }
}

impl<S: ByteSource> Decoder for WavDecoder<S> {
    fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    fn channels(&self) -> u8 {
        self.channels
    }

    fn next_pcm(&mut self, out: &mut [i16]) -> usize {
        // One source frame is 2 bytes (mono) or 4 bytes (stereo). Stop at the
        // data-chunk length so trailing chunks (if any) are never decoded.
        let frame_bytes = 2 * self.channels as u32;
        let mut count = 0;
        while count < out.len() {
            if self.data_remaining < frame_bytes {
                break;
            }
            let sample = if self.channels == 2 {
                // Stereo: average L + R into one mono sample. Use i32 so the sum
                // cannot overflow before the halving.
                let (Some(l), Some(r)) = (self.next_i16(), self.next_i16())
                else {
                    break;
                };
                ((l as i32 + r as i32) / 2) as i16
            } else {
                match self.next_i16() {
                    Some(s) => s,
                    None => break,
                }
            };
            out[count] = sample;
            count += 1;
            self.data_remaining -= frame_bytes;
        }
        count
    }
}
