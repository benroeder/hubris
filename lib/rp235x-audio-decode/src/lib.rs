// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]

//! Streaming audio decoders for the RP2350 PWM player, decoupled from the task
//! so the same no_std/no_alloc code can be tested on the host.
//!
//! Two small traits keep the player decoupled from both the byte transport and
//! the container format:
//!
//! - `ByteSource` yields raw file bytes (the SD-backed source lives in `sd.rs`).
//! - `Decoder` turns those bytes into a stream of interleaved L/R i16 PCM.
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

    /// True if a read ever failed with an IO error (as opposed to a clean EOF).
    /// A 0-byte `read` is ambiguous between EOF and error; this disambiguates so
    /// a truncated stream is not silently reported as complete.
    fn had_error(&self) -> bool {
        false
    }
}

/// A streaming PCM decoder: pulls bytes from an underlying `ByteSource` and
/// yields interleaved L/R i16 samples.
pub trait Decoder {
    /// Sample rate of the decoded stream, in Hz.
    fn sample_rate(&self) -> u32;
    /// Channel count of the SOURCE stream (1 = mono, 2 = stereo). `next_pcm`
    /// always emits interleaved stereo (a mono source is duplicated); this
    /// reports what was read. Part of the trait contract for future decoders; the
    /// player does not consume it today (it only needs the rate), hence the allow.
    #[allow(dead_code)]
    fn channels(&self) -> u8;
    /// Fill `out` with the next PCM samples as INTERLEAVED L, R pairs (a mono
    /// source is duplicated to both channels). Returns the number of i16 written
    /// (always even); a return of 0 means the stream is exhausted.
    fn next_pcm(&mut self, out: &mut [i16]) -> usize;

    /// True if the underlying byte source hit an IO error during decoding, i.e.
    /// the stream ended because of a fault rather than reaching the end.
    fn had_error(&self) -> bool {
        false
    }
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
/// stereo, followed immediately by the `data` chunk). Emits interleaved L/R;
/// a mono source is duplicated to both channels.
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
        if fmt_tag != 1 || bits != 16 || !(channels == 1 || channels == 2) {
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

    fn had_error(&self) -> bool {
        self.src.had_error()
    }

    fn next_pcm(&mut self, out: &mut [i16]) -> usize {
        // Fill `out` with INTERLEAVED L, R pairs. One source frame is 2 bytes
        // (mono) or 4 bytes (stereo); a mono source is duplicated to both
        // channels. Stop at the data-chunk length so trailing chunks (if any)
        // are never decoded.
        let frame_bytes = 2 * self.channels as u32;
        let mut count = 0;
        while count + 1 < out.len() {
            if self.data_remaining < frame_bytes {
                break;
            }
            let (l, r) = if self.channels == 2 {
                let (Some(l), Some(r)) = (self.next_i16(), self.next_i16())
                else {
                    break;
                };
                (l, r)
            } else {
                let Some(s) = self.next_i16() else {
                    break;
                };
                (s, s)
            };
            out[count] = l;
            out[count + 1] = r;
            count += 2;
            self.data_remaining -= frame_bytes;
        }
        count
    }
}

/// Streaming MP3 decoder built on the pure-Rust, no_std/no_alloc `nanomp3`
/// crate (a minimp3 port). Feeds compressed bytes from a `ByteSource` through a
/// sliding input window into `nanomp3::Decoder`, buffers the decoded f32 PCM of
/// one frame, and doles it out as interleaved L/R i16 to match the `Decoder`
/// contract (a mono frame is duplicated to both channels).
///
/// SEMANTICS (verified against nanomp3 0.1.1 / minimp3): `decode` writes
/// INTERLEAVED f32 into `pcm`, and `FrameInfo.samples_produced` is the count of
/// samples PER CHANNEL (minimp3's `mp3dec_decode_frame` returns
/// `hdr_frame_samples`, i.e. per-channel). So the number of valid f32 in `pcm`
/// after a frame is `samples_produced * channels`. For stereo, consecutive f32
/// are L, R, L, R, ...; we emit them as interleaved L/R i16 (mono is duplicated).
#[cfg(feature = "mp3")]
pub struct Nanomp3Decoder<S: ByteSource> {
    dec: nanomp3::Decoder,
    src: S,
    /// Sliding window of compressed MP3 bytes. `nanomp3` wants several frames
    /// resident at once (16 KiB recommended) to avoid artifacting; 4 KiB is a
    /// RAM-conscious compromise that still spans multiple frames.
    inbuf: [u8; MP3_IN_LEN],
    /// Number of valid bytes in `inbuf`.
    in_len: usize,
    /// Read cursor into the valid region of `inbuf` (bytes before this were
    /// consumed by the decoder).
    in_pos: usize,
    /// Decoded PCM of the current frame, interleaved f32 (must be >=
    /// `MAX_SAMPLES_PER_FRAME` or `decode` panics).
    pcm: [f32; MP3_MAX_SAMPLES],
    /// Count of valid interleaved f32 in `pcm` (= samples_produced * channels).
    pcm_len: usize,
    /// Read cursor into `pcm` (interleaved index).
    pcm_pos: usize,
    sample_rate: u32,
    channels: u8,
    /// Set once the byte source has returned EOF; no further refills are tried.
    io_done: bool,
}

/// Size of the compressed-MP3 sliding input window, in bytes.
#[cfg(feature = "mp3")]
const MP3_IN_LEN: usize = 4096;

/// Minimum PCM scratch length required by `nanomp3::decode` (it panics on a
/// smaller buffer). 2304 f32 = 1152 samples/channel * 2 channels.
#[cfg(feature = "mp3")]
const MP3_MAX_SAMPLES: usize = nanomp3::MAX_SAMPLES_PER_FRAME;

/// Low-water mark for the input window: refill only once the resident
/// (unconsumed) byte count drops below this, rather than on every frame. The
/// largest possible MPEG1 Layer III frame is 1441 bytes, so keeping at least 2
/// KiB resident guarantees a whole frame is always available to `decode` while
/// avoiding a memmove + read on each ~400-byte frame consumed.
#[cfg(feature = "mp3")]
const MP3_IN_LOWATER: usize = 2048;

/// Bytes to KEEP when force-skipping an unsyncable full window. If a FULL input
/// window yields no frame, its leading bytes are garbage (corrupt data, a non-
/// MP3 file, or an oversized tag minimp3 will not skip) and `refill_input` can
/// add nothing (no room), so we must drop bytes to make progress. We discard all
/// but this many trailing bytes -- kept > the 1441-byte max frame so a real sync
/// sitting near the window end survives to be confirmed against fresh data.
#[cfg(feature = "mp3")]
const MP3_RESYNC_KEEP: usize = 1536;

#[cfg(feature = "mp3")]
impl<S: ByteSource> Nanomp3Decoder<S> {
    /// Prime the input window and decode the first frame, capturing the format.
    /// `decode` returns `FrameInfo=None` while it skips junk / ID3 tags (still
    /// consuming bytes), and `Some` once a real frame lands; we loop, refilling
    /// as needed, until the first frame decodes. Returns `Unsupported` if the
    /// source runs dry before any frame is produced.
    ///
    /// `inline(never)`: this decoder struct is large (~20 KiB of buffers plus
    /// the minimp3 state), so inlining its construction into the caller would
    /// stack up multiple copies. Keeping it out-of-line bounds the stack.
    #[inline(never)]
    pub fn new(mut src: S) -> Result<Self, DecodeError> {
        // Prime the compressed-input window into a local buffer BEFORE the big
        // struct exists, then decode the first frame directly out of that local
        // to capture the format. Only after a frame lands do we build `Self` and
        // hand ownership of the primed buffer + decoder into it. This keeps the
        // ~30 KiB minimp3 decode scratch from ever stacking on top of a full
        // copy of this ~20 KiB struct, which bounds the pwm task stack.
        let mut inbuf = [0u8; MP3_IN_LEN];
        let mut in_len = 0usize;
        let mut in_pos = 0usize;
        let mut io_done = false;
        let mut dec = nanomp3::Decoder::new();

        // Local refill: slide the unconsumed tail down, then top up from src.
        let fill = |inbuf: &mut [u8; MP3_IN_LEN],
                    in_len: &mut usize,
                    in_pos: &mut usize,
                    io_done: &mut bool,
                    src: &mut S| {
            let tail = *in_len - *in_pos;
            if *in_pos > 0 {
                inbuf.copy_within(*in_pos..*in_len, 0);
                *in_pos = 0;
                *in_len = tail;
            }
            while *in_len < MP3_IN_LEN && !*io_done {
                let n = src.read(&mut inbuf[*in_len..]);
                if n == 0 {
                    *io_done = true;
                    break;
                }
                *in_len += n;
            }
        };

        fill(&mut inbuf, &mut in_len, &mut in_pos, &mut io_done, &mut src);

        // A scratch PCM buffer local to `new`. It is filled by the first decode;
        // the produced samples are copied into the struct's `pcm` so no audio is
        // dropped. Sharing one local buffer here avoids a second resident copy.
        let mut pcm = [0.0f32; MP3_MAX_SAMPLES];
        loop {
            // If the window drained without a frame yet -- e.g. a leading ID3v2
            // tag (album art) larger than the 4 KiB window, which minimp3 skips
            // by consuming the whole window with info == None -- refill and keep
            // scanning rather than giving up. Only report Unsupported once the
            // source is truly exhausted.
            if in_pos >= in_len {
                if io_done {
                    return Err(DecodeError::Unsupported);
                }
                fill(
                    &mut inbuf,
                    &mut in_len,
                    &mut in_pos,
                    &mut io_done,
                    &mut src,
                );
                continue;
            }
            let (consumed, info) = dec.decode(&inbuf[in_pos..in_len], &mut pcm);
            in_pos += consumed;
            if let Some(fi) = info {
                let channels = fi.channels.num();
                // samples_produced is per-channel; valid f32 = that * channels.
                let pcm_len = fi.samples_produced * channels as usize;
                return Ok(Self {
                    dec,
                    src,
                    inbuf,
                    in_len,
                    in_pos,
                    pcm,
                    pcm_len,
                    pcm_pos: 0,
                    sample_rate: fi.sample_rate,
                    channels,
                    io_done,
                });
            }
            // No frame yet (junk / ID3 skipped). Refill on no progress; give up
            // only once the source is drained and the window is empty.
            if consumed == 0 {
                fill(
                    &mut inbuf,
                    &mut in_len,
                    &mut in_pos,
                    &mut io_done,
                    &mut src,
                );
                if io_done && in_pos >= in_len {
                    return Err(DecodeError::Unsupported);
                }
                // Same unsyncable-full-window guard as decode_next_frame: drop
                // garbage (keep a max-frame tail) so a corrupt / non-MP3 file
                // eventually drains to Unsupported instead of hanging `new`.
                if in_pos == 0 && in_len == MP3_IN_LEN {
                    in_pos = MP3_IN_LEN - MP3_RESYNC_KEEP;
                }
            }
        }
    }

    /// Slide the unconsumed tail of `inbuf` to the front and read fresh bytes
    /// from `src` into the freed space. Sets `io_done` on a 0-byte read (EOF).
    fn refill_input(&mut self) {
        // Shift the still-unconsumed tail down to offset 0.
        let tail = self.in_len - self.in_pos;
        if self.in_pos > 0 {
            self.inbuf.copy_within(self.in_pos..self.in_len, 0);
            self.in_pos = 0;
            self.in_len = tail;
        }
        // Fill the remaining space, looping so a short read still tops up.
        while self.in_len < MP3_IN_LEN && !self.io_done {
            let n = self.src.read(&mut self.inbuf[self.in_len..]);
            if n == 0 {
                self.io_done = true;
                break;
            }
            self.in_len += n;
        }
    }

    /// Decode the next frame into `self.pcm`, refilling input as needed. Returns
    /// true if a frame was produced, false at true end-of-stream.
    fn decode_next_frame(&mut self) -> bool {
        loop {
            // Refill only when the resident window falls below the low-water
            // mark, so a single ~400-byte frame does not trigger a memmove +
            // read every call. Skip entirely once the source is drained.
            if !self.io_done && self.in_len - self.in_pos < MP3_IN_LOWATER {
                self.refill_input();
            }
            if self.in_pos >= self.in_len {
                return false;
            }
            let (consumed, info) = self
                .dec
                .decode(&self.inbuf[self.in_pos..self.in_len], &mut self.pcm);
            self.in_pos += consumed;
            if let Some(fi) = info {
                // Track channels per frame (normally constant across a stream)
                // so the interleaved stride below is always correct.
                self.channels = fi.channels.num();
                self.pcm_len = fi.samples_produced * self.channels as usize;
                self.pcm_pos = 0;
                return true;
            }
            // No frame and no progress: nothing left to decode from the window.
            if consumed == 0 {
                if self.io_done {
                    return false;
                }
                self.refill_input();
                // If the window is FULL and still yielded no frame, its leading
                // bytes are unsyncable and refill_input just added nothing (no
                // room) -- without dropping bytes this spins forever. Discard the
                // garbage, keeping a max-frame tail, so the next refill brings
                // fresh data to resync on. Guarantees forward progress.
                if self.in_pos == 0 && self.in_len == MP3_IN_LEN {
                    self.in_pos = MP3_IN_LEN - MP3_RESYNC_KEEP;
                }
            }
        }
    }

    /// Convert one f32 PCM sample in ~[-1.0, 1.0] to i16. `f32::clamp` is a core
    /// intrinsic (comparisons only, no libm) so it is safe here; MP3 output is
    /// never NaN, so the clamp's NaN behaviour is not a concern.
    fn f32_to_i16(s: f32) -> i16 {
        (s.clamp(-1.0, 1.0) * 32767.0) as i16
    }
}

#[cfg(feature = "mp3")]
impl<S: ByteSource> Decoder for Nanomp3Decoder<S> {
    fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    fn channels(&self) -> u8 {
        self.channels
    }

    fn had_error(&self) -> bool {
        self.src.had_error()
    }

    fn next_pcm(&mut self, out: &mut [i16]) -> usize {
        // Fill `out` with INTERLEAVED L, R i16 pairs; a mono frame is duplicated
        // to both channels.
        let mut count = 0;
        while count + 1 < out.len() {
            // Current frame drained: decode the next one (or stop at EOF).
            if self.pcm_pos >= self.pcm_len && !self.decode_next_frame() {
                break;
            }
            let (l, r) = if self.channels == 2 {
                // nanomp3 writes interleaved stereo f32: L, R, L, R, ...
                let l = self.pcm[self.pcm_pos];
                let r = self.pcm[self.pcm_pos + 1];
                self.pcm_pos += 2;
                (Self::f32_to_i16(l), Self::f32_to_i16(r))
            } else {
                let s = Self::f32_to_i16(self.pcm[self.pcm_pos]);
                self.pcm_pos += 1;
                (s, s)
            };
            out[count] = l;
            out[count + 1] = r;
            count += 2;
        }
        count
    }
}

/// Adapts a `ByteSource` (0 = EOF) to the fork's `ByteReader` (`Result<usize>`).
///
/// A clean end-of-stream is a 0-byte read, which the FLAC frame reader treats as
/// EOF at a frame boundary and as an `IoError` mid-frame; either way a genuine
/// source fault also surfaces to us as a decode error, which we record.
#[cfg(feature = "flac")]
struct ByteSourceReader<S: ByteSource> {
    src: S,
}

#[cfg(feature = "flac")]
impl<S: ByteSource> claxon_nostd::input::ByteReader for ByteSourceReader<S> {
    fn read(&mut self, out: &mut [u8]) -> claxon_nostd::Result<usize> {
        Ok(self.src.read(out))
    }
}

/// Largest channel count and per-channel block size the FLAC decoder will
/// accept. These bound the fixed planar decode buffer below (the no_alloc cap
/// lives here in the caller, exactly as the fork's `read_next_or_eof` expects).
/// 4608 is the FLAC "subset" maximum block size; only mono/stereo are decoded.
#[cfg(feature = "flac")]
const FLAC_MAX_CHANNELS: usize = 2;
/// Maximum per-channel block size (samples) the FLAC decoder will accept.
#[cfg(feature = "flac")]
const FLAC_MAX_BLOCK: usize = 4608;
/// Size of the planar decode buffer (all channels of one block).
#[cfg(feature = "flac")]
const FLAC_MAX_SAMPLES: usize = FLAC_MAX_BLOCK * FLAC_MAX_CHANNELS;

/// Streaming FLAC decoder built on the no_std/no_alloc `claxon-nostd` fork.
///
/// Owns the fork's `FlacReader` (over a `ByteSourceReader` adapter) plus one
/// fixed planar decode buffer. Each call to `decode_next_block` decodes a whole
/// FLAC block into that buffer (channel 0's samples, then channel 1's); `next_pcm`
/// then walks it emitting interleaved L/R i16 across as many calls as needed.
/// FLAC samples are `bits_per_sample`-bit integers, rescaled to 16-bit by a fixed
/// shift (identical to the reference conversion), so a 16-bit stream is bit-exact.
#[cfg(feature = "flac")]
pub struct FlacDecoder<S: ByteSource> {
    reader: claxon_nostd::FlacReader<ByteSourceReader<S>>,
    /// Planar decode buffer: `[ch0 samples ..., ch1 samples ...]`.
    buffer: [i32; FLAC_MAX_SAMPLES],
    /// Per-channel samples in the current block (0 before the first decode).
    block_size: usize,
    /// Per-channel read cursor into the current block, in `0..block_size`.
    pos: usize,
    sample_rate: u32,
    channels: u8,
    /// `bits_per_sample - 16`: >0 shifts right to 16-bit, <0 shifts left.
    /// Refreshed from each block's (frame-header) bits_per_sample, which is
    /// authoritative for that block and may differ from the streaminfo value.
    shift: i32,
    /// Set if a block failed to decode (a source fault or a corrupt stream),
    /// as opposed to reaching a clean end of stream.
    had_error: bool,
    /// Set once the stream has ended (cleanly or via error); stops further reads.
    done: bool,
}

#[cfg(feature = "flac")]
impl<S: ByteSource> FlacDecoder<S> {
    /// Read the FLAC header + metadata and capture the stream format. Returns
    /// `Unsupported` for >2 channels or a max block size beyond the fixed buffer,
    /// `BadMagic` for a malformed stream, and `Truncated` for a short stream.
    ///
    /// `inline(never)`: the decoder struct embeds a ~36 KiB decode buffer, so
    /// keeping construction out-of-line bounds the caller's stack.
    #[inline(never)]
    pub fn new(src: S) -> Result<Self, DecodeError> {
        let adapter = ByteSourceReader { src };

        // A small scratch is enough to walk and discard the metadata blocks; we
        // do not surface tags yet (the fork supports a streaming callback for
        // that when a "now playing" display needs it).
        let mut scratch = [0u8; 256];
        let reader = claxon_nostd::FlacReader::new_with_metadata(
            adapter,
            &mut scratch,
            |_| {},
        )
        .map_err(flac_err)?;

        let info = reader.streaminfo();
        if info.channels == 0 || info.channels as usize > FLAC_MAX_CHANNELS {
            return Err(DecodeError::Unsupported);
        }
        if info.max_block_size as usize > FLAC_MAX_BLOCK {
            return Err(DecodeError::Unsupported);
        }

        Ok(Self {
            reader,
            buffer: [0i32; FLAC_MAX_SAMPLES],
            block_size: 0,
            pos: 0,
            sample_rate: info.sample_rate,
            channels: info.channels as u8,
            shift: info.bits_per_sample as i32 - 16,
            had_error: false,
            done: false,
        })
    }

    /// Decode the next FLAC block into `self.buffer`. Returns true on a block,
    /// false at a clean end of stream or on a decode error (which sets flags).
    fn decode_next_block(&mut self) -> bool {
        if self.done {
            return false;
        }
        let mut frames = self.reader.blocks();
        match frames.read_next_or_eof(&mut self.buffer) {
            Ok(Some(block)) => {
                self.block_size = block.duration() as usize;
                self.channels = block.channels() as u8;
                // The frame header's bits_per_sample is authoritative for this
                // block; rescale to 16-bit accordingly rather than assuming the
                // streaminfo value.
                self.shift = block.bits_per_sample() as i32 - 16;
                self.pos = 0;
                if self.block_size == 0 {
                    self.done = true;
                    false
                } else {
                    true
                }
            }
            Ok(None) => {
                self.done = true;
                false
            }
            Err(_) => {
                self.had_error = true;
                self.done = true;
                false
            }
        }
    }

    /// Rescale one FLAC sample (`bits_per_sample`-bit signed) to 16-bit.
    fn scale(&self, v: i32) -> i16 {
        let s = if self.shift > 0 {
            v >> self.shift
        } else {
            v << (-self.shift)
        };
        s as i16
    }
}

#[cfg(feature = "flac")]
impl<S: ByteSource> Decoder for FlacDecoder<S> {
    fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    fn channels(&self) -> u8 {
        self.channels
    }

    fn had_error(&self) -> bool {
        // Either a decode error, or a fault reported by the byte source that the
        // fork saw as a bare 0-byte read (which it treats as clean EOF). Consult
        // the source directly so a mid-stream fault is not mistaken for the end
        // of the track, matching WavDecoder/Nanomp3Decoder.
        self.had_error || self.reader.get_ref().src.had_error()
    }

    fn next_pcm(&mut self, out: &mut [i16]) -> usize {
        // Fill `out` with INTERLEAVED L, R i16 pairs; a mono block is duplicated
        // to both channels. The block is stored planar, so the right channel of
        // sample `pos` lives at `block_size + pos`.
        let mut count = 0;
        while count + 1 < out.len() {
            if self.pos >= self.block_size && !self.decode_next_block() {
                break;
            }
            let (l, r) = if self.channels == 2 {
                let l = self.buffer[self.pos];
                let r = self.buffer[self.block_size + self.pos];
                (self.scale(l), self.scale(r))
            } else {
                let s = self.scale(self.buffer[self.pos]);
                (s, s)
            };
            self.pos += 1;
            out[count] = l;
            out[count + 1] = r;
            count += 2;
        }
        count
    }
}

/// Maps a fork decode error onto the crate's `DecodeError`.
#[cfg(feature = "flac")]
fn flac_err(e: claxon_nostd::Error) -> DecodeError {
    match e {
        // A read failure or an unexpected end of the byte source.
        claxon_nostd::Error::IoError => DecodeError::Truncated,
        // A currently unsupported FLAC feature (e.g. unencoded-binary residuals).
        claxon_nostd::Error::Unsupported(_) => DecodeError::Unsupported,
        // An ill-formed stream (bad magic, bad header, ...).
        claxon_nostd::Error::FormatError(_) => DecodeError::BadMagic,
    }
}

/// True if `name` (a FAT 8.3 short name, ASCII) has the FLAC extension.
///
/// FAT 8.3 extensions are at most three characters, so a `CLIP.FLAC` file is
/// stored with a short name ending in `.FLA` (e.g. `CLIP~1.FLA`). Match that
/// truncated extension case-insensitively so the SD path routes it correctly.
#[cfg(feature = "flac")]
pub fn is_flac_name(name: &[u8]) -> bool {
    let n = name.len();
    if n < 4 {
        return false;
    }
    let ext = &name[n - 4..];
    ext[0] == b'.'
        && ext[1].eq_ignore_ascii_case(&b'F')
        && ext[2].eq_ignore_ascii_case(&b'L')
        && ext[3].eq_ignore_ascii_case(&b'A')
}

/// Static (no `dyn`, no alloc) dispatch over the supported container formats.
/// The player is generic over `Decoder`, so wrapping the two concrete decoders
/// in one enum lets `play_file` / `play_mix` pick a format at open time by file
/// extension without changing the play/mix loops.
///
/// The MP3 variant is far larger than the WAV one (its decode buffers dominate),
/// but this is a `no_alloc` target: boxing the large field -- clippy's usual fix
/// -- is not available, and only one `AnyDecoder` is ever live at a time, so the
/// size asymmetry is intentional and harmless here.
#[cfg(any(feature = "mp3", feature = "flac"))]
#[allow(clippy::large_enum_variant)]
pub enum AnyDecoder<S: ByteSource> {
    /// A canonical 16-bit PCM WAV.
    Wav(WavDecoder<S>),
    /// An MP3 stream (via `nanomp3`).
    #[cfg(feature = "mp3")]
    Mp3(Nanomp3Decoder<S>),
    /// A FLAC stream (via the `claxon-nostd` fork).
    #[cfg(feature = "flac")]
    Flac(FlacDecoder<S>),
}

#[cfg(any(feature = "mp3", feature = "flac"))]
impl<S: ByteSource> Decoder for AnyDecoder<S> {
    fn sample_rate(&self) -> u32 {
        match self {
            AnyDecoder::Wav(d) => d.sample_rate(),
            #[cfg(feature = "mp3")]
            AnyDecoder::Mp3(d) => d.sample_rate(),
            #[cfg(feature = "flac")]
            AnyDecoder::Flac(d) => d.sample_rate(),
        }
    }

    fn channels(&self) -> u8 {
        match self {
            AnyDecoder::Wav(d) => d.channels(),
            #[cfg(feature = "mp3")]
            AnyDecoder::Mp3(d) => d.channels(),
            #[cfg(feature = "flac")]
            AnyDecoder::Flac(d) => d.channels(),
        }
    }

    fn had_error(&self) -> bool {
        match self {
            AnyDecoder::Wav(d) => d.had_error(),
            #[cfg(feature = "mp3")]
            AnyDecoder::Mp3(d) => d.had_error(),
            #[cfg(feature = "flac")]
            AnyDecoder::Flac(d) => d.had_error(),
        }
    }

    fn next_pcm(&mut self, out: &mut [i16]) -> usize {
        match self {
            AnyDecoder::Wav(d) => d.next_pcm(out),
            #[cfg(feature = "mp3")]
            AnyDecoder::Mp3(d) => d.next_pcm(out),
            #[cfg(feature = "flac")]
            AnyDecoder::Flac(d) => d.next_pcm(out),
        }
    }
}

/// True if `name` (an 8.3 short name, ASCII) ends in the `.mp3` extension,
/// compared case-insensitively (FAT short names are upper-cased, but be
/// defensive). Used to route the file to the MP3 decoder instead of WAV.
#[cfg(feature = "mp3")]
pub fn is_mp3_name(name: &[u8]) -> bool {
    let n = name.len();
    if n < 4 {
        return false;
    }
    let ext = &name[n - 4..];
    ext[0] == b'.'
        && ext[1].eq_ignore_ascii_case(&b'M')
        && ext[2].eq_ignore_ascii_case(&b'P')
        && ext[3] == b'3'
}
