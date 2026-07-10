//! Host decoder harness: `audio-decode-host <in.wav|mp3|flac> <out.wav>`.
//!
//! Runs the device's `rp235x-audio-decode` (no_std) over an in-memory copy of
//! the input file and writes the decoded interleaved-stereo PCM as a 16-bit WAV.
//! Compare that WAV against ffmpeg's decode to validate a codec without flashing.

use rp235x_audio_decode::{
    AnyDecoder, ByteSource, Decoder, FlacDecoder, Nanomp3Decoder, WavDecoder,
};

/// A `ByteSource` over an in-memory byte slice (the whole input file).
struct SliceSource {
    data: Vec<u8>,
    pos: usize,
}

impl ByteSource for SliceSource {
    fn read(&mut self, out: &mut [u8]) -> usize {
        let mut n = (self.data.len() - self.pos).min(out.len());
        // CHUNK=<bytes> caps each read to simulate an SD/block source that
        // returns short reads, exercising the decoder's partial-read handling
        // (the on-device SdFileSource behaves this way; SliceSource otherwise
        // always returns full reads).
        if let Ok(chunk) = std::env::var("CHUNK") {
            if let Ok(c) = chunk.parse::<usize>() {
                if c > 0 {
                    n = n.min(c);
                }
            }
        }
        out[..n].copy_from_slice(&self.data[self.pos..self.pos + n]);
        self.pos += n;
        n
    }
}

/// Reference FLAC decode with stock claxon (std). Writes a 16-bit WAV so we can
/// A/B it against ffmpeg and against the future no_std FlacDecoder.
fn decode_flac_reference(data: Vec<u8>, outfile: &str) {
    let mut r = claxon::FlacReader::new(std::io::Cursor::new(data)).expect("flac header");
    let info = r.streaminfo();
    let (rate, channels, bits) = (info.sample_rate, info.channels, info.bits_per_sample);
    let shift = bits as i32 - 16; // scale to 16-bit
    let spec = hound::WavSpec {
        channels: channels as u16,
        sample_rate: rate,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    let mut writer = hound::WavWriter::create(outfile, spec).expect("create wav");
    let mut total = 0u64;
    for s in r.samples() {
        let v = s.expect("flac sample");
        let v16 = if shift > 0 { v >> shift } else { v << (-shift) };
        writer.write_sample(v16 as i16).expect("write");
        total += 1;
    }
    writer.finalize().expect("finalize");
    println!(
        "flac(claxon-ref) -> {}: {} Hz, {} ch, {}-bit, {} samples",
        outfile, rate, channels, bits, total
    );
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: audio-decode-host <in.(wav|mp3)> <out.wav>");
        std::process::exit(2);
    }
    let (infile, outfile) = (&args[1], &args[2]);
    let data = std::fs::read(infile).expect("read input");

    let lower = infile.to_ascii_lowercase();
    let is_flac = lower.ends_with(".flac");
    let is_mp3 = lower.ends_with(".mp3");

    // A/B escape hatch: FLAC_REF=1 decodes .flac with STOCK claxon (std) instead
    // of our no_std fork, so the fork's output can be compared against a
    // known-good reference as well as against ffmpeg.
    if is_flac && std::env::var("FLAC_REF").is_ok() {
        decode_flac_reference(data, outfile);
        return;
    }

    let src = SliceSource { data, pos: 0 };
    let mut dec = if is_flac {
        AnyDecoder::Flac(FlacDecoder::new(src).expect("flac header"))
    } else if is_mp3 {
        AnyDecoder::Mp3(Nanomp3Decoder::new(src).expect("mp3 header"))
    } else {
        AnyDecoder::Wav(WavDecoder::new(src).expect("wav header"))
    };

    let rate = Decoder::sample_rate(&dec);
    // The Decoder contract emits INTERLEAVED L/R, so the output is always stereo.
    let spec = hound::WavSpec {
        channels: 2,
        sample_rate: rate,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    let mut writer = hound::WavWriter::create(outfile, spec).expect("create wav");

    let mut buf = [0i16; 4096];
    let mut total = 0u64;
    loop {
        let n = dec.next_pcm(&mut buf);
        if n == 0 {
            break;
        }
        for &s in &buf[..n] {
            writer.write_sample(s).expect("write sample");
        }
        total += n as u64;
    }
    writer.finalize().expect("finalize");
    println!(
        "decoded {} -> {}: {} Hz, {} stereo frames{}",
        infile,
        outfile,
        rate,
        total / 2,
        if Decoder::had_error(&dec) {
            " (TRUNCATED: source error)"
        } else {
            ""
        }
    );
}
