//! Host decoder harness: `audio-decode-host <in.wav|mp3|flac> <out.wav>`.
//!
//! Runs the device's `rp235x-audio-decode` (no_std) over an in-memory copy of
//! the input file and writes the decoded interleaved-stereo PCM as a 16-bit WAV.
//! Compare that WAV against ffmpeg's decode to validate a codec without flashing.

use rp235x_audio_decode::{AnyDecoder, ByteSource, Decoder, Nanomp3Decoder, WavDecoder};

/// A `ByteSource` over an in-memory byte slice (the whole input file).
struct SliceSource {
    data: Vec<u8>,
    pos: usize,
}

impl ByteSource for SliceSource {
    fn read(&mut self, out: &mut [u8]) -> usize {
        let n = (self.data.len() - self.pos).min(out.len());
        out[..n].copy_from_slice(&self.data[self.pos..self.pos + n]);
        self.pos += n;
        n
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: audio-decode-host <in.(wav|mp3)> <out.wav>");
        std::process::exit(2);
    }
    let (infile, outfile) = (&args[1], &args[2]);
    let data = std::fs::read(infile).expect("read input");
    let src = SliceSource { data, pos: 0 };

    let is_mp3 = infile.to_ascii_lowercase().ends_with(".mp3");
    let mut dec = if is_mp3 {
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
