// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Interactive command shell over the USB console.
//!
//! Owns no hardware: it is a pure IPC client of the USB console transport
//! (`task-rp235x-usb`) and of every peripheral Idol driver, so a parser bug
//! here cannot take out USB, and jefe can restart the shell freely.
//!
//! Attach with `screen /dev/cu.usbmodemHUBRIS_0001` and type `help`. Line
//! editing is minimal: echo, backspace, CR or LF submits. `status` runs the
//! self-tests the old logdemo task ran every second (UART loopback, SPI
//! internal loopback, I2C bus scan), on demand instead of spamming.

#![no_std]
#![no_main]

use drv_rp235x_flash_api::Rp235xFlash;
use drv_rp235x_gpio_api::Rp235xGpio;
use drv_rp235x_i2c_api::Rp235xI2c;
use drv_rp235x_spi_api::Rp235xSpi;
use drv_rp235x_uart_api::Rp235xUart;
use task_rp235x_usb_api::UsbCons;
use userlib::{hl, sys_get_timer, task_slot};

task_slot!(USB, usb);
task_slot!(GPIO, gpio_driver);
task_slot!(UART, uart_driver);
task_slot!(SPI, spi_driver);
task_slot!(I2C, i2c_driver);
task_slot!(FLASH, flash_driver);

/// Pico 2 onboard LED, the `led` command's target.
const LED_PIN: u8 = 25;

const PROMPT: &[u8] = b"hubris> ";
const HELP: &[u8] = b"commands:\r\n\
  help                  this text\r\n\
  status                run self-tests (uart loopback, spi loopback, i2c scan)\r\n\
  ticks                 ms since boot\r\n\
  led on|off|toggle|blink   onboard LED (on/off/toggle suspend the\r\n\
                            idle heartbeat; blink restores it)\r\n\
  gpio out|in|hi|lo|toggle|read <pin>\r\n\
  uart send <text>      send out UART0 TX (GP0)\r\n\
  uart recv             drain UART0 RX buffer\r\n\
  spi xfer <hex..>      full-duplex exchange, e.g. spi xfer a5 5a 3c\r\n\
  i2c scan              probe all 7-bit addresses\r\n\
  i2c read <addr> <n>   read n bytes, e.g. i2c read 42 8\r\n\
  i2c write <addr> <hex..>\r\n\
  flash read <hex-off> [n<=64]   dump flash, e.g. flash read 0 64\r\n\
  rom <CC>              boot-ROM table lookup, e.g. rom FO\r\n";

struct Shell {
    usb: UsbCons,
    gpio: Rp235xGpio,
    uart: Rp235xUart,
    spi: Rp235xSpi,
    i2c: Rp235xI2c,
    flash: Rp235xFlash,
    out: Out,
    /// Idle-loop LED heartbeat; `led on|off|toggle` takes manual control of
    /// the LED (turns this off), `led blink` gives it back.
    heartbeat: bool,
}

/// Small write-combining buffer so replies go to the USB task in a few IPCs
/// (and thus a few CDC packets) instead of one per fragment.
struct Out {
    usb: UsbCons,
    buf: [u8; 256],
    len: usize,
}

impl Out {
    fn put(&mut self, s: &[u8]) {
        for &b in s {
            if self.len == self.buf.len() {
                self.flush();
            }
            self.buf[self.len] = b;
            self.len += 1;
        }
    }

    fn put_u32(&mut self, n: u32) {
        self.put_u64(n as u64);
    }

    fn put_u64(&mut self, n: u64) {
        let mut tmp = [0u8; 20];
        let mut i = tmp.len();
        let mut m = n;
        if m == 0 {
            i -= 1;
            tmp[i] = b'0';
        }
        while m > 0 {
            i -= 1;
            tmp[i] = b'0' + (m % 10) as u8;
            m /= 10;
        }
        self.put(&tmp[i..]);
    }

    fn put_hex_byte(&mut self, b: u8) {
        const D: &[u8; 16] = b"0123456789abcdef";
        self.put(&[D[(b >> 4) as usize], D[(b & 0xf) as usize]]);
    }

    fn put_hex32(&mut self, n: u32) {
        for b in n.to_be_bytes() {
            self.put_hex_byte(b);
        }
    }

    fn flush(&mut self) {
        if self.len > 0 {
            self.usb.write(&self.buf[..self.len]);
            self.len = 0;
        }
    }
}

impl Shell {
    fn dispatch(&mut self, line: &str) {
        let mut words = line.split_whitespace();
        let Some(cmd) = words.next() else { return };
        match cmd {
            "help" => self.out.put(HELP),
            "status" => self.cmd_status(),
            "ticks" => {
                self.out.put(b"uptime ");
                self.out.put_u64(sys_get_timer().now);
                self.out.put(b" ms\r\n");
            }
            "led" => self.cmd_led(words.next()),
            "gpio" => self.cmd_gpio(words.next(), words.next()),
            "uart" => self.cmd_uart(line, words.next()),
            "spi" => self.cmd_spi(line, words.next()),
            "i2c" => self.cmd_i2c(words.next(), words.next(), words.next()),
            "flash" => self.cmd_flash(words.next(), words.next(), words.next()),
            "rom" => self.cmd_rom(words.next()),
            _ => {
                self.out.put(b"unknown command: ");
                self.out.put(cmd.as_bytes());
                self.out.put(b" (try `help`)\r\n");
            }
        }
    }

    /// The old logdemo self-tests, on demand.
    fn cmd_status(&mut self) {
        // UART: marker out TX, drain RX; with the GP0->GP1 jumper this echoes.
        const MARKER: &[u8] = b"uart-loopback\r\n";
        self.uart.write(MARKER);
        hl::sleep_for(5);
        let mut rx = [0u8; 32];
        let n = self.uart.read(&mut rx);
        self.out.put(b"uart: rx=");
        self.out.put_u32(n as u32);
        self.out.put(if n == MARKER.len() {
            b" (loopback OK)\r\n" as &[u8]
        } else if n == 0 {
            b" (no loopback jumper?)\r\n"
        } else {
            b"\r\n"
        });

        // SPI: 3 bytes through the PL022 internal loopback.
        let tx = [0xA5u8, 0x5A, 0x3C];
        let mut srx = [0u8; 3];
        let _ = self.spi.exchange(&tx, &mut srx);
        self.out.put(if srx == tx {
            b"spi:  loopback OK\r\n" as &[u8]
        } else {
            b"spi:  loopback FAILED\r\n"
        });

        // I2C: full bus scan.
        self.i2c_scan();
    }

    fn cmd_led(&mut self, verb: Option<&str>) {
        let _ = self.gpio.configure_output(LED_PIN);
        // Manual LED control suspends the idle heartbeat (which would
        // otherwise toggle the LED right back within ~500 ms).
        let r = match verb {
            Some("on") => {
                self.heartbeat = false;
                self.gpio.set_high(LED_PIN)
            }
            Some("off") => {
                self.heartbeat = false;
                self.gpio.set_low(LED_PIN)
            }
            Some("toggle") | None => {
                self.heartbeat = false;
                self.gpio.toggle(LED_PIN)
            }
            Some("blink") => {
                self.heartbeat = true;
                Ok(())
            }
            _ => {
                self.out.put(b"usage: led on|off|toggle|blink\r\n");
                return;
            }
        };
        self.out.put(if r.is_ok() { b"ok\r\n" as &[u8] } else { b"error\r\n" });
    }

    fn cmd_gpio(&mut self, verb: Option<&str>, pin: Option<&str>) {
        let (Some(verb), Some(pin)) = (verb, pin) else {
            self.out.put(b"usage: gpio out|in|hi|lo|toggle|read <pin>\r\n");
            return;
        };
        let Ok(pin) = pin.parse::<u8>() else {
            self.out.put(b"bad pin\r\n");
            return;
        };
        let r = match verb {
            "out" => self.gpio.configure_output(pin),
            "in" => self.gpio.configure_input(pin),
            "hi" => self.gpio.set_high(pin),
            "lo" => self.gpio.set_low(pin),
            "toggle" => self.gpio.toggle(pin),
            "read" => match self.gpio.read(pin) {
                Ok(v) => {
                    self.out.put(b"pin ");
                    self.out.put_u32(pin as u32);
                    self.out.put(if v != 0 { b" = 1\r\n" as &[u8] } else { b" = 0\r\n" });
                    return;
                }
                Err(e) => Err(e),
            },
            _ => {
                self.out.put(b"usage: gpio out|in|hi|lo|toggle|read <pin>\r\n");
                return;
            }
        };
        self.out.put(if r.is_ok() { b"ok\r\n" as &[u8] } else { b"error (bad pin?)\r\n" });
    }

    fn cmd_uart(&mut self, line: &str, verb: Option<&str>) {
        match verb {
            Some("send") => {
                // Everything after "uart send " goes out verbatim + CRLF.
                let text = subcommand_rest(line, "send");
                self.uart.write(text.as_bytes());
                self.uart.write(b"\r\n");
                self.out.put(b"sent ");
                self.out.put_u32(text.len() as u32 + 2);
                self.out.put(b" bytes\r\n");
            }
            Some("recv") => {
                let mut rx = [0u8; 64];
                let n = self.uart.read(&mut rx);
                self.out.put(b"rx ");
                self.out.put_u32(n as u32);
                self.out.put(b" bytes");
                if n > 0 {
                    self.out.put(b": ");
                    for &b in &rx[..n] {
                        if (0x20..0x7f).contains(&b) {
                            self.out.put(&[b]);
                        } else {
                            self.out.put(b"\\x");
                            self.out.put_hex_byte(b);
                        }
                    }
                }
                self.out.put(b"\r\n");
            }
            _ => self.out.put(b"usage: uart send <text> | uart recv\r\n"),
        }
    }

    fn cmd_spi(&mut self, line: &str, verb: Option<&str>) {
        if verb != Some("xfer") {
            self.out.put(b"usage: spi xfer <hex bytes>\r\n");
            return;
        }
        let mut tx = [0u8; 32];
        let Some(n) = parse_hex_bytes(subcommand_rest(line, "xfer"), &mut tx)
        else {
            self.out.put(b"bad hex (e.g. spi xfer a5 5a 3c)\r\n");
            return;
        };
        if n == 0 {
            self.out.put(b"usage: spi xfer <hex bytes>\r\n");
            return;
        }
        let mut rx = [0u8; 32];
        self.spi.exchange(&tx[..n], &mut rx[..n]);
        self.out.put(b"rx:");
        for &b in &rx[..n] {
            self.out.put(b" ");
            self.out.put_hex_byte(b);
        }
        self.out.put(b"\r\n");
    }

    fn cmd_i2c(
        &mut self,
        verb: Option<&str>,
        arg1: Option<&str>,
        arg2: Option<&str>,
    ) {
        match verb {
            Some("scan") => self.i2c_scan(),
            Some("read") => {
                let (Some(addr), Some(count)) = (
                    arg1.and_then(|a| u8::from_str_radix(a, 16).ok()),
                    arg2.and_then(|c| c.parse::<usize>().ok()),
                ) else {
                    self.out.put(b"usage: i2c read <hex-addr> <count<=32>\r\n");
                    return;
                };
                let count = count.min(32);
                let mut buf = [0u8; 32];
                match self.i2c.read(addr, &mut buf[..count]) {
                    Ok(n) => {
                        self.out.put(b"rx:");
                        for &b in &buf[..n] {
                            self.out.put(b" ");
                            self.out.put_hex_byte(b);
                        }
                        self.out.put(b"\r\n");
                    }
                    Err(_) => self.out.put(b"error (nak/timeout)\r\n"),
                }
            }
            Some("write") => {
                let Some(addr) =
                    arg1.and_then(|a| u8::from_str_radix(a, 16).ok())
                else {
                    self.out.put(b"usage: i2c write <hex-addr> <hex bytes>\r\n");
                    return;
                };
                let mut data = [0u8; 32];
                // arg2 onward: re-derive from arg2's position is awkward with the
                // iterator already consumed, so require the bytes in arg2 form:
                // accept a single run of hex pairs, e.g. `i2c write 42 00ff10`.
                let Some(n) =
                    arg2.and_then(|s| parse_hex_bytes(s, &mut data))
                else {
                    self.out.put(b"bad hex (e.g. i2c write 42 00ff10)\r\n");
                    return;
                };
                match self.i2c.write(addr, &data[..n]) {
                    Ok(_) => self.out.put(b"ok\r\n"),
                    Err(_) => self.out.put(b"error (nak/timeout)\r\n"),
                }
            }
            _ => self.out.put(b"usage: i2c scan|read|write\r\n"),
        }
    }

    fn cmd_flash(
        &mut self,
        verb: Option<&str>,
        off: Option<&str>,
        count: Option<&str>,
    ) {
        if verb != Some("read") {
            self.out.put(b"usage: flash read <hex-off> [n<=64]\r\n");
            return;
        }
        let Some(off) = off.and_then(|o| u32::from_str_radix(o, 16).ok())
        else {
            self.out.put(b"bad offset (hex, e.g. flash read 160)\r\n");
            return;
        };
        let count = match count {
            None => 64,
            Some(c) => match c.parse::<usize>() {
                Ok(n) => n.min(64),
                Err(_) => {
                    self.out.put(b"bad count (decimal, max 64)\r\n");
                    return;
                }
            },
        };
        let mut buf = [0u8; 64];
        match self.flash.read(off, &mut buf[..count]) {
            Ok(n) => {
                // Hexdump, 16 bytes per line with an ASCII gutter.
                for (li, chunk) in buf[..n].chunks(16).enumerate() {
                    self.out.put_hex32(off + (li as u32) * 16);
                    self.out.put(b": ");
                    for &b in chunk {
                        self.out.put_hex_byte(b);
                        self.out.put(b" ");
                    }
                    self.out.put(b" |");
                    for &b in chunk {
                        let printable = [b];
                        self.out.put(if (0x20..0x7f).contains(&b) {
                            &printable
                        } else {
                            b"."
                        });
                    }
                    self.out.put(b"|\r\n");
                }
            }
            Err(_) => self.out.put(b"error (bad address)\r\n"),
        }
    }

    fn cmd_rom(&mut self, code: Option<&str>) {
        let Some(code) = code.filter(|c| c.len() == 2) else {
            self.out.put(b"usage: rom <2-char code>, e.g. rom FO\r\n");
            return;
        };
        let b = code.as_bytes();
        let packed = u16::from_le_bytes([b[0], b[1]]);
        let addr = self.flash.rom_lookup(packed);
        self.out.put(b"rom['");
        self.out.put(b);
        self.out.put(b"'] = 0x");
        self.out.put_hex32(addr);
        self.out.put(if addr == 0 {
            b" (absent)\r\n" as &[u8]
        } else {
            b"\r\n"
        });
    }

    fn i2c_scan(&mut self) {
        self.out.put(b"i2c:  ");
        let mut found = 0u32;
        for addr in 0x08u8..0x78 {
            if let Ok(true) = self.i2c.probe(addr) {
                if found > 0 {
                    self.out.put(b", ");
                }
                self.out.put(b"0x");
                self.out.put_hex_byte(addr);
                found += 1;
            }
        }
        if found == 0 {
            self.out.put(b"no devices");
        }
        self.out.put(b"\r\n");
    }
}

/// Return the text after `verb ` in `line` (for commands whose final argument
/// is free text), trimmed.
fn subcommand_rest<'a>(line: &'a str, verb: &str) -> &'a str {
    match line.find(verb) {
        Some(i) => line[i + verb.len()..].trim(),
        None => "",
    }
}

/// Parse whitespace-separated hex byte pairs (or one contiguous run) into
/// `out`; returns the byte count, or None on bad input/overflow.
fn parse_hex_bytes(s: &str, out: &mut [u8]) -> Option<usize> {
    let mut n = 0;
    for word in s.split_whitespace() {
        if word.len() % 2 != 0 {
            return None;
        }
        for i in (0..word.len()).step_by(2) {
            let b = u8::from_str_radix(word.get(i..i + 2)?, 16).ok()?;
            if n == out.len() {
                return None;
            }
            out[n] = b;
            n += 1;
        }
    }
    Some(n)
}

#[export_name = "main"]
pub fn main() -> ! {
    let usb = UsbCons::from(USB.get_task_id());
    let mut shell = Shell {
        usb: UsbCons::from(USB.get_task_id()),
        gpio: Rp235xGpio::from(GPIO.get_task_id()),
        uart: Rp235xUart::from(UART.get_task_id()),
        spi: Rp235xSpi::from(SPI.get_task_id()),
        i2c: Rp235xI2c::from(I2C.get_task_id()),
        flash: Rp235xFlash::from(FLASH.get_task_id()),
        out: Out {
            usb,
            buf: [0u8; 256],
            len: 0,
        },
        heartbeat: true,
    };

    // Let enumeration settle, then greet.
    hl::sleep_for(1500);
    shell.out.put(b"\r\nHubris on RP2350 / Pico 2 -- type `help`\r\n");
    shell.out.put(PROMPT);
    shell.out.flush();

    let _ = shell.gpio.configure_output(LED_PIN);

    let mut line = [0u8; 128];
    let mut len = 0usize;
    let mut idle: u32 = 0;
    loop {
        let mut keys = [0u8; 32];
        let n = shell.usb.read(&mut keys);
        if n == 0 {
            // Nothing typed; poll at human speed. Blink the LED (~1 Hz) as a
            // liveness signal: blinking proves the shell loop, the GPIO IPC
            // chain, and the USB read IPC are all running.
            idle = idle.wrapping_add(1);
            if shell.heartbeat && idle.is_multiple_of(25) {
                let _ = shell.gpio.toggle(LED_PIN);
            }
            hl::sleep_for(20);
            continue;
        }
        for &b in &keys[..n] {
            match b {
                b'\r' | b'\n' => {
                    shell.out.put(b"\r\n");
                    if let Ok(cmd) = core::str::from_utf8(&line[..len]) {
                        shell.dispatch(cmd);
                    } else {
                        shell.out.put(b"(non-utf8 input)\r\n");
                    }
                    len = 0;
                    shell.out.put(PROMPT);
                }
                0x08 | 0x7f if len > 0 => {
                    len -= 1;
                    shell.out.put(b"\x08 \x08");
                }
                0x20..=0x7e if len < line.len() => {
                    line[len] = b;
                    len += 1;
                    shell.out.put(&[b]);
                }
                _ => {} // ignore other control bytes
            }
        }
        shell.out.flush();
    }
}
