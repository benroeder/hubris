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

use drv_rp235x_adc_api::{Rp235xAdc, TEMP_CHANNEL};
#[cfg(feature = "cyw43")]
use drv_rp235x_cyw43_api::Rp235xCyw43;
#[cfg(feature = "ds1302")]
use drv_rp235x_ds1302_api::Rp235xDs1302;
use drv_rp235x_flash_api::Rp235xFlash;
use drv_rp235x_gpio_api::Rp235xGpio;
use drv_rp235x_i2c_api::Rp235xI2c;
#[cfg(feature = "mailbox")]
use drv_rp235x_mailbox_api::Rp235xMailbox;
use drv_rp235x_pwm_api::Rp235xPwm;
#[cfg(feature = "sdcard")]
use drv_rp235x_sdcard_api::Rp235xSdcard;
#[cfg(feature = "slink")]
use drv_rp235x_slink_api::Rp235xSlink;
use drv_rp235x_spi_api::Rp235xSpi;
#[cfg(feature = "ws2812")]
use drv_rp235x_ws2812_api::Rp235xWs2812;

#[cfg(feature = "fat")]
mod fatfs;
use drv_rp235x_uart_api::Rp235xUart;
use task_rp235x_usb_api::UsbCons;
use userlib::{hl, sys_get_timer, task_slot};

task_slot!(USB, usb);
task_slot!(GPIO, gpio_driver);
task_slot!(UART, uart_driver);
task_slot!(SPI, spi_driver);
task_slot!(I2C, i2c_driver);
task_slot!(FLASH, flash_driver);
task_slot!(ADC, adc_driver);
task_slot!(PWM, pwm_driver);
#[cfg(feature = "cyw43")]
task_slot!(CYW43, cyw43);
#[cfg(feature = "mailbox")]
task_slot!(MAILBOX, mailbox_driver);
#[cfg(feature = "slink")]
task_slot!(SLINK, slink_driver);
#[cfg(feature = "ws2812")]
task_slot!(WS2812, ws2812);
#[cfg(feature = "ds1302")]
task_slot!(DS1302, ds1302);
#[cfg(feature = "sdcard")]
task_slot!(SDCARD, sdcard);

/// Pico 2 onboard LED, the `led` command's target.
const LED_PIN: u8 = 25;

const PROMPT: &[u8] = b"hubris> ";
const HELP: &[u8] = b"commands:\r\n\
  help                  this text\r\n\
  status                run self-tests (uart loopback, spi loopback, i2c scan)\r\n\
  bench all [addr]      throughput of every bus in one table\r\n\
  ticks                 ms since boot\r\n\
  led on|off|toggle|blink   onboard LED (on/off/toggle suspend the\r\n\
                            idle heartbeat; blink restores it)\r\n\
  gpio out|in|hi|lo|toggle|read <pin> | gpio pull <pin> up|down|none\r\n\
  uart send <text>      send out UART0 TX (GP0)\r\n\
  uart recv             drain UART0 RX buffer\r\n\
  uart bench [n]        time sending n bytes (default 4096); reports B/s\r\n\
  uart rxbench          count RX bytes over 3s (run while peer benches)\r\n\
  spi xfer <hex..>      full-duplex exchange, e.g. spi xfer a5 5a 3c\r\n\
  spi role controller|peripheral   set bus role for board-to-board (GP16-19)\r\n\
  spi load <hex<=8>     peripheral: stage response bytes for the controller\r\n\
  spi recv              peripheral: show bytes clocked in by the controller\r\n\
  spi bench [n]         controller: time clocking n bytes; reports B/s\r\n\
  i2c scan              probe all 7-bit addresses\r\n\
  i2c read <addr> <n>   read n bytes, e.g. i2c read 42 8\r\n\
  i2c write <addr> <hex..>\r\n\
  i2c target <addr> <hex<=16>   become an I2C target serving those bytes\r\n\
  i2c bench <addr> [n]  controller: time reading n bytes from a target\r\n\
  i2c speed <khz>       100 (std) | 400 (fast) | 1000 (fast-mode-plus)\r\n\
  flash read <hex-off> [n<=64]   dump flash, e.g. flash read 0 64\r\n\
  flash erase <hex-off>          erase a 4K sector (aligned)\r\n\
  flash write <hex-off> <hex..>  program bytes (within one 256B page)\r\n\
  rom <CC>              boot-ROM table lookup, e.g. rom FO\r\n\
  temp                  die temperature (internal sensor via ADC)\r\n\
  adc read <ch>         raw 12-bit ADC read (0-3 = GPIO26-29, 4 = temp)\r\n\
  led dim <pct>         PWM-dim the LED (led on|off|blink returns it to GPIO)\r\n\
  update <size-hex> <crc32-hex>  receive image over USB; write flash; verify\r\n\
  uart-update <size> <crc>  receive image over UART from a peer push\r\n\
  push <size> <crc>     stream own flash image to a peer over UART\r\n\
  reboot [bootsel]      reboot; with `bootsel`, land in USB flashing mode\r\n";

// Per-driver help lines, printed only when that driver's feature is enabled so
// `help` never advertises a command the build doesn't have.
#[cfg(feature = "mailbox")]
const HELP_MAILBOX: &[u8] =
    b"  core1 <n>|stress|speed|bulk <len> <it>   AMP cross-core mailbox + bulk xfer\r\n";
#[cfg(feature = "slink")]
const HELP_SLINK: &[u8] =
    b"  slink send <hex..>    send a Sony S-Link frame (2-3 bytes) on GP4\r\n\
  slink listen [ms]     wait for an S-Link frame; print the bytes\r\n";
#[cfg(feature = "ws2812")]
const HELP_WS2812: &[u8] =
    b"  rgb <r> <g> <b>       set the WS2812 NeoPixel on GP22 (0-255 each)\r\n";
#[cfg(feature = "ds1302")]
const HELP_DS1302: &[u8] =
    b"  rtc [get]             read the DS1302 clock (GP6/7/8)\r\n\
  rtc set <YY> <MM> <DD> <HH> <MM> <SS> [weekday]   set the DS1302 clock\r\n";
#[cfg(feature = "sdcard")]
const HELP_SDCARD: &[u8] = b"  sd init               run the SD SPI-mode init handshake (GP10-13)\r\n\
  sd read <block>       read a 512-byte block; hexdump it\r\n\
  sd find <start> <n>   scan n blocks; print any printable-ASCII runs (>=4)\r\n";
#[cfg(feature = "fat")]
const HELP_FAT: &[u8] = b"  sd ls                 list the FAT root directory (name + size)\r\n\
  sd cat <name>         print a file from the FAT root directory\r\n\
  sd df                 FAT volume total/used/free space (from BPB + FSInfo)\r\n\
  sd write <name> <text>  create/truncate <name> in the root dir; write <text>\r\n\
  sd rm <name>          delete <name> from the FAT root directory\r\n";

struct Shell {
    usb: UsbCons,
    gpio: Rp235xGpio,
    uart: Rp235xUart,
    spi: Rp235xSpi,
    i2c: Rp235xI2c,
    flash: Rp235xFlash,
    adc: Rp235xAdc,
    pwm: Rp235xPwm,
    #[cfg(feature = "cyw43")]
    cyw43: Rp235xCyw43,
    #[cfg(feature = "mailbox")]
    mailbox: Rp235xMailbox,
    #[cfg(feature = "slink")]
    slink: Rp235xSlink,
    #[cfg(feature = "ws2812")]
    ws2812: Rp235xWs2812,
    #[cfg(feature = "ds1302")]
    ds1302: Rp235xDs1302,
    #[cfg(feature = "sdcard")]
    sdcard: Rp235xSdcard,
    out: Out,
    /// Idle-loop LED heartbeat; `led on|off|toggle` takes manual control of
    /// the LED (turns this off), `led blink` gives it back.
    heartbeat: bool,
    /// Current I2C bus speed (kHz), tracked so `i2c bench` reports the right
    /// theoretical; updated by `i2c speed`.
    i2c_khz: u32,
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

    /// Two zero-padded decimal digits (clock fields + FAT `sd ls` dates, 0-99).
    #[cfg(any(feature = "ds1302", feature = "fat"))]
    fn put_pad2(&mut self, n: u8) {
        self.put(&[b'0' + (n / 10) % 10, b'0' + n % 10]);
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
            "help" => {
                self.out.put(HELP);
                #[cfg(feature = "mailbox")]
                self.out.put(HELP_MAILBOX);
                #[cfg(feature = "slink")]
                self.out.put(HELP_SLINK);
                #[cfg(feature = "ws2812")]
                self.out.put(HELP_WS2812);
                #[cfg(feature = "ds1302")]
                self.out.put(HELP_DS1302);
                #[cfg(feature = "sdcard")]
                self.out.put(HELP_SDCARD);
                #[cfg(feature = "fat")]
                self.out.put(HELP_FAT);
            }
            "status" => self.cmd_status(),
            "ticks" => {
                self.out.put(b"uptime ");
                self.out.put_u64(sys_get_timer().now);
                self.out.put(b" ms\r\n");
            }
            "bench" => self.cmd_bench(words.next(), words.next()),
            #[cfg(feature = "mailbox")]
            "core1" => self.cmd_core1(words.next(), words.next(), words.next()),
            "led" => self.cmd_led(words.next(), words.next()),
            "gpio" => self.cmd_gpio(words.next(), words.next(), words.next()),
            "uart" => self.cmd_uart(line, words.next()),
            "spi" => self.cmd_spi(line, words.next()),
            "i2c" => self.cmd_i2c(words.next(), words.next(), words.next()),
            "flash" => self.cmd_flash(words.next(), words.next(), words.next()),
            "rom" => self.cmd_rom(words.next()),
            "temp" => self.cmd_temp(),
            "adc" => self.cmd_adc(words.next(), words.next()),
            #[cfg(feature = "cyw43")]
            "wifi" => self.cmd_wifi(words.next()),
            "slot" => self.cmd_slot(),
            "update" => self.cmd_update(words.next(), words.next()),
            "uart-update" => self.cmd_uart_update(words.next(), words.next()),
            "push" => self.cmd_push(words.next(), words.next()),
            #[cfg(feature = "slink")]
            "slink" => self.cmd_slink(
                words.next(),
                words.next(),
                words.next(),
                words.next(),
            ),
            #[cfg(feature = "ws2812")]
            "rgb" => self.cmd_rgb(words.next(), words.next(), words.next()),
            #[cfg(feature = "ds1302")]
            "rtc" => self.cmd_rtc(
                words.next(),
                words.next(),
                words.next(),
                words.next(),
                words.next(),
                words.next(),
                words.next(),
                words.next(),
            ),
            #[cfg(feature = "sdcard")]
            "sd" => self.cmd_sd(line, words.next(), words.next(), words.next()),
            "crash" => {
                // Fault this task on purpose to test that jefe restarts the
                // shell + re-attaches the USB console (and, on AMP, that core
                // 1's kernel is unaffected).
                self.out
                    .put(b"crashing shell (jefe should restart me)...\r\n");
                self.out.flush();
                // Precisely-attributed task fault (undefined instruction).
                unsafe {
                    core::arch::asm!("udf #0");
                }
            }
            "reboot" => self.cmd_reboot(words.next()),
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

    fn cmd_led(&mut self, verb: Option<&str>, arg: Option<&str>) {
        if verb == Some("dim") {
            // Hand the pin to PWM (GPIO25 = slice 4, channel B, funcsel 4).
            let pct =
                arg.and_then(|a| a.parse::<u8>().ok()).filter(|p| *p <= 100);
            let Some(pct) = pct else {
                self.out.put(b"usage: led dim <0-100>\r\n");
                return;
            };
            self.heartbeat = false;
            let ok = self.gpio.set_function(LED_PIN, 4).is_ok()
                && self.pwm.set_duty(4, 1, pct).is_ok();
            self.out
                .put(if ok { b"ok\r\n" as &[u8] } else { b"error\r\n" });
            return;
        }
        // All other verbs drive the pin as a plain SIO output (this also
        // reclaims it from PWM after `led dim`).
        let _ = self.pwm.disable(4);
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
        self.out.put(if r.is_ok() {
            b"ok\r\n" as &[u8]
        } else {
            b"error\r\n"
        });
    }

    /// `rgb <r> <g> <b>`: set the WS2812 (NeoPixel) on GP22. Each channel is a
    /// decimal 0-255; packed into the WS2812 wire order (G<<16 | R<<8 | B).
    #[cfg(feature = "ws2812")]
    fn cmd_rgb(&mut self, r: Option<&str>, g: Option<&str>, b: Option<&str>) {
        let parse = |s: Option<&str>| s.and_then(|v| v.parse::<u8>().ok());
        let (Some(r), Some(g), Some(b)) = (parse(r), parse(g), parse(b)) else {
            self.out.put(b"usage: rgb <r> <g> <b> (0-255)\r\n");
            return;
        };
        let grb = ((g as u32) << 16) | ((r as u32) << 8) | (b as u32);
        self.out.put(if self.ws2812.set(grb).is_ok() {
            b"ok\r\n" as &[u8]
        } else {
            b"error\r\n"
        });
    }

    /// `rtc [get]` reads the DS1302 clock; `rtc set <YY> <MM> <DD> <HH> <MM>
    /// <SS> [weekday]` sets it. All fields are plain decimals; the driver does
    /// the BCD conversion. The packed u64 layout is byte0=sec, byte1=min,
    /// byte2=hour, byte3=date, byte4=month, byte5=weekday, byte6=year (0-99).
    #[cfg(feature = "ds1302")]
    fn cmd_rtc(
        &mut self,
        sub: Option<&str>,
        yy: Option<&str>,
        mm: Option<&str>,
        dd: Option<&str>,
        hh: Option<&str>,
        min: Option<&str>,
        ss: Option<&str>,
        wd: Option<&str>,
    ) {
        match sub {
            None | Some("get") => {
                let packed = self.ds1302.now();
                let sec = packed as u8;
                let minute = (packed >> 8) as u8;
                let hour = (packed >> 16) as u8;
                let date = (packed >> 24) as u8;
                let month = (packed >> 32) as u8;
                let year = (packed >> 48) as u8;
                // "20YY-MM-DD HH:MM:SS"
                self.out.put(b"20");
                self.out.put_pad2(year);
                self.out.put(b"-");
                self.out.put_pad2(month);
                self.out.put(b"-");
                self.out.put_pad2(date);
                self.out.put(b" ");
                self.out.put_pad2(hour);
                self.out.put(b":");
                self.out.put_pad2(minute);
                self.out.put(b":");
                self.out.put_pad2(sec);
                self.out.put(b"\r\n");
            }
            Some("set") => {
                let p = |s: Option<&str>| s.and_then(|v| v.parse::<u8>().ok());
                let (Some(yy), Some(mm), Some(dd), Some(hh), Some(min), Some(ss)) =
                    (p(yy), p(mm), p(dd), p(hh), p(min), p(ss))
                else {
                    self.out.put(
                        b"usage: rtc set <YY> <MM> <DD> <HH> <MM> <SS> [weekday]\r\n",
                    );
                    return;
                };
                let weekday = p(wd).unwrap_or(1);
                let packed = (ss as u64)
                    | (min as u64) << 8
                    | (hh as u64) << 16
                    | (dd as u64) << 24
                    | (mm as u64) << 32
                    | (weekday as u64) << 40
                    | (yy as u64) << 48;
                self.out.put(if self.ds1302.set(packed).is_ok() {
                    b"ok\r\n" as &[u8]
                } else {
                    b"bad time (check ranges)\r\n"
                });
            }
            _ => self.out.put(
                b"usage: rtc [get] | rtc set <YY> <MM> <DD> <HH> <MM> <SS> [weekday]\r\n",
            ),
        }
    }

    fn cmd_gpio(
        &mut self,
        verb: Option<&str>,
        pin: Option<&str>,
        arg: Option<&str>,
    ) {
        let (Some(verb), Some(pin)) = (verb, pin) else {
            self.out
                .put(b"usage: gpio out|in|hi|lo|toggle|read|pull <pin>\r\n");
            return;
        };
        let Ok(pin) = pin.parse::<u8>() else {
            self.out.put(b"bad pin\r\n");
            return;
        };
        let r = match verb {
            "pull" => {
                let pull = match arg {
                    Some("up") => drv_rp235x_gpio_api::PULL_UP,
                    Some("down") => drv_rp235x_gpio_api::PULL_DOWN,
                    Some("none") => drv_rp235x_gpio_api::PULL_NONE,
                    _ => {
                        self.out
                            .put(b"usage: gpio pull <pin> up|down|none\r\n");
                        return;
                    }
                };
                self.gpio.set_pull(pin, pull)
            }
            "out" => self.gpio.configure_output(pin),
            "in" => self.gpio.configure_input(pin),
            "hi" => self.gpio.set_high(pin),
            "lo" => self.gpio.set_low(pin),
            "toggle" => self.gpio.toggle(pin),
            "read" => match self.gpio.read(pin) {
                Ok(v) => {
                    self.out.put(b"pin ");
                    self.out.put_u32(pin as u32);
                    self.out.put(if v != 0 {
                        b" = 1\r\n" as &[u8]
                    } else {
                        b" = 0\r\n"
                    });
                    return;
                }
                Err(e) => Err(e),
            },
            _ => {
                self.out.put(
                    b"usage: gpio out|in|hi|lo|toggle|read|pull <pin>\r\n",
                );
                return;
            }
        };
        self.out.put(if r.is_ok() {
            b"ok\r\n" as &[u8]
        } else {
            b"error (bad pin?)\r\n"
        });
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
            Some("bench") => {
                // Send N bytes (default 4096) and time it. write() blocks on
                // TX-FIFO backpressure, so elapsed reflects the wire rate.
                let n: u32 = subcommand_rest(line, "bench")
                    .trim()
                    .parse()
                    .unwrap_or(4096);
                let buf = [0x55u8; 256];
                let t0 = sys_get_timer().now;
                let mut sent = 0u32;
                while sent < n {
                    let c = (n - sent).min(256);
                    self.uart.write(&buf[..c as usize]);
                    sent += c;
                }
                let ms = (sys_get_timer().now - t0) as u32;
                // 115200 8N1 -> 11520 B/s theoretical (10 bits/byte).
                self.bench_report(b"uart tx: ", n, ms, 11520);
            }
            Some("rxbench") => {
                // Count bytes received over a 3 s window while the peer runs
                // `uart bench`; reports the far-end throughput as a cross-check.
                let mut rx = [0u8; 256];
                let mut total = 0u32;
                let t0 = sys_get_timer().now;
                while sys_get_timer().now - t0 < 3000 {
                    total += self.uart.read(&mut rx) as u32;
                }
                self.bench_report(b"uart rx: ", total, 3000, 11520);
            }
            _ => self.out.put(
                b"usage: uart send <text> | recv | bench [n] | rxbench\r\n",
            ),
        }
    }

    /// `bench all [n]` -- run every bus back-to-back and print one comparison
    /// table. UART (TX) and SPI (loopback/controller) bench standalone; I2C
    /// needs a target on the bus, so `bench all 42` benches address 0x42 too.
    /// AMP: `core1 <n>` sends a number to core 1 over the inter-core mailbox and
    /// prints its reply (`n*2 + 1`); `core1 stress <count>` hammers the path.
    /// The mailbox driver bridges core 0's IPC to the SIO FIFO, answered by a
    /// task on core 1's own kernel.
    #[cfg(feature = "mailbox")]
    fn cmd_core1(&mut self, a: Option<&str>, b: Option<&str>, c: Option<&str>) {
        if a == Some("stress") {
            return self.core1_stress(b);
        }
        if a == Some("speed") {
            return self.core1_speed(b);
        }
        if a == Some("bulk") {
            return self.core1_bulk(b, c);
        }
        if a == Some("pipe") {
            return self.core1_pipe(b);
        }
        let Some(n) = a.and_then(|s| s.parse::<u32>().ok()) else {
            self.out.put(b"usage: core1 <n> | core1 stress <count>\r\n");
            return;
        };
        let reply = self.mailbox.exchange(n);
        if reply == 0xffff_ffff {
            self.out.put(b"core 1 did not answer (timeout)\r\n");
            return;
        }
        self.out.put(b"core 1: ");
        self.out.put_u32(n);
        self.out.put(b" -> ");
        self.out.put_u32(reply);
        self.out.put(if reply == n.wrapping_mul(2).wrapping_add(1) {
            b" (n*2+1, correct)\r\n" as &[u8]
        } else {
            b" (unexpected)\r\n"
        });
    }

    /// Stress the cross-core mailbox: `count` exchanges with distinct values,
    /// verifying every reply. Reports rate and any wrong/timed-out answers --
    /// proof the two-kernel FIFO path is correct under sustained load.
    #[cfg(feature = "mailbox")]
    fn core1_stress(&mut self, arg: Option<&str>) {
        let count = arg.and_then(|s| s.parse::<u32>().ok()).unwrap_or(2000);
        let t0 = sys_get_timer().now;
        let mut bad = 0u32;
        let mut timeouts = 0u32;
        let mut i = 0u32;
        while i < count {
            let r = self.mailbox.exchange(i);
            if r == 0xffff_ffff {
                timeouts += 1;
            } else if r != i.wrapping_mul(2).wrapping_add(1) {
                bad += 1;
            }
            i += 1;
        }
        let ms = (sys_get_timer().now - t0) as u32;
        self.out.put(b"stress: ");
        self.out.put_u32(count);
        self.out.put(b" exchanges in ");
        self.out.put_u32(ms);
        self.out.put(b" ms (");
        self.out
            .put_u32(count.wrapping_mul(1000).checked_div(ms).unwrap_or(0));
        self.out.put(b" exch/s), bad=");
        self.out.put_u32(bad);
        self.out.put(b" timeouts=");
        self.out.put_u32(timeouts);
        self.out.put(if bad == 0 && timeouts == 0 {
            b" -- PASS\r\n" as &[u8]
        } else {
            b" -- FAIL\r\n"
        });
    }

    /// Cross-core transfer speed: `n` round-trip exchanges timed inside the
    /// mailbox driver (one IPC), so the number reflects the raw SIO-FIFO +
    /// core-1 rate, not the per-command shell IPC (which caps `core1 stress`).
    /// Each exchange moves a 32-bit word each way = 8 bytes over the FIFO.
    #[cfg(feature = "mailbox")]
    fn core1_speed(&mut self, arg: Option<&str>) {
        let n = arg.and_then(|s| s.parse::<u32>().ok()).unwrap_or(1_000_000);
        let ms = self.mailbox.bench(n);
        let exch_per_s = (n as u64)
            .wrapping_mul(1000)
            .checked_div(ms as u64)
            .unwrap_or(0);
        let kb_per_s = exch_per_s.wrapping_mul(8) / 1024;
        self.out.put(b"core1 speed: ");
        self.out.put_u32(n);
        self.out.put(b" round-trips in ");
        self.out.put_u32(ms);
        self.out.put(b" ms = ");
        self.out.put_u64(exch_per_s);
        self.out.put(b" exch/s, ");
        self.out.put_u64(kb_per_s);
        self.out.put(b" KB/s (8 B/exchange on the FIFO)\r\n");
    }

    /// One-way bulk transfer ceiling: `core1 bulk <len> <iters>` moves `len`
    /// bytes (<=4096) through shared SRAM `iters` times -- core 0 writes the
    /// buffer, doorbells core 1, core 1 reads+checksums it. Reports MB/s of
    /// core->core payload.
    #[cfg(feature = "mailbox")]
    fn core1_bulk(&mut self, a: Option<&str>, b: Option<&str>) {
        let len = a.and_then(|s| s.parse::<u32>().ok()).unwrap_or(4096);
        let iters = b.and_then(|s| s.parse::<u32>().ok()).unwrap_or(50_000);
        let ms = self.mailbox.bulk_bench(len, iters);
        let bytes = (len as u64).wrapping_mul(iters as u64);
        let kb_per_s =
            bytes.wrapping_mul(1000).checked_div(ms as u64).unwrap_or(0) / 1024;
        self.out.put(b"core1 bulk: ");
        self.out.put_u32(iters);
        self.out.put(b" x ");
        self.out.put_u32(len);
        self.out.put(b" B in ");
        self.out.put_u32(ms);
        self.out.put(b" ms = ");
        self.out.put_u64(kb_per_s);
        self.out
            .put(b" KB/s one-way (core0->shared SRAM->core1)\r\n");
    }

    /// Pipelined bulk: `core1 pipe <iters>` transfers `iters` x 2 KiB blocks
    /// double-buffered, so core 0 fills one buffer while core 1 drains the
    /// other in parallel (buffers in different SRAM banks) -- the payoff of two
    /// cores. Compare to `core1 bulk` (sequential write-then-read).
    #[cfg(feature = "mailbox")]
    fn core1_pipe(&mut self, a: Option<&str>) {
        let iters = a.and_then(|s| s.parse::<u32>().ok()).unwrap_or(100_000);
        let ms = self.mailbox.bulk_pipe(iters);
        let bytes = (iters as u64).wrapping_mul(2048);
        let kb_per_s =
            bytes.wrapping_mul(1000).checked_div(ms as u64).unwrap_or(0) / 1024;
        self.out.put(b"core1 pipe: ");
        self.out.put_u32(iters);
        self.out.put(b" x 2048 B in ");
        self.out.put_u32(ms);
        self.out.put(b" ms = ");
        self.out.put_u64(kb_per_s);
        self.out.put(b" KB/s one-way (pipelined, 2 SRAM banks)\r\n");
    }

    fn cmd_bench(&mut self, sub: Option<&str>, addr: Option<&str>) {
        if sub != Some("all") {
            self.out.put(b"usage: bench all [i2c-target-hex-addr]\r\n");
            return;
        }
        const N: u32 = 4096;
        self.out.put(b"bus throughput (4096 bytes each):\r\n");

        // UART TX: write() blocks on the TX FIFO, so elapsed = wire rate.
        let buf = [0x55u8; 256];
        let t0 = sys_get_timer().now;
        let mut sent = 0u32;
        while sent < N {
            let c = (N - sent).min(256);
            self.uart.write(&buf[..c as usize]);
            sent += c;
        }
        self.bench_report(
            b"  uart:  ",
            N,
            (sys_get_timer().now - t0) as u32,
            11520,
        );

        // SPI: exchange through loopback (or the real bus if role=controller).
        let mut rx = [0u8; 256];
        let t0 = sys_get_timer().now;
        let mut sent = 0u32;
        while sent < N {
            let c = (N - sent).min(256) as usize;
            self.spi.exchange(&buf[..c], &mut rx[..c]);
            sent += c as u32;
        }
        self.bench_report(
            b"  spi:   ",
            N,
            (sys_get_timer().now - t0) as u32,
            187500,
        );

        // I2C: needs a target; bench it only if an address was given.
        match addr.and_then(|a| u8::from_str_radix(a, 16).ok()) {
            Some(a) => {
                let mut ib = [0u8; 32];
                let t0 = sys_get_timer().now;
                let mut done = 0u32;
                let mut ok = true;
                while done < N {
                    let c = (N - done).min(32) as usize;
                    if self.i2c.read(a, &mut ib[..c]).is_err() {
                        ok = false;
                        break;
                    }
                    done += c as u32;
                }
                if ok {
                    self.bench_report(
                        b"  i2c:   ",
                        N,
                        (sys_get_timer().now - t0) as u32,
                        self.i2c_khz * 1000 / 9,
                    );
                } else {
                    self.out.put(b"  i2c:   no target at that address\r\n");
                }
            }
            None => self
                .out
                .put(b"  i2c:   (give a target addr: `bench all 42`)\r\n"),
        }
    }

    /// Shared throughput report line for the bus speed-test demos:
    /// "<label> <bytes> bytes in <ms> ms = <B/s> B/s (<pct>% of <max>)".
    fn bench_report(
        &mut self,
        label: &[u8],
        bytes: u32,
        ms: u32,
        max_bps: u32,
    ) {
        let bps = if ms > 0 {
            (bytes as u64 * 1000 / ms as u64) as u32
        } else {
            0
        };
        self.out.put(label);
        self.out.put_u32(bytes);
        self.out.put(b" bytes in ");
        self.out.put_u32(ms);
        self.out.put(b" ms = ");
        self.out.put_u32(bps);
        self.out.put(b" B/s (");
        self.out
            .put_u32((bps * 100).checked_div(max_bps).unwrap_or(0));
        self.out.put(b"% of ");
        self.out.put_u32(max_bps);
        self.out.put(b" theoretical)\r\n");
    }

    fn cmd_spi(&mut self, line: &str, verb: Option<&str>) {
        match verb {
            Some("xfer") => {
                let mut tx = [0u8; 32];
                let Some(n) =
                    parse_hex_bytes(subcommand_rest(line, "xfer"), &mut tx)
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
            Some("role") => {
                let periph = match subcommand_rest(line, "role").trim() {
                    "peripheral" => 1u8,
                    "controller" => 0u8,
                    _ => {
                        self.out
                            .put(b"usage: spi role controller|peripheral\r\n");
                        return;
                    }
                };
                self.spi.set_role(periph);
                self.out.put(if periph != 0 {
                    b"spi role = peripheral (slave)\r\n" as &[u8]
                } else {
                    b"spi role = controller\r\n"
                });
            }
            Some("load") => {
                // Peripheral: stage up to 8 response bytes for the controller.
                let mut buf = [0u8; 8];
                let Some(n) =
                    parse_hex_bytes(subcommand_rest(line, "load"), &mut buf)
                else {
                    self.out.put(b"bad hex (max 8 bytes)\r\n");
                    return;
                };
                let k = self.spi.load_tx(&buf[..n]);
                self.out.put(b"loaded ");
                self.out.put_u32(k as u32);
                self.out.put(b" bytes into TX FIFO\r\n");
            }
            Some("recv") => {
                let mut rx = [0u8; 32];
                let n = self.spi.drain_rx(&mut rx);
                self.out.put(b"rx ");
                self.out.put_u32(n as u32);
                self.out.put(b":");
                for &b in &rx[..n] {
                    self.out.put(b" ");
                    self.out.put_hex_byte(b);
                }
                self.out.put(b"\r\n");
            }
            Some("bench") => {
                // Controller: clock n bytes and time it. Works standalone
                // (RX shifts in line state) or against a peripheral.
                let n: u32 = subcommand_rest(line, "bench")
                    .trim()
                    .parse()
                    .unwrap_or(4096);
                let tx = [0x55u8; 256];
                let mut rx = [0u8; 256];
                let t0 = sys_get_timer().now;
                let mut sent = 0u32;
                while sent < n {
                    let c = (n - sent).min(256) as usize;
                    self.spi.exchange(&tx[..c], &mut rx[..c]);
                    sent += c as u32;
                }
                let ms = (sys_get_timer().now - t0) as u32;
                // 1.5 MHz SCK, 8 bits/byte -> 187500 B/s theoretical.
                self.bench_report(b"spi: ", n, ms, 187500);
            }
            _ => self
                .out
                .put(b"usage: spi xfer|role|load|recv|bench ...\r\n"),
        }
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
                    self.out
                        .put(b"usage: i2c write <hex-addr> <hex bytes>\r\n");
                    return;
                };
                let mut data = [0u8; 32];
                // arg2 onward: re-derive from arg2's position is awkward with the
                // iterator already consumed, so require the bytes in arg2 form:
                // accept a single run of hex pairs, e.g. `i2c write 42 00ff10`.
                let Some(n) = arg2.and_then(|s| parse_hex_bytes(s, &mut data))
                else {
                    self.out.put(b"bad hex (e.g. i2c write 42 00ff10)\r\n");
                    return;
                };
                match self.i2c.write(addr, &data[..n]) {
                    Ok(_) => self.out.put(b"ok\r\n"),
                    Err(_) => self.out.put(b"error (nak/timeout)\r\n"),
                }
            }
            Some("target") => {
                // Become an I2C target at <addr> serving <hex> on every read.
                let (Some(addr), Some(hex)) =
                    (arg1.and_then(|a| u8::from_str_radix(a, 16).ok()), arg2)
                else {
                    self.out.put(
                        b"usage: i2c target <hex-addr> <hex bytes<=16>\r\n",
                    );
                    return;
                };
                let mut data = [0u8; 16];
                let Some(n) = parse_hex_bytes(hex, &mut data) else {
                    self.out.put(
                        b"bad hex (e.g. i2c target 42 48554252495321)\r\n",
                    );
                    return;
                };
                match self.i2c.serve(addr, &data[..n]) {
                    Ok(()) => {
                        self.out.put(b"serving ");
                        self.out.put_u32(n as u32);
                        self.out.put(b" bytes at 0x");
                        self.out.put_hex_byte(addr);
                        self.out.put(b" (this board is now an I2C target)\r\n");
                    }
                    Err(_) => self.out.put(b"error\r\n"),
                }
            }
            Some("speed") => {
                // Set bus speed in kHz: 100 (standard), 400 (fast), 1000 (FM+).
                let Some(khz) = arg1.and_then(|a| a.parse::<u32>().ok()) else {
                    self.out.put(b"usage: i2c speed <khz: 100|400|1000>\r\n");
                    return;
                };
                self.i2c.set_speed(khz);
                self.i2c_khz = khz;
                self.out.put(b"i2c speed set to ");
                self.out.put_u32(khz);
                self.out.put(b" kHz\r\n");
            }
            Some("bench") => {
                // Controller: read <addr> repeatedly to n bytes, timed on-
                // device (excludes USB overhead). Needs a target on the bus.
                let Some(addr) =
                    arg1.and_then(|a| u8::from_str_radix(a, 16).ok())
                else {
                    self.out.put(b"usage: i2c bench <hex-addr> [n]\r\n");
                    return;
                };
                let n: u32 = arg2.and_then(|c| c.parse().ok()).unwrap_or(4096);
                let mut buf = [0u8; 32];
                let t0 = sys_get_timer().now;
                let mut done = 0u32;
                let mut ok = true;
                while done < n {
                    let c = (n - done).min(32) as usize;
                    if self.i2c.read(addr, &mut buf[..c]).is_err() {
                        ok = false;
                        break;
                    }
                    done += c as u32;
                }
                let ms = (sys_get_timer().now - t0) as u32;
                if ok {
                    // ~9 bits/byte (data + ACK): theoretical = clock / 9.
                    let theo = self.i2c_khz * 1000 / 9;
                    self.bench_report(b"i2c: ", n, ms, theo);
                } else {
                    self.out
                        .put(b"i2c bench: read error (target present?)\r\n");
                }
            }
            _ => self
                .out
                .put(b"usage: i2c scan|read|write|target|speed|bench\r\n"),
        }
    }

    fn cmd_flash(
        &mut self,
        verb: Option<&str>,
        off: Option<&str>,
        count: Option<&str>,
    ) {
        if verb == Some("erase") {
            let Some(off) = off.and_then(|o| u32::from_str_radix(o, 16).ok())
            else {
                self.out.put(b"bad offset (hex, 4K-aligned)\r\n");
                return;
            };
            self.out.put(match self.flash.erase(off) {
                Ok(()) => b"erased\r\n" as &[u8],
                Err(drv_rp235x_flash_api::FlashError::BadAlignment) => {
                    b"not 4K-aligned\r\n"
                }
                Err(_) => b"error\r\n",
            });
            return;
        }
        if verb == Some("write") {
            let Some(off) = off.and_then(|o| u32::from_str_radix(o, 16).ok())
            else {
                self.out.put(b"bad offset (hex)\r\n");
                return;
            };
            let mut data = [0u8; 64];
            let Some(n) = count.and_then(|s| parse_hex_bytes(s, &mut data))
            else {
                self.out
                    .put(b"bad hex (e.g. flash write 3f0000 deadbeef)\r\n");
                return;
            };
            if n == 0 {
                self.out.put(b"no data\r\n");
                return;
            }
            self.out.put(match self.flash.program(off, &data[..n]) {
                Ok(()) => b"programmed\r\n" as &[u8],
                Err(drv_rp235x_flash_api::FlashError::BadAlignment) => {
                    b"crosses a page boundary\r\n"
                }
                Err(_) => b"error\r\n",
            });
            return;
        }
        if verb != Some("read") {
            self.out
                .put(b"usage: flash read|erase|write <hex-off> ...\r\n");
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

    fn cmd_temp(&mut self) {
        match self.adc.read(TEMP_CHANNEL) {
            Ok(raw) => {
                // T(C) = 27 - (V - 0.706 V)/1.721 mV, V = raw * 3.3 / 4096
                // (datasheet sec 12.4.6), in milli-units to stay integer.
                let uv = (raw as i64 * 3_300_000) / 4096;
                let milli_c = 27_000 - ((uv - 706_000) * 1000) / 1721;
                self.out.put(b"die temp ");
                if milli_c < 0 {
                    self.out.put(b"-");
                }
                let m = milli_c.unsigned_abs();
                self.out.put_u64(m / 1000);
                self.out.put(b".");
                self.out.put_u64((m % 1000) / 100);
                self.out.put(b" C (raw ");
                self.out.put_u32(raw as u32);
                self.out.put(b")\r\n");
            }
            Err(_) => self.out.put(b"adc error\r\n"),
        }
    }

    fn cmd_adc(&mut self, verb: Option<&str>, ch: Option<&str>) {
        let (Some("read"), Some(ch)) =
            (verb, ch.and_then(|c| c.parse::<u8>().ok()))
        else {
            self.out.put(b"usage: adc read <0-4>\r\n");
            return;
        };
        match self.adc.read(ch) {
            Ok(raw) => {
                self.out.put(b"adc[");
                self.out.put_u32(ch as u32);
                self.out.put(b"] = ");
                self.out.put_u32(raw as u32);
                self.out.put(b"\r\n");
            }
            Err(_) => self.out.put(b"error (bad channel?)\r\n"),
        }
    }

    /// CYW43439 Wi-Fi over the drv-rp235x-cyw43 Idol server: MAC, on-board LED,
    /// and an active scan (returns the number of AP result frames).
    #[cfg(feature = "cyw43")]
    fn cmd_wifi(&mut self, verb: Option<&str>) {
        match verb {
            Some("mac") => match self.cyw43.get_mac() {
                Ok(mac) => {
                    self.out.put(b"mac = ");
                    for (i, &b) in mac.iter().enumerate() {
                        if i != 0 {
                            self.out.put(b":");
                        }
                        self.out.put_hex_byte(b);
                    }
                    self.out.put(b"\r\n");
                }
                Err(_) => self.out.put(b"wifi not ready\r\n"),
            },
            Some("status") => match self.cyw43.wifi_status() {
                Ok(s) => {
                    self.out.put(b"status = 0x");
                    self.out.put_hex32(s);
                    self.out.put(b" (feedbead = up)\r\n");
                }
                Err(_) => self.out.put(b"wifi not ready\r\n"),
            },
            Some("on") => {
                let _ = self.cyw43.led(true);
                self.out.put(b"led on\r\n");
            }
            Some("off") => {
                let _ = self.cyw43.led(false);
                self.out.put(b"led off\r\n");
            }
            Some("scan") => {
                self.out.put(b"scanning...\r\n");
                match self.cyw43.scan() {
                    Ok(n) => {
                        self.out.put(b"found ");
                        self.out.put_u32(n);
                        self.out.put(b" AP result frames\r\n");
                    }
                    Err(_) => self.out.put(b"scan failed\r\n"),
                }
            }
            Some("ap") => {
                self.out.put(b"starting AP 'Pico2W-Setup'...\r\n");
                match self.cyw43.ap() {
                    Ok(s) => {
                        self.out.put(b"ap up, bss status = ");
                        self.out.put_u32(s);
                        self.out.put(b"\r\n");
                    }
                    Err(_) => self.out.put(b"ap failed\r\n"),
                }
            }
            _ => self.out.put(b"usage: wifi mac|status|on|off|scan|ap\r\n"),
        }
    }

    /// Receive `size` raw bytes over the console and program them into the
    /// boot region of flash, 256-byte page at a time, ACKing each page with
    /// a `.` so the host self-paces (the USB RX ring is finite). Safe while
    /// running because the whole image executes from SRAM; only a power cut
    /// mid-update leaves flash inconsistent (recover via BOOTSEL).
    /// Where an `update` should write. Detects an A/B partition table at
    /// flash 0 (PICOBIN marker + PARTITION_TABLE item) and returns the base of
    /// the *lower-versioned* partition; otherwise 0 (single image in place).
    fn cmd_slot(&mut self) {
        let mut hdr = [0u8; 8];
        let n = self.flash.read(0, &mut hdr).unwrap_or(0);
        self.out.put(b"flash[0..8] n=");
        self.out.put_u32(n as u32);
        self.out.put(b" bytes=");
        for &b in &hdr {
            self.out.put_hex_byte(b);
            self.out.put(b" ");
        }
        let mut va = [0u8; 4];
        let mut vb = [0u8; 4];
        let _ = self.flash.read(0x2000 + 0x184, &mut va);
        let _ = self.flash.read(0x42000 + 0x184, &mut vb);
        self.out.put(b"\r\nverA=");
        self.out.put_hex32(u32::from_le_bytes(va));
        self.out.put(b" verB=");
        self.out.put_hex32(u32::from_le_bytes(vb));
        self.out.put(b" target=0x");
        let t = self.update_target_base();
        self.out.put_hex32(t);
        self.out.put(b"\r\n");
    }

    fn update_target_base(&mut self) -> u32 {
        // Partition layout matches app/demo-pi-pico-2's provisioned table.
        const PART_A: u32 = 0x0000_2000;
        const PART_B: u32 = 0x0004_2000;
        // IMAGE_DEF version value: block at image+0x160, version word +0x24.
        const VER_OFF: u32 = 0x160 + 0x24;

        let mut hdr = [0u8; 8];
        if self.flash.read(0, &mut hdr).unwrap_or(0) < 8 {
            return 0;
        }
        let marker = u32::from_le_bytes([hdr[0], hdr[1], hdr[2], hdr[3]]);
        // hdr[4] is the first block item's type byte (0x0a = PARTITION_TABLE).
        if marker != 0xffff_ded3 || hdr[4] != 0x0a {
            return 0; // no A/B table: single image, write slot 0
        }
        let ver = |s: &mut Self, part: u32| -> u32 {
            let mut v = [0u8; 4];
            if s.flash.read(part + VER_OFF, &mut v).unwrap_or(0) < 4 {
                0
            } else {
                u32::from_le_bytes(v)
            }
        };
        // Write to the stale (lower-version) partition; ties go to A.
        if ver(self, PART_A) <= ver(self, PART_B) {
            PART_A
        } else {
            PART_B
        }
    }

    /// Receive a firmware image over this USB console (the host drives it).
    fn cmd_update(&mut self, size: Option<&str>, crc: Option<&str>) {
        let Some((size, want_crc)) = parse_update_args(size, crc) else {
            self.out.put(b"usage: update <size-hex> <crc32-hex>\r\n");
            return;
        };
        self.receive_update(Link::Usb, size, want_crc);
    }

    /// Receive a firmware image over UART from a peer board's `push`
    /// (example 04, cross-board A/B update -- no host touches this board).
    fn cmd_uart_update(&mut self, size: Option<&str>, crc: Option<&str>) {
        let Some((size, want_crc)) = parse_update_args(size, crc) else {
            self.out
                .put(b"usage: uart-update <size-hex> <crc32-hex>\r\n");
            return;
        };
        self.out.put(b"waiting for image over UART...\r\n");
        self.out.flush();
        self.receive_update(Link::Uart, size, want_crc);
    }

    /// Transport-agnostic image receiver: read `size` bytes from `link` (paced,
    /// one `.` ACK per 256-byte page back over the same link), write them to
    /// the inactive A/B slot (or slot 0 if single-image), then CRC-verify by
    /// readback. Shared by the USB (`update`) and UART (`uart-update`) paths.
    fn receive_update(&mut self, link: Link, size: u32, want_crc: u32) {
        const SECTOR: u32 = 4096;
        const PAGE: u32 = 256;
        // A/B: write the inactive (lower-version) partition; the ROM boots the
        // higher version next reboot, leaving the running image as fallback.
        // Single-image boards get base 0 (in-place).
        let base = self.update_target_base();
        self.out.put(b"target flash 0x");
        self.out.put_hex32(base);
        self.out.put(b"\r\n");
        self.out.flush();
        self.link_write(link, b"GO\r\n"); // tell the sender we are ready

        let mut page = [0u8; PAGE as usize];
        let mut off: u32 = 0;
        while off < size {
            let want = (size - off).min(PAGE) as usize;
            // Read one page. On UART each page carries a 2-byte checksum; a bad
            // page is NAK'd ("!") so the sender resends just that page (bounded
            // retries). USB CDC is reliable, so no per-page check there.
            let mut tries = 0u32;
            loop {
                if self.recv_bytes(link, &mut page[..want]).is_err() {
                    self.out.put(b"\r\ntimeout waiting for data\r\n");
                    self.link_write(link, b"ERR\r\n");
                    return;
                }
                if link != Link::Uart {
                    break;
                }
                let mut ck = [0u8; 2];
                if self.recv_bytes(link, &mut ck).is_err() {
                    self.link_write(link, b"ERR\r\n");
                    return;
                }
                if page16(&page[..want]) == u16::from_le_bytes(ck) {
                    break;
                }
                tries += 1;
                if tries > 12 {
                    self.out.put(b"\r\ntoo many page errors\r\n");
                    self.link_write(link, b"ERR\r\n");
                    return;
                }
                self.link_write(link, b"!"); // NAK: resend this page
            }
            if off.is_multiple_of(SECTOR)
                && self.flash.erase(base + off).is_err()
            {
                self.out.put(b"\r\nerase failed\r\n");
                self.link_write(link, b"ERR\r\n");
                return;
            }
            if self.flash.program(base + off, &page[..want]).is_err() {
                self.out.put(b"\r\nprogram failed\r\n");
                self.link_write(link, b"ERR\r\n");
                return;
            }
            off += want as u32;
            self.link_write(link, b"."); // ACK the page over the link
        }

        let crc = self.flash_crc32(base, size);
        if crc == want_crc {
            self.out.put(b"\r\nOK crc verified; `reboot` to apply\r\n");
            self.link_write(link, b"OK\r\n");
        } else {
            self.out.put(b"\r\nCRC MISMATCH: flash ");
            self.out.put_hex32(crc);
            self.out.put(b" != ");
            self.out.put_hex32(want_crc);
            self.out.put(b" -- do NOT reboot; retry\r\n");
            self.link_write(link, b"ERR\r\n");
        }
    }

    /// Push this board's own flash image to a peer over UART (example 04):
    /// wait for the peer's `GO`, stream `size` bytes (256-byte pages, one `.`
    /// ACK each), read the peer's final `OK`/`ERR`, and report the on-wire
    /// time. The image is read from flash `base` 0 (active slot / single
    /// image). The peer must already be in `uart-update <size> <crc>`.
    fn cmd_push(&mut self, size: Option<&str>, crc: Option<&str>) {
        const PAGE: u32 = 256;
        let Some((size, _crc)) = parse_update_args(size, crc) else {
            self.out.put(b"usage: push <size-hex> <crc32-hex>\r\n");
            return;
        };
        self.out.put(b"waiting for peer GO over UART...\r\n");
        self.out.flush();
        if !self.uart_wait_token(b"GO", 800) {
            self.out.put(b"no GO (is the peer in uart-update?)\r\n");
            return;
        }
        let start = sys_get_timer().now;
        let mut off: u32 = 0;
        let mut buf = [0u8; PAGE as usize];
        while off < size {
            let want = (size - off).min(PAGE) as usize;
            if self.flash.read(off, &mut buf[..want]).is_err() {
                self.out.put(b"flash read failed\r\n");
                return;
            }
            // Each page: data + 2-byte checksum, then wait for the peer's
            // `.` (ok) or `!` (resend). Bounded retries per page.
            let ck = page16(&buf[..want]).to_le_bytes();
            let mut tries = 0u32;
            loop {
                self.uart.write(&buf[..want]);
                self.uart.write(&ck);
                if self.uart_wait_ack(800) {
                    break; // "." -- page accepted
                }
                tries += 1;
                if tries > 12 {
                    self.out
                        .put(b"\r\npeer ACK timeout / too many retries\r\n");
                    return;
                }
            }
            off += want as u32;
        }
        let ms = (sys_get_timer().now - start) as u32;
        let ok = self.uart_wait_token(b"OK", 1500);
        self.out.put(if ok {
            b"\r\npush OK: peer verified; " as &[u8]
        } else {
            b"\r\npush done (peer status?); "
        });
        self.out.put_u32(size);
        self.out.put(b" bytes in ");
        self.out.put_u32(ms);
        self.out.put(b" ms\r\n");
    }

    /// CRC32 (zlib polynomial) over `size` bytes of flash at `base` (readback).
    fn flash_crc32(&mut self, base: u32, size: u32) -> u32 {
        let mut crc: u32 = 0xffff_ffff;
        let mut off: u32 = 0;
        let mut buf = [0u8; 256];
        while off < size {
            let want = (size - off).min(256) as usize;
            let Ok(n) = self.flash.read(base + off, &mut buf[..want]) else {
                return 0;
            };
            for &b in &buf[..n] {
                crc ^= b as u32;
                for _ in 0..8 {
                    let mask = (crc & 1).wrapping_neg();
                    crc = (crc >> 1) ^ (0xedb8_8320 & mask);
                }
            }
            off += n as u32;
        }
        !crc
    }

    /// Read UART until `token` appears or the idle timeout (in 10 ms units)
    /// elapses. Returns whether the token was seen. Simple substring match --
    /// fine for the short protocol tokens (GO, ., OK) with no repeated prefix.
    fn uart_wait_token(&mut self, token: &[u8], units: u32) -> bool {
        let mut matched = 0usize;
        let mut idle = 0u32;
        let mut rx = [0u8; 64];
        loop {
            let n = self.uart.read(&mut rx);
            if n == 0 {
                idle += 1;
                if idle > units {
                    return false;
                }
                hl::sleep_for(10);
                continue;
            }
            idle = 0;
            for &b in &rx[..n] {
                if b == token[matched] {
                    matched += 1;
                    if matched == token.len() {
                        return true;
                    }
                } else {
                    matched = usize::from(b == token[0]);
                }
            }
        }
    }

    /// Read bytes from a transport link.
    fn link_read(&mut self, link: Link, buf: &mut [u8]) -> usize {
        match link {
            Link::Usb => self.usb.read(buf),
            Link::Uart => self.uart.read(buf),
        }
    }

    /// Fill `buf` from `link`, blocking with an idle timeout. Err on timeout.
    fn recv_bytes(&mut self, link: Link, buf: &mut [u8]) -> Result<(), ()> {
        let mut got = 0usize;
        let mut idle = 0u32;
        while got < buf.len() {
            let n = self.link_read(link, &mut buf[got..]);
            if n == 0 {
                idle += 1;
                if idle > 500 {
                    return Err(());
                }
                hl::sleep_for(10);
            } else {
                idle = 0;
                got += n;
            }
        }
        Ok(())
    }

    /// Wait for a page ACK over UART: `.` = ok (true), `!` = resend (false).
    /// Times out to false after ~units*10 ms of no data.
    fn uart_wait_ack(&mut self, units: u32) -> bool {
        let mut idle = 0u32;
        let mut rx = [0u8; 16];
        loop {
            let n = self.uart.read(&mut rx);
            if n == 0 {
                idle += 1;
                if idle > units {
                    return false;
                }
                hl::sleep_for(10);
                continue;
            }
            for &b in &rx[..n] {
                if b == b'.' {
                    return true;
                }
                if b == b'!' {
                    return false;
                }
            }
        }
    }

    /// Send protocol bytes (GO / . / OK / ERR) back over a transport link.
    /// For USB, flush the console buffer first so ordering is preserved.
    fn link_write(&mut self, link: Link, data: &[u8]) {
        match link {
            Link::Usb => {
                self.out.flush();
                self.usb.write(data);
            }
            Link::Uart => {
                self.uart.write(data);
            }
        }
    }

    /// Sony S-Link / Control-A1 on GP4: `slink send <hex> <hex> [hex]` bit-bangs
    /// a 2-3 byte frame; `slink listen [ms]` waits for one and prints its bytes.
    #[cfg(feature = "slink")]
    fn cmd_slink(
        &mut self,
        sub: Option<&str>,
        a: Option<&str>,
        b: Option<&str>,
        c: Option<&str>,
    ) {
        match sub {
            Some("send") => {
                let p = |s: Option<&str>| {
                    s.and_then(|x| u8::from_str_radix(x, 16).ok())
                };
                let (Some(b0), Some(b1)) = (p(a), p(b)) else {
                    self.out.put(b"usage: slink send <hex> <hex> [hex]\r\n");
                    return;
                };
                let (b2, n) = match p(c) {
                    Some(b2) => (b2, 3u8),
                    None => (0, 2),
                };
                self.slink.send(b0, b1, b2, n);
                self.out.put(b"sent ");
                self.out.put_u32(n as u32);
                self.out.put(b" bytes: ");
                self.out.put_hex_byte(b0);
                self.out.put(b" ");
                self.out.put_hex_byte(b1);
                if n == 3 {
                    self.out.put(b" ");
                    self.out.put_hex_byte(b2);
                }
                self.out.put(b"\r\n");
            }
            Some("listen") => {
                let ms = a.and_then(|s| s.parse::<u32>().ok()).unwrap_or(5000);
                self.out.put(b"listening ");
                self.out.put_u32(ms);
                self.out.put(b" ms...\r\n");
                self.out.flush();
                let r = self.slink.listen(ms);
                let n = (r >> 24) & 0xff;
                if n == 0 {
                    self.out.put(b"no frame (timeout)\r\n");
                    return;
                }
                self.out.put(b"rx ");
                self.out.put_u32(n);
                self.out.put(b" bytes: ");
                self.out.put_hex_byte((r >> 16) as u8);
                self.out.put(b" ");
                self.out.put_hex_byte((r >> 8) as u8);
                if n == 3 {
                    self.out.put(b" ");
                    self.out.put_hex_byte(r as u8);
                }
                self.out.put(b"\r\n");
            }
            Some("flood") => {
                let n = a.and_then(|s| s.parse::<u32>().ok()).unwrap_or(256);
                self.out.put(b"flooding ");
                self.out.put_u32(n);
                self.out.put(b" frames...\r\n");
                self.out.flush();
                self.slink.flood(n);
                self.out.put(b"flood done (");
                self.out.put_u32(n);
                self.out.put(b" frames sent)\r\n");
            }
            Some("soak") => {
                let n = a.and_then(|s| s.parse::<u32>().ok()).unwrap_or(256);
                let ms =
                    b.and_then(|s| s.parse::<u32>().ok()).unwrap_or(60000);
                self.out.put(b"soaking up to ");
                self.out.put_u32(n);
                self.out.put(b" frames...\r\n");
                self.out.flush();
                let r = self.slink.soak(n, ms);
                let good = r & 0xffff;
                let bad = r >> 16;
                let m1 = self.slink.margin_ones();
                let m0 = self.slink.margin_zeros();
                self.out.put(b"soak: good=");
                self.out.put_u32(good);
                self.out.put(b" bad=");
                self.out.put_u32(bad);
                self.out.put(b" (");
                self.out.put(if bad == 0 && good > 0 {
                    b"PASS" as &[u8]
                } else {
                    b"CHECK"
                });
                self.out.put(b")\r\n  mark width us: ones ");
                self.out.put_u32(m1 >> 16);
                self.out.put(b"-");
                self.out.put_u32(m1 & 0xffff);
                self.out.put(b" (nom 1200), zeros ");
                self.out.put_u32(m0 >> 16);
                self.out.put(b"-");
                self.out.put_u32(m0 & 0xffff);
                self.out.put(b" (nom 600)\r\n");
            }
            _ => self.out.put(
                b"usage: slink send <hex..> | listen [ms] | flood <n> | soak <n> [ms]\r\n",
            ),
        }
    }

    /// `sd init` runs the SD SPI-mode handshake; `sd read <block>` hexdumps one
    /// 512-byte block; `sd find <start> <count>` scans blocks for printable-
    /// ASCII runs (>=4 bytes) -- a way to spot a text message on a formatted
    /// card with no filesystem. Block numbers are plain decimals (like `rgb`).
    #[cfg(feature = "sdcard")]
    fn cmd_sd(
        &mut self,
        line: &str,
        verb: Option<&str>,
        a: Option<&str>,
        b: Option<&str>,
    ) {
        let _ = line;
        match verb {
            Some("init") => match self.sdcard.init() {
                Ok(status) => {
                    self.out.put(b"init ok: ");
                    self.out.put(
                        if status & drv_rp235x_sdcard_api::STATUS_V2 != 0 {
                            b"SDv2 " as &[u8]
                        } else {
                            b"SDv1 "
                        },
                    );
                    self.out.put(
                        if status & drv_rp235x_sdcard_api::STATUS_CCS != 0 {
                            b"SDHC/block-addressed\r\n" as &[u8]
                        } else {
                            b"SDSC/byte-addressed\r\n"
                        },
                    );
                }
                Err(e) => {
                    self.out.put(b"init error ");
                    self.out.put_u32(e as u32);
                    self.out.put(b"\r\n");
                }
            },
            Some("read") => {
                let Some(block) = a.and_then(|s| s.parse::<u32>().ok()) else {
                    self.out.put(b"usage: sd read <block>\r\n");
                    return;
                };
                let mut buf = [0u8; 512];
                match self.sdcard.read_block(block, &mut buf) {
                    Ok(()) => {
                        // Hexdump, 16 bytes per line with an ASCII gutter.
                        for (li, chunk) in buf.chunks(16).enumerate() {
                            self.out.put_hex32((li as u32) * 16);
                            self.out.put(b": ");
                            for &x in chunk {
                                self.out.put_hex_byte(x);
                                self.out.put(b" ");
                            }
                            self.out.put(b" |");
                            for &x in chunk {
                                let p = [x];
                                self.out.put(if (0x20..0x7f).contains(&x) {
                                    &p
                                } else {
                                    b"."
                                });
                            }
                            self.out.put(b"|\r\n");
                        }
                    }
                    Err(e) => {
                        self.out.put(b"read error ");
                        self.out.put_u32(e as u32);
                        self.out.put(b"\r\n");
                    }
                }
            }
            Some("find") => {
                let (Some(start), Some(count)) = (
                    a.and_then(|s| s.parse::<u32>().ok()),
                    b.and_then(|s| s.parse::<u32>().ok()),
                ) else {
                    self.out.put(b"usage: sd find <start-block> <count>\r\n");
                    return;
                };
                let mut hits = 0u32;
                for i in 0..count {
                    let block = start.wrapping_add(i);
                    let mut buf = [0u8; 512];
                    if self.sdcard.read_block(block, &mut buf).is_err() {
                        self.out.put(b"read error at block ");
                        self.out.put_u32(block);
                        self.out.put(b"\r\n");
                        break;
                    }
                    // Print each run of >=4 consecutive printable ASCII bytes.
                    let mut run = 0usize;
                    for j in 0..=buf.len() {
                        let printable =
                            j < buf.len() && (0x20..0x7f).contains(&buf[j]);
                        if printable {
                            run += 1;
                        } else {
                            if run >= 4 {
                                let s = j - run;
                                self.out.put_u32(block);
                                self.out.put(b"+");
                                self.out.put_u32(s as u32);
                                self.out.put(b": ");
                                self.out.put(&buf[s..j]);
                                self.out.put(b"\r\n");
                                hits += 1;
                            }
                            run = 0;
                        }
                    }
                }
                self.out.put(b"done, ");
                self.out.put_u32(hits);
                self.out.put(b" run(s)\r\n");
            }
            #[cfg(feature = "fat")]
            Some("ls") => self.fat_ls(),
            #[cfg(feature = "fat")]
            Some("cat") => self.fat_cat(a),
            #[cfg(feature = "fat")]
            Some("df") => self.fat_df(),
            #[cfg(feature = "fat")]
            Some("write") => {
                // "sd write <NAME> <text...>": NAME is the first token after
                // "write", the file body is the rest of the line verbatim.
                let rest = subcommand_rest(line, "write");
                let mut it = rest.splitn(2, char::is_whitespace);
                let name = it.next().unwrap_or("");
                let text = it.next().unwrap_or("").trim_start();
                self.fat_write(name, text);
            }
            #[cfg(feature = "fat")]
            Some("rm") => self.fat_rm(a),
            _ => {
                self.out.put(
                    b"usage: sd init | read <block> | find <start> <count>",
                );
                #[cfg(feature = "fat")]
                self.out
                    .put(b" | ls | cat <name> | df | write <name> <text> | rm <name>");
                self.out.put(b"\r\n");
            }
        }
    }

    /// The `TimeSource` every `sd` FAT command hands to its `VolumeManager`.
    /// In a `ds1302` build this is the live DS1302 clock, so newly written
    /// files get real timestamps; otherwise the fixed `DummyTime`. Single
    /// place that picks the source.
    #[cfg(feature = "fat")]
    fn fat_time() -> fatfs::FatTime {
        #[cfg(feature = "ds1302")]
        {
            fatfs::RtcTime::new(Rp235xDs1302::from(DS1302.get_task_id()))
        }
        #[cfg(not(feature = "ds1302"))]
        {
            fatfs::DummyTime
        }
    }

    /// `sd ls`: mount FAT volume 0, list the root directory (name + size).
    /// UNPROVEN on hardware.
    #[cfg(feature = "fat")]
    fn fat_ls(&mut self) {
        use embedded_sdmmc::{VolumeIdx, VolumeManager};
        let vm = VolumeManager::new(
            fatfs::SdBlockDevice::new(Rp235xSdcard::from(SDCARD.get_task_id())),
            Self::fat_time(),
        );
        let volume = match vm.open_volume(VolumeIdx(0)) {
            Ok(v) => v,
            Err(_) => {
                self.out.put(b"sd ls: open volume failed\r\n");
                return;
            }
        };
        let root = match volume.open_root_dir() {
            Ok(d) => d,
            Err(_) => {
                self.out.put(b"sd ls: open root dir failed\r\n");
                return;
            }
        };
        let mut count = 0u32;
        let res = root.iterate_dir(|entry| {
            // Skip the volume-label / long-file-name pseudo-entries.
            if entry.attributes.is_volume() {
                return;
            }
            // Reassemble the 8.3 short name: BASE[.EXT], trailing `/` for dirs.
            self.out.put(entry.name.base_name());
            let ext = entry.name.extension();
            if !ext.is_empty() {
                self.out.put(b".");
                self.out.put(ext);
            }
            if entry.attributes.is_directory() {
                self.out.put(b"/");
            }
            self.out.put(b"\t");
            self.out.put_u32(entry.size);
            // Last-modified time as "YYYY-MM-DD HH:MM" (from the TimeSource
            // that stamped the entry when it was written).
            let t = &entry.mtime;
            self.out.put(b"\t");
            self.out.put_u32(1970 + t.year_since_1970 as u32);
            self.out.put(b"-");
            self.out.put_pad2(t.zero_indexed_month + 1);
            self.out.put(b"-");
            self.out.put_pad2(t.zero_indexed_day + 1);
            self.out.put(b" ");
            self.out.put_pad2(t.hours);
            self.out.put(b":");
            self.out.put_pad2(t.minutes);
            self.out.put(b"\r\n");
            count += 1;
        });
        if res.is_err() {
            self.out.put(b"sd ls: directory read error\r\n");
        } else if count == 0 {
            self.out.put(b"(empty)\r\n");
        }
    }

    /// `sd cat <NAME>`: open NAME in the root dir read-only, stream it to the
    /// console until EOF. UNPROVEN on hardware.
    #[cfg(feature = "fat")]
    fn fat_cat(&mut self, name: Option<&str>) {
        use embedded_sdmmc::{Mode, VolumeIdx, VolumeManager};
        let Some(name) = name else {
            self.out.put(b"usage: sd cat <name>\r\n");
            return;
        };
        let vm = VolumeManager::new(
            fatfs::SdBlockDevice::new(Rp235xSdcard::from(SDCARD.get_task_id())),
            Self::fat_time(),
        );
        let volume = match vm.open_volume(VolumeIdx(0)) {
            Ok(v) => v,
            Err(_) => {
                self.out.put(b"sd cat: open volume failed\r\n");
                return;
            }
        };
        let root = match volume.open_root_dir() {
            Ok(d) => d,
            Err(_) => {
                self.out.put(b"sd cat: open root dir failed\r\n");
                return;
            }
        };
        let file = match root.open_file_in_dir(name, Mode::ReadOnly) {
            Ok(f) => f,
            Err(_) => {
                self.out.put(b"sd cat: file not found\r\n");
                return;
            }
        };
        // Small on-stack buffer: embedded-sdmmc already keeps one 512-byte
        // Block on the stack per read, so keep ours modest.
        let mut buf = [0u8; 64];
        loop {
            match file.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => self.out.put(&buf[..n]),
                Err(_) => {
                    self.out.put(b"\r\nsd cat: read error\r\n");
                    return;
                }
            }
            if file.is_eof() {
                break;
            }
        }
        self.out.put(b"\r\n");
    }

    /// `sd write <NAME> <text...>`: create-or-truncate NAME in the FAT root
    /// directory and write `text` (the rest of the command line) as its body.
    /// The handle is closed before returning so embedded-sdmmc flushes the
    /// directory entry + FAT to the card. UNPROVEN on hardware.
    #[cfg(feature = "fat")]
    fn fat_write(&mut self, name: &str, text: &str) {
        use embedded_sdmmc::{Mode, VolumeIdx, VolumeManager};
        if name.is_empty() {
            self.out.put(b"usage: sd write <name> <text...>\r\n");
            return;
        }
        let vm = VolumeManager::new(
            fatfs::SdBlockDevice::new(Rp235xSdcard::from(SDCARD.get_task_id())),
            Self::fat_time(),
        );
        let volume = match vm.open_volume(VolumeIdx(0)) {
            Ok(v) => v,
            Err(_) => {
                self.out.put(b"sd write: open volume failed\r\n");
                return;
            }
        };
        let root = match volume.open_root_dir() {
            Ok(d) => d,
            Err(_) => {
                self.out.put(b"sd write: open root dir failed\r\n");
                return;
            }
        };
        let file = match root
            .open_file_in_dir(name, Mode::ReadWriteCreateOrTruncate)
        {
            Ok(f) => f,
            Err(_) => {
                self.out.put(b"sd write: open file failed\r\n");
                return;
            }
        };
        let bytes = text.as_bytes();
        if file.write(bytes).is_err() {
            self.out.put(b"sd write: write error\r\n");
            // Best-effort: release the handle (may itself fail on a bad card).
            let _ = file.close();
            return;
        }
        // close() flushes the metadata (directory entry + FAT) and commits the
        // write; without it the data would not be persisted.
        if file.close().is_err() {
            self.out.put(b"sd write: flush/close error\r\n");
            return;
        }
        self.out.put(b"wrote ");
        self.out.put_u32(bytes.len() as u32);
        self.out.put(b" bytes\r\n");
    }

    /// `sd rm <NAME>`: delete NAME from the FAT root directory. UNPROVEN on
    /// hardware.
    #[cfg(feature = "fat")]
    fn fat_rm(&mut self, name: Option<&str>) {
        use embedded_sdmmc::{VolumeIdx, VolumeManager};
        let Some(name) = name else {
            self.out.put(b"usage: sd rm <name>\r\n");
            return;
        };
        let vm = VolumeManager::new(
            fatfs::SdBlockDevice::new(Rp235xSdcard::from(SDCARD.get_task_id())),
            Self::fat_time(),
        );
        let volume = match vm.open_volume(VolumeIdx(0)) {
            Ok(v) => v,
            Err(_) => {
                self.out.put(b"sd rm: open volume failed\r\n");
                return;
            }
        };
        let root = match volume.open_root_dir() {
            Ok(d) => d,
            Err(_) => {
                self.out.put(b"sd rm: open root dir failed\r\n");
                return;
            }
        };
        match root.delete_file_in_dir(name) {
            Ok(()) => self.out.put(b"removed\r\n"),
            Err(_) => self.out.put(b"sd rm: delete failed (not found?)\r\n"),
        }
    }

    /// `sd df`: report the FAT volume's total / used / free space. embedded-sdmmc
    /// 0.9 exposes no free-space API, so this reads the boot sector (BPB) and
    /// FAT32 FSInfo sector directly via the raw block device. The free count is
    /// FSInfo's cached value and can be stale (a true df scans the FAT).
    /// UNPROVEN on hardware.
    #[cfg(feature = "fat")]
    fn fat_df(&mut self) {
        let sdcard = Rp235xSdcard::from(SDCARD.get_task_id());
        let mut blk = [0u8; 512];
        let rd16 =
            |b: &[u8; 512], o: usize| u16::from_le_bytes([b[o], b[o + 1]]);
        let rd32 = |b: &[u8; 512], o: usize| {
            u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
        };

        // 1. MBR at LBA 0 -> partition 1's start LBA (mirrors what open_volume
        //    mounts). This assumes an MBR-partitioned card (the usual case for
        //    SD/SDHC). A "superfloppy" (no MBR, VBR at LBA 0) is NOT handled: its
        //    boot sector also ends in 0xAA55, so the offset-454 read would yield
        //    garbage rather than 0 -- df would then misparse or error, not crash.
        if sdcard.read_block(0, &mut blk).is_err() {
            self.out.put(b"sd df: read MBR failed\r\n");
            return;
        }
        if rd16(&blk, 510) != 0xAA55 {
            self.out.put(b"sd df: no MBR signature\r\n");
            return;
        }
        let part_start = rd32(&blk, 446 + 8);

        // 2. Boot sector / BPB at the partition start.
        if sdcard.read_block(part_start, &mut blk).is_err() {
            self.out.put(b"sd df: read boot sector failed\r\n");
            return;
        }
        if rd16(&blk, 510) != 0xAA55 {
            self.out.put(b"sd df: bad boot signature\r\n");
            return;
        }
        let bytes_per_block = rd16(&blk, 11) as u32;
        let spc = blk[13] as u32; // blocks (sectors) per cluster
        let reserved = rd16(&blk, 14) as u32;
        let num_fats = blk[16] as u32;
        let root_entries = rd16(&blk, 17) as u32;
        let total16 = rd16(&blk, 19) as u32;
        let total32 = rd32(&blk, 32);
        let fat16 = rd16(&blk, 22) as u32;
        let fat32 = rd32(&blk, 36);
        let fs_info = rd16(&blk, 48) as u32;
        if bytes_per_block != 512 || spc == 0 {
            self.out.put(b"sd df: unsupported geometry\r\n");
            return;
        }
        let fat_size = if fat16 != 0 { fat16 } else { fat32 };
        let total_blocks = if total16 != 0 { total16 } else { total32 };
        let root_dir_blocks = (root_entries * 32).div_ceil(512);
        let non_data = reserved + num_fats * fat_size + root_dir_blocks;
        if total_blocks <= non_data {
            self.out.put(b"sd df: bad geometry\r\n");
            return;
        }
        let cluster_count = (total_blocks - non_data) / spc;
        let bytes_per_cluster = (spc * 512) as u64;
        let total_data_bytes = u64::from(cluster_count) * bytes_per_cluster;

        // 3. FAT32 FSInfo cached free-cluster count. FAT16 (fat_size16 != 0)
        //    has no FSInfo, so free stays unknown there.
        let mut free_bytes: Option<u64> = None;
        if fat16 == 0
            && sdcard.read_block(part_start + fs_info, &mut blk).is_ok()
            && rd32(&blk, 0) == 0x4161_5252
            && rd32(&blk, 484) == 0x6141_7272
        {
            let free_clusters = rd32(&blk, 488);
            if free_clusters != 0xFFFF_FFFF && free_clusters <= cluster_count {
                free_bytes = Some(u64::from(free_clusters) * bytes_per_cluster);
            }
        }

        let mib = |bytes: u64| (bytes / (1024 * 1024)) as u32;
        self.out.put(b"total ");
        self.out.put_u32(mib(total_data_bytes));
        self.out.put(b" MiB");
        match free_bytes {
            Some(free) => {
                let used = total_data_bytes.saturating_sub(free);
                self.out.put(b", used ");
                self.out.put_u32(mib(used));
                self.out.put(b" MiB, free ");
                self.out.put_u32(mib(free));
                self.out.put(
                    b" MiB (FSInfo cached free-count, may be stale)\r\n",
                );
            }
            None => self.out.put(
                b", used/free unknown (no valid FSInfo; real df scans the FAT)\r\n",
            ),
        }
    }

    fn cmd_reboot(&mut self, mode: Option<&str>) {
        let bootsel = match mode {
            Some("bootsel") => 1,
            None => 0,
            _ => {
                self.out.put(b"usage: reboot [bootsel]\r\n");
                return;
            }
        };
        let rc = self.flash.reboot(bootsel);
        if rc == 0 {
            self.out.put(if bootsel != 0 {
                b"rebooting to BOOTSEL...\r\n" as &[u8]
            } else {
                b"rebooting...\r\n"
            });
        } else {
            self.out.put(b"reboot failed, rc=");
            self.out.put_hex32(rc);
            self.out.put(b"\r\n");
        }
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

/// A firmware-image transport for the update path -- USB console (host) or
/// UART (a peer board). The paced protocol (size + CRC, 256-byte pages, `.`
/// ACKs) is the same over each; only the byte source/sink differs.
#[derive(Copy, Clone, PartialEq)]
enum Link {
    Usb,
    Uart,
}

/// 16-bit additive checksum of a page, for per-page integrity over UART.
fn page16(data: &[u8]) -> u16 {
    data.iter().fold(0u16, |a, &b| a.wrapping_add(b as u16))
}

/// Parse and range-check the `<size-hex> <crc32-hex>` update arguments.
fn parse_update_args(
    size: Option<&str>,
    crc: Option<&str>,
) -> Option<(u32, u32)> {
    const IMAGE_MAX: u32 = 0x4_0000; // the 256 KiB LOAD_MAP window
    let size = u32::from_str_radix(size?, 16).ok()?;
    let crc = u32::from_str_radix(crc?, 16).ok()?;
    if size == 0 || size > IMAGE_MAX {
        return None;
    }
    Some((size, crc))
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
        adc: Rp235xAdc::from(ADC.get_task_id()),
        pwm: Rp235xPwm::from(PWM.get_task_id()),
        #[cfg(feature = "cyw43")]
        cyw43: Rp235xCyw43::from(CYW43.get_task_id()),
        #[cfg(feature = "mailbox")]
        mailbox: Rp235xMailbox::from(MAILBOX.get_task_id()),
        #[cfg(feature = "slink")]
        slink: Rp235xSlink::from(SLINK.get_task_id()),
        #[cfg(feature = "ws2812")]
        ws2812: Rp235xWs2812::from(WS2812.get_task_id()),
        #[cfg(feature = "ds1302")]
        ds1302: Rp235xDs1302::from(DS1302.get_task_id()),
        #[cfg(feature = "sdcard")]
        sdcard: Rp235xSdcard::from(SDCARD.get_task_id()),
        out: Out {
            usb,
            buf: [0u8; 256],
            len: 0,
        },
        heartbeat: true,
        i2c_khz: 100,
    };

    // Let enumeration settle, then greet.
    hl::sleep_for(1500);
    shell
        .out
        .put(b"\r\nHubris on RP2350 / Pico 2 -- type `help`\r\n");
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
