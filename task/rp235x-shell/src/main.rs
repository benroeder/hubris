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
use drv_rp235x_flash_api::Rp235xFlash;
use drv_rp235x_gpio_api::Rp235xGpio;
use drv_rp235x_i2c_api::Rp235xI2c;
use drv_rp235x_pwm_api::Rp235xPwm;
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
task_slot!(ADC, adc_driver);
task_slot!(PWM, pwm_driver);

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

struct Shell {
    usb: UsbCons,
    gpio: Rp235xGpio,
    uart: Rp235xUart,
    spi: Rp235xSpi,
    i2c: Rp235xI2c,
    flash: Rp235xFlash,
    adc: Rp235xAdc,
    pwm: Rp235xPwm,
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
            "bench" => self.cmd_bench(words.next(), words.next()),
            "led" => self.cmd_led(words.next(), words.next()),
            "gpio" => self.cmd_gpio(words.next(), words.next(), words.next()),
            "uart" => self.cmd_uart(line, words.next()),
            "spi" => self.cmd_spi(line, words.next()),
            "i2c" => self.cmd_i2c(words.next(), words.next(), words.next()),
            "flash" => self.cmd_flash(words.next(), words.next(), words.next()),
            "rom" => self.cmd_rom(words.next()),
            "temp" => self.cmd_temp(),
            "adc" => self.cmd_adc(words.next(), words.next()),
            "slot" => self.cmd_slot(),
            "update" => self.cmd_update(words.next(), words.next()),
            "uart-update" => self.cmd_uart_update(words.next(), words.next()),
            "push" => self.cmd_push(words.next(), words.next()),
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
