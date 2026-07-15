# SB Components MusicPi (Pico audio expansion) research notes

Target for the Hubris `drv/rp235x-i2s` audio bring-up (2026-07-15), replacing
the loose GY-PCM5102 module (which never unmuted; suspect XSMT solder strap).
The Pico 2 seats in the female headers; all wiring is fixed by the HAT.

Sources: https://github.com/sbcshop/MusicPi_Software (pin map, images -- local
copies `MusicPi_Pinouts.png`, `gain_setting.png`),
https://github.com/sbcshop/MusicPi_Hardware (schematic -- local copy
`musicpi-schematic.pdf`, 3 sheets, V1.0 2024-02-26),
https://shop.sb-components.co.uk/products/musicpi-high-quality-stereo-audio.

Schematic-verified facts: DAC SCK is grounded on-board (3-wire PLL mode ✓);
the TF socket's **CD (card-detect) pin is UNCONNECTED** -- presence must be
probed by attempting SD init (CMD0); buttons BT1-3 (GP2/3/4) have 10K pull-ups,
switches to GND (active-low); WS2812B-2020 on GP26 is powered from VBUS;
USB-C is power-only; speaker amps are one NS4150C per channel fed from the
DAC line-out, always on -- the headphone amp (and DAC mute) gate on GP22.

## Audio chain

- **DAC: TI PCM5100A** -- same PCM510xA family as the PCM5102A; our datasheet
  `docs/pcm5102a-research/pcm5102a-datasheet.pdf` (SLAS859C) covers it.
  Same 3-wire I2S + internal BCK PLL. 2.1 Vrms line out.
- **Speaker amp: NS4150** class-D, 3 W per channel (speaker headers + JST).
- **Headphone amp: PAM8908JER** feeding the 3.5 mm jack.
- Outputs: 3.5 mm jack (amplified), raw unamplified L/R header, speaker headers.

## Pin map (Pico GPIO)

| Signal | GPIO | Notes |
|---|---|---|
| I2S DIN | GP9 | PIO OUT pin |
| I2S BCK | GP10 | PIO side-set bit 0 |
| I2S LRCK | GP11 | PIO side-set bit 1 (BCK+1, consecutive) |
| DAC XSMT + amp EN | GP22 | ONE net: PCM5100A XSMT and PAM8908 EN, 10K pull-up to 3V3. Drive HIGH = unmute + amp on (schematic sheet 1) |
| Amp gain | GP20/GP21 | **NEVER drive as outputs**: the DIP switches ("MODE SEL", SW1/SW2) drive these nets to **VBUS (5V!)** or GND -- a GPIO output fights a 5V rail. Inputs only |
| TF/SD card | GP16-19 | SPI0: MISO=16, CS=17, SCK=18, MOSI=19 |
| TFT ST7789V | GP14 SCLK, GP15 MOSI, GP6 D/C, GP12 RST, GP13 CS, GP7 BL | SPI1-capable pins |
| Buttons | GP2/GP3/GP4 | BT1..BT3 |
| WS2812 RGB | GP26 | |
| Power | USB-C (5 V) on the HAT; Pico USB still used for CDC/flash | |

## Conflicts with app/demo-pi-pico-2-sd (as of the i2s branch)

- **sdcard task (SPI1, GP10-13) collides with I2S BCK/LRCK (GP10/11)** and the
  TFT RST/CS (GP12/13). The i2s task starts after sdcard and re-muxes GP10/11
  to PIO0, so the tone works, but SD/`play` commands are broken until the SD
  driver moves to the MusicPi's SPI0 pins (GP16-19).
- pwm_driver's jack pins GP18/19 = MusicPi SD SCK/MOSI: do not run pwm `play`/
  `audio` commands on this HAT (they re-mux GP18/19 to PWM).
- ds1302 RTC task drives GP6-8 = TFT D/C + backlight -- cosmetic flicker only;
  disable ds1302 when the TFT driver lands.
- i2c_driver GP4/5: GP4 doubles as BT3 (unused -- fine).

## Follow-ups for full MusicPi support

1. Remap sdcard to SPI0 GP16-19 (matches the proven sdcard driver pattern).
2. ST7789V TFT driver (1.14", 135x240) on GP13-15/6/7/12.
3. Buttons GP2-4, WS2812 on GP26 (driver exists: drv/rp235x-ws2812).
4. Retire pwm_driver from this app (shell audio commands -> i2s).
