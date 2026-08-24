# ESP32-P4 LVGL Dashboard Firmware

PlatformIO/Arduino firmware for an ESP32-P4 board driving a 1024x600 LCD
touch dashboard for school science probes. It combines an LVGL UI, eight
selectable sensors, SD card logging, and upload to 지능형 과학실, with Korean
font rendering support.

## Features

- **Display & touch**: 1024x600 LCD UI built with [LVGL](https://lvgl.io/),
  driven through a GT911 capacitive touch panel.
- **Sensors**: pick one on the home screen. Most sit on the soft-I2C bus
  (GPIO2 SDA / GPIO3 SCL); the encoder and the DS18B20 are digital.

  | Tile | Part | Reports | Bus |
  |---|---|---|---|
  | 온도 · 기압 | DPS310 | °C, hPa | I2C 0x76/0x77 |
  | 수온 | DS18B20 | °C | 1-Wire GPIO2 |
  | 이산화탄소 | SCD41 | ppm, °C, %RH | I2C 0x62 |
  | 조도 | TSL2591 | lx | I2C 0x29 |
  | 정밀 온도 | TMP117 | °C | I2C 0x48 |
  | 거리 | VL53L1X | mm | I2C 0x29 |
  | 전압 · 전류 | INA228 | A, V, W | I2C 0x40 |
  | 회전 | Grove 광학 로터리 엔코더 | °, °/s | 센서포트 A/B |

  설정 → **센서 검색** walks the I2C bus and lists what actually answers,
  which is the first thing to try when a sensor reads nothing.
- **Data logging**: measurement sessions can be recorded to CSV on an
  SD/SDMMC card, previewed on-device, and uploaded to the cloud in
  real-time or batch mode over Wi-Fi/HTTP(S).
- **Korean font support**: `src/korean_14.c` and `src/korean_16.c` carry the
  full Hangul block; `src/korean_24_bold.c`, `src/digits_64.c` and
  `src/digits_48.c` are subsets cut from Malgun Gothic Bold by
  `tools/make_heading_font.py`, which `platformio.ini` runs before every build
  so a newly added Korean string cannot ship as tofu.

## BLE sensor nodes

The board is also a hub. [`sensor_node/`](sensor_node/) is a separate firmware
for a plain ESP32 dev board: it advertises the service the P4 scans for and
notifies one reading per second, and the P4 treats those readings like any
sensor on its own bus - measurement screen, CSV, the lot.

```bash
cd sensor_node && pio run -t upload
```

Then on the P4: the **블루투스** tab -> 다시 검색, tap the `SciNode-XXXX` row to
connect, and press **이 센서로 측정**. After a scan the board also links itself
to every node advertising this project's service, so a hub power-cycled
mid-lesson comes back talking to the room on its own.

**Up to three nodes at once** - that is what the controller holds
(`CONFIG_BT_NIMBLE_MAX_CONNECTIONS=3`), and the P4 keeps advertising as a
peripheral too, so a phone connecting to the board takes one of the three. All
linked nodes appear on the 블루투스 tab with their current reading; tapping one
makes it the node being recorded. Only one at a time is: the sample buffers,
the CSV and the 지능형 과학실 session each describe a single experiment.

The node ships sending an obvious 0-100 test ramp. Replace `readNodeSensor()`
at the bottom of [`sensor_node/src/main.cpp`](sensor_node/src/main.cpp) with a
real sensor read - it is the only function meant to be edited.

Two things to know about the payload, which is one line of
`<time_s>,<sensor>,<value>,<unit>`:

- Commas separate the fields, so no field may contain one.
- The unit is drawn in the 24 px subset font, so keep it ASCII (`C`, `hPa`,
  `lx`). A Korean unit renders as empty boxes; the quantity name is drawn in
  the full-range font and can be Korean.

Readings reach 지능형 과학실 by unit, since a node names its own quantities:
`C`/`℃` go up as `TPR` on channel 01 and `hPa` as `PRS` on channel 02, the
pairing the DPS310 already uses. A unit outside that table is not uploaded and
is named on screen as excluded, rather than being filed under a guessed
sensorType.

Bluetooth and the TLS uploads draw on the same pool: WiFi and BLE both reach
the ESP32-C6 over one SDIO link, and that driver needs DMA-capable internal
buffers. With a node linked and an upload starting, the board used to reach the
handshake with 32 kB contiguous left, and ESP-Hosted asserted in
`sdio_rx_get_buffer()` - a reboot loop, mid-lesson.

LVGL now allocates from PSRAM (`LV_MEM_CUSTOM_ALLOC` in
[`include/lv_conf.h`](include/lv_conf.h)), which costs the UI nothing and hands
back about 150 kB of internal RAM: building all eight screens no longer moves
the internal figure at all. An upload also refuses, with a message, if the
largest DMA block is under 56 kB, so a shortage says so instead of restarting
the board. `logHeapState()` prints the figures before every POST.

## Hardware pinout

See [`include/pins_config.h`](include/pins_config.h) for the LCD and touch
panel pin assignments (1024x600 LCD, GT911 touch controller).

### Two constants to check against your own hardware

Both are guesses that produce plausible-looking but wrong numbers if they do
not match the parts on your bench.

- `INA228_SHUNT_OHMS` in [`src/main.cpp`](src/main.cpp) is set to the 0.015 ohm
  resistor on the Adafruit breakout. Bus voltage does not depend on it, so
  check voltage first; if current reads high or low by a fixed ratio, this is
  why.
- `ENCODER_COUNTS_PER_REV` is set to 80, which is a 20-slot disk at four counts
  per slot. The Grove module counts slots in whatever disk is fitted, so this
  is a property of your wheel, not the module. Turn the wheel exactly once and
  read 각도: 360 degrees means it is right, 180 means the value is doubled.

### Grove 광학 로터리 엔코더 wiring

The board has one fixed sensor connector, so the encoder's A and B outputs land
on the same two signal lines every other sensor uses: **A on SDA (GPIO2)** and
**B on SCL (GPIO3)**, the way the DS18B20 already shares GPIO2. Only one sensor
mode runs at a time, and switching away from 회전 detaches the interrupts so
the soft-I2C driver can drive those pins again.

Because it is a quadrature device rather than an I2C one, it never answers an
address, so 센서 검색 reports it separately: press it and keep the wheel
turning, and it names whichever pins actually moved.

Power it from **3.3 V** - the module accepts 5 V, but then its outputs are 5 V
and the P4's pins are not 5 V tolerant.

It is a TCUT1600X01 photo-interrupter, so it only produces edges when a slotted
disk passes through its gate. A wheel with no slits, or a module with nothing
in the gap, reads a steady zero no matter how fast it spins.

## Getting started

This project uses [PlatformIO](https://platformio.org/). With the PlatformIO
CLI installed:

```bash
pio run
```

```bash
pio run -t upload
```

```bash
pio device monitor
```

Or open the folder in VS Code with the PlatformIO extension installed and
use the toolbar equivalents.

## Project layout

- `src/main.cpp` - main application: LVGL UI, sensor driver, SD logging,
  and cloud upload logic.
- `include/pins_config.h` - board pin definitions.
- `include/lv_conf.h` - LVGL configuration.
- `lib/` - project-local libraries (display driver, touch driver, UI
  helpers).
- `test/` - PlatformIO unit test directory (currently empty).

## Regenerating Korean fonts

`make_korean_full_font.cmd` and `make_korean_safe_fonts_12_14_16_v4_bpp2.cmd`
regenerate the bundled Korean LVGL font sources from
`NotoSansKR-Regular.ttf` and `font_symbols.txt` using
[`lv_font_conv`](https://github.com/lvgl/lv_font_conv). Run them from the
project root on Windows.
