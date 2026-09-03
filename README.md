# ESP32-P4 LVGL Dashboard Firmware

PlatformIO/Arduino firmware for an ESP32-P4 board driving a 1024x600 LCD touch
dashboard for school science probes, used by students and teachers together.

Sensors reach it three ways - wired to the board, over BLE from an ESP32 node,
or over BLE from a commercial PASCO sensor - and from the measurement screen's
point of view there is no difference between them. Readings go to the screen,
to CSV on an SD card, and to 지능형 과학실.

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
- **Tabs**: 홈 · 측정 · 기록 · 블루투스 · WiFi · 설정. 블루투스 and WiFi are
  where a lesson actually goes wrong - a node that dropped, a network that did
  not join - so they are one tap from anywhere rather than buried in 설정.
- **Wireless sensors**: up to three at once over BLE, each appearing on the
  home grid like a sensor wired to the board. Two kinds are supported:

  | Kind | What it is | Reports |
  |---|---|---|
  | `SciNode-XXXX` | an ESP32 running [`sensor_node/`](sensor_node/) | whatever sensor is plugged into it |
  | `Pressure 581-124` | PASCO PS-3203 | hPa |
  | `Temperature 171-753` | PASCO PS-3201 | °C |

  All connected nodes are recorded together, so pressure from one sensor and
  temperature from another fill two columns of one experiment.

- **보일의 법칙**: a second view for the pressure sensor, reached from the
  measurement screen. The volume is set by hand and each point is held
  deliberately - set the syringe, let the reading settle, record it - so the
  graph is pressure against volume rather than against time, with the fitted
  P = k/V drawn through the points and P x V beside every row.

- **Data logging**: measurement sessions can be recorded to CSV on an
  SD/SDMMC card, previewed on-device, and uploaded to 지능형 과학실 in
  real-time or batch mode over Wi-Fi/HTTPS.
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

The node works out for itself what is plugged into it, so swapping the sensor
needs no code change on either board:

| Address | Part | Reports |
|---|---|---|
| 0x76 / 0x77 | DPS310 | 온도, 기압 |
| 0x48 | TMP117 | 정밀온도 |
| 0x62 | SCD41 | 이산화탄소, 온도, 습도 |
| 0x40 | INA228 | 전류, 전압, 전력 |
| 0x29 | TSL2591 **or** VL53L1X | 조도 / 거리 |
| - | DS18B20 | 수온 (1-Wire; the pin is searched for) |

0x29 belongs to two parts, so it is settled by reading each one's ID register.
A sensor swapped while the node is running is noticed within three seconds -
the address stops acknowledging - and the P4 follows because every packet
carries its own names and units.

The drivers in [`sensor_node/src/sensors.cpp`](sensor_node/src/sensors.cpp) are
the P4's own, ported to `Wire` with their register sequences and compensation
unchanged, so a part reads the same number whichever board it hangs off.

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

## PASCO wireless sensors

The P4 connects to these directly; no node is involved. Their protocol is
published in PASCO's own [BLE examples](https://github.com/phasematching/pasco-BLE-examples)
and the `pasco-ble` library that ships the datasheets:

```
4a5c000<service>-000<char>-0000-0000-5c1e741f1c00
  service 0 = the device, 1 = the attached sensor
  char    2 = commands, written without response
          3 = replies and events

0x05, <bytes>  one sample
replies begin 0xC0: status, echoed command, payload
```

The sample is two bytes of raw ADC and the reading comes from the datasheet's
own calibration - a two-point factory fit for the PS-3203, a linear conversion
for the PS-3201. Pressure is published in hPa rather than the datasheet's kPa,
so it shares a scale with the DPS310 and the verified `PRS` upload code.

Tapping a device on the 블루투스 tab that is not one of this project's nodes
explores it instead: every service, every characteristic and its properties,
and the raw bytes of anything that notifies. That is how the above was
established, and it is what to reach for when a new make of sensor turns up.

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
