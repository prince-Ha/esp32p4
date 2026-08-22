# ESP32-P4 LVGL Dashboard Firmware

PlatformIO/Arduino firmware for an ESP32-P4 board driving a 1024x600 LCD
touch dashboard. It combines an LVGL UI, a VL53L1X time-of-flight distance
sensor, SD card logging, and Wi-Fi/HTTP(S) cloud upload, with Korean font
rendering support.

## Features

- **Display & touch**: 1024x600 LCD UI built with [LVGL](https://lvgl.io/),
  driven through a GT911 capacitive touch panel.
- **Distance sensing**: VL53L1X time-of-flight sensor read over a raw
  soft-I2C driver.
- **Data logging**: measurement sessions can be recorded to CSV on an
  SD/SDMMC card, previewed on-device, and uploaded to the cloud in
  real-time or batch mode over Wi-Fi/HTTP(S).
- **Korean font support**: bundled Noto Sans KR glyph subsets
  (`src/korean_12.c`, `src/korean_14.c`, `src/korean_16.c`) generated from
  `NotoSansKR-Regular.ttf`.

## Hardware pinout

See [`include/pins_config.h`](include/pins_config.h) for the LCD and touch
panel pin assignments (1024x600 LCD, GT911 touch controller).

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
