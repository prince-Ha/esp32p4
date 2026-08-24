// V3.8 KNOWN-GOOD BUILD - v3.7 unchanged + raw Soft-I2C VL53L1X
#pragma GCC push_options
#pragma GCC optimize("O2")

#include <Arduino.h>
#if __has_include(<WiFi.h>)
#include <WiFi.h>
#define HAS_ARDUINO_WIFI 1
#else
#define HAS_ARDUINO_WIFI 0
#endif

#include <assert.h>
#include <stdio.h>
#include <sys/stat.h>
#include <string.h>
#include <time.h>
#include <dirent.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#if __has_include(<HTTPClient.h>) && __has_include(<WiFiClientSecure.h>)
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#define HAS_HTTP_CLIENT 1
#else
#define HAS_HTTP_CLIENT 0
#endif

// ESP32-P4 + ESP-Hosted 환경에서는 Arduino HTTPClient/WiFiClientSecure가
// 빠져 있는 경우가 있습니다. 이때는 ESP-IDF의 esp_http_client를 사용합니다.
#if __has_include("esp_http_client.h")
#include "esp_http_client.h"
#define HAS_ESP_HTTP_CLIENT 1
#else
#define HAS_ESP_HTTP_CLIENT 0
#endif

#if __has_include("esp_crt_bundle.h")
#include "esp_crt_bundle.h"
#define HAS_ESP_CRT_BUNDLE 1
#else
#define HAS_ESP_CRT_BUNDLE 0
#endif

#include "esp_heap_caps.h"
#include "esp_err.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/gpio.h"

#if __has_include("esp_ldo_regulator.h")
#include "esp_ldo_regulator.h"
#define HAS_ESP_LDO 1
#else
#define HAS_ESP_LDO 0
#endif

#if __has_include("esp_wifi.h")
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/netdb.h"   // getaddrinfo for the DNS probe
#define HAS_IDF_WIFI 1
#else
#define HAS_IDF_WIFI 0
#endif

#if __has_include("esp_hosted.h")
#include "esp_hosted.h"
#define HAS_ESP_HOSTED 1
#else
#define HAS_ESP_HOSTED 0
#endif

#if HAS_IDF_WIFI
#ifndef ESP_ERR_WIFI_INIT_STATE
#define ESP_ERR_WIFI_INIT_STATE ESP_FAIL
#endif
#ifndef WIFI_AUTH_OWE
#define WIFI_AUTH_OWE WIFI_AUTH_OPEN
#endif
#endif

#include "lvgl.h"
#include "pins_config.h"
#include "lcd/jd9165_lcd.h"
#include "touch/gt911_touch.h"
#include "ble_sensor.h"

// =====================================================
// Korean font
// src/korean_16.c 안의 실제 폰트 이름도 korean_16 이어야 합니다.
// =====================================================
// Display weights, cut from Malgun Gothic Bold by tools/make_heading_font.py.
// Noto Sans KR ships here in Regular only, so a real bold needs another file.
LV_FONT_DECLARE(korean_24_bold);

LV_FONT_DECLARE(korean_16);
LV_FONT_DECLARE(korean_14);

// Digits only, for the measurement readout. A full Hangul face at this size
// would cost megabytes; the value itself never needs one.
LV_FONT_DECLARE(digits_64);
LV_FONT_DECLARE(digits_48);

#define FONT_KR &korean_16
#define FONT_TABLE &korean_16
#define FONT_GRAPH_SMALL &korean_16

// Headings. 24 px is the largest Hangul face built into the firmware, so it
// carries every screen title and the sensor tile names.
#define FONT_KR_HEAD &korean_24_bold
#define FONT_KR_SMALL &korean_14
#define FONT_VALUE &digits_64
// Used when a sensor reports two or three quantities that share the row.
#define FONT_VALUE_SMALL &digits_48

#define FONT_KR_NORMAL FONT_KR
// =====================================================
// WiFi scan result buffer
// 실제 스캔은 ESP32-C6 통신부에서 채워 넣는 구조입니다.
// =====================================================
#define WIFI_SCAN_MAX 12

struct WifiNetworkInfo
{
  char ssid[33];
  int rssi;
  bool secure;
};

static WifiNetworkInfo wifiScanResults[WIFI_SCAN_MAX];

// =====================================================
// DPS310 Soft I2C 설정
// =====================================================
#define SOFT_SDA 2
#define SOFT_SCL 3
#define DPS310_ADDR 0x77

// =====================================================
// Sensor port mode
// - DPS310 / SCD41 / TSL2591: GPIO2=SDA, GPIO3=SCL (Soft-I2C)
// - DS18B20 waterproof: GPIO2=DQ, GPIO3=unused
// 현재 하드웨어는 같은 GPIO2/3 포트를 센서별로 갈아 끼우는 구조입니다.
// 여러 센서를 동시에 꽂는 경우에는 배선/주소 충돌과 DS18B20 GPIO를 별도로 설계하세요.
// =====================================================
#define SENSOR_MODE_DPS310    1
#define SENSOR_MODE_DS18B20   2
#define SENSOR_MODE_SCD41     3
#define SENSOR_MODE_TSL2591   4
#define SENSOR_MODE_TMP117    5
#define SENSOR_MODE_VL53L1X   6
#define SENSOR_MODE_INA228    7
#define SENSOR_MODE_ENCODER   8

// A sensor on another board, reached over BLE. It has no tile on the home
// grid because it is not wired to this board: it is chosen on the 블루투스
// screen, where it was connected.
#define SENSOR_MODE_BLE       9

// A node publishes once a second. Past this the link is up but the node has
// gone quiet, and showing its last number as if it were current would be a
// lie, so the reading is refused instead.
#define BLE_READING_MAX_AGE_MS 5000

// =====================================================
// Grove 광학 로터리 엔코더 (TCUT1600X01)
//
// Not an I2C part: the module's two phototransistors drive a quadrature A/B
// pair, so it will never appear in the I2C scan. Both edges of both channels
// are counted, giving four counts per slot.
//
// The board has one fixed sensor connector, so A and B land on the same two
// signal lines every other sensor uses - SOFT_SDA and SOFT_SCL - the way the
// DS18B20 already shares SOFT_SDA. Only one sensor mode runs at a time, but
// the interrupts must come off those pins before anything bit-bangs I2C on
// them again, or every clock edge of every transaction fires the ISR.
//
// Power it from 3.3 V. The module accepts 3.3 V or 5 V, but at 5 V its outputs
// are 5 V and the P4's pins are not 5 V tolerant.
// =====================================================
#define ENCODER_PIN_A SOFT_SDA
#define ENCODER_PIN_B SOFT_SCL

// Counts for one full turn of the disk. A photo-interrupter counts slots in
// whatever disk is fitted, so this depends on the wheel, not the module: a
// 20-slot disk at four counts per slot is 80. Turn the wheel exactly once and
// read the angle — 360° means this is right, half that means it is doubled.
#define ENCODER_COUNTS_PER_REV 80

// =====================================================
// INA228 전압·전류·전력 (raw Soft-I2C)
// =====================================================
#define INA228_ADDR                0x40
#define INA228_REG_CONFIG          0x00
#define INA228_REG_ADC_CONFIG      0x01
#define INA228_REG_SHUNT_CAL       0x02
#define INA228_REG_VBUS            0x05
#define INA228_REG_CURRENT         0x07
#define INA228_REG_POWER           0x08
#define INA228_REG_MANUFACTURER_ID 0x3E
#define INA228_MANUFACTURER_TI     0x5449

// The shunt on the breakout board. Adafruit's INA228 carries 0.015 Ω rated to
// 10 A, which is the common case; a board with a different resistor needs this
// changed or every current and power reading is scaled wrong.
#define INA228_SHUNT_OHMS   0.015f
#define INA228_MAX_CURRENT  10.0f

// Datasheet fixed scalings.
#define INA228_VBUS_LSB_V   0.0001953125f
#define INA228_POWER_LSB_K  3.2f
#define ACTIVE_SENSOR_DEFAULT SENSOR_MODE_DPS310

#define DS18B20_DQ_GPIO GPIO_NUM_2
#define DS18B20_CONVERT_MS 1000
#define NO_PRESSURE_VALUE NAN

// =====================================================
// VL53L1X Time-of-Flight distance sensor (raw Soft-I2C)
// - Same GPIO2=SDA / GPIO3=SCL port.
// - 7-bit I2C address: 0x29
// - VL53L1X uses 16-bit register addresses.
// - No Wire / Adafruit / ESP-IDF legacy I2C driver.
// - Initialization follows the small-footprint ST ULD model:
//   reset -> boot -> model ID -> default config -> ranging.
// =====================================================
#define VL53L1X_ADDR                          0x29
#define VL53L1X_REG_SOFT_RESET                0x0000
#define VL53L1X_REG_VHV_TIMEOUT               0x0008
#define VL53L1X_REG_VHV_INIT                  0x000B
#define VL53L1X_REG_GPIO_HV_MUX_CTRL          0x0030
#define VL53L1X_REG_GPIO_TIO_HV_STATUS        0x0031
#define VL53L1X_REG_INTERRUPT_CLEAR           0x0086
#define VL53L1X_REG_MODE_START                0x0087
#define VL53L1X_REG_RANGE_STATUS              0x0089
#define VL53L1X_REG_RANGE_MM                  0x0096
#define VL53L1X_REG_FIRMWARE_SYSTEM_STATUS    0x00E5
#define VL53L1X_REG_MODEL_ID                  0x010F
#define VL53L1X_EXPECTED_MODEL_ID             0xEACC

static bool vl53RangingStarted = false;
static bool vl53LastReadWasWaiting = false;
static uint8_t vl53LastRawRangeStatus = 0xFF;

// ST ULD-style default register image for addresses 0x002D..0x0087.
// Stored as plain flash data; it does not instantiate another I2C stack.
static const uint8_t VL53L1X_DEFAULT_CONFIGURATION[] = {
  0x00,0x00,0x00,0x01,0x02,0x00,0x02,0x08,0x00,0x08,
  0x10,0x01,0x01,0x00,0x00,0x00,0x00,0xFF,0x00,0x0F,
  0x00,0x00,0x00,0x00,0x00,0x20,0x0B,0x00,0x00,0x02,
  0x0A,0x21,0x00,0x00,0x05,0x00,0x00,0x00,0x00,0xC8,
  0x00,0x00,0x38,0xFF,0x01,0x00,0x08,0x00,0x00,0x01,
  0xDB,0x0F,0x01,0xF1,0x0D,0x01,0x68,0x00,0x80,0x08,
  0xB8,0x00,0x00,0x00,0x00,0x0F,0x89,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x01,0x0F,0x0D,0x0E,0x0E,0x00,
  0x00,0x02,0xC7,0xFF,0x9B,0x00,0x00,0x00,0x01,0x01,
  0x40
};

// =====================================================
// TMP117 precision temperature sensor (raw Soft-I2C)
// - Same existing GPIO2=SDA / GPIO3=SCL sensor port.
// - Default I2C address: 0x48
// - Temperature result register: 0x00
// - Device ID register: 0x0F, expected 0x0117
// - Temperature format: signed 16-bit two's complement,
//   0.0078125 degC per LSB.
// - IMPORTANT: no Arduino Wire / Adafruit I2C library is used.
// =====================================================
#define TMP117_ADDR                 0x48
#define TMP117_REG_TEMPERATURE      0x00
#define TMP117_REG_DEVICE_ID        0x0F
#define TMP117_EXPECTED_DEVICE_ID   0x0117
#define TMP117_LSB_C                0.0078125f

static bool tmp117LastReadWasWaiting = false;

// INA228 measures three things at once; current is the headline and the other
// two ride along, the way SCD41's temperature and humidity do.
// Quadrature state. Written from an interrupt, so volatile.
static volatile int32_t encoderCount = 0;
static volatile uint8_t encoderLastState = 0;
// Counts every edge the ISR sees, including the invalid ones the
// quadrature table scores as zero, so a bouncing input still shows up.
static volatile uint32_t encoderEdgeCount = 0;
static bool encoderReady = false;
static int32_t encoderPrevCount = 0;
static unsigned long encoderPrevMs = 0;
static float encoderLastRateDegPerS = NAN;

// Filled in from each notification: a node names its own quantity and unit,
// so unlike every other sensor here these are not compile-time constants.
static char bleNodeQuantity[24] = "블루투스";
static char bleNodeUnit[24] = "-";

static bool ina228Ready = false;
static float ina228LastPowerW = NAN;
static float ina228CurrentLsb = 0.0f;

// =====================================================
// Adafruit / Sensirion SCD41 CO2 sensor (Soft-I2C)
// - Same Soft-I2C bus as DPS310/TSL2591: GPIO2=SDA, GPIO3=SCL
// - Default I2C address: 0x62
// - Periodic measurement updates every ~5 s.
// - CO2 is used as the primary value (ppm).
// - Temperature/RH returned by SCD41 are retained as diagnostics only;
//   this UI keeps one primary value for single-value sensors.
// =====================================================
#define SCD41_ADDR 0x62
#define SCD41_CMD_START_PERIODIC_MEASUREMENT 0x21B1
#define SCD41_CMD_READ_MEASUREMENT           0xEC05
#define SCD41_CMD_STOP_PERIODIC_MEASUREMENT  0x3F86
#define SCD41_CMD_GET_DATA_READY_STATUS      0xE4B8
#define SCD41_CMD_GET_SERIAL_NUMBER          0x3682
#define SCD41_CMD_REINIT                     0x3646
#define SCD41_CMD_WAKE_UP                    0x36F6
#define SCD41_COMMAND_DELAY_MS 1
#define SCD41_WAKE_DELAY_MS 30
#define SCD41_REINIT_DELAY_MS 30
#define SCD41_STOP_DELAY_MS 500

// The 2026-07-21 ISL document has a generic Concentration=CTRT type,
// but no explicit CO2/SCD41 sensor type. Keep direct ISL disabled until
// the server mapping is verified. Set to 1 only after confirming CTRT accepts CO2 ppm.
// CTRT (농도) is the closest the 2026-07-21 appendix offers for CO2; the spec
// has no dedicated code. The registration path accepts 001/015 and reports a
// refusal on screen, so let the server decide instead of leaving CO2 silently
// unable to transmit.
#define SCD41_DIRECT_ISL_ENABLED 1

// =====================================================
// TSL2591 digital light sensor
// - Same Soft-I2C bus as DPS310: GPIO2=SDA, GPIO3=SCL
// - I2C address is fixed at 0x29.
// - Medium gain (25x), 200 ms integration.
// =====================================================
#define TSL2591_ADDR 0x29
#define TSL2591_COMMAND_BIT 0xA0
#define TSL2591_REG_ENABLE 0x00
#define TSL2591_REG_CONTROL 0x01
#define TSL2591_REG_ID 0x12
#define TSL2591_REG_STATUS 0x13
#define TSL2591_REG_CHAN0_LOW 0x14
#define TSL2591_REG_CHAN1_LOW 0x16
#define TSL2591_ENABLE_PON 0x01
#define TSL2591_ENABLE_AEN 0x02
#define TSL2591_INTEGRATION_200MS 0x01
#define TSL2591_GAIN_MEDIUM 0x10
#define TSL2591_ATIME_MS 200.0f
#define TSL2591_AGAIN 25.0f
#define TSL2591_LUX_DF 408.0f
#define TSL2591_LUX_COEFB 1.64f
#define TSL2591_LUX_COEFC 0.59f
#define TSL2591_LUX_COEFD 0.86f

#define MEAS_BACKUP_MAGIC 0x44533138UL
#define MEAS_BACKUP_VERSION 6
#define MEAS_BACKUP_EVERY_SAMPLES 30
#define MEAS_BACKUP_MIN_INTERVAL_MS 30000UL

// =====================================================
// Stability / performance configuration
// =====================================================
// 1: use GT911 touch. If touch library aborts during boot, patch gt911_touch.cpp
//    using the included script instead of leaving this off permanently.
#define ENABLE_GT911_TOUCH 1

// Smaller LVGL draw buffers reduce memory pressure and CPU load.
// 80 rows is a stable compromise for 1024x600.
// v19 display stability compromise:
// PSRAM buffers caused horizontal strip artifacts.
// Large/double internal DMA buffers caused measurement instability.
// Use ONE small internal DMA-capable buffer.
#define LVGL_BUFFER_ROWS 12
#define LVGL_USE_INTERNAL_DMA_BUFFER 1
#define LVGL_SINGLE_DMA_BUFFER 1

// 0 = partial refresh. Full refresh caused horizontal strip artifacts on this JD9165 panel.
#define LVGL_USE_FULL_REFRESH 0

// LVGL handler throttling. 5 ms is usually enough for touch/UI responsiveness.
#define LVGL_HANDLER_PERIOD_MS 8

// Stack for the background WiFi/API task. It performs TLS handshakes, so it
// needs far more than the FreeRTOS default; 12 kB was overflowing.
#define WIFI_API_TASK_STACK_BYTES 24576

// Main loop idle time. Prevents 100% CPU busy-loop.
#define MAIN_LOOP_IDLE_MS 2

// UI/touch stability
#define UI_LIGHT_BG 0xEEF2F7

// =====================================================
// Light UI palette
// One accent carries every interactive affordance; semantic colours are kept
// separate from it so "connected" never reads as "tappable". Every state that
// uses colour also carries a word, so it survives colour-blind viewing and the
// glare of a classroom projector.
// =====================================================
#define UI_BG          0xF4F6F8
#define UI_SURFACE     0xFFFFFF
#define UI_LINE        0xE2E7EE
#define UI_TEXT        0x14181F
#define UI_TEXT_2      0x4A5566
#define UI_TEXT_3      0x79849A
// A step lighter again, for the part number and resolution under a tile
// name: present when looked for, silent otherwise.
#define UI_TEXT_4      0xA6AEBB
#define UI_ACCENT      0x0B6BCB
#define UI_ACCENT_SOFT 0xF2F7FE
#define UI_ACCENT_TINT 0xE9F1FC
#define UI_OK          0x1B7A44
#define UI_DANGER      0xC0392B

#define UI_STATUSBAR_H 44
#define UI_TABBAR_H    54
#define TOUCH_PRESS_DEBOUNCE_MS 12UL
#define TOUCH_RELEASE_DEBOUNCE_MS 45UL
#define TOUCH_REARM_MS 55UL

// Direct LCD clears left persistent dark strips on this panel, so page cleanup
// is handled by opaque LVGL backgrounds plus targeted invalidation instead.

#define REG_PRS_B2    0x00
#define REG_TMP_B2    0x03
#define REG_PRS_CFG   0x06
#define REG_TMP_CFG   0x07
#define REG_MEAS_CFG  0x08
#define REG_CFG_REG   0x09
#define REG_RESET     0x0C
#define REG_PROD_ID   0x0D
#define REG_COEF      0x10
#define REG_TMP_COEF  0x28

const int I2C_DELAY_US = 5;

// =====================================================
// ESP32-P4 내장 SDMMC 핀
// =====================================================
#define SD_CLK 43
#define SD_CMD 44
#define SD_D0  39
#define SD_D1  40
#define SD_D2  41
#define SD_D3  42

#define MOUNT_POINT "/sdcard"
#define CSV_DEFAULT_FILE "sensor_log.csv"

static char csvFileName[64] = CSV_DEFAULT_FILE;
static char csvPath[128] = MOUNT_POINT "/" CSV_DEFAULT_FILE;

// primary, secondary, third - the most any one sensor reports.
#define CSV_VALUE_COLUMNS 3
#define CSV_HEADER_LINE "no,time_s,sensor,primary_value,primary_unit,secondary_value,secondary_unit,third_value,third_unit"

// Rows buffered before the CSV handle is flushed to the card. At the 1 Hz
// sample rate this caps data loss on a power cut at CSV_FLUSH_EVERY_ROWS
// seconds without paying for an fsync every sample.
#define CSV_FLUSH_EVERY_ROWS 10

// WiFi detection 테스트 시에는 SDMMC가 C6 SDIO 통신과 충돌할 수 있습니다.
// SD는 부팅 자동 마운트 대신 설정 화면에서 수동으로 켜는 구조가 안전합니다.
#define SD_AUTO_MOUNT_ON_BOOT 0
#define CSV_LOG_DEFAULT false

// SD 초기화 안정성을 우선합니다. 성공 확인 후 false/고속으로 올리세요.
#define SD_USE_1BIT true


// =====================================================
// 전송 구조
// ESP32-P4 -> 지능형 과학실 ON 오픈API (직접 HTTPS)
//
// 중요:
// ESP32-P4에서 지능형과학실 ON으로 직접 HTTPS 전송하지 않습니다.
// 직접 전송은 TLS 메모리 오류와 API 인증/스키마 문제 때문에 비활성화합니다.
// =====================================================

// =====================================================
// 지능형 과학실 ON API 설정
// 실제 발급값으로 교체해야 전송이 시작됩니다.
// =====================================================
#define ISL_SERVICE_KEY "PUT_YOUR_SERVICE_KEY"

// 2026-07-21 지능형 과학실 ON 오픈API 설계서 기준
// 조도(Illuminance): 실제 sendSensorType API 호출 예제(page 27)는 sensorType=ILM 사용
// 별첨 표에는 ILMN으로 기재되어 있으나, 실제 API 예제의 정상 응답(001)에 맞춰 ILM 사용
#define ISL_CHANNEL_LIGHT "01"
#define ISL_UNIT_LIGHT "Lux"

// CO2 mapping candidate from the generic "Concentration" entry in the 2026-07-21 API appendix.
// Disabled by SCD41_DIRECT_ISL_ENABLED until server-side acceptance is verified.
// Distance. The appendix gives DITC; no worked example in the spec uses a
// distance sensor, and the codes the server actually accepts have not matched
// the appendix before (조도 is ILM in the examples, ILMN in the table), so the
// registration tries the alternatives in turn the way 조도 already does.
#define ISL_CHANNEL_DISTANCE "01"
#define ISL_UNIT_DISTANCE "mm"

#define ISL_SENSOR_TYPE_CO2 "CTRT"
#define ISL_CHANNEL_CO2 "01"
#define ISL_UNIT_CO2 "ppm"

// SCD41 reports temperature and humidity alongside CO2. 습도 is HMDT in the
// appendix; temperature reuses the TPR code the server already accepts. They
// are separate sensor types, so each takes channel 01 (별첨3).
// 별첨2: 전압 Voltage VOLT, 전류 Electric current ECRT, 전력 Electric power
// EPOW. Distinct sensor types, so each takes channel 01 (별첨3).
// 별첨2: 각도 Angle ANGL, 각속도 Angular velocity ANGV.
#define ISL_SENSOR_TYPE_ANGLE    "ANGL"
#define ISL_SENSOR_TYPE_ANGVEL   "ANGV"
#define ISL_UNIT_ANGLE  "deg"
#define ISL_UNIT_ANGVEL "deg/s"

#define ISL_SENSOR_TYPE_VOLTAGE "VOLT"
#define ISL_SENSOR_TYPE_CURRENT "ECRT"
#define ISL_SENSOR_TYPE_POWER   "EPOW"
#define ISL_UNIT_VOLTAGE "V"
#define ISL_UNIT_CURRENT "A"
#define ISL_UNIT_POWER   "W"

#define ISL_SENSOR_TYPE_HUMIDITY "HMDT"
#define ISL_CHANNEL_HUMIDITY "01"
#define ISL_UNIT_HUMIDITY "%"
#define ISL_CHANNEL_SCD41_TEMP "01"


// Gates the queue that carries samples to the upload task.
#define UPLOAD_QUEUE_ENABLED 1

// =====================================================
// 실시간 직접 전송 옵션
// - 실시간 start/data/stop: ESP32 -> 지능형과학실 ON 직접 전송
// - 일괄전송: 누적 측정값을 지능형과학실로 한 번에 전송
// 직접 전송을 쓰려면 지능형과학실 설정 화면에 serviceKey를 저장해야 합니다.
// =====================================================
#define REALTIME_DIRECT_ISL_ENABLED 1
#define DIRECT_ISL_AXIS_PADDED 1

#define DIRECT_ISL_START_URL "https://api-scion.kosac.re.kr/sensorapi/startExplortProcess"
#define DIRECT_ISL_SENSOR_TYPE_URL "https://api-scion.kosac.re.kr/sensorapi/sendSensorType"
#define DIRECT_ISL_STATUS_URL "https://api-scion.kosac.re.kr/sensorapi/setExplortProcessStatus"
#define DIRECT_ISL_DATA_URL "https://api-scion.kosac.re.kr/sensorapi/sendExplortData"
#define DIRECT_ISL_STOP_URL "https://api-scion.kosac.re.kr/sensorapi/stopExplortProcess"

// HTTPS 전송 안정화 설정
// 1초마다 HTTPS를 새로 연결하면 TLS handshake 때문에 실패할 수 있습니다.
// 우선 5개 샘플마다 1번만 전송하고, 안정화되면 1로 낮추면 됩니다.
#define CLOUD_SEND_EVERY_N_SAMPLES 1  // 실시간 모드: 1초 샘플마다 전송
#define CLOUD_HTTP_TIMEOUT_MS 12000  // 전송 안정화: 지능형과학실 API 응답 지연을 고려해 timeout 확대
#define CLOUD_QUEUE_DEPTH 4  // 제어 명령과 일괄전송 요청 안정화를 위해 약간 여유
#define BATCH_UPLOAD_CHUNK_SIZE 5  // WiFi 재연결 후 안정화: 더 작은 chunk + retry


// =====================================================
// 지능형 과학실 ON API 런타임 상태
// HTTP/WiFi 작업은 별도 task에서 처리하고, 측정 loop는 큐에 넣기만 합니다.
// =====================================================
// 업로드 task로 넘기는 측정값 패킷입니다.
struct CloudSamplePacket
{
  int no;
  uint32_t timeS;
  float tempC;
  float pressureHpa;
  float humidityPct;
  char collectDate[32];
  char sensorName[32];
  char modumId[64];
  char action[16];
  char eventType[32];
  char buttonName[32];
  char screenName[32];
  char uploadMode[16];
  int sampleCountSnapshot;
  int measurementCountSnapshot;
  int wifiRssi;
};

static QueueHandle_t cloudQueue = NULL;
static uint32_t cloudQueuedDropped = 0;

static char directIslUniqueCode[128] = "";
static char directIslModumId[64] = "";
static bool directIslStartOk = false;
static bool directIslSensorTypeOk = false;
static bool directIslStatusOk = false;

// 설계서가 조도 센서코드를 두 가지로 표기합니다.
// - 실제 sendSensorType 호출 예: ILM
// - 별첨 센서타입 정의: ILMN
// 따라서 등록 시 ILM -> ILMN 순서로 자동 fallback하고,
// 실제로 성공한 코드를 데이터 전송에도 그대로 사용합니다.
static char directIslLightSensorType[8] = "ILM";
static char directIslDistanceSensorType[8] = "DITC";

static unsigned long lastWifiLostNoticeMs = 0;

static TaskHandle_t wifiApiTaskHandle = NULL;

static volatile bool islApiConfigured = false;

static char islStatusText[128] = "전송: 대기";
static char islServiceKey[128] = ISL_SERVICE_KEY;
static char islRuntimeSerialNumber[64] = "";   // 지능형 과학실 모듈/모둠 코드 → API serialNumber로 사용


#ifndef SDMMC_FREQ_PROBING
#define SDMMC_FREQ_PROBING 400
#endif

#define SD_INIT_FREQ_KHZ SDMMC_FREQ_PROBING
#define BSP_LDO_PROBE_SD_CHAN 4
#define BSP_LDO_PROBE_SD_VOLTAGE_MV 3300

#ifndef GPIO_NUM_NC
#define GPIO_NUM_NC ((gpio_num_t)-1)
#endif

static sdmmc_card_t *sdcard = NULL;

// =====================================================
// LCD / Touch / LVGL
// =====================================================
jd9165_lcd lcd = jd9165_lcd(LCD_RST);
gt911_touch touch = gt911_touch(TP_I2C_SDA, TP_I2C_SCL, TP_RST, TP_INT);
static bool touchReady = false;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1;
static lv_color_t *buf2;

// =====================================================
// Stability-first deferred UI / manual WiFi state
// =====================================================
static volatile bool touchRawDown = false;
static volatile unsigned long touchLastChangeMs = 0;
static volatile unsigned long ignoreTouchUntilMs = 0;
static volatile bool uiInputLocked = false;

static lv_obj_t *pendingScreen = NULL;
static unsigned long pendingScreenRequestMs = 0;
static const unsigned long SCREEN_RELEASE_SETTLE_MS = 45;
static const unsigned long SCREEN_TOUCH_LOCKOUT_MS = 180;

enum WifiUiCommand
{
  WIFI_UI_NONE = 0,
  WIFI_UI_SCAN,
  WIFI_UI_CONNECT,
  WIFI_UI_DISCONNECT,
  WIFI_UI_FORGET
};

static volatile WifiUiCommand pendingWifiCommand = WIFI_UI_NONE;
static char pendingWifiSsid[64] = "";
static char pendingWifiPassword[64] = "";
static volatile int pendingSensorMode = 0;

void requestScreenSwitch(lv_obj_t *screen);
void servicePendingScreenSwitch();
void servicePendingWifiCommand();
void servicePendingSensorMode();
void activateScreenNow(lv_obj_t *screen);

// =====================================================
// 화면 객체
// =====================================================
static lv_obj_t *homeScreen;
static lv_obj_t *measureScreen;
static lv_obj_t *settingsScreen;
static lv_obj_t *csvScreen;
static lv_obj_t *fileViewerScreen;
static lv_obj_t *islScreen;
static lv_obj_t *bleScreen;

static lv_obj_t *labelHomeSd;
static lv_obj_t *labelHomeSensorMode;
static lv_obj_t *labelHomeSensorStatus;
static lv_obj_t *labelHomeRecovery;
static lv_obj_t *labelHomeResetReason;
static lv_obj_t *labelHomeWifi;
static lv_obj_t *labelHomeIp;
static lv_obj_t *labelHomeSignal;

static lv_obj_t *labelTempBig;
static lv_obj_t *labelPressureBig;
static lv_obj_t *labelHumidityBig;

// Measure screen: the value is split across three labels so the digits can use
// a digits-only face while the name and unit stay in the Hangul face.
static lv_obj_t *labelMeasureSensorName;

// Up to three readings share the row: DPS310 gives 온도 and 기압, SCD41 gives
// CO2, 온도 and 습도. Each column is a caption, a number and a unit.
#define MEASURE_VALUE_MAX 3
static lv_obj_t *labelValueCaption[MEASURE_VALUE_MAX];
static lv_obj_t *labelValueNumber[MEASURE_VALUE_MAX];
static lv_obj_t *labelValueUnit[MEASURE_VALUE_MAX];
static lv_obj_t *labelMeasureIslState;
static lv_obj_t *labelMeasureModum;
static lv_obj_t *labelMeasureBle;
static lv_obj_t *btnMeasureStart;
static lv_obj_t *btnMeasureStop;
static lv_obj_t *labelStatus;
static lv_obj_t *labelSd;
static lv_obj_t *labelCount;
static lv_obj_t *labelRuntime;

static lv_obj_t *labelGraphStart;
static lv_obj_t *labelGraphEnd;

static lv_obj_t *labelMeasureTempTicks[6];
static lv_obj_t *labelMeasurePressureTicks[6];
static lv_obj_t *labelMeasureTimeTicks[6];
static lv_obj_t *labelMeasureTempAxisTitle;
static lv_obj_t *labelMeasurePressureAxisTitle;

static lv_obj_t *chart;
static lv_chart_series_t *seriesTemp;
static lv_chart_series_t *seriesPressure;

static lv_obj_t *tableData;
static lv_obj_t *labelCsvRowRange;
static char sdLastError[64] = "";

// =====================================================
// Bluetooth, ISL and file viewer screen objects
// =====================================================
static lv_obj_t *bleList;
static lv_obj_t *labelBleState;
static lv_obj_t *labelBarBleTime;
static lv_obj_t *labelBarBleWifi;
static lv_obj_t *labelBarBleSd;
static volatile bool pendingBleScan = false;
// A tapped row asks for a connection; the radio work happens in the loop.
static volatile int pendingBleConnectIndex = -1;
static volatile bool pendingBleDisconnect = false;

// Rows are rebuilt on every refresh, so the addresses they connect to live
// here and the row only carries an index.
static char bleRowAddress[BLE_SCAN_MAX_RESULTS][18];
static lv_obj_t *labelBleLink = NULL;
// Scanning bit-bangs the bus, so it runs from the loop, not a callback.
static volatile bool pendingI2cScan = false;

// Bluetooth is off unless asked for. NimBLE holds DMA-capable internal RAM,
// and that is the same pool the AES accelerator draws from for a TLS
// handshake — with it running, uploads to 지능형 과학실 began failing
// intermittently with "esp-aes: Generating DMA descriptors failed". The
// upload path is what the board is for; Bluetooth is not yet.
static bool bleEnabled = false;
bool loadBleEnabled();
void saveBleEnabled(bool enabled);
static lv_obj_t *labelBleEnabledState;
static lv_obj_t *labelBarIslTime;
static lv_obj_t *labelBarIslWifi;
static lv_obj_t *labelBarIslSd;
static lv_obj_t *labelBarFileViewerTime;
static lv_obj_t *labelBarFileViewerWifi;
static lv_obj_t *labelBarFileViewerSd;
static lv_obj_t *labelFileViewerTitle;
static lv_obj_t *fileViewerTextArea;
static lv_obj_t *labelFileViewerPageInfo;



// Home sensor grid, kept so the selected tile can follow the active sensor.
#define HOME_SENSOR_TILE_COUNT 8
// The group code is edited in a modal sheet rather than in place: the keyboard
// covers the lower third of the screen, so an inline field would either sit
// under the keyboard or have to displace the sensor grid.
static lv_obj_t *homeCodeEditor;
static lv_obj_t *labelHomeModumCode;

static lv_obj_t *homeSensorTiles[HOME_SENSOR_TILE_COUNT];
static lv_obj_t *homeSensorTileMarks[HOME_SENSOR_TILE_COUNT];
static int homeSensorTileModes[HOME_SENSOR_TILE_COUNT];

static lv_obj_t *wifiSsidTa;
static lv_obj_t *wifiPassTa;
static lv_obj_t *wifiKeyboard;
static lv_obj_t *wifiList;
static lv_obj_t *homeKeyboard;
static lv_obj_t *csvKeyboard;
static lv_obj_t *islKeyboard;
static lv_obj_t *labelUiCopyStatus;
static char uiCopiedText[256] = "";

static lv_obj_t *homeIslModuleTa;
static lv_obj_t *islServiceKeyTa;
static lv_obj_t *labelHomeIsl;
static lv_obj_t *labelCloudMode;
static lv_obj_t *homeCsvFileTa;
static lv_obj_t *labelHomeCsv;
static lv_obj_t *labelHomeCsvPath;
static lv_obj_t *labelHomeCsvPreview;
static lv_obj_t *labelSelectedSdFile;

static lv_obj_t *labelHomeDateTime;
static lv_obj_t *labelDateTime;
static lv_obj_t *labelWifiState;
static lv_obj_t *labelWifiIp;
static lv_obj_t *labelWifiSignal;
static lv_obj_t *labelWifiMode;
static lv_obj_t *labelWifiScan;
static lv_obj_t *labelSettingsWifi;
static lv_obj_t *labelSettingsSd;
static lv_obj_t *labelSettingsCsv;
static lv_obj_t *labelSettingsNote;
static lv_obj_t *labelSettingsIsl;
static lv_obj_t *islModuleTa;
static lv_obj_t *sdFileList;

// Files the list can hold. This was 1, left over from a build that cut every
// buffer it could: the loop stopped after the first directory entry, and on a
// card whose first entry is a hidden system folder no CSV ever appeared.
// 32 x 96 bytes is 3 kB, which the reclaimed internal RAM affords.
#define SD_FILE_LIST_MAX 32
static char sdListedFiles[SD_FILE_LIST_MAX][96];
static int sdListedFileCount = 0;
// Bytes shown per page in the file viewer. Also cut to a stub; 256 bytes is
// about three CSV rows.
#define CSV_FILE_VIEW_PAGE_BYTES 2048
static char csvFileViewBuffer[CSV_FILE_VIEW_PAGE_BYTES + 1024];
static char sdSelectedTextFile[96] = "";
static long csvFileViewOffset = 0;
static long csvFileViewSize = 0;
static char csvFileViewName[96] = "";

static lv_obj_t *labelBarHomeTime;
static lv_obj_t *labelBarHomeWifi;
static lv_obj_t *labelBarHomeSd;
static lv_obj_t *labelBarMeasureTime;
static lv_obj_t *labelBarMeasureWifi;
static lv_obj_t *labelBarMeasureSd;
static lv_obj_t *labelBarSettingsTime;
static lv_obj_t *labelBarSettingsWifi;
static lv_obj_t *labelBarSettingsSd;
static lv_obj_t *labelBarCsvTime;
static lv_obj_t *labelBarCsvWifi;
static lv_obj_t *labelBarCsvSd;
static lv_obj_t *labelNow;
static lv_obj_t *labelSample;
static lv_obj_t *labelElapsed;
static lv_obj_t *labelTempNow;
static lv_obj_t *labelPressureNow;

bool wifiConnected = false;
char wifiSavedSsid[64] = "";
char wifiSavedPassword[64] = "";
bool csvLoggingEnabled = CSV_LOG_DEFAULT;
bool ntpSynced = false;
bool wifiAutoRetryEnabled = false;  // manual WiFi only
bool dataClockStarted = false;
unsigned long dataFirstSampleMs = 0;

enum CloudUploadMode
{
  CLOUD_UPLOAD_REALTIME = 0,
  CLOUD_UPLOAD_BATCH = 1
};

static CloudUploadMode cloudUploadMode = CLOUD_UPLOAD_REALTIME;

// 모드 전환 안전장치
// - 실시간으로 이미 보낸 구간과 일괄전송 구간이 섞이면
//   지능형과학실 표에서 시간이 뒤섞인다.
// - 실시간 측정 중 일괄전송으로 전환하면, 이후 새로 쌓이는 샘플만 일괄전송한다.
// - 순수 일괄전송 실험은 0부터 전체를 일괄전송한다.
static int batchUploadStartIndex = 0;
static int batchUploadProgressSent = 0;
static int batchUploadProgressTotal = 0;

// =====================================================
// Lightweight batch progress UI
// - No modal overlay objects.
// - Dashboard/measure labels only.
// - WiFi/API task only updates flags/counters.
// - Actual LVGL label updates are serviced in loop().
// =====================================================
static volatile bool batchUploading = false;

// V3 DEBUG:
// 일괄전송 요청을 cloudQueue 패킷 하나에 의존하지 않고 별도 플래그로 전달합니다.
// 이렇게 하면 버튼을 눌렀는데 큐가 비워지거나 WiFi task 타이밍 때문에 요청이 사라지는 경우를 막을 수 있습니다.
static volatile bool batchUploadRequested = false;
static volatile unsigned long batchUploadRequestMs = 0;

static volatile bool batchUiDirty = true;
static volatile bool islStatusUiDirty = true;
static unsigned long lastBatchUiServiceMs = 0;


unsigned long lastAutoWifiRetryMs = 0;
const unsigned long wifiAutoRetryIntervalMs = 30000;

void updateSdStatusLabels();
void updateClockLabels();
void updateStatusBars();
void updateHomeWifiLabels();
void updateWifiRuntimeLabels();
void updateIslStatusLabels();
void updateIslModuleCodeFromUi();
void setIslStatusText(const char *text);
void queueCloudSample(int no, uint32_t timeS, float tempC, float pressureHpa);
void queueCloudAction(const char *action);
void copyCurrentModumIdTo(char *out, size_t outSize);
void clearCloudQueue();
bool cloudSendSamplePacket(const CloudSamplePacket &packet);
bool cloudSendBatchHistory();
const char *cloudUploadModeName();
void updateCloudModeLabel();
bool isBatchUploading();
void requestBatchUiRefresh();
void serviceLightweightBatchUi();
void resetDirectIslSessionCache(const char *reason);
bool ensureWifiReadyForHttp(const char *context, int waitMs);
bool wifiReadyForHttp();
void httpCloseConnection(const char *reason);
bool directIslConfigured();
bool loadWifiCredentials();
bool c6WifiConnectBackground(const char *ssid, const char *password);
static bool cloudModumConfigured();


void createCsvUi();
void createFileViewerUi();
void createIslUi();
void createBleUi();
void refreshBleScreen();
void applyTableValueHeaders();
void resetTable();
void updateTable();

static void go_csv_event_cb(lv_event_t *e);
static void go_home_event_cb(lv_event_t *e);
static void go_measure_event_cb(lv_event_t *e);
static void go_settings_event_cb(lv_event_t *e);
static void go_isl_event_cb(lv_event_t *e);


// =====================================================
// Forward declarations used by early home dashboard callbacks
// These are intentionally placed before refreshHomeSensorLabels().
// =====================================================
extern bool dpsReady;
extern bool measuring;
extern bool measurementBackupRestored;
extern int restoredSampleCount;
extern int activeSensorMode;

const char *activeSensorName();
const char *activeMeasurementTitle();
void setActiveSensorMode(int mode);
bool saveMeasurementBackupToNvs(bool force);
void updateChartAutoScale();
void clearChart();

void styleSensorTile(lv_obj_t *tile, lv_obj_t *mark, bool selected);
const char *sensorDetailText(int mode);

// Move the "선택됨" state onto the tile for `mode`.
void refreshHomeSensorTilesFor(int mode)
{
  for (int i = 0; i < HOME_SENSOR_TILE_COUNT; i++)
  {
    styleSensorTile(
      homeSensorTiles[i],
      homeSensorTileMarks[i],
      homeSensorTileModes[i] == mode
    );
  }
}

void refreshHomeSensorTiles()
{
  refreshHomeSensorTilesFor(activeSensorMode);
}

void refreshHomeSensorLabels()
{
  char buf[96];

  refreshHomeSensorTiles();

  if (labelHomeSensorMode)
  {
    snprintf(buf, sizeof(buf), "선택: %s", activeMeasurementTitle());
    lv_label_set_text(labelHomeSensorMode, buf);
  }

  if (labelHomeSensorStatus)
  {
    snprintf(buf, sizeof(buf), "상태: %s", dpsReady ? "준비" : "인식 실패");
    lv_label_set_text(labelHomeSensorStatus, buf);
  }

  if (labelHomeRecovery)
  {
    if (measurementBackupRestored) snprintf(buf, sizeof(buf), "복구: %d개", restoredSampleCount);
    else snprintf(buf, sizeof(buf), "복구: 없음");
    lv_label_set_text(labelHomeRecovery, buf);
  }

  if (labelHomeResetReason)
  {
    esp_reset_reason_t reason = esp_reset_reason();
    const char *name = "전원/리셋";
    if (reason == ESP_RST_POWERON) name = "전원 켜짐";
    else if (reason == ESP_RST_SW) name = "소프트 재시작";
    else if (reason == ESP_RST_PANIC) name = "패닉/크래시";
    else if (reason == ESP_RST_INT_WDT || reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT) name = "WDT 재시작";
    else if (reason == ESP_RST_BROWNOUT) name = "전압 저하";
    snprintf(buf, sizeof(buf), "최근 재시작: %s", name);
    lv_label_set_text(labelHomeResetReason, buf);
  }
}

static void home_sensor_dps_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (measuring) { setIslStatusText("측정 정지 후 센서 변경"); return; }
  pendingSensorMode = SENSOR_MODE_DPS310;
}

static void home_sensor_water_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (measuring) { setIslStatusText("측정 정지 후 센서 변경"); return; }
  pendingSensorMode = SENSOR_MODE_DS18B20;
}

static void home_sensor_co2_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (measuring) { setIslStatusText("측정 정지 후 센서 변경"); return; }
  pendingSensorMode = SENSOR_MODE_SCD41;
}

static void home_sensor_light_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (measuring) { setIslStatusText("측정 정지 후 센서 변경"); return; }
  pendingSensorMode = SENSOR_MODE_TSL2591;
}

static void home_sensor_tmp117_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (measuring) { setIslStatusText("측정 정지 후 센서 변경"); return; }
  pendingSensorMode = SENSOR_MODE_TMP117;
}

static void home_sensor_vl53_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (measuring) { setIslStatusText("측정 정지 후 센서 변경"); return; }
  pendingSensorMode = SENSOR_MODE_VL53L1X;
}

static void home_sensor_ina228_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (measuring) { setIslStatusText("측정 정지 후 센서 변경"); return; }
  pendingSensorMode = SENSOR_MODE_INA228;
}

static void home_sensor_encoder_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (measuring) { setIslStatusText("측정 정지 후 센서 변경"); return; }
  pendingSensorMode = SENSOR_MODE_ENCODER;
}

void servicePendingSensorMode()
{
  int mode = pendingSensorMode;
  if (mode == 0 || uiInputLocked) return;
  pendingSensorMode = 0;

  if (measuring)
  {
    setIslStatusText("측정 정지 후 센서 변경");
    return;
  }

  uiInputLocked = true;
  ignoreTouchUntilMs = millis() + 200;

  // Move the selection and paint it before switching sensors. Bringing a
  // sensor up probes I2C and can block for a second or more, and doing that
  // first made the tap feel unacknowledged.
  refreshHomeSensorTilesFor(mode);
  lv_refr_now(NULL);

  setActiveSensorMode(mode);
  uiInputLocked = false;
  ignoreTouchUntilMs = millis() + 200;
}

static void board_restart_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  saveMeasurementBackupToNvs(true);
  delay(100);
  esp_restart();
}

static void dashboard_isl_save_event_cb(lv_event_t *e);
static void dashboard_isl_start_event_cb(lv_event_t *e);
static void dashboard_isl_stop_event_cb(lv_event_t *e);
static void dashboard_cloud_realtime_event_cb(lv_event_t *e);
static void dashboard_cloud_batch_event_cb(lv_event_t *e);
static void dashboard_cloud_batch_upload_event_cb(lv_event_t *e);
static void dashboard_csv_mount_event_cb(lv_event_t *e);
static void dashboard_csv_toggle_event_cb(lv_event_t *e);
static void dashboard_csv_preview_event_cb(lv_event_t *e);
static void csv_save_measurement_data_event_cb(lv_event_t *e);
static void csv_viewer_prev_event_cb(lv_event_t *e);
static void csv_viewer_next_event_cb(lv_event_t *e);
static void sd_file_item_event_cb(lv_event_t *e);
static void home_sensor_dps_event_cb(lv_event_t *e);
static void home_sensor_water_event_cb(lv_event_t *e);
static void home_sensor_co2_event_cb(lv_event_t *e);
static void home_sensor_light_event_cb(lv_event_t *e);
static void home_sensor_tmp117_event_cb(lv_event_t *e);
static void home_sensor_vl53_event_cb(lv_event_t *e);
static void home_sensor_ina228_event_cb(lv_event_t *e);
static void home_sensor_encoder_event_cb(lv_event_t *e);
static void board_restart_event_cb(lv_event_t *e);
bool isTextViewFile(const char *name);
void sanitizeBasicFileName(const char *input, char *out, size_t outSize);
void setSelectedSdTextFile(const char *fileName);
void openSdTextFileByName(const char *fileName);
void loadSdTextFilePage(const char *fileName, long offset);
static void dashboard_textarea_event_cb(lv_event_t *e);
static void dashboard_keyboard_event_cb(lv_event_t *e);
static void copy_label_event_cb(lv_event_t *e);
static void csv_textarea_event_cb(lv_event_t *e);
static void csv_keyboard_event_cb(lv_event_t *e);
void updateDashboardCsvPreview();
void updateDashboardCsvLabels();
void updateDashboardIslLabels();

void refreshHomeSensorLabels();
bool activeSensorBegin();
bool readActiveSensor(float *primaryValue, float *secondaryValue);
const char *activeSensorName();
const char *activePrimaryName();
const char *activePrimaryUnit();
const char *activeSecondaryName();
const char *activeSecondaryUnit();
bool activeSensorHasPressure();
bool pressureValueValid(float value);
bool activeSensorSupportsDirectIsl();
void updateActiveSensorUiLabels();
bool saveMeasurementBackupToNvs(bool force);
bool restoreMeasurementBackupFromNvs();
void clearMeasurementBackupFromNvs();
void applyRestoredMeasurementToUi();

lv_obj_t *makeButton(lv_obj_t *parent, const char *text, int x, int y, int w, int h, lv_event_cb_t cb);
lv_obj_t *makeTableNavButton(lv_obj_t *parent, const char *text, int x, int y, int w, int h, lv_event_cb_t cb);
void styleSensorTile(lv_obj_t *tile, lv_obj_t *mark, bool selected);

#if HAS_IDF_WIFI
static bool idfWifiStarted = false;
static bool espHostedStarted = false;
static esp_netif_t *wifiStaNetif = NULL;
#endif


unsigned long measureAccumulatedMs = 0;
unsigned long measureResumeMs = 0;
bool measureClockRunning = false;

// =====================================================
// DPS310 보정 계수
// =====================================================
int16_t c0, c1, c01, c11, c20, c21, c30;
int32_t c00, c10;

float tempScale = 524288.0f;
float pressureScale = 524288.0f;

int32_t lastRawTemp = 0;
int32_t lastRawPressure = 0;

bool dpsReady = false;
bool measuring = false;
bool sdReady = false;

unsigned long lastReadMs = 0;
unsigned long lastUiClockMs = 0;
unsigned long startMs = 0;
const unsigned long readIntervalMs = 1000;

int measurementCount = 0;

// =====================================================
// 그래프 누적 데이터
// =====================================================
#define MAX_SAMPLES 720   // 안정화: 1초 간격 기준 최대 12분 저장
#define CHART_POINTS 220   // 고정 크기: 측정 중 재할당 금지
#define TABLE_VISIBLE_ROWS 20
#define CHART_UI_REFRESH_MS 1200UL

int tablePageOffset = 0;

float tempHistory[MAX_SAMPLES];
float pressureHistory[MAX_SAMPLES];
float humidityHistory[MAX_SAMPLES];
uint32_t timeHistory[MAX_SAMPLES];
int noHistory[MAX_SAMPLES];
bool sampleEnabled[MAX_SAMPLES];

int sampleCount = 0;

// 그래프는 고정 220포인트 버퍼를 유지하고, 전체 측정 구간을 재표본화한다.
// 측정마다 point_count를 바꾸면 LVGL 내부 버퍼 재할당/메모리 단편화가 생길 수 있다.
static volatile bool chartUiDirty = true;
static unsigned long lastChartUiRefreshMs = 0;

int activeSensorMode = ACTIVE_SENSOR_DEFAULT;
bool measurementBackupRestored = false;
int restoredSampleCount = 0;
int lastBackupMeasurementCount = -1;
unsigned long lastBackupWriteMs = 0;

struct MeasurementBackupBlob
{
  uint32_t magic;
  uint32_t version;
  int storedSampleCount;
  int storedMeasurementCount;
  unsigned long storedElapsedMs;
  int storedActiveSensorMode;
  float temp[MAX_SAMPLES];
  float pressure[MAX_SAMPLES];
  float humidity[MAX_SAMPLES];
  uint32_t timeS[MAX_SAMPLES];
  int no[MAX_SAMPLES];
  uint8_t enabled[MAX_SAMPLES];
};

// 15 kB of scratch for the NVS snapshot. Kept in PSRAM rather than internal
// RAM: it is touched twice per backup at 1 Hz, so the slower bus costs
// nothing, and internal RAM is the pool a TLS handshake and the Bluetooth
// stack compete for.
static MeasurementBackupBlob *measurementBackupScratch = NULL;

static MeasurementBackupBlob *measurementBackupBuffer()
{
  if (measurementBackupScratch != NULL) return measurementBackupScratch;

  measurementBackupScratch = (MeasurementBackupBlob *)heap_caps_malloc(
    sizeof(MeasurementBackupBlob), MALLOC_CAP_SPIRAM);

  if (measurementBackupScratch == NULL)
  {
    // No PSRAM: fall back so a backup still works, just from the smaller pool.
    measurementBackupScratch = (MeasurementBackupBlob *)malloc(sizeof(MeasurementBackupBlob));
  }

  return measurementBackupScratch;
}


// =====================================================
// LVGL display flush
// =====================================================
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  lcd.lcd_draw_bitmap(
    area->x1,
    area->y1,
    area->x2 + 1,
    area->y2 + 1,
    &color_p->full
  );

  lv_disp_flush_ready(disp);
}

// LVGL is not re-entrant. Several blocking waits (WiFi connect, NTP sync)
// pump the UI so the screen keeps updating, and those waits can themselves be
// reached from an LVGL callback. Route every call through this guard so a
// nested pump becomes a no-op instead of corrupting LVGL's internal state.
static bool lvglHandlerBusy = false;

static void uiTimerHandler()
{
  if (lvglHandlerBusy) return;

  lvglHandlerBusy = true;
  lv_timer_handler();
  lvglHandlerBusy = false;
}

// =====================================================
// Touch
// =====================================================
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  (void)indev_driver;
  bool rawTouched = false;
  uint16_t rawX = 0;
  uint16_t rawY = 0;
  const unsigned long now = millis();

#if ENABLE_GT911_TOUCH
  rawTouched = touch.getTouch(&rawX, &rawY);
#endif

  // GT911 can briefly report RELEASE while a finger is still down.
  // Debounce both edges so one physical keyboard tap becomes one LVGL click.
  static bool rawCandidate = false;
  static bool debouncedDown = false;
  static unsigned long rawCandidateSinceMs = 0;
  static unsigned long lastDebouncedReleaseMs = 0;
  static uint16_t lastGoodX = 0;
  static uint16_t lastGoodY = 0;

  if (rawTouched)
  {
    lastGoodX = rawX;
    lastGoodY = rawY;
  }

  if (rawTouched != rawCandidate)
  {
    rawCandidate = rawTouched;
    rawCandidateSinceMs = now;
  }

  if (rawCandidate != debouncedDown)
  {
    const unsigned long stableMs = now - rawCandidateSinceMs;
    const unsigned long requiredMs = rawCandidate ? TOUCH_PRESS_DEBOUNCE_MS : TOUCH_RELEASE_DEBOUNCE_MS;

    if (stableMs >= requiredMs)
    {
      if (!rawCandidate || (now - lastDebouncedReleaseMs >= TOUCH_REARM_MS))
      {
        debouncedDown = rawCandidate;
        touchLastChangeMs = now;
        if (!debouncedDown) lastDebouncedReleaseMs = now;
      }
    }
  }

  touchRawDown = debouncedDown;

  const bool lockoutActive = ((int32_t)(ignoreTouchUntilMs - now) > 0);
  if (uiInputLocked || lockoutActive || !debouncedDown)
  {
    data->state = LV_INDEV_STATE_REL;
    return;
  }

  data->state = LV_INDEV_STATE_PR;
  data->point.x = lastGoodX;
  data->point.y = lastGoodY;
}

static void lvgl_port_update_callback(lv_disp_drv_t *drv)
{
  switch (drv->rotated)
  {
    case LV_DISP_ROT_NONE:
      touch.set_rotation(0);
      break;

    case LV_DISP_ROT_90:
      touch.set_rotation(1);
      break;

    case LV_DISP_ROT_180:
      touch.set_rotation(2);
      break;

    case LV_DISP_ROT_270:
      touch.set_rotation(3);
      break;
  }
}

// =====================================================
// Soft I2C
// =====================================================
void i2cDelay()
{
  delayMicroseconds(I2C_DELAY_US);
}

void sdaHigh()
{
  pinMode(SOFT_SDA, INPUT_PULLUP);
}

void sdaLow()
{
  digitalWrite(SOFT_SDA, LOW);
  pinMode(SOFT_SDA, OUTPUT);
}

void sclHigh()
{
  pinMode(SOFT_SCL, INPUT_PULLUP);
}

void sclLow()
{
  digitalWrite(SOFT_SCL, LOW);
  pinMode(SOFT_SCL, OUTPUT);
}

bool waitSclHigh()
{
  sclHigh();
  unsigned long start = micros();

  while (digitalRead(SOFT_SCL) == LOW)
  {
    if (micros() - start > 2000)
    {
      return false;
    }
  }

  return true;
}

void softI2cStart()
{
  sdaHigh();
  sclHigh();
  i2cDelay();

  sdaLow();
  i2cDelay();

  sclLow();
  i2cDelay();
}

void softI2cStop()
{
  sdaLow();
  i2cDelay();

  sclHigh();
  i2cDelay();

  sdaHigh();
  i2cDelay();
}

bool softI2cWriteByte(uint8_t value)
{
  for (int i = 7; i >= 0; i--)
  {
    if (value & (1 << i))
    {
      sdaHigh();
    }
    else
    {
      sdaLow();
    }

    i2cDelay();

    if (!waitSclHigh())
    {
      sclLow();
      return false;
    }

    i2cDelay();
    sclLow();
    i2cDelay();
  }

  sdaHigh();
  i2cDelay();

  if (!waitSclHigh())
  {
    sclLow();
    return false;
  }

  bool ack = digitalRead(SOFT_SDA) == LOW;

  i2cDelay();
  sclLow();
  i2cDelay();

  return ack;
}

uint8_t softI2cReadByte(bool sendAck)
{
  uint8_t value = 0;

  sdaHigh();

  for (int i = 7; i >= 0; i--)
  {
    if (!waitSclHigh())
    {
      sclLow();
      return 0xFF;
    }

    i2cDelay();

    if (digitalRead(SOFT_SDA))
    {
      value |= (1 << i);
    }

    sclLow();
    i2cDelay();
  }

  if (sendAck)
  {
    sdaLow();
  }
  else
  {
    sdaHigh();
  }

  i2cDelay();
  waitSclHigh();
  i2cDelay();

  sclLow();
  sdaHigh();
  i2cDelay();

  return value;
}

// =====================================================
// Generic Soft-I2C register helpers
// =====================================================
bool softI2cWriteRegister(uint8_t address, uint8_t reg, uint8_t value)
{
  softI2cStart();

  if (!softI2cWriteByte((address << 1) | 0))
  {
    softI2cStop();
    return false;
  }

  if (!softI2cWriteByte(reg))
  {
    softI2cStop();
    return false;
  }

  if (!softI2cWriteByte(value))
  {
    softI2cStop();
    return false;
  }

  softI2cStop();
  return true;
}

bool softI2cReadRegisters(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t length)
{
  if (buffer == NULL || length == 0) return false;

  softI2cStart();

  if (!softI2cWriteByte((address << 1) | 0))
  {
    softI2cStop();
    return false;
  }

  if (!softI2cWriteByte(reg))
  {
    softI2cStop();
    return false;
  }

  softI2cStart();

  if (!softI2cWriteByte((address << 1) | 1))
  {
    softI2cStop();
    return false;
  }

  for (uint8_t i = 0; i < length; i++)
  {
    buffer[i] = softI2cReadByte(i < length - 1);
  }

  softI2cStop();
  return true;
}

// =====================================================
// Generic Soft-I2C helpers for sensors with 16-bit register addresses
// =====================================================
static bool softI2cWriteRegister16Addr(
  uint8_t address,
  uint16_t reg,
  uint8_t value)
{
  softI2cStart();

  if (!softI2cWriteByte((address << 1) | 0))
  {
    softI2cStop();
    return false;
  }

  if (!softI2cWriteByte((uint8_t)(reg >> 8)) ||
      !softI2cWriteByte((uint8_t)(reg & 0xFF)) ||
      !softI2cWriteByte(value))
  {
    softI2cStop();
    return false;
  }

  softI2cStop();
  return true;
}

static bool softI2cReadRegisters16Addr(
  uint8_t address,
  uint16_t reg,
  uint8_t *buffer,
  uint8_t length)
{
  if (buffer == NULL || length == 0) return false;

  softI2cStart();

  if (!softI2cWriteByte((address << 1) | 0))
  {
    softI2cStop();
    return false;
  }

  if (!softI2cWriteByte((uint8_t)(reg >> 8)) ||
      !softI2cWriteByte((uint8_t)(reg & 0xFF)))
  {
    softI2cStop();
    return false;
  }

  softI2cStart();

  if (!softI2cWriteByte((address << 1) | 1))
  {
    softI2cStop();
    return false;
  }

  for (uint8_t i = 0; i < length; i++)
  {
    buffer[i] = softI2cReadByte(i < (uint8_t)(length - 1));
  }

  softI2cStop();
  return true;
}

static bool softI2cRead16BE(
  uint8_t address,
  uint16_t reg,
  uint16_t *value)
{
  if (value == NULL) return false;

  uint8_t data[2] = {0, 0};
  if (!softI2cReadRegisters16Addr(address, reg, data, 2))
  {
    return false;
  }

  *value = ((uint16_t)data[0] << 8) | data[1];
  return true;
}

// =====================================================
// DPS310 register
// =====================================================
bool dpsWriteRegister(uint8_t reg, uint8_t value)
{
  softI2cStart();

  if (!softI2cWriteByte((DPS310_ADDR << 1) | 0))
  {
    softI2cStop();
    return false;
  }

  if (!softI2cWriteByte(reg))
  {
    softI2cStop();
    return false;
  }

  if (!softI2cWriteByte(value))
  {
    softI2cStop();
    return false;
  }

  softI2cStop();
  return true;
}

bool dpsReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t length)
{
  softI2cStart();

  if (!softI2cWriteByte((DPS310_ADDR << 1) | 0))
  {
    softI2cStop();
    return false;
  }

  if (!softI2cWriteByte(reg))
  {
    softI2cStop();
    return false;
  }

  softI2cStart();

  if (!softI2cWriteByte((DPS310_ADDR << 1) | 1))
  {
    softI2cStop();
    return false;
  }

  for (uint8_t i = 0; i < length; i++)
  {
    buffer[i] = softI2cReadByte(i < length - 1);
  }

  softI2cStop();
  return true;
}

bool dpsReadRegister(uint8_t reg, uint8_t *value)
{
  return dpsReadRegisters(reg, value, 1);
}

int32_t signExtend(uint32_t value, uint8_t bits)
{
  uint32_t mask = 1UL << (bits - 1);
  value &= ((1UL << bits) - 1);
  return (value ^ mask) - mask;
}

bool readCoefficients()
{
  uint8_t data[18];

  if (!dpsReadRegisters(REG_COEF, data, 18))
  {
    return false;
  }

  c0  = signExtend(((uint16_t)data[0] << 4) | (data[1] >> 4), 12);
  c1  = signExtend(((uint16_t)(data[1] & 0x0F) << 8) | data[2], 12);
  c00 = signExtend(((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] >> 4), 20);
  c10 = signExtend(((uint32_t)(data[5] & 0x0F) << 16) | ((uint32_t)data[6] << 8) | data[7], 20);
  c01 = signExtend(((uint16_t)data[8] << 8) | data[9], 16);
  c11 = signExtend(((uint16_t)data[10] << 8) | data[11], 16);
  c20 = signExtend(((uint16_t)data[12] << 8) | data[13], 16);
  c21 = signExtend(((uint16_t)data[14] << 8) | data[15], 16);
  c30 = signExtend(((uint16_t)data[16] << 8) | data[17], 16);

  return true;
}

bool dps310Begin()
{
  uint8_t productId = 0;

  if (!dpsReadRegister(REG_PROD_ID, &productId))
  {
    Serial.println("DPS310 product ID read failed");
    return false;
  }

  Serial.print("DPS310 product ID: 0x");
  Serial.println(productId, HEX);

  dpsWriteRegister(REG_RESET, 0x89);
  delay(100);

  uint8_t status = 0;
  unsigned long start = millis();

  while (millis() - start < 250)
  {
    if (dpsReadRegister(REG_MEAS_CFG, &status))
    {
      if ((status & 0xC0) == 0xC0)
      {
        break;
      }
    }

    uiTimerHandler();
    delay(10);
  }

  if ((status & 0xC0) != 0xC0)
  {
    Serial.println("DPS310 ready timeout");
    return false;
  }

  if (!readCoefficients())
  {
    Serial.println("DPS310 coefficient read failed");
    return false;
  }

  uint8_t tempCoefSource = 0;
  dpsReadRegister(REG_TMP_COEF, &tempCoefSource);

  uint8_t tempSourceBit = (tempCoefSource & 0x80) ? 0x80 : 0x00;

  dpsWriteRegister(REG_PRS_CFG, 0x00);
  dpsWriteRegister(REG_TMP_CFG, tempSourceBit | 0x00);
  dpsWriteRegister(REG_CFG_REG, 0x00);
  dpsWriteRegister(REG_MEAS_CFG, 0x07);

  delay(100);

  Serial.println("DPS310 init OK");
  return true;
}

bool readDps310(float *temperatureC, float *pressureHpa)
{
  uint8_t status = 0;
  unsigned long start = millis();

  while (millis() - start < 250)
  {
    if (dpsReadRegister(REG_MEAS_CFG, &status))
    {
      if ((status & 0x30) == 0x30)
      {
        break;
      }
    }

    uiTimerHandler();
    delay(5);
  }

  if ((status & 0x30) != 0x30)
  {
    return false;
  }

  uint8_t pData[3];
  uint8_t tData[3];

  if (!dpsReadRegisters(REG_PRS_B2, pData, 3))
  {
    return false;
  }

  if (!dpsReadRegisters(REG_TMP_B2, tData, 3))
  {
    return false;
  }

  int32_t rawPressure = signExtend(
    ((uint32_t)pData[0] << 16) | ((uint32_t)pData[1] << 8) | pData[2],
    24
  );

  int32_t rawTemp = signExtend(
    ((uint32_t)tData[0] << 16) | ((uint32_t)tData[1] << 8) | tData[2],
    24
  );

  lastRawTemp = rawTemp;
  lastRawPressure = rawPressure;

  float tRawSc = rawTemp / tempScale;
  float pRawSc = rawPressure / pressureScale;

  float tempComp = (c0 * 0.5f) + (c1 * tRawSc);

  float pressurePa =
    c00 +
    pRawSc * (c10 + pRawSc * (c20 + pRawSc * c30)) +
    tRawSc * c01 +
    tRawSc * pRawSc * (c11 + pRawSc * c21);

  *temperatureC = tempComp;
  *pressureHpa = pressurePa / 100.0f;

  return true;
}

// =====================================================
// SCD41 CO2 + temperature + relative humidity
// Soft-I2C, Sensirion SCD4x command protocol
// =====================================================
static float scd41LastTemperatureC = NAN;
static float scd41LastHumidityPct = NAN;
static bool scd41PeriodicStarted = false;
static bool scd41LastReadWasWaiting = false;
static unsigned long scd41NextPollMs = 0;
static uint8_t scd41ConsecutiveErrors = 0;

static uint8_t scd4xCrc8(const uint8_t *data, size_t length)
{
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < length; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x31);
      else crc = (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static void softI2cRecoverBus()
{
  sdaHigh();
  sclHigh();
  i2cDelay();
  for (int i = 0; i < 9; i++)
  {
    sclLow();
    i2cDelay();
    sclHigh();
    i2cDelay();
  }
  softI2cStop();
}

static bool softI2cWriteCommand16(uint8_t address, uint16_t command)
{
  softI2cStart();
  if (!softI2cWriteByte((address << 1) | 0))
  {
    softI2cStop();
    return false;
  }
  if (!softI2cWriteByte((uint8_t)(command >> 8)))
  {
    softI2cStop();
    return false;
  }
  if (!softI2cWriteByte((uint8_t)(command & 0xFF)))
  {
    softI2cStop();
    return false;
  }
  softI2cStop();
  return true;
}

// SCD4x wake_up is special: the sensor may deliberately not ACK the command.
// Clock the complete command anyway and allow the specified wake time.
static void softI2cWriteCommand16BestEffort(uint8_t address, uint16_t command)
{
  softI2cStart();
  (void)softI2cWriteByte((address << 1) | 0);
  (void)softI2cWriteByte((uint8_t)(command >> 8));
  (void)softI2cWriteByte((uint8_t)(command & 0xFF));
  softI2cStop();
}

static bool scd41ReadCommandWords(uint16_t command, uint16_t *words, uint8_t wordCount, unsigned long waitMs)
{
  if (words == NULL || wordCount == 0) return false;
  if (!softI2cWriteCommand16(SCD41_ADDR, command)) return false;
  if (waitMs > 0) delay(waitMs);

  softI2cStart();
  if (!softI2cWriteByte((SCD41_ADDR << 1) | 1))
  {
    softI2cStop();
    return false;
  }

  for (uint8_t i = 0; i < wordCount; i++)
  {
    const uint8_t msb = softI2cReadByte(true);
    const uint8_t lsb = softI2cReadByte(true);
    const uint8_t crc = softI2cReadByte(i < (wordCount - 1));
    const uint8_t raw[2] = {msb, lsb};
    if (scd4xCrc8(raw, 2) != crc)
    {
      softI2cStop();
      Serial.println("SCD41 CRC error");
      return false;
    }
    words[i] = ((uint16_t)msb << 8) | lsb;
  }

  softI2cStop();
  return true;
}

static bool scd41StopPeriodicMeasurement()
{
  const bool ok = softI2cWriteCommand16(SCD41_ADDR, SCD41_CMD_STOP_PERIODIC_MEASUREMENT);
  if (ok) delay(SCD41_STOP_DELAY_MS);
  scd41PeriodicStarted = false;
  return ok;
}

bool scd41Begin()
{
  // Recovery build: stay on the original GPIO2/3 bit-banged I2C path.
  // No Wire/Wire1/Sensirion/Adafruit sensor libraries are linked.
  sdaHigh();
  sclHigh();
  delay(30);
  softI2cRecoverBus();

  Serial.print("[SCD41] bus idle SDA=");
  Serial.print(digitalRead(SOFT_SDA));
  Serial.print(" SCL=");
  Serial.println(digitalRead(SOFT_SCL));

  // Sensirion clean-start sequence:
  // wake_up -> stop_periodic_measurement -> reinit -> serial -> start.
  // wake_up may not ACK, so it is sent best-effort.
  softI2cWriteCommand16BestEffort(SCD41_ADDR, SCD41_CMD_WAKE_UP);
  delay(SCD41_WAKE_DELAY_MS);

  bool stopOk =
    softI2cWriteCommand16(SCD41_ADDR, SCD41_CMD_STOP_PERIODIC_MEASUREMENT);
  if (!stopOk)
  {
    Serial.println("[SCD41] stop command not ACKed; continue recovery");
    softI2cRecoverBus();
  }
  // Datasheet: other commands are accepted only 500 ms after stop.
  delay(SCD41_STOP_DELAY_MS);
  scd41PeriodicStarted = false;

  if (softI2cWriteCommand16(SCD41_ADDR, SCD41_CMD_REINIT))
  {
    delay(SCD41_REINIT_DELAY_MS);
  }
  else
  {
    Serial.println("[SCD41] reinit not ACKed; serial probe will decide");
    softI2cRecoverBus();
    delay(SCD41_REINIT_DELAY_MS);
  }

  uint16_t serialWords[3] = {0, 0, 0};
  bool serialOk = false;
  for (int attempt = 0; attempt < 3 && !serialOk; attempt++)
  {
    serialOk = scd41ReadCommandWords(
      SCD41_CMD_GET_SERIAL_NUMBER,
      serialWords,
      3,
      SCD41_COMMAND_DELAY_MS
    );

    if (!serialOk)
    {
      Serial.print("[SCD41] serial retry ");
      Serial.println(attempt + 1);
      softI2cRecoverBus();
      delay(40);
    }
  }

  if (!serialOk)
  {
    Serial.print("[SCD41] no response. SDA=");
    Serial.print(digitalRead(SOFT_SDA));
    Serial.print(" SCL=");
    Serial.println(digitalRead(SOFT_SCL));
    scd41PeriodicStarted = false;
    return false;
  }

  Serial.print("[SCD41] serial: 0x");
  Serial.print(serialWords[0], HEX);
  Serial.print(serialWords[1], HEX);
  Serial.println(serialWords[2], HEX);

  if (!softI2cWriteCommand16(
        SCD41_ADDR,
        SCD41_CMD_START_PERIODIC_MEASUREMENT))
  {
    Serial.println("[SCD41] start periodic measurement failed");
    scd41PeriodicStarted = false;
    return false;
  }

  scd41PeriodicStarted = true;
  scd41LastTemperatureC = NAN;
  scd41LastHumidityPct = NAN;
  scd41LastReadWasWaiting = true;
  scd41ConsecutiveErrors = 0;
  scd41NextPollMs = millis() + 5000UL;
  Serial.println("[SCD41] periodic mode started; first sample about 5 s");
  return true;
}

bool readScd41(float *co2Ppm, float *temperatureC, float *humidityPct)
{
  if (co2Ppm == NULL || temperatureC == NULL || humidityPct == NULL || !scd41PeriodicStarted)
  {
    scd41LastReadWasWaiting = false;
    return false;
  }

  const unsigned long now = millis();
  if ((int32_t)(now - scd41NextPollMs) < 0)
  {
    scd41LastReadWasWaiting = true;
    return false;
  }

  uint16_t readyWord = 0;
  if (!scd41ReadCommandWords(SCD41_CMD_GET_DATA_READY_STATUS, &readyWord, 1, SCD41_COMMAND_DELAY_MS))
  {
    scd41LastReadWasWaiting = false;
    scd41ConsecutiveErrors++;
    scd41NextPollMs = now + 500UL;
    return false;
  }

  if ((readyWord & 0x07FF) == 0)
  {
    scd41LastReadWasWaiting = true;
    scd41NextPollMs = now + 400UL;
    return false;
  }

  uint16_t words[3] = {0, 0, 0};
  if (!scd41ReadCommandWords(SCD41_CMD_READ_MEASUREMENT, words, 3, SCD41_COMMAND_DELAY_MS))
  {
    scd41LastReadWasWaiting = false;
    scd41ConsecutiveErrors++;
    scd41NextPollMs = now + 500UL;
    return false;
  }

  const uint16_t rawCo2 = words[0];
  if (rawCo2 == 0)
  {
    scd41LastReadWasWaiting = false;
    scd41ConsecutiveErrors++;
    scd41NextPollMs = now + 500UL;
    return false;
  }

  scd41LastTemperatureC = -45.0f + 175.0f * ((float)words[1] / 65535.0f);
  scd41LastHumidityPct = 100.0f * ((float)words[2] / 65535.0f);

  *co2Ppm = (float)rawCo2;
  *temperatureC = scd41LastTemperatureC;
  *humidityPct = scd41LastHumidityPct;

  lastRawTemp = (int32_t)rawCo2;
  lastRawPressure = (int32_t)lroundf(scd41LastHumidityPct * 100.0f);

  scd41LastReadWasWaiting = false;
  scd41ConsecutiveErrors = 0;
  scd41NextPollMs = now + 4500UL;
  return true;
}

// =====================================================
// TSL2591 digital light sensor (Soft-I2C)
// =====================================================
bool tsl2591Write8(uint8_t reg, uint8_t value)
{
  return softI2cWriteRegister(
    TSL2591_ADDR,
    TSL2591_COMMAND_BIT | reg,
    value
  );
}

bool tsl2591Read8(uint8_t reg, uint8_t *value)
{
  return softI2cReadRegisters(
    TSL2591_ADDR,
    TSL2591_COMMAND_BIT | reg,
    value,
    1
  );
}

bool tsl2591Read16(uint8_t reg, uint16_t *value)
{
  if (value == NULL) return false;

  uint8_t data[2];
  if (!softI2cReadRegisters(
    TSL2591_ADDR,
    TSL2591_COMMAND_BIT | reg,
    data,
    2
  ))
  {
    return false;
  }

  *value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
  return true;
}

bool tsl2591Begin()
{
  sdaHigh();
  sclHigh();
  delay(10);

  uint8_t id = 0;
  if (!tsl2591Read8(TSL2591_REG_ID, &id))
  {
    Serial.println("TSL2591 ID read failed");
    return false;
  }

  Serial.print("TSL2591 ID: 0x");
  Serial.println(id, HEX);

  if ((id & 0xF0) != 0x50)
  {
    Serial.println("TSL2591 unexpected ID");
    return false;
  }

  if (!tsl2591Write8(
    TSL2591_REG_CONTROL,
    TSL2591_GAIN_MEDIUM | TSL2591_INTEGRATION_200MS
  ))
  {
    return false;
  }

  if (!tsl2591Write8(
    TSL2591_REG_ENABLE,
    TSL2591_ENABLE_PON | TSL2591_ENABLE_AEN
  ))
  {
    return false;
  }

  delay(230);
  Serial.println("TSL2591 init OK");
  return true;
}

bool readTsl2591(float *luxValue)
{
  if (luxValue == NULL) return false;

  uint8_t status = 0;
  if (!tsl2591Read8(TSL2591_REG_STATUS, &status)) return false;

  uint16_t channel0 = 0;
  uint16_t channel1 = 0;

  if (!tsl2591Read16(TSL2591_REG_CHAN0_LOW, &channel0)) return false;
  if (!tsl2591Read16(TSL2591_REG_CHAN1_LOW, &channel1)) return false;

  lastRawTemp = channel0;
  lastRawPressure = channel1;

  if (channel0 == 0xFFFF || channel1 == 0xFFFF)
  {
    Serial.println("TSL2591 saturated");
    return false;
  }

  float cpl = (TSL2591_ATIME_MS * TSL2591_AGAIN) / TSL2591_LUX_DF;
  if (cpl <= 0.0f) return false;

  float lux1 = ((float)channel0 - TSL2591_LUX_COEFB * (float)channel1) / cpl;
  float lux2 = (TSL2591_LUX_COEFC * (float)channel0 - TSL2591_LUX_COEFD * (float)channel1) / cpl;
  float lux = lux1 > lux2 ? lux1 : lux2;

  if (lux < 0.0f) lux = 0.0f;
  if (isnan(lux) || isinf(lux)) return false;

  *luxValue = lux;
  return true;
}

// =====================================================
// VL53L1X raw Soft-I2C / compact ST-ULD-style driver
// =====================================================
static bool vl53Write8(uint16_t reg, uint8_t value)
{
  return softI2cWriteRegister16Addr(VL53L1X_ADDR, reg, value);
}

static bool vl53Read8(uint16_t reg, uint8_t *value)
{
  if (value == NULL) return false;
  return softI2cReadRegisters16Addr(VL53L1X_ADDR, reg, value, 1);
}

static bool vl53Read16(uint16_t reg, uint16_t *value)
{
  return softI2cRead16BE(VL53L1X_ADDR, reg, value);
}

static bool vl53WaitForBoot(uint32_t timeoutMs)
{
  unsigned long start = millis();

  while (millis() - start < timeoutMs)
  {
    uint8_t bootState = 0;
    if (vl53Read8(VL53L1X_REG_FIRMWARE_SYSTEM_STATUS, &bootState))
    {
      if (bootState & 0x01)
      {
        return true;
      }
    }

    delay(5);
  }

  return false;
}

static bool vl53DataReady(bool *ready)
{
  if (ready == NULL) return false;

  uint8_t muxCtrl = 0;
  uint8_t gpioStatus = 0;

  if (!vl53Read8(VL53L1X_REG_GPIO_HV_MUX_CTRL, &muxCtrl) ||
      !vl53Read8(VL53L1X_REG_GPIO_TIO_HV_STATUS, &gpioStatus))
  {
    return false;
  }

  // ULD polarity rule:
  // mux bit4 = 0 -> active high -> ready bit expected 1
  // mux bit4 = 1 -> active low  -> ready bit expected 0
  uint8_t expected = (muxCtrl & 0x10) ? 0 : 1;
  *ready = ((gpioStatus & 0x01) == expected);
  return true;
}

static bool vl53WaitDataReady(uint32_t timeoutMs)
{
  unsigned long start = millis();

  while (millis() - start < timeoutMs)
  {
    bool ready = false;
    if (!vl53DataReady(&ready)) return false;
    if (ready) return true;
    delay(5);
  }

  return false;
}

static bool vl53ClearInterrupt()
{
  return vl53Write8(VL53L1X_REG_INTERRUPT_CLEAR, 0x01);
}

static bool vl53StartRanging()
{
  if (!vl53Write8(VL53L1X_REG_MODE_START, 0x40))
  {
    vl53RangingStarted = false;
    return false;
  }

  vl53RangingStarted = true;
  vl53LastReadWasWaiting = true;
  return true;
}

static void vl53StopRanging()
{
  if (vl53RangingStarted)
  {
    (void)vl53Write8(VL53L1X_REG_MODE_START, 0x00);
  }

  vl53RangingStarted = false;
  vl53LastReadWasWaiting = false;
}

static bool vl53l1xBegin()
{
  sdaHigh();
  sclHigh();
  delay(10);
  softI2cRecoverBus();

  vl53RangingStarted = false;
  vl53LastReadWasWaiting = false;
  vl53LastRawRangeStatus = 0xFF;

  Serial.println("[VL53L1X] raw Soft-I2C init start");

  // Clean software reset.
  if (!vl53Write8(VL53L1X_REG_SOFT_RESET, 0x00))
  {
    Serial.println("[VL53L1X] reset LOW failed / no ACK at 0x29");
    return false;
  }
  delay(100);

  if (!vl53Write8(VL53L1X_REG_SOFT_RESET, 0x01))
  {
    Serial.println("[VL53L1X] reset release failed");
    return false;
  }

  if (!vl53WaitForBoot(1000))
  {
    Serial.println("[VL53L1X] boot timeout");
    return false;
  }

  uint16_t modelId = 0;
  if (!vl53Read16(VL53L1X_REG_MODEL_ID, &modelId))
  {
    Serial.println("[VL53L1X] model ID read failed");
    return false;
  }

  Serial.print("[VL53L1X] model ID=0x");
  Serial.println(modelId, HEX);

  if (modelId != VL53L1X_EXPECTED_MODEL_ID)
  {
    Serial.println("[VL53L1X] unexpected model ID; expected 0xEACC");
    return false;
  }

  // Write the compact ST ULD default configuration.
  const size_t cfgLen = sizeof(VL53L1X_DEFAULT_CONFIGURATION);
  if (cfgLen != 91)
  {
    Serial.println("[VL53L1X] internal config length error");
    return false;
  }

  for (size_t i = 0; i < cfgLen; i++)
  {
    uint16_t reg = (uint16_t)(0x002D + i);
    if (!vl53Write8(reg, VL53L1X_DEFAULT_CONFIGURATION[i]))
    {
      Serial.print("[VL53L1X] config write failed at 0x");
      Serial.println(reg, HEX);
      return false;
    }
  }

  // SensorInit calibration cycle used by the ULD:
  // start -> first data -> clear -> stop -> VHV follow-up.
  if (!vl53StartRanging())
  {
    Serial.println("[VL53L1X] initial start ranging failed");
    return false;
  }

  if (!vl53WaitDataReady(1000))
  {
    Serial.println("[VL53L1X] initial data-ready timeout");
    vl53StopRanging();
    return false;
  }

  if (!vl53ClearInterrupt())
  {
    Serial.println("[VL53L1X] initial clear interrupt failed");
    vl53StopRanging();
    return false;
  }

  vl53StopRanging();

  if (!vl53Write8(VL53L1X_REG_VHV_TIMEOUT, 0x09) ||
      !vl53Write8(VL53L1X_REG_VHV_INIT, 0x00))
  {
    Serial.println("[VL53L1X] VHV post-init write failed");
    return false;
  }

  if (!vl53StartRanging())
  {
    Serial.println("[VL53L1X] continuous ranging start failed");
    return false;
  }

  Serial.println("[VL53L1X] init OK / continuous ranging");
  return true;
}

static bool readVl53l1x(float *distanceMm)
{
  if (distanceMm == NULL || !vl53RangingStarted) return false;

  bool ready = false;
  if (!vl53DataReady(&ready))
  {
    vl53LastReadWasWaiting = false;
    return false;
  }

  if (!ready)
  {
    vl53LastReadWasWaiting = true;
    return false;
  }

  vl53LastReadWasWaiting = false;

  uint8_t rangeStatus = 0;
  uint16_t rangeMm = 0;

  if (!vl53Read8(VL53L1X_REG_RANGE_STATUS, &rangeStatus) ||
      !vl53Read16(VL53L1X_REG_RANGE_MM, &rangeMm))
  {
    (void)vl53ClearInterrupt();
    return false;
  }

  vl53LastRawRangeStatus = (uint8_t)(rangeStatus & 0x1F);

  // Always release the current measurement so the next one can run.
  if (!vl53ClearInterrupt())
  {
    return false;
  }

  // Raw status 9 is the normal valid-ranging state in the ST result block.
  // Status 8 is a min-range-clipped result; keep it usable for short classroom
  // demonstrations but log it. Other statuses are treated as invalid samples.
  if (vl53LastRawRangeStatus != 9 && vl53LastRawRangeStatus != 8)
  {
    Serial.print("[VL53L1X] range status not valid: ");
    Serial.println(vl53LastRawRangeStatus);
    return false;
  }

  if (rangeMm > 5000)
  {
    return false;
  }

  *distanceMm = (float)rangeMm;
  lastRawTemp = (int32_t)rangeMm;
  lastRawPressure = (int32_t)vl53LastRawRangeStatus;
  return true;
}

// =====================================================
// TMP117 raw Soft-I2C driver
// =====================================================
static bool tmp117Read16(uint8_t reg, uint16_t *value)
{
  if (value == NULL) return false;

  uint8_t data[2] = {0, 0};
  if (!softI2cReadRegisters(TMP117_ADDR, reg, data, 2))
  {
    return false;
  }

  // TMP117 sends MSB first.
  *value = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
  return true;
}

static bool tmp117Begin()
{
  // Use exactly the same already-stable bit-banged bus as SCD41/DPS310/TSL2591.
  sdaHigh();
  sclHigh();
  delay(10);
  softI2cRecoverBus();

  tmp117LastReadWasWaiting = false;

  uint16_t deviceId = 0;
  if (!tmp117Read16(TMP117_REG_DEVICE_ID, &deviceId))
  {
    Serial.println("[TMP117] no response at 0x48");
    return false;
  }

  Serial.print("[TMP117] device ID=0x");
  Serial.println(deviceId, HEX);

  if (deviceId != TMP117_EXPECTED_DEVICE_ID)
  {
    Serial.print("[TMP117] unexpected ID, expected 0x");
    Serial.println(TMP117_EXPECTED_DEVICE_ID, HEX);
    return false;
  }

  Serial.println("[TMP117] init OK; first valid conversion can take about 1 s");
  return true;
}

static bool readTmp117(float *temperatureC)
{
  if (temperatureC == NULL) return false;

  uint16_t rawU16 = 0;
  if (!tmp117Read16(TMP117_REG_TEMPERATURE, &rawU16))
  {
    tmp117LastReadWasWaiting = false;
    return false;
  }

  // After reset TMP117 reports 0x8000 (-256 C) until first conversion completes.
  if (rawU16 == 0x8000)
  {
    tmp117LastReadWasWaiting = true;
    return false;
  }

  tmp117LastReadWasWaiting = false;

  int16_t raw = (int16_t)rawU16;
  float t = (float)raw * TMP117_LSB_C;

  if (isnan(t) || isinf(t) || t < -55.0f || t > 150.0f)
  {
    return false;
  }

  *temperatureC = t;
  lastRawTemp = raw;
  lastRawPressure = 0;
  return true;
}

// =====================================================
// Active sensor helpers + DS18B20 1-Wire
// =====================================================
bool pressureValueValid(float value)
{
  return !isnan(value) && value > 100.0f && value < 2000.0f;
}

const char *activeSensorName()
{
  if (activeSensorMode == SENSOR_MODE_DS18B20) return "DS18B20 방수";
  if (activeSensorMode == SENSOR_MODE_SCD41) return "SCD41 CO2";
  if (activeSensorMode == SENSOR_MODE_TSL2591) return "TSL2591 조도";
  if (activeSensorMode == SENSOR_MODE_TMP117) return "TMP117 정밀온도";
  if (activeSensorMode == SENSOR_MODE_INA228) return "INA228 전압·전류";
  if (activeSensorMode == SENSOR_MODE_ENCODER) return "회전 엔코더";
  if (activeSensorMode == SENSOR_MODE_BLE) return bleNodeQuantity;
  if (activeSensorMode == SENSOR_MODE_VL53L1X) return "VL53L1X 거리";
  return "DPS310";
}

const char *activeMeasurementTitle()
{
  if (activeSensorMode == SENSOR_MODE_DS18B20) return "수온";
  if (activeSensorMode == SENSOR_MODE_SCD41) return "이산화탄소";
  if (activeSensorMode == SENSOR_MODE_TSL2591) return "조도";
  if (activeSensorMode == SENSOR_MODE_TMP117) return "정밀 온도";
  if (activeSensorMode == SENSOR_MODE_INA228) return "전압 · 전류";
  if (activeSensorMode == SENSOR_MODE_ENCODER) return "회전";
  if (activeSensorMode == SENSOR_MODE_BLE) return bleNodeQuantity;
  if (activeSensorMode == SENSOR_MODE_VL53L1X) return "거리";
  return "온도 · 기압";
}

const char *activePrimaryName()
{
  if (activeSensorMode == SENSOR_MODE_DS18B20) return "수온";
  if (activeSensorMode == SENSOR_MODE_SCD41) return "CO2";
  if (activeSensorMode == SENSOR_MODE_TSL2591) return "조도";
  if (activeSensorMode == SENSOR_MODE_TMP117) return "정밀온도";
  if (activeSensorMode == SENSOR_MODE_INA228) return "전류";
  if (activeSensorMode == SENSOR_MODE_ENCODER) return "각도";
  if (activeSensorMode == SENSOR_MODE_BLE) return bleNodeQuantity;
  if (activeSensorMode == SENSOR_MODE_VL53L1X) return "거리";
  return "온도";
}

const char *activePrimaryUnit()
{
  if (activeSensorMode == SENSOR_MODE_SCD41) return "ppm";
  if (activeSensorMode == SENSOR_MODE_TSL2591) return "lux";
  if (activeSensorMode == SENSOR_MODE_VL53L1X) return "mm";
  if (activeSensorMode == SENSOR_MODE_INA228) return "A";
  if (activeSensorMode == SENSOR_MODE_ENCODER) return "°";
  if (activeSensorMode == SENSOR_MODE_BLE) return bleNodeUnit;
  return "℃";
}

const char *activeSecondaryName()
{
  if (activeSensorMode == SENSOR_MODE_SCD41) return "온도";
  return activeSensorMode == SENSOR_MODE_DPS310 ? "기압" : "";
}

const char *activeSecondaryUnit()
{
  if (activeSensorMode == SENSOR_MODE_SCD41) return "℃";
  return activeSensorMode == SENSOR_MODE_DPS310 ? "hPa" : "";
}

bool activeSensorHasPressure()
{
  return activeSensorMode == SENSOR_MODE_DPS310;
}

// The 별칭 the platform shows beside the reading. Registration and every data
// packet must agree on it, so it lives in one place.
const char *islTemperatureNickname()
{
  if (activeSensorMode == SENSOR_MODE_DS18B20) return "수온센서";
  if (activeSensorMode == SENSOR_MODE_TMP117) return "정밀온도센서";
  if (activeSensorMode == SENSOR_MODE_INA228) return "전류센서";
  if (activeSensorMode == SENSOR_MODE_ENCODER) return "회전센서";
  if (activeSensorMode == SENSOR_MODE_BLE) return bleNodeQuantity;
  return "온도센서";
}

// The part behind each tile and the resolution it is displayed at. The
// figures are what the firmware actually prints, taken from
// formatPrimaryValueText(), not datasheet accuracy — a student comparing two
// readings cares which digits are real on screen.
const char *sensorDetailText(int mode)
{
  switch (mode)
  {
    case SENSOR_MODE_DPS310:  return "DPS310 · 0.01℃ · 0.1hPa";
    case SENSOR_MODE_DS18B20: return "DS18B20 · 0.01℃";
    case SENSOR_MODE_SCD41:   return "SCD41 · 1ppm · 0.1℃ · 0.1%";
    case SENSOR_MODE_TSL2591: return "TSL2591 · 0.1lx";
    case SENSOR_MODE_TMP117:  return "TMP117 · 0.001℃";
    case SENSOR_MODE_INA228:  return "INA228 · 0.001A · 0.001V · 0.001W";
    case SENSOR_MODE_ENCODER: return "TCUT1600X01 · 4.5°";
    case SENSOR_MODE_VL53L1X: return "VL53L1X · 1mm";
    default:                  return "";
  }
}

// How many quantities the active sensor reports at once.
int activeSensorValueCount()
{
  if (activeSensorMode == SENSOR_MODE_SCD41) return 3;   // CO2, 온도, 습도
  if (activeSensorMode == SENSOR_MODE_INA228) return 3;  // 전류, 전압, 전력
  if (activeSensorMode == SENSOR_MODE_ENCODER) return 2;  // 각도, 각속도
  if (activeSensorMode == SENSOR_MODE_DPS310) return 2;  // 온도, 기압
  return 1;
}

bool activeSensorSupportsDirectIsl()
{
  // 지능형과학실 ON:
  // - DPS310/DS18B20/TMP117: temperature, all on the verified TPR code
  // - TSL2591 조도: ILM / ILMN fallback
  // - VL53L1X 거리: DITC with fallbacks, since no example in the spec uses one
  // - SCD41: the spec has no CO2 code, so the CTRT(농도) mapping stays behind
  //   SCD41_DIRECT_ISL_ENABLED until the server is seen to accept it
  //
  // Every sensor the home screen offers must be listed here. A sensor that is
  // selectable but absent from this list measures and logs normally while
  // silently sending nothing, which is not a failure anyone notices in class.
  return activeSensorMode == SENSOR_MODE_DPS310 ||
         activeSensorMode == SENSOR_MODE_DS18B20 ||
         activeSensorMode == SENSOR_MODE_TMP117 ||
         activeSensorMode == SENSOR_MODE_TSL2591 ||
         activeSensorMode == SENSOR_MODE_VL53L1X ||
         activeSensorMode == SENSOR_MODE_INA228 ||
         activeSensorMode == SENSOR_MODE_ENCODER ||
         (SCD41_DIRECT_ISL_ENABLED && activeSensorMode == SENSOR_MODE_SCD41);
}

static int activePrimaryDecimals()
{
  if (activeSensorMode == SENSOR_MODE_SCD41) return 0;
  if (activeSensorMode == SENSOR_MODE_TSL2591) return 1;
  if (activeSensorMode == SENSOR_MODE_TMP117) return 3;
  if (activeSensorMode == SENSOR_MODE_VL53L1X) return 0;
  return 2;
}

static float activePrimaryMinimumSpan()
{
  if (activeSensorMode == SENSOR_MODE_SCD41) return 100.0f;
  if (activeSensorMode == SENSOR_MODE_TSL2591) return 10.0f;
  if (activeSensorMode == SENSOR_MODE_TMP117) return 0.5f;
  if (activeSensorMode == SENSOR_MODE_VL53L1X) return 100.0f;
  return 1.0f;
}

static void formatPrimaryValueText(char *out, size_t outSize, float value, bool includeName)
{
  if (out == NULL || outSize == 0) return;

  if (activeSensorMode == SENSOR_MODE_SCD41)
  {
    if (includeName) snprintf(out, outSize, "CO2: %.0fppm", value);
    else snprintf(out, outSize, "%.0f", value);
  }
  else if (activeSensorMode == SENSOR_MODE_TSL2591)
  {
    if (includeName) snprintf(out, outSize, "조도: %.1flux", value);
    else snprintf(out, outSize, "%.1f", value);
  }
  else if (activeSensorMode == SENSOR_MODE_TMP117)
  {
    if (includeName) snprintf(out, outSize, "정밀온도: %.3f℃", value);
    else snprintf(out, outSize, "%.3f", value);
  }
  else if (activeSensorMode == SENSOR_MODE_VL53L1X)
  {
    if (includeName) snprintf(out, outSize, "거리: %.0fmm", value);
    else snprintf(out, outSize, "%.0f", value);
  }
  else if (activeSensorMode == SENSOR_MODE_INA228)
  {
    if (includeName) snprintf(out, outSize, "전류: %.3fA", value);
    else snprintf(out, outSize, "%.3f", value);
  }
  else if (activeSensorMode == SENSOR_MODE_ENCODER)
  {
    if (includeName) snprintf(out, outSize, "각도: %.1f°", value);
    else snprintf(out, outSize, "%.1f", value);
  }
  else if (activeSensorMode == SENSOR_MODE_BLE)
  {
    // A node can be sending anything, so no assumption about decimals.
    if (includeName) snprintf(out, outSize, "%s: %.4g%s", bleNodeQuantity, value, bleNodeUnit);
    else snprintf(out, outSize, "%.4g", value);
  }
  else if (activeSensorMode == SENSOR_MODE_DS18B20)
  {
    if (includeName) snprintf(out, outSize, "수온: %.2f℃", value);
    else snprintf(out, outSize, "%.2f", value);
  }
  else
  {
    if (includeName) snprintf(out, outSize, "온도: %.2f℃", value);
    else snprintf(out, outSize, "%.2f", value);
  }
}

// Digits only, matching the precision each sensor reports. The big readout
// uses a digits-only face, so its text must never contain a name or a unit.
static void formatPrimaryPlaceholderNumber(char *out, size_t outSize)
{
  if (out == NULL || outSize == 0) return;

  if (activeSensorMode == SENSOR_MODE_SCD41) snprintf(out, outSize, "----");
  else if (activeSensorMode == SENSOR_MODE_TSL2591) snprintf(out, outSize, "----.-");
  else if (activeSensorMode == SENSOR_MODE_TMP117) snprintf(out, outSize, "--.---");
  else if (activeSensorMode == SENSOR_MODE_INA228) snprintf(out, outSize, "-.---");
  else if (activeSensorMode == SENSOR_MODE_ENCODER) snprintf(out, outSize, "---.-");
  else if (activeSensorMode == SENSOR_MODE_VL53L1X) snprintf(out, outSize, "----");
  else snprintf(out, outSize, "--.--");
}

// Arrange the readings for however many the active sensor reports: one gets
// the full width at 64 px, two or three share the row at 48 px. Called when
// the sensor changes, and again after each update because the labels are
// width-to-content and the units sit against their right edge.
static void refreshMeasureValueLayout()
{
  if (labelValueNumber[0] == NULL) return;

  const int count = activeSensorValueCount();
  const int colLeftX = 24;
  const int colLeftW = 700;
  const int contentTop = 58;
  const int columnW = colLeftW / (count > 0 ? count : 1);

  for (int i = 0; i < MEASURE_VALUE_MAX; i++)
  {
    const bool used = (i < count);

    if (!used)
    {
      lv_obj_add_flag(labelValueCaption[i], LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(labelValueNumber[i], LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(labelValueUnit[i], LV_OBJ_FLAG_HIDDEN);
      continue;
    }

    lv_obj_clear_flag(labelValueCaption[i], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(labelValueNumber[i], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(labelValueUnit[i], LV_OBJ_FLAG_HIDDEN);

    const int x = colLeftX + i * columnW;

    lv_obj_set_style_text_font(labelValueNumber[i], count == 1 ? FONT_VALUE : FONT_VALUE_SMALL, 0);

    lv_obj_set_width(labelValueCaption[i], columnW - 12);
    lv_obj_align(labelValueCaption[i], LV_ALIGN_TOP_LEFT, x, contentTop + 34);
    lv_obj_align(labelValueNumber[i], LV_ALIGN_TOP_LEFT, x, contentTop + 56);
    lv_obj_align_to(labelValueUnit[i], labelValueNumber[i], LV_ALIGN_OUT_RIGHT_BOTTOM, 8, -8);
  }
}

// What each column is showing, for the active sensor.
static void measureValueMeta(int index, const char **caption, const char **unit)
{
  *caption = "";
  *unit = "";

  if (activeSensorMode == SENSOR_MODE_SCD41)
  {
    if (index == 0) { *caption = "이산화탄소"; *unit = "ppm"; }
    else if (index == 1) { *caption = "온도"; *unit = "℃"; }
    else if (index == 2) { *caption = "습도"; *unit = "%"; }
    return;
  }

  if (activeSensorMode == SENSOR_MODE_ENCODER)
  {
    if (index == 0) { *caption = "각도"; *unit = "°"; }
    else if (index == 1) { *caption = "각속도"; *unit = "°/s"; }
    return;
  }

  if (activeSensorMode == SENSOR_MODE_INA228)
  {
    if (index == 0) { *caption = "전류"; *unit = "A"; }
    else if (index == 1) { *caption = "전압"; *unit = "V"; }
    else if (index == 2) { *caption = "전력"; *unit = "W"; }
    return;
  }

  if (activeSensorMode == SENSOR_MODE_DPS310)
  {
    if (index == 0) { *caption = "온도"; *unit = "℃"; }
    else if (index == 1) { *caption = "기압"; *unit = "hPa"; }
    return;
  }

  if (index == 0)
  {
    *caption = activePrimaryName();
    *unit = activePrimaryUnit();
  }
}

// Placeholder digits at the width each value will occupy, so the layout does
// not jump when the first reading lands.
static void measureValuePlaceholder(int index, char *out, size_t outSize)
{
  if (index == 0)
  {
    formatPrimaryPlaceholderNumber(out, outSize);
    return;
  }

  if (activeSensorMode == SENSOR_MODE_SCD41) snprintf(out, outSize, "--.-");
  else if (activeSensorMode == SENSOR_MODE_INA228) snprintf(out, outSize, "-.---");
  else if (activeSensorMode == SENSOR_MODE_ENCODER) snprintf(out, outSize, "---.-");
  else if (activeSensorMode == SENSOR_MODE_BLE) snprintf(out, outSize, "----");
  else snprintf(out, outSize, "----.-");
}

void refreshMeasureValueCaptions()
{
  if (labelValueCaption[0] == NULL) return;

  for (int i = 0; i < MEASURE_VALUE_MAX; i++)
  {
    const char *caption = "";
    const char *unit = "";
    measureValueMeta(i, &caption, &unit);

    lv_label_set_text(labelValueCaption[i], caption);
    lv_label_set_text(labelValueUnit[i], unit);

    // Every other sensor's unit is a string literal in this file, so the
    // build-time subset font is guaranteed to cover it. A BLE node names its
    // own unit at runtime and nothing here can have subset a glyph for it, so
    // that one case falls back to the full-range face.
    lv_obj_set_style_text_font(
      labelValueUnit[i],
      activeSensorMode == SENSOR_MODE_BLE ? FONT_KR : FONT_KR_HEAD,
      0
    );
  }
}

void updateActiveSensorUiLabels()
{
  char text[96];

  if (labelMeasureSensorName)
  {
    lv_label_set_text(labelMeasureSensorName, activeMeasurementTitle());

    // Same reason as the unit above: this title is the node's own wording.
    lv_obj_set_style_text_font(
      labelMeasureSensorName,
      activeSensorMode == SENSOR_MODE_BLE ? FONT_KR : FONT_KR_HEAD,
      0
    );
  }

  refreshMeasureValueCaptions();

  if (labelValueNumber[0])
  {
    for (int i = 0; i < MEASURE_VALUE_MAX; i++)
    {
      measureValuePlaceholder(i, text, sizeof(text));
      lv_label_set_text(labelValueNumber[i], text);
    }

    refreshMeasureValueLayout();
  }

  if (labelMeasureTempAxisTitle)
  {
    snprintf(text, sizeof(text), "%s(%s)", activePrimaryName(), activePrimaryUnit());
    lv_label_set_text(labelMeasureTempAxisTitle, text);
  }

  if (labelMeasurePressureAxisTitle)
  {
    if (activeSensorHasPressure())
    {
      snprintf(text, sizeof(text), "%s(%s)", activeSecondaryName(), activeSecondaryUnit());
      lv_label_set_text(labelMeasurePressureAxisTitle, text);
      lv_obj_clear_flag(labelMeasurePressureAxisTitle, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
      lv_label_set_text(labelMeasurePressureAxisTitle, "");
      lv_obj_add_flag(labelMeasurePressureAxisTitle, LV_OBJ_FLAG_HIDDEN);
    }
  }

  applyTableValueHeaders();
}

// Show exactly one of 시작 / 정지, and keep the send-status rail truthful about
// where the data has actually reached.
void refreshMeasureControls()
{
  if (btnMeasureStart && btnMeasureStop)
  {
    if (measuring)
    {
      lv_obj_add_flag(btnMeasureStart, LV_OBJ_FLAG_HIDDEN);
      lv_obj_clear_flag(btnMeasureStop, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
      lv_obj_clear_flag(btnMeasureStart, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(btnMeasureStop, LV_OBJ_FLAG_HIDDEN);
    }
  }

  if (labelMeasureIslState)
  {
    const bool configured = cloudModumConfigured();
    const char *text = "미설정";
    uint32_t color = UI_TEXT_3;

    if (configured && wifiConnected && measuring) { text = "전송 중"; color = UI_OK; }
    else if (configured && wifiConnected) { text = "준비"; color = UI_TEXT_2; }
    else if (configured) { text = "WiFi 없음"; color = UI_DANGER; }

    lv_label_set_text(labelMeasureIslState, text);
    lv_obj_set_style_text_color(labelMeasureIslState, lv_color_hex(color), 0);
  }

  if (labelSd)
  {
    const char *text = "안 씀";
    uint32_t color = UI_TEXT_3;

    if (sdReady && csvLoggingEnabled) { text = "저장 중"; color = UI_OK; }
    else if (sdReady) { text = "대기"; color = UI_TEXT_2; }

    lv_label_set_text(labelSd, text);
    lv_obj_set_style_text_color(labelSd, lv_color_hex(color), 0);
  }

  if (labelMeasureBle)
  {
    lv_label_set_text(labelMeasureBle, bleSensorStateText());
    lv_obj_set_style_text_color(
      labelMeasureBle,
      lv_color_hex(bleSensorIsSubscribed() ? UI_OK : (bleSensorIsConnected() ? UI_TEXT_2 : UI_TEXT_3)),
      0
    );
  }

  if (labelMeasureModum)
  {
    if (strlen(islRuntimeSerialNumber) > 0)
    {
      // Only the tail is identifying, and the full code does not fit.
      const size_t len = strlen(islRuntimeSerialNumber);
      const char *tail = len > 6 ? islRuntimeSerialNumber + len - 6 : islRuntimeSerialNumber;

      char buf[16];
      snprintf(buf, sizeof(buf), "…%s", tail);
      lv_label_set_text(labelMeasureModum, buf);
      lv_obj_set_style_text_color(labelMeasureModum, lv_color_hex(UI_TEXT_2), 0);
    }
    else
    {
      lv_label_set_text(labelMeasureModum, "미입력");
      lv_obj_set_style_text_color(labelMeasureModum, lv_color_hex(UI_TEXT_3), 0);
    }
  }
}

static void refreshLatestMeasurementLabels()
{
  if (labelValueNumber[0] == NULL) return;

  char text[96];

  if (sampleCount <= 0)
  {
    for (int i = 0; i < MEASURE_VALUE_MAX; i++)
    {
      measureValuePlaceholder(i, text, sizeof(text));
      lv_label_set_text(labelValueNumber[i], text);
    }

    refreshMeasureValueLayout();
    return;
  }

  const int idx = sampleCount - 1;

  // Column 0 is always the sensor's headline quantity.
  formatPrimaryValueText(text, sizeof(text), tempHistory[idx], false);
  lv_label_set_text(labelValueNumber[0], text);

  // Columns 1 and 2 carry whatever else the part measured in the same reading:
  // 기압 for DPS310, 온도 and 습도 for SCD41. Each is a bare number under its
  // own caption, at the same size as the first — a sensor that measures two
  // things has two readings, not one reading and a footnote.
  if (activeSensorMode == SENSOR_MODE_SCD41)
  {
    if (!isnan(pressureHistory[idx])) snprintf(text, sizeof(text), "%.1f", pressureHistory[idx]);
    else snprintf(text, sizeof(text), "--.-");
    lv_label_set_text(labelValueNumber[1], text);

    if (!isnan(humidityHistory[idx])) snprintf(text, sizeof(text), "%.1f", humidityHistory[idx]);
    else snprintf(text, sizeof(text), "--.-");
    lv_label_set_text(labelValueNumber[2], text);
  }
  else if (activeSensorMode == SENSOR_MODE_INA228)
  {
    if (!isnan(pressureHistory[idx])) snprintf(text, sizeof(text), "%.3f", pressureHistory[idx]);
    else snprintf(text, sizeof(text), "-.---");
    lv_label_set_text(labelValueNumber[1], text);

    if (!isnan(humidityHistory[idx])) snprintf(text, sizeof(text), "%.3f", humidityHistory[idx]);
    else snprintf(text, sizeof(text), "-.---");
    lv_label_set_text(labelValueNumber[2], text);
  }
  else if (activeSensorMode == SENSOR_MODE_ENCODER)
  {
    if (!isnan(pressureHistory[idx])) snprintf(text, sizeof(text), "%.1f", pressureHistory[idx]);
    else snprintf(text, sizeof(text), "---.-");
    lv_label_set_text(labelValueNumber[1], text);
  }
  else if (activeSensorHasPressure())
  {
    if (pressureValueValid(pressureHistory[idx])) snprintf(text, sizeof(text), "%.1f", pressureHistory[idx]);
    else snprintf(text, sizeof(text), "----.-");
    lv_label_set_text(labelValueNumber[1], text);
  }

  refreshMeasureValueLayout();
}


static inline void dsOwRelease()
{
  gpio_set_direction(DS18B20_DQ_GPIO, GPIO_MODE_INPUT);
  gpio_set_pull_mode(DS18B20_DQ_GPIO, GPIO_PULLUP_ONLY);
}

static inline void dsOwLow()
{
  gpio_set_level(DS18B20_DQ_GPIO, 0);
  gpio_set_direction(DS18B20_DQ_GPIO, GPIO_MODE_OUTPUT);
}

static inline int dsOwReadLevel()
{
  return gpio_get_level(DS18B20_DQ_GPIO);
}

static bool dsOwReset()
{
  bool presence;
  noInterrupts();
  dsOwLow();
  delayMicroseconds(520);
  dsOwRelease();
  delayMicroseconds(80);
  presence = (dsOwReadLevel() == 0);
  delayMicroseconds(450);
  interrupts();
  return presence;
}

static void dsOwWriteBit(uint8_t bitValue)
{
  noInterrupts();
  if (bitValue)
  {
    dsOwLow();
    delayMicroseconds(8);
    dsOwRelease();
    delayMicroseconds(80);
  }
  else
  {
    dsOwLow();
    delayMicroseconds(80);
    dsOwRelease();
    delayMicroseconds(12);
  }
  interrupts();
}

static uint8_t dsOwReadBit()
{
  uint8_t bitValue;
  noInterrupts();
  dsOwLow();
  delayMicroseconds(4);
  dsOwRelease();
  delayMicroseconds(15);
  bitValue = dsOwReadLevel();
  delayMicroseconds(65);
  interrupts();
  return bitValue;
}

static void dsOwWriteByte(uint8_t data)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    dsOwWriteBit(data & 0x01);
    data >>= 1;
  }
}

static uint8_t dsOwReadByte()
{
  uint8_t data = 0;
  for (uint8_t i = 0; i < 8; i++)
  {
    if (dsOwReadBit()) data |= (1 << i);
  }
  return data;
}

static uint8_t dsCrc8Dallas(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;
  while (len--)
  {
    uint8_t inByte = *data++;
    for (uint8_t i = 0; i < 8; i++)
    {
      uint8_t mix = (crc ^ inByte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      inByte >>= 1;
    }
  }
  return crc;
}

static bool ds18b20ConversionStarted = false;
static unsigned long ds18b20ConversionStartMs = 0;

static bool ds18b20StartConversion()
{
  if (!dsOwReset()) return false;
  dsOwWriteByte(0xCC);  // Skip ROM: one DS18B20 on the bus
  dsOwWriteByte(0x44);  // Convert T
  ds18b20ConversionStarted = true;
  ds18b20ConversionStartMs = millis();
  return true;
}

bool ds18b20Begin()
{
  pinMode(SOFT_SCL, INPUT);  // GPIO3 unused in DS18B20 mode
  dsOwRelease();
  ds18b20ConversionStarted = false;
  if (!dsOwReset()) return false;
  return ds18b20StartConversion();
}

bool readDs18b20(float *temperatureC)
{
  if (!ds18b20ConversionStarted)
  {
    ds18b20StartConversion();
    return false;
  }

  if (millis() - ds18b20ConversionStartMs < DS18B20_CONVERT_MS)
  {
    return false;
  }

  uint8_t data[9];
  if (!dsOwReset())
  {
    ds18b20ConversionStarted = false;
    return false;
  }

  dsOwWriteByte(0xCC);
  dsOwWriteByte(0xBE);

  for (uint8_t i = 0; i < 9; i++)
  {
    data[i] = dsOwReadByte();
  }

  uint8_t crc = dsCrc8Dallas(data, 8);
  if (crc != data[8])
  {
    ds18b20StartConversion();
    return false;
  }

  int16_t raw = (int16_t)((data[1] << 8) | data[0]);
  float t = raw / 16.0f;

  if (t < -55.0f || t > 125.0f)
  {
    ds18b20StartConversion();
    return false;
  }

  lastRawTemp = raw;
  lastRawPressure = 0;
  *temperatureC = t;

  ds18b20StartConversion();
  return true;
}

// The INA228 has 16-bit configuration registers; the byte-wide helper cannot
// reach them.
static bool ina228WriteRegister16(uint8_t reg, uint16_t value)
{
  softI2cStart();

  bool ok = softI2cWriteByte(INA228_ADDR << 1);
  ok = ok && softI2cWriteByte(reg);
  ok = ok && softI2cWriteByte((uint8_t)(value >> 8));
  ok = ok && softI2cWriteByte((uint8_t)(value & 0xFF));

  softI2cStop();
  return ok;
}

// VBUS, CURRENT and POWER are 24-bit. Returns the raw register contents.
static bool ina228ReadRegister24(uint8_t reg, uint32_t *out)
{
  uint8_t buf[3] = {0, 0, 0};
  if (!softI2cReadRegisters(INA228_ADDR, reg, buf, 3)) return false;

  *out = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
  return true;
}

bool ina228Begin()
{
  sdaHigh();
  sclHigh();
  delay(10);
  softI2cRecoverBus();

  ina228Ready = false;
  ina228LastPowerW = NAN;

  uint8_t idBuf[2] = {0, 0};
  if (!softI2cReadRegisters(INA228_ADDR, INA228_REG_MANUFACTURER_ID, idBuf, 2))
  {
    Serial.println("[INA228] no response at 0x40");
    return false;
  }

  const uint16_t manufacturer = ((uint16_t)idBuf[0] << 8) | idBuf[1];
  if (manufacturer != INA228_MANUFACTURER_TI)
  {
    Serial.printf("[INA228] unexpected manufacturer id 0x%04X\n", manufacturer);
    return false;
  }

  // Reset, then let the part come back up.
  ina228WriteRegister16(INA228_REG_CONFIG, 0x8000);
  delay(5);

  // Continuous conversion of bus, shunt and temperature, 1052 µs each,
  // averaged over 16 samples — steady enough to read once a second.
  if (!ina228WriteRegister16(INA228_REG_ADC_CONFIG, 0xFB6A))
  {
    Serial.println("[INA228] ADC config write failed");
    return false;
  }

  // CURRENT_LSB = max current / 2^19, and the calibration register scales the
  // shunt voltage into that unit.
  ina228CurrentLsb = INA228_MAX_CURRENT / 524288.0f;
  const float shuntCal = 13107.2e6f * ina228CurrentLsb * INA228_SHUNT_OHMS;
  const uint16_t shuntCalReg = (uint16_t)(shuntCal + 0.5f);

  if (!ina228WriteRegister16(INA228_REG_SHUNT_CAL, shuntCalReg))
  {
    Serial.println("[INA228] shunt calibration write failed");
    return false;
  }

  Serial.printf(
    "[INA228] ready: shunt %.3f ohm, max %.1f A, cal %u\n",
    INA228_SHUNT_OHMS, INA228_MAX_CURRENT, (unsigned)shuntCalReg
  );

  ina228Ready = true;
  return true;
}

// Bus voltage in V, current in A, power in W.
bool readIna228(float *voltageV, float *currentA, float *powerW)
{
  if (!ina228Ready || voltageV == NULL || currentA == NULL || powerW == NULL) return false;

  uint32_t rawVbus = 0;
  uint32_t rawCurrent = 0;
  uint32_t rawPower = 0;

  if (!ina228ReadRegister24(INA228_REG_VBUS, &rawVbus)) return false;
  if (!ina228ReadRegister24(INA228_REG_CURRENT, &rawCurrent)) return false;
  if (!ina228ReadRegister24(INA228_REG_POWER, &rawPower)) return false;

  // VBUS and CURRENT carry their value in the upper 20 bits.
  *voltageV = (float)(rawVbus >> 4) * INA228_VBUS_LSB_V;

  int32_t current20 = (int32_t)(rawCurrent >> 4);
  if (current20 & 0x00080000) current20 -= 0x00100000;   // sign-extend from 20 bits
  *currentA = (float)current20 * ina228CurrentLsb;

  *powerW = (float)rawPower * INA228_POWER_LSB_K * ina228CurrentLsb;

  ina228LastPowerW = *powerW;
  return true;
}

void encoderEnd();
void encoderZero();

// Addresses the firmware already knows how to talk to, so a scan can say
// which of what it found is a sensor the board supports.
static const char *knownI2cDeviceName(uint8_t addr)
{
  switch (addr)
  {
    case 0x29: return "VL53L1X 거리";
    case 0x40: return "INA228 전압·전류";
    case 0x48: return "TMP117 정밀온도";
    case 0x62: return "SCD41 이산화탄소";
    case 0x76: case 0x77: return "DPS310 온도·기압";
    default: return NULL;
  }
}

// Walks the 7-bit address space and reports every device that acknowledges.
// Adding a sensor starts here: a part whose address does not appear is a
// wiring or power problem, and no amount of driver work will help.
int scanSensorI2cBus(char *summary, size_t summaryLen)
{
  // Driving the bus with the encoder's interrupts still attached would fire
  // the ISR on every clock edge of every address probe.
  encoderEnd();

  sdaHigh();
  sclHigh();
  delay(5);
  softI2cRecoverBus();

  int found = 0;
  if (summary && summaryLen) summary[0] = '\0';

  Serial.println("[I2C] scanning sensor bus (GPIO2/3)");

  for (uint8_t addr = 0x08; addr <= 0x77; addr++)
  {
    softI2cStart();
    const bool ack = softI2cWriteByte((uint8_t)(addr << 1));
    softI2cStop();

    if (!ack) continue;

    found++;
    const char *known = knownI2cDeviceName(addr);

    Serial.printf("[I2C] 0x%02X  %s\n", addr, known ? known : "(모르는 장치)");

    if (summary && summaryLen)
    {
      char entry[64];
      snprintf(entry, sizeof(entry), "%s0x%02X %s",
               summary[0] ? " · " : "", addr, known ? known : "?");
      strncat(summary, entry, summaryLen - strlen(summary) - 1);
    }
  }

  Serial.printf("[I2C] %d device(s)\n", found);

  if (found == 0 && summary && summaryLen)
  {
    snprintf(summary, summaryLen, "응답한 장치 없음 - 전원과 SDA/SCL 배선 확인");
  }

  return found;
}

// Standard quadrature table: index by the previous two-bit state followed by
// the current one, and the value is the direction. 0 covers both the invalid
// double transitions and no movement.
static const int8_t kQuadratureStep[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

static void IRAM_ATTR encoderIsr()
{
  const uint8_t state = (uint8_t)((digitalRead(ENCODER_PIN_A) << 1) | digitalRead(ENCODER_PIN_B));
  encoderCount += kQuadratureStep[(encoderLastState << 2) | state];
  encoderLastState = state;
  encoderEdgeCount = encoderEdgeCount + 1;
}

bool encoderBegin()
{
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  encoderCount = 0;
  encoderPrevCount = 0;
  encoderPrevMs = millis();
  encoderLastRateDegPerS = NAN;
  encoderLastState = (uint8_t)((digitalRead(ENCODER_PIN_A) << 1) | digitalRead(ENCODER_PIN_B));

  if (!encoderReady)
  {
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderIsr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), encoderIsr, CHANGE);
    encoderReady = true;
  }

  Serial.printf(
    "[ENCODER] ready on GPIO%d/%d, %d counts per turn\n",
    ENCODER_PIN_A, ENCODER_PIN_B, ENCODER_COUNTS_PER_REV
  );

  // Nothing to probe: an idle encoder is indistinguishable from an absent one
  // until it is turned, so report ready and let the reading show the truth.
  return true;
}

// Back to zero degrees without disturbing the interrupts.
void encoderZero()
{
  noInterrupts();
  encoderCount = 0;
  interrupts();

  encoderPrevCount = 0;
  encoderPrevMs = millis();
  encoderLastRateDegPerS = NAN;
}

// Hands the two pins back so the soft-I2C driver can drive them again.
void encoderEnd()
{
  if (!encoderReady) return;

  detachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A));
  detachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B));
  encoderReady = false;

  Serial.println("[ENCODER] detached");
}

// Angle in degrees since the run started, and how fast it is turning.
bool readEncoder(float *angleDeg, float *rateDegPerS)
{
  if (!encoderReady || angleDeg == NULL || rateDegPerS == NULL) return false;

  noInterrupts();
  const int32_t count = encoderCount;
  interrupts();

  const unsigned long now = millis();
  const float degPerCount = 360.0f / (float)ENCODER_COUNTS_PER_REV;

  *angleDeg = (float)count * degPerCount;

  const unsigned long elapsed = now - encoderPrevMs;
  if (elapsed >= 200)
  {
    const float deltaDeg = (float)(count - encoderPrevCount) * degPerCount;
    encoderLastRateDegPerS = deltaDeg * 1000.0f / (float)elapsed;
    encoderPrevCount = count;
    encoderPrevMs = now;
  }

  *rateDegPerS = encoderLastRateDegPerS;

  Serial.printf(
    "[ENCODER] A=%d B=%d edges=%lu count=%ld angle=%.1f\n",
    digitalRead(ENCODER_PIN_A), digitalRead(ENCODER_PIN_B),
    (unsigned long)encoderEdgeCount, (long)count, *angleDeg
  );

  return true;
}

// Pins this board leaves free and that are safe to switch to an input with a
// pull-up. Deliberately no pin above 20: the ESP32-P4 runs its flash and PSRAM
// on the high pads, and reconfiguring one of those would take the board down.
// 2, 3, 7, 8, 14-19, 21-23 and 27 are left out because they are already
// driving the sensor bus, the touch panel, the radio link or the backlight.
static const uint8_t kEncoderCandidatePins[] = { SOFT_SDA, SOFT_SCL, 4, 5, 6, 9, 10, 11, 12, 13, 20 };

// Which of the free pins move while the wheel turns. If the encoder is wired
// somewhere other than GPIO4/5 this finds it, and if nothing moves anywhere
// then the signal is not reaching the board at all.
void encoderFindActivePins(char *out, size_t outSize)
{
  if (out == NULL || outSize == 0) return;

  const int pinCount = (int)(sizeof(kEncoderCandidatePins) / sizeof(kEncoderCandidatePins[0]));
  int changes[sizeof(kEncoderCandidatePins) / sizeof(kEncoderCandidatePins[0])] = { 0 };
  int last[sizeof(kEncoderCandidatePins) / sizeof(kEncoderCandidatePins[0])];

  for (int i = 0; i < pinCount; i++)
  {
    pinMode(kEncoderCandidatePins[i], INPUT_PULLUP);
  }

  delay(2);

  for (int i = 0; i < pinCount; i++)
  {
    last[i] = digitalRead(kEncoderCandidatePins[i]);
  }

  const unsigned long until = millis() + 3000;

  while ((long)(millis() - until) < 0)
  {
    for (int i = 0; i < pinCount; i++)
    {
      const int now = digitalRead(kEncoderCandidatePins[i]);
      if (now != last[i]) { changes[i]++; last[i] = now; }
    }

    delayMicroseconds(100);
  }

  out[0] = '\0';
  int moved = 0;

  for (int i = 0; i < pinCount; i++)
  {
    Serial.printf("[ENCODER] GPIO%-2d level=%d changes=%d\n",
                  kEncoderCandidatePins[i], last[i], changes[i]);

    if (changes[i] < 2) continue;

    char entry[32];
    snprintf(entry, sizeof(entry), "%sGPIO%d(%d회)",
             moved ? ", " : "", kEncoderCandidatePins[i], changes[i]);
    strncat(out, entry, outSize - strlen(out) - 1);
    moved++;
  }

  if (moved == 0)
  {
    snprintf(out, outSize,
             "움직인 핀 없음 - 배선/3.3V 전원, 그리고 슬릿 원판이 센서 홈을 지나는지 확인");
  }
}

bool activeSensorBegin()
{
  // The encoder holds SOFT_SDA/SOFT_SCL as interrupt inputs, so every other
  // mode has to take them back before it touches the bus.
  if (activeSensorMode != SENSOR_MODE_ENCODER)
  {
    encoderEnd();
  }

  if (activeSensorMode == SENSOR_MODE_ENCODER)
  {
    return encoderBegin();
  }

  // Nothing to initialise: the node is already connected, or it is not, and
  // that was settled on the 블루투스 screen.
  if (activeSensorMode == SENSOR_MODE_BLE)
  {
    return bleLinkIsSubscribed();
  }

  if (activeSensorMode == SENSOR_MODE_DS18B20)
  {
    return ds18b20Begin();
  }

  sdaHigh();
  sclHigh();

  if (activeSensorMode == SENSOR_MODE_SCD41)
  {
    return scd41Begin();
  }

  if (activeSensorMode == SENSOR_MODE_TSL2591)
  {
    return tsl2591Begin();
  }

  if (activeSensorMode == SENSOR_MODE_TMP117)
  {
    return tmp117Begin();
  }

  if (activeSensorMode == SENSOR_MODE_VL53L1X)
  {
    return vl53l1xBegin();
  }

  if (activeSensorMode == SENSOR_MODE_INA228)
  {
    return ina228Begin();
  }

  return dps310Begin();
}

bool readActiveSensor(float *primaryValue, float *secondaryValue)
{
  if (primaryValue == NULL || secondaryValue == NULL) return false;

  if (activeSensorMode == SENSOR_MODE_DS18B20)
  {
    if (!readDs18b20(primaryValue)) return false;
    *secondaryValue = NO_PRESSURE_VALUE;
    return true;
  }

  if (activeSensorMode == SENSOR_MODE_SCD41)
  {
    float sensorTempC = NAN;
    float sensorHumidityPct = NAN;
    if (!readScd41(primaryValue, &sensorTempC, &sensorHumidityPct)) return false;
    *secondaryValue = sensorTempC;
    return true;
  }

  if (activeSensorMode == SENSOR_MODE_TSL2591)
  {
    if (!readTsl2591(primaryValue)) return false;
    *secondaryValue = NO_PRESSURE_VALUE;
    return true;
  }

  if (activeSensorMode == SENSOR_MODE_TMP117)
  {
    if (!readTmp117(primaryValue)) return false;
    *secondaryValue = NO_PRESSURE_VALUE;
    return true;
  }

  if (activeSensorMode == SENSOR_MODE_VL53L1X)
  {
    if (!readVl53l1x(primaryValue)) return false;
    *secondaryValue = NO_PRESSURE_VALUE;
    return true;
  }

  if (activeSensorMode == SENSOR_MODE_INA228)
  {
    float busVoltage = NAN;
    float powerW = NAN;
    if (!readIna228(&busVoltage, primaryValue, &powerW)) return false;
    *secondaryValue = busVoltage;   // power rides along in ina228LastPowerW
    return true;
  }

  if (activeSensorMode == SENSOR_MODE_ENCODER)
  {
    float rateDegPerS = NAN;
    if (!readEncoder(primaryValue, &rateDegPerS)) return false;
    *secondaryValue = rateDegPerS;
    return true;
  }

  if (activeSensorMode == SENSOR_MODE_BLE)
  {
    float value = NAN;
    char quantity[24] = "";
    char unit[24] = "";
    uint32_t ageMs = 0;

    if (!bleLinkLatestReading(&value, quantity, unit, &ageMs)) return false;
    if (ageMs > BLE_READING_MAX_AGE_MS) return false;

    if (quantity[0]) snprintf(bleNodeQuantity, sizeof(bleNodeQuantity), "%s", quantity);
    if (unit[0]) snprintf(bleNodeUnit, sizeof(bleNodeUnit), "%s", unit);

    *primaryValue = value;
    *secondaryValue = NAN;
    return true;
  }

  return readDps310(primaryValue, secondaryValue);
}

void setActiveSensorMode(int mode)
{
  if (mode != SENSOR_MODE_DPS310 &&
      mode != SENSOR_MODE_DS18B20 &&
      mode != SENSOR_MODE_SCD41 &&
      mode != SENSOR_MODE_TSL2591 &&
      mode != SENSOR_MODE_TMP117 &&
      mode != SENSOR_MODE_INA228 &&
      mode != SENSOR_MODE_ENCODER &&
      mode != SENSOR_MODE_BLE &&
      mode != SENSOR_MODE_VL53L1X)
  {
    return;
  }

  if (isBatchUploading())
  {
    setIslStatusText("일괄전송 중: 센서 전환 무시");
    return;
  }

  measuring = false;
  measureClockRunning = false;
  resetDirectIslSessionCache("sensor mode changed");

  if (activeSensorMode == SENSOR_MODE_SCD41 && mode != SENSOR_MODE_SCD41)
  {
    scd41StopPeriodicMeasurement();
  }

  if (activeSensorMode == SENSOR_MODE_VL53L1X && mode != SENSOR_MODE_VL53L1X)
  {
    vl53StopRanging();
  }

  // 서로 다른 센서 단위가 한 history에 섞이지 않도록 센서를 바꾸면
  // 측정 버퍼를 새 실험으로 초기화합니다.
  if (activeSensorMode != mode)
  {
    measurementCount = 0;
    sampleCount = 0;
    tablePageOffset = 0;
    batchUploadStartIndex = 0;
    batchUploadProgressSent = 0;
    batchUploadProgressTotal = 0;
    measureAccumulatedMs = 0;
    measureResumeMs = 0;
    dataClockStarted = false;
    dataFirstSampleMs = 0;

    for (int i = 0; i < MAX_SAMPLES; i++)
    {
      sampleEnabled[i] = true;
      noHistory[i] = 0;
      tempHistory[i] = 0.0f;
      pressureHistory[i] = NO_PRESSURE_VALUE;
      humidityHistory[i] = NAN;
      timeHistory[i] = 0;
    }
  }

  activeSensorMode = mode;
  dpsReady = activeSensorBegin();

  if (labelStatus)
  {
    lv_label_set_text(labelStatus, dpsReady ? "준비" : "센서 초기화 실패");
  }

  updateActiveSensorUiLabels();

  if (labelRuntime) lv_label_set_text(labelRuntime, "시간 00:00:00");
  if (labelCount) lv_label_set_text(labelCount, "0");

  if (tableData)
  {
    resetTable();
  }

  clearChart();
  chartUiDirty = true;
  refreshHomeSensorLabels();

  if (activeSensorMode == SENSOR_MODE_SCD41)
  {
    setIslStatusText(dpsReady ? "CO2·온도·습도: SCD41 준비 / 첫 값 약 5초" : "SCD41 인식 실패: 0x62 / 전원·SDA·SCL 확인");
  }
  else if (activeSensorMode == SENSOR_MODE_TSL2591)
  {
    setIslStatusText(dpsReady ? "조도: TSL2591 준비" : "조도: TSL2591 인식 실패");
  }
  else if (activeSensorMode == SENSOR_MODE_TMP117)
  {
    setIslStatusText(dpsReady ? "정밀온도: TMP117 준비 / 약 1초 후 첫 값" : "TMP117 인식 실패: 0x48 / 배선 확인");
  }
  else if (activeSensorMode == SENSOR_MODE_VL53L1X)
  {
    setIslStatusText(dpsReady ? "거리: VL53L1X 준비" : "VL53L1X 인식 실패: 0x29 / 배선 확인");
  }
  else if (activeSensorMode == SENSOR_MODE_INA228)
  {
    setIslStatusText(dpsReady ? "전압·전류·전력: INA228 준비" : "INA228 인식 실패: 0x40 / 배선 확인");
  }
  else if (activeSensorMode == SENSOR_MODE_ENCODER)
  {
    setIslStatusText("회전: 엔코더 준비 / 센서포트, 3.3V 연결");
  }
  else if (activeSensorMode == SENSOR_MODE_BLE)
  {
    char text[96];
    snprintf(text, sizeof(text), "블루투스 센서: %s / %s",
             bleLinkPeerName()[0] ? bleLinkPeerName() : "-", bleLinkStateText());
    setIslStatusText(text);
  }
}

// =====================================================
// SDMMC CSV
// =====================================================
void powerOnSdCardLdo()
{
#if HAS_ESP_LDO
  static bool ldoDone = false;
  static esp_ldo_channel_handle_t sdLdo = NULL;

  if (!ldoDone)
  {
    esp_ldo_channel_config_t sdLdoConfig = {
      .chan_id = BSP_LDO_PROBE_SD_CHAN,
      .voltage_mv = BSP_LDO_PROBE_SD_VOLTAGE_MV
    };

    esp_err_t ret = esp_ldo_acquire_channel(&sdLdoConfig, &sdLdo);

    if (ret == ESP_OK)
    {
      Serial.println("SD LDO power ON");
    }
    else
    {
      Serial.print("SD LDO acquire result: ");
      Serial.println(esp_err_to_name(ret));
    }

    ldoDone = true;
  }
#else
  Serial.println("SD LDO helper not available");
#endif

  delay(80);
}

bool initSdCard()
{
  Serial.println("SDMMC init start");

  if (sdcard != NULL)
  {
    Serial.println("SD already mounted");
    sdReady = true;
    return true;
  }

  powerOnSdCardLdo();

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024
  };

  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.slot = SDMMC_HOST_SLOT_0;
  host.max_freq_khz = SD_INIT_FREQ_KHZ;

  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

  slot_config.width = SD_USE_1BIT ? 1 : 4;

  slot_config.clk = (gpio_num_t)SD_CLK;
  slot_config.cmd = (gpio_num_t)SD_CMD;
  slot_config.d0  = (gpio_num_t)SD_D0;
  slot_config.d1  = SD_USE_1BIT ? GPIO_NUM_NC : (gpio_num_t)SD_D1;
  slot_config.d2  = SD_USE_1BIT ? GPIO_NUM_NC : (gpio_num_t)SD_D2;
  slot_config.d3  = SD_USE_1BIT ? GPIO_NUM_NC : (gpio_num_t)SD_D3;

  slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

  esp_err_t ret = esp_vfs_fat_sdmmc_mount(
    MOUNT_POINT,
    &host,
    &slot_config,
    &mount_config,
    &sdcard
  );

  if (ret != ESP_OK)
  {
    Serial.print("SDMMC mount failed: ");
    Serial.println(esp_err_to_name(ret));
    snprintf(sdLastError, sizeof(sdLastError), "마운트 실패: %s", esp_err_to_name(ret));
    sdcard = NULL;
    sdReady = false;
    return false;
  }

  Serial.println("SDMMC mount OK");
  sdmmc_card_print_info(stdout, sdcard);
  sdLastError[0] = '\0';
  sdReady = true;

  struct stat st;

  if (stat(csvPath, &st) != 0)
  {
    FILE *file = fopen(csvPath, "w");

    if (file == NULL)
    {
      Serial.println("CSV create failed");
      snprintf(sdLastError, sizeof(sdLastError), "파일 생성 실패: %s", csvPath);
      sdReady = false;
      return false;
    }

    fprintf(file, CSV_HEADER_LINE ",raw1,raw2\n");
    fclose(file);

    Serial.println("CSV file created");
  }
  else
  {
    Serial.println("CSV file exists");
  }

  return true;
}

// The logging file stays open across samples. Re-opening and closing it once
// per second is what made per-sample CSV logging heavy enough to be ripped out
// before; holding the handle and flushing in batches keeps the loop cheap
// while still bounding how much data a power cut can lose.
static FILE *csvFile = NULL;
static int csvRowsSinceFlush = 0;

void closeCsvFile()
{
  if (csvFile == NULL) return;

  fflush(csvFile);
  fclose(csvFile);
  csvFile = NULL;
  csvRowsSinceFlush = 0;
}

void unmountSdCard()
{
  closeCsvFile();

  if (sdcard != NULL)
  {
    esp_vfs_fat_sdcard_unmount(MOUNT_POINT, sdcard);
    sdcard = NULL;
  }

  sdReady = false;
}

static void reportCsvWriteFailure()
{
  Serial.println("CSV append failed");
  closeCsvFile();
  sdReady = false;

  if (labelSd)
  {
    lv_label_set_text(labelSd, "SD: 저장 실패");
  }

  updateSdStatusLabels();
}

// Every reading a sensor produces, laid out as value,unit pairs for one row.
// Only the DPS310 used to reach the second column, so the SCD41's humidity and
// the INA228's voltage and power were measured, shown on screen and uploaded
// to the platform, but never landed on the card. Columns a sensor does not
// produce stay empty so a file that mixes sensors still lines up.
static void formatCsvValueColumns(
  float primaryValue, float secondaryValue, float thirdValue,
  char *out, size_t outSize
)
{
  const float values[CSV_VALUE_COLUMNS] = { primaryValue, secondaryValue, thirdValue };
  const int count = activeSensorValueCount();

  out[0] = '\0';

  for (int i = 0; i < CSV_VALUE_COLUMNS; i++)
  {
    char cell[64];
    const bool usable =
      i < count && !isnan(values[i]) &&
      (i != 1 || !activeSensorHasPressure() || pressureValueValid(values[i]));

    if (usable)
    {
      const char *caption = "";
      const char *unit = "";
      measureValueMeta(i, &caption, &unit);
      snprintf(cell, sizeof(cell), "%s%.4f,%s", i ? "," : "", values[i], unit);
    }
    else
    {
      snprintf(cell, sizeof(cell), "%s,", i ? "," : "");
    }

    strncat(out, cell, outSize - strlen(out) - 1);
  }
}

static float thirdValueForActiveSensor();

void appendCsv(uint32_t timeS, float primaryValue, float secondaryValue)
{
  if (!csvLoggingEnabled) return;
  if (!sdReady) return;

  if (csvFile == NULL)
  {
    csvFile = fopen(csvPath, "a");

    if (csvFile == NULL)
    {
      reportCsvWriteFailure();
      return;
    }

    csvRowsSinceFlush = 0;
  }

  char columns[160];
  formatCsvValueColumns(primaryValue, secondaryValue, thirdValueForActiveSensor(),
                        columns, sizeof(columns));

  const int written = fprintf(
    csvFile,
    "%d,%lu,%s,%s,%ld,%ld\n",
    measurementCount,
    (unsigned long)timeS,
    activeSensorName(),
    columns,
    (long)lastRawTemp,
    (long)lastRawPressure
  );

  if (written < 0)
  {
    reportCsvWriteFailure();
    return;
  }

  csvRowsSinceFlush++;

  if (csvRowsSinceFlush >= CSV_FLUSH_EVERY_ROWS)
  {
    csvRowsSinceFlush = 0;

    if (fflush(csvFile) != 0)
    {
      reportCsvWriteFailure();
    }
  }
}

void updateSdStatusLabels()
{
  const char *sdText = "SD: 해제됨";
  const char *csvText = csvLoggingEnabled ? "CSV: 대기 중" : "CSV: 꺼짐";

  if (sdReady && csvLoggingEnabled)
  {
    sdText = "SD: 마운트됨";
    csvText = "CSV: 자동 저장 켜짐";
  }
  else if (sdReady)
  {
    sdText = "SD: 마운트됨";
    csvText = "CSV: 저장 꺼짐";
  }
  else if (csvLoggingEnabled)
  {
    sdText = "SD: 필요";
    csvText = "CSV: SD 마운트 필요";
  }

  if (labelHomeSd) lv_label_set_text(labelHomeSd, sdText);
  if (labelSettingsSd) lv_label_set_text(labelSettingsSd, sdText);
  if (labelSettingsCsv) lv_label_set_text(labelSettingsCsv, csvText);
  if (labelHomeCsv)
  {
    char line[96];

    if (sdReady && csvLoggingEnabled) snprintf(line, sizeof(line), "SD: 자동 저장 중");
    else if (sdReady) snprintf(line, sizeof(line), "SD: 마운트됨 · 자동 저장 꺼짐");
    else if (sdLastError[0]) snprintf(line, sizeof(line), "SD: %s", sdLastError);
    else snprintf(line, sizeof(line), "SD: 마운트 안 됨");

    lv_label_set_text(labelHomeCsv, line);
    lv_obj_set_style_text_color(
      labelHomeCsv,
      lv_color_hex(sdReady ? UI_OK : (sdLastError[0] ? UI_DANGER : UI_TEXT_3)),
      0
    );
  }

  updateDashboardCsvLabels();
}

// =====================================================
// 시간 표시
// =====================================================
String getCurrentDateTimeText()
{
  time_t nowTime;
  struct tm timeInfo;

  time(&nowTime);

  if (localtime_r(&nowTime, &timeInfo) != NULL && (timeInfo.tm_year + 1900) >= 2025)
  {
    ntpSynced = true;

    char realTimeBuf[32];
    snprintf(
      realTimeBuf,
      sizeof(realTimeBuf),
      "%04d-%02d-%02d %02d:%02d:%02d",
      timeInfo.tm_year + 1900,
      timeInfo.tm_mon + 1,
      timeInfo.tm_mday,
      timeInfo.tm_hour,
      timeInfo.tm_min,
      timeInfo.tm_sec
    );
    return String(realTimeBuf);
  }

  // 실제 NTP/RTC 연결 전까지는 컴파일 시각을 기준으로 millis() 경과 시간을 더합니다.
  // C6 WiFi/NTP 연동 후에는 이 함수만 실제 시간 소스로 교체하면 됩니다.
  static bool initialized = false;
  static unsigned long baseMillis = 0;
  static int year = 2026;
  static int month = 1;
  static int day = 1;
  static int hour = 0;
  static int minute = 0;
  static int second = 0;

  if (!initialized)
  {
    initialized = true;
    baseMillis = millis();

    char monStr[4] = {0};
    int d = 1;
    int y = 2026;
    sscanf(__DATE__, "%3s %d %d", monStr, &d, &y);

    const char *months[] = {
      "Jan", "Feb", "Mar", "Apr", "May", "Jun",
      "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };

    int m = 1;
    for (int i = 0; i < 12; i++)
    {
      if (strncmp(monStr, months[i], 3) == 0)
      {
        m = i + 1;
        break;
      }
    }

    int hh = 0;
    int mm = 0;
    int ss = 0;
    sscanf(__TIME__, "%d:%d:%d", &hh, &mm, &ss);

    year = y;
    month = m;
    day = d;
    hour = hh;
    minute = mm;
    second = ss;
  }

  unsigned long elapsedSec = (millis() - baseMillis) / 1000;

  int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  int y = year;
  int m = month;
  int d = day;
  int total = hour * 3600 + minute * 60 + second + elapsedSec;

  while (total >= 86400)
  {
    total -= 86400;
    d++;

    bool leap = ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0));
    daysInMonth[1] = leap ? 29 : 28;

    if (d > daysInMonth[m - 1])
    {
      d = 1;
      m++;
      if (m > 12)
      {
        m = 1;
        y++;
      }
    }
  }

  int hh = total / 3600;
  int mm = (total % 3600) / 60;
  int ss = total % 60;

  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d", y, m, d, hh, mm, ss);
  return String(buf);
}

// =====================================================
// UI helper
// =====================================================
lv_obj_t *makeCard(lv_obj_t *parent, int x, int y, int w, int h, uint32_t color)
{
  lv_obj_t *card = lv_obj_create(parent);
  lv_obj_set_size(card, w, h);
  lv_obj_align(card, LV_ALIGN_TOP_LEFT, x, y);
  lv_obj_set_style_bg_color(card, lv_color_hex(color), 0);
  lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(card, 14, 0);
  lv_obj_set_style_border_width(card, 1, 0);
  lv_obj_set_style_border_color(card, lv_color_hex(0xD8DEE9), 0);
  lv_obj_set_style_pad_all(card, 8, 0);
  // 카드 내부 콘텐츠가 조금 커져도 카드 자체가 스크롤되지 않게 고정합니다.
  lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scrollbar_mode(card, LV_SCROLLBAR_MODE_OFF);
  return card;
}

lv_obj_t *makeLabel(lv_obj_t *parent, const char *text, int x, int y, uint32_t color)
{
  lv_obj_t *label = lv_label_create(parent);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
  lv_obj_set_style_text_font(label, FONT_KR, 0);
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, x, y);
  return label;
}

lv_obj_t *makeSmallLabel(lv_obj_t *parent, const char *text, int x, int y, uint32_t color)
{
  lv_obj_t *label = lv_label_create(parent);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
  lv_obj_set_style_text_font(label, FONT_GRAPH_SMALL, 0);
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, x, y);
  return label;
}

// =====================================================
// Light UI components
// =====================================================

// A plain surface. White on the grey ground carries the grouping on its own,
// the way iOS grouped lists do, so there is no border and no shadow to add
// visual noise.
lv_obj_t *makePanel(lv_obj_t *parent, int x, int y, int w, int h)
{
  lv_obj_t *panel = lv_obj_create(parent);
  lv_obj_set_size(panel, w, h);
  lv_obj_align(panel, LV_ALIGN_TOP_LEFT, x, y);
  lv_obj_set_style_bg_color(panel, lv_color_hex(UI_SURFACE), 0);
  lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(panel, 16, 0);
  lv_obj_set_style_border_width(panel, 0, 0);
  lv_obj_set_style_outline_width(panel, 0, 0);
  lv_obj_set_style_shadow_width(panel, 0, 0);
  lv_obj_set_style_pad_all(panel, 0, 0);
  lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scrollbar_mode(panel, LV_SCROLLBAR_MODE_OFF);
  return panel;
}

lv_obj_t *makeHeading(lv_obj_t *parent, const char *text, int x, int y, uint32_t color)
{
  lv_obj_t *label = lv_label_create(parent);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
  lv_obj_set_style_text_font(label, FONT_KR_HEAD, 0);
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, x, y);
  return label;
}

// Status pill. The caller owns the returned label so the text can change; the
// pill always carries a word, never colour alone.
lv_obj_t *makeChip(lv_obj_t *parent, const char *text, int x, int y, int w,
                   uint32_t bgColor, uint32_t textColor)
{
  lv_obj_t *chip = lv_obj_create(parent);
  lv_obj_set_size(chip, w, 26);
  lv_obj_align(chip, LV_ALIGN_TOP_LEFT, x, y);
  lv_obj_set_style_bg_color(chip, lv_color_hex(bgColor), 0);
  lv_obj_set_style_bg_opa(chip, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(chip, 13, 0);
  lv_obj_set_style_border_width(chip, 0, 0);
  lv_obj_set_style_shadow_width(chip, 0, 0);
  lv_obj_set_style_pad_all(chip, 0, 0);
  lv_obj_clear_flag(chip, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *label = lv_label_create(chip);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_color(label, lv_color_hex(textColor), 0);
  lv_obj_set_style_text_font(label, FONT_KR_SMALL, 0);
  lv_obj_center(label);
  return label;
}

// Filled accent button for the one primary action on a screen.
lv_obj_t *makePrimaryButton(lv_obj_t *parent, const char *text, int x, int y,
                            int w, int h, uint32_t color, lv_event_cb_t cb)
{
  lv_obj_t *btn = lv_btn_create(parent);
  lv_obj_set_size(btn, w, h);
  lv_obj_align(btn, LV_ALIGN_TOP_LEFT, x, y);
  lv_obj_set_style_bg_color(btn, lv_color_hex(color), 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(btn, 12, 0);
  lv_obj_set_style_border_width(btn, 0, 0);
  lv_obj_set_style_shadow_width(btn, 0, 0);
  lv_obj_set_style_pad_all(btn, 0, 0);
  lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *label = lv_label_create(btn);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_color(label, lv_color_hex(UI_SURFACE), 0);
  // Display weight only on the tall, screen-level actions; a compact button
  // wearing 24 px bold reads as shouting.
  lv_obj_set_style_text_font(label, h >= 56 ? FONT_KR_HEAD : FONT_KR, 0);
  lv_obj_center(label);
  return btn;
}

// Quiet button: accent text on a tinted ground, for secondary actions.
lv_obj_t *makeQuietButton(lv_obj_t *parent, const char *text, int x, int y,
                          int w, int h, lv_event_cb_t cb)
{
  lv_obj_t *btn = lv_btn_create(parent);
  lv_obj_set_size(btn, w, h);
  lv_obj_align(btn, LV_ALIGN_TOP_LEFT, x, y);
  lv_obj_set_style_bg_color(btn, lv_color_hex(UI_ACCENT_TINT), 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(btn, 10, 0);
  lv_obj_set_style_border_width(btn, 0, 0);
  lv_obj_set_style_shadow_width(btn, 0, 0);
  lv_obj_set_style_pad_all(btn, 0, 0);
  lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *label = lv_label_create(btn);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_color(label, lv_color_hex(UI_ACCENT), 0);
  lv_obj_set_style_text_font(label, FONT_KR, 0);
  lv_obj_center(label);
  return btn;
}

// Selection styling lives here so it can be re-applied when the active sensor
// changes, rather than being baked in when the tile is built.
void styleSensorTile(lv_obj_t *tile, lv_obj_t *mark, bool selected)
{
  if (tile == NULL) return;

  lv_obj_set_style_bg_color(tile, lv_color_hex(selected ? UI_ACCENT_SOFT : UI_SURFACE), 0);
  lv_obj_set_style_border_width(tile, selected ? 2 : 0, 0);
  lv_obj_set_style_border_color(tile, lv_color_hex(UI_ACCENT), 0);

  if (mark == NULL) return;

  if (selected) lv_obj_clear_flag(mark, LV_OBJ_FLAG_HIDDEN);
  else lv_obj_add_flag(mark, LV_OBJ_FLAG_HIDDEN);
}

// Sensor tile. Named by the quantity it measures, not the part number, and
// large enough that a fingertip cannot reach two of them at once. The caller
// keeps `markOut` so the selected state can be moved later.
lv_obj_t *makeSensorTile(lv_obj_t *parent, const char *name, const char *unit,
                         const char *detail, int x, int y, int w, int h,
                         bool selected, lv_event_cb_t cb, lv_obj_t **markOut)
{
  lv_obj_t *tile = lv_btn_create(parent);
  lv_obj_set_size(tile, w, h);
  lv_obj_align(tile, LV_ALIGN_TOP_LEFT, x, y);
  lv_obj_set_style_bg_opa(tile, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(tile, 16, 0);
  lv_obj_set_style_outline_width(tile, 0, 0);
  lv_obj_set_style_shadow_width(tile, 0, 0);
  lv_obj_set_style_pad_all(tile, 0, 0);
  lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(tile, cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *nameLabel = lv_label_create(tile);
  lv_label_set_text(nameLabel, name);
  lv_obj_set_style_text_color(nameLabel, lv_color_hex(UI_TEXT), 0);
  lv_obj_set_style_text_font(nameLabel, FONT_KR_HEAD, 0);
  lv_obj_align(nameLabel, LV_ALIGN_TOP_LEFT, 18, 16);

  lv_obj_t *detailLabel = lv_label_create(tile);
  lv_label_set_text(detailLabel, detail ? detail : "");
  lv_obj_set_style_text_color(detailLabel, lv_color_hex(UI_TEXT_4), 0);
  lv_obj_set_style_text_font(detailLabel, FONT_KR_SMALL, 0);
  lv_obj_set_width(detailLabel, w - 36);
  lv_label_set_long_mode(detailLabel, LV_LABEL_LONG_CLIP);
  lv_obj_align(detailLabel, LV_ALIGN_TOP_LEFT, 18, 50);

  lv_obj_t *unitLabel = lv_label_create(tile);
  lv_label_set_text(unitLabel, unit);
  lv_obj_set_style_text_color(unitLabel, lv_color_hex(UI_TEXT_3), 0);
  lv_obj_set_style_text_font(unitLabel, FONT_KR_SMALL, 0);
  lv_obj_align(unitLabel, LV_ALIGN_BOTTOM_LEFT, 18, -14);

  lv_obj_t *mark = lv_label_create(tile);
  lv_label_set_text(mark, "선택됨");
  lv_obj_set_style_text_color(mark, lv_color_hex(UI_ACCENT), 0);
  lv_obj_set_style_text_font(mark, FONT_KR_SMALL, 0);
  lv_obj_align(mark, LV_ALIGN_BOTTOM_RIGHT, -18, -14);

  styleSensorTile(tile, mark, selected);

  if (markOut != NULL) *markOut = mark;
  return tile;
}

// Bottom tab bar, identical on every screen so "back" is always in one place.
// Icons come from the Montserrat symbol range; the Korean caption sits below
// them in the Hangul face, so each tab reads without relying on the glyph.
void createTabBar(lv_obj_t *parent, int activeIndex)
{
  static const char *tabIcons[4] = {
    LV_SYMBOL_HOME, LV_SYMBOL_PLAY, LV_SYMBOL_LIST, LV_SYMBOL_SETTINGS
  };
  static const char *tabNames[4] = { "홈", "측정", "기록", "설정" };

  lv_event_cb_t tabCallbacks[4] = {
    go_home_event_cb, go_measure_event_cb, go_csv_event_cb, go_settings_event_cb
  };

  lv_obj_t *bar = lv_obj_create(parent);
  lv_obj_set_size(bar, LCD_H_RES, UI_TABBAR_H);
  lv_obj_align(bar, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_set_style_bg_color(bar, lv_color_hex(UI_SURFACE), 0);
  lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(bar, 0, 0);
  lv_obj_set_style_border_width(bar, 0, 0);
  lv_obj_set_style_pad_all(bar, 0, 0);
  lv_obj_set_style_shadow_width(bar, 0, 0);
  lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

  // Hairline along the top edge only.
  lv_obj_t *hairline = lv_obj_create(bar);
  lv_obj_set_size(hairline, LCD_H_RES, 1);
  lv_obj_align(hairline, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_set_style_bg_color(hairline, lv_color_hex(UI_LINE), 0);
  lv_obj_set_style_bg_opa(hairline, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(hairline, 0, 0);
  lv_obj_set_style_radius(hairline, 0, 0);
  lv_obj_clear_flag(hairline, LV_OBJ_FLAG_CLICKABLE);

  const int tabWidth = LCD_H_RES / 4;

  for (int i = 0; i < 4; i++)
  {
    const bool active = (i == activeIndex);
    const uint32_t tint = active ? UI_ACCENT : UI_TEXT_3;

    lv_obj_t *tab = lv_btn_create(bar);
    lv_obj_set_size(tab, tabWidth, UI_TABBAR_H - 1);
    lv_obj_align(tab, LV_ALIGN_TOP_LEFT, i * tabWidth, 1);
    lv_obj_set_style_bg_opa(tab, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(tab, 0, 0);
    lv_obj_set_style_shadow_width(tab, 0, 0);
    lv_obj_set_style_radius(tab, 0, 0);
    lv_obj_set_style_pad_all(tab, 0, 0);
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);

    // The active tab is already here; tapping it would reload the screen.
    if (!active) lv_obj_add_event_cb(tab, tabCallbacks[i], LV_EVENT_CLICKED, NULL);

    lv_obj_t *icon = lv_label_create(tab);
    lv_label_set_text(icon, tabIcons[i]);
    lv_obj_set_style_text_color(icon, lv_color_hex(tint), 0);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_16, 0);
    lv_obj_align(icon, LV_ALIGN_TOP_MID, 0, 8);

    lv_obj_t *caption = lv_label_create(tab);
    lv_label_set_text(caption, tabNames[i]);
    lv_obj_set_style_text_color(caption, lv_color_hex(tint), 0);
    lv_obj_set_style_text_font(caption, FONT_KR_SMALL, 0);
    lv_obj_align(caption, LV_ALIGN_BOTTOM_MID, 0, -7);
  }
}


lv_obj_t *makeButton(lv_obj_t *parent, const char *text, int x, int y, int w, int h, lv_event_cb_t cb)
{
  lv_obj_t *btn = lv_btn_create(parent);
  lv_obj_set_size(btn, w, h);
  lv_obj_align(btn, LV_ALIGN_TOP_LEFT, x, y);

  lv_obj_set_style_radius(btn, 10, 0);
  lv_obj_set_style_bg_color(btn, lv_color_hex(0x2563EB), 0);
  lv_obj_set_style_bg_color(btn, lv_color_hex(0x1D4ED8), LV_STATE_PRESSED);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);

  lv_obj_set_style_border_width(btn, 0, 0);
  lv_obj_set_style_shadow_width(btn, 0, 0);
  lv_obj_set_style_pad_all(btn, 0, 0);

  lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *label = lv_label_create(btn);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_font(label, FONT_KR, 0);
  lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_center(label);

  return btn;
}

lv_obj_t *makeArrowButton(lv_obj_t *parent, const char *text, int x, int y, int w, int h, lv_event_cb_t cb)
{
  lv_obj_t *btn = lv_btn_create(parent);
  lv_obj_set_size(btn, w, h);
  lv_obj_align(btn, LV_ALIGN_TOP_LEFT, x, y);

  lv_obj_set_style_radius(btn, 14, 0);
  lv_obj_set_style_bg_color(btn, lv_color_hex(0xF8FAFC), 0);
  lv_obj_set_style_bg_color(btn, lv_color_hex(0xE5E7EB), LV_STATE_PRESSED);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);

  lv_obj_set_style_border_width(btn, 1, 0);
  lv_obj_set_style_border_color(btn, lv_color_hex(0xCBD5E1), 0);
  lv_obj_set_style_shadow_width(btn, 0, 0);
  lv_obj_set_style_pad_all(btn, 0, 0);
  lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *label = lv_label_create(btn);
  lv_label_set_text(label, text);

  // LVGL 기본 심볼 폰트를 사용해 화살표가 깨지지 않게 표시
  lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(label, lv_color_hex(0x111827), 0);
  lv_obj_center(label);

  return btn;
}

void createStatusBar(
  lv_obj_t *parent,
  const char *title,
  lv_obj_t **timeLabel,
  lv_obj_t **wifiLabel,
  lv_obj_t **sdLabel
)
{
  lv_obj_t *bar = lv_obj_create(parent);
  lv_obj_set_size(bar, LCD_H_RES, UI_STATUSBAR_H);
  lv_obj_align(bar, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_set_style_bg_color(bar, lv_color_hex(UI_SURFACE), 0);
  lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(bar, 0, 0);
  lv_obj_set_style_radius(bar, 0, 0);
  lv_obj_set_style_shadow_width(bar, 0, 0);
  lv_obj_set_style_pad_all(bar, 0, 0);
  lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

  // Hairline along the bottom edge separates the bar from the page.
  lv_obj_t *hairline = lv_obj_create(bar);
  lv_obj_set_size(hairline, LCD_H_RES, 1);
  lv_obj_align(hairline, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_set_style_bg_color(hairline, lv_color_hex(UI_LINE), 0);
  lv_obj_set_style_bg_opa(hairline, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(hairline, 0, 0);
  lv_obj_set_style_radius(hairline, 0, 0);
  lv_obj_clear_flag(hairline, LV_OBJ_FLAG_CLICKABLE);

  lv_obj_t *titleLabel = lv_label_create(bar);
  lv_label_set_text(titleLabel, title);
  lv_obj_set_style_text_color(titleLabel, lv_color_hex(UI_TEXT), 0);
  lv_obj_set_style_text_font(titleLabel, FONT_KR, 0);
  lv_obj_align(titleLabel, LV_ALIGN_LEFT_MID, 24, 0);

  // Clock centred; connection state pinned to the right edge.
  *timeLabel = lv_label_create(bar);
  lv_label_set_text(*timeLabel, "--:--");
  lv_obj_set_style_text_color(*timeLabel, lv_color_hex(UI_TEXT), 0);
  lv_obj_set_style_text_font(*timeLabel, FONT_KR, 0);
  lv_obj_align(*timeLabel, LV_ALIGN_CENTER, 0, 0);

  // Connection state reads as two glyphs that go green when live. The text
  // never changes after this point; updateStatusBars() only recolours them.
  *sdLabel = lv_label_create(bar);
  lv_label_set_text(*sdLabel, LV_SYMBOL_SD_CARD);
  lv_obj_set_style_text_font(*sdLabel, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(*sdLabel, lv_color_hex(UI_TEXT_3), 0);
  lv_obj_align(*sdLabel, LV_ALIGN_RIGHT_MID, -24, 0);

  *wifiLabel = lv_label_create(bar);
  lv_label_set_text(*wifiLabel, LV_SYMBOL_WIFI);
  lv_obj_set_style_text_font(*wifiLabel, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(*wifiLabel, lv_color_hex(UI_TEXT_3), 0);
  lv_obj_align(*wifiLabel, LV_ALIGN_RIGHT_MID, -64, 0);
}

// Green when the thing is actually working, muted grey otherwise.
static void setStatusIconState(lv_obj_t *icon, bool active)
{
  if (icon == NULL) return;
  lv_obj_set_style_text_color(icon, lv_color_hex(active ? UI_OK : UI_TEXT_3), 0);
}

void updateStatusBars()
{
  String nowText = getCurrentDateTimeText();
  String shortTime = nowText.substring(0, 16);

  if (labelBarHomeTime) lv_label_set_text(labelBarHomeTime, shortTime.c_str());
  if (labelBarMeasureTime) lv_label_set_text(labelBarMeasureTime, shortTime.c_str());
  if (labelBarSettingsTime) lv_label_set_text(labelBarSettingsTime, shortTime.c_str());
  if (labelBarIslTime) lv_label_set_text(labelBarIslTime, shortTime.c_str());
  if (labelBarCsvTime) lv_label_set_text(labelBarCsvTime, shortTime.c_str());
  if (labelBarFileViewerTime) lv_label_set_text(labelBarFileViewerTime, shortTime.c_str());

  setStatusIconState(labelBarHomeWifi, wifiConnected);
  setStatusIconState(labelBarMeasureWifi, wifiConnected);
  setStatusIconState(labelBarSettingsWifi, wifiConnected);
  setStatusIconState(labelBarIslWifi, wifiConnected);
  setStatusIconState(labelBarCsvWifi, wifiConnected);
  setStatusIconState(labelBarFileViewerWifi, wifiConnected);

  const bool sdActive = sdReady && csvLoggingEnabled;
  setStatusIconState(labelBarHomeSd, sdActive);
  setStatusIconState(labelBarMeasureSd, sdActive);
  setStatusIconState(labelBarSettingsSd, sdActive);
  setStatusIconState(labelBarIslSd, sdActive);
  setStatusIconState(labelBarCsvSd, sdActive);
  setStatusIconState(labelBarFileViewerSd, sdActive);
}

lv_obj_t *makeInfoLabel(lv_obj_t *parent, const char *text, int w)
{
  lv_obj_t *label = lv_label_create(parent);
  lv_label_set_text(label, text);
  lv_obj_set_width(label, w);
  lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_font(label, FONT_KR_NORMAL, 0);
  lv_obj_set_style_text_color(label, lv_color_hex(0x1F2937), 0);
  return label;
}


// =====================================================
// Dashboard CSV / ISL helpers
// =====================================================
void sanitizeCsvFileName(const char *input, char *out, size_t outSize)
{
  if (out == NULL || outSize == 0) return;
  out[0] = '\0';

  if (input == NULL) return;

  size_t j = 0;
  for (size_t i = 0; input[i] != '\0' && j < outSize - 1; i++)
  {
    char c = input[i];

    if ((c >= 'A' && c <= 'Z') ||
        (c >= 'a' && c <= 'z') ||
        (c >= '0' && c <= '9') ||
        c == '_' || c == '-' || c == '.')
    {
      out[j++] = c;
    }
  }

  out[j] = '\0';

  if (strlen(out) == 0)
  {
    strncpy(out, CSV_DEFAULT_FILE, outSize - 1);
    out[outSize - 1] = '\0';
  }

  // 확장자가 없으면 .csv 추가
  if (strstr(out, ".csv") == NULL && strstr(out, ".CSV") == NULL)
  {
    size_t len = strlen(out);
    if (len + 4 < outSize)
    {
      strcat(out, ".csv");
    }
  }
}

void updateCsvPath()
{
  // Drop any handle still pointing at the previous file, otherwise logging
  // would keep appending to the old name after the user renames the file.
  closeCsvFile();
  snprintf(csvPath, sizeof(csvPath), "%s/%s", MOUNT_POINT, csvFileName);
}

void saveCsvFileNameToNvs(const char *fileName)
{
  if (fileName == NULL || strlen(fileName) == 0) return;

  nvs_handle_t handle;
  esp_err_t ret = nvs_open("csv_cfg", NVS_READWRITE, &handle);
  if (ret != ESP_OK) return;

  nvs_set_str(handle, "filename", fileName);
  nvs_commit(handle);
  nvs_close(handle);
}

void loadCsvFileNameFromNvs()
{
  nvs_handle_t handle;
  esp_err_t ret = nvs_open("csv_cfg", NVS_READONLY, &handle);
  if (ret != ESP_OK)
  {
    updateCsvPath();
    return;
  }

  char name[sizeof(csvFileName)] = {0};
  size_t len = sizeof(name);
  ret = nvs_get_str(handle, "filename", name, &len);
  nvs_close(handle);

  if (ret == ESP_OK && strlen(name) > 0)
  {
    sanitizeCsvFileName(name, csvFileName, sizeof(csvFileName));
  }

  updateCsvPath();
}

void updateCsvFileNameFromDashboard()
{
  char name[sizeof(csvFileName)];

  if (homeCsvFileTa)
  {
    sanitizeCsvFileName(lv_textarea_get_text(homeCsvFileTa), name, sizeof(name));
  }
  else
  {
    sanitizeCsvFileName(csvFileName, name, sizeof(name));
  }

  strncpy(csvFileName, name, sizeof(csvFileName) - 1);
  csvFileName[sizeof(csvFileName) - 1] = '\0';
  updateCsvPath();
  saveCsvFileNameToNvs(csvFileName);

  if (homeCsvFileTa) lv_textarea_set_text(homeCsvFileTa, csvFileName);
}

void updateDashboardCsvLabels()
{
  if (labelHomeCsvPath)
  {
    char line[180];
    snprintf(line, sizeof(line), "파일: %s", csvPath);
    lv_label_set_text(labelHomeCsvPath, line);
  }

  if (labelHomeCsv)
  {
    if (sdReady && csvLoggingEnabled)
    {
      lv_label_set_text(labelHomeCsv, "CSV: 자동 저장 중");
    }
    else if (sdReady)
    {
      lv_label_set_text(labelHomeCsv, "CSV: SD 준비됨");
    }
    else if (csvLoggingEnabled)
    {
      lv_label_set_text(labelHomeCsv, "CSV: SD 필요");
    }
    else
    {
      lv_label_set_text(labelHomeCsv, "CSV: 수동 저장 모드");
    }
  }
}

void updateDashboardCsvPreview()
{
  if (!labelHomeCsvPreview) return;

  char text[512];
  text[0] = '\0';

  if (sampleCount <= 0)
  {
    snprintf(text, sizeof(text), "최근 데이터 없음\n파일명과 SD 상태를 확인하세요.");
    lv_label_set_text(labelHomeCsvPreview, text);
    return;
  }

  char header[120];
  if (activeSensorHasPressure())
  {
    snprintf(
      header,
      sizeof(header),
      "최근 데이터\nNo  t(s)  %s(%s)  %s(%s)\n",
      activePrimaryName(),
      activePrimaryUnit(),
      activeSecondaryName(),
      activeSecondaryUnit()
    );
  }
  else
  {
    snprintf(
      header,
      sizeof(header),
      "최근 데이터\nNo  t(s)  %s(%s)\n",
      activePrimaryName(),
      activePrimaryUnit()
    );
  }
  strncat(text, header, sizeof(text) - strlen(text) - 1);

  int start = sampleCount - 6;
  if (start < 0) start = 0;

  char line[80];
  for (int i = sampleCount - 1; i >= start; i--)
  {
    if (activeSensorHasPressure() && pressureValueValid(pressureHistory[i]))
    {
      snprintf(
        line,
        sizeof(line),
        "%d  %lu  %.2f  %.1f\n",
        noHistory[i] > 0 ? noHistory[i] : (i + 1),
        (unsigned long)timeHistory[i],
        tempHistory[i],
        pressureHistory[i]
      );
    }
    else
    {
      snprintf(
        line,
        sizeof(line),
        "%d  %lu  %.2f\n",
        noHistory[i] > 0 ? noHistory[i] : (i + 1),
        (unsigned long)timeHistory[i],
        tempHistory[i]
      );
    }
    strncat(text, line, sizeof(text) - strlen(text) - 1);
  }

  lv_label_set_text(labelHomeCsvPreview, text);
}

void updateDashboardIslLabels()
{
  if (labelHomeIsl) lv_label_set_text(labelHomeIsl, islStatusText);
  if (labelSettingsIsl) lv_label_set_text(labelSettingsIsl, islStatusText);

  updateCloudModeLabel();

  // 모둠코드 입력칸은 사용자가 수정 중일 수 있으므로 상태 갱신 때 강제로 덮어쓰지 않는다.
  // 저장/전송 버튼을 눌렀을 때만 updateIslModuleCodeFromUi()에서 값을 확정한다.
}

const char *cloudUploadModeName()
{
  return cloudUploadMode == CLOUD_UPLOAD_BATCH ? "일괄전송" : "실시간";
}

bool isBatchUploading()
{
  // 요청 접수 후 WiFi task가 실제 작업을 시작하기 전까지도
  // 중복 클릭/센서 전환을 막기 위해 pending 요청을 전송 중으로 취급합니다.
  return batchUploading || batchUploadRequested;
}

void requestBatchUiRefresh()
{
  batchUiDirty = true;
  islStatusUiDirty = true;
}

void updateCloudModeLabel()
{
  if (!labelCloudMode) return;

  char text[180];
  if (batchUploading)
  {
    snprintf(
      text,
      sizeof(text),
      "방식: %s / 저장 %d개 / 전송중 %d/%d",
      cloudUploadModeName(),
      sampleCount,
      batchUploadProgressSent,
      batchUploadProgressTotal
    );
  }
  else
  {
    snprintf(
      text,
      sizeof(text),
      "방식: %s / 저장 %d개 / 일괄전송 %d/%d / 실시간간격 %d초",
      cloudUploadModeName(),
      sampleCount,
      batchUploadProgressSent,
      batchUploadProgressTotal,
      CLOUD_SEND_EVERY_N_SAMPLES
    );
  }

  lv_label_set_text(labelCloudMode, text);
}

void serviceLightweightBatchUi()
{
  unsigned long now = millis();

  // Too-frequent label updates can disturb display/touch/HTTP timing on this board.
  if (!batchUiDirty && !islStatusUiDirty) return;
  if (now - lastBatchUiServiceMs < 250) return;

  lastBatchUiServiceMs = now;
  batchUiDirty = false;
  islStatusUiDirty = false;

  updateDashboardIslLabels();

  if (labelStatus && batchUploading)
  {
    char statusText[80];
    snprintf(
      statusText,
      sizeof(statusText),
      "일괄전송 %d/%d",
      batchUploadProgressSent,
      batchUploadProgressTotal
    );
    lv_label_set_text(labelStatus, statusText);
  }
}

static void dashboard_textarea_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_FOCUSED) return;

  lv_obj_t *keyboard = NULL;

  if (lv_scr_act() == islScreen)
  {
    keyboard = islKeyboard;
  }
  else
  {
    keyboard = homeKeyboard;
  }

  if (keyboard)
  {
    lv_keyboard_set_textarea(keyboard, lv_event_get_target(e));
    lv_obj_clear_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
  }

}

static void dashboard_keyboard_event_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL)
  {
    lv_obj_t *keyboard = lv_event_get_target(e);

    if (keyboard)
    {
      lv_obj_add_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
      lv_keyboard_set_textarea(keyboard, NULL);
    }

  }
}

static void copy_label_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  static lv_obj_t *lastObj = NULL;
  static unsigned long lastClickMs = 0;

  lv_obj_t *target = lv_event_get_target(e);
  unsigned long now = millis();

  if (target == lastObj && now - lastClickMs < 500)
  {
    const char *text = lv_label_get_text(target);

    if (text)
    {
      strncpy(uiCopiedText, text, sizeof(uiCopiedText) - 1);
      uiCopiedText[sizeof(uiCopiedText) - 1] = '\0';

      Serial.print("UI COPY TEXT: ");
      Serial.println(uiCopiedText);

      if (labelUiCopyStatus)
      {
        lv_label_set_text(labelUiCopyStatus, "복사됨");
      }
    }
  }

  lastObj = target;
  lastClickMs = now;
}

void enableCopyOnDoubleClick(lv_obj_t *label)
{
  if (label == NULL) return;

  lv_obj_add_flag(label, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(label, copy_label_event_cb, LV_EVENT_CLICKED, NULL);
}

static void csv_textarea_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_FOCUSED) return;

  if (csvKeyboard)
  {
    lv_keyboard_set_textarea(csvKeyboard, lv_event_get_target(e));
    lv_obj_clear_flag(csvKeyboard, LV_OBJ_FLAG_HIDDEN);
  }
}

static void csv_keyboard_event_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL)
  {
    if (csvKeyboard)
    {
      lv_obj_add_flag(csvKeyboard, LV_OBJ_FLAG_HIDDEN);
      lv_keyboard_set_textarea(csvKeyboard, NULL);
    }
  }
}

// =====================================================
// Table
// =====================================================


// Both the table rebuild and the sensor-change refresh used to spell these
// headings out, and they disagreed: whichever ran last won. One writer.
void applyTableValueHeaders()
{
  if (tableData == NULL) return;

  char header[40];

  snprintf(header, sizeof(header), "%s(%s)", activePrimaryName(), activePrimaryUnit());
  lv_table_set_cell_value(tableData, 0, 2, header);

  const int valueCount = activeSensorValueCount();

  if (valueCount >= 3)
  {
    const char *caption2 = "";
    const char *caption3 = "";
    const char *unit2 = "";
    const char *unit3 = "";
    measureValueMeta(1, &caption2, &unit2);
    measureValueMeta(2, &caption3, &unit3);
    snprintf(header, sizeof(header), "%s/%s", caption2, caption3);
    lv_table_set_cell_value(tableData, 0, 3, header);
  }
  else if (valueCount == 2)
  {
    const char *caption2 = "";
    const char *unit2 = "";
    measureValueMeta(1, &caption2, &unit2);
    snprintf(header, sizeof(header), "%s(%s)", caption2, unit2);
    lv_table_set_cell_value(tableData, 0, 3, header);
  }
  else
  {
    lv_table_set_cell_value(tableData, 0, 3, "-");
  }
}

void resetTable()
{
  tablePageOffset = 0;

  // The table lives on whichever screen chooses to build it; the measurement
  // screen no longer does.
  if (tableData == NULL) return;

  lv_table_set_cell_value(tableData, 0, 0, "No");
  lv_table_set_cell_value(tableData, 0, 1, "시간(s)");

  applyTableValueHeaders();

  for (int r = 1; r <= TABLE_VISIBLE_ROWS; r++)
  {
    lv_table_set_cell_value(tableData, r, 0, "-");
    lv_table_set_cell_value(tableData, r, 1, "-");
    lv_table_set_cell_value(tableData, r, 2, "-");
    lv_table_set_cell_value(tableData, r, 3, "-");
  }
}

void updateTable()
{
  if (tableData == NULL) return;

  char text[32];

  // Which slice of the run is on screen, so paging has a reference point.
  if (labelCsvRowRange)
  {
    char range[48];

    if (sampleCount <= 0)
    {
      snprintf(range, sizeof(range), "측정값 없음");
    }
    else
    {
      const int first = tablePageOffset + 1;
      int last = tablePageOffset + TABLE_VISIBLE_ROWS;
      if (last > sampleCount) last = sampleCount;
      snprintf(range, sizeof(range), "%d-%d / %d", first, last, sampleCount);
    }

    lv_label_set_text(labelCsvRowRange, range);
  }

  int maxOffset = 0;

  if (sampleCount > 0)
  {
    maxOffset = ((sampleCount - 1) / TABLE_VISIBLE_ROWS) * TABLE_VISIBLE_ROWS;
  }

  if (tablePageOffset > maxOffset)
  {
    tablePageOffset = maxOffset;
  }

  if (tablePageOffset < 0)
  {
    tablePageOffset = 0;
  }

  for (int r = 1; r <= TABLE_VISIBLE_ROWS; r++)
  {
    int idx = sampleCount - 1 - tablePageOffset - (r - 1);

    if (idx >= 0)
    {
      int displayNo = noHistory[idx] > 0 ? noHistory[idx] : (idx + 1);
      snprintf(text, sizeof(text), "%d", displayNo);
      lv_table_set_cell_value(tableData, r, 0, text);

      // 시간 옆 s 제거
      snprintf(text, sizeof(text), "%lu", (unsigned long)timeHistory[idx]);
      lv_table_set_cell_value(tableData, r, 1, text);

      formatPrimaryValueText(text, sizeof(text), tempHistory[idx], false);
      lv_table_set_cell_value(tableData, r, 2, text);

      // %g so one column can hold a bus voltage, a pressure and an angle
      // without either dropping the INA228's milliamps or padding hPa with
      // decimals it does not have.
      const int extraValues = activeSensorValueCount();
      const bool secondUsable =
        extraValues >= 2 && !isnan(pressureHistory[idx]) &&
        (!activeSensorHasPressure() || pressureValueValid(pressureHistory[idx]));

      if (extraValues >= 3 && secondUsable && !isnan(humidityHistory[idx]))
      {
        snprintf(text, sizeof(text), "%.4g/%.4g", pressureHistory[idx], humidityHistory[idx]);
        lv_table_set_cell_value(tableData, r, 3, text);
      }
      else if (secondUsable)
      {
        snprintf(text, sizeof(text), "%.4g", pressureHistory[idx]);
        lv_table_set_cell_value(tableData, r, 3, text);
      }
      else
      {
        lv_table_set_cell_value(tableData, r, 3, "-");
      }
    }
    else
    {
      lv_table_set_cell_value(tableData, r, 0, "-");
      lv_table_set_cell_value(tableData, r, 1, "-");
      lv_table_set_cell_value(tableData, r, 2, "-");
      lv_table_set_cell_value(tableData, r, 3, "-");
    }
  }
}

static void table_newer_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  tablePageOffset -= TABLE_VISIBLE_ROWS;

  if (tablePageOffset < 0)
  {
    tablePageOffset = 0;
  }

  updateTable();
}

static void table_older_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  int maxOffset = 0;

  if (sampleCount > 0)
  {
    maxOffset = ((sampleCount - 1) / TABLE_VISIBLE_ROWS) * TABLE_VISIBLE_ROWS;
  }

  tablePageOffset += TABLE_VISIBLE_ROWS;

  if (tablePageOffset > maxOffset)
  {
    tablePageOffset = maxOffset;
  }

  updateTable();
}

// =====================================================
// Graph
// =====================================================
void clearChart()
{
  updateActiveSensorUiLabels();
  // 고정 포인트 수를 유지하여 측정 중 LVGL 메모리 재할당을 막는다.
  lv_chart_set_point_count(chart, CHART_POINTS);
  lv_chart_set_all_value(chart, seriesTemp, LV_CHART_POINT_NONE);
  lv_chart_set_all_value(chart, seriesPressure, LV_CHART_POINT_NONE);
  lv_chart_refresh(chart);

  if (labelGraphStart) lv_label_set_text(labelGraphStart, "시간(s)");
  if (labelGraphEnd) lv_label_set_text(labelGraphEnd, "");

  bool pressureAxisOn = activeSensorHasPressure();

  if (labelMeasurePressureAxisTitle)
  {
    if (pressureAxisOn) lv_obj_clear_flag(labelMeasurePressureAxisTitle, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(labelMeasurePressureAxisTitle, LV_OBJ_FLAG_HIDDEN);
  }

  for (int i = 0; i < 6; i++)
  {
    if (labelMeasureTempTicks[i]) lv_label_set_text(labelMeasureTempTicks[i], "--");
    if (labelMeasurePressureTicks[i])
    {
      if (pressureAxisOn)
      {
        lv_obj_clear_flag(labelMeasurePressureTicks[i], LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(labelMeasurePressureTicks[i], "--");
      }
      else
      {
        lv_obj_add_flag(labelMeasurePressureTicks[i], LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(labelMeasurePressureTicks[i], "");
      }
    }
    if (labelMeasureTimeTicks[i]) lv_label_set_text(labelMeasureTimeTicks[i], "--");
  }
}

// Sensors that report three quantities keep the third in a global, because
// readActiveSensor() only returns two.
static float thirdValueForActiveSensor()
{
  if (activeSensorMode == SENSOR_MODE_SCD41) return scd41LastHumidityPct;
  if (activeSensorMode == SENSOR_MODE_INA228) return ina228LastPowerW;
  return NAN;
}

void addSample(uint32_t timeS, float tempC, float pressureHpa)
{
  String collectText = getCurrentDateTimeText();
  int currentNo = measurementCount;
  if (currentNo <= 0) currentNo = sampleCount + 1;

  if (sampleCount < MAX_SAMPLES)
  {
    noHistory[sampleCount] = currentNo;
    tempHistory[sampleCount] = tempC;
    pressureHistory[sampleCount] = pressureHpa;
    humidityHistory[sampleCount] = thirdValueForActiveSensor();
    timeHistory[sampleCount] = timeS;
    sampleEnabled[sampleCount] = true;
    sampleCount++;
  }
  else
  {
    // MAX_SAMPLES를 넘으면 화면/일괄전송용 버퍼는 최근 MAX_SAMPLES개만 보관한다.
    // 단, noHistory에는 전체 측정 순번을 저장하므로 No는 MAX_SAMPLES에서 멈추지 않고 계속 증가한다.
    for (int i = 1; i < MAX_SAMPLES; i++)
    {
      noHistory[i - 1] = noHistory[i];
      tempHistory[i - 1] = tempHistory[i];
      pressureHistory[i - 1] = pressureHistory[i];
      humidityHistory[i - 1] = humidityHistory[i];
      timeHistory[i - 1] = timeHistory[i];
      sampleEnabled[i - 1] = sampleEnabled[i];
    }

    noHistory[MAX_SAMPLES - 1] = currentNo;
    tempHistory[MAX_SAMPLES - 1] = tempC;
    pressureHistory[MAX_SAMPLES - 1] = pressureHpa;
    humidityHistory[MAX_SAMPLES - 1] = thirdValueForActiveSensor();
    timeHistory[MAX_SAMPLES - 1] = timeS;
    sampleEnabled[MAX_SAMPLES - 1] = true;
  }
}


bool saveMeasurementBackupToNvs(bool force)
{
  // 안정화: 대용량 측정 버퍼를 NVS에 주기적으로 저장하면 화면 재부팅/WDT 원인이 될 수 있어 비활성화합니다.
  // 측정 데이터는 RAM history와 CSV/Cloud 전송을 사용합니다.
  (void)force;
  return true;

  if (sampleCount <= 0 && !force) return true;
  if (!force && measurementCount == lastBackupMeasurementCount) return true;
  if (!force && MEAS_BACKUP_EVERY_SAMPLES > 1 && (measurementCount % MEAS_BACKUP_EVERY_SAMPLES) != 0) return true;
  if (!force && (millis() - lastBackupWriteMs < MEAS_BACKUP_MIN_INTERVAL_MS)) return true;

  MeasurementBackupBlob *backupPtr = measurementBackupBuffer();
  if (backupPtr == NULL) return false;
  MeasurementBackupBlob &backup = *backupPtr;
  memset(&backup, 0, sizeof(backup));
  backup.magic = MEAS_BACKUP_MAGIC;
  backup.version = MEAS_BACKUP_VERSION;
  backup.storedSampleCount = sampleCount;
  backup.storedMeasurementCount = measurementCount;
  backup.storedElapsedMs = measureAccumulatedMs;
  if (measureClockRunning) backup.storedElapsedMs += millis() - measureResumeMs;
  backup.storedActiveSensorMode = activeSensorMode;

  for (int i = 0; i < sampleCount && i < MAX_SAMPLES; i++)
  {
    backup.temp[i] = tempHistory[i];
    backup.pressure[i] = pressureHistory[i];
    backup.humidity[i] = humidityHistory[i];
    backup.timeS[i] = timeHistory[i];
    backup.no[i] = noHistory[i];
    backup.enabled[i] = sampleEnabled[i] ? 1 : 0;
  }

  nvs_handle_t handle;
  esp_err_t ret = nvs_open("meas_bak", NVS_READWRITE, &handle);
  if (ret != ESP_OK) return false;
  ret = nvs_set_blob(handle, "data", &backup, sizeof(backup));
  if (ret == ESP_OK) ret = nvs_commit(handle);
  nvs_close(handle);

  if (ret == ESP_OK)
  {
    lastBackupMeasurementCount = measurementCount;
    lastBackupWriteMs = millis();
    return true;
  }
  return false;
}

bool restoreMeasurementBackupFromNvs()
{
  nvs_handle_t handle;
  esp_err_t ret = nvs_open("meas_bak", NVS_READONLY, &handle);
  if (ret != ESP_OK) return false;

  MeasurementBackupBlob *backupPtr = measurementBackupBuffer();
  if (backupPtr == NULL) return false;
  MeasurementBackupBlob &backup = *backupPtr;
  size_t len = sizeof(backup);
  ret = nvs_get_blob(handle, "data", &backup, &len);
  nvs_close(handle);

  if (ret != ESP_OK || len != sizeof(backup)) return false;
  if (backup.magic != MEAS_BACKUP_MAGIC || backup.version != MEAS_BACKUP_VERSION) return false;
  if (backup.storedSampleCount < 0 || backup.storedSampleCount > MAX_SAMPLES) return false;

  sampleCount = backup.storedSampleCount;
  measurementCount = backup.storedMeasurementCount;
  measureAccumulatedMs = backup.storedElapsedMs;
  measureResumeMs = 0;
  measureClockRunning = false;
  activeSensorMode = backup.storedActiveSensorMode;
  if (activeSensorMode != SENSOR_MODE_DPS310 &&
      activeSensorMode != SENSOR_MODE_DS18B20 &&
      activeSensorMode != SENSOR_MODE_SCD41 &&
      activeSensorMode != SENSOR_MODE_TSL2591 &&
      activeSensorMode != SENSOR_MODE_TMP117 &&
      activeSensorMode != SENSOR_MODE_VL53L1X)
  {
    activeSensorMode = ACTIVE_SENSOR_DEFAULT;
  }

  for (int i = 0; i < sampleCount; i++)
  {
    tempHistory[i] = backup.temp[i];
    pressureHistory[i] = backup.pressure[i];
    humidityHistory[i] = backup.humidity[i];
    timeHistory[i] = backup.timeS[i];
    noHistory[i] = backup.no[i] > 0 ? backup.no[i] : (i + 1);
    sampleEnabled[i] = backup.enabled[i] != 0;
  }
  for (int i = sampleCount; i < MAX_SAMPLES; i++)
  {
    sampleEnabled[i] = true;
    humidityHistory[i] = NAN;
  }

  tablePageOffset = 0;
  restoredSampleCount = sampleCount;
  measurementBackupRestored = sampleCount > 0;
  lastBackupMeasurementCount = measurementCount;
  return measurementBackupRestored;
}

void clearMeasurementBackupFromNvs()
{
  nvs_handle_t handle;
  esp_err_t ret = nvs_open("meas_bak", NVS_READWRITE, &handle);
  if (ret == ESP_OK)
  {
    nvs_erase_key(handle, "data");
    nvs_commit(handle);
    nvs_close(handle);
  }
  measurementBackupRestored = false;
  restoredSampleCount = 0;
  lastBackupMeasurementCount = -1;
}

void applyRestoredMeasurementToUi()
{
  if (!measurementBackupRestored || sampleCount <= 0) return;

  char text[64];
  int idx = sampleCount - 1;

  if (labelTempBig)
  {
    formatPrimaryValueText(text, sizeof(text), tempHistory[idx], true);
    lv_label_set_text(labelTempBig, text);
  }

  if (labelPressureBig)
  {
    if (activeSensorMode == SENSOR_MODE_SCD41 && !isnan(pressureHistory[idx]))
    {
      snprintf(text, sizeof(text), "온도: %.1f℃", pressureHistory[idx]);
      lv_label_set_text(labelPressureBig, text);
      lv_obj_clear_flag(labelPressureBig, LV_OBJ_FLAG_HIDDEN);
    }
    else if (activeSensorHasPressure() && pressureValueValid(pressureHistory[idx]))
    {
      snprintf(text, sizeof(text), "압력: %.1fhPa", pressureHistory[idx]);
      lv_label_set_text(labelPressureBig, text);
      lv_obj_clear_flag(labelPressureBig, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
      lv_label_set_text(labelPressureBig, "");
      lv_obj_add_flag(labelPressureBig, LV_OBJ_FLAG_HIDDEN);
    }
  }

  if (labelHumidityBig)
  {
    if (activeSensorMode == SENSOR_MODE_SCD41 && !isnan(humidityHistory[idx]))
    {
      snprintf(text, sizeof(text), "습도: %.1f%%", humidityHistory[idx]);
      lv_label_set_text(labelHumidityBig, text);
      lv_obj_clear_flag(labelHumidityBig, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
      lv_label_set_text(labelHumidityBig, "");
      lv_obj_add_flag(labelHumidityBig, LV_OBJ_FLAG_HIDDEN);
    }
  }

  if (labelCount)
  {
    snprintf(text, sizeof(text), " %d", measurementCount);
    lv_label_set_text(labelCount, text);
  }

  updateTable();
  updateChartAutoScale();
  updateDashboardCsvPreview();
  refreshHomeSensorLabels();
}

void formatMeasureTimeTick(uint32_t sec, char *buf, size_t len)
{
  if (sec < 60)
  {
    snprintf(buf, len, "%lus", (unsigned long)sec);
  }
  else if (sec < 3600)
  {
    snprintf(buf, len, "%lum%02lu", (unsigned long)(sec / 60), (unsigned long)(sec % 60));
  }
  else
  {
    snprintf(buf, len, "%luh%02lu", (unsigned long)(sec / 3600), (unsigned long)((sec % 3600) / 60));
  }
}

static float median3f(float a, float b, float c)
{
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

static float graphPrimaryAt(int idx)
{
  if (idx < 0) idx = 0;
  if (idx >= sampleCount) idx = sampleCount - 1;
  if (idx < 0) return 0.0f;

  // 조도는 순간적인 I2C/광원 흔들림 한 점 때문에 축 전체가 무너지는 것을 막기 위해
  // 그래프에만 3점 중앙값을 적용한다. 표와 CSV에는 원본값을 그대로 보존한다.
  if (activeSensorMode == SENSOR_MODE_TSL2591 && sampleCount >= 3)
  {
    int i0 = idx > 0 ? idx - 1 : idx;
    int i2 = idx + 1 < sampleCount ? idx + 1 : idx;
    return median3f(tempHistory[i0], tempHistory[idx], tempHistory[i2]);
  }

  return tempHistory[idx];
}

static float graphPrimaryInterpolated(float sourcePosition)
{
  if (sampleCount <= 0) return 0.0f;
  if (sampleCount == 1) return graphPrimaryAt(0);

  if (sourcePosition < 0.0f) sourcePosition = 0.0f;
  float maxPosition = (float)(sampleCount - 1);
  if (sourcePosition > maxPosition) sourcePosition = maxPosition;

  int left = (int)floorf(sourcePosition);
  int right = left + 1;
  if (right >= sampleCount) right = sampleCount - 1;
  float fraction = sourcePosition - (float)left;

  float a = graphPrimaryAt(left);
  float b = graphPrimaryAt(right);
  return a + (b - a) * fraction;
}

static float graphSecondaryInterpolated(float sourcePosition)
{
  if (sampleCount <= 0) return NO_PRESSURE_VALUE;
  if (sampleCount == 1) return pressureHistory[0];

  if (sourcePosition < 0.0f) sourcePosition = 0.0f;
  float maxPosition = (float)(sampleCount - 1);
  if (sourcePosition > maxPosition) sourcePosition = maxPosition;

  int left = (int)floorf(sourcePosition);
  int right = left + 1;
  if (right >= sampleCount) right = sampleCount - 1;
  float fraction = sourcePosition - (float)left;

  float a = pressureHistory[left];
  float b = pressureHistory[right];
  if (!pressureValueValid(a)) return b;
  if (!pressureValueValid(b)) return a;
  return a + (b - a) * fraction;
}

void updateChartAutoScale()
{
  if (sampleCount <= 0 || chart == NULL || seriesTemp == NULL || seriesPressure == NULL) return;

  const bool pressureAxisOn = activeSensorHasPressure();

  if (labelMeasurePressureAxisTitle)
  {
    if (pressureAxisOn) lv_obj_clear_flag(labelMeasurePressureAxisTitle, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(labelMeasurePressureAxisTitle, LV_OBJ_FLAG_HIDDEN);
  }

  // point_count는 UI 생성/초기화 때 한 번 CHART_POINTS로 고정한다.
  // 측정 루프에서는 절대 다시 설정하지 않아 내부 버퍼 재할당을 막는다.

  float primaryMin = graphPrimaryAt(0);
  float primaryMax = primaryMin;
  bool hasPressure = false;
  float pressureMin = 0.0f;
  float pressureMax = 0.0f;

  // 최근 구간이 아니라 현재 RAM에 보관된 전체 sampleCount를 축 계산에 반영한다.
  for (int i = 0; i < sampleCount; i++)
  {
    float value = graphPrimaryAt(i);
    if (value < primaryMin) primaryMin = value;
    if (value > primaryMax) primaryMax = value;

    if (pressureAxisOn && pressureValueValid(pressureHistory[i]))
    {
      if (!hasPressure)
      {
        pressureMin = pressureHistory[i];
        pressureMax = pressureHistory[i];
        hasPressure = true;
      }
      else
      {
        if (pressureHistory[i] < pressureMin) pressureMin = pressureHistory[i];
        if (pressureHistory[i] > pressureMax) pressureMax = pressureHistory[i];
      }
    }
  }

  float minimumSpan = activePrimaryMinimumSpan();
  if (primaryMax - primaryMin < minimumSpan)
  {
    float center = (primaryMax + primaryMin) * 0.5f;
    primaryMin = center - minimumSpan * 0.5f;
    primaryMax = center + minimumSpan * 0.5f;
  }
  else
  {
    float margin = (primaryMax - primaryMin) * 0.08f;
    primaryMin -= margin;
    primaryMax += margin;
  }

  // 물리적으로 음수가 될 수 없는 센서는 음수 축을 만들지 않는다.
  if ((activeSensorMode == SENSOR_MODE_TSL2591 ||
       activeSensorMode == SENSOR_MODE_SCD41 ||
       activeSensorMode == SENSOR_MODE_VL53L1X) && primaryMin < 0.0f)
  {
    primaryMin = 0.0f;
  }

  if (hasPressure)
  {
    if (pressureMax - pressureMin < 1.0f)
    {
      float center = (pressureMax + pressureMin) * 0.5f;
      pressureMin = center - 0.5f;
      pressureMax = center + 0.5f;
    }
    else
    {
      float margin = (pressureMax - pressureMin) * 0.08f;
      pressureMin -= margin;
      pressureMax += margin;
    }
  }

  // 차트 내부는 작은 고정 범위만 사용하고 실제 수치는 외부 축 라벨에 표시한다.
  // 따라서 수만 lux도 int16 범위를 넘지 않는다.
  static const int CHART_NORM_MIN = 0;
  static const int CHART_NORM_MAX = 1000;
  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, CHART_NORM_MIN, CHART_NORM_MAX);
  lv_chart_set_range(chart, LV_CHART_AXIS_SECONDARY_Y, CHART_NORM_MIN, CHART_NORM_MAX);
  lv_chart_set_all_value(chart, seriesTemp, LV_CHART_POINT_NONE);
  lv_chart_set_all_value(chart, seriesPressure, LV_CHART_POINT_NONE);

  float primarySpan = primaryMax - primaryMin;
  if (primarySpan < 0.000001f) primarySpan = 1.0f;
  float pressureSpan = pressureMax - pressureMin;
  if (pressureSpan < 0.000001f) pressureSpan = 1.0f;

  // 1~720개의 전체 기록을 220개의 고정 표시점으로 선형 재표본화한다.
  // 데이터가 적어도 그래프 전체 폭을 사용하고, 많아져도 처음부터 마지막까지 모두 반영한다.
  for (int point = 0; point < CHART_POINTS; point++)
  {
    float sourcePosition = 0.0f;
    if (sampleCount > 1)
    {
      sourcePosition = ((float)(sampleCount - 1) * (float)point) / (float)(CHART_POINTS - 1);
    }

    float primaryValue = graphPrimaryInterpolated(sourcePosition);
    float primaryRatio = (primaryValue - primaryMin) / primarySpan;
    if (primaryRatio < 0.0f) primaryRatio = 0.0f;
    if (primaryRatio > 1.0f) primaryRatio = 1.0f;
    int chartPrimary = (int)lroundf(primaryRatio * (float)CHART_NORM_MAX);
    lv_chart_set_value_by_id(chart, seriesTemp, point, chartPrimary);

    if (hasPressure)
    {
      float pressureValue = graphSecondaryInterpolated(sourcePosition);
      if (pressureValueValid(pressureValue))
      {
        float pressureRatio = (pressureValue - pressureMin) / pressureSpan;
        if (pressureRatio < 0.0f) pressureRatio = 0.0f;
        if (pressureRatio > 1.0f) pressureRatio = 1.0f;
        int chartPressure = (int)lroundf(pressureRatio * (float)CHART_NORM_MAX);
        lv_chart_set_value_by_id(chart, seriesPressure, point, chartPressure);
      }
    }
  }

  char text[32];
  for (int i = 0; i < 6; i++)
  {
    float primaryTick = primaryMin + (primaryMax - primaryMin) * i / 5.0f;
    if (activePrimaryDecimals() >= 2) snprintf(text, sizeof(text), "%.2f", primaryTick);
    else snprintf(text, sizeof(text), "%.1f", primaryTick);

    if (labelMeasureTempTicks[i])
    {
      lv_obj_clear_flag(labelMeasureTempTicks[i], LV_OBJ_FLAG_HIDDEN);
      lv_label_set_text(labelMeasureTempTicks[i], text);
    }

    if (labelMeasurePressureTicks[i])
    {
      if (hasPressure)
      {
        float pressureTick = pressureMin + (pressureMax - pressureMin) * i / 5.0f;
        snprintf(text, sizeof(text), "%.1f", pressureTick);
        lv_obj_clear_flag(labelMeasurePressureTicks[i], LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(labelMeasurePressureTicks[i], text);
      }
      else
      {
        lv_obj_add_flag(labelMeasurePressureTicks[i], LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(labelMeasurePressureTicks[i], "");
      }
    }
  }

  uint32_t startTime = timeHistory[0];
  uint32_t endTime = timeHistory[sampleCount - 1];
  uint32_t spanTime = endTime >= startTime ? endTime - startTime : 0;

  for (int i = 0; i < 6; i++)
  {
    uint32_t tickTime = spanTime > 0 ? (uint32_t)(((uint64_t)spanTime * i) / 5) : 0;
    formatMeasureTimeTick(tickTime, text, sizeof(text));
    if (labelMeasureTimeTicks[i]) lv_label_set_text(labelMeasureTimeTicks[i], text);
  }

  if (labelGraphStart) lv_label_set_text(labelGraphStart, "측정 시간(s)");
  if (labelGraphEnd) lv_label_set_text(labelGraphEnd, "");
  lv_chart_refresh(chart);
}


void makePlotTickLabel(lv_obj_t *parent, const char *text, int x, int y, int w, lv_text_align_t align)
{
  lv_obj_t *label = lv_label_create(parent);
  lv_label_set_text(label, text);
  lv_obj_set_width(label, w);
  lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_font(label, FONT_GRAPH_SMALL, 0);
  lv_obj_set_style_text_color(label, lv_color_hex(0x374151), 0);
  lv_obj_set_style_text_align(label, align, 0);
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, x, y);
  lv_obj_clear_flag(label, LV_OBJ_FLAG_SCROLLABLE);
}

bool isPlotSampleEnabled(int idx)
{
  if (idx < 0 || idx >= sampleCount) return false;
  return sampleEnabled[idx];
}

// =====================================================
// Screen transition helper
// =====================================================
uint32_t screenBgColor(lv_obj_t *screen);


void createHomeUi();

void hideAllFloatingUi()
{
  if (homeKeyboard)
  {
    lv_keyboard_set_textarea(homeKeyboard, NULL);
    lv_obj_add_flag(homeKeyboard, LV_OBJ_FLAG_HIDDEN);
  }
  if (csvKeyboard)
  {
    lv_keyboard_set_textarea(csvKeyboard, NULL);
    lv_obj_add_flag(csvKeyboard, LV_OBJ_FLAG_HIDDEN);
  }
  if (islKeyboard)
  {
    lv_keyboard_set_textarea(islKeyboard, NULL);
    lv_obj_add_flag(islKeyboard, LV_OBJ_FLAG_HIDDEN);
  }
  if (wifiKeyboard)
  {
    lv_keyboard_set_textarea(wifiKeyboard, NULL);
    lv_obj_add_flag(wifiKeyboard, LV_OBJ_FLAG_HIDDEN);
  }
}

void prepareScreenNoScroll(lv_obj_t *screen)
{
  if (screen == NULL) return;

  lv_obj_set_size(screen, LCD_H_RES, LCD_V_RES);
  lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scrollbar_mode(screen, LV_SCROLLBAR_MODE_OFF);
  lv_obj_scroll_to_y(screen, 0, LV_ANIM_OFF);

  // v14: Every screen itself must be an opaque 1024x600 surface.
  // If the root screen is transparent, old LCD pixels can remain in unused areas.
  lv_obj_set_style_bg_color(screen, lv_color_hex(screenBgColor(screen)), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(screen, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(screen, 0, LV_PART_MAIN);
  lv_obj_set_style_shadow_width(screen, 0, LV_PART_MAIN);
}

static lv_obj_t *currentLoadedScreen = NULL;

// v13: each screen gets an opaque full-screen base object.
// This prevents old pixels/black lines from remaining in empty areas
// when switching between pre-created LVGL screens.
static lv_obj_t *homeScreenBase = NULL;
static lv_obj_t *measureScreenBase = NULL;
static lv_obj_t *csvScreenBase = NULL;
static lv_obj_t *settingsScreenBase = NULL;
static lv_obj_t *fileViewerScreenBase = NULL;
static lv_obj_t *islScreenBase = NULL;
static lv_obj_t *bleScreenBase = NULL;

uint32_t screenBgColor(lv_obj_t *screen)
{
  // ensureOpaqueScreenBase() paints a full-screen object in this colour behind
  // every widget, so it — not the screen's own bg_color — is what the user
  // sees. Screens still on the old dark theme keep 0x0B1020 until they are
  // converted.
  if (screen == homeScreen) return UI_BG;
  if (screen == measureScreen) return UI_BG;
  if (screen == csvScreen) return UI_BG;
  if (screen == settingsScreen) return UI_BG;
  if (screen == fileViewerScreen) return UI_BG;
  if (screen == islScreen) return UI_BG;
  if (screen == bleScreen) return UI_BG;
  return 0x0B1020;
}

lv_obj_t **screenBaseSlot(lv_obj_t *screen)
{
  if (screen == homeScreen) return &homeScreenBase;
  if (screen == measureScreen) return &measureScreenBase;
  if (screen == csvScreen) return &csvScreenBase;
  if (screen == settingsScreen) return &settingsScreenBase;
  if (screen == fileViewerScreen) return &fileViewerScreenBase;
  if (screen == islScreen) return &islScreenBase;
  if (screen == bleScreen) return &bleScreenBase;
  return NULL;
}

void ensureOpaqueScreenBase(lv_obj_t *screen)
{
  if (screen == NULL) return;

  lv_obj_t **slot = screenBaseSlot(screen);
  if (slot == NULL) return;

  uint32_t colorHex = screenBgColor(screen);

  if (*slot == NULL || lv_obj_get_parent(*slot) != screen)
  {
    *slot = lv_obj_create(screen);
    lv_obj_set_size(*slot, LCD_H_RES, LCD_V_RES);
    lv_obj_align(*slot, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_clear_flag(*slot, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(*slot, LV_SCROLLBAR_MODE_OFF);

    lv_obj_set_style_radius(*slot, 0, 0);
    lv_obj_set_style_border_width(*slot, 0, 0);
    lv_obj_set_style_shadow_width(*slot, 0, 0);
    lv_obj_set_style_pad_all(*slot, 0, 0);
  }

  lv_obj_set_style_bg_color(*slot, lv_color_hex(colorHex), 0);
  lv_obj_set_style_bg_opa(*slot, LV_OPA_COVER, 0);
  lv_obj_move_background(*slot);
  lv_obj_invalidate(*slot);
}

void forceScreenFlush(lv_obj_t *screen)
{
  // Never re-enter LVGL from an event callback.
  if (screen) lv_obj_invalidate(screen);
}

void loadScreenSafe(lv_obj_t *screen)
{
  requestScreenSwitch(screen);
}

void loadHomeScreenFresh()
{
  // Reuse the original dashboard object tree. Never delete/recreate it.
  requestScreenSwitch(homeScreen);
}

void requestScreenSwitch(lv_obj_t *screen)
{
  if (screen == NULL) return;
  pendingScreen = screen;
  pendingScreenRequestMs = millis();
}

void prepareScreenContent(lv_obj_t *screen)
{
  if (screen == homeScreen)
  {
    updateHomeWifiLabels();
    refreshHomeSensorLabels();
    updateDashboardIslLabels();
  }
  else if (screen == measureScreen)
  {
    if (sampleCount > 0)
    {
      updateChartAutoScale();
      chartUiDirty = false;
      lastChartUiRefreshMs = millis();
    }
    refreshLatestMeasurementLabels();
  }
  else if (screen == settingsScreen)
  {
    if (labelSettingsWifi)
    {
      lv_label_set_text(labelSettingsWifi, wifiConnected ? "WiFi: 연결됨" : "WiFi: 연결 안 됨");
    }
  }
  else if (screen == islScreen)
  {
    if (islModuleTa && strlen(islRuntimeSerialNumber) > 0)
      lv_textarea_set_text(islModuleTa, islRuntimeSerialNumber);

    if (islServiceKeyTa && strcmp(islServiceKey, "PUT_YOUR_SERVICE_KEY") != 0)
      lv_textarea_set_text(islServiceKeyTa, islServiceKey);

    updateIslStatusLabels();
  }
}

void activateScreenNow(lv_obj_t *screen)
{
  if (screen == NULL) return;

  hideAllFloatingUi();
  prepareScreenNoScroll(screen);
  ensureOpaqueScreenBase(screen);
  prepareScreenContent(screen);

  lv_scr_load(screen);
  currentLoadedScreen = screen;
  lv_obj_invalidate(screen);
  ignoreTouchUntilMs = millis() + SCREEN_TOUCH_LOCKOUT_MS;
}

void servicePendingScreenSwitch()
{
  lv_obj_t *target = pendingScreen;
  if (target == NULL || uiInputLocked) return;

  unsigned long now = millis();
  if (touchRawDown) return;
  if (now - touchLastChangeMs < SCREEN_RELEASE_SETTLE_MS) return;
  if (now - pendingScreenRequestMs < SCREEN_RELEASE_SETTLE_MS) return;

  pendingScreen = NULL;
  activateScreenNow(target);
}


// =====================================================
// Button callbacks
// =====================================================
static void go_measure_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  requestScreenSwitch(measureScreen);
}

static void go_home_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  requestScreenSwitch(homeScreen);
}


static void go_csv_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  requestScreenSwitch(csvScreen);
}

static void go_isl_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  requestScreenSwitch(islScreen);
}

static void start_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) == LV_EVENT_CLICKED)
  {
    if (isBatchUploading())
    {
      setIslStatusText("일괄전송 중: 시작 무시");
      return;
    }
    if (!dpsReady)
    {
      if (activeSensorMode == SENSOR_MODE_SCD41)
      {
        if (labelStatus) lv_label_set_text(labelStatus, "SCD41 재연결...");
        dpsReady = scd41Begin();
      }

      if (!dpsReady)
      {
        lv_label_set_text(labelStatus, activeSensorMode == SENSOR_MODE_SCD41 ? "SCD41 인식 실패" : "센서오류");
        if (activeSensorMode == SENSOR_MODE_SCD41)
          setIslStatusText("SCD41 0x62 없음: 안정적 전원·SDA2·SCL3 확인");
        return;
      }
    }

    // The angle is cumulative, so a fresh run has to start from zero the way
    // the clock and the sample count do - otherwise the first reading carries
    // over however far the wheel was turned while setting the experiment up.
    if (activeSensorMode == SENSOR_MODE_ENCODER && measurementCount == 0 && sampleCount == 0)
    {
      encoderZero();
    }

    if (measurementCount == 0 && sampleCount == 0)
    {
      startMs = millis();
      measureAccumulatedMs = 0;
      dataClockStarted = false;
      dataFirstSampleMs = 0;
      lv_label_set_text(labelRuntime, "시간 00:00:00");
    }

    if (!measureClockRunning)
    {
      measureResumeMs = millis();
      measureClockRunning = true;
    }

    measuring = true;
    wifiAutoRetryEnabled = false;

    // 실시간 모드에서만 세션 start를 보냅니다.
    // 일괄전송 모드는 보드에만 쌓아두고 [일괄전송] 버튼을 누를 때 한 번에 보냅니다.
    updateIslModuleCodeFromUi();
    if (!activeSensorSupportsDirectIsl())
    {
      clearCloudQueue();
      setIslStatusText("현재 센서: 보드 측정 모드 / ON 전송 미지원");
    }
    else if (cloudModumConfigured())
    {
      if (cloudUploadMode == CLOUD_UPLOAD_REALTIME)
      {
        resetDirectIslSessionCache("realtime start button");
        clearCloudQueue();
        queueCloudAction("start");
        setIslStatusText("직접전송: 실시간 start 요청");
      }
      else
      {
        // 정지 상태에서 일괄전송 측정을 새로 시작하면 전체 구간을 일괄전송한다.
        // 단, 기존 데이터가 남아 있는 상태에서 이어 측정하는 경우에는 전체 history를 보낼 수 있다.
        if (sampleCount <= 0) batchUploadStartIndex = 0;
        setIslStatusText("일괄전송 측정 시작");
      }
    }
    else
    {
      setIslStatusText("설정: 모둠코드 필요");
    }
    updateIslStatusLabels();

    lv_label_set_text(labelStatus, "측정");
    updateClockLabels();

    lastReadMs = millis();
  }
}

static void stop_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) == LV_EVENT_CLICKED)
  {
    if (isBatchUploading())
    {
      setIslStatusText("일괄전송 중: 정지 무시");
      return;
    }
    if (measureClockRunning)
    {
      measureAccumulatedMs += millis() - measureResumeMs;
      measureClockRunning = false;
    }

    measuring = false;
    wifiAutoRetryEnabled = false;
    lastAutoWifiRetryMs = millis();

    // 실시간 모드에서만 세션 stop을 보냅니다.
    // 일괄전송 모드는 지능형과학실 세션을 건드리지 않고, [일괄전송] 버튼에서 전송을 시작합니다.
    updateIslModuleCodeFromUi();
    if (!activeSensorSupportsDirectIsl())
    {
      clearCloudQueue();
      setIslStatusText("현재 센서: 로컬 측정 정지");
    }
    else if (cloudModumConfigured())
    {
      if (cloudUploadMode == CLOUD_UPLOAD_REALTIME)
      {
        clearCloudQueue();
        queueCloudAction("stop");
        setIslStatusText("정지 요청: 실시간 전송 큐 비움");
      }
      else
      {
        setIslStatusText("일괄전송 정지 - 일괄전송 클릭");
      }
    }
    else
    {
      setIslStatusText("설정: 모둠코드 필요");
    }
    updateIslStatusLabels();

    lv_label_set_text(labelStatus, "정지");
  }
}

static void clear_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) == LV_EVENT_CLICKED)
  {
    if (isBatchUploading())
    {
      setIslStatusText("일괄전송 중: 초기화 무시");
      return;
    }
    measuring = false;
    updateIslModuleCodeFromUi();
    if (cloudModumConfigured() && activeSensorSupportsDirectIsl())
    {
      queueCloudAction("stop");
      setIslStatusText("전송: stop 요청");
    }
    measurementCount = 0;
    sampleCount = 0;
    batchUploadStartIndex = 0;
    batchUploadProgressSent = 0;
    batchUploadProgressTotal = 0;
    tablePageOffset = 0;
    clearMeasurementBackupFromNvs();

    startMs = millis();

    measureAccumulatedMs = 0;
    measureResumeMs = 0;
    measureClockRunning = false;
    dataClockStarted = false;
    dataFirstSampleMs = 0;
    for (int i = 0; i < MAX_SAMPLES; i++)
    {
      sampleEnabled[i] = true;
      noHistory[i] = 0;
    }
    wifiAutoRetryEnabled = false;
    lastAutoWifiRetryMs = millis();

    lv_label_set_text(labelRuntime, "시간 00:00:00");
    updateActiveSensorUiLabels();
    lv_label_set_text(labelStatus, "초기화");
    lv_label_set_text(labelCount, "0");

    resetTable();
    clearChart();
  }
}

static void go_settings_event_cb(lv_event_t *e);
static void wifi_scan_event_cb(lv_event_t *e);
static void wifi_connect_event_cb(lv_event_t *e);
static void wifi_disconnect_event_cb(lv_event_t *e);
static void wifi_forget_event_cb(lv_event_t *e);
static void wifi_textarea_event_cb(lv_event_t *e);
static void wifi_keyboard_event_cb(lv_event_t *e);

static bool cloudModumConfigured()
{
  return strlen(islRuntimeSerialNumber) > 0;
}

static void dashboard_isl_save_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  updateIslModuleCodeFromUi();

  if (cloudModumConfigured()) setIslStatusText("전송: 모둠코드 저장됨");
  else setIslStatusText("설정: 모둠코드 필요");

  updateIslStatusLabels();
}

static void dashboard_isl_start_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  updateIslModuleCodeFromUi();

  if (!activeSensorSupportsDirectIsl())
  {
    clearCloudQueue();
    setIslStatusText("현재 센서: 지능형과학실 센서코드 미설정");
  }
  else if (cloudModumConfigured())
  {
    resetDirectIslSessionCache("dashboard start");
    clearCloudQueue();
    queueCloudAction("start");
  }
  else
  {
    setIslStatusText("설정: 모둠코드 필요");
  }

  updateIslStatusLabels();
}

static void dashboard_isl_stop_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  updateIslModuleCodeFromUi();

  if (!activeSensorSupportsDirectIsl())
  {
    clearCloudQueue();
    setIslStatusText("현재 센서: 로컬 측정 모드 / ON 전송 미지원");
  }
  else if (cloudModumConfigured())
  {
    queueCloudAction("stop");
  }
  else
  {
    setIslStatusText("설정: 모둠코드 필요");
  }

  updateIslStatusLabels();
}

static void dashboard_cloud_realtime_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (isBatchUploading())
  {
    setIslStatusText("일괄전송 중: 실시간 전환 무시");
    return;
  }

  updateIslModuleCodeFromUi();
  cloudUploadMode = CLOUD_UPLOAD_REALTIME;

  if (measuring)
  {
    // 일괄전송에서 실시간으로 전환하면 이후 새 샘플부터 실시간 전송한다.
    // 과거에 모아둔 값은 자동으로 섞어 보내지 않는다.
    clearCloudQueue();
    if (!activeSensorSupportsDirectIsl())
    {
      setIslStatusText("현재 센서: 보드 측정 모드 / ON 전송 미지원");
    }
    else if (cloudModumConfigured())
    {
      resetDirectIslSessionCache("realtime mode switch");
      queueCloudAction("start");
      setIslStatusText("직접전송: 실시간 전환 - 새 세션 시작");
    }
    else
    {
      setIslStatusText("전송: 실시간 전환 - 모둠코드 필요");
    }
  }
  else
  {
    setIslStatusText("전송: 실시간 전송 모드");
  }

  updateCloudModeLabel();
  updateIslStatusLabels();
}

static void dashboard_cloud_batch_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (isBatchUploading())
  {
    setIslStatusText("일괄전송 중: 모드 전환 무시");
    return;
  }

  updateIslModuleCodeFromUi();

  bool wasRealtimeMeasuring = measuring && cloudUploadMode == CLOUD_UPLOAD_REALTIME;
  cloudUploadMode = CLOUD_UPLOAD_BATCH;

  if (wasRealtimeMeasuring)
  {
    // 핵심 수정:
    // 실시간 구간은 이미 지능형과학실/Google Sheets로 전송된 값이 있으므로
    // 일괄전송 전환 시점 이후의 새 샘플만 일괄전송 대상으로 삼는다.
    // 여기서 stop을 보내면 지능형과학실 세션이 꼬일 수 있으므로 즉시 stop하지 않는다.
    // 최종 stop은 [일괄] 전송 완료 후 한 번만 보낸다.
    batchUploadStartIndex = sampleCount;
    // 실시간 그래프 세션은 즉시 정리하고, 이후부터 보드 내부에만 저장한다.
    // batch 전송은 나중에 [일괄전송] 버튼에서 처리한다.
    if (activeSensorSupportsDirectIsl())
    {
      queueCloudAction("stop");
      setIslStatusText("일괄전송 전환: 실시간 정지 요청");
    }
    else
    {
      setIslStatusText("현재 센서: 일괄 직접전송 코드 미설정");
    }
  }
  else if (measuring)
  {
    setIslStatusText("일괄전송 측정 중");
  }
  else
  {
    // 정지 상태에서 일괄전송을 선택하면 새 실험 구간 전체를 보낼 수 있게 0부터 시작한다.
    batchUploadStartIndex = 0;
    setIslStatusText("일괄전송 모드");
  }

  updateCloudModeLabel();
  updateIslStatusLabels();
}

static void dashboard_cloud_batch_upload_event_cb(lv_event_t *e)
{
  // V3 DEBUG: 버튼 callback 자체가 들어오는지 가장 먼저 확인합니다.
  Serial.println();
  Serial.println("========== BATCH BUTTON EVENT ==========");
  Serial.print("LVGL event code = ");
  Serial.println((int)lv_event_get_code(e));

  if (lv_event_get_code(e) != LV_EVENT_CLICKED)
  {
    Serial.println("BATCH DEBUG: ignored because event is not LV_EVENT_CLICKED");
    return;
  }

  Serial.println("BATCH DEBUG: CLICKED callback entered");

  if (isBatchUploading())
  {
    Serial.print("BATCH DEBUG: already busy. uploading=");
    Serial.print(batchUploading ? 1 : 0);
    Serial.print(" requested=");
    Serial.println(batchUploadRequested ? 1 : 0);
    setIslStatusText("일괄전송: 이미 요청/전송 중");
    requestBatchUiRefresh();
    updateIslStatusLabels();
    return;
  }

  updateIslModuleCodeFromUi();

  char debugModumId[64] = "";
  copyCurrentModumIdTo(debugModumId, sizeof(debugModumId));

  Serial.print("BATCH DEBUG: sensorMode=");
  Serial.print(activeSensorMode);
  Serial.print(" sensor=");
  Serial.println(activeSensorName());
  Serial.print("BATCH DEBUG: supportsISL=");
  Serial.println(activeSensorSupportsDirectIsl() ? 1 : 0);
  Serial.print("BATCH DEBUG: sampleCount=");
  Serial.println(sampleCount);
  Serial.print("BATCH DEBUG: measuring=");
  Serial.println(measuring ? 1 : 0);
  Serial.print("BATCH DEBUG: wifiConnected=");
  Serial.println(wifiConnected ? 1 : 0);
  Serial.print("BATCH DEBUG: wifiReadyForHttp=");
  Serial.println(wifiReadyForHttp() ? 1 : 0);
  Serial.print("BATCH DEBUG: cloudQueue=");
  Serial.println(cloudQueue != NULL ? "OK" : "NULL");
  Serial.print("BATCH DEBUG: modumId=");
  Serial.println(strlen(debugModumId) > 0 ? debugModumId : "(empty)");
  Serial.print("BATCH DEBUG: directIslConfigured=");
  Serial.println(directIslConfigured() ? 1 : 0);

  if (!activeSensorSupportsDirectIsl())
  {
    setIslStatusText("일괄전송: 현재 센서는 ON 전송 미지원");
    Serial.println("BATCH DEBUG FAIL: activeSensorSupportsDirectIsl() == false");
    updateIslStatusLabels();
    return;
  }

  if (!cloudModumConfigured() || strlen(debugModumId) == 0)
  {
    setIslStatusText("일괄전송: 모둠코드 필요");
    Serial.println("BATCH DEBUG FAIL: modumId not configured");
    updateIslStatusLabels();
    return;
  }

  if (!directIslConfigured())
  {
    setIslStatusText("일괄전송: serviceKey/모둠코드 확인");
    Serial.println("BATCH DEBUG FAIL: directIslConfigured() == false");
    updateIslStatusLabels();
    return;
  }

  if (sampleCount <= 0)
  {
    setIslStatusText("일괄전송: 보낼 데이터 없음");
    Serial.println("BATCH DEBUG FAIL: sampleCount <= 0");
    updateIslStatusLabels();
    return;
  }

  // WiFi task가 요청을 처리하지 못한 채 pending으로 남는 상황을 막기 위해
  // 버튼 단계에서 WiFi/IP 준비 상태를 명확하게 표시합니다.
  if (!wifiConnected || !wifiReadyForHttp())
  {
    setIslStatusText("일괄전송: WiFi/IP 연결 확인");
    Serial.println("BATCH DEBUG FAIL: WiFi/IP not ready");
    updateIslStatusLabels();
    return;
  }

  // 버튼을 눌렀을 때 측정 중이면 현재까지의 값을 확정하고 측정을 정지합니다.
  if (measuring)
  {
    if (measureClockRunning)
    {
      measureAccumulatedMs += millis() - measureResumeMs;
      measureClockRunning = false;
    }
    measuring = false;
    if (labelStatus) lv_label_set_text(labelStatus, "정지");
  }

  cloudUploadMode = CLOUD_UPLOAD_BATCH;

  // 현재까지 누적된 전체 샘플을 보냅니다.
  batchUploadStartIndex = 0;
  batchUploadProgressSent = 0;
  batchUploadProgressTotal = sampleCount;

  // 중요 수정:
  // 기존에는 queueCloudAction("batch") 패킷 하나에 의존했습니다.
  // V3에서는 전용 flag를 WiFi task가 직접 확인하므로 batch 요청이 큐에서 사라질 수 없습니다.
  clearCloudQueue();
  batchUploading = false;
  batchUploadRequested = true;
  batchUploadRequestMs = millis();

  char pendingLine[128];
  snprintf(
    pendingLine,
    sizeof(pendingLine),
    "일괄전송 요청 접수: %s %d개",
    activeSensorMode == SENSOR_MODE_TSL2591 ? "조도" : "데이터",
    sampleCount
  );
  setIslStatusText(pendingLine);

  Serial.print("BATCH DEBUG: REQUEST ACCEPTED, samples=");
  Serial.println(sampleCount);
  Serial.print("BATCH DEBUG: light sensor type preferred=");
  Serial.println(directIslLightSensorType);
  Serial.println("========================================");

  requestBatchUiRefresh();
  updateCloudModeLabel();
  updateIslStatusLabels();
}
static void dashboard_csv_mount_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  updateCsvFileNameFromDashboard();
  sdReady = initSdCard();
  updateSdStatusLabels();
  updateDashboardCsvPreview();
}

static void dashboard_csv_toggle_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  updateCsvFileNameFromDashboard();
  csvLoggingEnabled = !csvLoggingEnabled;
  if (!csvLoggingEnabled) closeCsvFile();

  if (csvLoggingEnabled && !sdReady)
  {
    sdReady = initSdCard();
  }

  updateSdStatusLabels();
  updateDashboardCsvPreview();
}

static void dashboard_csv_preview_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  updateDashboardCsvLabels();
  updateDashboardCsvPreview();
}

static void csv_save_measurement_data_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  updateCsvFileNameFromDashboard();

  if (sampleCount <= 0)
  {
    if (labelHomeCsv) lv_label_set_text(labelHomeCsv, "CSV: 저장할 데이터 없음");
    if (labelHomeCsvPreview)
    {
      lv_label_set_text(labelHomeCsvPreview, "저장할 측정 데이터가 없습니다.\n먼저 센서 측정을 시작하세요.");
    }
    updateDashboardCsvLabels();
    if (labelHomeCsv) lv_label_set_text(labelHomeCsv, "CSV: 저장할 데이터 없음");
    return;
  }

  if (!sdReady)
  {
    sdReady = initSdCard();
  }

  if (!sdReady)
  {
    if (labelHomeCsv) lv_label_set_text(labelHomeCsv, "CSV: SD 카드 오류");
    if (labelHomeCsvPreview)
    {
      lv_label_set_text(labelHomeCsvPreview, "SD 카드를 마운트하지 못했습니다.\nSD 카드 삽입 상태와 핀 설정을 확인하세요.");
    }
    updateSdStatusLabels();
    if (labelHomeCsv) lv_label_set_text(labelHomeCsv, "CSV: SD 카드 오류");
    return;
  }

  FILE *file = fopen(csvPath, "w");
  if (file == NULL)
  {
    if (labelHomeCsv) lv_label_set_text(labelHomeCsv, "CSV: 파일 생성 실패");
    if (labelHomeCsvPreview)
    {
      char msg[220];
      snprintf(msg, sizeof(msg), "CSV 파일을 만들 수 없습니다.\n%s", csvPath);
      lv_label_set_text(labelHomeCsvPreview, msg);
    }
    Serial.print("CSV snapshot open failed: ");
    Serial.println(csvPath);
    return;
  }

  fprintf(file, CSV_HEADER_LINE "\n");

  for (int i = 0; i < sampleCount; i++)
  {
    char columns[160];
    formatCsvValueColumns(tempHistory[i], pressureHistory[i], humidityHistory[i],
                          columns, sizeof(columns));

    fprintf(
      file,
      "%d,%lu,%s,%s\n",
      noHistory[i] > 0 ? noHistory[i] : (i + 1),
      (unsigned long)timeHistory[i],
      activeSensorName(),
      columns
    );

  }

  fclose(file);

  // 이 버튼은 현재까지 누적된 측정 데이터를 한 번에 저장한다.
  // 측정 루프의 자동 append 방식은 사용하지 않는다.
  csvLoggingEnabled = false;
  closeCsvFile();

  if (labelSelectedSdFile)
  {
    char selected[96];
    snprintf(selected, sizeof(selected), "저장: %s", csvFileName);
    lv_label_set_text(labelSelectedSdFile, selected);
  }

  updateSdStatusLabels();
  updateDashboardCsvLabels();

  if (labelHomeCsv) lv_label_set_text(labelHomeCsv, "CSV: 측정 데이터 저장됨");

  if (labelHomeCsvPreview)
  {
    char msg[512];
    snprintf(
      msg,
      sizeof(msg),
      "측정 데이터 저장 완료\n파일: %s\n샘플 수: %d\n\n최근 데이터는 [미리보기], 저장 파일은 [파일 목록] 또는 [열기]로 확인하세요.",
      csvPath,
      sampleCount
    );
    lv_label_set_text(labelHomeCsvPreview, msg);
  }
}

bool endsWithIgnoreCase(const char *text, const char *suffix)
{
  if (text == NULL || suffix == NULL) return false;

  size_t textLen = strlen(text);
  size_t suffixLen = strlen(suffix);

  if (textLen < suffixLen) return false;

  const char *start = text + textLen - suffixLen;

  for (size_t i = 0; i < suffixLen; i++)
  {
    char a = start[i];
    char b = suffix[i];

    if (a >= 'A' && a <= 'Z') a = a - 'A' + 'a';
    if (b >= 'A' && b <= 'Z') b = b - 'A' + 'a';

    if (a != b) return false;
  }

  return true;
}

void sanitizeBasicFileName(const char *input, char *out, size_t outSize)
{
  if (out == NULL || outSize == 0) return;
  out[0] = '\0';
  if (input == NULL) return;

  size_t j = 0;
  for (size_t i = 0; input[i] != '\0' && j < outSize - 1; i++)
  {
    char c = input[i];

    // SD 루트 폴더의 파일명만 허용한다. 경로 구분자는 제거한다.
    if ((c >= 'A' && c <= 'Z') ||
        (c >= 'a' && c <= 'z') ||
        (c >= '0' && c <= '9') ||
        c == '_' || c == '-' || c == '.')
    {
      out[j++] = c;
    }
  }

  out[j] = '\0';
}

bool isTextViewFile(const char *name)
{
  if (name == NULL) return false;

  return endsWithIgnoreCase(name, ".csv") ||
         endsWithIgnoreCase(name, ".txt");
}

void setSelectedSdTextFile(const char *fileName)
{
  char clean[sizeof(sdSelectedTextFile)];
  sanitizeBasicFileName(fileName, clean, sizeof(clean));

  if (!isTextViewFile(clean))
  {
    sdSelectedTextFile[0] = '\0';
    if (labelSelectedSdFile) lv_label_set_text(labelSelectedSdFile, "선택: csv/txt 파일 없음");
    return;
  }

  strncpy(sdSelectedTextFile, clean, sizeof(sdSelectedTextFile) - 1);
  sdSelectedTextFile[sizeof(sdSelectedTextFile) - 1] = '\0';

  if (homeCsvFileTa) lv_textarea_set_text(homeCsvFileTa, sdSelectedTextFile);

  if (labelSelectedSdFile)
  {
    char line[140];
    snprintf(line, sizeof(line), "선택: %s", sdSelectedTextFile);
    lv_label_set_text(labelSelectedSdFile, line);
  }
}

void showFileViewerText(const char *text)
{
  if (fileViewerTextArea)
  {
    lv_textarea_set_text(fileViewerTextArea, text ? text : "");
    lv_textarea_set_cursor_pos(fileViewerTextArea, 0);
    lv_obj_scroll_to_y(fileViewerTextArea, 0, LV_ANIM_OFF);
  }
}

void loadSdTextFilePage(const char *fileName, long offset)
{
  char cleanName[96];
  sanitizeBasicFileName(fileName, cleanName, sizeof(cleanName));

  if (strlen(cleanName) == 0 || !isTextViewFile(cleanName))
  {
    snprintf(csvFileViewBuffer, sizeof(csvFileViewBuffer), "열람 가능한 파일은 .csv 또는 .txt 입니다.");
    showFileViewerText(csvFileViewBuffer);
    if (labelHomeCsvPreview) lv_label_set_text(labelHomeCsvPreview, csvFileViewBuffer);
    return;
  }

  if (!sdReady)
  {
    sdReady = initSdCard();
    updateSdStatusLabels();
  }

  if (!sdReady)
  {
    snprintf(csvFileViewBuffer, sizeof(csvFileViewBuffer), "SD 카드가 준비되지 않았습니다.");
    showFileViewerText(csvFileViewBuffer);
    if (labelHomeCsvPreview) lv_label_set_text(labelHomeCsvPreview, csvFileViewBuffer);
    return;
  }

  char path[180];
  snprintf(path, sizeof(path), "%s/%s", MOUNT_POINT, cleanName);

  struct stat st;
  if (stat(path, &st) != 0)
  {
    snprintf(csvFileViewBuffer, sizeof(csvFileViewBuffer), "파일 정보를 읽을 수 없습니다.\n%s", path);
    showFileViewerText(csvFileViewBuffer);
    if (labelHomeCsvPreview) lv_label_set_text(labelHomeCsvPreview, csvFileViewBuffer);
    return;
  }

  csvFileViewSize = (long)st.st_size;
  if (offset < 0) offset = 0;
  if (offset >= csvFileViewSize && csvFileViewSize > 0)
  {
    offset = (csvFileViewSize / CSV_FILE_VIEW_PAGE_BYTES) * CSV_FILE_VIEW_PAGE_BYTES;
  }
  csvFileViewOffset = offset;

  FILE *file = fopen(path, "rb");
  if (file == NULL)
  {
    snprintf(csvFileViewBuffer, sizeof(csvFileViewBuffer), "파일을 열 수 없습니다.\n%s", path);
    showFileViewerText(csvFileViewBuffer);
    if (labelHomeCsvPreview) lv_label_set_text(labelHomeCsvPreview, csvFileViewBuffer);
    Serial.print("Open failed: ");
    Serial.println(path);
    return;
  }

  if (fseek(file, csvFileViewOffset, SEEK_SET) != 0)
  {
    fclose(file);
    snprintf(csvFileViewBuffer, sizeof(csvFileViewBuffer), "파일 위치 이동 실패\n%s", path);
    showFileViewerText(csvFileViewBuffer);
    if (labelHomeCsvPreview) lv_label_set_text(labelHomeCsvPreview, csvFileViewBuffer);
    return;
  }

  strncpy(csvFileViewName, cleanName, sizeof(csvFileViewName) - 1);
  csvFileViewName[sizeof(csvFileViewName) - 1] = '\0';

  long endByte = csvFileViewOffset + (long)CSV_FILE_VIEW_PAGE_BYTES;
  if (endByte > csvFileViewSize) endByte = csvFileViewSize;

  size_t head = snprintf(
    csvFileViewBuffer,
    sizeof(csvFileViewBuffer),
    "파일: %s\n크기: %ld bytes | 표시: %ld - %ld\n----------------------------------------\n",
    cleanName,
    csvFileViewSize,
    csvFileViewOffset,
    endByte
  );

  size_t maxRead = sizeof(csvFileViewBuffer) - head - 256;
  if (maxRead > CSV_FILE_VIEW_PAGE_BYTES) maxRead = CSV_FILE_VIEW_PAGE_BYTES;

  size_t readBytes = fread(csvFileViewBuffer + head, 1, maxRead, file);
  fclose(file);

  size_t total = head + readBytes;

  for (size_t i = head; i < total; i++)
  {
    if (csvFileViewBuffer[i] == '\0') csvFileViewBuffer[i] = ' ';
  }
  csvFileViewBuffer[total] = '\0';

  if (csvFileViewOffset + (long)readBytes < csvFileViewSize)
  {
    strncat(csvFileViewBuffer, "\n\n--- 파일이 커서 앞부분만 표시됩니다. ---", sizeof(csvFileViewBuffer) - strlen(csvFileViewBuffer) - 1);
  }

  if (labelFileViewerTitle)
  {
    char title[140];
    snprintf(title, sizeof(title), "파일 보기: %s", cleanName);
    lv_label_set_text(labelFileViewerTitle, title);
  }

  if (labelFileViewerPageInfo)
  {
    char info[160];
    long shownEnd = csvFileViewOffset + (long)readBytes;
    if (shownEnd > csvFileViewSize) shownEnd = csvFileViewSize;
    snprintf(info, sizeof(info), "%ld / %ld bytes", shownEnd, csvFileViewSize);
    lv_label_set_text(labelFileViewerPageInfo, info);
  }

  showFileViewerText(csvFileViewBuffer);

  if (labelHomeCsvPreview)
  {
    char msg[220];
    snprintf(msg, sizeof(msg), "열림: %s\n파일 보기 화면에서 스크롤해 확인하세요.", cleanName);
    lv_label_set_text(labelHomeCsvPreview, msg);
  }

  if (labelSelectedSdFile)
  {
    char line[140];
    snprintf(line, sizeof(line), "선택: %s", cleanName);
    lv_label_set_text(labelSelectedSdFile, line);
  }

  Serial.print("Opened SD text file page: ");
  Serial.print(path);
  Serial.print(" offset=");
  Serial.println(csvFileViewOffset);
}

void openSdTextFileByName(const char *fileName)
{
  char cleanName[96];
  sanitizeBasicFileName(fileName, cleanName, sizeof(cleanName));

  if (strlen(cleanName) == 0)
  {
    if (labelHomeCsvPreview) lv_label_set_text(labelHomeCsvPreview, "파일명이 비어 있습니다. 파일 목록에서 csv/txt 파일을 선택하세요.");
    return;
  }

  if (!isTextViewFile(cleanName))
  {
    if (labelHomeCsvPreview) lv_label_set_text(labelHomeCsvPreview, "열람 가능한 파일은 .csv 또는 .txt 입니다.");
    return;
  }

  setSelectedSdTextFile(cleanName);
  csvFileViewOffset = 0;
  loadSdTextFilePage(cleanName, 0);

  if (fileViewerScreen)
  {
    loadScreenSafe(fileViewerScreen);
  }
}

static void csv_viewer_prev_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (strlen(csvFileViewName) == 0) return;

  long nextOffset = csvFileViewOffset - CSV_FILE_VIEW_PAGE_BYTES;
  if (nextOffset < 0) nextOffset = 0;
  loadSdTextFilePage(csvFileViewName, nextOffset);
}

static void csv_viewer_next_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (strlen(csvFileViewName) == 0) return;

  long nextOffset = csvFileViewOffset + CSV_FILE_VIEW_PAGE_BYTES;
  if (nextOffset >= csvFileViewSize) nextOffset = csvFileViewOffset;
  loadSdTextFilePage(csvFileViewName, nextOffset);
}

static void sd_file_item_event_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code != LV_EVENT_CLICKED && code != LV_EVENT_SHORT_CLICKED && code != LV_EVENT_RELEASED) return;

  const char *fileName = (const char *)lv_event_get_user_data(e);
  setSelectedSdTextFile(fileName);
  openSdTextFileByName(sdSelectedTextFile);
}

void refreshSdFileList()
{
  if (sdFileList)
  {
    lv_obj_clean(sdFileList);
  }

  sdListedFileCount = 0;
  sdSelectedTextFile[0] = '\0';
  if (labelSelectedSdFile) lv_label_set_text(labelSelectedSdFile, "선택: 없음");

  if (!sdReady)
  {
    sdReady = initSdCard();
    updateSdStatusLabels();
  }

  if (!sdReady)
  {
    if (sdFileList) lv_list_add_text(sdFileList, "SD 카드 마운트 실패");
    if (labelSettingsNote) lv_label_set_text(labelSettingsNote, "SD 카드 파일 목록을 읽을 수 없습니다.");
    return;
  }

  DIR *dir = opendir(MOUNT_POINT);

  if (dir == NULL)
  {
    if (sdFileList) lv_list_add_text(sdFileList, "폴더 열기 실패");
    if (labelSettingsNote) lv_label_set_text(labelSettingsNote, "SD 루트 폴더 열기 실패");
    return;
  }

  struct dirent *entry;
  int count = 0;
  char line[128];

  while ((entry = readdir(dir)) != NULL && count < SD_FILE_LIST_MAX)
  {
    if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
    {
      continue;
    }

    strncpy(sdListedFiles[count], entry->d_name, sizeof(sdListedFiles[count]) - 1);
    sdListedFiles[count][sizeof(sdListedFiles[count]) - 1] = '\0';

    bool canOpen = isTextViewFile(sdListedFiles[count]);

    snprintf(
      line,
      sizeof(line),
      "%02d  %s%s",
      count + 1,
      sdListedFiles[count],
      canOpen ? "" : "  -"
    );

    if (sdFileList)
    {
      lv_obj_t *item;

      if (canOpen)
      {
        item = lv_list_add_btn(sdFileList, NULL, line);
        lv_obj_add_event_cb(item, sd_file_item_event_cb, LV_EVENT_ALL, sdListedFiles[count]);
      }
      else
      {
        item = lv_list_add_text(sdFileList, line);
      }

      lv_obj_set_style_text_font(item, FONT_KR, 0);
    }

    Serial.print("SD file: ");
    Serial.println(sdListedFiles[count]);
    count++;
  }

  closedir(dir);
  sdListedFileCount = count;

  if (count == 0)
  {
    if (sdFileList) lv_list_add_text(sdFileList, "파일 없음");
    if (labelSettingsNote) lv_label_set_text(labelSettingsNote, "SD 카드가 비어 있습니다.");
  }
  else
  {
    snprintf(line, sizeof(line), "SD 파일 %d개 표시 / csv·txt는 터치해서 열기", count);
    if (labelSettingsNote) lv_label_set_text(labelSettingsNote, line);

    if (labelHomeCsvPreview)
    {
      lv_label_set_text(labelHomeCsvPreview, "파일 목록에서 .csv 또는 .txt 파일을 터치하면 내용이 열립니다.");
    }
  }
}

static void settings_sd_list_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  refreshSdFileList();
}


// =====================================================
// Home UI
// =====================================================



// Show the saved group code on the home row, or say plainly that there is none.
void refreshHomeModumCodeLabel()
{
  if (labelHomeModumCode == NULL) return;

  if (strlen(islRuntimeSerialNumber) > 0)
  {
    lv_label_set_text(labelHomeModumCode, islRuntimeSerialNumber);
    lv_obj_set_style_text_color(labelHomeModumCode, lv_color_hex(UI_TEXT), 0);
  }
  else
  {
    lv_label_set_text(labelHomeModumCode, "미입력");
    lv_obj_set_style_text_color(labelHomeModumCode, lv_color_hex(UI_TEXT_3), 0);
  }
}

void closeHomeCodeEditor()
{
  if (homeKeyboard)
  {
    lv_keyboard_set_textarea(homeKeyboard, NULL);
    lv_obj_add_flag(homeKeyboard, LV_OBJ_FLAG_HIDDEN);
  }

  if (homeCodeEditor) lv_obj_add_flag(homeCodeEditor, LV_OBJ_FLAG_HIDDEN);
}

static void home_code_open_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (homeCodeEditor == NULL || homeIslModuleTa == NULL) return;

  lv_textarea_set_text(homeIslModuleTa, islRuntimeSerialNumber);

  lv_obj_clear_flag(homeCodeEditor, LV_OBJ_FLAG_HIDDEN);
  lv_obj_move_foreground(homeCodeEditor);

  if (homeKeyboard)
  {
    lv_keyboard_set_textarea(homeKeyboard, homeIslModuleTa);
    lv_obj_clear_flag(homeKeyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(homeKeyboard);
  }
}

static void home_code_cancel_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  closeHomeCodeEditor();
}

static void home_code_confirm_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  dashboard_isl_save_event_cb(e);
  refreshHomeModumCodeLabel();
  closeHomeCodeEditor();
}

// A sheet over the sensor grid, so the field being typed into is never the
// thing the keyboard hides, and 저장/취소 always close it.
void createHomeCodeEditor()
{
  homeCodeEditor = lv_obj_create(homeScreen);
  lv_obj_set_size(homeCodeEditor, LCD_H_RES, LCD_V_RES - UI_STATUSBAR_H - 220);
  lv_obj_align(homeCodeEditor, LV_ALIGN_TOP_LEFT, 0, UI_STATUSBAR_H);
  lv_obj_set_style_bg_color(homeCodeEditor, lv_color_hex(UI_BG), 0);
  lv_obj_set_style_bg_opa(homeCodeEditor, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(homeCodeEditor, 0, 0);
  lv_obj_set_style_border_width(homeCodeEditor, 0, 0);
  lv_obj_set_style_pad_all(homeCodeEditor, 0, 0);
  lv_obj_clear_flag(homeCodeEditor, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(homeCodeEditor, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *card = makePanel(homeCodeEditor, 162, 36, 700, 190);

  makeHeading(card, "모둠코드", 28, 22, UI_TEXT);
  makeSmallLabel(card, "지능형 과학실 ON에서 받은 코드를 입력하세요.", 28, 58, UI_TEXT_3);

  homeIslModuleTa = lv_textarea_create(card);
  lv_obj_set_size(homeIslModuleTa, 644, 48);
  lv_obj_align(homeIslModuleTa, LV_ALIGN_TOP_LEFT, 28, 86);
  lv_textarea_set_one_line(homeIslModuleTa, true);
  lv_textarea_set_password_mode(homeIslModuleTa, false);
  lv_textarea_set_placeholder_text(homeIslModuleTa, "예: ON040000093851");
  lv_obj_set_style_text_font(homeIslModuleTa, FONT_KR, 0);
  lv_obj_set_style_bg_color(homeIslModuleTa, lv_color_hex(UI_BG), 0);
  lv_obj_set_style_bg_opa(homeIslModuleTa, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(homeIslModuleTa, 0, 0);
  lv_obj_set_style_radius(homeIslModuleTa, 10, 0);
  lv_obj_set_style_text_color(homeIslModuleTa, lv_color_hex(UI_TEXT), 0);

  makeQuietButton(card, "취소", 452, 144, 100, 40, home_code_cancel_event_cb);
  makePrimaryButton(card, "저장", 562, 144, 110, 40, UI_ACCENT, home_code_confirm_event_cb);
}

void createHomeUi()
{
  homeScreen = lv_obj_create(NULL);
  lv_obj_set_size(homeScreen, LCD_H_RES, LCD_V_RES);
  lv_obj_set_style_text_font(homeScreen, FONT_KR, 0);
  lv_obj_set_style_bg_color(homeScreen, lv_color_hex(UI_BG), 0);
  lv_obj_set_style_bg_opa(homeScreen, LV_OPA_COVER, 0);
  lv_obj_clear_flag(homeScreen, LV_OBJ_FLAG_SCROLLABLE);

  createStatusBar(
    homeScreen,
    "개운중학교",
    &labelBarHomeTime,
    &labelBarHomeWifi,
    &labelBarHomeSd
  );

  // =====================================================
  // The home screen asks one question: which sensor?
  // Everything else — WiFi detail, upload mode, restart — lives one tap away
  // in Settings, so a student meets six choices instead of fifteen controls.
  // =====================================================
  makeHeading(homeScreen, "SENSOR", 28, 60, UI_TEXT);

  // Seven tiles on a 4x2 grid. Narrower than the old 3x2 at 231 px, but still
  // twice the width of a fingertip, and it keeps the grid to two rows so the
  // group code and 측정 시작 stay where they were.
  const int tileW = 231;
  const int tileH = 112;
  const int tileGapX = 14;
  const int tileGapY = 14;
  const int tileLeft = 28;
  const int tileTop = 128;
  const int tileCols = 4;

  struct SensorTileSpec
  {
    const char *name;
    const char *unit;
    int mode;
    lv_event_cb_t cb;
  };

  // Named by the quantity measured, not the part number: a student reads
  // "이산화탄소", not "SCD41".
  const SensorTileSpec tiles[HOME_SENSOR_TILE_COUNT] = {
    { "온도 · 기압", "°C · hPa", SENSOR_MODE_DPS310,  home_sensor_dps_event_cb },
    { "수온",        "°C",       SENSOR_MODE_DS18B20, home_sensor_water_event_cb },
    { "이산화탄소",  "ppm",      SENSOR_MODE_SCD41,   home_sensor_co2_event_cb },
    { "조도",        "lx",       SENSOR_MODE_TSL2591, home_sensor_light_event_cb },
    { "정밀 온도",   "°C",       SENSOR_MODE_TMP117,  home_sensor_tmp117_event_cb },
    { "거리",        "mm",       SENSOR_MODE_VL53L1X, home_sensor_vl53_event_cb },
    { "전압 · 전류", "V · A · W", SENSOR_MODE_INA228, home_sensor_ina228_event_cb },
    { "회전",        "° · °/s",   SENSOR_MODE_ENCODER, home_sensor_encoder_event_cb }
  };

  for (int i = 0; i < HOME_SENSOR_TILE_COUNT; i++)
  {
    const int col = i % tileCols;
    const int row = i / tileCols;

    homeSensorTileModes[i] = tiles[i].mode;
    homeSensorTiles[i] = makeSensorTile(
      homeScreen,
      tiles[i].name,
      tiles[i].unit,
      sensorDetailText(tiles[i].mode),
      tileLeft + col * (tileW + tileGapX),
      tileTop + row * (tileH + tileGapY),
      tileW,
      tileH,
      activeSensorMode == tiles[i].mode,
      tiles[i].cb,
      &homeSensorTileMarks[i]
    );
  }

  // The selected tile already carries 선택됨, so no "선택: ..." line here.
  labelHomeSensorMode = NULL;

  labelHomeSensorStatus = makeSmallLabel(homeScreen, "상태: 준비", 604, 388, UI_TEXT_2);
  lv_obj_set_width(labelHomeSensorStatus, 392);
  lv_obj_set_style_text_align(labelHomeSensorStatus, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(labelHomeSensorStatus, LV_LABEL_LONG_CLIP);

  // =====================================================
  // Group code: show the saved value, and only raise the keyboard on 변경.
  // The old screen kept an always-open text field plus a save button.
  // =====================================================
  // Resting state: the saved code is read-only text. Editing happens in a
  // modal sheet, opened by 변경.
  lv_obj_t *codePanel = makePanel(homeScreen, 28, 416, 604, 72);

  makeSmallLabel(codePanel, "모둠코드", 18, 12, UI_TEXT_3);

  labelHomeModumCode = makeLabel(codePanel, "미입력", 18, 34, UI_TEXT);
  lv_obj_set_width(labelHomeModumCode, 448);
  lv_label_set_long_mode(labelHomeModumCode, LV_LABEL_LONG_CLIP);

  makeQuietButton(codePanel, "변경", 486, 18, 100, 38, home_code_open_event_cb);

  // The one primary action on this screen.
  makePrimaryButton(homeScreen, "측정 시작", 652, 416, 344, 72, UI_ACCENT, go_measure_event_cb);

  // Upload mode and queue counts belong in Settings, not on the home screen.
  labelCloudMode = NULL;

  createHomeCodeEditor();

  homeKeyboard = lv_keyboard_create(homeScreen);
  lv_obj_set_size(homeKeyboard, LCD_H_RES, 220);
  lv_obj_align(homeKeyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_add_flag(homeKeyboard, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_event_cb(homeKeyboard, dashboard_keyboard_event_cb, LV_EVENT_ALL, NULL);
  lv_obj_set_style_text_font(homeKeyboard, &lv_font_montserrat_16, 0);
  lv_btnmatrix_set_btn_ctrl_all(homeKeyboard, LV_BTNMATRIX_CTRL_NO_REPEAT);

  createTabBar(homeScreen, 0);

  updateCloudModeLabel();
  updateHomeWifiLabels();
  refreshHomeSensorLabels();
  refreshHomeModumCodeLabel();
}

// =====================================================
// Measure UI
// =====================================================
void createMeasureUi()
{
  measureScreen = lv_obj_create(NULL);
  lv_obj_set_size(measureScreen, LCD_H_RES, LCD_V_RES);
  lv_obj_set_style_text_font(measureScreen, FONT_KR, 0);
  lv_obj_set_style_bg_color(measureScreen, lv_color_hex(UI_BG), 0);
  lv_obj_set_style_bg_opa(measureScreen, LV_OPA_COVER, 0);
  lv_obj_clear_flag(measureScreen, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scrollbar_mode(measureScreen, LV_SCROLLBAR_MODE_OFF);

  createStatusBar(
    measureScreen,
    "",
    &labelBarMeasureTime,
    &labelBarMeasureWifi,
    &labelBarMeasureSd
  );

  // =====================================================
  // Two columns: the reading and its history on the left, the session facts
  // and the one destructive control on the right. The value is the largest
  // thing on the screen because it is the only thing a student is watching.
  // =====================================================
  const int colLeftX = 24;
  const int colLeftW = 700;
  const int railX = 740;
  const int railW = 260;
  const int contentTop = 58;

  // ---- sensor name and live state ----
  labelMeasureSensorName = makeHeading(measureScreen, "온도 · 기압", colLeftX, contentTop, UI_TEXT);

  labelStatus = lv_label_create(measureScreen);
  lv_label_set_text(labelStatus, "준비");
  lv_obj_set_style_text_color(labelStatus, lv_color_hex(UI_OK), 0);
  lv_obj_set_style_text_font(labelStatus, FONT_KR_SMALL, 0);
  lv_obj_set_width(labelStatus, 300);
  lv_obj_set_style_text_align(labelStatus, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(labelStatus, LV_LABEL_LONG_CLIP);
  lv_obj_align(labelStatus, LV_ALIGN_TOP_LEFT, colLeftX + colLeftW - 300, contentTop + 8);

  // ---- the readings ----
  // Three columns are always built; refreshMeasureValueLayout() sizes and
  // hides them to match whatever the active sensor reports.
  for (int i = 0; i < MEASURE_VALUE_MAX; i++)
  {
    labelValueCaption[i] = makeSmallLabel(measureScreen, "", colLeftX, contentTop + 34, UI_TEXT_3);
    lv_label_set_long_mode(labelValueCaption[i], LV_LABEL_LONG_CLIP);

    labelValueNumber[i] = lv_label_create(measureScreen);
    lv_label_set_text(labelValueNumber[i], "--");
    lv_obj_set_style_text_color(labelValueNumber[i], lv_color_hex(UI_TEXT), 0);
    lv_obj_set_style_text_font(labelValueNumber[i], FONT_VALUE, 0);

    labelValueUnit[i] = makeHeading(measureScreen, "", 0, 0, UI_TEXT_3);
  }

  // The older update paths still write through these names.
  labelTempBig = labelValueNumber[0];
  labelPressureBig = labelValueNumber[1];
  labelHumidityBig = labelValueNumber[2];

  // =====================================================
  // Chart
  // =====================================================
  const int chartCardY = contentTop + 152;
  const int chartCardH = 530 - chartCardY;

  lv_obj_t *chartCard = makePanel(measureScreen, colLeftX, chartCardY, colLeftW, chartCardH);

  const int gChartX = 62;
  const int gChartY = 34;
  const int gChartW = 556;
  const int gChartH = chartCardH - 82;

  labelMeasureTempAxisTitle = makeSmallLabel(chartCard, "온도(℃)", 14, 10, UI_ACCENT);
  labelMeasurePressureAxisTitle = makeSmallLabel(chartCard, "기압", gChartX + gChartW + 8, 10, 0xB7791F);

  chart = lv_chart_create(chartCard);
  lv_obj_set_size(chart, gChartW, gChartH);
  lv_obj_align(chart, LV_ALIGN_TOP_LEFT, gChartX, gChartY);
  lv_obj_clear_flag(chart, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scrollbar_mode(chart, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_style_bg_opa(chart, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(chart, 0, 0);
  lv_obj_set_style_pad_all(chart, 0, 0);
  lv_obj_set_style_line_color(chart, lv_color_hex(0xEDF1F6), LV_PART_MAIN);
  lv_obj_set_style_line_width(chart, 1, LV_PART_MAIN);
  lv_obj_set_style_size(chart, 0, LV_PART_INDICATOR);

  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(chart, CHART_POINTS);
  lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_SHIFT);
  lv_chart_set_div_line_count(chart, 5, 0);

  lv_chart_set_axis_tick(chart, LV_CHART_AXIS_PRIMARY_X, 0, 0, 0, 0, false, 0);
  lv_chart_set_axis_tick(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 0, 0, 0, false, 0);
  lv_chart_set_axis_tick(chart, LV_CHART_AXIS_SECONDARY_Y, 0, 0, 0, 0, false, 0);

  seriesTemp = lv_chart_add_series(chart, lv_color_hex(UI_ACCENT), LV_CHART_AXIS_PRIMARY_Y);
  seriesPressure = lv_chart_add_series(chart, lv_color_hex(0xB7791F), LV_CHART_AXIS_SECONDARY_Y);

  for (int i = 0; i < 6; i++)
  {
    const int tickX = gChartX + (gChartW * i) / 5;
    const int tickY = gChartY + gChartH - (gChartH * i) / 5;

    labelMeasureTimeTicks[i] = makeSmallLabel(chartCard, "--", tickX - 40, gChartY + gChartH + 8, UI_TEXT_3);
    lv_obj_set_width(labelMeasureTimeTicks[i], 80);
    lv_obj_set_style_text_align(labelMeasureTimeTicks[i], LV_TEXT_ALIGN_CENTER, 0);

    labelMeasureTempTicks[i] = makeSmallLabel(chartCard, "--", 6, tickY - 9, UI_TEXT_3);
    lv_obj_set_width(labelMeasureTempTicks[i], 50);
    lv_obj_set_style_text_align(labelMeasureTempTicks[i], LV_TEXT_ALIGN_RIGHT, 0);

    labelMeasurePressureTicks[i] = makeSmallLabel(chartCard, "--", gChartX + gChartW + 6, tickY - 9, UI_TEXT_3);
    lv_obj_set_width(labelMeasurePressureTicks[i], 70);
    lv_obj_set_style_text_align(labelMeasurePressureTicks[i], LV_TEXT_ALIGN_LEFT, 0);
  }

  labelGraphStart = makeSmallLabel(chartCard, "시간(s)", gChartX + gChartW / 2 - 50, chartCardH - 24, UI_TEXT_3);
  lv_obj_set_width(labelGraphStart, 100);
  lv_obj_set_style_text_align(labelGraphStart, LV_TEXT_ALIGN_CENTER, 0);

  labelGraphEnd = makeSmallLabel(chartCard, "", 0, 0, UI_TEXT_3);
  lv_obj_add_flag(labelGraphEnd, LV_OBJ_FLAG_HIDDEN);

  // =====================================================
  // Session rail
  // =====================================================
  lv_obj_t *elapsedCard = makePanel(measureScreen, railX, contentTop, 126, 70);
  makeSmallLabel(elapsedCard, "경과 시간", 12, 10, UI_TEXT_3);
  labelRuntime = makeLabel(elapsedCard, "00:00", 12, 32, UI_TEXT);
  lv_obj_set_width(labelRuntime, 102);
  lv_label_set_long_mode(labelRuntime, LV_LABEL_LONG_CLIP);

  lv_obj_t *countCard = makePanel(measureScreen, railX + 134, contentTop, 126, 70);
  makeSmallLabel(countCard, "모은 값", 12, 10, UI_TEXT_3);
  labelCount = makeLabel(countCard, "0", 12, 32, UI_TEXT);
  lv_obj_set_width(labelCount, 102);
  lv_label_set_long_mode(labelCount, LV_LABEL_LONG_CLIP);

  // Where the data has actually reached: the cloud, the card, the group.
  lv_obj_t *sendCard = makePanel(measureScreen, railX, contentTop + 82, railW, 142);
  makeSmallLabel(sendCard, "전송 상태", 14, 10, UI_TEXT_3);

  makeSmallLabel(sendCard, "과학실 ON", 14, 40, UI_TEXT_2);
  labelMeasureIslState = makeSmallLabel(sendCard, "대기", 140, 40, UI_TEXT_3);
  lv_obj_set_width(labelMeasureIslState, 106);
  lv_obj_set_style_text_align(labelMeasureIslState, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(labelMeasureIslState, LV_LABEL_LONG_CLIP);

  makeSmallLabel(sendCard, "SD 카드", 14, 66, UI_TEXT_2);
  labelSd = makeSmallLabel(sendCard, "--", 140, 66, UI_TEXT_3);
  lv_obj_set_width(labelSd, 106);
  lv_obj_set_style_text_align(labelSd, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(labelSd, LV_LABEL_LONG_CLIP);

  makeSmallLabel(sendCard, "모둠", 14, 92, UI_TEXT_2);
  labelMeasureModum = makeSmallLabel(sendCard, "--", 140, 92, UI_TEXT_3);
  lv_obj_set_width(labelMeasureModum, 106);
  lv_obj_set_style_text_align(labelMeasureModum, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(labelMeasureModum, LV_LABEL_LONG_CLIP);

  makeSmallLabel(sendCard, "블루투스", 14, 118, UI_TEXT_2);
  labelMeasureBle = makeSmallLabel(sendCard, "--", 140, 118, UI_TEXT_3);
  lv_obj_set_width(labelMeasureBle, 106);
  lv_obj_set_style_text_align(labelMeasureBle, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(labelMeasureBle, LV_LABEL_LONG_CLIP);

  // Secondary actions, kept quiet so they do not compete with 시작/정지.
  makeQuietButton(measureScreen, "일괄전송", railX, contentTop + 236, 126, 44, dashboard_cloud_batch_upload_event_cb);
  makeQuietButton(measureScreen, "초기화", railX + 134, contentTop + 236, 126, 44, clear_event_cb);

  // The primary control. Start is the accent; stop is the only red on screen,
  // because stopping is the only thing here that cannot be undone.
  btnMeasureStart = makePrimaryButton(measureScreen, "측정 시작", railX, 466, railW, 64, UI_ACCENT, start_event_cb);
  btnMeasureStop = makePrimaryButton(measureScreen, "측정 정지", railX, 466, railW, 64, UI_DANGER, stop_event_cb);
  lv_obj_add_flag(btnMeasureStop, LV_OBJ_FLAG_HIDDEN);

  createTabBar(measureScreen, 1);

  // Aliases kept for the older update paths.
  labelNow = labelDateTime;
  labelSample = labelCount;
  labelElapsed = labelRuntime;
  labelTempNow = labelTempBig;
  labelPressureNow = labelPressureBig;

  resetTable();
  clearChart();
  updateActiveSensorUiLabels();
  refreshMeasureControls();
}



// =====================================================
// =====================================================
// =====================================================
// ISL UI
// =====================================================
void createIslUi()
{
  islScreen = lv_obj_create(NULL);
  lv_obj_set_size(islScreen, LCD_H_RES, LCD_V_RES);
  lv_obj_set_style_text_font(islScreen, FONT_KR, 0);
  lv_obj_set_style_bg_color(islScreen, lv_color_hex(UI_BG), 0);
  lv_obj_set_style_bg_opa(islScreen, LV_OPA_COVER, 0);
  lv_obj_clear_flag(islScreen, LV_OBJ_FLAG_SCROLLABLE);

  createStatusBar(
    islScreen,
    "",
    &labelBarIslTime,
    &labelBarIslWifi,
    &labelBarIslSd
  );

  makeHeading(islScreen, "전송 설정", 28, 58, UI_TEXT);
  makeSmallLabel(islScreen, "지능형 과학실 ON으로 측정값을 보내기 위한 값입니다.", 28, 92, UI_TEXT_3);

  lv_obj_t *card = makePanel(islScreen, 28, 124, 968, 250);

  makeSmallLabel(card, "인증키 (serviceKey)", 24, 18, UI_TEXT_3);

  islServiceKeyTa = lv_textarea_create(card);
  lv_obj_set_size(islServiceKeyTa, 920, 46);
  lv_obj_align(islServiceKeyTa, LV_ALIGN_TOP_LEFT, 24, 42);
  lv_textarea_set_one_line(islServiceKeyTa, true);
  lv_textarea_set_password_mode(islServiceKeyTa, true);
  lv_textarea_set_placeholder_text(islServiceKeyTa, "발급받은 인증키");
  lv_obj_set_style_text_font(islServiceKeyTa, FONT_KR, 0);
  lv_obj_set_style_bg_color(islServiceKeyTa, lv_color_hex(UI_BG), 0);
  lv_obj_set_style_bg_opa(islServiceKeyTa, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(islServiceKeyTa, 0, 0);
  lv_obj_set_style_radius(islServiceKeyTa, 10, 0);
  lv_obj_set_style_text_color(islServiceKeyTa, lv_color_hex(UI_TEXT), 0);
  lv_obj_add_event_cb(islServiceKeyTa, dashboard_textarea_event_cb, LV_EVENT_FOCUSED, NULL);

  if (strlen(islServiceKey) > 0 && strcmp(islServiceKey, "PUT_YOUR_SERVICE_KEY") != 0)
  {
    lv_textarea_set_text(islServiceKeyTa, islServiceKey);
  }

  makeSmallLabel(card, "모둠코드", 24, 104, UI_TEXT_3);

  islModuleTa = lv_textarea_create(card);
  lv_obj_set_size(islModuleTa, 560, 46);
  lv_obj_align(islModuleTa, LV_ALIGN_TOP_LEFT, 24, 128);
  lv_textarea_set_one_line(islModuleTa, true);
  lv_textarea_set_password_mode(islModuleTa, false);
  lv_textarea_set_placeholder_text(islModuleTa, "예: ON040000093851");
  lv_obj_set_style_text_font(islModuleTa, FONT_KR, 0);
  lv_obj_set_style_bg_color(islModuleTa, lv_color_hex(UI_BG), 0);
  lv_obj_set_style_bg_opa(islModuleTa, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(islModuleTa, 0, 0);
  lv_obj_set_style_radius(islModuleTa, 10, 0);
  lv_obj_set_style_text_color(islModuleTa, lv_color_hex(UI_TEXT), 0);
  lv_obj_add_event_cb(islModuleTa, dashboard_textarea_event_cb, LV_EVENT_FOCUSED, NULL);

  if (strlen(islRuntimeSerialNumber) > 0)
  {
    lv_textarea_set_text(islModuleTa, islRuntimeSerialNumber);
  }

  makePrimaryButton(card, "저장", 600, 128, 150, 46, UI_ACCENT, dashboard_isl_save_event_cb);
  makeQuietButton(card, "전송 시작", 760, 128, 184, 46, dashboard_isl_start_event_cb);

  makeSmallLabel(card, "전송 방식", 24, 194, UI_TEXT_3);
  makeQuietButton(card, "실시간", 120, 186, 130, 40, dashboard_cloud_realtime_event_cb);
  makeQuietButton(card, "일괄전송", 260, 186, 150, 40, dashboard_cloud_batch_event_cb);
  makeQuietButton(card, "전송 중지", 420, 186, 150, 40, dashboard_isl_stop_event_cb);

  labelCloudMode = makeSmallLabel(card, "", 588, 198, UI_TEXT_3);
  lv_obj_set_width(labelCloudMode, 356);
  lv_obj_set_style_text_align(labelCloudMode, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(labelCloudMode, LV_LABEL_LONG_CLIP);

  lv_obj_t *statusCard = makePanel(islScreen, 28, 390, 968, 100);

  makeSmallLabel(statusCard, "상태", 24, 16, UI_TEXT_3);

  labelHomeIsl = makeLabel(statusCard, "API: 대기", 24, 44, UI_ACCENT);
  lv_obj_set_width(labelHomeIsl, 920);
  lv_label_set_long_mode(labelHomeIsl, LV_LABEL_LONG_CLIP);

  islKeyboard = lv_keyboard_create(islScreen);
  lv_obj_set_size(islKeyboard, LCD_H_RES, 220);
  lv_obj_align(islKeyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_add_flag(islKeyboard, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_event_cb(islKeyboard, dashboard_keyboard_event_cb, LV_EVENT_ALL, NULL);
  lv_obj_set_style_text_font(islKeyboard, &lv_font_montserrat_16, 0);
  lv_btnmatrix_set_btn_ctrl_all(islKeyboard, LV_BTNMATRIX_CTRL_NO_REPEAT);

  createTabBar(islScreen, 3);

  updateCloudModeLabel();
  updateIslStatusLabels();
}


// =====================================================
// CSV/TXT File Viewer UI
// =====================================================
void createFileViewerUi()
{
  fileViewerScreen = lv_obj_create(NULL);
  lv_obj_set_size(fileViewerScreen, LCD_H_RES, LCD_V_RES);
  lv_obj_set_style_text_font(fileViewerScreen, FONT_KR, 0);
  lv_obj_set_style_bg_color(fileViewerScreen, lv_color_hex(UI_BG), 0);
  lv_obj_set_style_bg_opa(fileViewerScreen, LV_OPA_COVER, 0);
  lv_obj_clear_flag(fileViewerScreen, LV_OBJ_FLAG_SCROLLABLE);

  createStatusBar(
    fileViewerScreen,
    "",
    &labelBarFileViewerTime,
    &labelBarFileViewerWifi,
    &labelBarFileViewerSd
  );

  labelFileViewerTitle = makeHeading(fileViewerScreen, "파일 보기", 28, 58, UI_TEXT);
  lv_obj_set_width(labelFileViewerTitle, 700);
  lv_label_set_long_mode(labelFileViewerTitle, LV_LABEL_LONG_CLIP);

  makeQuietButton(fileViewerScreen, "이전", 700, 60, 92, 40, csv_viewer_prev_event_cb);
  makeQuietButton(fileViewerScreen, "다음", 800, 60, 92, 40, csv_viewer_next_event_cb);
  makeQuietButton(fileViewerScreen, "기록", 900, 60, 96, 40, go_csv_event_cb);

  lv_obj_t *card = makePanel(fileViewerScreen, 28, 110, 968, 372);

  // Read-only: this is a viewer, and an editable field invites a keyboard the
  // screen has no room for.
  fileViewerTextArea = lv_textarea_create(card);
  lv_obj_set_size(fileViewerTextArea, 940, 344);
  lv_obj_align(fileViewerTextArea, LV_ALIGN_TOP_LEFT, 14, 14);
  lv_textarea_set_one_line(fileViewerTextArea, false);
  lv_textarea_set_text(fileViewerTextArea, "기록 화면에서 파일을 열면 여기에 표시됩니다.");
  lv_textarea_set_cursor_click_pos(fileViewerTextArea, false);
  lv_obj_set_style_text_font(fileViewerTextArea, FONT_KR_SMALL, 0);
  lv_obj_set_style_bg_opa(fileViewerTextArea, LV_OPA_TRANSP, 0);
  lv_obj_set_style_text_color(fileViewerTextArea, lv_color_hex(UI_TEXT), 0);
  lv_obj_set_style_border_width(fileViewerTextArea, 0, 0);
  lv_obj_set_style_pad_all(fileViewerTextArea, 0, 0);
  lv_obj_set_scrollbar_mode(fileViewerTextArea, LV_SCROLLBAR_MODE_AUTO);

  labelFileViewerPageInfo = makeSmallLabel(fileViewerScreen, "0 / 0 bytes", 28, 494, UI_TEXT_3);
  lv_obj_set_width(labelFileViewerPageInfo, 968);
  lv_label_set_long_mode(labelFileViewerPageInfo, LV_LABEL_LONG_CLIP);

  createTabBar(fileViewerScreen, 2);
}

// =====================================================
// CSV UI
// =====================================================
void createCsvUi()
{
  csvScreen = lv_obj_create(NULL);
  lv_obj_set_size(csvScreen, LCD_H_RES, LCD_V_RES);
  lv_obj_set_style_text_font(csvScreen, FONT_KR, 0);
  lv_obj_set_style_bg_color(csvScreen, lv_color_hex(UI_BG), 0);
  lv_obj_set_style_bg_opa(csvScreen, LV_OPA_COVER, 0);
  lv_obj_clear_flag(csvScreen, LV_OBJ_FLAG_SCROLLABLE);

  createStatusBar(
    csvScreen,
    "",
    &labelBarCsvTime,
    &labelBarCsvWifi,
    &labelBarCsvSd
  );

  makeHeading(csvScreen, "기록", 28, 58, UI_TEXT);

  // =====================================================
  // The measurement table lives here rather than on the measurement screen.
  // During an experiment the value and its graph are what matter; the numbers
  // are what you go back and read afterwards, which is this screen's job.
  // =====================================================
  lv_obj_t *tableCard = makePanel(csvScreen, 28, 104, 620, 412);

  tableData = lv_table_create(tableCard);
  lv_obj_set_size(tableData, 596, 346);
  lv_obj_align(tableData, LV_ALIGN_TOP_LEFT, 12, 12);

  lv_obj_set_style_text_font(tableData, FONT_KR, LV_PART_MAIN);
  lv_obj_set_style_text_font(tableData, FONT_KR, LV_PART_ITEMS);
  lv_obj_set_style_text_color(tableData, lv_color_hex(UI_TEXT), LV_PART_ITEMS);
  lv_obj_set_style_text_align(tableData, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_align(tableData, LV_TEXT_ALIGN_CENTER, LV_PART_ITEMS);
  lv_obj_set_style_bg_opa(tableData, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(tableData, 0, LV_PART_MAIN);
  lv_obj_set_style_border_color(tableData, lv_color_hex(UI_LINE), LV_PART_ITEMS);
  lv_obj_set_style_border_width(tableData, 1, LV_PART_ITEMS);
  lv_obj_set_style_border_side(tableData, LV_BORDER_SIDE_BOTTOM, LV_PART_ITEMS);
  lv_obj_set_style_pad_ver(tableData, 4, LV_PART_ITEMS);
  lv_obj_set_style_pad_hor(tableData, 0, LV_PART_ITEMS);

  lv_obj_clear_flag(tableData, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scrollbar_mode(tableData, LV_SCROLLBAR_MODE_OFF);

  lv_table_set_col_cnt(tableData, 4);
  lv_table_set_row_cnt(tableData, TABLE_VISIBLE_ROWS + 1);

  lv_table_set_col_width(tableData, 0, 70);   // No
  lv_table_set_col_width(tableData, 1, 120);  // 시간(s)
  lv_table_set_col_width(tableData, 2, 200);  // primary
  lv_table_set_col_width(tableData, 3, 200);  // secondary

  makeQuietButton(tableCard, "이전", 12, 366, 140, 40, table_older_event_cb);
  makeQuietButton(tableCard, "다음", 160, 366, 140, 40, table_newer_event_cb);

  labelCsvRowRange = makeSmallLabel(tableCard, "", 316, 378, UI_TEXT_3);
  lv_obj_set_width(labelCsvRowRange, 290);
  lv_obj_set_style_text_align(labelCsvRowRange, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(labelCsvRowRange, LV_LABEL_LONG_CLIP);

  // =====================================================
  // File side
  // =====================================================
  lv_obj_t *fileCard = makePanel(csvScreen, 668, 104, 328, 412);

  // SD is not mounted at boot: SDMMC can collide with the C6 SDIO link that
  // WiFi and Bluetooth ride on, so it is brought up on demand. The redesign
  // dropped this control, which left no way to mount the card at all.
  makeSmallLabel(fileCard, "SD 카드", 16, 14, UI_TEXT_3);

  labelHomeCsv = makeSmallLabel(fileCard, "SD: 마운트 안 됨", 16, 36, UI_TEXT_2);
  lv_obj_set_width(labelHomeCsv, 296);
  lv_label_set_long_mode(labelHomeCsv, LV_LABEL_LONG_CLIP);

  makeQuietButton(fileCard, "마운트", 16, 62, 142, 40, dashboard_csv_mount_event_cb);
  makeQuietButton(fileCard, "자동 저장", 170, 62, 142, 40, dashboard_csv_toggle_event_cb);

  makeSmallLabel(fileCard, "저장 파일", 16, 118, UI_TEXT_3);

  homeCsvFileTa = lv_textarea_create(fileCard);
  lv_obj_set_size(homeCsvFileTa, 296, 44);
  lv_obj_align(homeCsvFileTa, LV_ALIGN_TOP_LEFT, 16, 140);
  lv_textarea_set_one_line(homeCsvFileTa, true);
  lv_textarea_set_placeholder_text(homeCsvFileTa, "dps310_log.csv");
  lv_obj_set_style_text_font(homeCsvFileTa, FONT_KR, 0);
  lv_obj_set_style_bg_color(homeCsvFileTa, lv_color_hex(UI_BG), 0);
  lv_obj_set_style_bg_opa(homeCsvFileTa, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(homeCsvFileTa, 0, 0);
  lv_obj_set_style_radius(homeCsvFileTa, 10, 0);
  lv_obj_set_style_text_color(homeCsvFileTa, lv_color_hex(UI_TEXT), 0);
  lv_obj_add_event_cb(homeCsvFileTa, csv_textarea_event_cb, LV_EVENT_FOCUSED, NULL);
  lv_textarea_set_text(homeCsvFileTa, csvFileName);

  makePrimaryButton(fileCard, "측정값 저장", 16, 192, 296, 44, UI_ACCENT, csv_save_measurement_data_event_cb);

  makeQuietButton(fileCard, "미리보기", 16, 244, 142, 40, dashboard_csv_preview_event_cb);
  makeQuietButton(fileCard, "파일 목록", 170, 244, 142, 40, settings_sd_list_event_cb);

  labelSelectedSdFile = makeSmallLabel(fileCard, "선택: 없음", 16, 292, UI_TEXT_3);
  lv_obj_set_width(labelSelectedSdFile, 296);
  lv_label_set_long_mode(labelSelectedSdFile, LV_LABEL_LONG_CLIP);

  sdFileList = lv_list_create(fileCard);
  lv_obj_set_size(sdFileList, 296, 88);
  lv_obj_align(sdFileList, LV_ALIGN_TOP_LEFT, 16, 316);
  lv_obj_set_style_text_font(sdFileList, FONT_KR_SMALL, 0);
  lv_obj_set_style_bg_opa(sdFileList, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(sdFileList, 0, 0);
  lv_obj_set_style_pad_all(sdFileList, 0, 0);
  lv_list_add_text(sdFileList, "SD 파일 목록 대기");

  // The preview is only meaningful once a file is opened, so it sits behind
  // 미리보기 rather than taking permanent space.
  labelHomeCsvPreview = makeSmallLabel(csvScreen, "", 28, 522, UI_TEXT_3);
  lv_obj_set_width(labelHomeCsvPreview, 968);
  lv_label_set_long_mode(labelHomeCsvPreview, LV_LABEL_LONG_CLIP);

  csvKeyboard = lv_keyboard_create(csvScreen);
  lv_obj_set_size(csvKeyboard, LCD_H_RES, 220);
  lv_obj_align(csvKeyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_add_flag(csvKeyboard, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_event_cb(csvKeyboard, csv_keyboard_event_cb, LV_EVENT_ALL, NULL);
  lv_obj_set_style_text_font(csvKeyboard, &lv_font_montserrat_16, 0);
  lv_btnmatrix_set_btn_ctrl_all(csvKeyboard, LV_BTNMATRIX_CTRL_NO_REPEAT);

  createTabBar(csvScreen, 2);

  resetTable();
  updateTable();
  updateDashboardCsvLabels();
}

// =====================================================
// Settings UI
// =====================================================
static void go_settings_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  requestScreenSwitch(settingsScreen);
}

// =====================================================
// Bluetooth screen
// Lists what the board can see. Until a sensor's advertised service UUID is
// known there is nothing to connect to, so finding and naming devices comes
// before pairing with them.
// =====================================================
static void ble_enable_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  bleEnabled = !bleEnabled;
  saveBleEnabled(bleEnabled);

  // NimBLE cannot be torn down cleanly once the host task is running, so the
  // change takes effect on the next boot rather than pretending otherwise.
  if (labelBleEnabledState)
  {
    lv_label_set_text(
      labelBleEnabledState,
      bleEnabled ? "블루투스: 켜짐 (다시 시작하면 적용)"
                 : "블루투스: 꺼짐 (다시 시작하면 적용)"
    );
  }
}

static void i2c_scan_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (measuring) { setIslStatusText("측정 정지 후 센서 검색"); return; }

  pendingI2cScan = true;
}

static void ble_row_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  const int index = (int)(intptr_t)lv_event_get_user_data(e);
  if (index < 0 || index >= BLE_SCAN_MAX_RESULTS) return;

  pendingBleConnectIndex = index;
}

static void ble_disconnect_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  pendingBleDisconnect = true;
}

static void ble_use_sensor_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  if (!bleLinkIsSubscribed())
  {
    setIslStatusText("먼저 센서 노드에 연결하세요");
    return;
  }

  pendingSensorMode = SENSOR_MODE_BLE;
  requestScreenSwitch(measureScreen);
}

static void ble_scan_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (bleScanIsRunning()) return;

  pendingBleScan = true;
}

static void go_ble_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  requestScreenSwitch(bleScreen);
}

void refreshBleScreen()
{
  if (bleList == NULL) return;

  if (labelBleLink)
  {
    char text[160];

    if (bleLinkIsSubscribed())
    {
      float value = NAN;
      char quantity[24] = "";
      char unit[24] = "";
      uint32_t ageMs = 0;

      if (bleLinkLatestReading(&value, quantity, unit, &ageMs))
      {
        snprintf(text, sizeof(text), "%s · %s · %s %.4g%s (%lu초 전)",
                 bleLinkPeerName(), bleLinkStateText(), quantity, value, unit,
                 (unsigned long)(ageMs / 1000));
      }
      else
      {
        snprintf(text, sizeof(text), "%s · %s · 아직 값 없음",
                 bleLinkPeerName(), bleLinkStateText());
      }
    }
    else if (bleLinkPeerName()[0])
    {
      snprintf(text, sizeof(text), "%s · %s", bleLinkPeerName(), bleLinkStateText());
    }
    else
    {
      snprintf(text, sizeof(text), "%s", bleLinkStateText());
    }

    lv_label_set_text(labelBleLink, text);
  }

  if (labelBleState)
  {
    if (!bleEnabled)
    {
      lv_label_set_text(labelBleState, "블루투스가 꺼져 있습니다.");
    }
    else if (bleScanIsRunning())
    {
      lv_label_set_text(labelBleState, "검색 중...");
    }
    else
    {
      char text[96];
      const int candidates = bleScanCandidateCount();
      snprintf(
        text,
        sizeof(text),
        "센서 후보 %d개 · 이름 없는 기기 %d개 · 내 이름 %s",
        candidates,
        bleScanResultCount() - candidates,
        bleAdvertisedName()
      );
      lv_label_set_text(labelBleState, text);
    }
  }

  // Rebuild only when the count changed, so the list does not flicker while a
  // scan is filling in.
  static int lastShownCount = -1;
  static bool lastScanning = false;
  const int count = bleScanResultCount();
  const bool scanning = bleScanIsRunning();

  if (count == lastShownCount && scanning == lastScanning) return;
  lastShownCount = count;
  lastScanning = scanning;

  lv_obj_clean(bleList);

  if (count == 0)
  {
    lv_obj_t *empty = lv_label_create(bleList);
    lv_label_set_text(empty, scanning ? "검색 중입니다." : "찾은 기기가 없습니다. 센서 전원을 켜고 다시 검색하세요.");
    lv_obj_set_style_text_font(empty, FONT_KR_SMALL, 0);
    lv_obj_set_style_text_color(empty, lv_color_hex(UI_TEXT_3), 0);
    return;
  }

  // Anonymous devices are collapsed into one line: thirteen rows of random
  // addresses hide the one row that matters.
  int anonymous = 0;
  int rowCount = 0;

  for (int i = 0; i < count; i++)
  {
    BleScanResult r;
    if (!bleScanResultAt(i, &r)) continue;

    if (!bleScanResultIsCandidate(&r)) { anonymous++; continue; }

    lv_obj_t *row = lv_obj_create(bleList);
    lv_obj_set_size(row, 908, 52);

    // The row is the connect button: a separate one per row would not fit
    // beside the address and the signal strength.
    if (rowCount < BLE_SCAN_MAX_RESULTS)
    {
      snprintf(bleRowAddress[rowCount], sizeof(bleRowAddress[rowCount]), "%s", r.address);
      lv_obj_add_flag(row, LV_OBJ_FLAG_CLICKABLE);
      lv_obj_add_event_cb(row, ble_row_event_cb, LV_EVENT_CLICKED,
                          (void *)(intptr_t)rowCount);
      rowCount++;
    }
    lv_obj_set_style_bg_color(row, lv_color_hex(UI_SURFACE), 0);
    lv_obj_set_style_bg_opa(row, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(row, 10, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_shadow_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *name = lv_label_create(row);
    lv_label_set_text(name, r.name[0] ? r.name : "(이름 없음)");
    lv_obj_set_style_text_font(name, FONT_KR, 0);
    lv_obj_set_style_text_color(name, lv_color_hex(r.name[0] ? UI_TEXT : UI_TEXT_3), 0);
    lv_obj_align(name, LV_ALIGN_TOP_LEFT, 14, 6);

    // The service UUID is the field that identifies an unknown sensor, so it
    // is shown rather than hidden behind a detail view.
    lv_obj_t *detail = lv_label_create(row);
    char text[96];
    snprintf(text, sizeof(text), "%s  ·  %s", r.address,
             r.services[0] ? r.services : "서비스 광고 없음");
    lv_label_set_text(detail, text);
    lv_obj_set_style_text_font(detail, FONT_KR_SMALL, 0);
    lv_obj_set_style_text_color(detail, lv_color_hex(UI_TEXT_3), 0);
    lv_obj_align(detail, LV_ALIGN_TOP_LEFT, 14, 28);

    const bool isLinked =
      bleLinkIsSubscribed() && strcmp(bleLinkPeerName(), r.name[0] ? r.name : r.address) == 0;

    if (isLinked)
    {
      lv_obj_set_style_bg_color(row, lv_color_hex(UI_ACCENT_TINT), 0);
      lv_obj_set_style_border_color(row, lv_color_hex(UI_ACCENT), 0);
      lv_obj_set_style_border_width(row, 2, 0);
    }

    lv_obj_t *rssi = lv_label_create(row);
    snprintf(text, sizeof(text), "%s%d dBm", isLinked ? "연결됨  ·  " : "", r.rssi);
    lv_label_set_text(rssi, text);
    lv_obj_set_style_text_font(rssi, FONT_KR_SMALL, 0);
    lv_obj_set_style_text_color(rssi, lv_color_hex(UI_TEXT_2), 0);
    lv_obj_align(rssi, LV_ALIGN_RIGHT_MID, -14, 0);
  }

  if (anonymous > 0)
  {
    lv_obj_t *note = lv_label_create(bleList);
    char text[96];
    snprintf(
      text,
      sizeof(text),
      "이름도 서비스도 알리지 않는 기기 %d개는 숨겼습니다. 주변 휴대폰·노트북입니다.",
      anonymous
    );
    lv_label_set_text(note, text);
    lv_obj_set_style_text_font(note, FONT_KR_SMALL, 0);
    lv_obj_set_style_text_color(note, lv_color_hex(UI_TEXT_3), 0);
  }

  if (count > 0 && anonymous == count)
  {
    lv_obj_t *empty = lv_label_create(bleList);
    lv_label_set_text(empty, "연결할 수 있는 센서를 찾지 못했습니다. 센서 전원을 켜고 가까이에서 다시 검색하세요.");
    lv_obj_set_style_text_font(empty, FONT_KR_SMALL, 0);
    lv_obj_set_style_text_color(empty, lv_color_hex(UI_TEXT_2), 0);
  }
}

void createBleUi()
{
  bleScreen = lv_obj_create(NULL);
  lv_obj_set_size(bleScreen, LCD_H_RES, LCD_V_RES);
  lv_obj_set_style_text_font(bleScreen, FONT_KR, 0);
  lv_obj_set_style_bg_color(bleScreen, lv_color_hex(UI_BG), 0);
  lv_obj_set_style_bg_opa(bleScreen, LV_OPA_COVER, 0);
  lv_obj_clear_flag(bleScreen, LV_OBJ_FLAG_SCROLLABLE);

  createStatusBar(
    bleScreen,
    "",
    &labelBarBleTime,
    &labelBarBleWifi,
    &labelBarBleSd
  );

  makeHeading(bleScreen, "블루투스 센서", 28, 58, UI_TEXT);

  labelBleState = makeSmallLabel(bleScreen, "검색 전", 28, 94, UI_TEXT_3);
  lv_obj_set_width(labelBleState, 600);
  lv_label_set_long_mode(labelBleState, LV_LABEL_LONG_CLIP);

  makeQuietButton(bleScreen, "다시 검색", 828, 60, 168, 44, ble_scan_event_cb);
  makeQuietButton(bleScreen, "켜기/끄기", 648, 60, 168, 44, ble_enable_event_cb);

  labelBleLink = makeSmallLabel(bleScreen, "연결 안 됨", 28, 116, UI_TEXT_2);
  lv_obj_set_width(labelBleLink, 968);
  lv_label_set_long_mode(labelBleLink, LV_LABEL_LONG_CLIP);

  labelBleEnabledState = makeSmallLabel(bleScreen, "", 28, 138, UI_TEXT_2);
  lv_obj_set_width(labelBleEnabledState, 968);
  lv_label_set_long_mode(labelBleEnabledState, LV_LABEL_LONG_CLIP);

  bleList = lv_obj_create(bleScreen);
  lv_obj_set_size(bleList, 968, 300);
  lv_obj_align(bleList, LV_ALIGN_TOP_LEFT, 28, 166);
  lv_obj_set_style_bg_color(bleList, lv_color_hex(UI_BG), 0);
  lv_obj_set_style_bg_opa(bleList, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(bleList, 0, 0);
  lv_obj_set_style_radius(bleList, 0, 0);
  lv_obj_set_style_pad_all(bleList, 0, 0);
  lv_obj_set_style_pad_row(bleList, 8, 0);
  lv_obj_set_flex_flow(bleList, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_scrollbar_mode(bleList, LV_SCROLLBAR_MODE_AUTO);

  makeQuietButton(bleScreen, "연결 해제", 628, 474, 170, 44, ble_disconnect_event_cb);
  makePrimaryButton(bleScreen, "이 센서로 측정", 808, 474, 188, 44, UI_ACCENT, ble_use_sensor_event_cb);

  makeSmallLabel(
    bleScreen,
    "목록에서 센서를 누르면 연결합니다. 연결된 뒤 [이 센서로 측정]을 누르면 측정 화면으로 넘어갑니다.",
    28,
    486,
    UI_TEXT_3
  );

  lv_label_set_text(
    labelBleEnabledState,
    bleEnabled ? "블루투스: 켜짐" : "블루투스: 꺼짐 — 켜면 다음 시작부터 검색할 수 있습니다."
  );

  createTabBar(bleScreen, 3);
}

void createSettingsUi()
{
  settingsScreen = lv_obj_create(NULL);
  lv_obj_set_size(settingsScreen, LCD_H_RES, LCD_V_RES);
  lv_obj_set_style_text_font(settingsScreen, FONT_KR, 0);
  lv_obj_set_style_bg_color(settingsScreen, lv_color_hex(UI_BG), 0);
  lv_obj_set_style_bg_opa(settingsScreen, LV_OPA_COVER, 0);
  lv_obj_clear_flag(settingsScreen, LV_OBJ_FLAG_SCROLLABLE);

  createStatusBar(
    settingsScreen,
    "",
    &labelBarSettingsTime,
    &labelBarSettingsWifi,
    &labelBarSettingsSd
  );

  makeHeading(settingsScreen, "설정", 28, 58, UI_TEXT);

  // =====================================================
  // WiFi
  // =====================================================
  lv_obj_t *wifiCard = makePanel(settingsScreen, 28, 104, 620, 412);

  makeSmallLabel(wifiCard, "WiFi", 20, 16, UI_TEXT_3);

  wifiSsidTa = lv_textarea_create(wifiCard);
  lv_obj_set_size(wifiSsidTa, 580, 46);
  lv_obj_align(wifiSsidTa, LV_ALIGN_TOP_LEFT, 20, 40);
  lv_textarea_set_one_line(wifiSsidTa, true);
  lv_textarea_set_placeholder_text(wifiSsidTa, "네트워크 이름");
  lv_obj_set_style_text_font(wifiSsidTa, FONT_KR, 0);
  lv_obj_set_style_bg_color(wifiSsidTa, lv_color_hex(UI_BG), 0);
  lv_obj_set_style_bg_opa(wifiSsidTa, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(wifiSsidTa, 0, 0);
  lv_obj_set_style_radius(wifiSsidTa, 10, 0);
  lv_obj_set_style_text_color(wifiSsidTa, lv_color_hex(UI_TEXT), 0);
  lv_obj_add_event_cb(wifiSsidTa, wifi_textarea_event_cb, LV_EVENT_FOCUSED, NULL);

  wifiPassTa = lv_textarea_create(wifiCard);
  lv_obj_set_size(wifiPassTa, 580, 46);
  lv_obj_align(wifiPassTa, LV_ALIGN_TOP_LEFT, 20, 94);
  lv_textarea_set_one_line(wifiPassTa, true);
  lv_textarea_set_password_mode(wifiPassTa, true);
  lv_textarea_set_placeholder_text(wifiPassTa, "비밀번호");
  lv_obj_set_style_text_font(wifiPassTa, FONT_KR, 0);
  lv_obj_set_style_bg_color(wifiPassTa, lv_color_hex(UI_BG), 0);
  lv_obj_set_style_bg_opa(wifiPassTa, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(wifiPassTa, 0, 0);
  lv_obj_set_style_radius(wifiPassTa, 10, 0);
  lv_obj_set_style_text_color(wifiPassTa, lv_color_hex(UI_TEXT), 0);
  lv_obj_add_event_cb(wifiPassTa, wifi_textarea_event_cb, LV_EVENT_FOCUSED, NULL);

  makePrimaryButton(wifiCard, "연결", 20, 154, 180, 46, UI_ACCENT, wifi_connect_event_cb);
  makeQuietButton(wifiCard, "해제", 210, 154, 130, 46, wifi_disconnect_event_cb);
  makeQuietButton(wifiCard, "저장 삭제", 350, 154, 150, 46, wifi_forget_event_cb);

  labelWifiState = makeLabel(wifiCard, "WiFi: 대기", 20, 224, UI_ACCENT);
  lv_obj_set_width(labelWifiState, 580);
  lv_label_set_long_mode(labelWifiState, LV_LABEL_LONG_CLIP);

  labelWifiIp = makeSmallLabel(wifiCard, "IP: --", 20, 258, UI_TEXT_2);
  lv_obj_set_width(labelWifiIp, 280);
  lv_label_set_long_mode(labelWifiIp, LV_LABEL_LONG_CLIP);

  labelWifiSignal = makeSmallLabel(wifiCard, "신호: --", 320, 258, UI_TEXT_2);
  lv_obj_set_width(labelWifiSignal, 280);
  lv_label_set_long_mode(labelWifiSignal, LV_LABEL_LONG_CLIP);

  labelWifiMode = makeSmallLabel(wifiCard, "방식: ESP32-C6 ESP-Hosted", 20, 284, UI_TEXT_3);
  lv_obj_set_width(labelWifiMode, 580);
  lv_label_set_long_mode(labelWifiMode, LV_LABEL_LONG_CLIP);

  // Device-level entries that are not WiFi.
  makeQuietButton(wifiCard, "센서 검색", 20, 330, 150, 46, i2c_scan_event_cb);
  makeQuietButton(wifiCard, "블루투스 센서", 180, 330, 180, 46, go_ble_event_cb);
  makeQuietButton(wifiCard, "전송 설정", 370, 330, 130, 46, go_isl_event_cb);
  makeQuietButton(wifiCard, "다시 시작", 510, 330, 110, 46, board_restart_event_cb);

  labelSettingsNote = makeSmallLabel(wifiCard, "", 20, 384, UI_TEXT_3);
  lv_obj_set_width(labelSettingsNote, 580);
  lv_label_set_long_mode(labelSettingsNote, LV_LABEL_LONG_CLIP);

  // =====================================================
  // Nearby networks
  // =====================================================
  lv_obj_t *scanCard = makePanel(settingsScreen, 668, 104, 328, 412);

  makeSmallLabel(scanCard, "주변 WiFi", 16, 14, UI_TEXT_3);
  makeQuietButton(scanCard, "검색", 196, 8, 116, 38, wifi_scan_event_cb);

  labelWifiScan = makeSmallLabel(scanCard, "검색 버튼을 누르세요.", 16, 52, UI_TEXT_3);
  lv_obj_set_width(labelWifiScan, 296);
  lv_label_set_long_mode(labelWifiScan, LV_LABEL_LONG_CLIP);

  wifiList = lv_list_create(scanCard);
  lv_obj_set_size(wifiList, 296, 330);
  lv_obj_align(wifiList, LV_ALIGN_TOP_LEFT, 16, 76);
  lv_obj_set_style_text_font(wifiList, FONT_KR_SMALL, 0);
  lv_obj_set_style_bg_opa(wifiList, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(wifiList, 0, 0);
  lv_obj_set_style_pad_all(wifiList, 0, 0);

  wifiKeyboard = lv_keyboard_create(settingsScreen);
  lv_obj_set_size(wifiKeyboard, LCD_H_RES, 220);
  lv_obj_align(wifiKeyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_add_flag(wifiKeyboard, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_event_cb(wifiKeyboard, wifi_keyboard_event_cb, LV_EVENT_ALL, NULL);
  lv_obj_set_style_text_font(wifiKeyboard, &lv_font_montserrat_16, 0);
  lv_btnmatrix_set_btn_ctrl_all(wifiKeyboard, LV_BTNMATRIX_CTRL_NO_REPEAT);

  createTabBar(settingsScreen, 3);

  updateWifiRuntimeLabels();
}


void updateMeasureLabels(float tempC, float pressureHpa, int sampleCount, const char* elapsedText)
{
    char buf[64];

    lv_label_set_text(labelStatus, "상태: 측정 중");

    snprintf(buf, sizeof(buf), "측정 시간: %s", elapsedText);
    lv_label_set_text(labelElapsed, buf);

    formatPrimaryValueText(buf, sizeof(buf), tempC, true);
    lv_label_set_text(labelTempNow, buf);

    if (activeSensorHasPressure() && pressureValueValid(pressureHpa))
    {
      snprintf(buf, sizeof(buf), "기압: %.2f hPa", pressureHpa);
      lv_label_set_text(labelPressureNow, buf);
      lv_obj_clear_flag(labelPressureNow, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
      lv_label_set_text(labelPressureNow, "");
      lv_obj_add_flag(labelPressureNow, LV_OBJ_FLAG_HIDDEN);
    }

    snprintf(buf, sizeof(buf), "샘플 수: %d", sampleCount);
    lv_label_set_text(labelSample, buf);
}

bool loadWifiCredentials()
{
  nvs_handle_t handle;
  esp_err_t ret = nvs_open("wifi_cfg", NVS_READONLY, &handle);

  if (ret != ESP_OK)
  {
    Serial.print("NVS WiFi open read failed: ");
    Serial.println(esp_err_to_name(ret));
    return false;
  }

  size_t ssidLen = sizeof(wifiSavedSsid);
  size_t passLen = sizeof(wifiSavedPassword);

  ret = nvs_get_str(handle, "ssid", wifiSavedSsid, &ssidLen);
  if (ret != ESP_OK || strlen(wifiSavedSsid) == 0)
  {
    nvs_close(handle);
    return false;
  }

  ret = nvs_get_str(handle, "pass", wifiSavedPassword, &passLen);
  if (ret != ESP_OK)
  {
    wifiSavedPassword[0] = '\0';
  }

  nvs_close(handle);

  if (wifiSsidTa) lv_textarea_set_text(wifiSsidTa, wifiSavedSsid);
  if (wifiPassTa) lv_textarea_set_text(wifiPassTa, wifiSavedPassword);

  Serial.print("NVS WiFi loaded: ");
  Serial.println(wifiSavedSsid);
  return true;
}

bool loadBleEnabled()
{
  nvs_handle_t handle;
  if (nvs_open("ble_cfg", NVS_READONLY, &handle) != ESP_OK) return false;

  uint8_t value = 0;
  const esp_err_t ret = nvs_get_u8(handle, "enabled", &value);
  nvs_close(handle);

  return ret == ESP_OK && value != 0;
}

void saveBleEnabled(bool enabled)
{
  nvs_handle_t handle;
  if (nvs_open("ble_cfg", NVS_READWRITE, &handle) != ESP_OK) return;

  nvs_set_u8(handle, "enabled", enabled ? 1 : 0);
  nvs_commit(handle);
  nvs_close(handle);
}

void initNvsStorage()
{
  esp_err_t ret = nvs_flash_init();

  if (ret == ESP_OK)
  {
    Serial.println("NVS init OK");
    return;
  }

  Serial.print("NVS init failed: ");
  Serial.println(esp_err_to_name(ret));

  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    Serial.println("NVS partition requires erase. WiFi saved data may be cleared once.");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();

    if (ret == ESP_OK)
    {
      Serial.println("NVS re-init OK");
    }
    else
    {
      Serial.print("NVS re-init failed: ");
      Serial.println(esp_err_to_name(ret));
    }
  }
}

void saveWifiCredentials(const char *ssid, const char *password)
{
  if (ssid == NULL || strlen(ssid) == 0) return;

  nvs_handle_t handle;
  esp_err_t ret = nvs_open("wifi_cfg", NVS_READWRITE, &handle);

  if (ret != ESP_OK)
  {
    Serial.print("NVS WiFi open write failed: ");
    Serial.println(esp_err_to_name(ret));
    return;
  }

  ret = nvs_set_str(handle, "ssid", ssid);
  if (ret == ESP_OK)
  {
    ret = nvs_set_str(handle, "pass", password ? password : "");
  }

  if (ret == ESP_OK)
  {
    ret = nvs_commit(handle);
  }

  nvs_close(handle);

  if (ret == ESP_OK)
  {
    Serial.println("NVS WiFi credentials saved");
  }
  else
  {
    Serial.print("NVS WiFi save failed: ");
    Serial.println(esp_err_to_name(ret));
  }
}

void clearWifiCredentials()
{
  nvs_handle_t handle;
  esp_err_t ret = nvs_open("wifi_cfg", NVS_READWRITE, &handle);

  if (ret == ESP_OK)
  {
    nvs_erase_all(handle);
    nvs_commit(handle);
    nvs_close(handle);
  }

  wifiSavedSsid[0] = '\0';
  wifiSavedPassword[0] = '\0';

  if (wifiSsidTa) lv_textarea_set_text(wifiSsidTa, "");
  if (wifiPassTa) lv_textarea_set_text(wifiPassTa, "");

  Serial.println("WiFi credentials cleared");
}

void saveIslServiceKey(const char *key)
{
  if (key == NULL || strlen(key) == 0) return;

  nvs_handle_t handle;
  esp_err_t ret = nvs_open("isl_cfg", NVS_READWRITE, &handle);
  if (ret != ESP_OK)
  {
    Serial.print("NVS ISL serviceKey open write failed: ");
    Serial.println(esp_err_to_name(ret));
    return;
  }

  ret = nvs_set_str(handle, "serviceKey", key);
  if (ret == ESP_OK) ret = nvs_commit(handle);
  nvs_close(handle);

  if (ret == ESP_OK) Serial.println("NVS ISL serviceKey saved");
  else Serial.println("NVS ISL serviceKey save failed");
}

bool loadIslServiceKey()
{
  nvs_handle_t handle;
  esp_err_t ret = nvs_open("isl_cfg", NVS_READONLY, &handle);
  if (ret != ESP_OK) return false;

  size_t keyLen = sizeof(islServiceKey);
  ret = nvs_get_str(handle, "serviceKey", islServiceKey, &keyLen);
  nvs_close(handle);

  if (ret == ESP_OK && strlen(islServiceKey) > 0)
  {
    Serial.println("NVS ISL serviceKey loaded");
    return true;
  }

  return false;
}

void sanitizeIslModuleCode(const char *src, char *dst, size_t dstLen)
{
  if (dst == NULL || dstLen == 0) return;
  dst[0] = '\0';
  if (src == NULL) return;

  size_t j = 0;
  for (size_t i = 0; src[i] != '\0' && j < dstLen - 1; i++)
  {
    char c = src[i];
    if (c == ' ' || c == '\n' || c == '\r' || c == '\t') continue;
    if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';
    dst[j++] = c;
  }
  dst[j] = '\0';
}

void saveIslModuleCode(const char *code)
{
  if (code == NULL || strlen(code) == 0) return;

  nvs_handle_t handle;
  esp_err_t ret = nvs_open("isl_cfg", NVS_READWRITE, &handle);
  if (ret != ESP_OK)
  {
    Serial.print("NVS ISL open write failed: ");
    Serial.println(esp_err_to_name(ret));
    return;
  }

  ret = nvs_set_str(handle, "module", code);
  if (ret == ESP_OK) ret = nvs_commit(handle);
  nvs_close(handle);

  if (ret == ESP_OK) Serial.println("NVS ISL module code saved");
  else Serial.println("NVS ISL module code save failed");
}

bool loadIslModuleCode()
{
  nvs_handle_t handle;
  esp_err_t ret = nvs_open("isl_cfg", NVS_READONLY, &handle);
  if (ret != ESP_OK) return false;

  size_t codeLen = sizeof(islRuntimeSerialNumber);
  ret = nvs_get_str(handle, "module", islRuntimeSerialNumber, &codeLen);
  nvs_close(handle);

  if (ret == ESP_OK && strlen(islRuntimeSerialNumber) > 0)
  {
    Serial.print("NVS ISL module loaded: ");
    Serial.println(islRuntimeSerialNumber);
    return true;
  }

  return false;
}

void updateIslModuleCodeFromUi()
{
  char code[sizeof(islRuntimeSerialNumber)];
  code[0] = '\0';

  // 현재 보고 있는 화면의 입력칸을 우선 사용합니다.
  // 두 입력칸이 모두 존재할 때 메인화면에서 입력한 모둠코드가 무시되는 문제를 막습니다.
  if (lv_scr_act() == homeScreen && homeIslModuleTa)
  {
    sanitizeIslModuleCode(lv_textarea_get_text(homeIslModuleTa), code, sizeof(code));
  }
  else if (lv_scr_act() == islScreen && islModuleTa)
  {
    sanitizeIslModuleCode(lv_textarea_get_text(islModuleTa), code, sizeof(code));
  }
  else if (homeIslModuleTa && strlen(lv_textarea_get_text(homeIslModuleTa)) > 0)
  {
    sanitizeIslModuleCode(lv_textarea_get_text(homeIslModuleTa), code, sizeof(code));
  }
  else if (islModuleTa && strlen(lv_textarea_get_text(islModuleTa)) > 0)
  {
    sanitizeIslModuleCode(lv_textarea_get_text(islModuleTa), code, sizeof(code));
  }
  else
  {
    sanitizeIslModuleCode(islRuntimeSerialNumber, code, sizeof(code));
  }

  if (strlen(code) > 0)
  {
    strncpy(islRuntimeSerialNumber, code, sizeof(islRuntimeSerialNumber) - 1);
    islRuntimeSerialNumber[sizeof(islRuntimeSerialNumber) - 1] = '\0';
    saveIslModuleCode(islRuntimeSerialNumber);

    if (islModuleTa) lv_textarea_set_text(islModuleTa, islRuntimeSerialNumber);
    if (homeIslModuleTa) lv_textarea_set_text(homeIslModuleTa, islRuntimeSerialNumber);
  }

  // 서비스키는 지능형과학실 환경변수에서 관리하므로 ESP32에서는 필수가 아닙니다.
  // 기존 직접 API 테스트용 입력값은 보존합니다.
  if (islServiceKeyTa)
  {
    const char *key = lv_textarea_get_text(islServiceKeyTa);
    if (key && strlen(key) > 0)
    {
      strncpy(islServiceKey, key, sizeof(islServiceKey) - 1);
      islServiceKey[sizeof(islServiceKey) - 1] = '\0';
      saveIslServiceKey(islServiceKey);
    }
  }

  islApiConfigured = cloudModumConfigured();
}

void updateIslStatusLabels()
{
  updateDashboardIslLabels();
}

#if HAS_IDF_WIFI
// The P4 has no radio of its own. Both WiFi and Bluetooth ride the ESP-Hosted
// link to the companion ESP32-C6, so the transport is brought up once and
// shared rather than owned by the WiFi path.
bool ensureEspHostedTransport()
{
#if !HAS_ESP_HOSTED
  Serial.println("esp_hosted.h not found. ESP-Hosted host component include path is missing.");
  return false;
#else
  if (espHostedStarted) return true;

  Serial.println("ESP-Hosted transport init start");
  esp_hosted_init();
  espHostedStarted = true;

  unsigned long waitStart = millis();
  while (millis() - waitStart < 1500)
  {
    delay(10);
  }

  Serial.println("ESP-Hosted transport init done");
  return true;
#endif
}

bool ensureHostedWifiStarted()
{
#if !HAS_ESP_HOSTED
  return false;
#else
  if (!ensureEspHostedTransport()) return false;

  if (!idfWifiStarted)
  {
    esp_err_t ret;

    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
      Serial.print("esp_netif_init failed: ");
      Serial.println(esp_err_to_name(ret));
      return false;
    }

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
      Serial.print("esp_event_loop_create_default failed: ");
      Serial.println(esp_err_to_name(ret));
      return false;
    }

    if (wifiStaNetif == NULL)
    {
      wifiStaNetif = esp_netif_create_default_wifi_sta();
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_INIT_STATE)
    {
      Serial.print("esp_wifi_init failed: ");
      Serial.println(esp_err_to_name(ret));
      return false;
    }

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK)
    {
      Serial.print("esp_wifi_set_mode failed: ");
      Serial.println(esp_err_to_name(ret));
      return false;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK)
    {
      Serial.print("esp_wifi_start failed: ");
      Serial.println(esp_err_to_name(ret));
      return false;
    }

    idfWifiStarted = true;
  }

  return true;
#endif
}
#endif

// The host every upload depends on. Resolving it is the difference between a
// usable network and one that only looks connected.
#define ISL_API_HOST "api-scion.kosac.re.kr"

// Public resolvers to fall back through when the network's own cannot answer.
static const char *kFallbackDnsServers[] = { "8.8.8.8", "1.1.1.1" };

static void setDnsServer(const char *address, esp_netif_dns_type_t slot)
{
#if HAS_IDF_WIFI
  esp_netif_dns_info_t dns;
  memset(&dns, 0, sizeof(dns));
  dns.ip.type = ESP_IPADDR_TYPE_V4;
  dns.ip.u_addr.ip4.addr = esp_ip4addr_aton(address);
  esp_netif_set_dns_info(wifiStaNetif, slot, &dns);
#else
  (void)address;
  (void)slot;
#endif
}

// True when the API host resolves. lwIP caches the answer, so a success here
// also warms the cache for the upload that follows.
static bool resolvesApiHost()
{
  struct addrinfo hints;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo *res = NULL;
  const int rc = getaddrinfo(ISL_API_HOST, "443", &hints, &res);

  if (rc == 0 && res != NULL)
  {
    char text[16] = "";
    struct sockaddr_in *addr = (struct sockaddr_in *)res->ai_addr;
    esp_ip4_addr_t ip4;
    ip4.addr = addr->sin_addr.s_addr;
    snprintf(text, sizeof(text), IPSTR, IP2STR(&ip4));
    Serial.printf("[DNS] %s -> %s\n", ISL_API_HOST, text);
    freeaddrinfo(res);
    return true;
  }

  if (res != NULL) freeaddrinfo(res);
  Serial.printf("[DNS] %s did not resolve (rc=%d)\n", ISL_API_HOST, rc);
  return false;
}

// A connected board with no working name server fails every upload with
// "getaddrinfo() returns 202", which on screen looks identical to the server
// being down. Report what DHCP handed over, and install a public resolver when
// it handed over nothing — school networks do that more often than not.
void ensureDnsServer()
{
#if HAS_IDF_WIFI
  if (wifiStaNetif == NULL) return;

  esp_netif_dns_info_t dns;
  memset(&dns, 0, sizeof(dns));

  const esp_err_t ret = esp_netif_get_dns_info(wifiStaNetif, ESP_NETIF_DNS_MAIN, &dns);
  const uint32_t addr = (ret == ESP_OK) ? dns.ip.u_addr.ip4.addr : 0;

  if (addr != 0)
  {
    Serial.printf("[DNS] server " IPSTR "\n", IP2STR(&dns.ip.u_addr.ip4));
  }
  else
  {
    Serial.println("[DNS] none from DHCP");
    setDnsServer(kFallbackDnsServers[0], ESP_NETIF_DNS_MAIN);
  }

  // Always keep a public resolver in the backup slot. lwIP falls through to
  // it on its own when the primary does not answer, which is what this
  // network does intermittently: the router resolves the API host at connect
  // time and goes quiet minutes later. A backup server covers that without
  // the upload path having to notice or recover.
  setDnsServer(kFallbackDnsServers[1], ESP_NETIF_DNS_BACKUP);
  Serial.printf("[DNS] backup %s\n", kFallbackDnsServers[1]);
#endif
}

// Check the resolver the network gave us, and move to a public one if it
// cannot answer for the API host. A DHCP-provided server that resolves
// nothing looks exactly like a working network until the first upload.
void verifyDnsOrFallback()
{
#if HAS_IDF_WIFI
  if (wifiStaNetif == NULL) return;

  ensureDnsServer();

  if (resolvesApiHost())
  {
    setIslStatusText("WiFi: 서버 주소 확인됨");
    return;
  }

  for (size_t i = 0; i < sizeof(kFallbackDnsServers) / sizeof(kFallbackDnsServers[0]); i++)
  {
    Serial.printf("[DNS] trying %s\n", kFallbackDnsServers[i]);
    setDnsServer(kFallbackDnsServers[i], ESP_NETIF_DNS_MAIN);

    if (resolvesApiHost())
    {
      char line[96];
      snprintf(line, sizeof(line), "WiFi: DNS를 %s로 대체함", kFallbackDnsServers[i]);
      setIslStatusText(line);
      return;
    }
  }

  // Say so now rather than letting every upload fail with a generic message.
  setIslStatusText("WiFi: 서버 주소를 찾지 못함 (이 네트워크는 전송 불가)");
#endif
}

bool c6WifiConnect(const char *ssid, const char *password)
{
  if (ssid == NULL || strlen(ssid) == 0)
  {
    return false;
  }

  Serial.print("C6 WiFi connect request SSID: ");
  Serial.println(ssid);

#if HAS_IDF_WIFI
  if (!ensureHostedWifiStarted())
  {
    return false;
  }

  wifi_config_t wifiConfig;
  memset(&wifiConfig, 0, sizeof(wifiConfig));

  strncpy((char *)wifiConfig.sta.ssid, ssid, sizeof(wifiConfig.sta.ssid) - 1);

  if (password != NULL)
  {
    strncpy((char *)wifiConfig.sta.password, password, sizeof(wifiConfig.sta.password) - 1);
  }

  esp_wifi_disconnect();

  esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &wifiConfig);
  if (ret != ESP_OK)
  {
    Serial.print("esp_wifi_set_config failed: ");
    Serial.println(esp_err_to_name(ret));
    return false;
  }

  ret = esp_wifi_connect();
  if (ret != ESP_OK)
  {
    Serial.print("esp_wifi_connect failed: ");
    Serial.println(esp_err_to_name(ret));
    return false;
  }

  unsigned long start = millis();
  wifi_ap_record_t apInfo;

  while (millis() - start < 12000)
  {
    uiTimerHandler();

    if (esp_wifi_sta_get_ap_info(&apInfo) == ESP_OK)
    {
      Serial.println("WiFi connected by esp_wifi");

      unsigned long ipStart = millis();
      while (millis() - ipStart < 8000)
      {
        uiTimerHandler();

        if (wifiStaNetif != NULL)
        {
          esp_netif_ip_info_t ipInfo;
          if (esp_netif_get_ip_info(wifiStaNetif, &ipInfo) == ESP_OK && ipInfo.ip.addr != 0)
          {
            char ipText[32];
            snprintf(ipText, sizeof(ipText), IPSTR, IP2STR(&ipInfo.ip));
            Serial.print("WiFi DHCP IP: ");
            Serial.println(ipText);
            verifyDnsOrFallback();
            return true;
          }
        }

        delay(100);
      }

      Serial.println("WiFi connected, but DHCP IP is not assigned yet");
      return false;
    }

    delay(100);
  }

  Serial.println("WiFi connect timeout");
  return false;
#else
#if HAS_ARDUINO_WIFI
  WiFi.mode(WIFI_STA);

  if (password != NULL && strlen(password) > 0)
  {
    WiFi.begin(ssid, password);
  }
  else
  {
    WiFi.begin(ssid);
  }

  unsigned long start = millis();

  while (millis() - start < 12000)
  {
    uiTimerHandler();

    if (WiFi.status() == WL_CONNECTED && WiFi.localIP().toString() != "0.0.0.0")
    {
      Serial.print("WiFi connected IP: ");
      Serial.println(WiFi.localIP());
      return true;
    }

    delay(100);
  }

  Serial.println("WiFi connect timeout");
  return false;
#else
  Serial.println("WiFi.h not available. C6 command implementation is required.");
  return false;
#endif
#endif
}

void c6WifiDisconnect()
{
  Serial.println("C6 WiFi disconnect request");
  httpCloseConnection("WiFi disconnect");

#if HAS_IDF_WIFI
  esp_wifi_disconnect();
#else
#if HAS_ARDUINO_WIFI
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
#endif
#endif
}

int c6WifiScan(WifiNetworkInfo *results, int maxResults)
{
  if (results == NULL || maxResults <= 0)
  {
    return 0;
  }

  Serial.println("C6 WiFi scan request");

#if HAS_IDF_WIFI
  if (!ensureHostedWifiStarted())
  {
    return 0;
  }

  wifi_scan_config_t scanConfig;
  memset(&scanConfig, 0, sizeof(scanConfig));
  scanConfig.show_hidden = true;

  esp_err_t ret = esp_wifi_scan_start(&scanConfig, true);
  if (ret != ESP_OK)
  {
    Serial.print("esp_wifi_scan_start failed: ");
    Serial.println(esp_err_to_name(ret));
    return 0;
  }

  uint16_t apCount = 0;
  ret = esp_wifi_scan_get_ap_num(&apCount);
  if (ret != ESP_OK)
  {
    Serial.print("esp_wifi_scan_get_ap_num failed: ");
    Serial.println(esp_err_to_name(ret));
    return 0;
  }

  if (apCount == 0)
  {
    Serial.println("No WiFi networks found");
    return 0;
  }

  uint16_t number = apCount;
  if (number > maxResults) number = maxResults;
  if (number > WIFI_SCAN_MAX) number = WIFI_SCAN_MAX;

  wifi_ap_record_t apInfo[WIFI_SCAN_MAX];
  memset(apInfo, 0, sizeof(apInfo));

  ret = esp_wifi_scan_get_ap_records(&number, apInfo);
  if (ret != ESP_OK)
  {
    Serial.print("esp_wifi_scan_get_ap_records failed: ");
    Serial.println(esp_err_to_name(ret));
    return 0;
  }

  for (int i = 0; i < number; i++)
  {
    strncpy(results[i].ssid, (const char *)apInfo[i].ssid, sizeof(results[i].ssid) - 1);
    results[i].ssid[sizeof(results[i].ssid) - 1] = '\0';
    results[i].rssi = apInfo[i].rssi;
    results[i].secure = !(apInfo[i].authmode == WIFI_AUTH_OPEN || apInfo[i].authmode == WIFI_AUTH_OWE);
  }

  Serial.print("WiFi networks found by esp_wifi: ");
  Serial.println(number);
  return number;
#else
#if HAS_ARDUINO_WIFI
  Serial.println("Arduino WiFi.h is available");
  Serial.println("Start WiFi scan");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false);
  delay(150);

  int found = WiFi.scanNetworks(false, true);

  Serial.print("WiFi.scanNetworks result: ");
  Serial.println(found);

  if (found <= 0)
  {
    if (found == WIFI_SCAN_RUNNING)
    {
      Serial.println("WiFi scan is still running");
    }
    else if (found == WIFI_SCAN_FAILED)
    {
      Serial.println("WiFi scan failed. ESP32-C6 hosted driver is probably not initialized.");
    }
    else
    {
      Serial.println("No WiFi networks found");
    }

    return 0;
  }

  int count = found;
  if (count > maxResults) count = maxResults;

  for (int i = 0; i < count; i++)
  {
    String ssid = WiFi.SSID(i);
    ssid.toCharArray(results[i].ssid, sizeof(results[i].ssid));
    results[i].rssi = WiFi.RSSI(i);
    results[i].secure = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
  }

  WiFi.scanDelete();

  Serial.print("WiFi networks found: ");
  Serial.println(count);

  return count;
#else
  Serial.println("WiFi.h not available. C6 scan command implementation is required.");
  return 0;
#endif
#endif
}

String getWifiIpText()
{
#if HAS_IDF_WIFI
  if (wifiStaNetif != NULL)
  {
    esp_netif_ip_info_t ipInfo;
    if (esp_netif_get_ip_info(wifiStaNetif, &ipInfo) == ESP_OK && ipInfo.ip.addr != 0)
    {
      char buf[32];
      snprintf(buf, sizeof(buf), IPSTR, IP2STR(&ipInfo.ip));
      return String(buf);
    }
  }
#elif HAS_ARDUINO_WIFI
  if (WiFi.status() == WL_CONNECTED)
  {
    return WiFi.localIP().toString();
  }
#endif

  return String("--");
}

int getWifiRssiValue()
{
#if HAS_IDF_WIFI
  wifi_ap_record_t apInfo;
  if (esp_wifi_sta_get_ap_info(&apInfo) == ESP_OK)
  {
    return apInfo.rssi;
  }
#elif HAS_ARDUINO_WIFI
  if (WiFi.status() == WL_CONNECTED)
  {
    return WiFi.RSSI();
  }
#endif

  return 0;
}



// =====================================================
// 지능형 과학실 ON API / 비동기 WiFi 전송
// 측정 루프에서는 절대 HTTP/WiFi 연결을 직접 수행하지 않고 큐에만 넣습니다.
// =====================================================
void setIslStatusText(const char *text)
{
  if (text == NULL) return;
  strncpy(islStatusText, text, sizeof(islStatusText) - 1);
  islStatusText[sizeof(islStatusText) - 1] = '\0';
  Serial.println(islStatusText);

  // LVGL objects must be updated from the main loop only.
  // The WiFi/API task only marks the UI dirty.
  islStatusUiDirty = true;
  batchUiDirty = true;
}

void formatIslCollectDate(char *out, size_t len)
{
  if (out == NULL || len == 0) return;

  String nowText = getCurrentDateTimeText();
  strncpy(out, nowText.c_str(), len - 1);
  out[len - 1] = '\0';
}

bool extractJsonStringValue(const String &json, const char *key, char *out, size_t outLen)
{
  if (out == NULL || outLen == 0 || key == NULL) return false;

  int keyPos = json.indexOf(key);
  if (keyPos < 0) return false;

  int colon = json.indexOf(':', keyPos);
  if (colon < 0) return false;

  int firstQuote = json.indexOf('"', colon + 1);
  if (firstQuote < 0) return false;

  int secondQuote = json.indexOf('"', firstQuote + 1);
  if (secondQuote < 0) return false;

  String value = json.substring(firstQuote + 1, secondQuote);
  value.toCharArray(out, outLen);
  return true;
}

int extractJsonIntValue(const String &json, const char *key, int fallback)
{
  char value[24];
  if (!extractJsonStringValue(json, key, value, sizeof(value)))
  {
    int keyPos = json.indexOf(key);
    if (keyPos < 0) return fallback;
    int colon = json.indexOf(':', keyPos);
    if (colon < 0) return fallback;
    return json.substring(colon + 1).toInt();
  }

  int parsed = atoi(value);
  return parsed > 0 ? parsed : fallback;
}

#if HAS_ESP_HTTP_CLIENT
// Where the body of the request in flight is collected. Set immediately
// before each perform(); the handle is reused, so user_data cannot carry it.
static String *httpResponseTarget = NULL;

static esp_err_t cloudHttpEventHandler(esp_http_client_event_t *evt)
{
  if (evt == NULL) return ESP_FAIL;

  if (evt->event_id == HTTP_EVENT_ON_DATA && evt->data != NULL && evt->data_len > 0 &&
      httpResponseTarget != NULL)
  {
    httpResponseTarget->concat((const char *)evt->data, evt->data_len);
  }

  return ESP_OK;
}

// One HTTPS connection, kept open between requests.
//
// Every POST used to build a client, hand it "Connection: close", and destroy
// it — a full TLS handshake per sample. At 1 Hz that is a handshake a second,
// each one needing a contiguous DMA buffer for the AES accelerator, and each
// one a chance to fail and lose that sample. Holding the connection open
// drops that to one handshake per session.
static esp_http_client_handle_t httpClient = NULL;
static char httpClientHost[96] = "";

static void httpUrlHost(const char *url, char *out, size_t outSize)
{
  out[0] = '\0';
  if (url == NULL) return;

  const char *start = strstr(url, "://");
  start = start ? start + 3 : url;

  const char *end = strchr(start, '/');
  size_t len = end ? (size_t)(end - start) : strlen(start);
  if (len >= outSize) len = outSize - 1;

  memcpy(out, start, len);
  out[len] = '\0';
}

void httpCloseConnection(const char *reason)
{
  if (httpClient == NULL) return;

  Serial.print("[HTTP] closing connection: ");
  Serial.println(reason ? reason : "");

  esp_http_client_cleanup(httpClient);
  httpClient = NULL;
  httpClientHost[0] = '\0';
}
#endif

#if !HAS_ESP_HTTP_CLIENT
void httpCloseConnection(const char *reason) { (void)reason; }
#endif

void logHeapState(const char *context)
{
  Serial.printf(
    "[HEAP] %s free=%u internal=%u dma=%u largest_dma=%u\n",
    context,
    (unsigned)esp_get_free_heap_size(),
    (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
    (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
    (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA)
  );
}

bool httpPostJson(const char *url, const String &payload, String *response)
{
#if HAS_HTTP_CLIENT
  WiFiClientSecure client;
  client.setInsecure(); // 테스트/교실용: 인증서 검증 생략. 운영 환경에서는 루트 인증서 사용 권장.
  client.setTimeout(CLOUD_HTTP_TIMEOUT_MS / 1000);

  HTTPClient http;
  http.setTimeout(CLOUD_HTTP_TIMEOUT_MS);

  Serial.println("----- HTTP POST START -----");
  logHeapState("before POST");
  Serial.print("URL: ");
  Serial.println(url);
  Serial.print("Payload length: ");
  Serial.println(payload.length());
  Serial.println("HTTP backend: Arduino HTTPClient");

  if (!http.begin(client, url))
  {
    Serial.println("HTTP begin failed");
    Serial.println("----- HTTP POST END -----");
    return false;
  }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("Accept", "text/plain");
  http.addHeader("Connection", "close");

  int statusCode = http.POST(payload);
  String body = http.getString();

  Serial.print("HTTP status: ");
  Serial.println(statusCode);

  if (statusCode <= 0)
  {
    Serial.print("HTTP client error: ");
    Serial.println(http.errorToString(statusCode));
  }

  Serial.print("HTTP response: ");
  Serial.println(body);

  if (response != NULL)
  {
    *response = body;
  }

  http.end();
  Serial.println("----- HTTP POST END -----");

  return statusCode >= 200 && statusCode < 300;
#elif HAS_ESP_HTTP_CLIENT
  String body;

  Serial.println("----- HTTP POST START -----");
  logHeapState("before POST");
  Serial.print("URL: ");
  Serial.println(url);
  Serial.print("Payload length: ");
  Serial.println(payload.length());
  Serial.println("HTTP backend: ESP-IDF esp_http_client");

  // A cached connection only helps while the host matches; ISL and the cloud
  // function are different servers.
  char host[96];
  httpUrlHost(url, host, sizeof(host));

  if (httpClient != NULL && strcmp(host, httpClientHost) != 0)
  {
    httpCloseConnection("different host");
  }

  if (httpClient == NULL)
  {
    esp_http_client_config_t config = {};
    config.url = url;
    config.event_handler = cloudHttpEventHandler;
    config.timeout_ms = CLOUD_HTTP_TIMEOUT_MS;
    config.buffer_size = 2048;
    config.buffer_size_tx = 2048;
    config.keep_alive_enable = true;

#if HAS_ESP_CRT_BUNDLE
    // run.app은 Google 인증서를 사용하므로 ESP-IDF 인증서 번들을 붙여 HTTPS 검증을 처리합니다.
    config.crt_bundle_attach = esp_crt_bundle_attach;
    Serial.println("TLS: ESP CRT bundle enabled");
#else
    // 인증서 번들이 없는 빌드에서는 HTTPS 검증이 실패할 수 있습니다.
    config.skip_cert_common_name_check = true;
    Serial.println("TLS: CRT bundle not available; HTTPS may fail");
#endif

    httpClient = esp_http_client_init(&config);
    if (httpClient == NULL)
    {
      Serial.println("esp_http_client_init failed");
      Serial.println("----- HTTP POST END -----");
      return false;
    }

    snprintf(httpClientHost, sizeof(httpClientHost), "%s", host);
    Serial.print("[HTTP] new connection to ");
    Serial.println(host);
  }
  else
  {
    Serial.println("[HTTP] reusing connection");
    esp_http_client_set_url(httpClient, url);
  }

  esp_http_client_handle_t client = httpClient;

  esp_http_client_set_method(client, HTTP_METHOD_POST);
  esp_http_client_set_header(client, "Content-Type", "application/json");
  esp_http_client_set_header(client, "Accept", "text/plain");
  esp_http_client_set_header(client, "Connection", "keep-alive");
  esp_http_client_set_post_field(client, payload.c_str(), payload.length());

  httpResponseTarget = &body;
  esp_err_t err = esp_http_client_perform(client);
  httpResponseTarget = NULL;

  int statusCode = esp_http_client_get_status_code(client);
  int contentLength = esp_http_client_get_content_length(client);

  Serial.print("esp_http_client result: ");
  Serial.println(esp_err_to_name(err));

  if (err == ESP_ERR_HTTP_CONNECT)
  {
    // Could not reach the server at all. On this network that is the resolver
    // going quiet rather than the API being down — it answered at connect time
    // and stopped minutes later — and it comes back on a public resolver. So
    // re-check here, while a retry still follows, instead of reporting a dead
    // end that a single check at connect time cannot prevent.
    setIslStatusText("전송 실패: 서버 주소 확인 중");
    verifyDnsOrFallback();
  }
  Serial.print("HTTP status: ");
  Serial.println(statusCode);
  Serial.print("HTTP content length: ");
  Serial.println(contentLength);
  Serial.print("HTTP response: ");
  Serial.println(body);

  if (response != NULL)
  {
    *response = body;
  }

  // Keep the connection for the next request, but never keep a broken one:
  // a failed perform can leave the socket half-open, and reusing it turns one
  // failure into every subsequent request failing.
  if (err != ESP_OK)
  {
    httpCloseConnection(esp_err_to_name(err));
  }

  Serial.println("----- HTTP POST END -----");

  return err == ESP_OK && statusCode >= 200 && statusCode < 300;
#else
  (void)url;
  (void)payload;
  (void)response;
  Serial.println("No HTTP backend available: HAS_HTTP_CLIENT=0, HAS_ESP_HTTP_CLIENT=0");
  return false;
#endif
}


bool wifiReadyForHttp()
{
  if (!wifiConnected) return false;
  String ip = getWifiIpText();
  if (ip.length() == 0) return false;
  if (ip == "--") return false;
  if (ip == "0.0.0.0") return false;
  return true;
}

bool jsonCodeInResponse(const String &response, const char *ok1, const char *ok2 = NULL, const char *ok3 = NULL)
{
  char code[12] = "";
  if (!extractJsonStringValue(response, "\"code\"", code, sizeof(code))) return false;
  if (ok1 != NULL && strcmp(code, ok1) == 0) return true;
  if (ok2 != NULL && strcmp(code, ok2) == 0) return true;
  if (ok3 != NULL && strcmp(code, ok3) == 0) return true;
  return false;
}

bool httpPostJsonReliable(const char *url, const String &payload, String *response, const char *label, int maxAttempts, int delayMs)
{
  if (maxAttempts < 1) maxAttempts = 1;
  if (delayMs < 100) delayMs = 100;

  for (int attempt = 1; attempt <= maxAttempts; attempt++)
  {
    if (!wifiReadyForHttp())
    {
      Serial.print(label ? label : "HTTP");
      Serial.println(" skipped: WiFi has no IP yet");
      delay(delayMs);
      continue;
    }

    String localResponse;
    bool ok = httpPostJson(url, payload, &localResponse);
    if (response != NULL) *response = localResponse;

    if (ok)
    {
      return true;
    }

    Serial.print(label ? label : "HTTP");
    Serial.print(" retry ");
    Serial.print(attempt);
    Serial.print("/");
    Serial.println(maxAttempts);

    if (attempt < maxAttempts)
    {
      delay(delayMs * attempt);
    }
  }

  return false;
}

bool httpPostJsonReliableCode(const char *url, const String &payload, String *response, const char *label, const char *ok1, const char *ok2 = NULL, const char *ok3 = NULL)
{
  const int maxAttempts = 3;
  const int retryDelayMs = 700;

  for (int attempt = 1; attempt <= maxAttempts; attempt++)
  {
    String localResponse;
    bool httpOk = httpPostJsonReliable(url, payload, &localResponse, label, 1, retryDelayMs);
    if (response != NULL) *response = localResponse;

    if (httpOk && jsonCodeInResponse(localResponse, ok1, ok2, ok3))
    {
      return true;
    }

    Serial.print(label ? label : "HTTP_CODE");
    Serial.print(" code retry ");
    Serial.print(attempt);
    Serial.print("/");
    Serial.println(maxAttempts);
    Serial.println(localResponse);

    if (attempt < maxAttempts)
    {
      delay(retryDelayMs * attempt);
    }
  }

  return false;
}


String jsonEscapeString(const char *text)
{
  String out = "";
  if (text == NULL) return out;
  for (const char *p = text; *p; ++p)
  {
    char c = *p;
    if (c == '\\') out += "\\\\";
    else if (c == '"') out += "\\\"";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else out += c;
  }
  return out;
}

void resetDirectIslSessionCache(const char *reason)
{
  directIslUniqueCode[0] = '\0';
  directIslStartOk = false;
  directIslSensorTypeOk = false;
  directIslStatusOk = false;
  directIslModumId[0] = '\0';

  // 새 세션에서는 실제 API 예제의 ILM부터 다시 시도합니다.
  strncpy(directIslLightSensorType, "ILM", sizeof(directIslLightSensorType) - 1);
  directIslLightSensorType[sizeof(directIslLightSensorType) - 1] = '\0';

  Serial.print("Direct ISL session cache reset");
  if (reason != NULL && strlen(reason) > 0)
  {
    Serial.print(": ");
    Serial.println(reason);
  }
  else
  {
    Serial.println();
  }
}

bool ensureWifiReadyForHttp(const char *context, int waitMs)
{
  (void)waitMs;
  if (wifiReadyForHttp()) return true;

  const char *label = context ? context : "WiFi";
  char msg[128];
  snprintf(msg, sizeof(msg), "%s: WiFi 설정에서 직접 연결 필요", label);
  setIslStatusText(msg);
  wifiConnected = false;
  resetDirectIslSessionCache("WiFi/IP not ready - manual mode");
  return false;
}

void formatDirectAxisTick(uint32_t timeS, char *out, size_t outSize)
{
  if (out == NULL || outSize == 0) return;
#if DIRECT_ISL_AXIS_PADDED
  snprintf(out, outSize, "%04lu", (unsigned long)timeS);
#else
  snprintf(out, outSize, "%lu", (unsigned long)timeS);
#endif
}

bool directIslConfigured()
{
#if REALTIME_DIRECT_ISL_ENABLED
  if (strlen(islServiceKey) == 0) return false;
  if (strcmp(islServiceKey, "PUT_YOUR_SERVICE_KEY") == 0) return false;
  if (strcmp(islServiceKey, "PUT_YOUR_ISL_SERVICE_KEY_HERE") == 0) return false;
  if (strlen(islRuntimeSerialNumber) == 0) return false;
  return true;
#else
  return false;
#endif
}

bool directIslStartProcess(const char *modumId)
{
#if REALTIME_DIRECT_ISL_ENABLED
  if (!ensureWifiReadyForHttp("직접전송", 20000)) return false;
  if (!directIslConfigured())
  {
    setIslStatusText("직접전송: serviceKey/모둠코드 필요");
    return false;
  }

  if (modumId == NULL || strlen(modumId) == 0) return false;
  if (directIslStartOk && strlen(directIslUniqueCode) > 0 && strcmp(directIslModumId, modumId) == 0)
  {
    return true;
  }

  if (strcmp(directIslModumId, modumId) != 0)
  {
    directIslUniqueCode[0] = '\0';
    directIslStartOk = false;
    directIslSensorTypeOk = false;
    directIslStatusOk = false;
    strncpy(directIslModumId, modumId, sizeof(directIslModumId) - 1);
    directIslModumId[sizeof(directIslModumId) - 1] = '\0';
  }

  String payload = "{";
  payload += "\"serviceKey\":\"";
  payload += jsonEscapeString(islServiceKey);
  payload += "\",\"modumId\":\"";
  payload += jsonEscapeString(modumId);
  payload += "\"}";

  String response;
  if (!httpPostJsonReliable(DIRECT_ISL_START_URL, payload, &response, "DIRECT_START", 3, 700))
  {
    setIslStatusText("직접전송: start 실패");
    return false;
  }

  char code[12] = "";
  char uniqueCode[128] = "";
  extractJsonStringValue(response, "\"code\"", code, sizeof(code));
  extractJsonStringValue(response, "\"uniqueCode\"", uniqueCode, sizeof(uniqueCode));

  if ((strcmp(code, "001") != 0 && strcmp(code, "005") != 0) || strlen(uniqueCode) == 0)
  {
    Serial.print("Direct ISL start bad response: ");
    Serial.println(response);
    setIslStatusText("직접전송: start 응답 오류");
    return false;
  }

  strncpy(directIslUniqueCode, uniqueCode, sizeof(directIslUniqueCode) - 1);
  directIslUniqueCode[sizeof(directIslUniqueCode) - 1] = '\0';
  directIslStartOk = true;
  directIslSensorTypeOk = false;
  directIslStatusOk = false;
  setIslStatusText("직접전송: start OK");
  return true;
#else
  (void)modumId;
  return false;
#endif
}

bool directIslSendSensorTypeIfNeeded()
{
#if REALTIME_DIRECT_ISL_ENABLED
  if (directIslSensorTypeOk) return true;
  if (strlen(directIslUniqueCode) == 0)
  {
    Serial.println("DIRECT_TYPE DEBUG FAIL: uniqueCode empty");
    return false;
  }
  if (!activeSensorSupportsDirectIsl())
  {
    Serial.println("DIRECT_TYPE DEBUG FAIL: sensor not supported");
    return false;
  }

  // =====================================================
  // SCD41 CO2: generic Concentration(CTRT) candidate.
  // This path is reachable only when SCD41_DIRECT_ISL_ENABLED == 1.
  // =====================================================
  if (activeSensorMode == SENSOR_MODE_SCD41)
  {
    String payload = "{";
    payload += "\"serviceKey\":\"";
    payload += jsonEscapeString(islServiceKey);
    payload += "\",\"uniqueCode\":\"";
    payload += jsonEscapeString(directIslUniqueCode);
    payload += "\",\"sensorCount\":3,\"items\":[";
    payload += "{\"sensorType\":\"" ISL_SENSOR_TYPE_CO2 "\",\"sensorNicNm\":\"이산화탄소센서\",\"channelCode\":\"" ISL_CHANNEL_CO2 "\"},";
    payload += "{\"sensorType\":\"TPR\",\"sensorNicNm\":\"온도센서\",\"channelCode\":\"" ISL_CHANNEL_SCD41_TEMP "\"},";
    payload += "{\"sensorType\":\"" ISL_SENSOR_TYPE_HUMIDITY "\",\"sensorNicNm\":\"습도센서\",\"channelCode\":\"" ISL_CHANNEL_HUMIDITY "\"}";
    payload += "]}";

    String response;
    if (!httpPostJsonReliable(DIRECT_ISL_SENSOR_TYPE_URL, payload, &response, "DIRECT_TYPE_CO2", 2, 700))
    {
      setIslStatusText("CO2 센서등록 실패: CTRT 확인 필요");
      return false;
    }

    char code[12] = "";
    extractJsonStringValue(response, "\"code\"", code, sizeof(code));
    if (strcmp(code, "001") == 0 || strcmp(code, "015") == 0)
    {
      directIslSensorTypeOk = true;
      setIslStatusText("CO2 센서등록 OK: CTRT");
      return true;
    }

    Serial.print("CO2 sensor type response: ");
    Serial.println(response);
    setIslStatusText("CO2 센서등록 거부: CTRT 확인 필요");
    return false;
  }

  // =====================================================
  // TSL2591 조도: 문서 내부 센서코드 불일치 자동 대응
  // =====================================================
  if (activeSensorMode == SENSOR_MODE_TSL2591)
  {
    const char *lightTypeCandidates[] = {"ILM", "ILMN"};

    for (int candidate = 0; candidate < 2; candidate++)
    {
      const char *sensorType = lightTypeCandidates[candidate];

      String payload = "{";
      payload += "\"serviceKey\":\"";
      payload += jsonEscapeString(islServiceKey);
      payload += "\",\"uniqueCode\":\"";
      payload += jsonEscapeString(directIslUniqueCode);
      payload += "\",\"sensorCount\":1,\"items\":[";
      payload += "{\"sensorType\":\"";
      payload += sensorType;
      payload += "\",\"sensorNicNm\":\"조도센서\",\"channelCode\":\"01\"}";
      payload += "]}";

      Serial.println();
      Serial.print("LIGHT TYPE DEBUG: try ");
      Serial.println(sensorType);
      Serial.print("LIGHT TYPE REQUEST: ");
      Serial.println(payload);

      String response;
      bool httpOk = httpPostJsonReliable(
        DIRECT_ISL_SENSOR_TYPE_URL,
        payload,
        &response,
        candidate == 0 ? "DIRECT_TYPE_ILM" : "DIRECT_TYPE_ILMN",
        2,
        700
      );

      Serial.print("LIGHT TYPE HTTP OK: ");
      Serial.println(httpOk ? 1 : 0);
      Serial.print("LIGHT TYPE RESPONSE: ");
      Serial.println(response);

      char code[12] = "";
      extractJsonStringValue(response, "\"code\"", code, sizeof(code));

      Serial.print("LIGHT TYPE RESPONSE CODE: ");
      Serial.println(strlen(code) > 0 ? code : "(empty)");

      if (httpOk && (strcmp(code, "001") == 0 || strcmp(code, "015") == 0))
      {
        strncpy(directIslLightSensorType, sensorType, sizeof(directIslLightSensorType) - 1);
        directIslLightSensorType[sizeof(directIslLightSensorType) - 1] = '\0';
        directIslSensorTypeOk = true;

        char okLine[96];
        snprintf(okLine, sizeof(okLine), "조도 센서등록 OK: %s", directIslLightSensorType);
        setIslStatusText(okLine);

        Serial.print("LIGHT TYPE SELECTED: ");
        Serial.println(directIslLightSensorType);
        return true;
      }

      Serial.print("LIGHT TYPE REJECTED: ");
      Serial.println(sensorType);
    }

    setIslStatusText("조도 센서등록 실패: ILM/ILMN 모두 거부");
    Serial.println("LIGHT TYPE DEBUG FAIL: both ILM and ILMN rejected");
    return false;
  }

  // =====================================================
  // 회전 엔코더: 각도와 각속도.
  // =====================================================
  if (activeSensorMode == SENSOR_MODE_ENCODER)
  {
    String payload = "{";
    payload += "\"serviceKey\":\"";
    payload += jsonEscapeString(islServiceKey);
    payload += "\",\"uniqueCode\":\"";
    payload += jsonEscapeString(directIslUniqueCode);
    payload += "\",\"sensorCount\":2,\"items\":[";
    payload += "{\"sensorType\":\"" ISL_SENSOR_TYPE_ANGLE "\",\"sensorNicNm\":\"각도센서\",\"channelCode\":\"01\"},";
    payload += "{\"sensorType\":\"" ISL_SENSOR_TYPE_ANGVEL "\",\"sensorNicNm\":\"각속도센서\",\"channelCode\":\"01\"}";
    payload += "]}";

    String response;
    if (!httpPostJsonReliable(DIRECT_ISL_SENSOR_TYPE_URL, payload, &response, "DIRECT_TYPE_ENCODER", 2, 700))
    {
      setIslStatusText("회전 센서등록 실패");
      return false;
    }

    char code[12] = "";
    extractJsonStringValue(response, "\"code\"", code, sizeof(code));

    if (strcmp(code, "001") == 0 || strcmp(code, "015") == 0)
    {
      directIslSensorTypeOk = true;
      setIslStatusText("회전 센서등록 OK: ANGL/ANGV");
      return true;
    }

    Serial.print("Encoder sensor type response: ");
    Serial.println(response);
    setIslStatusText("회전 센서등록 거부: 코드 확인 필요");
    return false;
  }

  // =====================================================
  // INA228: three quantities from one reading, like SCD41.
  // =====================================================
  if (activeSensorMode == SENSOR_MODE_INA228)
  {
    String payload = "{";
    payload += "\"serviceKey\":\"";
    payload += jsonEscapeString(islServiceKey);
    payload += "\",\"uniqueCode\":\"";
    payload += jsonEscapeString(directIslUniqueCode);
    payload += "\",\"sensorCount\":3,\"items\":[";
    payload += "{\"sensorType\":\"" ISL_SENSOR_TYPE_CURRENT "\",\"sensorNicNm\":\"전류센서\",\"channelCode\":\"01\"},";
    payload += "{\"sensorType\":\"" ISL_SENSOR_TYPE_VOLTAGE "\",\"sensorNicNm\":\"전압센서\",\"channelCode\":\"01\"},";
    payload += "{\"sensorType\":\"" ISL_SENSOR_TYPE_POWER "\",\"sensorNicNm\":\"전력센서\",\"channelCode\":\"01\"}";
    payload += "]}";

    String response;
    if (!httpPostJsonReliable(DIRECT_ISL_SENSOR_TYPE_URL, payload, &response, "DIRECT_TYPE_INA228", 2, 700))
    {
      setIslStatusText("전압·전류 센서등록 실패");
      return false;
    }

    char code[12] = "";
    extractJsonStringValue(response, "\"code\"", code, sizeof(code));

    if (strcmp(code, "001") == 0 || strcmp(code, "015") == 0)
    {
      directIslSensorTypeOk = true;
      setIslStatusText("전압·전류 센서등록 OK: VOLT/ECRT/EPOW");
      return true;
    }

    Serial.print("INA228 sensor type response: ");
    Serial.println(response);
    setIslStatusText("전압·전류 센서등록 거부: 코드 확인 필요");
    return false;
  }

  // =====================================================
  // VL53L1X 거리: try the appendix code, then plausible alternatives.
  // =====================================================
  if (activeSensorMode == SENSOR_MODE_VL53L1X)
  {
    static const char *distanceCandidates[] = { "DITC", "DIST", "DIS" };

    for (int candidate = 0; candidate < 3; candidate++)
    {
      const char *sensorType = distanceCandidates[candidate];

      String payload = "{";
      payload += "\"serviceKey\":\"";
      payload += jsonEscapeString(islServiceKey);
      payload += "\",\"uniqueCode\":\"";
      payload += jsonEscapeString(directIslUniqueCode);
      payload += "\",\"sensorCount\":1,\"items\":[";
      payload += "{\"sensorType\":\"";
      payload += sensorType;
      payload += "\",\"sensorNicNm\":\"거리센서\",\"channelCode\":\"" ISL_CHANNEL_DISTANCE "\"}";
      payload += "]}";

      Serial.print("DISTANCE TYPE DEBUG: try ");
      Serial.println(sensorType);

      String response;
      const bool httpOk = httpPostJsonReliable(
        DIRECT_ISL_SENSOR_TYPE_URL, payload, &response, "DIRECT_TYPE_DISTANCE", 2, 700);

      Serial.print("DISTANCE TYPE RESPONSE: ");
      Serial.println(response);

      char code[12] = "";
      extractJsonStringValue(response, "\"code\"", code, sizeof(code));

      if (httpOk && (strcmp(code, "001") == 0 || strcmp(code, "015") == 0))
      {
        strncpy(directIslDistanceSensorType, sensorType, sizeof(directIslDistanceSensorType) - 1);
        directIslDistanceSensorType[sizeof(directIslDistanceSensorType) - 1] = '\0';
        directIslSensorTypeOk = true;

        char okLine[96];
        snprintf(okLine, sizeof(okLine), "거리 센서등록 OK: %s", directIslDistanceSensorType);
        setIslStatusText(okLine);
        return true;
      }

      Serial.print("DISTANCE TYPE REJECTED: ");
      Serial.println(sensorType);
    }

    setIslStatusText("거리 센서등록 실패: DITC/DIST/DIS 모두 거부");
    return false;
  }

  // =====================================================
  // 기존에 정상 동작하던 온도/기압/수온 로직은 그대로 유지
  // TMP117 also lands here: it is a temperature sensor on the verified TPR code.
  // =====================================================
  String payload = "{";
  payload += "\"serviceKey\":\"";
  payload += jsonEscapeString(islServiceKey);
  payload += "\",\"uniqueCode\":\"";
  payload += jsonEscapeString(directIslUniqueCode);

  if (activeSensorMode == SENSOR_MODE_DPS310)
  {
    payload += "\",\"sensorCount\":2,";
    payload += "\"items\":[";
    payload += "{\"sensorType\":\"TPR\",\"sensorNicNm\":\"온도센서\",\"channelCode\":\"01\"},";
    payload += "{\"sensorType\":\"PRS\",\"sensorNicNm\":\"기압센서\",\"channelCode\":\"02\"}";
  }
  else
  {
    payload += "\",\"sensorCount\":1,";
    payload += "\"items\":[";
    payload += "{\"sensorType\":\"TPR\",\"sensorNicNm\":\"";
    payload += islTemperatureNickname();
    payload += "\",\"channelCode\":\"01\"}";
  }

  payload += "]}";

  Serial.print("Direct sensor type request: ");
  Serial.println(payload);

  String response;
  if (!httpPostJsonReliable(DIRECT_ISL_SENSOR_TYPE_URL, payload, &response, "DIRECT_TYPE", 3, 700))
  {
    setIslStatusText("직접전송: 센서등록 실패");
    return false;
  }

  Serial.print("Direct sensor type response: ");
  Serial.println(response);

  char code[12] = "";
  extractJsonStringValue(response, "\"code\"", code, sizeof(code));
  if (strcmp(code, "001") == 0 || strcmp(code, "015") == 0)
  {
    directIslSensorTypeOk = true;
    return true;
  }

  Serial.print("Direct ISL sensor type bad response: ");
  Serial.println(response);
  setIslStatusText("직접전송: 센서등록 응답 오류");
  return false;
#else
  return false;
#endif
}
bool directIslSetStatusOnIfNeeded()
{
#if REALTIME_DIRECT_ISL_ENABLED
  if (directIslStatusOk) return true;
  if (strlen(directIslUniqueCode) == 0) return false;

  String payload = "{";
  payload += "\"serviceKey\":\"";
  payload += jsonEscapeString(islServiceKey);
  payload += "\",\"uniqueCode\":\"";
  payload += jsonEscapeString(directIslUniqueCode);
  payload += "\",\"status\":\"on\",\"code\":\"001\",\"message\":\"ESP32 직접 전송 시작\"}";

  String response;
  if (!httpPostJsonReliable(DIRECT_ISL_STATUS_URL, payload, &response, "DIRECT_STATUS", 3, 700))
  {
    setIslStatusText("직접전송: status 실패");
    return false;
  }

  char code[12] = "";
  extractJsonStringValue(response, "\"code\"", code, sizeof(code));
  if (strcmp(code, "001") == 0)
  {
    directIslStatusOk = true;
    return true;
  }

  Serial.print("Direct ISL status bad response: ");
  Serial.println(response);
  setIslStatusText("직접전송: status 응답 오류");
  return false;
#else
  return false;
#endif
}

bool directIslPrepareSession(const char *modumId)
{
#if REALTIME_DIRECT_ISL_ENABLED
  if (!directIslStartProcess(modumId)) return false;
  if (!directIslSendSensorTypeIfNeeded()) return false;
  if (!directIslSetStatusOnIfNeeded()) return false;
  return true;
#else
  (void)modumId;
  return false;
#endif
}

bool directIslSendSamplePacket(const CloudSamplePacket &packet)
{
#if REALTIME_DIRECT_ISL_ENABLED
  if (!activeSensorSupportsDirectIsl())
  {
    setIslStatusText("현재 센서: 직접전송 코드 미설정");
    return false;
  }

  for (int attempt = 1; attempt <= 2; attempt++)
  {
    if (!ensureWifiReadyForHttp("실시간전송", 20000)) return false;
    if (!directIslPrepareSession(packet.modumId)) return false;

    char primaryValue[24];
    char pressureValue[24];
    char axisTick[24];
    snprintf(primaryValue, sizeof(primaryValue), "%.4f", packet.tempC);
    formatDirectAxisTick(packet.timeS, axisTick, sizeof(axisTick));

    String payload = "{";
    payload += "\"serviceKey\":\"";
    payload += jsonEscapeString(islServiceKey);
    payload += "\",\"uniqueCode\":\"";
    payload += jsonEscapeString(directIslUniqueCode);
    payload += "\",\"transMethod\":\"01\",\"items\":[";

    if (activeSensorMode == SENSOR_MODE_SCD41)
    {
      // The part measures all three at once; send all three.
      payload += "{\"sensorType\":\"" ISL_SENSOR_TYPE_CO2 "\",\"sensorNicNm\":\"이산화탄소센서\",\"channelCode\":\"" ISL_CHANNEL_CO2 "\",\"sensorData\":\"";
      payload += primaryValue;
      payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
      payload += axisTick;
      payload += "\",\"collectUnit\":\"" ISL_UNIT_CO2 "\"}";

      if (!isnan(packet.pressureHpa))
      {
        char scdTemp[24];
        snprintf(scdTemp, sizeof(scdTemp), "%.4f", packet.pressureHpa);
        payload += ",{\"sensorType\":\"TPR\",\"sensorNicNm\":\"온도센서\",\"channelCode\":\"" ISL_CHANNEL_SCD41_TEMP "\",\"sensorData\":\"";
        payload += scdTemp;
        payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
        payload += axisTick;
        payload += "\",\"collectUnit\":\"C\"}";
      }

      if (!isnan(packet.humidityPct))
      {
        char scdHum[24];
        snprintf(scdHum, sizeof(scdHum), "%.4f", packet.humidityPct);
        payload += ",{\"sensorType\":\"" ISL_SENSOR_TYPE_HUMIDITY "\",\"sensorNicNm\":\"습도센서\",\"channelCode\":\"" ISL_CHANNEL_HUMIDITY "\",\"sensorData\":\"";
        payload += scdHum;
        payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
        payload += axisTick;
        payload += "\",\"collectUnit\":\"" ISL_UNIT_HUMIDITY "\"}";
      }
    }
    else if (activeSensorMode == SENSOR_MODE_TSL2591)
    {
      payload += "{\"sensorType\":\"";
      payload += directIslLightSensorType;
      payload += "\",\"sensorNicNm\":\"조도센서\",\"channelCode\":\"" ISL_CHANNEL_LIGHT "\",\"sensorData\":\"";
      payload += primaryValue;
      payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
      payload += axisTick;
      payload += "\",\"collectUnit\":\"" ISL_UNIT_LIGHT "\"}";
    }
    else if (activeSensorMode == SENSOR_MODE_ENCODER)
    {
      payload += "{\"sensorType\":\"" ISL_SENSOR_TYPE_ANGLE "\",\"sensorNicNm\":\"각도센서\",\"channelCode\":\"01\",\"sensorData\":\"";
      payload += primaryValue;
      payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
      payload += axisTick;
      payload += "\",\"collectUnit\":\"" ISL_UNIT_ANGLE "\"}";

      if (!isnan(packet.pressureHpa))
      {
        char rate[24];
        snprintf(rate, sizeof(rate), "%.4f", packet.pressureHpa);
        payload += ",{\"sensorType\":\"" ISL_SENSOR_TYPE_ANGVEL "\",\"sensorNicNm\":\"각속도센서\",\"channelCode\":\"01\",\"sensorData\":\"";
        payload += rate;
        payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
        payload += axisTick;
        payload += "\",\"collectUnit\":\"" ISL_UNIT_ANGVEL "\"}";
      }
    }
    else if (activeSensorMode == SENSOR_MODE_INA228)
    {
      payload += "{\"sensorType\":\"" ISL_SENSOR_TYPE_CURRENT "\",\"sensorNicNm\":\"전류센서\",\"channelCode\":\"01\",\"sensorData\":\"";
      payload += primaryValue;
      payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
      payload += axisTick;
      payload += "\",\"collectUnit\":\"" ISL_UNIT_CURRENT "\"}";

      if (!isnan(packet.pressureHpa))
      {
        char busV[24];
        snprintf(busV, sizeof(busV), "%.4f", packet.pressureHpa);
        payload += ",{\"sensorType\":\"" ISL_SENSOR_TYPE_VOLTAGE "\",\"sensorNicNm\":\"전압센서\",\"channelCode\":\"01\",\"sensorData\":\"";
        payload += busV;
        payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
        payload += axisTick;
        payload += "\",\"collectUnit\":\"" ISL_UNIT_VOLTAGE "\"}";
      }

      if (!isnan(packet.humidityPct))
      {
        char watts[24];
        snprintf(watts, sizeof(watts), "%.4f", packet.humidityPct);
        payload += ",{\"sensorType\":\"" ISL_SENSOR_TYPE_POWER "\",\"sensorNicNm\":\"전력센서\",\"channelCode\":\"01\",\"sensorData\":\"";
        payload += watts;
        payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
        payload += axisTick;
        payload += "\",\"collectUnit\":\"" ISL_UNIT_POWER "\"}";
      }
    }
    else if (activeSensorMode == SENSOR_MODE_VL53L1X)
    {
      payload += "{\"sensorType\":\"";
      payload += directIslDistanceSensorType;
      payload += "\",\"sensorNicNm\":\"거리센서\",\"channelCode\":\"" ISL_CHANNEL_DISTANCE "\",\"sensorData\":\"";
      payload += primaryValue;
      payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
      payload += axisTick;
      payload += "\",\"collectUnit\":\"" ISL_UNIT_DISTANCE "\"}";
    }
    else
    {
      payload += "{\"sensorType\":\"TPR\",\"sensorNicNm\":\"";
      payload += islTemperatureNickname();
      payload += "\",\"channelCode\":\"01\",\"sensorData\":\"";
      payload += primaryValue;
      payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
      payload += axisTick;
      payload += "\",\"collectUnit\":\"C\"}";
    }

    if (activeSensorMode == SENSOR_MODE_DPS310 && pressureValueValid(packet.pressureHpa))
    {
      snprintf(pressureValue, sizeof(pressureValue), "%.4f", packet.pressureHpa);
      payload += ",";
      payload += "{\"sensorType\":\"PRS\",\"sensorNicNm\":\"기압센서\",\"channelCode\":\"02\",\"sensorData\":\"";
      payload += pressureValue;
      payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
      payload += axisTick;
      payload += "\",\"collectUnit\":\"hPa\"}";
    }

    payload += "]}";

    String response;
    bool ok = httpPostJsonReliableCode(DIRECT_ISL_DATA_URL, payload, &response, "DIRECT_DATA", "001");
    if (ok)
    {
      setIslStatusText("직접전송: 데이터 OK");
      return true;
    }

    Serial.println("DIRECT_DATA failed; reset session and retry once");
    Serial.println(response);
    resetDirectIslSessionCache("data send failed");
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  setIslStatusText("직접전송: 데이터 실패");
  return false;
#else
  (void)packet;
  return false;
#endif
}

bool directIslStopProcess(const char *modumId)
{
#if REALTIME_DIRECT_ISL_ENABLED
  if (!ensureWifiReadyForHttp("직접종료", 15000)) return false;
  if (strlen(directIslUniqueCode) == 0)
  {
    setIslStatusText("직접전송: 종료할 세션 없음");
    return true;
  }

  // 중요:
  // 지능형과학실 실시간 그래프 화면의 [탐구 데이터 불러오기] 버튼은
  // WebSocket으로 status != "on" 상태가 들어와야 활성화됩니다.
  // stopExplortProcess만 보내면 서버 상황에 따라 그래프는 멈춰도
  // 버튼 활성화 이벤트가 늦거나 누락될 수 있어, 종료 시 status=off를 먼저 보냅니다.
  String statusPayload = "{";
  statusPayload += "\"serviceKey\":\"";
  statusPayload += jsonEscapeString(islServiceKey);
  statusPayload += "\",\"uniqueCode\":\"";
  statusPayload += jsonEscapeString(directIslUniqueCode);
  statusPayload += "\",\"status\":\"off\",\"code\":\"001\",\"message\":\"ESP32 직접 전송 종료\"}";

  String statusResponse;
  bool statusOffOk = httpPostJsonReliable(DIRECT_ISL_STATUS_URL, statusPayload, &statusResponse, "DIRECT_STATUS_OFF", 3, 500);

  // status off 직후 stop을 바로 보내면 WebSocket 반영 전에 세션이 닫히는 경우가 있어 짧게 양보합니다.
  vTaskDelay(pdMS_TO_TICKS(250));

  String stopPayload = "{";
  stopPayload += "\"serviceKey\":\"";
  stopPayload += jsonEscapeString(islServiceKey);
  stopPayload += "\",\"uniqueCode\":\"";
  stopPayload += jsonEscapeString(directIslUniqueCode);
  stopPayload += "\"}";

  String stopResponse;
  bool stopOk = httpPostJsonReliable(DIRECT_ISL_STOP_URL, stopPayload, &stopResponse, "DIRECT_STOP", 3, 700);

  // stop 요청 후에는 성공/실패와 무관하게 로컬 세션 캐시를 비운다.
  // 실패한 uniqueCode를 계속 재사용하면 다음 시작/일괄전송이 꼬일 수 있기 때문이다.
  directIslUniqueCode[0] = '\0';
  directIslStartOk = false;
  directIslSensorTypeOk = false;
  directIslStatusOk = false;
  directIslModumId[0] = '\0';

  if (statusOffOk && stopOk)
  {
    setIslStatusText("직접전송: 종료 OK");
  }
  else if (statusOffOk)
  {
    setIslStatusText("직접전송: 종료표시 OK / stop 확인 필요");
  }
  else if (stopOk)
  {
    setIslStatusText("직접전송: stop OK / 불러오기 확인");
  }
  else
  {
    Serial.print("Direct status off failed: ");
    Serial.println(statusResponse);
    Serial.print("Direct stop failed: ");
    Serial.println(stopResponse);
    setIslStatusText("직접전송: 종료 실패 / 캐시 초기화");
  }

  (void)modumId;
  return statusOffOk || stopOk;
#else
  (void)modumId;
  return false;
#endif
}

bool cloudSendBatchHistory()
{
#if REALTIME_DIRECT_ISL_ENABLED
  Serial.println("BATCH CORE: cloudSendBatchHistory() entered");
  Serial.print("BATCH CORE: activeSensorMode=");
  Serial.print(activeSensorMode);
  Serial.print(" / ");
  Serial.println(activeSensorName());
  Serial.print("BATCH CORE: sampleCount=");
  Serial.println(sampleCount);
  Serial.print("BATCH CORE: lightRuntimeType=");
  Serial.println(directIslLightSensorType);

  if (!activeSensorSupportsDirectIsl())
  {
    setIslStatusText("현재 센서: 지능형과학실 센서코드 미설정");
    batchUploading = false;
    requestBatchUiRefresh();
    return false;
  }

  batchUploading = true;
  batchUploadProgressSent = 0;
  if (batchUploadProgressTotal <= 0) batchUploadProgressTotal = sampleCount;
  requestBatchUiRefresh();

  if (!ensureWifiReadyForHttp("일괄전송", 20000))
  {
    setIslStatusText("일괄전송: WiFi 연결 필요/DHCP 실패");
    batchUploading = false;
    requestBatchUiRefresh();
    return false;
  }

  if (!directIslConfigured())
  {
    setIslStatusText("일괄전송: serviceKey/모둠코드 필요");
    batchUploading = false;
    requestBatchUiRefresh();
    return false;
  }

  char modumId[64];
  copyCurrentModumIdTo(modumId, sizeof(modumId));

  if (strlen(modumId) == 0)
  {
    setIslStatusText("일괄전송: 모둠코드 필요");
    batchUploading = false;
    requestBatchUiRefresh();
    return false;
  }

  if (sampleCount <= 0)
  {
    setIslStatusText("일괄전송: 보낼 데이터 없음");
    batchUploading = false;
    requestBatchUiRefresh();
    return false;
  }

  // 전체 화면 overlay는 만들지 않는다.
  // 기존 대시보드 labelCloudMode와 측정 상태 label만 갱신해서 보드 부담을 줄인다.
  int uploadStartIndex = 0;
  int uploadCount = sampleCount;
  int totalChunks = (uploadCount + BATCH_UPLOAD_CHUNK_SIZE - 1) / BATCH_UPLOAD_CHUNK_SIZE;
  int sentRows = 0;

  batchUploadProgressSent = 0;
  batchUploadProgressTotal = uploadCount;
  requestBatchUiRefresh();

  char startLine[128];
  snprintf(startLine, sizeof(startLine), "일괄전송 시작: %d개", uploadCount);
  setIslStatusText(startLine);
  Serial.println(startLine);

  // 새 일괄 세션을 확실하게 잡기 위해 기존 직접 세션 캐시를 초기화한다.
  resetDirectIslSessionCache("batch start");

  Serial.print("BATCH CORE: prepare session modumId=");
  Serial.println(modumId);

  if (!directIslPrepareSession(modumId))
  {
    setIslStatusText("일괄전송: 세션 시작 실패");
    batchUploading = false;
    requestBatchUiRefresh();
    return false;
  }

  Serial.print("BATCH CORE: session prepared, uniqueCode=");
  Serial.println(directIslUniqueCode);
  Serial.print("BATCH CORE: registered light type=");
  Serial.println(directIslLightSensorType);

  bool allOk = true;

  for (int chunk = 0; chunk < totalChunks; chunk++)
  {
    int startIndex = uploadStartIndex + chunk * BATCH_UPLOAD_CHUNK_SIZE;
    int endIndex = startIndex + BATCH_UPLOAD_CHUNK_SIZE;
    if (endIndex > sampleCount) endIndex = sampleCount;

    String payload = "{";
    payload += "\"serviceKey\":\"";
    payload += jsonEscapeString(islServiceKey);
    payload += "\",\"uniqueCode\":\"";
    payload += jsonEscapeString(directIslUniqueCode);
    payload += "\",\"transMethod\":\"02\",\"items\":[";

    bool first = true;
    int chunkRows = 0;

    // sampleEnabled와 무관하게 보드에 누적된 1초 간격 데이터 전체를 전송한다.
    for (int i = startIndex; i < endIndex; i++)
    {
      char axisTick[24];
      char tempValue[24];
      char pressureValue[24];

      formatDirectAxisTick(timeHistory[i], axisTick, sizeof(axisTick));
      snprintf(tempValue, sizeof(tempValue), "%.4f", tempHistory[i]);

      if (!first) payload += ",";
      first = false;

      if (activeSensorMode == SENSOR_MODE_SCD41)
      {
        payload += "{\"sensorType\":\"" ISL_SENSOR_TYPE_CO2 "\",\"sensorNicNm\":\"이산화탄소센서\",\"channelCode\":\"" ISL_CHANNEL_CO2 "\",\"sensorData\":\"";
        payload += tempValue;
        payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
        payload += axisTick;
        payload += "\",\"collectUnit\":\"" ISL_UNIT_CO2 "\"}";

        if (!isnan(pressureHistory[i]))
        {
          char scdTemp[24];
          snprintf(scdTemp, sizeof(scdTemp), "%.4f", pressureHistory[i]);
          payload += ",{\"sensorType\":\"TPR\",\"sensorNicNm\":\"온도센서\",\"channelCode\":\"" ISL_CHANNEL_SCD41_TEMP "\",\"sensorData\":\"";
          payload += scdTemp;
          payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
          payload += axisTick;
          payload += "\",\"collectUnit\":\"C\"}";
        }

        if (!isnan(humidityHistory[i]))
        {
          char scdHum[24];
          snprintf(scdHum, sizeof(scdHum), "%.4f", humidityHistory[i]);
          payload += ",{\"sensorType\":\"" ISL_SENSOR_TYPE_HUMIDITY "\",\"sensorNicNm\":\"습도센서\",\"channelCode\":\"" ISL_CHANNEL_HUMIDITY "\",\"sensorData\":\"";
          payload += scdHum;
          payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
          payload += axisTick;
          payload += "\",\"collectUnit\":\"" ISL_UNIT_HUMIDITY "\"}";
        }
      }
      else if (activeSensorMode == SENSOR_MODE_TSL2591)
      {
        payload += "{\"sensorType\":\"";
        payload += directIslLightSensorType;
        payload += "\",\"sensorNicNm\":\"조도센서\",\"channelCode\":\"" ISL_CHANNEL_LIGHT "\",\"sensorData\":\"";
        payload += tempValue;
        payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
        payload += axisTick;
        payload += "\",\"collectUnit\":\"" ISL_UNIT_LIGHT "\"}";
      }
      else if (activeSensorMode == SENSOR_MODE_ENCODER)
      {
        payload += "{\"sensorType\":\"" ISL_SENSOR_TYPE_ANGLE "\",\"sensorNicNm\":\"각도센서\",\"channelCode\":\"01\",\"sensorData\":\"";
        payload += tempValue;
        payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
        payload += axisTick;
        payload += "\",\"collectUnit\":\"" ISL_UNIT_ANGLE "\"}";

        if (!isnan(pressureHistory[i]))
        {
          char rate[24];
          snprintf(rate, sizeof(rate), "%.4f", pressureHistory[i]);
          payload += ",{\"sensorType\":\"" ISL_SENSOR_TYPE_ANGVEL "\",\"sensorNicNm\":\"각속도센서\",\"channelCode\":\"01\",\"sensorData\":\"";
          payload += rate;
          payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
          payload += axisTick;
          payload += "\",\"collectUnit\":\"" ISL_UNIT_ANGVEL "\"}";
        }
      }
      else if (activeSensorMode == SENSOR_MODE_INA228)
      {
        payload += "{\"sensorType\":\"" ISL_SENSOR_TYPE_CURRENT "\",\"sensorNicNm\":\"전류센서\",\"channelCode\":\"01\",\"sensorData\":\"";
        payload += tempValue;
        payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
        payload += axisTick;
        payload += "\",\"collectUnit\":\"" ISL_UNIT_CURRENT "\"}";

        if (!isnan(pressureHistory[i]))
        {
          char busV[24];
          snprintf(busV, sizeof(busV), "%.4f", pressureHistory[i]);
          payload += ",{\"sensorType\":\"" ISL_SENSOR_TYPE_VOLTAGE "\",\"sensorNicNm\":\"전압센서\",\"channelCode\":\"01\",\"sensorData\":\"";
          payload += busV;
          payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
          payload += axisTick;
          payload += "\",\"collectUnit\":\"" ISL_UNIT_VOLTAGE "\"}";
        }

        if (!isnan(humidityHistory[i]))
        {
          char watts[24];
          snprintf(watts, sizeof(watts), "%.4f", humidityHistory[i]);
          payload += ",{\"sensorType\":\"" ISL_SENSOR_TYPE_POWER "\",\"sensorNicNm\":\"전력센서\",\"channelCode\":\"01\",\"sensorData\":\"";
          payload += watts;
          payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
          payload += axisTick;
          payload += "\",\"collectUnit\":\"" ISL_UNIT_POWER "\"}";
        }
      }
      else if (activeSensorMode == SENSOR_MODE_VL53L1X)
      {
        payload += "{\"sensorType\":\"";
        payload += directIslDistanceSensorType;
        payload += "\",\"sensorNicNm\":\"거리센서\",\"channelCode\":\"" ISL_CHANNEL_DISTANCE "\",\"sensorData\":\"";
        payload += tempValue;
        payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
        payload += axisTick;
        payload += "\",\"collectUnit\":\"" ISL_UNIT_DISTANCE "\"}";
      }
      else
      {
        payload += "{\"sensorType\":\"TPR\",\"sensorNicNm\":\"";
        payload += islTemperatureNickname();
        payload += "\",\"channelCode\":\"01\",\"sensorData\":\"";
        payload += tempValue;
        payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
        payload += axisTick;
        payload += "\",\"collectUnit\":\"C\"}";
      }

      if (activeSensorMode == SENSOR_MODE_DPS310 && pressureValueValid(pressureHistory[i]))
      {
        snprintf(pressureValue, sizeof(pressureValue), "%.4f", pressureHistory[i]);
        payload += ",";
        payload += "{\"sensorType\":\"PRS\",\"sensorNicNm\":\"기압센서\",\"channelCode\":\"02\",\"sensorData\":\"";
        payload += pressureValue;
        payload += "\",\"dataType\":\"01\",\"collectDate\":\"";
        payload += axisTick;
        payload += "\",\"collectUnit\":\"hPa\"}";
      }

      chunkRows++;
    }

    payload += "]}";

    if (activeSensorMode == SENSOR_MODE_TSL2591)
    {
      Serial.print("LIGHT BATCH REQUEST chunk ");
      Serial.print(chunk + 1);
      Serial.print("/");
      Serial.println(totalChunks);
      Serial.println(payload);
    }

    String response;
    bool ok = httpPostJsonReliableCode(DIRECT_ISL_DATA_URL, payload, &response, "DIRECT_BATCH_DATA", "001");

    if (activeSensorMode == SENSOR_MODE_TSL2591)
    {
      Serial.print("LIGHT BATCH RESPONSE: ");
      Serial.println(response);
    }
    if (!ok)
    {
      allOk = false;
      Serial.print("Direct batch failed at chunk ");
      Serial.print(chunk + 1);
      Serial.print("/");
      Serial.println(totalChunks);
      Serial.print("Payload length: ");
      Serial.println(payload.length());
      if (activeSensorMode == SENSOR_MODE_TSL2591)
      {
        Serial.print("Light batch payload: ");
        Serial.println(payload);
      }
      Serial.println(response);
      setIslStatusText("일괄전송: 전송 실패");
      break;
    }

    sentRows += chunkRows;
    batchUploadProgressSent = sentRows;
    batchUploadProgressTotal = uploadCount;
    requestBatchUiRefresh();

    char statusLine[128];
    snprintf(statusLine, sizeof(statusLine), "일괄전송 중: %d/%d개", sentRows, uploadCount);
    setIslStatusText(statusLine);

    Serial.print("Direct batch OK ");
    Serial.print(chunk + 1);
    Serial.print("/");
    Serial.print(totalChunks);
    Serial.print(" rows=");
    Serial.println(chunkRows);
    Serial.println(response);

    // 서버 부담은 낮추되, 불필요한 LVGL 객체 생성은 하지 않는다.
    vTaskDelay(pdMS_TO_TICKS(300));
  }

  bool stopOk = directIslStopProcess(modumId);

  batchUploading = false;
  requestBatchUiRefresh();

  if (allOk && sentRows == uploadCount)
  {
    batchUploadStartIndex = sampleCount;
    batchUploadProgressSent = sentRows;
    batchUploadProgressTotal = uploadCount;

    char doneLine[128];
    snprintf(doneLine, sizeof(doneLine), "일괄전송 완료: %d개%s", sentRows, stopOk ? "" : " / stop 확인 필요");
    setIslStatusText(doneLine);
    return true;
  }

  char failLine[128];
  snprintf(failLine, sizeof(failLine), "일괄전송 실패: %d/%d개", sentRows, uploadCount);
  setIslStatusText(failLine);
  return false;
#else
  setIslStatusText("일괄전송: 비활성");
  batchUploading = false;
  requestBatchUiRefresh();
  return false;
#endif
}

// Routes a queued packet to 지능형 과학실.
//
// A Cloud Run relay used to sit behind this, forwarding to Google Sheets. It
// was already unreachable for measurements — every start/stop/data/batch path
// returned above it once REALTIME_DIRECT_ISL_ENABLED was on — and the only
// traffic left for it was a button log switched off by a separate flag.
bool cloudSendSamplePacket(const CloudSamplePacket &packet)
{
#if REALTIME_DIRECT_ISL_ENABLED
  if (!ensureWifiReadyForHttp("전송", 20000)) return false;

  if (strcmp(packet.action, "batch") == 0) return cloudSendBatchHistory();
  if (strcmp(packet.action, "start") == 0) return directIslPrepareSession(packet.modumId);
  if (strcmp(packet.action, "stop") == 0) return directIslStopProcess(packet.modumId);
  if (strlen(packet.action) == 0) return directIslSendSamplePacket(packet);

  return true;
#else
  (void)packet;
  return false;
#endif
}

void copyCurrentModumIdTo(char *out, size_t outSize)
{
  if (out == NULL || outSize == 0) return;
  out[0] = '\0';

  char code[64];
  sanitizeIslModuleCode(islRuntimeSerialNumber, code, sizeof(code));
  if (strlen(code) > 0)
  {
    strncpy(out, code, outSize - 1);
    out[outSize - 1] = '\0';
  }
}

void clearCloudQueue()
{
#if UPLOAD_QUEUE_ENABLED
  if (cloudQueue == NULL) return;

  CloudSamplePacket dropped;
  while (xQueueReceive(cloudQueue, &dropped, 0) == pdPASS)
  {
    cloudQueuedDropped++;
  }
#endif
}

void sendCloudPacketLatestOnly(const CloudSamplePacket &packet)
{
#if UPLOAD_QUEUE_ENABLED
  if (cloudQueue == NULL) return;

  // CLOUD_QUEUE_DEPTH=1 운영 기준: 오래된 패킷은 버리고 최신 패킷만 유지합니다.
  if (xQueueSend(cloudQueue, &packet, 0) != pdPASS)
  {
    CloudSamplePacket dropped;
    xQueueReceive(cloudQueue, &dropped, 0);
    cloudQueuedDropped++;
    xQueueSend(cloudQueue, &packet, 0);
  }
#else
  (void)packet;
#endif
}

void queueCloudAction(const char *action)
{
#if UPLOAD_QUEUE_ENABLED
  if (cloudQueue == NULL) return;
  if (action == NULL || strlen(action) == 0) return;

  CloudSamplePacket packet = {};
  strncpy(packet.action, action, sizeof(packet.action) - 1);
  copyCurrentModumIdTo(packet.modumId, sizeof(packet.modumId));

  if (strlen(packet.modumId) == 0)
  {
    setIslStatusText("설정: 모둠코드 필요");
    return;
  }

  // stop은 사용자가 누르는 즉시 반영되어야 하므로 기존 data/event 큐를 모두 비우고 맨 앞에 넣는다.
  // 실시간 직접 전송 중 HTTP 1건이 이미 진행 중이면 그 요청이 끝난 뒤 바로 stop이 실행된다.
  if (strcmp(action, "stop") == 0 || strcmp(action, "batch") == 0)
  {
    clearCloudQueue();
    if (xQueueSendToFront(cloudQueue, &packet, 0) != pdPASS)
    {
      cloudQueuedDropped++;
    }
  }
  else
  {
    sendCloudPacketLatestOnly(packet);
  }

  char statusLine[128];
  snprintf(statusLine, sizeof(statusLine), "%s 요청 대기", strcmp(action, "stop") == 0 ? "정지" : (strcmp(action, "batch") == 0 ? "일괄전송" : action));
  setIslStatusText(statusLine);
#else
  (void)action;
#endif
}


void queueCloudSample(int no, uint32_t timeS, float tempC, float pressureHpa)
{
#if UPLOAD_QUEUE_ENABLED
  if (cloudQueue == NULL) return;


  if (cloudUploadMode == CLOUD_UPLOAD_BATCH)
  {
    if (no % 10 == 0)
    {
      setIslStatusText("일괄전송 중");
      updateCloudModeLabel();
    }
    return;
  }

  if (!activeSensorSupportsDirectIsl())
  {
    if (no % 10 == 0)
    {
      setIslStatusText("현재 센서: 보드 기록 중 / ON 전송 미지원");
    }
    return;
  }

  // 지능형과학실은 HTTPS라 매 샘플마다 보내면 TLS 연결 비용이 큽니다.
  // CLOUD_SEND_EVERY_N_SAMPLES=1이면 1초 샘플마다 전송을 시도합니다.
  if (CLOUD_SEND_EVERY_N_SAMPLES > 1 && (no % CLOUD_SEND_EVERY_N_SAMPLES) != 0)
  {
    return;
  }

  CloudSamplePacket packet = {};
  packet.no = no;
  packet.timeS = timeS;
  packet.tempC = tempC;
  packet.pressureHpa = pressureHpa;
  // Read straight after the sample that produced it, in the same loop pass.
  packet.humidityPct = scd41LastHumidityPct;
  formatIslCollectDate(packet.collectDate, sizeof(packet.collectDate));
  snprintf(packet.sensorName, sizeof(packet.sensorName), "%s", activeSensorName());
  copyCurrentModumIdTo(packet.modumId, sizeof(packet.modumId));

  if (strlen(packet.modumId) == 0)
  {
    // 모둠코드가 없으면 지능형과학실 기본값으로 들어가므로 유동 모둠 운영에서는 위험합니다.
    // 의도치 않은 모둠으로 전송하지 않도록 전송을 막습니다.
    setIslStatusText("설정: 모둠코드 필요");
    return;
  }

  sendCloudPacketLatestOnly(packet);
#else
  (void)no;
  (void)timeS;
  (void)tempC;
  (void)pressureHpa;
#endif
}

bool c6WifiConnectBackground(const char *ssid, const char *password)
{
  if (ssid == NULL || strlen(ssid) == 0) return false;

#if HAS_IDF_WIFI
  if (!ensureHostedWifiStarted()) return false;

  wifi_config_t wifiConfig;
  memset(&wifiConfig, 0, sizeof(wifiConfig));
  strncpy((char *)wifiConfig.sta.ssid, ssid, sizeof(wifiConfig.sta.ssid) - 1);

  if (password != NULL)
  {
    strncpy((char *)wifiConfig.sta.password, password, sizeof(wifiConfig.sta.password) - 1);
  }

  esp_wifi_disconnect();

  esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &wifiConfig);
  if (ret != ESP_OK) return false;

  ret = esp_wifi_connect();
  if (ret != ESP_OK) return false;

  unsigned long start = millis();
  wifi_ap_record_t apInfo;

  while (millis() - start < 12000)
  {
    if (esp_wifi_sta_get_ap_info(&apInfo) == ESP_OK)
    {
      unsigned long ipStart = millis();

      while (millis() - ipStart < 8000)
      {
        if (wifiStaNetif != NULL)
        {
          esp_netif_ip_info_t ipInfo;
          if (esp_netif_get_ip_info(wifiStaNetif, &ipInfo) == ESP_OK && ipInfo.ip.addr != 0)
          {
            verifyDnsOrFallback();
            return true;
          }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
      }

      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }

  return false;
#else
#if HAS_ARDUINO_WIFI
  WiFi.mode(WIFI_STA);

  if (password != NULL && strlen(password) > 0) WiFi.begin(ssid, password);
  else WiFi.begin(ssid);

  unsigned long start = millis();

  while (millis() - start < 12000)
  {
    if (WiFi.status() == WL_CONNECTED && WiFi.localIP().toString() != "0.0.0.0") return true;
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  return false;
#else
  return false;
#endif
#endif
}

void tryAutoConnectSavedWifiBackground()
{
  if (wifiConnected) return;

  if (!loadWifiCredentials())
  {
    setIslStatusText("WiFi/API: 저장 WiFi 없음");
    return;
  }

  setIslStatusText("WiFi/API: 백그라운드 연결 중");
  wifiConnected = c6WifiConnectBackground(wifiSavedSsid, wifiSavedPassword);

  if (wifiConnected)
  {
    resetDirectIslSessionCache("WiFi background reconnect");
    setIslStatusText("WiFi/API: WiFi 연결됨");
    configTime(
      9 * 3600,
      0,
      "pool.ntp.org",
      "time.google.com",
      "time.cloudflare.com"
    );
  }
  else
  {
    setIslStatusText("WiFi/API: WiFi 연결 실패");
  }
}

void wifiApiTask(void *parameter)
{
  (void)parameter;

  unsigned long lastWifiAttempt = 0;
  bool bleStartAttempted = false;

  while (true)
  {
    unsigned long now = millis();

    // Bluetooth shares the ESP-Hosted link with WiFi, so it is brought up here
    // rather than in setup(): the transport costs about 1.5 s and this task is
    // already the one that waits on it.
    if (!bleStartAttempted && bleEnabled)
    {
      bleStartAttempted = true;

      if (ensureEspHostedTransport())
      {
        char bleName[32];
        const size_t codeLen = strlen(islRuntimeSerialNumber);
        const char *tail = codeLen > 4 ? islRuntimeSerialNumber + codeLen - 4 : "";
        snprintf(bleName, sizeof(bleName), "SciSensor%s%s", codeLen > 4 ? "-" : "", tail);

        if (bleSensorBegin(bleName))
        {
          // Let the host settle, then look around once. The scan report names
          // every service UUID nearby, which is what identifies a sensor whose
          // protocol is not documented here.
          vTaskDelay(pdMS_TO_TICKS(2000));
          bleScanStart(6000);
        }
      }
    }

    if (wifiConnected && !wifiReadyForHttp())
    {
      if (now - lastWifiLostNoticeMs > 3000)
      {
        lastWifiLostNoticeMs = now;
        setIslStatusText("WiFi: 연결 끊김/DHCP 손실 - 재연결 대기");
        resetDirectIslSessionCache("WiFi lost in task");
      }

      httpCloseConnection("WiFi lost");
      wifiConnected = false;
      lastWifiAttempt = 0;
    }

    if (wifiAutoRetryEnabled && !wifiConnected && now - lastWifiAttempt >= wifiAutoRetryIntervalMs)
    {
      lastWifiAttempt = now;
      tryAutoConnectSavedWifiBackground();
    }

    // =====================================================
    // V3 DEBUG: 일괄전송 전용 요청 처리
    // - cloudQueue의 "batch" packet에 의존하지 않습니다.
    // - WiFi 연결 여부와 상관없이 요청 자체는 task가 반드시 회수합니다.
    // - 실제 전송 함수 안에서 WiFi/IP 상태를 다시 검증합니다.
    // =====================================================
    if (batchUploadRequested)
    {
      unsigned long requestAge = now - batchUploadRequestMs;

      Serial.println();
      Serial.println("========== BATCH WORKER START ==========");
      Serial.print("BATCH WORKER: requestAgeMs=");
      Serial.println(requestAge);
      Serial.print("BATCH WORKER: sensorMode=");
      Serial.print(activeSensorMode);
      Serial.print(" sensor=");
      Serial.println(activeSensorName());
      Serial.print("BATCH WORKER: sampleCount=");
      Serial.println(sampleCount);
      Serial.print("BATCH WORKER: wifiConnected=");
      Serial.println(wifiConnected ? 1 : 0);
      Serial.print("BATCH WORKER: wifiReadyForHttp=");
      Serial.println(wifiReadyForHttp() ? 1 : 0);

      batchUploadRequested = false;
      batchUploading = true;
      requestBatchUiRefresh();
      setIslStatusText("일괄전송 작업 시작");

      bool batchOk = cloudSendBatchHistory();

      // cloudSendBatchHistory()가 모든 경로에서 정리하지만
      // 예외적인 early return에도 상태가 고정되지 않도록 여기서 한 번 더 정리합니다.
      batchUploading = false;
      batchUploadRequested = false;
      requestBatchUiRefresh();

      Serial.print("BATCH WORKER RESULT: ");
      Serial.println(batchOk ? "SUCCESS" : "FAIL");
      Serial.println("=========== BATCH WORKER END ===========");
    }

    if (wifiConnected && cloudQueue != NULL)
    {
      CloudSamplePacket cloudPacket;
      if (xQueueReceive(cloudQueue, &cloudPacket, pdMS_TO_TICKS(10)) == pdPASS)
      {
        if (!cloudSendSamplePacket(cloudPacket))
        {
          // 실패한 패킷을 계속 맨 앞에 재삽입하면 같은 데이터만 반복 전송 시도하면서
          // 새 측정값 전송이 막힙니다. 실패 패킷은 버리고 다음 측정값을 계속 시도합니다.
          cloudQueuedDropped++;
          Serial.print("Cloud packet dropped after send failure. dropped=");
          Serial.println(cloudQueuedDropped);
          vTaskDelay(pdMS_TO_TICKS(3000));
        }
      }
    }


    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void startWifiApiTask()
{
  if (cloudQueue == NULL)
  {
    cloudQueue = xQueueCreate(CLOUD_QUEUE_DEPTH, sizeof(CloudSamplePacket));
  }

  if (wifiApiTaskHandle == NULL)
  {
    // This task runs the HTTPS uploads. An mbedTLS handshake with certificate
    // bundle verification needs well over 12 kB of stack; the old 12288 value
    // overflowed and showed up as a random freeze or reboot during WiFi work.
    xTaskCreatePinnedToCore(
      wifiApiTask,
      "wifi_api_task",
      WIFI_API_TASK_STACK_BYTES,
      NULL,
      1,
      &wifiApiTaskHandle,
      0
    );
  }
}

void updateHomeWifiLabels()
{
  char buf[80];

  if (!wifiConnected)
  {
    if (labelHomeWifi) lv_label_set_text(labelHomeWifi, "WiFi: 연결 안 됨");
    if (labelHomeIp) lv_label_set_text(labelHomeIp, "IP: --");
    if (labelHomeSignal) lv_label_set_text(labelHomeSignal, "신호: --");
    return;
  }

  if (labelHomeWifi)
  {
    if (strlen(wifiSavedSsid) > 0)
    {
      snprintf(buf, sizeof(buf), "WiFi: %s", wifiSavedSsid);
    }
    else
    {
      snprintf(buf, sizeof(buf), "WiFi: 연결됨");
    }

    lv_label_set_text(labelHomeWifi, buf);
  }

  String ip = getWifiIpText();

  if (labelHomeIp)
  {
    if (ip == "--")
    {
      lv_label_set_text(labelHomeIp, "IP: DHCP 대기");
    }
    else
    {
      snprintf(buf, sizeof(buf), "IP: %s", ip.c_str());
      lv_label_set_text(labelHomeIp, buf);
    }
  }

  int rssi = getWifiRssiValue();

  if (labelHomeSignal)
  {
    if (rssi != 0)
    {
      snprintf(buf, sizeof(buf), "신호: %d dBm", rssi);
    }
    else
    {
      snprintf(buf, sizeof(buf), "신호: --");
    }

    lv_label_set_text(labelHomeSignal, buf);
  }
}


void updateWifiRuntimeLabels()
{
  if (!wifiConnected) return;

  char info[64];
  String ip = getWifiIpText();

  if (ip == "--")
  {
    snprintf(info, sizeof(info), "IP: DHCP 대기");
  }
  else
  {
    snprintf(info, sizeof(info), "IP: %s", ip.c_str());
  }

  if (labelWifiIp) lv_label_set_text(labelWifiIp, info);

  int rssi = getWifiRssiValue();
  if (rssi != 0)
  {
    snprintf(info, sizeof(info), "신호: %ddBm", rssi);
  }
  else
  {
    snprintf(info, sizeof(info), "신호: --");
  }

  if (labelWifiSignal) lv_label_set_text(labelWifiSignal, info);

  // 추가
  updateHomeWifiLabels();
}



bool syncNtpTime()
{
  if (!wifiConnected)
  {
    return false;
  }

  Serial.println("NTP sync start");

  configTime(
    9 * 3600,
    0,
    "pool.ntp.org",
    "time.google.com",
    "time.cloudflare.com"
  );

  unsigned long start = millis();

  while (millis() - start < 10000)
  {
    uiTimerHandler();

    time_t nowTime;
    struct tm timeInfo;
    time(&nowTime);

    if (localtime_r(&nowTime, &timeInfo) != NULL && (timeInfo.tm_year + 1900) >= 2025)
    {
      ntpSynced = true;
      Serial.println("NTP sync OK");
      updateStatusBars();
      updateClockLabels();
      return true;
    }

    delay(200);
  }

  ntpSynced = false;
  Serial.println("NTP sync timeout");
  return false;
}

void tryAutoConnectSavedWifi()
{
  if (!loadWifiCredentials())
  {
    if (labelWifiState) lv_label_set_text(labelWifiState, "WiFi: 저장 정보 없음");
    if (labelSettingsWifi) lv_label_set_text(labelSettingsWifi, "WiFi: 저장 정보 없음");
    return;
  }

  if (labelWifiState) lv_label_set_text(labelWifiState, "WiFi: 자동 연결 중");
  if (labelSettingsWifi) lv_label_set_text(labelSettingsWifi, "WiFi: 자동 연결 중");
  uiTimerHandler();

  wifiConnected = c6WifiConnect(wifiSavedSsid, wifiSavedPassword);

  if (wifiConnected)
  {
    resetDirectIslSessionCache("WiFi auto connect");
    if (labelWifiState) lv_label_set_text(labelWifiState, "WiFi: 자동 연결됨");
    updateWifiRuntimeLabels();
    syncNtpTime();
    if (labelSettingsWifi) lv_label_set_text(labelSettingsWifi, "WiFi: 자동 연결됨");
    Serial.println("Saved WiFi auto connect OK");
  }
  else
  {
    if (labelWifiState) lv_label_set_text(labelWifiState, "WiFi: 자동 연결 실패");
    if (labelSettingsWifi) lv_label_set_text(labelSettingsWifi, "WiFi: 자동 연결 실패");
    Serial.println("Saved WiFi auto connect failed");
  }
}

static void wifi_scan_item_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  const char *ssid = (const char *)lv_event_get_user_data(e);

  if (ssid && wifiSsidTa)
  {
    lv_textarea_set_text(wifiSsidTa, ssid);
    lv_label_set_text(labelWifiState, "WiFi: SSID 선택됨");
  }
}

void populateWifiScanList(int count)
{
  if (wifiList)
  {
    lv_obj_clean(wifiList);
  }

  if (count <= 0)
  {
    if (labelWifiScan)
    {
      lv_label_set_text(labelWifiScan, "검색 결과 없음 / ESP-Hosted 확인");
    }
    return;
  }

  char line[80];

  for (int i = 0; i < count && i < WIFI_SCAN_MAX; i++)
  {
    snprintf(
      line,
      sizeof(line),
      "%s  %ddBm  %s",
      wifiScanResults[i].ssid,
      wifiScanResults[i].rssi,
      wifiScanResults[i].secure ? "LOCK" : "OPEN"
    );

    if (wifiList)
    {
      lv_obj_t *btn = lv_list_add_btn(wifiList, NULL, line);
      lv_obj_set_style_text_font(btn, FONT_KR, 0);
      lv_obj_add_event_cb(btn, wifi_scan_item_event_cb, LV_EVENT_CLICKED, wifiScanResults[i].ssid);
    }
  }

  if (labelWifiScan)
  {
    snprintf(line, sizeof(line), "%d개 검색됨", count);
    lv_label_set_text(labelWifiScan, line);
  }
}

void bootScanThenAutoConnectWifi()
{
  if (labelWifiState) lv_label_set_text(labelWifiState, "WiFi: 부팅 검색 중");
  if (labelSettingsWifi) lv_label_set_text(labelSettingsWifi, "WiFi: 부팅 검색 중");
  if (labelWifiScan) lv_label_set_text(labelWifiScan, "부팅 검색 중...");
  uiTimerHandler();

  int count = c6WifiScan(wifiScanResults, WIFI_SCAN_MAX);
  populateWifiScanList(count);

  if (!loadWifiCredentials())
  {
    if (labelWifiState) lv_label_set_text(labelWifiState, "WiFi: 저장 정보 없음");
    if (labelSettingsWifi) lv_label_set_text(labelSettingsWifi, "WiFi: 저장 정보 없음");
    return;
  }

  bool savedSsidVisible = false;

  for (int i = 0; i < count && i < WIFI_SCAN_MAX; i++)
  {
    if (strcmp(wifiScanResults[i].ssid, wifiSavedSsid) == 0)
    {
      savedSsidVisible = true;
      break;
    }
  }

  if (count > 0 && !savedSsidVisible)
  {
    if (labelWifiState) lv_label_set_text(labelWifiState, "WiFi: 저장 SSID 없음");
    if (labelSettingsWifi) lv_label_set_text(labelSettingsWifi, "WiFi: 저장 SSID 없음");
    Serial.println("Saved WiFi SSID not found in scan result");
    return;
  }

  if (labelWifiState) lv_label_set_text(labelWifiState, "WiFi: 자동 연결 중");
  if (labelSettingsWifi) lv_label_set_text(labelSettingsWifi, "WiFi: 자동 연결 중");
  uiTimerHandler();

  wifiConnected = c6WifiConnect(wifiSavedSsid, wifiSavedPassword);

  if (wifiConnected)
  {
    resetDirectIslSessionCache("WiFi boot auto connect");
    if (labelWifiState) lv_label_set_text(labelWifiState, "WiFi: 자동 연결됨");
    updateWifiRuntimeLabels();
    syncNtpTime();
    if (labelSettingsWifi) lv_label_set_text(labelSettingsWifi, "WiFi: 자동 연결됨");
    Serial.println("Boot WiFi scan and auto connect OK");
  }
  else
  {
    if (labelWifiState) lv_label_set_text(labelWifiState, "WiFi: 자동 연결 실패");
    if (labelSettingsWifi) lv_label_set_text(labelSettingsWifi, "WiFi: 자동 연결 실패");
    Serial.println("Boot WiFi auto connect failed");
  }
}

static void wifi_scan_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (measuring)
  {
    if (labelWifiScan) lv_label_set_text(labelWifiScan, "측정 정지 후 검색");
    return;
  }
  if (pendingWifiCommand != WIFI_UI_NONE) return;
  if (labelWifiScan) lv_label_set_text(labelWifiScan, "검색 대기...");
  pendingWifiCommand = WIFI_UI_SCAN;
}

static void wifi_textarea_event_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_FOCUSED)
  {
    lv_obj_t *ta = lv_event_get_target(e);

    if (wifiKeyboard)
    {
      lv_keyboard_set_textarea(wifiKeyboard, ta);
      lv_obj_clear_flag(wifiKeyboard, LV_OBJ_FLAG_HIDDEN);
    }
  }
}

static void wifi_keyboard_event_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL)
  {
    if (wifiKeyboard)
    {
      lv_obj_add_flag(wifiKeyboard, LV_OBJ_FLAG_HIDDEN);
      lv_keyboard_set_textarea(wifiKeyboard, NULL);
    }
  }
}

static void wifi_connect_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (measuring)
  {
    if (labelWifiState) lv_label_set_text(labelWifiState, "WiFi: 측정 정지 후 연결");
    return;
  }
  if (pendingWifiCommand != WIFI_UI_NONE) return;

  const char *ssid = wifiSsidTa ? lv_textarea_get_text(wifiSsidTa) : "";
  const char *password = wifiPassTa ? lv_textarea_get_text(wifiPassTa) : "";
  strncpy(pendingWifiSsid, ssid ? ssid : "", sizeof(pendingWifiSsid) - 1);
  pendingWifiSsid[sizeof(pendingWifiSsid) - 1] = '\0';
  strncpy(pendingWifiPassword, password ? password : "", sizeof(pendingWifiPassword) - 1);
  pendingWifiPassword[sizeof(pendingWifiPassword) - 1] = '\0';
  if (labelWifiState) lv_label_set_text(labelWifiState, "WiFi: 연결 대기...");
  pendingWifiCommand = WIFI_UI_CONNECT;
}

static void wifi_disconnect_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (pendingWifiCommand == WIFI_UI_NONE) pendingWifiCommand = WIFI_UI_DISCONNECT;
}

static void wifi_forget_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  if (pendingWifiCommand == WIFI_UI_NONE) pendingWifiCommand = WIFI_UI_FORGET;
}

void servicePendingWifiCommand()
{
  WifiUiCommand command = pendingWifiCommand;
  if (command == WIFI_UI_NONE || uiInputLocked) return;

  pendingWifiCommand = WIFI_UI_NONE;
  uiInputLocked = true;
  ignoreTouchUntilMs = millis() + 250;

  if (command == WIFI_UI_SCAN)
  {
    if (labelWifiScan) lv_label_set_text(labelWifiScan, "검색 중...");
    if (wifiList) lv_obj_clean(wifiList);
    int count = c6WifiScan(wifiScanResults, WIFI_SCAN_MAX);
    populateWifiScanList(count);
  }
  else if (command == WIFI_UI_CONNECT)
  {
    strncpy(wifiSavedSsid, pendingWifiSsid, sizeof(wifiSavedSsid) - 1);
    wifiSavedSsid[sizeof(wifiSavedSsid) - 1] = '\0';
    strncpy(wifiSavedPassword, pendingWifiPassword, sizeof(wifiSavedPassword) - 1);
    wifiSavedPassword[sizeof(wifiSavedPassword) - 1] = '\0';

    // Save before connecting: the entry is worth keeping whether or not the
    // access point answers, and retyping a password on a touch keyboard to
    // retry is the worst part of a failed attempt.
    saveWifiCredentials(wifiSavedSsid, wifiSavedPassword);

    if (labelWifiState) lv_label_set_text(labelWifiState, "WiFi: 연결 시도 중");
    wifiConnected = c6WifiConnect(wifiSavedSsid, wifiSavedPassword);
    if (wifiConnected)
    {
      resetDirectIslSessionCache("WiFi manual connect");
      configTime(9 * 3600, 0, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
      if (labelWifiState) lv_label_set_text(labelWifiState, "WiFi: 연결됨");
      if (labelWifiMode) lv_label_set_text(labelWifiMode, "방식: ESP32-C6 ESP-Hosted / 수동");
      if (labelSettingsWifi) lv_label_set_text(labelSettingsWifi, "WiFi: 연결됨");
      updateWifiRuntimeLabels();
    }
    else
    {
      if (labelWifiState) lv_label_set_text(labelWifiState, "WiFi: 연결 실패");
      if (labelWifiIp) lv_label_set_text(labelWifiIp, "IP: --");
      if (labelWifiSignal) lv_label_set_text(labelWifiSignal, "신호: --");
      if (labelSettingsWifi) lv_label_set_text(labelSettingsWifi, "WiFi: 연결 실패");
    }
  }
  else if (command == WIFI_UI_DISCONNECT)
  {
    c6WifiDisconnect();
    wifiConnected = false;
    resetDirectIslSessionCache("WiFi manual disconnect");
    if (labelWifiState) lv_label_set_text(labelWifiState, "WiFi: 연결 해제");
    if (labelWifiIp) lv_label_set_text(labelWifiIp, "IP: --");
    if (labelWifiSignal) lv_label_set_text(labelWifiSignal, "신호: --");
    if (labelSettingsWifi) lv_label_set_text(labelSettingsWifi, "WiFi: 연결 해제");
    updateHomeWifiLabels();
  }
  else if (command == WIFI_UI_FORGET)
  {
    clearWifiCredentials();
    wifiSavedSsid[0] = '\0';
    wifiSavedPassword[0] = '\0';
    if (labelWifiState) lv_label_set_text(labelWifiState, "WiFi: 저장 정보 삭제됨");
    if (labelSettingsWifi) lv_label_set_text(labelSettingsWifi, "WiFi: 저장 정보 없음");
  }

  uiInputLocked = false;
  ignoreTouchUntilMs = millis() + 250;
}

// =====================================================
// Setup
// =====================================================
void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("ESP32-P4 Dashboard V3.8 TMP117 + VL53L1X start");
  Serial.printf("[RESET] esp_reset_reason=%d\n", (int)esp_reset_reason());

  initNvsStorage();
  // CSV disabled in stability build.
  loadIslModuleCode();
  loadIslServiceKey();

  Serial.printf("[PIN] TP_I2C_SDA=%d TP_I2C_SCL=%d | SENSOR_SDA=%d SENSOR_SCL=%d DS18B20_DQ=%d\n",
                (int)TP_I2C_SDA, (int)TP_I2C_SCL, SOFT_SDA, SOFT_SCL, (int)DS18B20_DQ_GPIO);
  if ((int)TP_I2C_SDA == SOFT_SDA || (int)TP_I2C_SCL == SOFT_SCL ||
      (int)TP_I2C_SDA == (int)DS18B20_DQ_GPIO || (int)TP_I2C_SCL == (int)DS18B20_DQ_GPIO)
  {
    Serial.println("[WARN] Touch I2C pin overlaps with sensor pin. Move sensor to a different GPIO.");
  }


  Serial.println("[BOOT] before lcd.begin()");
  lcd.begin();
  delay(120);
  Serial.println("[BOOT] after lcd.begin()");

  // Let the touch rail settle after the LCD comes up. The controller does its
  // own reset sequence in begin(), so this only has to cover the supply.
  delay(200);

#if ENABLE_GT911_TOUCH
  Serial.println("[BOOT] before GT911 touch.begin()");
  touchReady = touch.begin();
  Serial.println(touchReady ? "[BOOT] GT911 touch ready" : "[WARN] GT911 touch unavailable");
#else
  Serial.println("[WARN] GT911 touch disabled by ENABLE_GT911_TOUCH=0");
#endif

  Serial.println("[BOOT] before lv_init()");
  lv_init();
  Serial.println("[BOOT] after lv_init()");

  const int lvglBufferRows = (LVGL_BUFFER_ROWS > LCD_V_RES) ? LCD_V_RES : LVGL_BUFFER_ROWS;
  size_t bufferSize = sizeof(lv_color_t) * LCD_H_RES * lvglBufferRows;
  Serial.printf("[LVGL] buffer rows=%d, bytes per buffer=%u\n", lvglBufferRows, (unsigned int)bufferSize);

#if LVGL_USE_INTERNAL_DMA_BUFFER
  // Use a single small internal DMA buffer.
  // This avoids the PSRAM strip artifact while keeping internal RAM pressure low.
  buf1 = (lv_color_t *)heap_caps_malloc(
    bufferSize,
    MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT
  );
  buf2 = NULL;

  if (buf1 == NULL)
  {
    Serial.println("[LVGL] single internal DMA buffer failed. Falling back to single PSRAM buffer.");
  }
#else
  buf1 = NULL;
  buf2 = NULL;
#endif

  if (buf1 == NULL)
  {
    buf1 = (lv_color_t *)heap_caps_malloc(
      bufferSize,
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );
    buf2 = NULL;
  }

  if (buf1 == NULL)
  {
    Serial.println("LVGL buffer allocation failed");

    while (true)
    {
      delay(1000);
    }
  }

  Serial.printf("[LVGL] buf1=%p buf2=%p single_dma=%d\n", buf1, buf2, LVGL_SINGLE_DMA_BUFFER);

  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LCD_H_RES * lvglBufferRows);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);

  disp_drv.hor_res = LCD_H_RES;
  disp_drv.ver_res = LCD_V_RES;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.full_refresh = LVGL_USE_FULL_REFRESH ? true : false;
  disp_drv.drv_update_cb = lvgl_port_update_callback;

  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);

  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;

  lv_indev_drv_register(&indev_drv);

  createHomeUi();
  createMeasureUi();
  createSettingsUi();
  createBleUi();
  createCsvUi();
  createFileViewerUi();
  createIslUi();
  // its drawing buffers were cut down to stubs.

  // Allocate every main screen base during setup, not on the first user tap.
  ensureOpaqueScreenBase(homeScreen);
  ensureOpaqueScreenBase(measureScreen);
  ensureOpaqueScreenBase(settingsScreen);
  ensureOpaqueScreenBase(islScreen);
  ensureOpaqueScreenBase(bleScreen);
  ensureOpaqueScreenBase(csvScreen);
  ensureOpaqueScreenBase(fileViewerScreen);

  // Put the saved network back in the fields now that they exist, so Settings
  // shows the remembered entry rather than blanks.
  loadWifiCredentials();

  bleEnabled = loadBleEnabled();
  Serial.print("[BLE] enabled at boot: ");
  Serial.println(bleEnabled ? 1 : 0);

  activateScreenNow(homeScreen);

  for (int i = 0; i < 30; i++)
  {
    uiTimerHandler();
    delay(10);
  }

#if SD_AUTO_MOUNT_ON_BOOT
  sdReady = initSdCard();
  csvLoggingEnabled = sdReady;
#else
  sdReady = false;
  csvLoggingEnabled = CSV_LOG_DEFAULT;
#endif

  updateSdStatusLabels();

  restoreMeasurementBackupFromNvs();
  dpsReady = activeSensorBegin();
  updateActiveSensorUiLabels();

  if (dpsReady)
  {
    lv_label_set_text(labelStatus, "준비");
  }
  else
  {
    lv_label_set_text(labelStatus, "센서 초기화 실패");
  }

  applyRestoredMeasurementToUi();
  refreshHomeSensorLabels();

  lastAutoWifiRetryMs = millis();
  startWifiApiTask();

  Serial.println("Dashboard ready");
}

String getMeasureElapsedText()
{
  unsigned long elapsed = measureAccumulatedMs;

  if (measureClockRunning)
  {
    elapsed += millis() - measureResumeMs;
  }

  uint32_t sec = elapsed / 1000;
  uint32_t h = sec / 3600;
  uint32_t m = (sec % 3600) / 60;
  uint32_t s = sec % 60;

  char buf[32];
  snprintf(
    buf,
    sizeof(buf),
    "%02lu:%02lu:%02lu",
    (unsigned long)h,
    (unsigned long)m,
    (unsigned long)s
  );
  return String(buf);
}

int readBatteryPercent()
{
  // 아직 배터리 ADC 핀 또는 배터리 게이지 칩 정보가 없으므로 미지원
  // 나중에 배터리 전압 ADC 핀을 알면 여기서 계산하면 됨
  return -1;
}

void updateClockLabels()
{
  String currentText = "현재: " + getCurrentDateTimeText();

  if (labelHomeDateTime) lv_label_set_text(labelHomeDateTime, currentText.c_str());
  if (labelDateTime) lv_label_set_text(labelDateTime, currentText.c_str());

  String measureText = "시간 " + getMeasureElapsedText();

  if (labelRuntime) lv_label_set_text(labelRuntime, measureText.c_str());

  updateStatusBars();
  updateIslStatusLabels();
}



// =====================================================
// Loop
// =====================================================
void loop()
{
  unsigned long now = millis();

  static unsigned long lastLvglHandlerMs = 0;
  if (now - lastLvglHandlerMs >= LVGL_HANDLER_PERIOD_MS)
  {
    lastLvglHandlerMs = now;
    serviceLightweightBatchUi();
    uiTimerHandler();
    servicePendingScreenSwitch();
  }

  // Sensor initialization and WiFi commands run outside LVGL callbacks.
  servicePendingSensorMode();
  servicePendingWifiCommand();

  if (now - lastUiClockMs >= 1000)
  {
    lastUiClockMs = now;
    updateClockLabels();
    updateWifiRuntimeLabels();
    refreshMeasureControls();
    refreshBleScreen();

    if (pendingBleScan)
    {
      pendingBleScan = false;
      bleScanStart(6000);
    }

    // A scan that just finished is the one moment the result list is fresh.
    static bool bleScanWasRunning = false;
    const bool bleScanNow = bleScanIsRunning();

    if (bleScanWasRunning && !bleScanNow && bleAutoLinkToSensorNode())
    {
      refreshBleScreen();
    }

    bleScanWasRunning = bleScanNow;

    if (pendingBleDisconnect)
    {
      pendingBleDisconnect = false;
      bleLinkDisconnect();
      refreshBleScreen();
    }

    if (pendingBleConnectIndex >= 0)
    {
      const int index = pendingBleConnectIndex;
      pendingBleConnectIndex = -1;

      if (index < BLE_SCAN_MAX_RESULTS && bleRowAddress[index][0])
      {
        bleLinkConnect(bleRowAddress[index]);
        refreshBleScreen();
      }
    }

    if (pendingI2cScan)
    {
      pendingI2cScan = false;

      // The scan blocks for a few seconds, so say what is happening and get
      // it on the glass before starting.
      const char *busyText = "검색 중... 엔코더 바퀴를 계속 돌려 주세요";
      setIslStatusText(busyText);
      if (labelSettingsNote) lv_label_set_text(labelSettingsNote, busyText);
      lv_refr_now(NULL);

      char summary[160];
      const int found = scanSensorI2cBus(summary, sizeof(summary));

      // The encoder is not on the bus, so an address scan says nothing about
      // the one sensor whose wiring cannot be confirmed that way. Watch every
      // free pin instead and report whichever ones the wheel moves.
      char activePins[160];
      encoderFindActivePins(activePins, sizeof(activePins));

      // Both the scan and the sweep left the shared pins as bare inputs.
      dpsReady = activeSensorBegin();

      char line[420];
      snprintf(line, sizeof(line), "I2C %d개: %s\n엔코더 신호: %s", found, summary, activePins);
      setIslStatusText(line);

      if (labelSettingsNote) lv_label_set_text(labelSettingsNote, line);
    }
  }

  // WiFi 검색/자동 연결/API 전송은 백그라운드 task에서 처리한다.
  // 측정 루프에서는 네트워크 작업을 직접 실행하지 않는다.

  // V3 DEBUG: 버튼 요청이 task에서 5초 이상 회수되지 않으면 화면에 명확히 표시합니다.
  if (batchUploadRequested && (millis() - batchUploadRequestMs > 5000UL))
  {
    setIslStatusText("일괄전송 요청 대기 >5초: WiFi task 확인");
    requestBatchUiRefresh();
  }

  if (measuring && dpsReady)
  {
    if (now - lastReadMs >= readIntervalMs)
    {
      lastReadMs = now;

      float temperatureC = 0.0f;
      float pressureHpa = 0.0f;

      if (readActiveSensor(&temperatureC, &pressureHpa))
      {
        measurementCount++;

        // 기존 1초 센서는 측정 순번을 time_s로 사용해 왔습니다.
        // SCD41은 새 CO2 값이 약 5초마다 나오므로 1,2,3...으로 기록하면
        // 실제 시간축이 5배 압축됩니다. SCD41만 실제 누적 측정시간(s)을 사용합니다.
        uint32_t timeS = (uint32_t)measurementCount;
        if (activeSensorMode == SENSOR_MODE_SCD41)
        {
          unsigned long elapsedMs = measureAccumulatedMs;
          if (measureClockRunning) elapsedMs += millis() - measureResumeMs;
          timeS = (uint32_t)(elapsedMs / 1000UL);
          if (timeS == 0) timeS = 1;
        }

        if (labelStatus) lv_label_set_text(labelStatus, "상태: 측정 중");

        addSample(timeS, temperatureC, pressureHpa);
        appendCsv(timeS, temperatureC, pressureHpa);
        bleSensorPublish(timeS, activeSensorName(), temperatureC, activePrimaryUnit());
        saveMeasurementBackupToNvs(false);
        if (activeSensorSupportsDirectIsl())
        {
          queueCloudSample(measurementCount, timeS, temperatureC, pressureHpa);
        }

        char text[64];

        snprintf(text, sizeof(text), " %d", measurementCount);
        lv_label_set_text(labelCount, text);

        // labelSd is owned by refreshMeasureControls(); writing it here too
        // made the two settle on different wording each second and flicker.

        if (tablePageOffset == 0)
        {
          updateTable();
        }

        // 측정값은 매초 저장하되, 무거운 그래프 다시 그리기는 제한한다.
        // 측정 화면 밖에서는 dirty flag만 남기고 화면 진입 시 한 번에 갱신한다.
        chartUiDirty = true;
        if (lv_scr_act() == measureScreen &&
            (sampleCount <= 2 || now - lastChartUiRefreshMs >= CHART_UI_REFRESH_MS))
        {
          updateChartAutoScale();
          chartUiDirty = false;
          lastChartUiRefreshMs = now;
        }

        // 그래프 처리 뒤에 현재값을 다시 표시하여 placeholder로 덮이는 경로를 차단한다.
        refreshLatestMeasurementLabels();

        Serial.print(measurementCount);
        Serial.print(",");
        Serial.print(timeS);
        Serial.print(",");
        Serial.print(temperatureC, 2);
        Serial.print(",");
        if (activeSensorMode == SENSOR_MODE_SCD41)
        {
          Serial.print(pressureHpa, 2);
          Serial.print(",");
          Serial.println(scd41LastHumidityPct, 2);
        }
        else if (pressureValueValid(pressureHpa)) Serial.println(pressureHpa, 2);
        else Serial.println("-");
      }
      else
      {
        // The loop polls every second, but a sensor between conversions is
        // still measuring: SCD41 emits a sample about every 5 s, DS18B20
        // converts on demand, TMP117 and VL53L1X have their own settling.
        // Reporting that gap as a distinct state made the status read
        // "측정 대기" for four seconds out of every five on SCD41, which
        // looks exactly like a sensor that has stopped. Only a real fault
        // should move the status away from 측정 중.
        const bool waitingForNextSample =
          activeSensorMode == SENSOR_MODE_DS18B20 ||
          (activeSensorMode == SENSOR_MODE_TMP117 && tmp117LastReadWasWaiting) ||
          (activeSensorMode == SENSOR_MODE_VL53L1X && vl53LastReadWasWaiting) ||
          (activeSensorMode == SENSOR_MODE_SCD41 && scd41LastReadWasWaiting);

        if (waitingForNextSample)
        {
          if (labelStatus) lv_label_set_text(labelStatus, "상태: 측정 중");
        }
        else if (activeSensorMode == SENSOR_MODE_SCD41)
        {
          if (labelStatus) lv_label_set_text(labelStatus, "상태: SCD41 통신 재시도");

          if (scd41ConsecutiveErrors >= 4)
          {
            dpsReady = scd41Begin();
            if (labelStatus) lv_label_set_text(labelStatus, dpsReady ? "상태: SCD41 재연결 / 5초 대기" : "상태: SCD41 인식 실패");
          }
        }
        else
        {
          if (labelStatus) lv_label_set_text(labelStatus, "상태: 센서 읽기 실패");
        }
      }
    }
  }

  vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_IDLE_MS));
}