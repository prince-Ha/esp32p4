#include "ble_sensor.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// Bluetooth on the ESP32-P4 is only present when the ESP-Hosted link to the
// companion radio is compiled in. Guard the whole implementation so the
// firmware still builds on a target without it.
#if __has_include("host/ble_hs.h") && __has_include("nimble/nimble_port.h")
#define BLE_SENSOR_SUPPORTED 1
#else
#define BLE_SENSOR_SUPPORTED 0
#endif

#if BLE_SENSOR_SUPPORTED

#include "Arduino.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// CONFIG_LOG_DEFAULT_LEVEL is 1 (ERROR) in this build, so ESP_LOGI and
// ESP_LOGW are compiled out entirely. The rest of the firmware logs over
// Serial, so this does too.
static void blePrintf(const char *fmt, ...)
{
  char line[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  Serial.print("[BLE] ");
  Serial.println(line);
}

// Custom 128-bit UUIDs. Nothing standard describes "whatever this science
// probe is currently measuring", so the service is our own.
//   service        6e5f0001-8b9a-4c1d-9f2e-7a3b5c8d1e40
//   measurement    6e5f0002-8b9a-4c1d-9f2e-7a3b5c8d1e40  (read + notify)
static const ble_uuid128_t kServiceUuid =
  BLE_UUID128_INIT(0x40, 0x1e, 0x8d, 0x5c, 0x3b, 0x7a, 0x2e, 0x9f,
                   0x1d, 0x4c, 0x9a, 0x8b, 0x01, 0x00, 0x5f, 0x6e);

static const ble_uuid128_t kMeasurementUuid =
  BLE_UUID128_INIT(0x40, 0x1e, 0x8d, 0x5c, 0x3b, 0x7a, 0x2e, 0x9f,
                   0x1d, 0x4c, 0x9a, 0x8b, 0x02, 0x00, 0x5f, 0x6e);

static bool bleStarted = false;
static bool bleConnected = false;
static bool bleSubscribed = false;

static uint8_t bleAddrType = 0;
static uint16_t bleConnHandle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t bleMeasurementHandle = 0;

// Last formatted reading, served on read and sent on notify.
static char bleMeasurementText[96] = "";
static char bleDeviceName[32] = "ESP32-P4";

static void bleStartAdvertising();

static int bleMeasurementAccess(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
  (void)conn_handle;
  (void)attr_handle;
  (void)arg;

  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;

  const int rc = os_mbuf_append(ctxt->om, bleMeasurementText, strlen(bleMeasurementText));
  return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static const struct ble_gatt_chr_def kCharacteristics[] = {
  {
    .uuid = &kMeasurementUuid.u,
    .access_cb = bleMeasurementAccess,
    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
    .val_handle = &bleMeasurementHandle,
  },
  { 0 },
};

static const struct ble_gatt_svc_def kServices[] = {
  {
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = &kServiceUuid.u,
    .characteristics = kCharacteristics,
  },
  { 0 },
};

static int bleGapEvent(struct ble_gap_event *event, void *arg)
{
  (void)arg;

  switch (event->type)
  {
    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status == 0)
      {
        bleConnected = true;
        bleConnHandle = event->connect.conn_handle;
        blePrintf("central connected");
      }
      else
      {
        // Connection attempt failed; go back to being findable.
        bleStartAdvertising();
      }
      return 0;

    case BLE_GAP_EVENT_DISCONNECT:
      blePrintf("central disconnected (reason %d)", event->disconnect.reason);
      bleConnected = false;
      bleSubscribed = false;
      bleConnHandle = BLE_HS_CONN_HANDLE_NONE;
      bleStartAdvertising();
      return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
      bleStartAdvertising();
      return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
      if (event->subscribe.attr_handle == bleMeasurementHandle)
      {
        bleSubscribed = event->subscribe.cur_notify != 0;
        blePrintf("notifications %s", bleSubscribed ? "on" : "off");
      }
      return 0;

    default:
      return 0;
  }
}

static void bleStartAdvertising()
{
  struct ble_hs_adv_fields fields;
  memset(&fields, 0, sizeof(fields));

  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  fields.tx_pwr_lvl_is_present = 1;
  fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
  fields.name = (uint8_t *)bleDeviceName;
  fields.name_len = strlen(bleDeviceName);
  fields.name_is_complete = 1;

  int rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0)
  {
    blePrintf("ble_gap_adv_set_fields failed: %d", rc);
    return;
  }

  struct ble_gap_adv_params params;
  memset(&params, 0, sizeof(params));
  params.conn_mode = BLE_GAP_CONN_MODE_UND;
  params.disc_mode = BLE_GAP_DISC_MODE_GEN;

  rc = ble_gap_adv_start(bleAddrType, NULL, BLE_HS_FOREVER, &params, bleGapEvent, NULL);
  if (rc != 0) blePrintf("ble_gap_adv_start failed: %d", rc);
  else blePrintf("advertising started");
}

static void bleOnSync()
{
  int rc = ble_hs_util_ensure_addr(0);
  if (rc != 0)
  {
    blePrintf("ble_hs_util_ensure_addr failed: %d", rc);
    return;
  }

  rc = ble_hs_id_infer_auto(0, &bleAddrType);
  if (rc != 0)
  {
    blePrintf("ble_hs_id_infer_auto failed: %d", rc);
    return;
  }

  blePrintf("host synced, addr type %d", (int)bleAddrType);
  bleStartAdvertising();
}

static void bleOnReset(int reason)
{
  blePrintf("nimble reset, reason %d", reason);
  bleConnected = false;
  bleSubscribed = false;
}

static void bleHostTask(void *param)
{
  (void)param;
  nimble_port_run();               // returns only when the stack is stopped
  nimble_port_freertos_deinit();
}

bool bleSensorBegin(const char *deviceName)
{
  if (bleStarted) return true;

  if (deviceName != NULL && deviceName[0] != '\0')
  {
    snprintf(bleDeviceName, sizeof(bleDeviceName), "%s", deviceName);
  }

  esp_err_t err = nimble_port_init();
  if (err != ESP_OK)
  {
    blePrintf("nimble_port_init failed: %d", (int)err);
    return false;
  }

  ble_hs_cfg.sync_cb = bleOnSync;
  ble_hs_cfg.reset_cb = bleOnReset;

  ble_svc_gap_init();
  ble_svc_gatt_init();

  int rc = ble_gatts_count_cfg(kServices);
  if (rc != 0)
  {
    blePrintf("ble_gatts_count_cfg failed: %d", rc);
    return false;
  }

  rc = ble_gatts_add_svcs(kServices);
  if (rc != 0)
  {
    blePrintf("ble_gatts_add_svcs failed: %d", rc);
    return false;
  }

  rc = ble_svc_gap_device_name_set(bleDeviceName);
  if (rc != 0) blePrintf("device name not set: %d", rc);

  nimble_port_freertos_init(bleHostTask);

  bleStarted = true;
  blePrintf("advertising as %s", bleDeviceName);
  return true;
}

bool bleSensorIsConnected()
{
  return bleConnected;
}

bool bleSensorIsSubscribed()
{
  return bleSubscribed;
}

void bleSensorPublish(uint32_t timeS, const char *sensorName, float value, const char *unit)
{
  if (!bleStarted) return;

  snprintf(
    bleMeasurementText,
    sizeof(bleMeasurementText),
    "%lu,%s,%.4f,%s",
    (unsigned long)timeS,
    sensorName ? sensorName : "",
    value,
    unit ? unit : ""
  );

  if (!bleSubscribed || bleConnHandle == BLE_HS_CONN_HANDLE_NONE) return;

  struct os_mbuf *om = ble_hs_mbuf_from_flat(bleMeasurementText, strlen(bleMeasurementText));
  if (om == NULL) return;

  // Ownership of om passes to NimBLE whether or not this succeeds.
  ble_gattc_notify_custom(bleConnHandle, bleMeasurementHandle, om);
}

const char *bleSensorStateText()
{
  if (!bleStarted) return "꺼짐";
  if (bleSubscribed) return "전송 중";
  if (bleConnected) return "연결됨";
  return "대기 중";
}

#else  // BLE_SENSOR_SUPPORTED

bool bleSensorBegin(const char *deviceName) { (void)deviceName; return false; }
bool bleSensorIsConnected() { return false; }
bool bleSensorIsSubscribed() { return false; }
void bleSensorPublish(uint32_t timeS, const char *sensorName, float value, const char *unit)
{
  (void)timeS; (void)sensorName; (void)value; (void)unit;
}
const char *bleSensorStateText() { return "지원 안 함"; }

#endif  // BLE_SENSOR_SUPPORTED
