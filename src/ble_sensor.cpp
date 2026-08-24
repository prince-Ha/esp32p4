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

const char *bleAdvertisedName()
{
  return bleDeviceName;
}

const char *bleSensorStateText()
{
  if (!bleStarted) return "꺼짐";
  if (bleSubscribed) return "전송 중";
  if (bleConnected) return "연결됨";
  return "대기 중";
}


// =====================================================
// Central role: scanning
// =====================================================

void bleScanSortResults();

static BleScanResult bleScanResults[BLE_SCAN_MAX_RESULTS];
static int bleScanCount = 0;
static bool bleScanning = false;

static void bleFormatAddr(const ble_addr_t *addr, char *out, size_t outLen)
{
  const uint8_t *v = addr->val;   // NimBLE stores the address little-endian
  snprintf(out, outLen, "%02x:%02x:%02x:%02x:%02x:%02x",
           v[5], v[4], v[3], v[2], v[1], v[0]);
}

// Collects every advertised service UUID into one comma-separated string. This
// is the field that identifies an unknown sensor: a commercial probe exposing
// 0x181A is speaking standard Environmental Sensing, while a custom 128-bit
// UUID means its protocol has to come from the vendor.
static void bleFormatServices(const struct ble_hs_adv_fields *fields, char *out, size_t outLen)
{
  out[0] = '\0';
  size_t used = 0;

  for (int i = 0; i < fields->num_uuids16; i++)
  {
    const int n = snprintf(out + used, outLen - used, "%s0x%04x",
                           used ? "," : "", ble_uuid_u16(&fields->uuids16[i].u));
    if (n < 0 || (size_t)n >= outLen - used) return;
    used += n;
  }

  for (int i = 0; i < fields->num_uuids32; i++)
  {
    const int n = snprintf(out + used, outLen - used, "%s0x%08lx",
                           used ? "," : "", (unsigned long)fields->uuids32[i].value);
    if (n < 0 || (size_t)n >= outLen - used) return;
    used += n;
  }

  for (int i = 0; i < fields->num_uuids128; i++)
  {
    // Only the distinguishing half is worth showing at this width.
    const uint8_t *v = fields->uuids128[i].value;
    const int n = snprintf(out + used, outLen - used, "%s%02x%02x%02x%02x-…",
                           used ? "," : "", v[15], v[14], v[13], v[12]);
    if (n < 0 || (size_t)n >= outLen - used) return;
    used += n;
  }
}

static void bleRecordScanResult(const struct ble_gap_disc_desc *disc)
{
  struct ble_hs_adv_fields fields;
  if (ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data) != 0) return;

  char address[18];
  bleFormatAddr(&disc->addr, address, sizeof(address));

  // A device advertises repeatedly; keep one entry and refresh it, since a
  // later packet often carries the name when the first did not.
  int slot = -1;
  for (int i = 0; i < bleScanCount; i++)
  {
    if (strcmp(bleScanResults[i].address, address) == 0) { slot = i; break; }
  }

  if (slot < 0)
  {
    if (bleScanCount >= BLE_SCAN_MAX_RESULTS) return;
    slot = bleScanCount++;
    memset(&bleScanResults[slot], 0, sizeof(BleScanResult));
    snprintf(bleScanResults[slot].address, sizeof(bleScanResults[slot].address), "%s", address);
  }

  BleScanResult *r = &bleScanResults[slot];
  r->rssi = disc->rssi;

  if (fields.name != NULL && fields.name_len > 0)
  {
    const size_t len = fields.name_len < sizeof(r->name) - 1 ? fields.name_len : sizeof(r->name) - 1;
    memcpy(r->name, fields.name, len);
    r->name[len] = '\0';
  }

  if (fields.num_uuids16 || fields.num_uuids32 || fields.num_uuids128)
  {
    bleFormatServices(&fields, r->services, sizeof(r->services));
  }
}

static int bleScanEvent(struct ble_gap_event *event, void *arg)
{
  (void)arg;

  switch (event->type)
  {
    case BLE_GAP_EVENT_DISC:
      bleRecordScanResult(&event->disc);
      return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
      bleScanning = false;
      bleScanSortResults();
      blePrintf("scan complete, %d device(s), %d candidate(s)", bleScanCount, bleScanCandidateCount());
      bleScanDumpResults();
      return 0;

    default:
      return 0;
  }
}

bool bleScanStart(uint32_t durationMs)
{
  if (!bleStarted)
  {
    blePrintf("scan requested before the host was up");
    return false;
  }

  if (bleScanning) return true;

  bleScanCount = 0;

  struct ble_gap_disc_params params;
  memset(&params, 0, sizeof(params));
  params.passive = 0;      // active: ask for the scan response, which carries names
  params.filter_duplicates = 0;

  const int rc = ble_gap_disc(bleAddrType, durationMs, &params, bleScanEvent, NULL);
  if (rc != 0)
  {
    blePrintf("ble_gap_disc failed: %d", rc);
    return false;
  }

  bleScanning = true;
  blePrintf("scanning for %lu ms", (unsigned long)durationMs);
  return true;
}

bool bleScanIsRunning()
{
  return bleScanning;
}

int bleScanResultCount()
{
  return bleScanCount;
}

bool bleScanResultAt(int index, BleScanResult *out)
{
  if (out == NULL || index < 0 || index >= bleScanCount) return false;
  memcpy(out, &bleScanResults[index], sizeof(BleScanResult));
  return true;
}

bool bleScanResultIsCandidate(const BleScanResult *r)
{
  if (r == NULL) return false;
  return r->name[0] != '\0' || r->services[0] != '\0';
}

int bleScanCandidateCount()
{
  int n = 0;
  for (int i = 0; i < bleScanCount; i++)
  {
    if (bleScanResultIsCandidate(&bleScanResults[i])) n++;
  }
  return n;
}

// Named or service-bearing devices first, then by signal strength.
void bleScanSortResults()
{
  for (int i = 1; i < bleScanCount; i++)
  {
    BleScanResult key = bleScanResults[i];
    const bool keyCand = bleScanResultIsCandidate(&key);
    int j = i - 1;

    while (j >= 0)
    {
      const bool cand = bleScanResultIsCandidate(&bleScanResults[j]);
      const bool worse = (cand == keyCand) ? (bleScanResults[j].rssi < key.rssi) : (!cand && keyCand);
      if (!worse) break;

      bleScanResults[j + 1] = bleScanResults[j];
      j--;
    }

    bleScanResults[j + 1] = key;
  }
}

void bleScanDumpResults()
{
  Serial.println("[BLE] --- scan results ---");

  for (int i = 0; i < bleScanCount; i++)
  {
    const BleScanResult *r = &bleScanResults[i];
    Serial.printf(
      "[BLE] %2d  %-20s  %s  %4d dBm  services: %s\n",
      i,
      r->name[0] ? r->name : "(unnamed)",
      r->address,
      r->rssi,
      r->services[0] ? r->services : "(none advertised)"
    );
  }

  Serial.println("[BLE] --- end ---");
}

// =====================================================
// Central role: linking to one sensor node
//
// NimBLE drives this as a chain of callbacks, each starting the next step:
//   connect -> discover the service -> discover the characteristic
//           -> discover its CCCD -> write 0x0001 to it -> notifications arrive
//
// The CCCD write is the step that is easy to leave out. Without it the link
// looks healthy - connected, characteristic found - and no reading ever
// arrives, because a GATT server only notifies subscribers.
// =====================================================

static uint16_t bleLinkConnHandle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t bleLinkValueHandle = 0;
static uint16_t bleLinkCccdHandle = 0;
static uint16_t bleLinkSvcEndHandle = 0;
static bool bleLinkSubscribed = false;
static bool bleLinkBusy = false;
static char bleLinkPeer[32] = "";
static char bleLinkState[48] = "연결 안 됨";

// Last notification, kept unparsed until something asks for it.
static char bleLinkPayload[96] = "";
static uint32_t bleLinkPayloadMs = 0;
static bool bleLinkHasReading = false;
static uint32_t bleLinkNotifyCount = 0;

static void bleLinkSetState(const char *text)
{
  snprintf(bleLinkState, sizeof(bleLinkState), "%s", text);
  blePrintf("link: %s", text);
}

static void bleLinkReset(const char *why)
{
  const bool wasIdle = bleLinkConnHandle == BLE_HS_CONN_HANDLE_NONE && !bleLinkBusy;

  bleLinkConnHandle = BLE_HS_CONN_HANDLE_NONE;
  bleLinkValueHandle = 0;
  bleLinkCccdHandle = 0;
  bleLinkSvcEndHandle = 0;
  bleLinkSubscribed = false;
  bleLinkBusy = false;

  if (wasIdle) snprintf(bleLinkState, sizeof(bleLinkState), "%s", why);
  else bleLinkSetState(why);
}

// Step 4: the subscription write came back.
static int bleLinkOnSubscribed(uint16_t conn_handle,
                               const struct ble_gatt_error *error,
                               struct ble_gatt_attr *attr, void *arg)
{
  (void)conn_handle; (void)attr; (void)arg;

  if (error->status != 0)
  {
    blePrintf("CCCD write failed: %d", error->status);
    ble_gap_terminate(bleLinkConnHandle, BLE_ERR_REM_USER_CONN_TERM);
    bleLinkReset("구독 실패");
    return 0;
  }

  bleLinkSubscribed = true;
  bleLinkBusy = false;
  bleLinkSetState("수신 중");
  return 0;
}

// Step 3: found the descriptors on the characteristic; the CCCD is 0x2902.
static int bleLinkOnDescriptor(uint16_t conn_handle,
                               const struct ble_gatt_error *error,
                               uint16_t chr_val_handle,
                               const struct ble_gatt_dsc *dsc, void *arg)
{
  (void)conn_handle; (void)chr_val_handle; (void)arg;

  if (error->status == 0 && dsc != NULL &&
      ble_uuid_u16(&dsc->uuid.u) == BLE_GATT_DSC_CLT_CFG_UUID16)
  {
    bleLinkCccdHandle = dsc->handle;
    return 0;
  }

  if (error->status != BLE_HS_EDONE) return 0;

  if (bleLinkCccdHandle == 0)
  {
    blePrintf("no CCCD on the measurement characteristic");
    ble_gap_terminate(bleLinkConnHandle, BLE_ERR_REM_USER_CONN_TERM);
    bleLinkReset("알림 미지원 장치");
    return 0;
  }

  bleLinkSetState("구독 중");

  const uint8_t enableNotify[2] = { 0x01, 0x00 };
  const int rc = ble_gattc_write_flat(bleLinkConnHandle, bleLinkCccdHandle,
                                      enableNotify, sizeof(enableNotify),
                                      bleLinkOnSubscribed, NULL);
  if (rc != 0)
  {
    blePrintf("ble_gattc_write_flat failed: %d", rc);
    ble_gap_terminate(bleLinkConnHandle, BLE_ERR_REM_USER_CONN_TERM);
    bleLinkReset("구독 실패");
  }

  return 0;
}

// Step 2: found the measurement characteristic.
static int bleLinkOnCharacteristic(uint16_t conn_handle,
                                   const struct ble_gatt_error *error,
                                   const struct ble_gatt_chr *chr, void *arg)
{
  (void)conn_handle; (void)arg;

  if (error->status == 0 && chr != NULL)
  {
    bleLinkValueHandle = chr->val_handle;
    return 0;
  }

  if (error->status != BLE_HS_EDONE) return 0;

  if (bleLinkValueHandle == 0)
  {
    blePrintf("measurement characteristic not found");
    ble_gap_terminate(bleLinkConnHandle, BLE_ERR_REM_USER_CONN_TERM);
    bleLinkReset("측정 특성 없음");
    return 0;
  }

  const int rc = ble_gattc_disc_all_dscs(bleLinkConnHandle, bleLinkValueHandle,
                                         bleLinkSvcEndHandle,
                                         bleLinkOnDescriptor, NULL);
  if (rc != 0)
  {
    blePrintf("ble_gattc_disc_all_dscs failed: %d", rc);
    ble_gap_terminate(bleLinkConnHandle, BLE_ERR_REM_USER_CONN_TERM);
    bleLinkReset("구독 실패");
  }

  return 0;
}

static int bleLinkOnService(uint16_t conn_handle, const struct ble_gatt_error *error,
                            const struct ble_gatt_svc *service, void *arg);

// Step 0: agree an MTU.
//
// The default ATT MTU is 23, which leaves 20 bytes for a notification. A
// reading like "202,테스트,2.0000,-" is 23 bytes once the Korean is UTF-8
// encoded, so without this the unit field is silently cut off the end and the
// packet no longer parses. Do it before subscribing, not after, or the first
// readings arrive at the old size.
static int bleLinkOnMtu(uint16_t conn_handle, const struct ble_gatt_error *error,
                        uint16_t mtu, void *arg)
{
  (void)arg;

  if (error->status == 0) blePrintf("MTU is %u", (unsigned)mtu);
  else blePrintf("MTU exchange failed: %d, staying at 23", error->status);

  bleLinkSetState("서비스 검색 중");
  ble_gattc_disc_svc_by_uuid(conn_handle, &kServiceUuid.u, bleLinkOnService, NULL);
  return 0;
}

// Step 1: found the service.
static int bleLinkOnService(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            const struct ble_gatt_svc *service, void *arg)
{
  (void)arg;

  if (error->status == 0 && service != NULL)
  {
    bleLinkSvcEndHandle = service->end_handle;

    const int rc = ble_gattc_disc_chrs_by_uuid(
      conn_handle, service->start_handle, service->end_handle,
      &kMeasurementUuid.u, bleLinkOnCharacteristic, NULL
    );

    if (rc != 0)
    {
      blePrintf("ble_gattc_disc_chrs_by_uuid failed: %d", rc);
      ble_gap_terminate(bleLinkConnHandle, BLE_ERR_REM_USER_CONN_TERM);
      bleLinkReset("서비스 검색 실패");
    }

    return 0;
  }

  if (error->status != BLE_HS_EDONE) return 0;

  if (bleLinkSvcEndHandle == 0)
  {
    blePrintf("sensor service not found on this device");
    ble_gap_terminate(bleLinkConnHandle, BLE_ERR_REM_USER_CONN_TERM);
    bleLinkReset("센서 노드가 아님");
  }

  return 0;
}

static int bleLinkGapEvent(struct ble_gap_event *event, void *arg)
{
  (void)arg;

  switch (event->type)
  {
    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status != 0)
      {
        blePrintf("connect failed: %d", event->connect.status);
        bleLinkReset("연결 실패");
        return 0;
      }

      bleLinkConnHandle = event->connect.conn_handle;
      bleLinkSetState("MTU 협상 중");

      if (ble_gattc_exchange_mtu(bleLinkConnHandle, bleLinkOnMtu, NULL) != 0)
      {
        // Not fatal on its own; carry on at the default size.
        bleLinkSetState("서비스 검색 중");
        ble_gattc_disc_svc_by_uuid(bleLinkConnHandle, &kServiceUuid.u,
                                   bleLinkOnService, NULL);
      }

      return 0;

    case BLE_GAP_EVENT_DISCONNECT:
      blePrintf("link dropped, reason %d", event->disconnect.reason);
      bleLinkReset("연결 끕김");
      return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
    {
      if (event->notify_rx.attr_handle != bleLinkValueHandle) return 0;

      const uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
      const uint16_t copy = len < sizeof(bleLinkPayload) - 1
                            ? len : (uint16_t)(sizeof(bleLinkPayload) - 1);

      if (ble_hs_mbuf_to_flat(event->notify_rx.om, bleLinkPayload, copy, NULL) == 0)
      {
        bleLinkPayload[copy] = '\0';
        bleLinkPayloadMs = (uint32_t)(esp_timer_get_time() / 1000);

        // The first packet is the one that proves the subscription took; after
        // that a line every ten seconds is enough to show the node is alive
        // without burying everything else in the log.
        if (!bleLinkHasReading || (bleLinkNotifyCount % 10) == 0)
        {
          blePrintf("rx #%lu: %s", (unsigned long)bleLinkNotifyCount, bleLinkPayload);
        }

        bleLinkNotifyCount++;
        bleLinkHasReading = true;
      }

      return 0;
    }

    default:
      return 0;
  }
}

bool bleLinkConnect(const char *address)
{
  if (!bleStarted)
  {
    bleLinkSetState("블루투스 꺼짐");
    return false;
  }

  if (address == NULL || strlen(address) != 17)
  {
    bleLinkSetState("주소 형식 오류");
    return false;
  }

  // Scanning and connecting cannot share the radio.
  if (bleScanning)
  {
    ble_gap_disc_cancel();
    bleScanning = false;
  }

  bleLinkDisconnect();

  unsigned int b[6] = { 0 };
  if (sscanf(address, "%02x:%02x:%02x:%02x:%02x:%02x",
             &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6)
  {
    bleLinkSetState("주소 형식 오류");
    return false;
  }

  ble_addr_t peer;
  memset(&peer, 0, sizeof(peer));

  // bleFormatAddr() prints most significant byte first; NimBLE stores it the
  // other way round.
  for (int i = 0; i < 6; i++) peer.val[i] = (uint8_t)b[5 - i];

  snprintf(bleLinkPeer, sizeof(bleLinkPeer), "%s", address);

  // A node advertising a random static address has its two top bits set. Guess
  // from the address itself rather than trying one type and retrying, because
  // ble_gap_connect() accepts a wrong type and then simply times out.
  peer.type = ((peer.val[5] & 0xC0) == 0xC0) ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;

  for (int i = 0; i < bleScanCount; i++)
  {
    if (strcmp(bleScanResults[i].address, address) == 0 && bleScanResults[i].name[0])
    {
      snprintf(bleLinkPeer, sizeof(bleLinkPeer), "%s", bleScanResults[i].name);
      break;
    }
  }

  bleLinkBusy = true;
  bleLinkHasReading = false;
  bleLinkNotifyCount = 0;
  bleLinkSetState("연결 중");

  const int rc = ble_gap_connect(bleAddrType, &peer, 10000, NULL,
                                 bleLinkGapEvent, NULL);

  if (rc != 0)
  {
    blePrintf("ble_gap_connect failed: %d", rc);
    bleLinkReset("연결 실패");
    return false;
  }

  return true;
}

void bleLinkDisconnect()
{
  if (bleLinkConnHandle != BLE_HS_CONN_HANDLE_NONE)
  {
    ble_gap_terminate(bleLinkConnHandle, BLE_ERR_REM_USER_CONN_TERM);
  }

  bleLinkReset("연결 안 됨");
  bleLinkHasReading = false;
}

// bleFormatServices() prints a 128-bit UUID as its leading four bytes, which
// is enough to tell this project's nodes from anything else in the room.
#define BLE_SENSOR_SERVICE_TAG "6e5f0001"

bool bleAutoLinkToSensorNode()
{
  if (bleLinkSubscribed || bleLinkBusy) return false;

  int best = -1;

  for (int i = 0; i < bleScanCount; i++)
  {
    if (strstr(bleScanResults[i].services, BLE_SENSOR_SERVICE_TAG) == NULL) continue;
    if (best < 0 || bleScanResults[i].rssi > bleScanResults[best].rssi) best = i;
  }

  if (best < 0) return false;

  blePrintf("auto-linking to %s (%s, %d dBm)",
            bleScanResults[best].name[0] ? bleScanResults[best].name : "(unnamed)",
            bleScanResults[best].address, bleScanResults[best].rssi);

  return bleLinkConnect(bleScanResults[best].address);
}

bool bleLinkIsSubscribed() { return bleLinkSubscribed; }
bool bleLinkIsBusy() { return bleLinkBusy; }
const char *bleLinkPeerName() { return bleLinkPeer; }
const char *bleLinkStateText() { return bleLinkState; }

bool bleLinkLatestReading(float *value, char *sensorName, char *unit, uint32_t *ageMs)
{
  if (!bleLinkHasReading) return false;

  // "<time_s>,<sensor>,<value>,<unit>"
  char work[sizeof(bleLinkPayload)];
  snprintf(work, sizeof(work), "%s", bleLinkPayload);

  char *cursor = work;
  char *fields[4] = { NULL, NULL, NULL, NULL };
  int found = 0;

  fields[found++] = cursor;

  while (*cursor && found < 4)
  {
    if (*cursor == ',')
    {
      *cursor = '\0';
      fields[found++] = cursor + 1;
    }
    cursor++;
  }

  // The unit is optional: a node that omits it, or whose packet was clipped by
  // a small MTU, still gives a usable number.
  if (found < 3) return false;

  if (value) *value = strtof(fields[2], NULL);
  if (sensorName) snprintf(sensorName, 24, "%s", fields[1]);
  if (unit) snprintf(unit, 24, "%s", found >= 4 ? fields[3] : "");

  if (ageMs)
  {
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    *ageMs = now - bleLinkPayloadMs;
  }

  return true;
}

#else  // BLE_SENSOR_SUPPORTED

bool bleSensorBegin(const char *deviceName) { (void)deviceName; return false; }
bool bleSensorIsConnected() { return false; }
bool bleSensorIsSubscribed() { return false; }
void bleSensorPublish(uint32_t timeS, const char *sensorName, float value, const char *unit)
{
  (void)timeS; (void)sensorName; (void)value; (void)unit;
}
const char *bleAdvertisedName() { return "-"; }
const char *bleSensorStateText() { return "지원 안 함"; }
bool bleScanStart(uint32_t durationMs) { (void)durationMs; return false; }
bool bleScanIsRunning() { return false; }
int bleScanResultCount() { return 0; }
bool bleScanResultAt(int index, BleScanResult *out) { (void)index; (void)out; return false; }
void bleScanDumpResults() {}
bool bleScanResultIsCandidate(const BleScanResult *r) { (void)r; return false; }
int bleScanCandidateCount() { return 0; }
bool bleLinkConnect(const char *address) { (void)address; return false; }
void bleLinkDisconnect() {}
bool bleLinkIsSubscribed() { return false; }
bool bleAutoLinkToSensorNode() { return false; }
bool bleLinkIsBusy() { return false; }
const char *bleLinkPeerName() { return ""; }
const char *bleLinkStateText() { return "지원 안 함"; }
bool bleLinkLatestReading(float *value, char *sensorName, char *unit, uint32_t *ageMs)
{
  (void)value; (void)sensorName; (void)unit; (void)ageMs;
  return false;
}

#endif  // BLE_SENSOR_SUPPORTED
