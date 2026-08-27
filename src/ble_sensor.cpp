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
// Central role: linking to sensor nodes
//
// NimBLE drives each link as a chain of callbacks, one starting the next:
//   connect -> agree an MTU -> discover the service -> discover the
//   characteristic -> discover its CCCD -> write 0x0001 -> notifications
//
// Two of those steps are easy to leave out and both fail quietly. Without the
// CCCD write the link looks healthy - connected, characteristic found - and no
// reading ever arrives, because a GATT server only notifies subscribers. And
// without the MTU exchange every notification is capped at 20 bytes, which
// silently clips the end off a reading rather than failing.
//
// Each connection gets a slot. The slot index rides along as the callback
// `arg`, so a packet from one node cannot be mistaken for another's.
// =====================================================

typedef struct
{
  uint16_t connHandle;
  uint16_t valueHandle;
  uint16_t cccdHandle;
  uint16_t svcEndHandle;
  bool subscribed;
  bool busy;
  char peerName[32];
  char peerAddress[18];
  char state[48];

  char payload[96];
  uint32_t payloadMs;
  bool hasReading;
  uint32_t notifyCount;

  // Exploration: connected to learn what the device is, not to read it.
  bool exploring;
  int exploreServices;
  int exploreChrs;
  int exploreSubscribed;
  char exploreSummary[64];

  // Readable characteristics, walked one at a time after discovery. A sensor
  // that will not stream until it is told to may still hand over its current
  // measurement to a plain read.
  uint16_t exploreReadHandles[16];
  int exploreReadCount;
  int exploreReadIndex;

  // The characteristic that takes commands, and how far through the candidate
  // list this link has got.
  // Every characteristic that both takes writes and notifies. Both PASCO
  // services have one, and the live one - the channel already streaming status
  // - is as likely to be the way in as the silent one.
  uint16_t exploreCommandHandles[4];
  int exploreCommandCount;
  int exploreProbeIndex;
  unsigned long exploreNextProbeMs;

  // PASCO: handles for 4a5c000<service>-000<char>-…, indexed [service][char].
  // Service 0 is the device, 1 the sensor; characteristic 2 takes commands and
  // 3 carries the replies.
  uint16_t pascoHandle[2][6];
  bool pascoPresent;
  int pascoStep;
  unsigned long pascoNextStepMs;
  uint16_t pascoSensorId;
  uint8_t pascoSampleBytes;
} BleLink;

static BleLink bleLinks[BLE_LINK_MAX_NODES];
static bool bleLinksInitialised = false;

static void bleLinkInitAll()
{
  if (bleLinksInitialised) return;

  for (int i = 0; i < BLE_LINK_MAX_NODES; i++)
  {
    memset(&bleLinks[i], 0, sizeof(bleLinks[i]));
    bleLinks[i].connHandle = BLE_HS_CONN_HANDLE_NONE;
    snprintf(bleLinks[i].state, sizeof(bleLinks[i].state), "%s", "연결 안 됨");
  }

  bleLinksInitialised = true;
}

static bool bleSlotValid(int slot)
{
  return slot >= 0 && slot < BLE_LINK_MAX_NODES;
}

static void bleLinkSetState(int slot, const char *text)
{
  if (!bleSlotValid(slot)) return;

  snprintf(bleLinks[slot].state, sizeof(bleLinks[slot].state), "%s", text);
  blePrintf("link %d (%s): %s", slot,
            bleLinks[slot].peerName[0] ? bleLinks[slot].peerName : "-", text);
}

static void bleLinkClear(int slot, const char *why, bool keepName)
{
  if (!bleSlotValid(slot)) return;

  BleLink *link = &bleLinks[slot];
  const bool wasIdle = link->connHandle == BLE_HS_CONN_HANDLE_NONE && !link->busy;

  link->connHandle = BLE_HS_CONN_HANDLE_NONE;
  link->valueHandle = 0;
  link->cccdHandle = 0;
  link->svcEndHandle = 0;
  link->subscribed = false;
  link->busy = false;
  link->hasReading = false;
  link->exploring = false;

  if (!keepName)
  {
    link->peerName[0] = '\0';
    link->peerAddress[0] = '\0';
  }

  // Announcing a state that was already the state only clutters the log.
  if (wasIdle) snprintf(link->state, sizeof(link->state), "%s", why);
  else bleLinkSetState(slot, why);
}

static int bleLinkSlotForConn(uint16_t connHandle)
{
  for (int i = 0; i < BLE_LINK_MAX_NODES; i++)
  {
    if (bleLinks[i].connHandle == connHandle) return i;
  }

  return -1;
}

int bleLinkSlotForAddress(const char *address)
{
  if (address == NULL) return -1;

  for (int i = 0; i < BLE_LINK_MAX_NODES; i++)
  {
    if (strcmp(bleLinks[i].peerAddress, address) == 0) return i;
  }

  return -1;
}

static int bleLinkFreeSlot()
{
  for (int i = 0; i < BLE_LINK_MAX_NODES; i++)
  {
    if (bleLinks[i].connHandle == BLE_HS_CONN_HANDLE_NONE && !bleLinks[i].busy) return i;
  }

  return -1;
}

static int bleLinkOnService(uint16_t conn_handle, const struct ble_gatt_error *error,
                            const struct ble_gatt_svc *service, void *arg);
static int bleExploreOnSvc(uint16_t conn_handle, const struct ble_gatt_error *error,
                           const struct ble_gatt_svc *service, void *arg);
static void bleExploreDump(const char *what, uint16_t handle, const struct os_mbuf *om, int slot);
static void blePascoHandleNotification(int slot, const uint8_t *data, uint16_t len);
static void blePascoNoteCharacteristic(int slot, const struct ble_gatt_chr *chr);
static void blePascoService(int slot);

// Step 4: the subscription write came back.
static int bleLinkOnSubscribed(uint16_t conn_handle, const struct ble_gatt_error *error,
                               struct ble_gatt_attr *attr, void *arg)
{
  (void)attr;
  const int slot = (int)(intptr_t)arg;
  if (!bleSlotValid(slot)) return 0;

  if (error->status != 0)
  {
    blePrintf("CCCD write failed on slot %d: %d", slot, error->status);
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    bleLinkClear(slot, "구독 실패", true);
    return 0;
  }

  bleLinks[slot].subscribed = true;
  bleLinks[slot].busy = false;
  bleLinkSetState(slot, "수신 중");
  return 0;
}

// Step 3: found the descriptors; the CCCD is 0x2902.
static int bleLinkOnDescriptor(uint16_t conn_handle, const struct ble_gatt_error *error,
                               uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
  (void)chr_val_handle;
  const int slot = (int)(intptr_t)arg;
  if (!bleSlotValid(slot)) return 0;

  if (error->status == 0 && dsc != NULL &&
      ble_uuid_u16(&dsc->uuid.u) == BLE_GATT_DSC_CLT_CFG_UUID16)
  {
    bleLinks[slot].cccdHandle = dsc->handle;
    return 0;
  }

  if (error->status != BLE_HS_EDONE) return 0;

  if (bleLinks[slot].cccdHandle == 0)
  {
    blePrintf("slot %d: no CCCD on the measurement characteristic", slot);
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    bleLinkClear(slot, "알림 미지원 장치", true);
    return 0;
  }

  bleLinkSetState(slot, "구독 중");

  const uint8_t enableNotify[2] = { 0x01, 0x00 };
  const int rc = ble_gattc_write_flat(conn_handle, bleLinks[slot].cccdHandle,
                                      enableNotify, sizeof(enableNotify),
                                      bleLinkOnSubscribed, arg);
  if (rc != 0)
  {
    blePrintf("slot %d: ble_gattc_write_flat failed: %d", slot, rc);
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    bleLinkClear(slot, "구독 실패", true);
  }

  return 0;
}

// Step 2: found the measurement characteristic.
static int bleLinkOnCharacteristic(uint16_t conn_handle, const struct ble_gatt_error *error,
                                   const struct ble_gatt_chr *chr, void *arg)
{
  const int slot = (int)(intptr_t)arg;
  if (!bleSlotValid(slot)) return 0;

  if (error->status == 0 && chr != NULL)
  {
    bleLinks[slot].valueHandle = chr->val_handle;
    return 0;
  }

  if (error->status != BLE_HS_EDONE) return 0;

  if (bleLinks[slot].valueHandle == 0)
  {
    blePrintf("slot %d: measurement characteristic not found", slot);
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    bleLinkClear(slot, "측정 특성 없음", true);
    return 0;
  }

  const int rc = ble_gattc_disc_all_dscs(conn_handle, bleLinks[slot].valueHandle,
                                         bleLinks[slot].svcEndHandle,
                                         bleLinkOnDescriptor, arg);
  if (rc != 0)
  {
    blePrintf("slot %d: ble_gattc_disc_all_dscs failed: %d", slot, rc);
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    bleLinkClear(slot, "구독 실패", true);
  }

  return 0;
}

// Step 1: found the service.
static int bleLinkOnService(uint16_t conn_handle, const struct ble_gatt_error *error,
                            const struct ble_gatt_svc *service, void *arg)
{
  const int slot = (int)(intptr_t)arg;
  if (!bleSlotValid(slot)) return 0;

  if (error->status == 0 && service != NULL)
  {
    bleLinks[slot].svcEndHandle = service->end_handle;

    const int rc = ble_gattc_disc_chrs_by_uuid(conn_handle, service->start_handle,
                                               service->end_handle, &kMeasurementUuid.u,
                                               bleLinkOnCharacteristic, arg);
    if (rc != 0)
    {
      blePrintf("slot %d: ble_gattc_disc_chrs_by_uuid failed: %d", slot, rc);
      ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
      bleLinkClear(slot, "서비스 검색 실패", true);
    }

    return 0;
  }

  if (error->status != BLE_HS_EDONE) return 0;

  if (bleLinks[slot].svcEndHandle == 0)
  {
    blePrintf("slot %d: sensor service not found on this device", slot);
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    bleLinkClear(slot, "센서 노드가 아님", true);
  }

  return 0;
}

// Step 0: agree an MTU.
//
// The default ATT MTU is 23, which leaves 20 bytes for a notification. A
// reading carrying two quantities with Korean names is well past that, and
// without this the tail is cut off silently. Do it before subscribing, not
// after, or the first readings still arrive at the old size.
static int bleLinkOnMtu(uint16_t conn_handle, const struct ble_gatt_error *error,
                        uint16_t mtu, void *arg)
{
  const int slot = (int)(intptr_t)arg;
  if (!bleSlotValid(slot)) return 0;

  if (error->status == 0) blePrintf("slot %d: MTU is %u", slot, (unsigned)mtu);
  else blePrintf("slot %d: MTU exchange failed: %d, staying at 23", slot, error->status);

  if (bleLinks[slot].exploring)
  {
    bleLinkSetState(slot, "서비스 검색 중");
    ble_gattc_disc_all_svcs(conn_handle, bleExploreOnSvc, arg);
    return 0;
  }

  bleLinkSetState(slot, "서비스 검색 중");
  ble_gattc_disc_svc_by_uuid(conn_handle, &kServiceUuid.u, bleLinkOnService, arg);
  return 0;
}

static int bleLinkGapEvent(struct ble_gap_event *event, void *arg)
{
  const int slot = (int)(intptr_t)arg;
  if (!bleSlotValid(slot)) return 0;

  switch (event->type)
  {
    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status != 0)
      {
        blePrintf("slot %d: connect failed: %d", slot, event->connect.status);
        bleLinkClear(slot, "연결 실패", true);
        return 0;
      }

      bleLinks[slot].connHandle = event->connect.conn_handle;
      bleLinkSetState(slot, "MTU 협상 중");

      if (ble_gattc_exchange_mtu(event->connect.conn_handle, bleLinkOnMtu, arg) != 0)
      {
        // Not fatal on its own; carry on at the default size.
        bleLinkSetState(slot, "서비스 검색 중");

        if (bleLinks[slot].exploring)
        {
          ble_gattc_disc_all_svcs(event->connect.conn_handle, bleExploreOnSvc, arg);
        }
        else
        {
          ble_gattc_disc_svc_by_uuid(event->connect.conn_handle, &kServiceUuid.u,
                                     bleLinkOnService, arg);
        }
      }

      return 0;

    case BLE_GAP_EVENT_DISCONNECT:
      blePrintf("slot %d: link dropped, reason %d", slot, event->disconnect.reason);
      bleLinkClear(slot, "연결 끊김", true);
      return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
    {
      if (bleLinks[slot].exploring)
      {
        // Print it as bytes. Which of them carry the reading, and in what
        // order and scale, is what these dumps are for.
        bleExploreDump("notify", event->notify_rx.attr_handle, event->notify_rx.om, slot);
        bleLinks[slot].notifyCount++;

        if (bleLinks[slot].pascoPresent)
        {
          uint8_t raw[64];
          const uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
          const uint16_t copy = len < sizeof(raw) ? len : (uint16_t)sizeof(raw);

          if (ble_hs_mbuf_to_flat(event->notify_rx.om, raw, copy, NULL) == 0)
          {
            blePascoHandleNotification(slot, raw, copy);
          }
        }

        return 0;
      }

      if (event->notify_rx.attr_handle != bleLinks[slot].valueHandle) return 0;

      BleLink *link = &bleLinks[slot];
      const uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
      const uint16_t copy = len < sizeof(link->payload) - 1
                            ? len : (uint16_t)(sizeof(link->payload) - 1);

      if (ble_hs_mbuf_to_flat(event->notify_rx.om, link->payload, copy, NULL) == 0)
      {
        link->payload[copy] = '\0';
        link->payloadMs = (uint32_t)(esp_timer_get_time() / 1000);

        // The first packet proves the subscription took; after that one line
        // every ten seconds shows the node is alive without burying the log.
        if (!link->hasReading || (link->notifyCount % 10) == 0)
        {
          blePrintf("slot %d rx #%lu: %s", slot,
                    (unsigned long)link->notifyCount, link->payload);
        }

        link->notifyCount++;
        link->hasReading = true;
      }

      return 0;
    }

    default:
      return 0;
  }
}

// ---- exploring an unknown device ------------------------------------------
//
// Discovery runs across the whole attribute range in one pass per kind, rather
// than walking service by service. It is cruder than the targeted path used
// for this project's own nodes, and it is the right shape here: nothing is
// known yet, so everything is worth seeing.

static void bleExploreSetSummary(int slot)
{
  if (!bleSlotValid(slot)) return;

  snprintf(bleLinks[slot].exploreSummary, sizeof(bleLinks[slot].exploreSummary),
           "서비스 %d · 특성 %d · 구독 %d",
           bleLinks[slot].exploreServices,
           bleLinks[slot].exploreChrs,
           bleLinks[slot].exploreSubscribed);
}

static void bleExploreReadNext(uint16_t conn_handle, int slot);

// =====================================================
// PASCO wireless sensors
//
// Their protocol is not guessable and did not need to be guessed: PASCO's own
// BLE examples and a working ESP32 client for the AirLink both spell it out.
//
//   UUIDs   4a5c000<service>-000<char>-0000-0000-5c1e741f1c00
//   service 0 = the device itself, 1 = the attached sensor
//   char    2 = commands, written without response
//           3 = replies and events, subscribed to
//
//   0x08            ask what sensor this is
//   0x05, <bytes>   ask for one sample of that many bytes
//
//   replies start 0xC0: status, echoed command, then the payload
//   events start 0x82 (sensor id) or 0x85 (device status, which is the battery
//   stream that was arriving before any of this was understood)
//
// Values are 16.16 fixed point: a little-endian int32 over 65536.
// =====================================================

#define PASCO_CMD_READ_ONE_SAMPLE 0x05
#define PASCO_CMD_GET_SENSOR_ID   0x08
#define PASCO_RSP_RESULT          0xC0
#define PASCO_EVT_SENSOR_ID       0x82

// Records a characteristic if it belongs to PASCO's UUID family.
static void blePascoNoteCharacteristic(int slot, const struct ble_gatt_chr *chr)
{
  if (!bleSlotValid(slot) || chr == NULL) return;
  if (chr->uuid.u.type != BLE_UUID_TYPE_128) return;

  // NimBLE stores a 128-bit UUID least significant byte first, so the leading
  // "4a5c" of the printed form sits at the top of the array.
  const uint8_t *v = chr->uuid.u128.value;
  if (v[15] != 0x4a || v[14] != 0x5c) return;

  const uint8_t serviceId = v[12];
  const uint8_t charId = v[10];

  if (serviceId > 1 || charId > 5) return;

  bleLinks[slot].pascoHandle[serviceId][charId] = chr->val_handle;
  bleLinks[slot].pascoPresent = true;
}

static void blePascoSend(int slot, uint8_t serviceId, const uint8_t *bytes, uint8_t len)
{
  BleLink *link = &bleLinks[slot];
  const uint16_t handle = link->pascoHandle[serviceId][2];
  if (handle == 0) return;

  char hex[16] = "";
  int used = 0;
  for (uint8_t i = 0; i < len && used < (int)sizeof(hex) - 3; i++)
  {
    used += snprintf(hex + used, sizeof(hex) - used, "%02X ", bytes[i]);
  }

  blePrintf("pasco: -> service %u handle %u: %s", serviceId, (unsigned)handle, hex);

  // Written without a response: that is what the command characteristic
  // offers, and asking for one is refused.
  ble_gattc_write_no_rsp_flat(link->connHandle, handle, bytes, len);
}

// Unpacks PASCO's 16.16 fixed point.
static float blePascoValue(const uint8_t *payload, int offset)
{
  const uint32_t raw =
    (uint32_t)payload[offset] |
    ((uint32_t)payload[offset + 1] << 8) |
    ((uint32_t)payload[offset + 2] << 16) |
    ((uint32_t)payload[offset + 3] << 24);

  return (float)(int32_t)raw / 65536.0f;
}

static void blePascoHandleNotification(int slot, const uint8_t *data, uint16_t len)
{
  if (len < 1) return;

  BleLink *link = &bleLinks[slot];
  const uint8_t header = data[0];

  if (header == PASCO_EVT_SENSOR_ID && len >= 3)
  {
    link->pascoSensorId = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
    blePrintf("pasco: sensor id 0x%04X", link->pascoSensorId);
    return;
  }

  if (header != PASCO_RSP_RESULT || len < 3) return;

  const uint8_t status = data[1];
  const uint8_t command = data[2];

  if (status != 0x00)
  {
    blePrintf("pasco: command 0x%02X refused, status 0x%02X", command, status);
    return;
  }

  if (command != PASCO_CMD_READ_ONE_SAMPLE || len < 7) return;

  // Everything after the header is measurement, four bytes per value.
  const int payloadLen = len - 3;
  const int values = payloadLen / 4;

  char line[192];
  int used = snprintf(line, sizeof(line), "pasco: ask %u -> %d byte(s) [",
                      (unsigned)link->pascoSampleBytes, payloadLen);

  for (int i = 0; i < payloadLen && used < (int)sizeof(line) - 4; i++)
  {
    used += snprintf(line + used, sizeof(line) - used, "%02X ", data[3 + i]);
  }

  used += snprintf(line + used, sizeof(line) - used, "] =");

  for (int i = 0; i < values && used < (int)sizeof(line) - 16; i++)
  {
    used += snprintf(line + used, sizeof(line) - used, " %.4f",
                     blePascoValue(data + 3, i * 4));
  }

  blePrintf("%s", line);

  if (values > 0)
  {
    // Both readings of the same four bytes: 16.16 as the reference library
    // decodes it, and the low half on its own. One of them is the pressure and
    // a single known value from the sensor's own app decides which.
    const uint16_t low = (uint16_t)data[3] | ((uint16_t)data[4] << 8);

    blePrintf("pasco: field0 fixed=%.4f low16=%u", blePascoValue(data + 3, 0), (unsigned)low);

    snprintf(link->exploreSummary, sizeof(link->exploreSummary), "%.4f / %u",
             blePascoValue(data + 3, 0), (unsigned)low);
  }
}

// Walks the opening exchange: wake the device, ask what it is, then sample.
static void blePascoService(int slot)
{
  BleLink *link = &bleLinks[slot];

  if (!link->pascoPresent) return;
  if (link->connHandle == BLE_HS_CONN_HANDLE_NONE) return;
  if ((long)(millis() - link->pascoNextStepMs) < 0) return;

  const uint8_t getId = PASCO_CMD_GET_SENSOR_ID;

  switch (link->pascoStep)
  {
    case 0:
      blePascoSend(slot, 0, &getId, 1);   // device service: bring it up
      link->pascoNextStepMs = millis() + 600;
      link->pascoStep = 1;
      break;

    case 1:
      blePascoSend(slot, 1, &getId, 1);   // sensor service: what is attached
      link->pascoNextStepMs = millis() + 600;
      link->pascoStep = 2;
      bleLinkSetState(slot, "PASCO 측정 요청 중");
      break;

    default:
    {
      // Sixteen is everything this sensor has: asking for twenty or
      // twenty-four returns sixteen just the same. Four fields, and only the
      // first moved when the pressure did - the other three sat at
      // 8192.2549, 8192.2090 and 4098.8579 through the whole run, which makes
      // them configuration rather than measurement.
      link->pascoSampleBytes = 16;

      const uint8_t sample[2] = { PASCO_CMD_READ_ONE_SAMPLE, link->pascoSampleBytes };
      blePascoSend(slot, 1, sample, 2);
      link->pascoNextStepMs = millis() + 1000;
      break;
    }
  }
}

// A write that is refused says why, and the reason narrows the search: a
// length complaint fixes the command length, "not permitted" rules the
// characteristic out entirely.
static int bleExploreOnWrite(uint16_t conn_handle, const struct ble_gatt_error *error,
                             struct ble_gatt_attr *attr, void *arg)
{
  (void)conn_handle; (void)arg;

  blePrintf("explore: write to handle %u -> status %d",
            attr ? (unsigned)attr->handle : 0, error->status);
  return 0;
}

// Openers to try on a sensor that stays silent once connected.
//
// Deliberately empty. Thirty-eight candidates were written to this sensor's
// two command characteristics - request-shaped bytes first, then the whole
// 0x8_ opcode range its own status frames use - and every single one was
// accepted with status 0 and then ignored. Writes are permitted; the framing
// is simply not one that has been guessed yet.
//
// Leaving a list here would mean the board writes unknown bytes into whatever
// a student happens to tap on the 블루투스 tab, which is not a thing to ship
// to a classroom for the sake of a search that was not converging. The
// exploration that pays - services, characteristics, reads and notification
// dumps - is all observation and stays.
//
// Fill this in when the command frame is known, from the PASCO driver that
// already works elsewhere in this project.
typedef struct
{
  const char *note;
  uint8_t bytes[4];
  uint8_t len;
} BleProbeCommand;

static const BleProbeCommand kBleProbeCommands[] = {
  { NULL, { 0 }, 0 }   // placeholder: the sweep is skipped while len is 0
};

// Turns an attribute's payload into hex for the log. What the bytes mean is
// exactly the question these dumps exist to answer.
static void bleExploreDump(const char *what, uint16_t handle, const struct os_mbuf *om, int slot)
{
  uint8_t raw[64];
  const uint16_t len = OS_MBUF_PKTLEN(om);
  const uint16_t copy = len < sizeof(raw) ? len : (uint16_t)sizeof(raw);

  if (ble_hs_mbuf_to_flat(om, raw, copy, NULL) != 0) return;

  char hex[3 * sizeof(raw) + 1];
  int used = 0;
  for (uint16_t i = 0; i < copy && used < (int)sizeof(hex) - 3; i++)
  {
    used += snprintf(hex + used, sizeof(hex) - used, "%02X ", raw[i]);
  }

  blePrintf("explore: %s handle %u (%u bytes) %s", what, (unsigned)handle,
            (unsigned)len, hex);

  if (bleSlotValid(slot))
  {
    snprintf(bleLinks[slot].exploreSummary, sizeof(bleLinks[slot].exploreSummary),
             "h%u %ubyte %s", (unsigned)handle, (unsigned)len, hex);
  }
}

static int bleExploreOnRead(uint16_t conn_handle, const struct ble_gatt_error *error,
                            struct ble_gatt_attr *attr, void *arg)
{
  const int slot = (int)(intptr_t)arg;
  if (!bleSlotValid(slot)) return 0;

  if (error->status == 0 && attr != NULL && attr->om != NULL)
  {
    bleExploreDump("read", attr->handle, attr->om, slot);
  }
  else if (error->status != 0)
  {
    blePrintf("explore: read of handle %u failed: %d",
              (unsigned)bleLinks[slot].exploreReadHandles[bleLinks[slot].exploreReadIndex],
              error->status);
  }

  bleLinks[slot].exploreReadIndex++;
  bleExploreReadNext(conn_handle, slot);
  return 0;
}

static void bleExploreReadNext(uint16_t conn_handle, int slot)
{
  if (!bleSlotValid(slot)) return;

  BleLink *link = &bleLinks[slot];

  if (link->exploreReadIndex >= link->exploreReadCount)
  {
    link->busy = false;

    if (link->pascoPresent)
    {
      link->pascoStep = 0;
      link->pascoNextStepMs = millis();
      link->pascoSampleBytes = 4;
      bleLinkSetState(slot, "PASCO 준비 중");
    }
    else if (link->exploreCommandCount > 0)
    {
      link->exploreProbeIndex = 0;
      link->exploreNextProbeMs = millis();
      bleLinkSetState(slot, "측정 시작 명령 시도 중");
    }
    else
    {
      bleLinkSetState(slot, "분석 완료 · 데이터 대기");
    }

    return;
  }

  const uint16_t handle = link->exploreReadHandles[link->exploreReadIndex];

  if (ble_gattc_read(conn_handle, handle, bleExploreOnRead, (void *)(intptr_t)slot) != 0)
  {
    link->exploreReadIndex++;
    bleExploreReadNext(conn_handle, slot);
  }
}

static int bleExploreOnDsc(uint16_t conn_handle, const struct ble_gatt_error *error,
                           uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
  (void)chr_val_handle;
  const int slot = (int)(intptr_t)arg;
  if (!bleSlotValid(slot)) return 0;

  if (error->status == 0 && dsc != NULL &&
      ble_uuid_u16(&dsc->uuid.u) == BLE_GATT_DSC_CLT_CFG_UUID16)
  {
    // Subscribe to everything that can notify. Which characteristic carries
    // the reading is exactly what is not known yet.
    const uint8_t enableNotify[2] = { 0x01, 0x00 };
    if (ble_gattc_write_flat(conn_handle, dsc->handle, enableNotify,
                             sizeof(enableNotify), NULL, NULL) == 0)
    {
      bleLinks[slot].exploreSubscribed++;
      blePrintf("explore: subscribed via CCCD handle %u", (unsigned)dsc->handle);
    }

    return 0;
  }

  if (error->status != BLE_HS_EDONE) return 0;

  bleExploreSetSummary(slot);
  blePrintf("explore: done - %d service(s), %d characteristic(s), %d subscription(s)",
            bleLinks[slot].exploreServices, bleLinks[slot].exploreChrs,
            bleLinks[slot].exploreSubscribed);

  // Nothing may ever be notified: a sensor that waits for a start command
  // stays silent. Read whatever is readable, which on some parts hands over
  // the current measurement anyway.
  bleLinkSetState(slot, "특성 읽는 중");
  bleLinks[slot].exploreReadIndex = 0;
  bleExploreReadNext(conn_handle, slot);
  return 0;
}

static int bleExploreOnChr(uint16_t conn_handle, const struct ble_gatt_error *error,
                           const struct ble_gatt_chr *chr, void *arg)
{
  const int slot = (int)(intptr_t)arg;
  if (!bleSlotValid(slot)) return 0;

  if (error->status == 0 && chr != NULL)
  {
    char uuid[BLE_UUID_STR_LEN];
    ble_uuid_to_str(&chr->uuid.u, uuid);

    // The property bits say which characteristic is worth listening to.
    // write-without-response was missing from this list, and it is the one
    // PASCO uses for commands - so the two characteristics that matter printed
    // with an empty property list and were passed over.
    blePrintf("explore: chr %s handle=%u props=%s%s%s%s%s",
              uuid, (unsigned)chr->val_handle,
              (chr->properties & BLE_GATT_CHR_PROP_READ) ? "read " : "",
              (chr->properties & BLE_GATT_CHR_PROP_WRITE) ? "write " : "",
              (chr->properties & BLE_GATT_CHR_PROP_WRITE_NO_RSP) ? "write-nr " : "",
              (chr->properties & BLE_GATT_CHR_PROP_NOTIFY) ? "notify " : "",
              (chr->properties & BLE_GATT_CHR_PROP_INDICATE) ? "indicate" : "");

    blePascoNoteCharacteristic(slot, chr);

    bleLinks[slot].exploreChrs++;

    // The measurement service's writable characteristic is where a request
    // would go. Handle 44 on the pressure sensor; found by shape, not by
    // number, so another PASCO part lands on its own.
    if ((chr->properties & BLE_GATT_CHR_PROP_WRITE) &&
        (chr->properties & BLE_GATT_CHR_PROP_NOTIFY) &&
        bleLinks[slot].exploreCommandCount < (int)(sizeof(bleLinks[slot].exploreCommandHandles) /
                                                   sizeof(bleLinks[slot].exploreCommandHandles[0])))
    {
      bleLinks[slot].exploreCommandHandles[bleLinks[slot].exploreCommandCount++] = chr->val_handle;
    }

    if ((chr->properties & BLE_GATT_CHR_PROP_READ) &&
        bleLinks[slot].exploreReadCount < (int)(sizeof(bleLinks[slot].exploreReadHandles) /
                                                sizeof(bleLinks[slot].exploreReadHandles[0])))
    {
      bleLinks[slot].exploreReadHandles[bleLinks[slot].exploreReadCount++] = chr->val_handle;
    }

    bleExploreSetSummary(slot);
    return 0;
  }

  if (error->status != BLE_HS_EDONE) return 0;

  bleLinkSetState(slot, "구독 중");
  ble_gattc_disc_all_dscs(conn_handle, 1, 0xFFFF, bleExploreOnDsc, arg);
  return 0;
}

static int bleExploreOnSvc(uint16_t conn_handle, const struct ble_gatt_error *error,
                           const struct ble_gatt_svc *service, void *arg)
{
  const int slot = (int)(intptr_t)arg;
  if (!bleSlotValid(slot)) return 0;

  if (error->status == 0 && service != NULL)
  {
    char uuid[BLE_UUID_STR_LEN];
    ble_uuid_to_str(&service->uuid.u, uuid);
    blePrintf("explore: service %s handles %u-%u", uuid,
              (unsigned)service->start_handle, (unsigned)service->end_handle);

    bleLinks[slot].exploreServices++;
    bleExploreSetSummary(slot);
    return 0;
  }

  if (error->status != BLE_HS_EDONE) return 0;

  bleLinkSetState(slot, "특성 검색 중");
  ble_gattc_disc_all_chrs(conn_handle, 1, 0xFFFF, bleExploreOnChr, arg);
  return 0;
}

// Sends the next opener, if one is due. Called once a second from the board's
// own loop, so the radio work stays off the callback stack.
void bleLinkServiceExploration()
{
  bleLinkInitAll();

  const int probeCount = (int)(sizeof(kBleProbeCommands) / sizeof(kBleProbeCommands[0]));

  for (int slot = 0; slot < BLE_LINK_MAX_NODES; slot++)
  {
    BleLink *link = &bleLinks[slot];

    if (!link->exploring) continue;

    if (link->pascoPresent)
    {
      blePascoService(slot);
      continue;
    }

    if (link->exploreProbeIndex < 0 || link->exploreProbeIndex >= probeCount) continue;
    if (link->exploreCommandCount == 0) continue;
    if (link->connHandle == BLE_HS_CONN_HANDLE_NONE) continue;
    if ((long)(millis() - link->exploreNextProbeMs) < 0) continue;

    const BleProbeCommand *cmd = &kBleProbeCommands[link->exploreProbeIndex];

    // Nothing to send while the table is empty.
    if (cmd->len == 0)
    {
      link->exploreProbeIndex = probeCount;
      bleLinkSetState(slot, "분석 완료 · 데이터 대기");
      continue;
    }

    // Each opener goes to every command-shaped characteristic; which one is
    // the way in is part of what is being established.
    for (int i = 0; i < link->exploreCommandCount; i++)
    {
      blePrintf("explore: trying opener %s on handle %u",
                cmd->note, (unsigned)link->exploreCommandHandles[i]);

      ble_gattc_write_flat(link->connHandle, link->exploreCommandHandles[i],
                           cmd->bytes, cmd->len, bleExploreOnWrite, NULL);
    }

    snprintf(link->exploreSummary, sizeof(link->exploreSummary),
             "명령 시도 %s", cmd->note);

    link->exploreProbeIndex++;
    link->exploreNextProbeMs = millis() + 1500;

    if (link->exploreProbeIndex >= probeCount)
    {
      blePrintf("explore: all openers tried; watching for anything new");
      bleLinkSetState(slot, "명령 시도 끝 · 응답 관찰");
    }
  }
}

bool bleLinkSlotIsExploring(int slot)
{
  bleLinkInitAll();
  return bleSlotValid(slot) && bleLinks[slot].exploring;
}

const char *bleLinkExploreSummary(int slot)
{
  bleLinkInitAll();
  return bleSlotValid(slot) ? bleLinks[slot].exploreSummary : "";
}

bool bleLinkConnect(const char *address)
{
  bleLinkInitAll();

  if (!bleStarted) return false;
  if (address == NULL || strlen(address) != 17) return false;

  // An address already in a slot reconnects there rather than taking a second.
  int slot = bleLinkSlotForAddress(address);

  if (slot >= 0)
  {
    if (bleLinks[slot].subscribed || bleLinks[slot].busy) return true;
  }
  else
  {
    slot = bleLinkFreeSlot();
  }

  if (slot < 0)
  {
    blePrintf("no free slot: %d nodes already connected", BLE_LINK_MAX_NODES);
    return false;
  }

  // Scanning and connecting cannot share the radio.
  if (bleScanning)
  {
    ble_gap_disc_cancel();
    bleScanning = false;
  }

  unsigned int b[6] = { 0 };
  if (sscanf(address, "%02x:%02x:%02x:%02x:%02x:%02x",
             &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6)
  {
    return false;
  }

  ble_addr_t peer;
  memset(&peer, 0, sizeof(peer));

  // bleFormatAddr() prints most significant byte first; NimBLE stores it the
  // other way round.
  for (int i = 0; i < 6; i++) peer.val[i] = (uint8_t)b[5 - i];

  // A random static address has its two top bits set. Read that from the
  // address rather than trying one type and retrying, because ble_gap_connect
  // accepts a wrong type and then simply times out.
  peer.type = ((peer.val[5] & 0xC0) == 0xC0) ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;

  snprintf(bleLinks[slot].peerAddress, sizeof(bleLinks[slot].peerAddress), "%s", address);
  snprintf(bleLinks[slot].peerName, sizeof(bleLinks[slot].peerName), "%s", address);

  for (int i = 0; i < bleScanCount; i++)
  {
    if (strcmp(bleScanResults[i].address, address) == 0 && bleScanResults[i].name[0])
    {
      snprintf(bleLinks[slot].peerName, sizeof(bleLinks[slot].peerName), "%s",
               bleScanResults[i].name);
      break;
    }
  }

  bleLinks[slot].busy = true;
  bleLinks[slot].hasReading = false;
  bleLinks[slot].notifyCount = 0;
  bleLinks[slot].exploreServices = 0;
  bleLinks[slot].exploreChrs = 0;
  bleLinks[slot].exploreSubscribed = 0;
  bleLinks[slot].exploreReadCount = 0;
  bleLinks[slot].exploreReadIndex = 0;
  bleLinks[slot].exploreCommandCount = 0;
  bleLinks[slot].exploreProbeIndex = -1;
  bleLinks[slot].pascoPresent = false;
  bleLinks[slot].pascoStep = 0;
  bleLinks[slot].pascoSensorId = 0;
  bleLinks[slot].pascoSampleBytes = 4;
  memset(bleLinks[slot].pascoHandle, 0, sizeof(bleLinks[slot].pascoHandle));
  bleLinks[slot].exploreSummary[0] = '\0';
  bleLinkSetState(slot, bleLinks[slot].exploring ? "분석 연결 중" : "연결 중");

  const int rc = ble_gap_connect(bleAddrType, &peer, 10000, NULL,
                                 bleLinkGapEvent, (void *)(intptr_t)slot);

  if (rc != 0)
  {
    blePrintf("slot %d: ble_gap_connect failed: %d", slot, rc);
    bleLinkClear(slot, "연결 실패", true);
    return false;
  }

  return true;
}

bool bleLinkExplore(const char *address)
{
  bleLinkInitAll();

  const int existing = bleLinkSlotForAddress(address);
  if (existing >= 0) bleLinkDisconnectSlot(existing);

  const int slot = bleLinkFreeSlot();
  if (slot < 0) return false;

  // Set before connecting: the GAP callback reads it to decide which discovery
  // to run.
  bleLinks[slot].exploring = true;

  if (!bleLinkConnect(address))
  {
    bleLinks[slot].exploring = false;
    return false;
  }

  return true;
}

void bleLinkDisconnectSlot(int slot)
{
  bleLinkInitAll();
  if (!bleSlotValid(slot)) return;

  if (bleLinks[slot].connHandle != BLE_HS_CONN_HANDLE_NONE)
  {
    ble_gap_terminate(bleLinks[slot].connHandle, BLE_ERR_REM_USER_CONN_TERM);
  }

  bleLinkClear(slot, "연결 안 됨", false);
}

void bleLinkDisconnect()
{
  for (int i = 0; i < BLE_LINK_MAX_NODES; i++) bleLinkDisconnectSlot(i);
}

bool bleLinkSlotIsSubscribed(int slot)
{
  bleLinkInitAll();
  return bleSlotValid(slot) && bleLinks[slot].subscribed;
}

bool bleLinkSlotIsBusy(int slot)
{
  bleLinkInitAll();
  return bleSlotValid(slot) && bleLinks[slot].busy;
}

bool bleLinkIsBusy()
{
  for (int i = 0; i < BLE_LINK_MAX_NODES; i++)
  {
    if (bleLinkSlotIsBusy(i)) return true;
  }

  return false;
}

int bleLinkNodeCount()
{
  int count = 0;

  for (int i = 0; i < BLE_LINK_MAX_NODES; i++)
  {
    if (bleLinkSlotIsSubscribed(i)) count++;
  }

  return count;
}

int bleLinkFirstSubscribedSlot()
{
  for (int i = 0; i < BLE_LINK_MAX_NODES; i++)
  {
    if (bleLinkSlotIsSubscribed(i)) return i;
  }

  return -1;
}

const char *bleLinkSlotName(int slot)
{
  bleLinkInitAll();
  return bleSlotValid(slot) ? bleLinks[slot].peerName : "";
}

const char *bleLinkSlotState(int slot)
{
  bleLinkInitAll();
  return bleSlotValid(slot) ? bleLinks[slot].state : "연결 안 됨";
}

int bleLinkValueCount(int slot)
{
  bleLinkInitAll();
  if (!bleSlotValid(slot) || !bleLinks[slot].hasReading) return 0;

  int commas = 0;
  for (const char *c = bleLinks[slot].payload; *c; c++)
  {
    if (*c == ',') commas++;
  }

  // One timestamp then groups of three. An incomplete trailing group - which
  // is what a too-small MTU produces - is not counted, so a clipped packet
  // yields fewer values rather than a wrong one.
  int groups = commas / 3;
  if (groups > BLE_LINK_MAX_VALUES) groups = BLE_LINK_MAX_VALUES;
  return groups;
}

bool bleLinkValueAt(int slot, int index, float *value, char *name, char *unit, uint32_t *ageMs)
{
  if (index < 0 || index >= bleLinkValueCount(slot)) return false;

  const BleLink *link = &bleLinks[slot];

  char work[sizeof(link->payload)];
  snprintf(work, sizeof(work), "%s", link->payload);

  char *fields[1 + 3 * BLE_LINK_MAX_VALUES];
  const int maxFields = (int)(sizeof(fields) / sizeof(fields[0]));
  int found = 0;
  char *cursor = work;

  fields[found++] = cursor;

  while (*cursor && found < maxFields)
  {
    if (*cursor == ',')
    {
      *cursor = '\0';
      fields[found++] = cursor + 1;
    }
    cursor++;
  }

  const int base = 1 + index * 3;   // field 0 is the node's own timestamp
  if (base + 2 >= found) return false;

  if (name) snprintf(name, 24, "%s", fields[base]);
  if (value) *value = strtof(fields[base + 1], NULL);
  if (unit) snprintf(unit, 24, "%s", fields[base + 2]);

  if (ageMs)
  {
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    *ageMs = now - link->payloadMs;
  }

  return true;
}

// bleFormatServices() prints a 128-bit UUID as its leading four bytes, which
// is enough to tell this project's nodes from anything else in the room.
#define BLE_SENSOR_SERVICE_TAG "6e5f0001"

int bleAutoLinkToSensorNode()
{
  bleLinkInitAll();

  int started = 0;

  for (int i = 0; i < bleScanCount; i++)
  {
    if (strstr(bleScanResults[i].services, BLE_SENSOR_SERVICE_TAG) == NULL) continue;

    const int existing = bleLinkSlotForAddress(bleScanResults[i].address);
    if (existing >= 0 && (bleLinks[existing].subscribed || bleLinks[existing].busy)) continue;

    if (bleLinkFreeSlot() < 0 && existing < 0) break;

    blePrintf("auto-linking to %s (%s, %d dBm)",
              bleScanResults[i].name[0] ? bleScanResults[i].name : "(unnamed)",
              bleScanResults[i].address, bleScanResults[i].rssi);

    if (bleLinkConnect(bleScanResults[i].address)) started++;
  }

  return started;
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
void bleLinkDisconnectSlot(int slot) { (void)slot; }
void bleLinkDisconnect() {}
int bleLinkNodeCount() { return 0; }
int bleLinkFirstSubscribedSlot() { return -1; }
bool bleLinkSlotIsSubscribed(int slot) { (void)slot; return false; }
bool bleLinkSlotIsBusy(int slot) { (void)slot; return false; }
bool bleLinkIsBusy() { return false; }
int bleLinkSlotForAddress(const char *address) { (void)address; return -1; }
const char *bleLinkSlotName(int slot) { (void)slot; return ""; }
const char *bleLinkSlotState(int slot) { (void)slot; return "지원 안 함"; }
int bleLinkValueCount(int slot) { (void)slot; return 0; }
bool bleLinkValueAt(int slot, int index, float *value, char *name, char *unit, uint32_t *ageMs)
{
  (void)slot; (void)index; (void)value; (void)name; (void)unit; (void)ageMs;
  return false;
}
int bleAutoLinkToSensorNode() { return 0; }
bool bleLinkExplore(const char *address) { (void)address; return false; }
void bleLinkServiceExploration() {}
bool bleLinkSlotIsExploring(int slot) { (void)slot; return false; }
const char *bleLinkExploreSummary(int slot) { (void)slot; return ""; }

#endif  // BLE_SENSOR_SUPPORTED
