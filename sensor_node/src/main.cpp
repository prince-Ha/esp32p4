// =====================================================
// BLE 센서 노드 (ESP32 dev board)
//
// Advertises the service the ESP32-P4 dashboard scans for and notifies one
// reading per second. The P4 connects to this board, subscribes, and treats
// the readings as if they came from a sensor on its own bus: they land on the
// measurement screen, in the CSV on the SD card, and in the 지능형 과학실
// upload.
//
// The payload is one UTF-8 line, the same shape the P4 publishes on its own
// characteristic:
//
//     <time_s>,<sensor>,<value>,<unit>
//     42,온도,23.5,C
//
// Commas are the separator, so no field may contain one.
//
// To send a real measurement, replace readNodeSensor() at the bottom. Out of
// the box it sends an obvious test ramp so the link can be proven before any
// sensor is wired up.
// =====================================================

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_mac.h>

// Must match src/ble_sensor.cpp on the P4 exactly.
#define SERVICE_UUID     "6e5f0001-8b9a-4c1d-9f2e-7a3b5c8d1e40"
#define MEASUREMENT_UUID "6e5f0002-8b9a-4c1d-9f2e-7a3b5c8d1e40"

// The P4's scan list shows advertised names, so this prefix is what makes a
// node recognisable among the phones and laptops in a classroom.
#define NODE_NAME_PREFIX "SciNode-"

#define PUBLISH_INTERVAL_MS 1000

static BLECharacteristic *measurementChar = NULL;
static bool centralConnected = false;
static char nodeName[24] = NODE_NAME_PREFIX;
static uint32_t bootSeconds = 0;
static unsigned long lastPublishMs = 0;

static bool readNodeSensor(float *value, const char **name, const char **unit);

class NodeServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *server) override
  {
    (void)server;
    centralConnected = true;
    Serial.println("[BLE] central connected");
  }

  void onDisconnect(BLEServer *server) override
  {
    centralConnected = false;
    Serial.println("[BLE] central disconnected, advertising again");

    // Without this the node goes quiet after the hub drops, and the only way
    // back is a power cycle.
    server->startAdvertising();
  }
};

void setup()
{
  Serial.begin(115200);
  delay(200);

  // Last two bytes of the MAC, so several nodes in one room stay apart.
  uint8_t mac[6] = { 0 };
  esp_read_mac(mac, ESP_MAC_BT);
  snprintf(nodeName, sizeof(nodeName), NODE_NAME_PREFIX "%02X%02X", mac[4], mac[5]);

  Serial.printf("\n[BLE] sensor node starting as %s\n", nodeName);

  BLEDevice::init(nodeName);

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new NodeServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);

  measurementChar = service->createCharacteristic(
    MEASUREMENT_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );

  // The client subscribes by writing to this descriptor. Without it the P4 can
  // connect and read, but will never receive a notification.
  measurementChar->addDescriptor(new BLE2902());
  measurementChar->setValue("0,대기,0,-");

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);

  // The P4 identifies candidates by advertised service UUID, so it has to be
  // in the advertisement itself, not only in the scan response.
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();

  Serial.println("[BLE] advertising, waiting for the P4 to connect");
}

void loop()
{
  const unsigned long now = millis();

  if (now - lastPublishMs < PUBLISH_INTERVAL_MS)
  {
    delay(10);
    return;
  }

  lastPublishMs = now;
  bootSeconds++;

  float value = NAN;
  const char *name = "";
  const char *unit = "";

  if (!readNodeSensor(&value, &name, &unit))
  {
    Serial.println("[SENSOR] read failed");
    return;
  }

  char payload[96];
  snprintf(payload, sizeof(payload), "%lu,%s,%.4f,%s",
           (unsigned long)bootSeconds, name, value, unit);

  measurementChar->setValue((uint8_t *)payload, strlen(payload));

  if (centralConnected)
  {
    measurementChar->notify();
  }

  Serial.printf("[SENSOR] %s%s\n", payload, centralConnected ? "" : "  (nobody listening)");
}

// =====================================================
// 여기를 바꾸세요 - Replace this with your sensor
//
// Return false when the reading is not available yet; the node simply skips
// that second rather than sending a wrong number.
//
// `name` is what appears on the P4's measurement screen and in the CSV, and
// `unit` is what goes to 지능형 과학실, so keep both short and free of commas.
//
// Example for a DS18B20 on GPIO4:
//
//   #include <OneWire.h>
//   #include <DallasTemperature.h>
//   static OneWire wire(4);
//   static DallasTemperature probe(&wire);
//   // in setup(): probe.begin();
//   probe.requestTemperatures();
//   const float c = probe.getTempCByIndex(0);
//   if (c == DEVICE_DISCONNECTED_C) return false;
//   *value = c; *name = "수온"; *unit = "C";
//   return true;
// =====================================================
static bool readNodeSensor(float *value, const char **name, const char **unit)
{
  // A 0 to 100 ramp that repeats every 100 seconds. Obviously synthetic, which
  // is the point: if the P4 shows this climbing, the whole path works and the
  // only thing left to do is put a real sensor in here.
  *value = (float)(bootSeconds % 100);
  *name = "테스트";
  *unit = "-";
  return true;
}
