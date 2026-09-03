// =====================================================
// BLE 센서 노드 (ESP32 dev board)
//
// Advertises the service the ESP32-P4 dashboard scans for and notifies its
// readings once a second. The P4 connects to this board, subscribes, and
// treats what arrives like a sensor on its own bus: measurement screen, CSV on
// the SD card, the lot.
//
// The payload is one UTF-8 line - a timestamp followed by one to three
// quantities, each a name, a value and a unit:
//
//     <time_s>,<name>,<value>,<unit>[,<name>,<value>,<unit>]...
//     42,온도,23.5000,C,기압,1013.2000,hPa
//
// Commas separate the fields, so no field may contain one. Keep units ASCII:
// the P4 draws them in a font subset cut at build time, which cannot know what
// a node will say at runtime.
//
// To send something else, replace readNodeSensor() at the bottom - it is the
// only function meant to be edited.
//
// I2C wiring for the Grove DPS310, and for any other I2C sensor:
//
//     Grove          ESP32 dev
//     SCL (yellow)   GPIO22
//     SDA (white)    GPIO21
//     VCC (red)      3.3V
//     GND (black)    GND
// =====================================================

#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_mac.h>
#include "sensors.h"

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

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  delay(50);

  nodeScanI2cBus();

  if (!nodeSensorDetect())
  {
    Serial.println("[SENSOR] nothing recognised on the bus - the node keeps looking");
  }

  // Last two bytes of the MAC, so several nodes in one room stay apart.
  uint8_t mac[6] = { 0 };
  esp_read_mac(mac, ESP_MAC_BT);
  snprintf(nodeName, sizeof(nodeName), NODE_NAME_PREFIX "%02X%02X", mac[4], mac[5]);

  Serial.printf("\n[BLE] sensor node starting as %s\n", nodeName);

  BLEDevice::init(nodeName);

  // The default ATT MTU of 23 leaves 20 bytes per notification, and a reading
  // with two quantities and Korean names is well past that. The P4 asks for a
  // bigger one, but the server has to be willing to grant it.
  BLEDevice::setMTU(247);

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
  // in the advertisement itself. That leaves no room for the name, which goes
  // in the scan response instead.
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

  // A sensor swapped or plugged in after boot is picked up without a reset,
  // and the P4 follows on its own because every packet carries its own names
  // and units.
  if (nodeSensorKind() == NODE_SENSOR_NONE && (bootSeconds % 5) == 0)
  {
    if (nodeSensorDetect()) Serial.printf("[SENSOR] %s appeared\n", nodeSensorName());
  }

  NodeValue values[NODE_MAX_VALUES];
  const int count = nodeSensorRead(values, NODE_MAX_VALUES);

  if (count <= 0)
  {
    Serial.println("[SENSOR] no reading this second");
    return;
  }

  char payload[160];
  int used = snprintf(payload, sizeof(payload), "%lu", (unsigned long)bootSeconds);

  for (int i = 0; i < count && used < (int)sizeof(payload); i++)
  {
    used += snprintf(payload + used, sizeof(payload) - used, ",%s,%.4f,%s",
                     values[i].name, values[i].value, values[i].unit);
  }

  measurementChar->setValue((uint8_t *)payload, strlen(payload));

  if (centralConnected)
  {
    measurementChar->notify();
  }

  Serial.printf("[SENSOR] %s%s\n", payload, centralConnected ? "" : "  (nobody listening)");
}
