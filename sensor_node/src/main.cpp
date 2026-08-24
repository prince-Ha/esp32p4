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

// Must match src/ble_sensor.cpp on the P4 exactly.
#define SERVICE_UUID     "6e5f0001-8b9a-4c1d-9f2e-7a3b5c8d1e40"
#define MEASUREMENT_UUID "6e5f0002-8b9a-4c1d-9f2e-7a3b5c8d1e40"

// The P4's scan list shows advertised names, so this prefix is what makes a
// node recognisable among the phones and laptops in a classroom.
#define NODE_NAME_PREFIX "SciNode-"

#define PUBLISH_INTERVAL_MS 1000

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// The P4 shows at most three values at once, so there is no point sending more.
#define NODE_MAX_VALUES 3

typedef struct
{
  float value;
  const char *name;
  const char *unit;
} NodeValue;

static BLECharacteristic *measurementChar = NULL;
static bool centralConnected = false;
static char nodeName[24] = NODE_NAME_PREFIX;
static uint32_t bootSeconds = 0;
static unsigned long lastPublishMs = 0;

static bool dpsReady = false;

static bool dps310Begin();
static int readNodeSensor(NodeValue *out, int maxValues);

// Every address that answers on the bus. A sensor that does not appear here is
// a wiring or power fault, and no amount of driver work will help.
static int scanI2cBus()
{
  int found = 0;

  Serial.printf("[I2C] scanning SDA=GPIO%d SCL=GPIO%d\n", I2C_SDA_PIN, I2C_SCL_PIN);

  for (uint8_t addr = 0x08; addr <= 0x77; addr++)
  {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) continue;

    found++;
    Serial.printf("[I2C] 0x%02X responded\n", addr);
  }

  if (found == 0)
  {
    Serial.println("[I2C] nothing on the bus - check SDA/SCL are not swapped, "
                   "that VCC is 3.3V, and that GND is shared");
  }

  return found;
}

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

  scanI2cBus();

  if (!dps310Begin())
  {
    Serial.println("[SENSOR] DPS310 not found - check SDA/SCL, 3.3V, and the address");
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

  // A sensor connected after boot should just start working.
  if (!dpsReady && (bootSeconds % 5) == 0)
  {
    if (dps310Begin()) Serial.println("[SENSOR] DPS310 appeared");
  }

  NodeValue values[NODE_MAX_VALUES];
  const int count = readNodeSensor(values, NODE_MAX_VALUES);

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

// =====================================================
// DPS310 기압·온도
//
// Ported from the P4's own driver so a sensor on a node and the same part on
// the board read identically. The compensation is the one from the datasheet:
// nine calibration coefficients, and a temperature term that also has to be
// applied to the pressure.
// =====================================================

#define DPS310_REG_PRS_B2   0x00
#define DPS310_REG_TMP_B2   0x03
#define DPS310_REG_PRS_CFG  0x06
#define DPS310_REG_TMP_CFG  0x07
#define DPS310_REG_MEAS_CFG 0x08
#define DPS310_REG_CFG_REG  0x09
#define DPS310_REG_RESET    0x0C
#define DPS310_REG_PROD_ID  0x0D
#define DPS310_REG_COEF     0x10
#define DPS310_REG_TMP_COEF 0x28

// Single measurement per second, no oversampling: both scale factors are 2^19.
#define DPS310_SCALE_FACTOR 524288.0f

static uint8_t dpsAddress = 0x00;

static int16_t dpsC0, dpsC1, dpsC01, dpsC11, dpsC20, dpsC21, dpsC30;
static int32_t dpsC00, dpsC10;

static int32_t signExtend(uint32_t value, uint8_t bits)
{
  const uint32_t mask = 1UL << (bits - 1);
  value &= ((1UL << bits) - 1);
  return (int32_t)((value ^ mask) - mask);
}

static bool dpsWriteRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(dpsAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool dpsReadRegisters(uint8_t reg, uint8_t *out, size_t len)
{
  Wire.beginTransmission(dpsAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((int)dpsAddress, (int)len) != (int)len) return false;

  for (size_t i = 0; i < len; i++) out[i] = Wire.read();
  return true;
}

static bool dpsReadRegister(uint8_t reg, uint8_t *out)
{
  return dpsReadRegisters(reg, out, 1);
}

static bool dpsReadCoefficients()
{
  uint8_t d[18];
  if (!dpsReadRegisters(DPS310_REG_COEF, d, 18)) return false;

  dpsC0  = (int16_t)signExtend(((uint16_t)d[0] << 4) | (d[1] >> 4), 12);
  dpsC1  = (int16_t)signExtend(((uint16_t)(d[1] & 0x0F) << 8) | d[2], 12);
  dpsC00 = signExtend(((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | (d[5] >> 4), 20);
  dpsC10 = signExtend(((uint32_t)(d[5] & 0x0F) << 16) | ((uint32_t)d[6] << 8) | d[7], 20);
  dpsC01 = (int16_t)signExtend(((uint16_t)d[8] << 8) | d[9], 16);
  dpsC11 = (int16_t)signExtend(((uint16_t)d[10] << 8) | d[11], 16);
  dpsC20 = (int16_t)signExtend(((uint16_t)d[12] << 8) | d[13], 16);
  dpsC21 = (int16_t)signExtend(((uint16_t)d[14] << 8) | d[15], 16);
  dpsC30 = (int16_t)signExtend(((uint16_t)d[16] << 8) | d[17], 16);

  return true;
}

static bool dps310Begin()
{
  // Seeed's Grove board answers at 0x77; bare breakouts often strap 0x76. Try
  // both rather than making the wiring depend on which one was bought.
  const uint8_t candidates[2] = { 0x77, 0x76 };

  for (int i = 0; i < 2; i++)
  {
    dpsAddress = candidates[i];

    uint8_t productId = 0;
    if (!dpsReadRegister(DPS310_REG_PROD_ID, &productId)) continue;

    Serial.printf("[SENSOR] DPS310 at 0x%02X, product ID 0x%02X\n", dpsAddress, productId);

    dpsWriteRegister(DPS310_REG_RESET, 0x89);
    delay(100);

    uint8_t status = 0;
    const unsigned long start = millis();

    while (millis() - start < 250)
    {
      if (dpsReadRegister(DPS310_REG_MEAS_CFG, &status) && (status & 0xC0) == 0xC0) break;
      delay(10);
    }

    if ((status & 0xC0) != 0xC0)
    {
      Serial.println("[SENSOR] DPS310 ready timeout");
      continue;
    }

    if (!dpsReadCoefficients())
    {
      Serial.println("[SENSOR] DPS310 coefficient read failed");
      continue;
    }

    // Bit 7 of TMP_COEF says which internal sensor the coefficients were taken
    // from, and TMP_CFG has to name the same one or the temperature comes out
    // several degrees off.
    uint8_t tempCoefSource = 0;
    dpsReadRegister(DPS310_REG_TMP_COEF, &tempCoefSource);
    const uint8_t tempSourceBit = (tempCoefSource & 0x80) ? 0x80 : 0x00;

    dpsWriteRegister(DPS310_REG_PRS_CFG, 0x00);
    dpsWriteRegister(DPS310_REG_TMP_CFG, tempSourceBit | 0x00);
    dpsWriteRegister(DPS310_REG_CFG_REG, 0x00);
    dpsWriteRegister(DPS310_REG_MEAS_CFG, 0x07);   // continuous pressure + temperature

    delay(100);

    Serial.println("[SENSOR] DPS310 init OK");
    dpsReady = true;
    return true;
  }

  dpsAddress = 0x00;
  dpsReady = false;
  return false;
}

static bool readDps310(float *temperatureC, float *pressureHpa)
{
  if (!dpsReady) return false;

  uint8_t status = 0;
  const unsigned long start = millis();

  while (millis() - start < 250)
  {
    if (dpsReadRegister(DPS310_REG_MEAS_CFG, &status) && (status & 0x30) == 0x30) break;
    delay(5);
  }

  if ((status & 0x30) != 0x30) return false;

  uint8_t pData[3];
  uint8_t tData[3];

  if (!dpsReadRegisters(DPS310_REG_PRS_B2, pData, 3)) return false;
  if (!dpsReadRegisters(DPS310_REG_TMP_B2, tData, 3)) return false;

  const int32_t rawPressure = signExtend(
    ((uint32_t)pData[0] << 16) | ((uint32_t)pData[1] << 8) | pData[2], 24);
  const int32_t rawTemp = signExtend(
    ((uint32_t)tData[0] << 16) | ((uint32_t)tData[1] << 8) | tData[2], 24);

  const float tRawSc = rawTemp / DPS310_SCALE_FACTOR;
  const float pRawSc = rawPressure / DPS310_SCALE_FACTOR;

  *temperatureC = (dpsC0 * 0.5f) + (dpsC1 * tRawSc);

  const float pressurePa =
    dpsC00 +
    pRawSc * (dpsC10 + pRawSc * (dpsC20 + pRawSc * dpsC30)) +
    tRawSc * dpsC01 +
    tRawSc * pRawSc * (dpsC11 + pRawSc * dpsC21);

  *pressureHpa = pressurePa / 100.0f;
  return true;
}

// =====================================================
// 여기를 바꾸세요 - Replace this with your sensor
//
// Fill in up to `maxValues` quantities and return how many. Return 0 when
// nothing is available yet; the node skips that second rather than sending a
// wrong number.
//
// The names appear on the P4's measurement screen and in the CSV, so they can
// be Korean. Keep the units ASCII - see the note at the top of this file.
// =====================================================
static int readNodeSensor(NodeValue *out, int maxValues)
{
  if (maxValues < 2) return 0;

  float temperatureC = NAN;
  float pressureHpa = NAN;

  if (!readDps310(&temperatureC, &pressureHpa)) return 0;

  out[0].value = temperatureC;
  out[0].name = "온도";
  out[0].unit = "C";

  out[1].value = pressureHpa;
  out[1].name = "기압";
  out[1].unit = "hPa";

  return 2;
}
