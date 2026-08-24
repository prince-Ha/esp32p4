// Drivers ported from the P4's src/main.cpp, bit-banged there and on Wire
// here. Register sequences and compensation maths are unchanged on purpose:
// the same part must read the same number on either board.

#include "sensors.h"

#include <math.h>
#include <driver/gpio.h>

static NodeSensorKind detectedKind = NODE_SENSOR_NONE;
static uint8_t detectedAddress = 0x00;
// Consecutive seconds the detected part has failed to acknowledge.
static int missingReads = 0;

// ---------------------------------------------------------------- I2C ------

static bool i2cWrite(uint8_t addr, const uint8_t *data, size_t len)
{
  Wire.beginTransmission(addr);
  Wire.write(data, len);
  return Wire.endTransmission() == 0;
}

static bool i2cWriteRegister8(uint8_t addr, uint8_t reg, uint8_t value)
{
  const uint8_t buf[2] = { reg, value };
  return i2cWrite(addr, buf, 2);
}

static bool i2cReadRegisters(uint8_t addr, uint8_t reg, uint8_t *out, size_t len)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)len) != (int)len) return false;

  for (size_t i = 0; i < len; i++) out[i] = Wire.read();
  return true;
}

static bool i2cReadRegister8(uint8_t addr, uint8_t reg, uint8_t *out)
{
  return i2cReadRegisters(addr, reg, out, 1);
}

// The VL53L1X addresses its registers with 16 bits, unlike everything else here.
static bool i2cWrite16Addr8(uint8_t addr, uint16_t reg, uint8_t value)
{
  const uint8_t buf[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), value };
  return i2cWrite(addr, buf, 3);
}

static bool i2cRead16Addr(uint8_t addr, uint16_t reg, uint8_t *out, size_t len)
{
  const uint8_t buf[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };

  Wire.beginTransmission(addr);
  Wire.write(buf, 2);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)len) != (int)len) return false;

  for (size_t i = 0; i < len; i++) out[i] = Wire.read();
  return true;
}

static int32_t signExtend(uint32_t value, uint8_t bits)
{
  const uint32_t mask = 1UL << (bits - 1);
  value &= ((1UL << bits) - 1);
  return (int32_t)((value ^ mask) - mask);
}

int nodeScanI2cBus()
{
  int found = 0;

  Serial.println("[I2C] scanning the sensor bus");

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

// ------------------------------------------------------------- DPS310 ------

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

// One measurement per second, no oversampling: both scale factors are 2^19.
#define DPS310_SCALE_FACTOR 524288.0f

static int16_t dpsC0, dpsC1, dpsC01, dpsC11, dpsC20, dpsC21, dpsC30;
static int32_t dpsC00, dpsC10;

static bool dps310Begin(uint8_t addr)
{
  uint8_t productId = 0;
  if (!i2cReadRegister8(addr, DPS310_REG_PROD_ID, &productId)) return false;

  Serial.printf("[DPS310] at 0x%02X, product ID 0x%02X\n", addr, productId);

  i2cWriteRegister8(addr, DPS310_REG_RESET, 0x89);
  delay(100);

  uint8_t status = 0;
  const unsigned long start = millis();

  while (millis() - start < 250)
  {
    if (i2cReadRegister8(addr, DPS310_REG_MEAS_CFG, &status) && (status & 0xC0) == 0xC0) break;
    delay(10);
  }

  if ((status & 0xC0) != 0xC0)
  {
    Serial.println("[DPS310] ready timeout");
    return false;
  }

  uint8_t d[18];
  if (!i2cReadRegisters(addr, DPS310_REG_COEF, d, 18))
  {
    Serial.println("[DPS310] coefficient read failed");
    return false;
  }

  dpsC0  = (int16_t)signExtend(((uint16_t)d[0] << 4) | (d[1] >> 4), 12);
  dpsC1  = (int16_t)signExtend(((uint16_t)(d[1] & 0x0F) << 8) | d[2], 12);
  dpsC00 = signExtend(((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | (d[5] >> 4), 20);
  dpsC10 = signExtend(((uint32_t)(d[5] & 0x0F) << 16) | ((uint32_t)d[6] << 8) | d[7], 20);
  dpsC01 = (int16_t)signExtend(((uint16_t)d[8] << 8) | d[9], 16);
  dpsC11 = (int16_t)signExtend(((uint16_t)d[10] << 8) | d[11], 16);
  dpsC20 = (int16_t)signExtend(((uint16_t)d[12] << 8) | d[13], 16);
  dpsC21 = (int16_t)signExtend(((uint16_t)d[14] << 8) | d[15], 16);
  dpsC30 = (int16_t)signExtend(((uint16_t)d[16] << 8) | d[17], 16);

  // Bit 7 of TMP_COEF says which internal sensor the coefficients came from,
  // and TMP_CFG has to name the same one or the temperature is degrees out.
  uint8_t tempCoefSource = 0;
  i2cReadRegister8(addr, DPS310_REG_TMP_COEF, &tempCoefSource);

  i2cWriteRegister8(addr, DPS310_REG_PRS_CFG, 0x00);
  i2cWriteRegister8(addr, DPS310_REG_TMP_CFG, (tempCoefSource & 0x80) | 0x00);
  i2cWriteRegister8(addr, DPS310_REG_CFG_REG, 0x00);
  i2cWriteRegister8(addr, DPS310_REG_MEAS_CFG, 0x07);

  delay(100);
  Serial.println("[DPS310] init OK");
  return true;
}

static int dps310Read(NodeValue *out, int maxValues)
{
  if (maxValues < 2) return 0;

  uint8_t status = 0;

  // If it does not answer at all, say so now rather than polling a part that
  // has been unplugged for a quarter of a second.
  if (!i2cReadRegister8(detectedAddress, DPS310_REG_MEAS_CFG, &status)) return 0;

  const unsigned long start = millis();

  while ((status & 0x30) != 0x30 && millis() - start < 250)
  {
    delay(5);
    if (!i2cReadRegister8(detectedAddress, DPS310_REG_MEAS_CFG, &status)) return 0;
  }

  if ((status & 0x30) != 0x30) return 0;

  uint8_t pData[3];
  uint8_t tData[3];

  if (!i2cReadRegisters(detectedAddress, DPS310_REG_PRS_B2, pData, 3)) return 0;
  if (!i2cReadRegisters(detectedAddress, DPS310_REG_TMP_B2, tData, 3)) return 0;

  const float tRawSc = signExtend(((uint32_t)tData[0] << 16) | ((uint32_t)tData[1] << 8) | tData[2], 24)
                       / DPS310_SCALE_FACTOR;
  const float pRawSc = signExtend(((uint32_t)pData[0] << 16) | ((uint32_t)pData[1] << 8) | pData[2], 24)
                       / DPS310_SCALE_FACTOR;

  const float pressurePa =
    dpsC00 +
    pRawSc * (dpsC10 + pRawSc * (dpsC20 + pRawSc * dpsC30)) +
    tRawSc * dpsC01 +
    tRawSc * pRawSc * (dpsC11 + pRawSc * dpsC21);

  out[0].value = (dpsC0 * 0.5f) + (dpsC1 * tRawSc);
  out[0].name = "온도";
  out[0].unit = "C";

  out[1].value = pressurePa / 100.0f;
  out[1].name = "기압";
  out[1].unit = "hPa";

  return 2;
}

// ------------------------------------------------------------- TMP117 ------

#define TMP117_REG_TEMPERATURE    0x00
#define TMP117_REG_DEVICE_ID      0x0F
#define TMP117_EXPECTED_DEVICE_ID 0x0117
#define TMP117_LSB_C              0.0078125f

static bool tmp117Read16(uint8_t reg, uint16_t *out)
{
  uint8_t buf[2];
  if (!i2cReadRegisters(detectedAddress, reg, buf, 2)) return false;
  *out = ((uint16_t)buf[0] << 8) | buf[1];
  return true;
}

static bool tmp117Begin(uint8_t addr)
{
  uint8_t buf[2];
  if (!i2cReadRegisters(addr, TMP117_REG_DEVICE_ID, buf, 2)) return false;

  const uint16_t deviceId = ((uint16_t)buf[0] << 8) | buf[1];
  Serial.printf("[TMP117] device ID 0x%04X\n", deviceId);

  if (deviceId != TMP117_EXPECTED_DEVICE_ID) return false;

  Serial.println("[TMP117] init OK; the first conversion takes about a second");
  return true;
}

static int tmp117Read(NodeValue *out, int maxValues)
{
  if (maxValues < 1) return 0;

  uint16_t raw = 0;
  if (!tmp117Read16(TMP117_REG_TEMPERATURE, &raw)) return 0;

  // 0x8000 is what it reports until the first conversion finishes.
  if (raw == 0x8000) return 0;

  const float t = (float)(int16_t)raw * TMP117_LSB_C;
  if (isnan(t) || isinf(t) || t < -55.0f || t > 150.0f) return 0;

  out[0].value = t;
  out[0].name = "정밀온도";
  out[0].unit = "C";
  return 1;
}

// -------------------------------------------------------------- SCD41 ------

#define SCD41_CMD_START_PERIODIC 0x21B1
#define SCD41_CMD_READ_MEASUREMENT 0xEC05
#define SCD41_CMD_STOP_PERIODIC 0x3F86
#define SCD41_CMD_GET_DATA_READY 0xE4B8
#define SCD41_CMD_REINIT 0x3646
#define SCD41_CMD_WAKE_UP 0x36F6

static unsigned long scd41NextPollMs = 0;

static uint8_t scd41Crc8(const uint8_t *data, size_t len)
{
  uint8_t crc = 0xFF;

  for (size_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
  }

  return crc;
}

static bool scd41Command(uint8_t addr, uint16_t command)
{
  const uint8_t buf[2] = { (uint8_t)(command >> 8), (uint8_t)(command & 0xFF) };
  return i2cWrite(addr, buf, 2);
}

static bool scd41ReadWords(uint8_t addr, uint16_t command, uint16_t *words, int count)
{
  if (!scd41Command(addr, command)) return false;
  delay(2);

  const int bytes = count * 3;   // each word carries its own CRC
  if (Wire.requestFrom((int)addr, bytes) != bytes) return false;

  for (int i = 0; i < count; i++)
  {
    uint8_t chunk[3];
    chunk[0] = Wire.read();
    chunk[1] = Wire.read();
    chunk[2] = Wire.read();

    if (scd41Crc8(chunk, 2) != chunk[2]) return false;
    words[i] = ((uint16_t)chunk[0] << 8) | chunk[1];
  }

  return true;
}

static bool scd41Begin(uint8_t addr)
{
  // Sensirion's clean start: wake, stop, reinit, start. wake_up is not ACKed,
  // so it goes out best-effort.
  scd41Command(addr, SCD41_CMD_WAKE_UP);
  delay(30);

  scd41Command(addr, SCD41_CMD_STOP_PERIODIC);
  delay(500);   // other commands are only accepted 500 ms after a stop

  scd41Command(addr, SCD41_CMD_REINIT);
  delay(30);

  if (!scd41Command(addr, SCD41_CMD_START_PERIODIC))
  {
    Serial.println("[SCD41] start command not acknowledged");
    return false;
  }

  scd41NextPollMs = millis() + 5000;
  Serial.println("[SCD41] init OK; the first reading takes about 5 s");
  return true;
}

static int scd41Read(NodeValue *out, int maxValues)
{
  if (maxValues < 3) return 0;

  const unsigned long now = millis();
  if ((int32_t)(now - scd41NextPollMs) < 0) return 0;

  uint16_t ready = 0;
  if (!scd41ReadWords(detectedAddress, SCD41_CMD_GET_DATA_READY, &ready, 1))
  {
    scd41NextPollMs = now + 500;
    return 0;
  }

  // The low 11 bits are zero while the next measurement is still cooking.
  if ((ready & 0x07FF) == 0)
  {
    scd41NextPollMs = now + 400;
    return 0;
  }

  uint16_t words[3] = { 0, 0, 0 };
  if (!scd41ReadWords(detectedAddress, SCD41_CMD_READ_MEASUREMENT, words, 3))
  {
    scd41NextPollMs = now + 500;
    return 0;
  }

  if (words[0] == 0) return 0;   // a zero ppm reading is the sensor not ready

  out[0].value = (float)words[0];
  out[0].name = "이산화탄소";
  out[0].unit = "ppm";

  out[1].value = -45.0f + 175.0f * ((float)words[1] / 65535.0f);
  out[1].name = "온도";
  out[1].unit = "C";

  out[2].value = 100.0f * ((float)words[2] / 65535.0f);
  out[2].name = "습도";
  out[2].unit = "%";

  scd41NextPollMs = now + 4500;
  return 3;
}

// ------------------------------------------------------------ TSL2591 ------

#define TSL2591_COMMAND_BIT 0xA0
#define TSL2591_REG_ENABLE 0x00
#define TSL2591_REG_CONTROL 0x01
#define TSL2591_REG_ID 0x12
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

static bool tsl2591Read16(uint8_t reg, uint16_t *out)
{
  uint8_t buf[2];
  if (!i2cReadRegisters(detectedAddress, (uint8_t)(TSL2591_COMMAND_BIT | reg), buf, 2)) return false;
  *out = ((uint16_t)buf[1] << 8) | buf[0];   // little-endian, unlike the rest
  return true;
}

static bool tsl2591Identify(uint8_t addr)
{
  uint8_t id = 0;
  if (!i2cReadRegister8(addr, (uint8_t)(TSL2591_COMMAND_BIT | TSL2591_REG_ID), &id)) return false;
  return (id & 0xF0) == 0x50;
}

static bool tsl2591Begin(uint8_t addr)
{
  if (!tsl2591Identify(addr)) return false;

  if (!i2cWriteRegister8(addr, (uint8_t)(TSL2591_COMMAND_BIT | TSL2591_REG_CONTROL),
                         TSL2591_GAIN_MEDIUM | TSL2591_INTEGRATION_200MS)) return false;

  if (!i2cWriteRegister8(addr, (uint8_t)(TSL2591_COMMAND_BIT | TSL2591_REG_ENABLE),
                         TSL2591_ENABLE_PON | TSL2591_ENABLE_AEN)) return false;

  delay(230);
  Serial.println("[TSL2591] init OK");
  return true;
}

static int tsl2591Read(NodeValue *out, int maxValues)
{
  if (maxValues < 1) return 0;

  uint16_t channel0 = 0;
  uint16_t channel1 = 0;

  if (!tsl2591Read16(TSL2591_REG_CHAN0_LOW, &channel0)) return 0;
  if (!tsl2591Read16(TSL2591_REG_CHAN1_LOW, &channel1)) return 0;

  if (channel0 == 0xFFFF || channel1 == 0xFFFF) return 0;   // saturated

  const float cpl = (TSL2591_ATIME_MS * TSL2591_AGAIN) / TSL2591_LUX_DF;
  if (cpl <= 0.0f) return 0;

  const float lux1 = ((float)channel0 - TSL2591_LUX_COEFB * (float)channel1) / cpl;
  const float lux2 = (TSL2591_LUX_COEFC * (float)channel0 - TSL2591_LUX_COEFD * (float)channel1) / cpl;
  float lux = lux1 > lux2 ? lux1 : lux2;

  if (lux < 0.0f) lux = 0.0f;
  if (isnan(lux) || isinf(lux)) return 0;

  out[0].value = lux;
  out[0].name = "조도";
  out[0].unit = "lx";
  return 1;
}

// ------------------------------------------------------------ VL53L1X ------

#define VL53L1X_REG_SOFT_RESET             0x0000
#define VL53L1X_REG_GPIO_TIO_HV_STATUS     0x0031
#define VL53L1X_REG_INTERRUPT_CLEAR        0x0086
#define VL53L1X_REG_MODE_START             0x0087
#define VL53L1X_REG_RANGE_STATUS           0x0089
#define VL53L1X_REG_RANGE_MM               0x0096
#define VL53L1X_REG_FIRMWARE_SYSTEM_STATUS 0x00E5
#define VL53L1X_REG_MODEL_ID               0x010F
#define VL53L1X_EXPECTED_MODEL_ID          0xEACC

// ST's compact ULD default configuration, written from register 0x002D.
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

static bool vl53Read16(uint16_t reg, uint16_t *out)
{
  uint8_t buf[2];
  if (!i2cRead16Addr(detectedAddress, reg, buf, 2)) return false;
  *out = ((uint16_t)buf[0] << 8) | buf[1];
  return true;
}

static bool vl53Identify(uint8_t addr)
{
  uint8_t buf[2];
  if (!i2cRead16Addr(addr, VL53L1X_REG_MODEL_ID, buf, 2)) return false;
  return (((uint16_t)buf[0] << 8) | buf[1]) == VL53L1X_EXPECTED_MODEL_ID;
}

static bool vl53Begin(uint8_t addr)
{
  if (!i2cWrite16Addr8(addr, VL53L1X_REG_SOFT_RESET, 0x00)) return false;
  delay(100);
  if (!i2cWrite16Addr8(addr, VL53L1X_REG_SOFT_RESET, 0x01)) return false;

  const unsigned long start = millis();
  uint8_t systemStatus = 0;

  while (millis() - start < 1000)
  {
    uint8_t buf[1];
    if (i2cRead16Addr(addr, VL53L1X_REG_FIRMWARE_SYSTEM_STATUS, buf, 1))
    {
      systemStatus = buf[0];
      if (systemStatus & 0x01) break;
    }
    delay(10);
  }

  if (!(systemStatus & 0x01))
  {
    Serial.println("[VL53L1X] boot timeout");
    return false;
  }

  if (!vl53Identify(addr))
  {
    Serial.println("[VL53L1X] unexpected model ID");
    return false;
  }

  for (size_t i = 0; i < sizeof(VL53L1X_DEFAULT_CONFIGURATION); i++)
  {
    if (!i2cWrite16Addr8(addr, (uint16_t)(0x002D + i), VL53L1X_DEFAULT_CONFIGURATION[i]))
    {
      Serial.println("[VL53L1X] configuration write failed");
      return false;
    }
  }

  if (!i2cWrite16Addr8(addr, VL53L1X_REG_MODE_START, 0x40)) return false;

  Serial.println("[VL53L1X] init OK");
  return true;
}

static int vl53Read(NodeValue *out, int maxValues)
{
  if (maxValues < 1) return 0;

  uint8_t status[1];
  if (!i2cRead16Addr(detectedAddress, VL53L1X_REG_GPIO_TIO_HV_STATUS, status, 1)) return 0;
  if ((status[0] & 0x01) != 0) return 0;   // interrupt not raised: no new range

  uint8_t rangeStatus[1];
  uint16_t rangeMm = 0;

  if (!i2cRead16Addr(detectedAddress, VL53L1X_REG_RANGE_STATUS, rangeStatus, 1)) return 0;
  if (!vl53Read16(VL53L1X_REG_RANGE_MM, &rangeMm)) return 0;

  // Release this measurement so the next one can run, whatever we do with it.
  i2cWrite16Addr8(detectedAddress, VL53L1X_REG_INTERRUPT_CLEAR, 0x01);

  const uint8_t raw = (uint8_t)(rangeStatus[0] & 0x1F);

  // 9 is a normal valid range; 8 is clipped at the near end but still usable
  // for a classroom demonstration.
  if (raw != 9 && raw != 8) return 0;

  out[0].value = (float)rangeMm;
  out[0].name = "거리";
  out[0].unit = "mm";
  return 1;
}

// ------------------------------------------------------------- INA228 ------

#define INA228_REG_CONFIG          0x00
#define INA228_REG_ADC_CONFIG      0x01
#define INA228_REG_SHUNT_CAL       0x02
#define INA228_REG_VBUS            0x05
#define INA228_REG_CURRENT         0x07
#define INA228_REG_POWER           0x08
#define INA228_REG_MANUFACTURER_ID 0x3E
#define INA228_MANUFACTURER_TI     0x5449

// The resistor on the breakout. Bus voltage does not depend on it; a current
// that is wrong by a fixed ratio does.
#define INA228_SHUNT_OHMS   0.015f
#define INA228_MAX_CURRENT  10.0f
#define INA228_VBUS_LSB_V   0.0001953125f
#define INA228_POWER_LSB_K  3.2f

static float ina228CurrentLsb = 0.0f;

static bool ina228Write16(uint8_t addr, uint8_t reg, uint16_t value)
{
  const uint8_t buf[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
  return i2cWrite(addr, buf, 3);
}

static bool ina228Read24(uint8_t reg, uint32_t *out)
{
  uint8_t buf[3];
  if (!i2cReadRegisters(detectedAddress, reg, buf, 3)) return false;
  *out = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
  return true;
}

static bool ina228Begin(uint8_t addr)
{
  uint8_t idBuf[2];
  if (!i2cReadRegisters(addr, INA228_REG_MANUFACTURER_ID, idBuf, 2)) return false;

  const uint16_t manufacturer = ((uint16_t)idBuf[0] << 8) | idBuf[1];
  if (manufacturer != INA228_MANUFACTURER_TI)
  {
    Serial.printf("[INA228] unexpected manufacturer id 0x%04X\n", manufacturer);
    return false;
  }

  ina228Write16(addr, INA228_REG_CONFIG, 0x8000);
  delay(5);

  if (!ina228Write16(addr, INA228_REG_ADC_CONFIG, 0xFB6A)) return false;

  ina228CurrentLsb = INA228_MAX_CURRENT / 524288.0f;
  const uint16_t shuntCal = (uint16_t)(13107.2e6f * ina228CurrentLsb * INA228_SHUNT_OHMS + 0.5f);

  if (!ina228Write16(addr, INA228_REG_SHUNT_CAL, shuntCal)) return false;

  Serial.printf("[INA228] ready: shunt %.3f ohm, max %.1f A\n",
                INA228_SHUNT_OHMS, INA228_MAX_CURRENT);
  return true;
}

static int ina228Read(NodeValue *out, int maxValues)
{
  if (maxValues < 3) return 0;

  uint32_t rawVbus = 0;
  uint32_t rawCurrent = 0;
  uint32_t rawPower = 0;

  if (!ina228Read24(INA228_REG_VBUS, &rawVbus)) return 0;
  if (!ina228Read24(INA228_REG_CURRENT, &rawCurrent)) return 0;
  if (!ina228Read24(INA228_REG_POWER, &rawPower)) return 0;

  int32_t current20 = (int32_t)(rawCurrent >> 4);
  if (current20 & 0x00080000) current20 -= 0x00100000;   // sign-extend from 20 bits

  out[0].value = (float)current20 * ina228CurrentLsb;
  out[0].name = "전류";
  out[0].unit = "A";

  out[1].value = (float)(rawVbus >> 4) * INA228_VBUS_LSB_V;
  out[1].name = "전압";
  out[1].unit = "V";

  out[2].value = (float)rawPower * INA228_POWER_LSB_K * ina228CurrentLsb;
  out[2].name = "전력";
  out[2].unit = "W";

  return 3;
}

// ------------------------------------------------------------ DS18B20 ------
//
// 1-Wire, ported from the P4 with its timings unchanged. Every slot is timed
// in microseconds with interrupts off, because a late edge is read as the
// wrong bit.
//
// Conversion takes up to 750 ms at 12-bit resolution, which is most of the
// node's one-second cycle, so a reading is started on one pass and collected
// on the next rather than blocking the loop.

// The datasheet allows 750 ms for a 12-bit conversion. Waiting the full second
// the P4 uses put this a hair over the node's own one-second cycle, so every
// other tick had no reading and the probe reported half as often as it could.
#define DS18B20_CONVERT_MS 800

static bool dsConversionStarted = false;
static unsigned long dsConversionStartMs = 0;

// Which pin the probe is actually on. ONE_WIRE_PIN is only where the search
// starts: a three-wire probe gets soldered wherever there was room, and the
// part announces itself on any pin, so there is no reason to insist on one.
static uint8_t oneWirePin = ONE_WIRE_PIN;

// The line is held in open-drain the whole time it is being talked to, so
// driving it and letting go are both a single digitalWrite.
//
// This is the difference between the P4's copy of this driver and this one.
// There the pin is switched with ESP-IDF calls that take a few hundred
// nanoseconds; pinMode() on Arduino reconfigures the IO MUX and costs
// microseconds. A read slot has to release the line within 15 us of pulling it
// down, and pinMode() inside that window overruns it - the part then reads the
// master's timing as a write instead of a read and says nothing, which comes
// back as FF for every byte. A 520 us reset pulse has room to absorb the same
// overhead, which is why presence worked while nothing could be read.
static inline void dsOwBusIdle()
{
  pinMode(oneWirePin, OUTPUT_OPEN_DRAIN);
  digitalWrite(oneWirePin, HIGH);

  // Open drain on its own leaves an unconnected pin floating, and a floating
  // pin reads back whatever it last held - which is how an empty GPIO4 came to
  // return a scratchpad of zeros. The weak internal pull-up puts an empty pin
  // firmly high again; a real line has its own 4.7k and does not notice.
  gpio_pullup_en((gpio_num_t)oneWirePin);
}

static inline void dsOwRelease()
{
  digitalWrite(oneWirePin, HIGH);
}

static inline void dsOwLow()
{
  digitalWrite(oneWirePin, LOW);
}

static inline int dsOwReadLevel()
{
  return digitalRead(oneWirePin);
}

// Drives the line high from the pin itself, instead of leaving it to a
// resistor.
//
// The ESP32's internal pull-up is about 45k. That is enough to hold an idle
// line high, but not to power a DS18B20 wired without its VCC - in parasite
// mode the part draws its supply from the data line during conversion, and on
// 45k it never charges enough to answer, which reads back as FF for every
// byte. The datasheet's own answer is a strong pull-up held across the
// conversion, which is what this is.
//
// Only safe while nothing else is driving: between transactions, and during
// the conversion wait. Never during a slot the part might pull low.
static inline void dsOwStrongPullup()
{
  digitalWrite(oneWirePin, HIGH);
  pinMode(oneWirePin, OUTPUT);   // push-pull: outside any timed slot
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
      const uint8_t mix = (crc ^ inByte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      inByte >>= 1;
    }
  }

  return crc;
}

static bool dsStartConversion()
{
  if (!dsOwReset()) return false;

  dsOwWriteByte(0xCC);   // Skip ROM: one part on the line
  dsOwWriteByte(0x44);   // Convert T

  // Hold the line up for the whole conversion. A part with its VCC connected
  // ignores this; one running on parasite power needs it.
  dsOwStrongPullup();

  dsConversionStarted = true;
  dsConversionStartMs = millis();
  return true;
}

// Pins on a classic ESP32 dev board that can drive a 1-Wire line.
//
// GPIO2 is in the list despite being a strapping pin: the P4 runs its own
// DS18B20 on GPIO2, so anyone wiring a node to match will have used it, and a
// 1-Wire line idles high, which is the level GPIO2 needs at boot anyway.
//
// Still out: 0 is the boot button, 12 picks the flash voltage and reads the
// wrong thing with a pull-up on it, 1 and 3 are the console, 6-11 are the
// flash, and 34-39 are input only so they cannot pull the line down at all.
static const uint8_t kOneWireCandidates[] = {
  4, 2, 5, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33
};

// What the last probe saw, for the report when nothing is found.
static int dsLastIdleLevel = -1;
static bool dsLastPresence = false;

// A CRC is not enough on its own: eight zero bytes have a CRC of zero, so a
// line stuck low passes, and so does one stuck high. Those two patterns are
// rejected by name.
//
// The reserved bytes are deliberately not checked. The datasheet gives 0xFF
// for byte 5 and 0x10 for byte 7, and the part on this bench answers A5 and
// 66 with a CRC that verifies - plenty of DS18B20s in circulation are clones
// that fill the reserved bytes differently. Insisting on the datasheet values
// threw away a perfectly good 25.0 C reading.
static bool dsScratchpadLooksReal(const uint8_t *data)
{
  if (dsCrc8Dallas(data, 8) != data[8]) return false;

  bool allZero = true;
  bool allOnes = true;

  for (int i = 0; i < 9; i++)
  {
    if (data[i] != 0x00) allZero = false;
    if (data[i] != 0xFF) allOnes = false;
  }

  return !allZero && !allOnes;
}

static bool dsProbePin(uint8_t pin)
{
  oneWirePin = pin;
  dsLastIdleLevel = -1;
  dsLastPresence = false;

  // Handing the pad back from the I2C peripheral has to happen before the
  // line is driven, not after a candidate is chosen: the peripheral fights the
  // bit-banging hard enough to corrupt every byte.
  if (pin == I2C_SDA_PIN || pin == I2C_SCL_PIN) Wire.end();

  // Charge a parasite-powered part before asking it anything, or its first
  // answer is the one that fails.
  dsOwStrongPullup();
  delay(10);
  dsOwBusIdle();
  delayMicroseconds(10);

  // An empty pin idles high on the pull-up; one shorted to ground never rises.
  dsLastIdleLevel = dsOwReadLevel();
  if (dsLastIdleLevel == 0) return false;

  dsLastPresence = dsOwReset();
  if (!dsLastPresence) return false;

  // A presence pulse on its own is not proof. A long probe cable on a weak
  // internal pull-up can read low at the moment presence is sampled, which is
  // how an empty pin came to be adopted and then returned FF FF FF for every
  // byte. Read the scratchpad instead: a DS18B20 has valid power-on contents
  // with a good CRC before any conversion has been asked for, so this settles
  // it in microseconds rather than waiting 750 ms for a measurement.
  dsOwWriteByte(0xCC);
  dsOwWriteByte(0xBE);

  uint8_t data[9];
  for (uint8_t i = 0; i < 9; i++) data[i] = dsOwReadByte();

  if (dsScratchpadLooksReal(data)) return true;

  // A pin that answered a reset and then returned something unreadable is the
  // interesting case: the part is there, and what came back says whether the
  // problem is the line or the timing.
  Serial.printf("[DS18B20] GPIO%-2d answered but read %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                pin, data[0], data[1], data[2], data[3], data[4],
                data[5], data[6], data[7], data[8]);
  return false;
}

static bool ds18b20Begin()
{
  dsConversionStarted = false;
  dsOwBusIdle();

  if (!dsProbePin(oneWirePin))
  {
    bool found = false;

    for (size_t i = 0; i < sizeof(kOneWireCandidates) / sizeof(kOneWireCandidates[0]); i++)
    {
      if (kOneWireCandidates[i] == ONE_WIRE_PIN) continue;   // already tried

      if (dsProbePin(kOneWireCandidates[i]))
      {
        found = true;
        break;
      }

      // Presence without a readable scratchpad is a different fault from
      // silence, and the difference is what says whether the probe is on this
      // pin at all.
      Serial.printf("[DS18B20] GPIO%-2d idle=%d presence=%d\n",
                    kOneWireCandidates[i], dsLastIdleLevel, dsLastPresence ? 1 : 0);
    }

    if (!found)
    {
      oneWirePin = ONE_WIRE_PIN;
      // Put the bus back: a probe that is not there must not cost the node
      // its I2C sensors.
      Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
      Wire.setClock(100000);

      // Naming the pins that were tried turns "it does not work" into
      // something a teacher can check against the board in front of them.
      Serial.print("[DS18B20] no probe answered. Tried GPIO");
      for (size_t i = 0; i < sizeof(kOneWireCandidates) / sizeof(kOneWireCandidates[0]); i++)
      {
        Serial.printf("%s%d", i ? "," : " ", kOneWireCandidates[i]);
      }
      Serial.println();
      Serial.println("[DS18B20] check: data line on one of those pins, VCC on 3.3V, "
                     "and a 4.7k resistor between data and 3.3V. GPIO34-39 cannot "
                     "work - they are input only.");
      return false;
    }
  }

  if (oneWirePin == I2C_SDA_PIN || oneWirePin == I2C_SCL_PIN)
  {
    Serial.println("[DS18B20] shares a pin with I2C; the bus has been released to it");
  }

  Serial.printf("[DS18B20] answering on GPIO%d\n", oneWirePin);
  return dsStartConversion();
}

static int ds18b20Read(NodeValue *out, int maxValues)
{
  if (maxValues < 1) return 0;

  if (!dsConversionStarted)
  {
    dsStartConversion();
    return 0;
  }

  if (millis() - dsConversionStartMs < DS18B20_CONVERT_MS) return 0;

  // Out of push-pull and back to open drain before the part is asked to answer.
  dsOwBusIdle();
  delayMicroseconds(10);

  if (!dsOwReset())
  {
    dsConversionStarted = false;
    return 0;
  }

  dsOwWriteByte(0xCC);
  dsOwWriteByte(0xBE);   // Read scratchpad

  uint8_t data[9];
  for (uint8_t i = 0; i < 9; i++) data[i] = dsOwReadByte();

  if (!dsScratchpadLooksReal(data))
  {
    // All ones is an open data line - no pull-up, or nothing on the other end.
    // Anything else is a line that answers but is being read at the wrong
    // moment.
    Serial.printf("[DS18B20] bad CRC: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                  data[0], data[1], data[2], data[3], data[4],
                  data[5], data[6], data[7], data[8]);
    dsStartConversion();
    return 0;
  }

  const float t = (float)(int16_t)((data[1] << 8) | data[0]) / 16.0f;

  if (t < -55.0f || t > 125.0f)
  {
    dsStartConversion();
    return 0;
  }

  out[0].value = t;
  out[0].name = "수온";
  out[0].unit = "C";

  dsStartConversion();
  return 1;
}

// ------------------------------------------------------------ dispatch -----

bool nodeSensorDetect()
{
  detectedKind = NODE_SENSOR_NONE;
  detectedAddress = 0x00;

  struct Candidate
  {
    uint8_t address;
    NodeSensorKind kind;
  };

  // 0x29 is deliberately listed twice: the TSL2591 and the VL53L1X share it,
  // and each begin() checks its own ID register, so whichever answers wins.
  const Candidate candidates[] = {
    { 0x77, NODE_SENSOR_DPS310 },
    { 0x76, NODE_SENSOR_DPS310 },
    { 0x48, NODE_SENSOR_TMP117 },
    { 0x62, NODE_SENSOR_SCD41 },
    { 0x40, NODE_SENSOR_INA228 },
    { 0x29, NODE_SENSOR_TSL2591 },
    { 0x29, NODE_SENSOR_VL53L1X }
  };

  for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++)
  {
    const uint8_t addr = candidates[i].address;

    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) continue;

    bool ok = false;

    switch (candidates[i].kind)
    {
      case NODE_SENSOR_DPS310:  ok = dps310Begin(addr); break;
      case NODE_SENSOR_TMP117:  ok = tmp117Begin(addr); break;
      case NODE_SENSOR_SCD41:   ok = scd41Begin(addr); break;
      case NODE_SENSOR_INA228:  ok = ina228Begin(addr); break;
      case NODE_SENSOR_TSL2591: ok = tsl2591Begin(addr); break;
      case NODE_SENSOR_VL53L1X: ok = vl53Begin(addr); break;
      default: break;
    }

    if (!ok) continue;

    detectedKind = candidates[i].kind;
    detectedAddress = addr;
    Serial.printf("[SENSOR] using %s at 0x%02X\n", nodeSensorName(), addr);
    return true;
  }

  // Nothing on I2C. The DS18B20 is 1-Wire and answers a reset pulse instead of
  // an address, so it is looked for separately rather than being missed.
  if (ds18b20Begin())
  {
    detectedKind = NODE_SENSOR_DS18B20;
    detectedAddress = 0x00;
    Serial.printf("[SENSOR] using %s on GPIO%d\n", nodeSensorName(), oneWirePin);
    return true;
  }

  return false;
}

NodeSensorKind nodeSensorKind()
{
  return detectedKind;
}

const char *nodeSensorName()
{
  switch (detectedKind)
  {
    case NODE_SENSOR_DPS310:  return "DPS310 온도·기압";
    case NODE_SENSOR_TMP117:  return "TMP117 정밀온도";
    case NODE_SENSOR_SCD41:   return "SCD41 이산화탄소";
    case NODE_SENSOR_TSL2591: return "TSL2591 조도";
    case NODE_SENSOR_VL53L1X: return "VL53L1X 거리";
    case NODE_SENSOR_INA228:  return "INA228 전압·전류";
    case NODE_SENSOR_DS18B20: return "DS18B20 수온";
    default: return "없음";
  }
}

int nodeSensorRead(NodeValue *out, int maxValues)
{
  if (out == NULL || detectedKind == NODE_SENSOR_NONE) return 0;

  int count = 0;

  switch (detectedKind)
  {
    case NODE_SENSOR_DPS310:  count = dps310Read(out, maxValues); break;
    case NODE_SENSOR_TMP117:  count = tmp117Read(out, maxValues); break;
    case NODE_SENSOR_SCD41:   count = scd41Read(out, maxValues); break;
    case NODE_SENSOR_TSL2591: count = tsl2591Read(out, maxValues); break;
    case NODE_SENSOR_VL53L1X: count = vl53Read(out, maxValues); break;
    case NODE_SENSOR_INA228:  count = ina228Read(out, maxValues); break;
    case NODE_SENSOR_DS18B20: count = ds18b20Read(out, maxValues); break;
    default: break;
  }

  if (count > 0)
  {
    missingReads = 0;
    return count;
  }

  // Zero is not the same as gone. An SCD41 warming up returns nothing for
  // five seconds and is still very much on the bus, and a DS18B20 says nothing
  // for the first second of every conversion. Ask the part directly: one that
  // still answers is simply not ready.
  bool stillThere = false;

  if (detectedKind == NODE_SENSOR_DS18B20)
  {
    stillThere = dsOwReset();
  }
  else
  {
    Wire.beginTransmission(detectedAddress);
    stillThere = Wire.endTransmission() == 0;
  }

  if (stillThere)
  {
    missingReads = 0;
    return 0;
  }

  // It has stopped acknowledging - unplugged, or swapped for something else.
  // Forget it and look again, so changing a sensor while the node runs works
  // the same as changing it before boot.
  if (++missingReads >= 3)
  {
    Serial.printf("[SENSOR] %s stopped answering; looking again\n", nodeSensorName());
    missingReads = 0;
    detectedKind = NODE_SENSOR_NONE;
    detectedAddress = 0x00;
    nodeSensorDetect();
  }

  return 0;
}
