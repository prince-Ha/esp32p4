// =====================================================
// 노드가 붙은 센서를 스스로 알아봅니다
//
// One driver per part the board supports, chosen by what answers on the I2C
// bus. Ported from the P4's own drivers - the same register sequences and the
// same compensation - so a part reads identically whether it hangs off a node
// or off the board itself.
//
// 0x29 is shared by the TSL2591 and the VL53L1X, so that address is settled by
// reading each one's ID register rather than by the address alone.
// =====================================================

#pragma once

#include <Arduino.h>
#include <Wire.h>

#define NODE_MAX_VALUES 3

typedef struct
{
  float value;
  const char *name;
  const char *unit;
} NodeValue;

typedef enum
{
  NODE_SENSOR_NONE = 0,
  NODE_SENSOR_DPS310,
  NODE_SENSOR_TMP117,
  NODE_SENSOR_SCD41,
  NODE_SENSOR_TSL2591,
  NODE_SENSOR_VL53L1X,
  NODE_SENSOR_INA228
} NodeSensorKind;

// Walks the bus, identifies what is there and initialises it. Safe to call
// repeatedly: a sensor plugged in after boot is picked up on a later call.
bool nodeSensorDetect();

// What was found, and its Korean name for the log.
NodeSensorKind nodeSensorKind();
const char *nodeSensorName();

// Fills in up to `maxValues` quantities and returns how many. Zero means
// nothing usable this second - a sensor still warming up, or one that has
// stopped answering.
int nodeSensorRead(NodeValue *out, int maxValues);

// Lists every address that answers. Called once at startup; a part that does
// not appear here is a wiring or power fault.
int nodeScanI2cBus();
