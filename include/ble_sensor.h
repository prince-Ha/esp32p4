#pragma once

#include <stdbool.h>
#include <stdint.h>

// BLE peripheral that publishes the current reading to a tablet or PC.
//
// The ESP32-P4 has no radio of its own; Bluetooth runs over ESP-Hosted to the
// companion ESP32-C6, the same link WiFi uses. Call bleSensorBegin() only
// after ESP-Hosted is up.
//
// Arduino's BLEDevice wrapper is declared `architectures=esp32` and is not
// built for the P4, so this sits directly on the NimBLE C API.

#ifdef __cplusplus
extern "C" {
#endif

// Brings up the NimBLE host and starts advertising. Safe to call more than
// once; later calls are ignored. Returns false if Bluetooth is unavailable in
// this build.
bool bleSensorBegin(const char *deviceName);

// True once a central has connected.
bool bleSensorIsConnected();

// True when a central has subscribed to measurement notifications.
bool bleSensorIsSubscribed();

// Publishes one reading. Formats as a single UTF-8 line:
//   "<time_s>,<sensor>,<value>,<unit>"
// Cheap and a no-op when nothing is subscribed.
void bleSensorPublish(uint32_t timeS, const char *sensorName, float value, const char *unit);

// Human-readable state for the UI: "꺼짐", "대기 중", "연결됨", "전송 중".
const char *bleSensorStateText();

#ifdef __cplusplus
}
#endif
