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

// The name this board advertises under.
const char *bleAdvertisedName();

// Human-readable state for the UI: "꺼짐", "대기 중", "연결됨", "전송 중".
const char *bleSensorStateText();

// =====================================================
// Central role: finding nearby sensor peripherals
//
// The board is also the hub. Before it can subscribe to another sensor board
// it has to know what that board advertises, so scanning comes first and
// reports every service UUID it sees — that is what identifies a commercial
// MBL sensor whose protocol is not documented here.
// =====================================================

#define BLE_SCAN_MAX_RESULTS 16

typedef struct
{
  char name[32];       // advertised name, or "" when the device is unnamed
  char address[18];    // "aa:bb:cc:dd:ee:ff"
  char services[64];   // advertised service UUIDs, comma separated
  int rssi;
} BleScanResult;

// Starts a scan for `durationMs`. Results accumulate until the next scan.
bool bleScanStart(uint32_t durationMs);
bool bleScanIsRunning();

// Number of distinct devices seen in the most recent scan.
int bleScanResultCount();

// Copies one result out. Returns false when `index` is out of range.
bool bleScanResultAt(int index, BleScanResult *out);

// A device worth showing: it advertises a name or a service UUID. Anything
// else is a phone or laptop advertising anonymously and cannot be identified.
bool bleScanResultIsCandidate(const BleScanResult *r);
int bleScanCandidateCount();

// Prints the current results over Serial, one device per line.
void bleScanDumpResults();

// =====================================================
// Central role: linking to one sensor node
//
// Scanning only says what is nearby. Linking connects to one of those devices,
// finds the measurement characteristic and subscribes to it, after which its
// readings arrive as notifications and are treated like any other sensor on
// this board.
// =====================================================

// Connects to `address` ("aa:bb:cc:dd:ee:ff", as printed by the scan) and
// subscribes to its measurement characteristic. Asynchronous: this returns as
// soon as the attempt starts, and bleLinkStateText() reports progress.
bool bleLinkConnect(const char *address);

// Drops the link. Safe when nothing is connected.
void bleLinkDisconnect();

// True once the subscription is in place and readings are arriving.
bool bleLinkIsSubscribed();

// True from the moment a connection attempt starts until it fails or drops.
bool bleLinkIsBusy();

// The name of the node this board is linked to, or "" when there is none.
const char *bleLinkPeerName();

// Progress for the UI: "연결 안 됨", "연결 중", "서비스 검색 중", "구독 중",
// "수신 중", or a failure reason.
const char *bleLinkStateText();

// The most recent reading, unpacked from "<time_s>,<sensor>,<value>,<unit>".
// `sensorName` and `unit` must have room for 24 bytes each. Returns false when
// nothing has arrived yet. `ageMs` is how long ago it landed, which is what
// tells a stalled node from a slow one.
bool bleLinkLatestReading(float *value, char *sensorName, char *unit, uint32_t *ageMs);

// Links to the strongest node in the last scan that advertises this project's
// service, so a hub that is power-cycled mid-lesson comes back talking to its
// node instead of waiting for someone to tap the list. Returns false when no
// such node was seen.
bool bleAutoLinkToSensorNode();

#ifdef __cplusplus
}
#endif
