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
// Central role: linking to sensor nodes
//
// The board is the hub, and a lesson can have more than one node in the room:
// one group measuring pressure, another temperature. The controller holds
// three connections at once (CONFIG_BT_NIMBLE_MAX_CONNECTIONS=3), and the P4
// keeps advertising as a peripheral as well, so a phone connecting to the
// board takes one of those three.
//
// Scanning only says what is nearby. Linking connects to one of those devices,
// finds the measurement characteristic and subscribes to it, after which its
// readings arrive as notifications.
// =====================================================

// Connections the controller can hold at once.
#define BLE_LINK_MAX_NODES 3

// The most quantities one node may report in a single packet. The P4 shows
// three values at once, so more would have nowhere to go.
#define BLE_LINK_MAX_VALUES 3

// Connects to `address` ("aa:bb:cc:dd:ee:ff", as printed by the scan) using
// the first free slot. Asynchronous: this returns as soon as the attempt
// starts, and the slot's state text reports progress. An address that already
// holds a slot reuses it rather than taking a second one.
bool bleLinkConnect(const char *address);

// Drops one link, or all of them. Safe when nothing is connected.
void bleLinkDisconnectSlot(int slot);
void bleLinkDisconnect();

// Slots that are connected and receiving, and the first of them. The slot
// finder returns -1 when nothing is linked.
int bleLinkNodeCount();
int bleLinkFirstSubscribedSlot();

bool bleLinkSlotIsSubscribed(int slot);

// True from the moment a connection attempt starts until it fails or drops.
bool bleLinkSlotIsBusy(int slot);
bool bleLinkIsBusy();

// The slot holding `address`, or -1.
int bleLinkSlotForAddress(const char *address);

// The node's advertised name, or its address when it advertised none. Empty
// for a free slot.
const char *bleLinkSlotName(int slot);

// Progress for the UI: "연결 안 됨", "연결 중", "MTU 협상 중", "서비스 검색 중",
// "구독 중", "수신 중", or a failure reason.
const char *bleLinkSlotState(int slot);

// How many complete (name, value, unit) groups the slot's last packet carried.
// Zero when nothing has arrived, or when the packet was clipped mid-group.
int bleLinkValueCount(int slot);

// One of those groups. `name` and `unit` must have room for 24 bytes each.
// `ageMs` is how long ago the packet landed, which is what tells a stalled
// node from a slow one.
bool bleLinkValueAt(int slot, int index, float *value, char *name, char *unit, uint32_t *ageMs);

// Connects to a device that is not one of this project's nodes and reports
// what it exposes: every service, every characteristic and its properties, and
// then the raw bytes of anything that will notify.
//
// This exists because a commercial sensor's protocol cannot be guessed. A
// wrong UUID gives a link that never delivers; a wrong byte layout gives
// numbers that look plausible and are not, which is worse. The device is asked
// instead.
bool bleLinkExplore(const char *address);

// True while a slot is being explored rather than read.
bool bleLinkSlotIsExploring(int slot);

// A one-line summary of what the exploration has found so far, for the screen.
const char *bleLinkExploreSummary(int slot);

// Links to every node in the last scan that advertises this project's service
// and is not already connected, so a hub power-cycled mid-lesson comes back
// talking to the room rather than waiting for someone to tap a list. Returns
// how many attempts it started.
int bleAutoLinkToSensorNode();

#ifdef __cplusplus
}
#endif
