#pragma once

/* SunSeek Platform v2.0 — SYSTEM LAYER */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "Config_System.h"

/*
  Communication.h — T02 v1.6 / GS v1.8 Compatibility

  Transport contract
    USB = Development Link
    BLE = TT&C / Control Plane

  Message framing
    One UTF-8 ASCII message per line, terminated by \n.

  Notes
    - The BLE Nordic-UART-style UUIDs are intentionally unchanged so the
      Ground Station does not need a transport-layer change.
    - Preferred ATT MTU is raised to reduce failures with commands such as
      ADCS_STRATEGY,MOMENTUM and future T03-T07 messages.
    - RX is line-buffered so newline-framed commands remain deterministic.
*/

#define TTC_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define TTC_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define TTC_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define TTC_PREFERRED_MTU 185
#define TTC_MAX_RX_LINE   240

typedef void (*TelecommandHandler)(String command);

static BLEServer *_ttcServer = nullptr;
static BLECharacteristic *_ttcTx = nullptr;
static bool _ttcConnected = false;
static bool _ttcOldConnected = false;
static String _ttcSerialBuffer = "";
static String _ttcBleBuffer = "";
static TelecommandHandler _ttcHandler = nullptr;

inline String getSpacecraftID() {
  return String("SUNSEEK-") + TEAM_NAME;
}

inline void sendTelemetry(const String &text) {
  // Every emitted message is also visible on the USB Development Link.
  Serial.println(text);

  if (_ttcConnected && _ttcTx) {
    String packet = text + "\n";
    _ttcTx->setValue((uint8_t*)packet.c_str(), packet.length());
    _ttcTx->notify();
    delay(3);
  }
}

inline void _ttcDispatchBufferedLines(String &buffer) {
  int newlineIndex = buffer.indexOf('\n');

  while (newlineIndex >= 0) {
    String command = buffer.substring(0, newlineIndex);
    buffer.remove(0, newlineIndex + 1);
    command.trim();

    if (command.length() && _ttcHandler) {
      _ttcHandler(command);
    }

    newlineIndex = buffer.indexOf('\n');
  }
}

class TTCServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    _ttcConnected = true;
    Serial.println("BLE client connected");
  }

  void onDisconnect(BLEServer*) override {
    _ttcConnected = false;
    _ttcBleBuffer = "";
    Serial.println("BLE client disconnected");
  }
};

class TTCRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    // ESP32 Arduino Core compatibility:
    // - older BLE API revisions may return std::string
    // - ESP32 Arduino Core 3.3.x returns Arduino String
    // Keep the inferred return type so the same code does not depend on either.
    auto rx = characteristic->getValue();
    if (rx.length() == 0) return;

    for (size_t i = 0; i < rx.length(); ++i) {
      char c = rx[i];

      if (c != '\r') {
        _ttcBleBuffer += c;
      }

      if (_ttcBleBuffer.length() > TTC_MAX_RX_LINE) {
        _ttcBleBuffer = "";
        sendTelemetry("ERR,RX_LINE_TOO_LONG");
        return;
      }
    }

    _ttcDispatchBufferedLines(_ttcBleBuffer);
  }
};

inline void communicationBegin(TelecommandHandler handler) {
  _ttcHandler = handler;

  String spacecraftID = getSpacecraftID();

  BLEDevice::init(spacecraftID.c_str());
  BLEDevice::setMTU(TTC_PREFERRED_MTU);

  _ttcServer = BLEDevice::createServer();
  _ttcServer->setCallbacks(new TTCServerCallbacks());

  BLEService *service = _ttcServer->createService(TTC_SERVICE_UUID);

  _ttcTx = service->createCharacteristic(
    TTC_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  _ttcTx->addDescriptor(new BLE2902());

  BLECharacteristic *rx = service->createCharacteristic(
    TTC_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  rx->setCallbacks(new TTCRxCallbacks());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(TTC_SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();
}

inline void communicationUpdate() {
  // USB Serial uses the same newline-framed TC contract as BLE.
  while (Serial.available()) {
    char c = Serial.read();

    if (c != '\r') {
      _ttcSerialBuffer += c;
    }

    if (_ttcSerialBuffer.length() > TTC_MAX_RX_LINE) {
      _ttcSerialBuffer = "";
      sendTelemetry("ERR,RX_LINE_TOO_LONG");
      break;
    }

    _ttcDispatchBufferedLines(_ttcSerialBuffer);
  }

  // Resume advertising after a disconnect.
  if (!_ttcConnected && _ttcOldConnected) {
    delay(200);
    _ttcServer->startAdvertising();
    _ttcOldConnected = false;
  }

  if (_ttcConnected && !_ttcOldConnected) {
    _ttcOldConnected = true;
  }
}

inline bool isCommunicationConnected() {
  return _ttcConnected;
}
