// BLE stand-in: no ground station is connected in the PC test, telemetry goes out on "USB" only.
#pragma once
#include "Arduino.h"
class BLEDescriptor {};
class BLE2902 : public BLEDescriptor {};
class BLECharacteristic;
class BLECharacteristicCallbacks { public: virtual ~BLECharacteristicCallbacks() {} virtual void onWrite(BLECharacteristic *) {} };
class BLECharacteristic {
 public:
  enum { PROPERTY_NOTIFY = 1, PROPERTY_WRITE = 2, PROPERTY_WRITE_NR = 4 };
  void setValue(uint8_t *, size_t) {}
  void notify() {}
  void addDescriptor(BLEDescriptor *) {}
  void setCallbacks(BLECharacteristicCallbacks *) {}
  String getValue() { return String(); }
};
class BLEService { public: BLECharacteristic *createCharacteristic(const char *, uint32_t) { return new BLECharacteristic(); } void start() {} };
class BLEServer;
class BLEServerCallbacks { public: virtual ~BLEServerCallbacks() {} virtual void onConnect(BLEServer *) {} virtual void onDisconnect(BLEServer *) {} };
class BLEServer { public: void setCallbacks(BLEServerCallbacks *) {} BLEService *createService(const char *) { return new BLEService(); } void startAdvertising() {} };
class BLEAdvertising { public: void addServiceUUID(const char *) {} void setScanResponse(bool) {} };
class BLEDevice {
 public:
  static void init(const char *) {}
  static void setMTU(int) {}
  static BLEServer *createServer() { return new BLEServer(); }
  static BLEAdvertising *getAdvertising() { static BLEAdvertising a; return &a; }
  static void startAdvertising() {}
};
