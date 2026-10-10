#pragma once

/* SunSeek Platform v2.0 — MODULE LAYER */
#include <Arduino.h>
#include "Config_Payload.h"
#include "System_TTC.h"

static HardwareSerial _payloadSerial(1);
static String _payloadLine;
static bool _payloadSeen = false;
static unsigned long _payloadLastRxMs = 0;
// [PIO port] capture transaction result for the Mission Manager
static bool _payloadImageReady = false;
static bool _payloadCaptureError = false;
static String _payloadLastImageName = "---";
static unsigned long _payloadLastImageBytes = 0;

inline String _payloadField(const String &line, const String &key) {
  int start = 0;
  while (start < line.length()) {
    int comma = line.indexOf(',', start);
    String token = comma < 0 ? line.substring(start) : line.substring(start, comma);
    if (token == key) {
      if (comma < 0) return "";
      int valueStart = comma + 1;
      int valueEnd = line.indexOf(',', valueStart);
      return valueEnd < 0 ? line.substring(valueStart) : line.substring(valueStart, valueEnd);
    }
    if (comma < 0) break;
    start = comma + 1;
  }
  return "";
}

inline void payloadSendCommand(const String &command) {
  _payloadSerial.print(command);
  _payloadSerial.print('\n');
}

inline void _payloadForwardStatus(const String &line) {
  // Preserve the payload-native message for engineering diagnosis.
  sendTelemetry("PAYLOAD," + line);

  // Also publish the fields used by Ground Station v1.9's flat TM parser.
  String camera = _payloadField(line, "CAMERA");
  String wifi = _payloadField(line, "WIFI");
  String ip = _payloadField(line, "IP");
  String count = _payloadField(line, "IMAGE_COUNT");
  String last = _payloadField(line, "LAST_IMAGE");
  if (!camera.length()) camera = "UNKNOWN";
  if (!wifi.length()) wifi = "UNKNOWN";
  if (!ip.length()) ip = "0.0.0.0";
  if (!count.length()) count = "0";
  if (!last.length()) last = "---";
  sendTelemetry("TM,CAMERA," + camera + ",PAYLOAD_WIFI," + wifi +
                ",PAYLOAD_IP," + ip + ",IMAGE_COUNT," + count +
                ",LAST_IMAGE," + last);
}

inline void _payloadHandleLine(String line) {
  line.trim();
  if (!line.length()) return;
  _payloadSeen = true;
  _payloadLastRxMs = millis();

  if (line.startsWith("STATUS,")) {
    _payloadForwardStatus(line);
    return;
  }
  if (line.startsWith("IMAGE_READY,")) {
    sendTelemetry("PAYLOAD," + line);
    int a = line.indexOf(',');
    int b = line.indexOf(',', a + 1);
    String name = (a >= 0 && b > a) ? line.substring(a + 1, b) : "---";
    String bytes = b >= 0 ? line.substring(b + 1) : "0";
    sendTelemetry("TM,CAMERA,READY,LAST_IMAGE," + name + ",IMAGE_SIZE," + bytes);
    _payloadLastImageName = name;            // [PIO port]
    _payloadLastImageBytes = (unsigned long)bytes.toInt();
    _payloadImageReady = true;
    // Refresh count/status after every successful capture.
    payloadSendCommand("STATUS");
    return;
  }
  if (line.startsWith("EVENT,")) {
    sendTelemetry("PAYLOAD," + line);
    return;
  }
  if (line.startsWith("ACK,STREAM_") ||
      line.startsWith("STREAM_URL,") ||
      line.startsWith("STREAM_STATUS,")) {
    // Payload v0.3 Live View control/status. Keep the payload-native
    // response intact so the Ground Station can drive its stream state.
    sendTelemetry("PAYLOAD," + line);
    return;
  }
  if (line.startsWith("PONG,")) {
    sendTelemetry("PAYLOAD," + line);
    return;
  }
  if (line.startsWith("IMAGE_COUNT,")) {
    sendTelemetry("PAYLOAD," + line);
    sendTelemetry("TM,IMAGE_COUNT," + line.substring(12));
    return;
  }
  if (line.startsWith("LAST_IMAGE,")) {
    sendTelemetry("PAYLOAD," + line);
    int a = line.indexOf(',');
    int b = line.indexOf(',', a + 1);
    String name = (a >= 0 && b > a) ? line.substring(a + 1, b) : "---";
    String bytes = b >= 0 ? line.substring(b + 1) : "0";
    sendTelemetry("TM,LAST_IMAGE," + name + ",IMAGE_SIZE," + bytes);
    return;
  }
  if (line.startsWith("ERR,")) {
    sendTelemetry("PAYLOAD," + line);
    if (line.indexOf("CAPTURE") >= 0) _payloadCaptureError = true;   // [PIO port]
    return;
  }
  sendTelemetry("PAYLOAD,RX," + line);
}

inline void payloadUARTBegin() {
  _payloadLine.reserve(128);
  _payloadSerial.begin(PAYLOAD_UART_BAUD, SERIAL_8N1,
                       PAYLOAD_UART_RX_PIN, PAYLOAD_UART_TX_PIN);
}

inline void payloadUARTUpdate() {
  while (_payloadSerial.available()) {
    char c = (char)_payloadSerial.read();
    if (c == '\n' || c == '\r') {
      if (_payloadLine.length()) {
        _payloadHandleLine(_payloadLine);
        _payloadLine = "";
      }
    } else if (_payloadLine.length() < PAYLOAD_UART_MAX_LINE) {
      _payloadLine += c;
    } else {
      _payloadLine = "";
      sendTelemetry("ERR,PAYLOAD_RX_LINE_TOO_LONG");
    }
  }
}

// [PIO port] Mission capture transaction -------------------------------
inline void payloadRequestCapture() {
  _payloadImageReady = false;
  _payloadCaptureError = false;
  payloadSendCommand("CAPTURE");
}
inline bool payloadTakeImageReady() { bool r = _payloadImageReady; _payloadImageReady = false; return r; }
inline bool payloadTakeCaptureError() { bool r = _payloadCaptureError; _payloadCaptureError = false; return r; }
inline const String& payloadLastImageName() { return _payloadLastImageName; }
inline unsigned long payloadLastImageBytes() { return _payloadLastImageBytes; }

inline bool payloadHasResponded() { return _payloadSeen; }
inline unsigned long payloadLastRxMs() { return _payloadLastRxMs; }
