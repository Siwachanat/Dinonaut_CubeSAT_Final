#pragma once
// ============================================================
// System_PayloadState — the payload's reported state (core layer)
//
// CAMERA, STORAGE, WIFI and STREAM are independent flags.
// PAYLOAD READY  = camera + Wi-Fi ready (storage is NOT required for live streaming).
// ============================================================
#include <Arduino.h>

// Stream state: last value commanded over UART; survives TT&C loss while powered. Read by the stream
// server task, written by the UART task.
bool   payloadStreamEnabled();
void   payloadSetStreamEnabled(bool v);

// Wi-Fi link
void   payloadSetApStarted(bool ok);     // set by the web module after WiFi.softAP()
bool   payloadWifiReady();
String payloadIP();
String payloadStreamURL();               // "http://<ip>:<PAYLOAD_STREAM_PORT>/stream"

bool   payloadReady();                   // camera && wifi
String payloadStatusLine();              // "STATUS,READY,CAMERA,OK,STORAGE,OK,WIFI,READY,IP,...,STREAM,ON,IMAGE_COUNT,n,LAST_IMAGE,/IMG_0001.JPG"
// "DIAG,PSRAM,<found>,<free bytes>,HEAP,<free bytes>,CAM,<OK|ERR>,<esp_err hex>,<frame size>,FALLBACK,<0|1>,
//  FRAME,<bytes>,<w>x<h>,SD,<type>,<MB>,STREAM_CLIENTS,<n>,AP_STA,<n>,HTTP_SOCK,<api>/<stream>,UPTIME,<s>"
//  Grabs one test frame; FRAME,0,0x0 means the sensor delivered nothing. AP_STA = laptops associated.
String payloadDiagLine();
