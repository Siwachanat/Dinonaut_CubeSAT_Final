#pragma once
// ============================================================
// System_PayloadCommand — UART command dispatcher (core layer)
//
//  PING            -> PONG,CAMERA
//  STATUS          -> STATUS,<READY|NOT_READY>,CAMERA,<OK|ERR>,STORAGE,<OK|ERR>,WIFI,<READY|NOT_READY>,IP,<ip>,
//                     STREAM,<ON|OFF>,IMAGE_COUNT,<n>,LAST_IMAGE,<path>
//  CAPTURE         -> EVENT,CAPTURE_STARTED  then  IMAGE_READY,<path>,<bytes> | ERR,CAPTURE_FAILED
//  IMAGE_COUNT     -> IMAGE_COUNT,<n>
//  LAST_IMAGE      -> LAST_IMAGE,<path>,<bytes>
//  STREAM_START    -> ACK,STREAM_START + EVENT,STREAM_ON | ERR,STREAM_CAMERA_NOT_READY
//  STREAM_STOP     -> ACK,STREAM_STOP + EVENT,STREAM_OFF
//  STREAM_STATUS   -> STREAM_STATUS,<ON|OFF>
//  VERSION         -> VERSION,<fw>                         (new in 3.1)
//  STREAM_URL      -> STREAM_URL,http://<ip>:<port>/stream  (new in 3.1)
//  STORAGE_REMOUNT -> ACK,STORAGE_REMOUNT,<OK|ERR>          (new in 3.1 — card inserted after boot)
//  WIFI_RESTART    -> ACK,WIFI_RESTART then WIFI_IP,... STATUS,...   (new in 3.1 — recover a wedged data link)
//  DIAG            -> DIAG,PSRAM,...,CAM,...,FRAME,<bytes>,<w>x<h>,SD,<type>,<MB>,...  (new in 3.1 — bring-up)
//  anything else   -> ERR,UNKNOWN_COMMAND
// ============================================================
#include <Arduino.h>

void payloadProcessCommand(String cmd);
