#include "System_PayloadState.h"
#include "Config_Payload.h"
#include "Module_Camera.h"
#include "Module_Storage.h"
#include <WiFi.h>
#include "Module_PayloadWeb.h"

static volatile bool _streamEnabled = (PAYLOAD_STREAM_DEFAULT_ON != 0);
static volatile bool _apStarted     = false;

bool payloadStreamEnabled()         { return _streamEnabled; }
void payloadSetStreamEnabled(bool v){ _streamEnabled = v; }
void payloadSetApStarted(bool ok)   { _apStarted = ok; }

bool payloadWifiReady() {
#if PAYLOAD_WIFI_AP_MODE
  return _apStarted;                                   // softAP() result, not just "mode is AP"
#else
  return WiFi.status() == WL_CONNECTED;
#endif
}

String payloadIP() {
#if PAYLOAD_WIFI_AP_MODE
  return WiFi.softAPIP().toString();
#else
  return WiFi.localIP().toString();
#endif
}

String payloadStreamURL() {
  return "http://" + payloadIP() + ":" + String(PAYLOAD_STREAM_PORT) + "/stream";
}

bool payloadReady() { return cameraModuleReady() && payloadWifiReady(); }

String payloadStatusLine() {
  String s = "STATUS,";
  s += payloadReady() ? "READY" : "NOT_READY";
  s += ",CAMERA,";      s += cameraModuleReady()   ? "OK" : "ERR";
  s += ",STORAGE,";     s += storageModuleReady()  ? "OK" : "ERR";
  s += ",WIFI,";        s += payloadWifiReady()    ? "READY" : "NOT_READY";
  s += ",IP,";          s += payloadIP();
  s += ",STREAM,";      s += payloadStreamEnabled()? "ON" : "OFF";
  s += ",IMAGE_COUNT,"; s += String(storageImageCount());
  s += ",LAST_IMAGE,";  s += storageLastImage();
  return s;
}

String payloadDiagLine() {
  String s = "DIAG,PSRAM,";
  s += psramFound() ? "1" : "0";
  s += ","; s += String((unsigned long)ESP.getFreePsram());
  s += ",HEAP,"; s += String((unsigned long)ESP.getFreeHeap());
  s += ",CAM,"; s += cameraModuleReady() ? "OK" : "ERR";
  char hex[16]; snprintf(hex, sizeof(hex), "0x%X", (unsigned)cameraModuleLastError());
  s += ","; s += hex;
  s += ","; s += cameraModuleFrameSizeName();
  s += ",FALLBACK,"; s += cameraModuleOnFallback() ? "1" : "0";
  size_t len = 0; int w = 0, h = 0;
  if (cameraModuleReady()) {
    camera_fb_t* fb = cameraModuleGetFrame();              // esp_camera_fb_get() times out → nullptr
    if (fb) { len = fb->len; w = fb->width; h = fb->height; cameraModuleReturnFrame(fb); }
  }
  s += ",FRAME,"; s += String((unsigned long)len); s += ","; s += String(w); s += "x"; s += String(h);
  s += ",SD,"; s += storageCardInfo();
  s += ",STREAM_CLIENTS,"; s += String((unsigned long)payloadWebStreamClients());
  s += ",AP_STA,"; s += String((unsigned long)payloadWebStations());
  s += ",HTTP_SOCK,"; s += String((unsigned long)payloadWebApiClients()); s += "/"; s += String((unsigned long)payloadWebStreamSockets());
  s += ",UPTIME,"; s += String((unsigned long)(millis() / 1000));
  return s;
}
