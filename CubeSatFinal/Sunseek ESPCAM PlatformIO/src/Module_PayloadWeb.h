#pragma once
// ============================================================
// Module_PayloadWeb — Wi-Fi bring-up + HTTP data plane (developer extension layer)
//
// Two esp_http_server instances, because each instance runs ALL its handlers in one task:
//   port PAYLOAD_HTTP_PORT   (80)  GET /  /status  /images  /image?name=/IMG_0001.JPG
//   port PAYLOAD_STREAM_PORT (81)  GET /stream   (MJPEG, blocks its task while a client is attached)
// With one server (v3.0) an open /stream made /status, /images and /image unreachable.
// ============================================================
#include <Arduino.h>

void     payloadWebBegin();              // Wi-Fi (AP or STA) + both servers
bool     payloadWebApiRunning();
bool     payloadWebStreamRunning();
uint32_t payloadWebStreamClients();      // browsers currently attached to /stream (handler running)
uint32_t payloadWebApiClients();         // open sockets on the :80 server (zombies show up here)
uint32_t payloadWebStreamSockets();      // open sockets on the :81 server
uint32_t payloadWebStations();           // Wi-Fi stations associated with the AP (1 = the laptop is there)
void     payloadWebStop();               // stop both servers + Wi-Fi
void     payloadWebRestart();            // stop + begin again — recovery without a power cycle (UART WIFI_RESTART)
