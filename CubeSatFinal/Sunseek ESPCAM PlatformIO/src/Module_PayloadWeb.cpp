#include "Module_PayloadWeb.h"
#include "Config_Payload.h"
#include "Module_Camera.h"
#include "Module_Storage.h"
#include "System_PayloadState.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "SD_MMC.h"
#include <atomic>

static httpd_handle_t    _httpApi       = nullptr;
static httpd_handle_t    _httpStream    = nullptr;
static std::atomic<uint32_t> _streamClients{0};
static uint32_t clientCount(httpd_handle_t h);

// ------------------------------------------------------------ helpers
static void commonHeaders(httpd_req_t* r) {
  httpd_resp_set_hdr(r, "Access-Control-Allow-Origin", "*");   // lets a browser-based ground station fetch /status
  httpd_resp_set_hdr(r, "Cache-Control", "no-store");
}

static esp_err_t sendText(httpd_req_t* r, const char* type, const String& s, int code = 200) {
  httpd_resp_set_type(r, type);
  commonHeaders(r);
  if (code == 400) httpd_resp_set_status(r, "400 Bad Request");
  if (code == 404) httpd_resp_set_status(r, "404 Not Found");
  if (code == 503) httpd_resp_set_status(r, "503 Service Unavailable");
  return httpd_resp_send(r, s.c_str(), s.length());
}

// In-place %XX decoding so /image?name=%2FIMG_0001.JPG works as well as name=/IMG_0001.JPG
static void urlDecode(char* s) {
  char* o = s;
  while (*s) {
    if (*s == '%' && isxdigit((unsigned char)s[1]) && isxdigit((unsigned char)s[2])) {
      char h[3] = { s[1], s[2], 0 };
      *o++ = (char)strtol(h, nullptr, 16);
      s += 3;
    } else if (*s == '+') { *o++ = ' '; s++; }
    else                  { *o++ = *s++; }
  }
  *o = 0;
}

static String statusJson() {
  String s = "{";
  s += "\"fw\":\"" PAYLOAD_FW_VERSION "\"";
  s += ",\"ready\":";           s += payloadReady()        ? "true" : "false";
  s += ",\"camera\":";          s += cameraModuleReady()   ? "true" : "false";
  s += ",\"storage\":";         s += storageModuleReady()  ? "true" : "false";
  s += ",\"wifi\":";            s += payloadWifiReady()    ? "true" : "false";
  s += ",\"stream\":\"";        s += payloadStreamEnabled()? "ON" : "OFF"; s += "\"";
  s += ",\"stream_port\":";     s += String(PAYLOAD_STREAM_PORT);
  s += ",\"stream_clients\":";  s += String((unsigned long)_streamClients.load());
  s += ",\"image_count\":";     s += String(storageImageCount());
  s += ",\"last_image\":\"";    s += storageLastImage(); s += "\"";
  s += ",\"last_image_size\":"; s += String((unsigned long)storageLastImageSize());
  s += ",\"frame_size\":\"";    s += cameraModuleFrameSizeName(); s += "\"";
  s += ",\"psram\":";           s += cameraModuleUsingPsram() ? "true" : "false";
  s += ",\"camera_fallback\":"; s += cameraModuleOnFallback() ? "true" : "false";
  s += ",\"ap_stations\":";     s += String((unsigned long)payloadWebStations());
  s += ",\"http_clients\":";    s += String((unsigned long)clientCount(_httpApi));
  s += ",\"uptime_s\":";        s += String((unsigned long)(millis() / 1000));
  s += ",\"free_heap\":";       s += String((unsigned long)ESP.getFreeHeap());
  s += "}";
  return s;
}

// ------------------------------------------------------------ port 80 handlers
static esp_err_t hStatus(httpd_req_t* r) { return sendText(r, "application/json", statusJson()); }

static esp_err_t hImages(httpd_req_t* r) {
  if (!storageModuleReady()) return sendText(r, "application/json", "{\"error\":\"STORAGE_NOT_READY\"}", 503);
  String s = "[";
  bool first = true;
  File root = SD_MMC.open("/");
  if (root) {
    File f = root.openNextFile();
    while (f) {
      String n = f.name(), u = n; u.toUpperCase();
      if (!f.isDirectory() && u.endsWith(".JPG")) {
        if (!first) s += ",";
        if (!n.startsWith("/")) n = "/" + n;
        s += "{\"name\":\"" + n + "\",\"size\":" + String((unsigned long)f.size()) + "}";
        first = false;
      }
      f = root.openNextFile();
    }
    root.close();
  }
  s += "]";
  return sendText(r, "application/json", s);
}

static esp_err_t hImage(httpd_req_t* r) {
  if (!storageModuleReady()) return sendText(r, "text/plain", "STORAGE_NOT_READY", 503);

  char q[160] = {0}, name[128] = {0};
  if (httpd_req_get_url_query_str(r, q, sizeof(q)) != ESP_OK ||
      httpd_query_key_value(q, "name", name, sizeof(name)) != ESP_OK)
    return sendText(r, "text/plain", "BAD_REQUEST", 400);
  urlDecode(name);

  String p = name;
  if (!p.startsWith("/")) p = "/" + p;
  String u = p; u.toUpperCase();
  if (p.indexOf("..") >= 0 || !u.endsWith(".JPG")) return sendText(r, "text/plain", "INVALID_IMAGE_NAME", 400);

  File f = SD_MMC.open(p, FILE_READ);
  if (!f || f.isDirectory()) { if (f) f.close(); return sendText(r, "text/plain", "IMAGE_NOT_FOUND", 404); }

  httpd_resp_set_type(r, "image/jpeg");
  commonHeaders(r);
  // Static buffer: the server runs its handlers one at a time, and the default task stack is small (4 KB
  // in v3.0; 8 KB here) — a large buffer on the stack is what overflows it.
  static uint8_t buf[2048];
  while (f.available()) {
    size_t n = f.read(buf, sizeof(buf));
    if (n == 0) break;                           // a 0-byte chunk would terminate the response early
    if (httpd_resp_send_chunk(r, (const char*)buf, n) != ESP_OK) { f.close(); return ESP_FAIL; }
  }
  f.close();
  return httpd_resp_send_chunk(r, nullptr, 0);
}

static esp_err_t hRoot(httpd_req_t* r) {
  static const char page[] = R"HTML(<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SunSeek Payload</title><style>body{font:14px system-ui,sans-serif;margin:16px;max-width:900px}pre{background:#f3f3f3;padding:8px;overflow:auto}
img{max-width:100%;border:1px solid #ccc}a{margin-right:12px}.off{color:#a00}</style></head><body>
<h2>SunSeek Payload v)HTML" PAYLOAD_FW_VERSION R"HTML( <small>engineering / fallback interface</small></h2>
<p><a href="/status">/status</a><a href="/images">/images</a><a href="/capture">/capture (single frame, no SD)</a><a id="sl" href="#">/stream (port STREAMPORT)</a>
<button onclick="refresh()">Refresh</button> <button onclick="snap()">Take still</button></p>
<div id="still"></div>
<pre id="st">loading…</pre>
<p id="note">Live stream is only available while STREAM is ON — send <b>STREAM_START</b> over Serial/OBC.</p>
<div id="live"></div>
<h3>Stored images</h3><div id="imgs"></div>
<script>
const sp=STREAMPORT;const su=location.protocol+'//'+location.hostname+':'+sp+'/stream';
document.getElementById('sl').href=su;
async function refresh(){
 try{const s=await (await fetch('/status')).json();document.getElementById('st').textContent=JSON.stringify(s,null,1);
  const live=document.getElementById('live');live.innerHTML=s.stream==='ON'?'<img src="'+su+'?t='+Date.now()+'">':'<span class="off">STREAM OFF</span>';
  if(s.storage){const l=await (await fetch('/images')).json();
   document.getElementById('imgs').innerHTML=l.length?l.map(i=>'<a href="/image?name='+encodeURIComponent(i.name)+'">'+i.name+'</a> ('+i.size+' B)').join('<br>'):'(none)';}
  else document.getElementById('imgs').textContent='storage not ready';
 }catch(e){document.getElementById('st').textContent='status fetch failed: '+e;}
}
function snap(){document.getElementById('still').innerHTML='<img src="/capture?t='+Date.now()+'">';}
refresh();
</script></body></html>)HTML";
  String h = page;
  h.replace("STREAMPORT", String(PAYLOAD_STREAM_PORT));
  return sendText(r, "text/html", h);
}

// One JPEG straight from the sensor, no SD involved. Separates "camera gives black/nothing" from "SD" and
// from "stream plumbing" during bring-up, and gives the GS a live still without enabling the stream.
static esp_err_t hCapture(httpd_req_t* r) {
  if (!cameraModuleReady()) return sendText(r, "text/plain", "CAMERA_NOT_READY", 503);
  camera_fb_t* fb = cameraModuleGetFrame();
  if (!fb) return sendText(r, "text/plain", "NO_FRAME", 503);
  if (fb->format != PIXFORMAT_JPEG) { cameraModuleReturnFrame(fb); return sendText(r, "text/plain", "NOT_JPEG", 503); }
  httpd_resp_set_type(r, "image/jpeg");
  commonHeaders(r);
  esp_err_t e = httpd_resp_send(r, (const char*)fb->buf, fb->len);
  cameraModuleReturnFrame(fb);
  return e;
}

// ------------------------------------------------------------ port 81 handler
static esp_err_t hStream(httpd_req_t* r) {
  if (!cameraModuleReady())    return sendText(r, "text/plain", "CAMERA_NOT_READY", 503);
  if (!payloadStreamEnabled()) return sendText(r, "text/plain", "STREAM_OFF", 503);

  httpd_resp_set_type(r, "multipart/x-mixed-replace;boundary=frame");
  commonHeaders(r);
  _streamClients++;

  esp_err_t e = ESP_OK;
  while (payloadStreamEnabled() && cameraModuleReady()) {       // STREAM_STOP over UART ends this loop
    camera_fb_t* fb = cameraModuleGetFrame();
    if (!fb) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
    if (fb->format != PIXFORMAT_JPEG) { cameraModuleReturnFrame(fb); e = ESP_FAIL; break; }

    char head[96];
    int n = snprintf(head, sizeof(head), "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", (unsigned)fb->len);
    e = httpd_resp_send_chunk(r, head, n);
    if (e == ESP_OK) e = httpd_resp_send_chunk(r, (const char*)fb->buf, fb->len);
    if (e == ESP_OK) e = httpd_resp_send_chunk(r, "\r\n", 2);
    cameraModuleReturnFrame(fb);
    if (e != ESP_OK) break;                                     // client went away
    vTaskDelay(pdMS_TO_TICKS(PAYLOAD_STREAM_FRAME_MS));
  }

  _streamClients--;
  if (e != ESP_OK) return e;                                    // socket already dead, nothing to finish
  return httpd_resp_send_chunk(r, nullptr, 0);                  // clean end when STREAM_STOP was commanded
}

// ------------------------------------------------------------ server plumbing
static httpd_uri_t makeUri(const char* uri, esp_err_t (*handler)(httpd_req_t*)) {
  httpd_uri_t u = {};            // no designated initialisers: the struct gained fields between IDF 4.4 and 5.x
  u.uri = uri; u.method = HTTP_GET; u.handler = handler; u.user_ctx = nullptr;
  return u;
}

static bool startServer(httpd_handle_t* h, uint16_t port, uint16_t ctrlPort, uint16_t maxSockets,
                        const httpd_uri_t* uris, size_t count) {
  httpd_config_t c  = HTTPD_DEFAULT_CONFIG();
  c.server_port     = port;
  c.ctrl_port       = ctrlPort;      // MUST differ between the two instances or the second httpd_start() fails
  c.max_uri_handlers= 8;
  c.max_open_sockets= maxSockets;    // the two servers share LWIP's pool (CONFIG_LWIP_MAX_SOCKETS = 16); each needs +3
  c.stack_size      = 8192;          // v3.0 used the 4 KB default with ~1.4 KB of locals in /image
  c.lru_purge_enable= true;          // drop the least-recently-used idle connection instead of refusing new ones
  c.recv_wait_timeout = 3;           // a stalled client can block this single-threaded server for at most 3 s
  c.send_wait_timeout = 3;
  // TCP keep-alive: a laptop that silently drops a connection (Wi-Fi hiccup, sleep, tab closed while the
  // radio was bad) otherwise leaves a zombie socket open forever. Enough of those and the server stops
  // accepting new connections — "status lost, never comes back". Dead peers are reaped in ~14 s.
  c.keep_alive_enable   = true;
  c.keep_alive_idle     = 5;
  c.keep_alive_interval = 3;
  c.keep_alive_count    = 3;
  if (httpd_start(h, &c) != ESP_OK) { *h = nullptr; return false; }
  for (size_t i = 0; i < count; i++) httpd_register_uri_handler(*h, &uris[i]);
  return true;
}

void payloadWebBegin() {
#if PAYLOAD_WIFI_AP_MODE
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(PAYLOAD_WIFI_SSID, PAYLOAD_WIFI_PASSWORD, PAYLOAD_WIFI_AP_CHANNEL);
  payloadSetApStarted(ok);
  WiFi.setTxPower(PAYLOAD_WIFI_TX_POWER);
#else
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(PAYLOAD_STA_SSID, PAYLOAD_STA_PASSWORD);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < PAYLOAD_STA_CONNECT_TIMEOUT_MS) delay(100);
  WiFi.setTxPower(PAYLOAD_WIFI_TX_POWER);
#endif

  static const httpd_uri_t apiUris[] = {
    makeUri("/status", hStatus),
    makeUri("/images", hImages),
    makeUri("/image",  hImage),
    makeUri("/capture",hCapture),
    makeUri("/",       hRoot),
  };
  static const httpd_uri_t streamUris[] = {
    makeUri("/stream", hStream),
  };
  // Chrome opens up to 6 connections to one host; give the API room for that. (6+3) + (2+3) = 14 of 16.
  startServer(&_httpApi,    PAYLOAD_HTTP_PORT,   32768, 6, apiUris,    sizeof(apiUris)    / sizeof(apiUris[0]));
  startServer(&_httpStream, PAYLOAD_STREAM_PORT, 32769, 2, streamUris, sizeof(streamUris) / sizeof(streamUris[0]));
}

void payloadWebStop() {
  if (_httpStream) { httpd_stop(_httpStream); _httpStream = nullptr; }   // closes every session; the stream handler's send fails and it returns
  if (_httpApi)    { httpd_stop(_httpApi);    _httpApi    = nullptr; }
#if PAYLOAD_WIFI_AP_MODE
  WiFi.softAPdisconnect(true);
#else
  WiFi.disconnect(true);
#endif
  payloadSetApStarted(false);
}

void payloadWebRestart() {
  payloadWebStop();
  delay(300);
  payloadWebBegin();
}

static uint32_t clientCount(httpd_handle_t h) {
  if (!h) return 0;
  size_t n = 16; int fds[16];
  return httpd_get_client_list(h, &n, fds) == ESP_OK ? (uint32_t)n : 0;
}
uint32_t payloadWebApiClients()    { return clientCount(_httpApi); }
uint32_t payloadWebStreamSockets() { return clientCount(_httpStream); }
uint32_t payloadWebStations()      {
#if PAYLOAD_WIFI_AP_MODE
  return WiFi.softAPgetStationNum();
#else
  return WiFi.status() == WL_CONNECTED ? 1 : 0;
#endif
}

bool     payloadWebApiRunning()    { return _httpApi    != nullptr; }
bool     payloadWebStreamRunning() { return _httpStream != nullptr; }
uint32_t payloadWebStreamClients() { return _streamClients.load(); }
