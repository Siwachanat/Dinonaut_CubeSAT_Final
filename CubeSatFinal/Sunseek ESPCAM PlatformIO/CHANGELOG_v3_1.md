# v3.1 changes (PlatformIO port of v3.0)

Protocol compatibility: every v3.0 UART response and HTTP endpoint is byte-identical. Additions only.

## Structure
- Ported from a single `.ino` + header-only files to a PlatformIO project (`src/*.h` + `src/*.cpp`, `main.cpp`).
  The `Config_* / Module_* / System_*` layering and all function names are kept.
- Header-only `static` state (`_cameraReady`, `_streamEnabled`, `_rx`, ...) moved into `.cpp` files so there is
  exactly one copy of each — in v3.0 any second translation unit would silently have received its own copy.

## Fixes
- **`/stream` no longer blocks the API.** esp_http_server runs all handlers of one instance in one task, so in v3.0
  an open `/stream` made `/status`, `/images` and `/image` unreachable. The stream now runs on its own server
  instance (port `PAYLOAD_STREAM_PORT` = 81, separate `ctrl_port`).
- Image metadata (`_lastImage`, count, size) shared between the UART task and the HTTP task is now mutex-guarded
  and returned by value (v3.0 read a `String` while another task could be reassigning it).
- `LAST_IMAGE` / next file index after boot are now the **highest `IMG_NNNN` number**, not FAT directory order.
- Short SD writes are detected (`written != frame length`), the partial file is deleted and `ERR,CAPTURE_FAILED` is
  returned (v3.0 reported `IMAGE_READY` for any non-zero write).
- UART lines longer than 95 chars are discarded with `ERR,LINE_TOO_LONG` (v3.0 split them into two commands).
- ESP-IDF / Arduino log output on UART0 (the OBC link) is suppressed (`CORE_DEBUG_LEVEL=0`, `esp_log_level_set`).
- HTTP server task stack 4 KB → 8 KB; `/image` uses a static 2 KB buffer instead of 1.4 KB of stack locals.
- `/image`: `%XX` URL-decoding of `name`; a zero-length read no longer terminates the response early.
- `payloadWifiReady()` reflects the actual `WiFi.softAP()` result instead of "mode is AP".
- Camera init falls back once to `FRAMESIZE_VGA` in DRAM if the PSRAM/SVGA configuration fails.
- Frame format is checked before writing/streaming (guards against a non-JPEG `pixel_format` in Config).
- Flash LED (GPIO4) parked LOW (SD runs in 1-bit mode, so the pin is free). `PAYLOAD_FLASH_LED_OFF`.
- SD_MMC clock set explicitly to 20 MHz (`PAYLOAD_SD_FREQ_KHZ`) — the safe value for 1-bit mode on cheap cards.
- STA mode: `WiFi.setAutoReconnect(true)`.
- `httpd_uri_t` built without designated initialisers (struct layout differs between IDF 4.4 and 5.x).
- Partition table `huge_app.csv` (3 MB app). The binary is 1.13 MB; PlatformIO's default for `esp32cam` is 1.25 MB.

## Additions
- HTTP servers: TCP keep-alive (5 s idle / 3 s interval / 3 probes), 3 s send/recv timeouts, :80 socket budget 4 → 6. Zombie connections from a flaky client no longer accumulate until the server stops accepting.
- UART: `WIFI_RESTART` → restarts AP + both HTTP servers without a power cycle.
- `DIAG` adds `AP_STA,<n>`, `HTTP_SOCK,<api>/<stream>`, `UPTIME,<s>`; `/status` adds `ap_stations`, `http_clients`, `uptime_s`, `free_heap`.
- Boot prints `RESET_REASON,<POWERON|BROWNOUT|PANIC|TASK_WDT|...>` after `BOOT,…` so a field reset is visible on the OBC link.
- Wi-Fi TX power limited to 8.5 dBm (`PAYLOAD_WIFI_TX_POWER`) — the TX burst after a capture is the usual brown-out trigger.
- UART: `DIAG` → `DIAG,PSRAM,...,CAM,...,FRAME,<bytes>,<w>x<h>,SD,<type>,<MB>,...` (bring-up); also printed once at boot.
- HTTP: `GET /capture` on port 80 — one JPEG straight from the sensor, no SD needed.
- `esp32cam_debug` build env: all ESP-IDF / Arduino log output kept on UART0 for bench diagnosis.
- UART: `VERSION` → `VERSION,3.1`; `STREAM_URL` → `STREAM_URL,http://<ip>:81/stream`;
  `STORAGE_REMOUNT` → `ACK,STORAGE_REMOUNT,OK|ERR` (card inserted after boot).
- Boot prints one extra line `STREAM_URL,...` between `WIFI_IP,...` and `STATUS,...`.
- `/status` JSON gains `fw`, `stream_port`, `stream_clients`, `frame_size`, `psram`, `camera_fallback`.
- `Access-Control-Allow-Origin: *` on all HTTP responses (browser-based ground station can fetch `/status`).
- Root page `/` shows live status, the stored-image list with links and the stream (from port 81).
