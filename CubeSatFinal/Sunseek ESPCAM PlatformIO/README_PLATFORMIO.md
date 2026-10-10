# SunSeek ESP32-CAM Payload Firmware v3.1 — PlatformIO port

PlatformIO port of the organisers' `SunSeek_ESP32CAM_Payload_Firmware_v3_0.ino` package, with the
bugs found in review fixed. **All v3.0 UART responses and HTTP endpoints are unchanged**; new
commands/fields are additive only (see `CHANGELOG_v3_1.md`). Full structure, diagrams and the review
are in `docs/ARCHITECTURE.md`.

## Build / flash

```sh
cd PlatformIO
pio run -e esp32cam                 # build  (pioarduino 55.03.30 = Arduino-ESP32 3.3.x, same as the S3 ports)
pio run -e esp32cam -t upload       # flash  (see wiring below)
pio device monitor                  # 115200 8N1
pio run -e esp32cam_core2           # fallback: official espressif32@6.9.0 (Arduino-ESP32 2.0.17)
```

Verified: `esp32cam` env builds clean (0 warnings in `src/`), RAM 17.6 %, **Flash 1.13 MB / 3 MB**
(`huge_app.csv`). The `esp32cam_core2` env is provided but was not compiled in this environment.

### Flashing wiring (ESP32-CAM has no USB)

| FTDI / CP210x (3.3 V logic) | ESP32-CAM |
|---|---|
| TX | U0R (GPIO3) |
| RX | U0T (GPIO1) |
| GND | GND |
| **5 V** (not 3V3) | 5V |

1. **Disconnect the OBC UART** from GPIO1/GPIO3 — same pins as the programmer.
2. Jumper **IO0 → GND**, press RST, run `pio run -t upload`, remove the jumper, press RST.
3. If upload times out, set `upload_speed = 115200` in `platformio.ini`.
4. Power from a 5 V supply able to give ≥ 500 mA. Powering from the FTDI 3V3 pin brown-outs on Wi-Fi start.

## Where to edit

| Layer | Files | Who edits |
|---|---|---|
| `Config_*` | `Config_Payload.h`, `Config_Camera.h` | workshop / team — SSID, ports, frame size, LED, log silencing |
| `Module_*` | `Module_Camera`, `Module_Storage`, `Module_PayloadWeb` | developer extension (sensor, SD, HTTP) |
| `System_*` | `System_Payload`, `System_PayloadState`, `System_PayloadInterface`, `System_PayloadCommand` | core: boot, state, UART protocol |
| | `main.cpp` | `setup()`/`loop()` only |

## Bring-up checklist (T05)

1. Flash with OBC disconnected, open the monitor. Expect, after the ROM banner:
   `BOOT,SUNSEEK_PAYLOAD_V3.1` · `PAYLOAD,READY` · `WIFI_IP,192.168.4.1` · `STREAM_URL,http://192.168.4.1:81/stream` · `STATUS,...`
2. `PING` → `PONG,CAMERA`. `STATUS`. `CAPTURE` → `EVENT,CAPTURE_STARTED` then `IMAGE_READY,/IMG_0001.JPG,<bytes>`.
3. `STREAM_START` → `ACK,STREAM_START` + `EVENT,STREAM_ON`.
4. Join Wi-Fi `SUNSEEK-PAYLOAD` / `sunseek01`, open `http://192.168.4.1/` — the page shows status,
   the stored-image list and the live stream (from port 81) in one place.
5. `STREAM_STOP`, confirm the browser stream ends and `/status` still answers.
6. Only then connect the OBC UART1 (GPIO41 TX → U0R, GPIO42 RX ← U0T) and repeat via OBC/BLE.

## Notes for the OBC / ground-station side

- The ESP32 ROM prints a boot banner (`rst:0x1 (POWERON_RESET)...`, `ets Jul 29 2019...`) on UART0
  at 115200 before our code runs. **The OBC parser must ignore lines it does not recognise.** All
  other system log output is suppressed in 3.1.
- `CAPTURE` blocks the payload for the SD write (~0.3–1 s at SVGA). Wait for `IMAGE_READY` /
  `ERR,CAPTURE_FAILED` with a timeout of ≥ 3 s rather than sending the next command immediately.
- The live stream is on **port 81** (`/stream`), the API stays on port 80. One stream client at a time;
  a second one waits until the first disconnects.

## Troubleshooting (first bench run: "SD card not found", "stream is black")

Flash the **debug** build first — it keeps every camera / SD / Wi-Fi error message on the serial monitor
(the normal build silences them so the OBC parser only sees protocol lines). OBC must be unplugged.

```sh
pio run -e esp32cam_debug -t upload && pio device monitor
```

Then send `DIAG` and read the line:

```
DIAG,PSRAM,1,4150000,HEAP,170000,CAM,OK,0x0,SVGA,FALLBACK,0,FRAME,48213,800x600,SD,SDHC,15193MB,STREAM_CLIENTS,0
       │                        │                              │                  └ card type + size (NONE = mount failed)
       │                        │                              └ one test frame: bytes and WxH
       │                        └ camera init result (esp_err_t), frame size, fallback used?
       └ PSRAM detected? (0 = no PSRAM → camera in DRAM → small frames, no SVGA)
```

| What you see | Meaning | Do |
|---|---|---|
| debug log `Failed to initialize the card (0x107)` | card did not answer (timeout) | card not inserted / not clicked in, bad contacts, weak power, dead card. Try another card. |
| debug log `Failed to mount filesystem` | card answers but no FAT filesystem | card is exFAT (all cards > 32 GB) or unformatted → format **FAT32** (SD Formatter / `diskutil eraseDisk FAT32 SD MBRFormat /dev/diskN`) |
| `SD,NONE` and no log line at all | `SD_MMC.begin` returned false before talking to the card | pin conflict — report it |
| `FRAME,0,0x0` | camera initialised but delivers no frames | ribbon cable not seated, sensor underpowered, or XCLK/PSRAM problem → see power |
| `FRAME,<~5–15 kB>,800x600` | frames arrive but are **black** | lens protective film still on, lens cap, or sensor underpowered (SVGA black JPEG ≈ 8–15 kB; a real scene ≈ 40–100 kB) |
| `FRAME,<40 kB+>` but browser stream black | camera fine, stream path | open `http://192.168.4.1/capture` — if that shows a picture, the problem is the stream/browser, not the camera |
| `PSRAM,0` | no PSRAM detected | board is not a genuine AI-Thinker, or `-DBOARD_HAS_PSRAM` missing — check `pio run -v` flags |
| `CAM,ERR,0x105` | `ESP_ERR_NOT_FOUND`: sensor not detected on SCCB | ribbon cable, wrong board model, insufficient power |
| `CAM,ERR,0x101` | `ESP_ERR_NO_MEM` | PSRAM not usable → fallback should have caught it |
| reboot loop with `Brownout detector was triggered` | power | 5 V ≥ 500 mA supply; do not power from the FTDI 3V3 pin or a thin USB cable |

Quick checks that need no software: the OV2640 lens ships with a clear protective film — peel it; press
the microSD until it clicks; power from a USB charger or bench supply at 5 V, not the programmer.

Two A/B tests that isolate the cause in minutes:
1. Flash the organisers' Arduino v3.0 `.ino` on the same board/card. Same failure → hardware (card, cable, power).
2. `pio run -e esp32cam_core2 -t upload` (Arduino-ESP32 2.0.17). Works there but not on `esp32cam` → core-3.x issue, report it.

### "Lost connection after capture"

The Wi-Fi AP disappearing right after a capture means the board **reset**. Watch the serial monitor when it
happens — the first lines after the ROM banner say why:

```
BOOT,SUNSEEK_PAYLOAD_V3.1
RESET_REASON,BROWNOUT      ← supply sagged: power problem (see below)
RESET_REASON,PANIC         ← software crash: flash esp32cam_debug and send the backtrace
RESET_REASON,TASK_WDT      ← a task hung: same, send the log
```

If `BOOT,…` does **not** reappear, the board did not reset and the drop is on the client side (phone/laptop
leaving a Wi-Fi network that has no internet, or a weak link — check the antenna selector resistor next to
the u.FL socket points at the PCB antenna, not the empty socket).

Brownout fixes, in order of effect: 5 V ≥ 500 mA supply straight into the 5V pin (not the FTDI's 3V3 pin, not
through a breadboard rail) · a 470–1000 µF electrolytic across 5V/GND right at the board · short, thick leads.
Firmware already limits Wi-Fi TX power to 8.5 dBm (`PAYLOAD_WIFI_TX_POWER`), which halves the TX current spike.

### "Wi-Fi still connected on the laptop, but /status stops answering"

Not a reset (the AP would disappear). Two candidates; one test each, done **while it is lost**:

1. In a terminal on the laptop: `ping 192.168.4.1`
   - **no reply** → the radio link is bad even though the laptop shows "connected". Raise `PAYLOAD_WIFI_TX_POWER`
     to `WIFI_POWER_19_5dBm` (and add the 470–1000 µF cap so it does not brown out), check the antenna-selector
     resistor next to the u.FL socket selects the PCB antenna, move the laptop closer / away from other 2.4 GHz APs
     (`PAYLOAD_WIFI_AP_CHANNEL`: try 1 or 11).
   - **replies** → the Wi-Fi is fine and the HTTP server is wedged → step 2.
2. On the serial monitor send `DIAG` and look at the tail:
   `...,AP_STA,1,HTTP_SOCK,6/1,UPTIME,812`
   - `AP_STA,0` → the ESP32 thinks nobody is associated: radio/driver problem, same fixes as above.
   - `HTTP_SOCK,6/…` (the :80 server at its 6-socket limit) → zombie connections. v3.1 now enables TCP keep-alive
     (dead peers reaped in ~14 s) and 3 s send/recv timeouts, so this should clear itself; if it still sticks, report it.
   - no answer to `DIAG` at all → the firmware is hung: flash `esp32cam_debug`, reproduce, send the log.

Recovery without a power cycle: `WIFI_RESTART` over serial (or from the OBC) stops both HTTP servers and the AP
and brings them back in ~2 s. Stream state is kept.
