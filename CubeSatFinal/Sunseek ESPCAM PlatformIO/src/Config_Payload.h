#pragma once
// ============================================================
// SunSeek ESP32-CAM Payload v3.1 — USER / WORKSHOP CONFIG
// Safe to edit for normal workshop deployment.
// ============================================================

#ifndef PAYLOAD_FW_VERSION
#define PAYLOAD_FW_VERSION         "3.1"           // normally injected from platformio.ini
#endif

// ---- Wi-Fi (data plane) ----
#define PAYLOAD_WIFI_AP_MODE       1               // 1 = payload is its own access point, 0 = join PAYLOAD_STA_* network
#define PAYLOAD_WIFI_SSID          "DinoCAM"
#define PAYLOAD_WIFI_PASSWORD      "12345678"     // >= 8 chars (WPA2 minimum); shorter = open AP
#define PAYLOAD_WIFI_AP_CHANNEL    6
#define PAYLOAD_STA_SSID           "YOUR_WIFI"
#define PAYLOAD_STA_PASSWORD       "YOUR_PASSWORD"
#define PAYLOAD_STA_CONNECT_TIMEOUT_MS 15000
// Wi-Fi transmit power. The ESP32-CAM's 5 V → 3.3 V regulator sags on the TX burst that follows a capture;
// 8.5 dBm is plenty for a ground station in the same room and is the usual fix for "AP drops after capture".
// Raise to WIFI_POWER_11dBm / WIFI_POWER_19_5dBm only with a solid supply (and a 470–1000 µF cap on 5 V).
#define PAYLOAD_WIFI_TX_POWER      WIFI_POWER_8_5dBm

// ---- UART0 (control plane, shared with the programmer: GPIO3 RX / GPIO1 TX) ----
#define PAYLOAD_UART_BAUD          115200
#define PAYLOAD_UART_LINE_MAX      95              // longest accepted command line (chars, without CR/LF)
#ifndef PAYLOAD_SILENCE_SYSTEM_LOGS
#define PAYLOAD_SILENCE_SYSTEM_LOGS 1              // 1 = suppress ESP-IDF/Arduino log lines on UART0 after boot (debug env sets 0)
#endif

// ---- HTTP ----
#define PAYLOAD_HTTP_PORT          80              // /, /status, /images, /image
#define PAYLOAD_STREAM_PORT        81              // /stream lives on its own server so it cannot block the port-80 API
#define PAYLOAD_STREAM_DEFAULT_ON  0               // stream state at boot (0 = OFF until STREAM_START)
#define PAYLOAD_STREAM_FRAME_MS    80              // pause between MJPEG frames (80 ms ~ 12 fps cap)

// ---- Storage ----
#define PAYLOAD_IMAGE_PREFIX       "/IMG_"         // files are <prefix>NNNN.JPG in the card root
#define PAYLOAD_SD_FREQ_KHZ        20000           // 20 MHz is the safe choice for 1-bit mode on no-name cards; 40000 = max

// ---- Board ----
#define PAYLOAD_FLASH_LED_OFF      1               // 1 = hold the white flash LED (GPIO4) off. SD runs in 1-bit mode so GPIO4 is free.
#define PAYLOAD_FLASH_LED_PIN      4
