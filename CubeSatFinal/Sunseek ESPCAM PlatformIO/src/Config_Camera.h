#pragma once
#include "esp_camera.h"
// ============================================================
// SunSeek ESP32-CAM Payload v3.1 — CAMERA CONFIG
// Safe to edit after hardware bring-up is verified.
// ============================================================
#define PAYLOAD_JPEG_QUALITY        30              // 0..63, lower = better quality / bigger file (10..12 is the sweet spot for OV2640)
#define PAYLOAD_FRAME_SIZE          FRAMESIZE_SVGA  // 800x600. UXGA (1600x1200) works but ~4x the bytes per capture/stream frame.
#define PAYLOAD_XCLK_HZ             20000000

// If esp_camera_init() fails with the settings above (typically: no PSRAM detected, or not enough free
// DRAM) the driver is re-initialised once with this smaller frame size and a single frame buffer in DRAM.
#define PAYLOAD_CAMERA_FALLBACK     1
#define PAYLOAD_FRAME_SIZE_FALLBACK FRAMESIZE_VGA   // 640x480

// Orientation fixes for how the module is mounted in the CubeSat (0 / 1)
#define PAYLOAD_CAMERA_VFLIP        0
#define PAYLOAD_CAMERA_HMIRROR      0
