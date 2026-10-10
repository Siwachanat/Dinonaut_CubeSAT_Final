#pragma once
// ============================================================
// Module_Camera — OV2640 bring-up and frame access (developer extension layer)
// ============================================================
#include <Arduino.h>
#include "esp_camera.h"

// AI Thinker ESP32-CAM + OV2640 pin map
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// Initialise the sensor. Returns true when esp_camera_init() succeeded (possibly on the fallback config).
bool          cameraModuleBegin();
bool          cameraModuleReady();
bool          cameraModuleUsingPsram();     // frame buffers in PSRAM (true) or DRAM (false)
bool          cameraModuleOnFallback();     // true when running on PAYLOAD_FRAME_SIZE_FALLBACK
const char*   cameraModuleFrameSizeName();  // "SVGA", "VGA", ...
esp_err_t     cameraModuleLastError();      // esp_camera_init() result of the last attempt

// Frame access. The driver is task-safe: the UART task (CAPTURE) and the stream server task may both
// call these. Every frame obtained with Get must be handed back with Return.
camera_fb_t*  cameraModuleGetFrame();
void          cameraModuleReturnFrame(camera_fb_t* fb);
