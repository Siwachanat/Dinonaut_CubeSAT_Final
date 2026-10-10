#include "Module_Camera.h"
#include "Config_Camera.h"

static bool        _cameraReady   = false;
static bool        _usingPsram    = false;
static bool        _onFallback    = false;
static framesize_t _frameSize     = PAYLOAD_FRAME_SIZE;
static esp_err_t   _lastErr       = ESP_OK;

static void fillConfig(camera_config_t& c, framesize_t fs, bool psram) {
  c = camera_config_t{};
  c.ledc_channel  = LEDC_CHANNEL_0;
  c.ledc_timer    = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk      = XCLK_GPIO_NUM;
  c.pin_pclk      = PCLK_GPIO_NUM;
  c.pin_vsync     = VSYNC_GPIO_NUM;
  c.pin_href      = HREF_GPIO_NUM;
  c.pin_sccb_sda  = SIOD_GPIO_NUM;
  c.pin_sccb_scl  = SIOC_GPIO_NUM;
  c.pin_pwdn      = PWDN_GPIO_NUM;
  c.pin_reset     = RESET_GPIO_NUM;
  c.xclk_freq_hz  = PAYLOAD_XCLK_HZ;
  c.pixel_format  = PIXFORMAT_JPEG;
  c.frame_size    = fs;
  c.jpeg_quality  = PAYLOAD_JPEG_QUALITY;
  // Two buffers in PSRAM let the stream server and a UART CAPTURE each hold a frame at the same time.
  c.fb_count      = psram ? 2 : 1;
  c.fb_location   = psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.grab_mode     = CAMERA_GRAB_LATEST;       // always deliver the newest frame, never a stale one
}

static bool tryInit(framesize_t fs, bool psram) {
  camera_config_t c;
  fillConfig(c, fs, psram);
  _lastErr = esp_camera_init(&c);           // on failure the driver deinitialises itself
  if (_lastErr != ESP_OK) return false;
  _frameSize  = fs;
  _usingPsram = psram;
  return true;
}

bool cameraModuleBegin() {
  const bool psram = psramFound();
  _cameraReady = tryInit(PAYLOAD_FRAME_SIZE, psram);
#if PAYLOAD_CAMERA_FALLBACK
  if (!_cameraReady) {
    delay(100);
    _cameraReady = tryInit(PAYLOAD_FRAME_SIZE_FALLBACK, false);
    _onFallback  = _cameraReady;
  }
#endif
  if (_cameraReady) {
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
      s->set_vflip(s, PAYLOAD_CAMERA_VFLIP);
      s->set_hmirror(s, PAYLOAD_CAMERA_HMIRROR);
    }
  }
  return _cameraReady;
}

bool      cameraModuleReady()       { return _cameraReady; }
bool      cameraModuleUsingPsram()  { return _usingPsram; }
bool      cameraModuleOnFallback()  { return _onFallback; }
esp_err_t cameraModuleLastError()   { return _lastErr; }

const char* cameraModuleFrameSizeName() {
  switch (_frameSize) {
    case FRAMESIZE_QVGA: return "QVGA";
    case FRAMESIZE_CIF:  return "CIF";
    case FRAMESIZE_VGA:  return "VGA";
    case FRAMESIZE_SVGA: return "SVGA";
    case FRAMESIZE_XGA:  return "XGA";
    case FRAMESIZE_SXGA: return "SXGA";
    case FRAMESIZE_UXGA: return "UXGA";
    default:             return "OTHER";
  }
}

camera_fb_t* cameraModuleGetFrame()                  { return _cameraReady ? esp_camera_fb_get() : nullptr; }
void         cameraModuleReturnFrame(camera_fb_t* fb) { if (fb) esp_camera_fb_return(fb); }
