#include "Module_Storage.h"
#include "Module_Camera.h"
#include "Config_Payload.h"
#include "SD_MMC.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

static SemaphoreHandle_t _lock          = nullptr;
static bool              _storageReady  = false;
static String            _lastImage     = "";
static size_t            _lastImageSize = 0;
static uint32_t          _imageCount    = 0;
static uint32_t          _nextIndex     = 1;

static void lock()   { if (_lock) xSemaphoreTake(_lock, portMAX_DELAY); }
static void unlock() { if (_lock) xSemaphoreGive(_lock); }

// Prefix without the leading slash, e.g. "IMG_" — File::name() returns a bare basename on
// Arduino-ESP32 2.x and 3.x, a full path on 1.x. Both are handled.
static String basenameOf(const String& n) {
  int slash = n.lastIndexOf('/');
  return slash >= 0 ? n.substring(slash + 1) : n;
}
static String prefixBase() {
  String p = PAYLOAD_IMAGE_PREFIX;
  while (p.startsWith("/")) p.remove(0, 1);
  return p;
}

String storageMakeFilename(uint32_t n) {
  char b[48];
  snprintf(b, sizeof(b), "%s%04lu.JPG", PAYLOAD_IMAGE_PREFIX, (unsigned long)n);
  String s(b);
  if (!s.startsWith("/")) s = "/" + s;
  return s;
}

// Parse "IMG_0012.JPG" -> 12. Returns false for any other *.JPG.
static bool parseIndex(const String& base, uint32_t& idx) {
  String pb = prefixBase();
  if (!base.startsWith(pb)) return false;
  String digits = base.substring(pb.length(), base.length() - 4);   // strip prefix and ".JPG"
  if (digits.length() == 0) return false;
  for (size_t i = 0; i < digits.length(); i++) if (!isDigit(digits[i])) return false;
  idx = (uint32_t)strtoul(digits.c_str(), nullptr, 10);
  return true;
}

static void storageScan() {
  uint32_t count = 0, maxIdx = 0;
  String   last  = "";
  size_t   lastSize = 0;

  File root = SD_MMC.open("/");
  if (root) {
    File f = root.openNextFile();
    while (f) {
      String base = basenameOf(String(f.name()));
      String u = base; u.toUpperCase();
      if (!f.isDirectory() && u.endsWith(".JPG")) {
        count++;
        uint32_t idx;
        if (parseIndex(u, idx) && idx >= maxIdx) {   // highest index wins, not directory order
          maxIdx   = idx;
          last     = "/" + base;
          lastSize = f.size();
        }
      }
      f = root.openNextFile();
    }
    root.close();
  }

  lock();
  _imageCount    = count;
  _lastImage     = last;
  _lastImageSize = lastSize;
  _nextIndex     = maxIdx + 1;
  unlock();
}

bool storageModuleBegin() {
  if (!_lock) _lock = xSemaphoreCreateMutex();
  _storageReady = SD_MMC.begin("/sdcard", /*mode1bit=*/true, /*format_if_mount_failed=*/false,
                               PAYLOAD_SD_FREQ_KHZ);
  if (_storageReady) storageScan();
  return _storageReady;
}

bool storageModuleRemount() {
  if (_storageReady) { SD_MMC.end(); _storageReady = false; }
  delay(50);
  return storageModuleBegin();
}

bool     storageModuleReady()   { return _storageReady; }
uint32_t storageImageCount()    { lock(); uint32_t v = _imageCount;    unlock(); return v; }
String   storageLastImage()     { lock(); String   v = _lastImage;     unlock(); return v; }
size_t   storageLastImageSize() { lock(); size_t   v = _lastImageSize; unlock(); return v; }
uint32_t storageNextIndex()     { lock(); uint32_t v = _nextIndex;     unlock(); return v; }

String storageCardInfo() {
  if (!_storageReady) return "NONE";
  const char* t = "UNKNOWN";
  switch (SD_MMC.cardType()) {
    case CARD_NONE: t = "NONE"; break;
    case CARD_MMC:  t = "MMC";  break;
    case CARD_SD:   t = "SD";   break;
    case CARD_SDHC: t = "SDHC"; break;
    default: break;
  }
  return String(t) + "," + String((unsigned long)(SD_MMC.cardSize() / (1024ULL * 1024ULL))) + "MB";
}

bool storageCapture(String& filename, size_t& fileSize) {
  filename = ""; fileSize = 0;
  if (!_storageReady || !cameraModuleReady()) return false;

  camera_fb_t* fb = cameraModuleGetFrame();
  if (!fb) return false;
  if (fb->format != PIXFORMAT_JPEG || fb->len == 0) { cameraModuleReturnFrame(fb); return false; }

  uint32_t next = storageNextIndex();
  String   p    = storageMakeFilename(next);
  while (SD_MMC.exists(p)) { next++; p = storageMakeFilename(next); }   // never overwrite

  File f = SD_MMC.open(p, FILE_WRITE);
  if (!f) { cameraModuleReturnFrame(fb); return false; }
  const size_t expected = fb->len;
  size_t w = f.write(fb->buf, fb->len);
  f.close();
  cameraModuleReturnFrame(fb);                 // hand the buffer back as early as possible

  if (w != expected) {                         // short write = card full / write error: drop the partial file
    SD_MMC.remove(p);
    return false;
  }

  lock();
  _imageCount++;
  _nextIndex     = next + 1;
  _lastImage     = p;
  _lastImageSize = w;
  unlock();

  filename = p;
  fileSize = w;
  return true;
}
