#pragma once
// ============================================================
// Module_Storage — microSD (SD_MMC, 1-bit) image store (developer extension layer)
//
// Thread model: storageCapture() and storageModuleBegin() are called from the main (UART) task only.
// The read-only getters are also called from the HTTP server tasks, so the metadata behind them is
// guarded by a mutex and returned by value.
// ============================================================
#include <Arduino.h>

bool     storageModuleBegin();          // mount card, scan existing images
bool     storageModuleRemount();        // unmount + mount again (card inserted after boot)
bool     storageModuleReady();

uint32_t storageImageCount();           // number of *.JPG files in the card root
String   storageLastImage();            // full path of the highest-numbered IMG_NNNN.JPG ("" if none)
size_t   storageLastImageSize();
uint32_t storageNextIndex();            // index the next CAPTURE will use
String   storageCardInfo();             // "SDHC,15193MB" / "NONE" — for DIAG

// Grab one frame from the camera and write it as the next IMG_NNNN.JPG.
// Requires camera AND storage ready. On success fills filename/fileSize and returns true.
bool     storageCapture(String& filename, size_t& fileSize);

// Build "<PAYLOAD_IMAGE_PREFIX>NNNN.JPG" for index n.
String   storageMakeFilename(uint32_t n);
