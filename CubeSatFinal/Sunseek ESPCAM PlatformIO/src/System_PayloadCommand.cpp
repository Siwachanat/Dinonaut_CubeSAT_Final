#include "System_PayloadCommand.h"
#include "System_PayloadInterface.h"
#include "System_PayloadState.h"
#include "Module_Camera.h"
#include "Module_Storage.h"
#include "Module_PayloadWeb.h"
#include "Config_Payload.h"

void payloadProcessCommand(String c) {
  c.trim();
  c.toUpperCase();

  if (c == "PING") {
    payloadSendLine("PONG,CAMERA");

  } else if (c == "STATUS") {
    payloadSendLine(payloadStatusLine());

  } else if (c == "CAPTURE") {
    String f; size_t z = 0;
    payloadSendLine("EVENT,CAPTURE_STARTED");
    if (storageCapture(f, z)) payloadSendLine("IMAGE_READY," + f + "," + String((unsigned long)z));
    else                      payloadSendLine("ERR,CAPTURE_FAILED");

  } else if (c == "IMAGE_COUNT") {
    payloadSendLine("IMAGE_COUNT," + String(storageImageCount()));

  } else if (c == "LAST_IMAGE") {
    payloadSendLine("LAST_IMAGE," + storageLastImage() + "," + String((unsigned long)storageLastImageSize()));

  } else if (c == "STREAM_START") {
    if (!cameraModuleReady()) {
      payloadSendLine("ERR,STREAM_CAMERA_NOT_READY");
    } else {
      payloadSetStreamEnabled(true);
      payloadSendLine("ACK,STREAM_START");
      payloadSendLine("EVENT,STREAM_ON");
    }

  } else if (c == "STREAM_STOP") {
    payloadSetStreamEnabled(false);
    payloadSendLine("ACK,STREAM_STOP");
    payloadSendLine("EVENT,STREAM_OFF");

  } else if (c == "STREAM_STATUS") {
    payloadSendLine(String("STREAM_STATUS,") + (payloadStreamEnabled() ? "ON" : "OFF"));

  } else if (c == "VERSION") {
    payloadSendLine(String("VERSION,") + PAYLOAD_FW_VERSION);

  } else if (c == "STREAM_URL") {
    payloadSendLine("STREAM_URL," + payloadStreamURL());

  } else if (c == "DIAG") {
    payloadSendLine(payloadDiagLine());

  } else if (c == "WIFI_RESTART") {
    payloadSendLine("ACK,WIFI_RESTART");
    payloadWebRestart();
    payloadSendLine("WIFI_IP," + payloadIP());
    payloadSendLine(payloadStatusLine());

  } else if (c == "STORAGE_REMOUNT") {
    payloadSendLine(String("ACK,STORAGE_REMOUNT,") + (storageModuleRemount() ? "OK" : "ERR"));

  } else {
    payloadSendLine("ERR,UNKNOWN_COMMAND");
  }
}
