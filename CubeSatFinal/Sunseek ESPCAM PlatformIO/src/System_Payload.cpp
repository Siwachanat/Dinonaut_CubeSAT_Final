#include "System_Payload.h"
#include "Config_Payload.h"
#include "Module_Camera.h"
#include "Module_Storage.h"
#include "Module_PayloadWeb.h"
#include "System_PayloadInterface.h"
#include "System_PayloadState.h"
#include "esp_system.h"

// Why did we (re)boot? Printed right after BOOT so a reset in the field is visible on the OBC link.
static const char* resetReasonName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT_PIN";
    case ESP_RST_SW:        return "SOFTWARE";
    case ESP_RST_PANIC:     return "PANIC";        // crash (Guru Meditation) — debug build prints the backtrace
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";     // supply sagged below ~2.8 V — power problem, not software
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

// Boot order matters:
//   1. UART first so every later step can report.
//   2. Camera before storage — both use the shared GPIO0/XCLK & 20 MHz clocks; the camera driver
//      takes LEDC timer 0 and the DMA I2S peripheral, SD_MMC takes the SDMMC host.
//   3. Storage (1-bit SD_MMC frees GPIO4/12/13).
//   4. Wi-Fi + HTTP last — brings the biggest current spike, after the camera/SD are already stable.
void payloadSystemBegin() {
  payloadInterfaceBegin();
  delay(1000);                                             // let the programmer/OBC UART settle after reset
  payloadSendLine("BOOT,SUNSEEK_PAYLOAD_V" PAYLOAD_FW_VERSION);
  payloadSendLine(String("RESET_REASON,") + resetReasonName());

  cameraModuleBegin();
  storageModuleBegin();

#if PAYLOAD_FLASH_LED_OFF
  // The white flash LED sits on GPIO4 = SD DAT1. In 1-bit mode the SD host never drives DAT1, so the
  // pin can be parked low; otherwise it floats and the LED glows / flickers on SD traffic.
  pinMode(PAYLOAD_FLASH_LED_PIN, OUTPUT);
  digitalWrite(PAYLOAD_FLASH_LED_PIN, LOW);
#endif

  payloadWebBegin();

  payloadSendLine(String("PAYLOAD,") + (payloadReady() ? "READY" : "NOT_READY"));
  payloadSendLine("WIFI_IP," + payloadIP());
  payloadSendLine("STREAM_URL," + payloadStreamURL());
  payloadSendLine(payloadStatusLine());
  payloadSendLine(payloadDiagLine());
}

void payloadSystemUpdate() {
  payloadInterfaceUpdate();
}
