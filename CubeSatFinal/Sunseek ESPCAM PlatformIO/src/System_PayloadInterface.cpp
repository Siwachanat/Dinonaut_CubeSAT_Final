#include "System_PayloadInterface.h"
#include "System_PayloadCommand.h"
#include "Config_Payload.h"
#include "esp_log.h"

static String _rx;
static bool   _overflow = false;

void payloadSendLine(const String& s) { Serial.println(s); }

void payloadInterfaceBegin() {
  Serial.begin(PAYLOAD_UART_BAUD);
#if PAYLOAD_SILENCE_SYSTEM_LOGS
  // UART0 is the OBC control link. Anything that is not a protocol line is noise for the OBC parser,
  // so turn off the Arduino debug output and every ESP-IDF log tag (camera, sdmmc, wifi, httpd ...).
  // The ROM bootloader banner ("rst:0x1 (POWERON_RESET) ...") is printed before this code runs and
  // cannot be suppressed — the OBC must ignore lines it does not recognise.
  Serial.setDebugOutput(false);
  esp_log_level_set("*", ESP_LOG_NONE);
#endif
  _rx.reserve(PAYLOAD_UART_LINE_MAX + 1);
}

void payloadInterfaceUpdate() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (_overflow)             payloadSendLine("ERR,LINE_TOO_LONG");
      else if (_rx.length())     payloadProcessCommand(_rx);
      _rx = "";
      _overflow = false;
    } else if (_overflow) {
      continue;                                  // discard the rest of an oversize line
    } else if (_rx.length() < PAYLOAD_UART_LINE_MAX) {
      _rx += c;
    } else {
      _overflow = true;                          // v3.0 silently split long lines into two commands
    }
  }
}
