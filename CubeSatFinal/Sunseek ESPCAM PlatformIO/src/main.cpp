/* SunSeek ESP32-CAM Payload Firmware v3.1 (PlatformIO port of v3.0)
   Target:        AI Thinker ESP32-CAM + OV2640 + microSD
   Control plane: UART0 (GPIO3 RX / GPIO1 TX) <- SunSeek OBC UART1 (GPIO41 TX / GPIO42 RX), 115200
   Data plane:    Wi-Fi AP "SUNSEEK-PAYLOAD" -> HTTP :80 (API) and :81 (/stream)
   File policy:   Config_* user editable; Module_* developer extension; System_* core.
*/
#include <Arduino.h>
#include "System_Payload.h"

void setup() { payloadSystemBegin(); }
void loop()  { payloadSystemUpdate(); delay(2); }
