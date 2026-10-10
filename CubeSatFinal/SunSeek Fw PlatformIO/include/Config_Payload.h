#pragma once
/*
  SunSeek Platform v2.0
  TRAINING LEVEL: INTERMEDIATE
  Payload interface parameters. Change only when the payload interface changes.
*/
#include <Arduino.h>

/* T05 Payload UART — hardware-validated target wiring
   SunSeek GPIO41 TX  -> ESP32-CAM GPIO3 / RX0
   SunSeek GPIO42 RX  <- ESP32-CAM GPIO1 / TX0
   Common GND required.
*/
#define PAYLOAD_UART_BAUD 115200
#define PAYLOAD_UART_TX_PIN 41
#define PAYLOAD_UART_RX_PIN 42
#define PAYLOAD_UART_MAX_LINE 220
