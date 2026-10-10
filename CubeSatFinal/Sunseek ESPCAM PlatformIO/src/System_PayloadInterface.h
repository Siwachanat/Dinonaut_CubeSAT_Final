#pragma once
// ============================================================
// System_PayloadInterface — UART0 line protocol to the OBC (core layer)
//
// Wire format: ASCII, one command per line, terminated by CR, LF or CRLF.
// Responses are comma-separated lines terminated by CRLF (Serial.println).
// ============================================================
#include <Arduino.h>

void payloadInterfaceBegin();            // Serial.begin + log silencing
void payloadInterfaceUpdate();           // call every loop(): drains RX, dispatches complete lines
void payloadSendLine(const String& s);   // only from the main task
