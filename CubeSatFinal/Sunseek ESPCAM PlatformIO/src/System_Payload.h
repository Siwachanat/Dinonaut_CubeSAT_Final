#pragma once
// ============================================================
// System_Payload — boot sequence and main loop (core layer)
// ============================================================
#include <Arduino.h>

void payloadSystemBegin();    // called once from setup()
void payloadSystemUpdate();   // called from loop()
