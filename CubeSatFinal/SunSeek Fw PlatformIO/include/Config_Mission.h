#pragma once
#include <Arduino.h>

/*
  Config_Mission.h — [PIO port v3.0-pio] Mission advance settings
  TRAINING LEVEL: INTERMEDIATE

  The organisers' Mission Manager stops in CAPTURING after the first target.
  Module_Mission.h (port) completes the loop:

    ACQUIRING → HOLDING → CAPTURING → IMAGE_READY → (next target) ACQUIRING … → COMPLETE

  CAPTURING sends CAPTURE to the ESP32-CAM over UART1 and waits for
  IMAGE_READY,<name>,<bytes>. The ADCS stays in AUTO on the current target
  the whole time; the next target is loaded without leaving AUTO, so the
  wheel never stops between targets (important for MOMENTUM strategy).
  After the last target the mission is COMPLETE and the spacecraft keeps
  holding the last target (competition rule: no automatic RW STOP).
*/

// Default for a target uploaded WITHOUT the capture field (MISSION_TARGET,<angle>,<tol>,<hold>):
// 1 = send CAPTURE and wait for IMAGE_READY at that target. 0 = skip the camera and go
// straight to the next target after the hold (useful for rig tests).
// [Team 2026-10-08] A target uploaded with the 4th field, MISSION_TARGET,<angle>,<tol>,<hold>,<0|1>,
// decides for itself: 1 = capture, 0 = no capture (EVT,MISSION_CAPTURE_SKIPPED,<i>).
#define MISSION_CAPTURE_ENABLE 1

// How long to wait for IMAGE_READY before giving up on this capture [ms].
// The payload v3.1 answers in ~1–3 s; SD-card writes can take longer.
#define MISSION_CAPTURE_TIMEOUT_MS 8000UL

// Extra CAPTURE attempts after a timeout or PAYLOAD ERR (0 = none).
#define MISSION_CAPTURE_RETRIES 1

// 1 = after all retries fail, carry on to the next target anyway (pointing
// score is kept). 0 = mission state FAILED (ADCS stays AUTO on this target).
#define MISSION_CONTINUE_ON_CAPTURE_FAIL 1

// Settle time between IMAGE_READY and loading the next target [ms].
#define MISSION_NEXT_TARGET_DELAY_MS 200UL
