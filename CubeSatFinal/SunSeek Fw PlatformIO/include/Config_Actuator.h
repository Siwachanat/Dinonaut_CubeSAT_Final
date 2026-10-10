#pragma once

/*
  Config_Actuator.h — carried forward from T01

  TRAINING LEVEL: BASIC — actuator parameters intended for learner adjustment.

  This file belongs to YOUR spacecraft.
  Keep the values your team obtained during T01.
*/

/* Competition kit: flywheel motor on TB6612FNG channel A
   GPIO1  -> PWMA (analogWrite, 8-bit; core default ~1 kHz)
   GPIO38 -> AIN1
   GPIO39 -> AIN2
   TB6612 STBY must be HIGH (tied on the board, or wire it to 3V3). */
#define RW_PIN_PWMA  1
#define RW_PIN_AIN1  38
#define RW_PIN_AIN2  39

/* [PIO port] The board's ULN2003 stepper driver (IN1..IN4 = GPIO15/6/7/18) is
   not used by this firmware. Drive its inputs LOW at boot so a connected
   stepper cannot be half-energised by floating pins. Set to 0 to disable. */
#define BOARD_PARK_ULN2003_PINS 1

#define RW_DEFAULT_BIAS_PERCENT   40
#define RW_MIN_START_PERCENT      5
#define RW_DEFAULT_ASSIST_PERCENT 80
