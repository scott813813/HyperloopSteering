// Header file containing software configurations, constants, and clarifying definitions for the Arduino.
// This code has been taken from the ENPH 459 Engineering Physics group from the University of British Columbia
// and modified by Trent Peterson to work with the Cal Poly ME 423 Stewart Platform robot and the Stewart Platform Motor Driver PCB hat.

# pragma once

#include "pin_layout.h"

// Platform parameters
#define NUM_MOTORS 6

// Actuator value bounds
#define MIN_POS 0
#define MAX_POS 1023 // 10 bit ADC resolution, can be increased to 12 bit if necessary (4095)
#define MIN_PWM 0
#define MAX_PWM 255 // 8 bit PWM resolution, default, can be increased to 12 bit if necessary (4095)

// Default platform calibration settings (average analog values at extrema for each actuator)
#define OFF_THRESHOLD 100  // ignore calibration if motors aren't powered (i.e. large reading difference from default is found), originally set to 50, change end pos later
int16_t ZERO_POS[NUM_MOTORS]  = { 989, 987, 989, 990, 989, 990 }; 
int16_t END_POS[NUM_MOTORS]   = { 707, 699, 697, 698, 696, 696 };

// Pin group arrays; each value corresponding to the actuator (see "pin_layout.h" for specific values)
const uint8_t DIR_PINS[NUM_MOTORS] = { DIR_1, DIR_2, DIR_3, DIR_4, DIR_5, DIR_6 };
const uint8_t PWM_PINS[NUM_MOTORS] = { PWM_1, PWM_2, PWM_3, PWM_4, PWM_5, PWM_6 };
const uint8_t POT_PINS[NUM_MOTORS] = { POT_1, POT_2, POT_3, POT_4, POT_5, POT_6 }; // Red is POT, Yellow is GND, and White is 5V (3V3 on accident)

// Movement parameters
#define RESET_DELAY 15000      // at full PWM, the actuator should fully extend/retract by 5s (8" stroke, 2.00"/s with minimal load)
typedef enum _MotorDirection  // to clarify the direction in which actuators move
{
    RETRACT = 1,
    EXTEND  = 0
} MotorDirection;

// PID feedback parameters
const uint8_t POS_THRESHOLD[NUM_MOTORS] = { 2, 2, 2, 2, 2, 2 };
const float P_COEFF[NUM_MOTORS]         = { 70, 70, 70, 70, 70, 70 };
const float I_COEFF[NUM_MOTORS]         = { 0.09, 0.09, 0.09, 0.09, 0.09, 0.09 };

//const float I_COEFF[NUM_MOTORS]         = { 0.0275, 0.0275, 0.0275, 0.0275, 0.0275, 0.0275 };
const float D_COEFF[NUM_MOTORS]         = { 0.02, 0.02, 0.02, 0.02, 0.02, 0.02 };

// Serial configuration parameters
#define BAUD 115200  // baud  for serial port (also needs to be set on host side)

// Serial input parameters
#define INPUT_TRIGGER 14   // at minimum, 6 numbers + 5 spaces (0 0 0 0 0 0 20)
#define NUM_READINGS  255  // number of analog readings to average to acquire position

// Serial print/output parameters
// NOTE: feedback measurement settings: PRINT_INTERVAL = 10, ENABLE_PRINT_HEADERS = 0, only PRINT_CURRENT_POS
#define PRINT_INTERVAL          200                 // minimum time (ms) between printing serial info
#define ENABLE_PRINT            0                  // flag to enable printing variables (for debugging)
#define ENABLE_PRINT_HEADERS    ENABLE_PRINT && 1  // flag to enable variable headers (when not running analysis)
#define PRINT_DESIRED_POS       ENABLE_PRINT && 1  // print desired position values when printing serial info
#define PRINT_CURRENT_POS       ENABLE_PRINT && 1  // print current position values when printing serial info
#define PRINT_PWM               ENABLE_PRINT && 1  // print PWM values when printing serial info
#define PRINT_DIR               ENABLE_PRINT && 1  // print direction of motors (1 or 0 according to MotorDirection)