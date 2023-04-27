// Header file containing hardware configurations for the Arduino Due mapping to the linear actuators.
// This code has been taken from the ENPH 459 Engineering Physics group from the University of British Columbia
// and modified by Trent Peterson to work with the Cal Poly ME 423 Stewart Platform robot and the Stewart Platform Motor Driver PCB hat.

#pragma once

// PWM pins
#define PWM_1 13
#define PWM_2 12
#define PWM_3 11
#define PWM_4 10
#define PWM_5 9
#define PWM_6 8
// DIRection pins
#define DIR_1 7 
#define DIR_2 6
#define DIR_3 5
#define DIR_4 4
#define DIR_5 3
#define DIR_6 2

// PA-14P Potentiometer pins
#define POT_1 A0
#define POT_2 A1
#define POT_3 A2
#define POT_4 A3
#define POT_5 A4
#define POT_6 A5

// Enable for motors
#define DISABLE_MOTORS 61 
//This pin is jumped to, because the original board design connects to TX0, which is used for serial communication (pin 1)

