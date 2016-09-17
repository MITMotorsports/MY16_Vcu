#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

const int MCP_INT_PIN = 2; // PE4, INT4, OC3B, pin_6
const int MCP_CS_PIN = 53; // PB0, PCINT0, SS, pin_19

const int BRAKE_LIGHT_PIN = 62; //A8, PCINT16, PK0, pin_89;
const int BSPD_LATCHED_PIN = 63; //A9, PCINT17, PK1, pin_88
const int FAN_PIN = 64; //A10, PCINT18, PK2, pin_87
const int SHUTDOWN_CIRCUIT_PIN = 65; // A11, PCINT19, PK3, pin_86

// IMPORTANT: If you change any of these four pins, you must update the Rtd_Controller::setEnablePins function and the Rtd_Controller::begin function
const int MC_ENABLE_PIN_1 = 66; // pin_85
const int MC_ENABLE_PIN_2 = 67; // pin_84
const int MC_ENABLE_PIN_3 = 68; // pin_83
const int MC_ENABLE_PIN_4 = 69; // pin_82

#endif

