#include "Rtd_Controller.h"
#include "Pins.h"
#include "Logger.h"

// Must define instance prior to use
Rtd_Controller* Rtd_Controller::instance = NULL;

// Private constructor
Rtd_Controller::Rtd_Controller()
: enabled(false),
  begun(false)
{
  // Initialization done above
}

void Rtd_Controller::begin() {
  if(begun) {
    return;
  }
  begun = true;
  pinMode(SHUTDOWN_CIRCUIT_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_CIRCUIT_PIN, HIGH);

  pinMode(BSPD_LATCHED_PIN, OUTPUT);
  digitalWrite(BSPD_LATCHED_PIN, HIGH);

  pinMode(MC_ENABLE_PIN_1, OUTPUT);
  pinMode(MC_ENABLE_PIN_2, OUTPUT);
  pinMode(MC_ENABLE_PIN_3, OUTPUT);
  pinMode(MC_ENABLE_PIN_4, OUTPUT);

  setEnablePins(LOW);

  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);
  analogWrite(FAN_PIN, 128);
}

Rtd_Controller& Rtd_Controller::getInstance() {
  if(!instance) {
    instance = new Rtd_Controller();
    instance->begin();
  }
  return *instance;
}

Rtd_Controller& RTD() {
  return Rtd_Controller::getInstance();
}

void Rtd_Controller::enable() {
  enabled = true;
  setEnablePins(HIGH);
}

void Rtd_Controller::disable() {
  enabled = false;
  setEnablePins(LOW);
}

void Rtd_Controller::setEnablePins(uint8_t direction) {
  if (direction == LOW) {
    // Set pins 4-7 to low, all others stay the same
    // (note that pins are ordered as 0b7...0)
    PORTK = PORTK & 0b00001111;
  }
  else if (direction == HIGH) {
    // Set pins 4-7 to high, all others stay the same
    // (note that pins are ordered as 0b7...0)
    PORTK = PORTK | 0b11110000;
  }
}

void Rtd_Controller::shutdown(String reason = "") {
  disable();
  Computer().logTwo("vehicle_shutdown", reason);
  Xbee().logTwo("vehicle_shutdown", reason);
  digitalWrite(SHUTDOWN_CIRCUIT_PIN, LOW);
}

bool Rtd_Controller::isEnabled() {
  return enabled;
}
