#include "Store_Controller.h"
#include "Can_Ids.h"
#include "Dispatch_Controller.h"

Store_Controller* Store_Controller::instance = NULL;

bool shouldLogAnything = false;

Store_Controller::Store_Controller() 
  : speeds{SENTINAL, SENTINAL, SENTINAL, SENTINAL}
  , responses{false, false}
  , currents{SENTINAL, SENTINAL}
  , analogThrottle(0)
  , analogBrake(0)
  , outputTorque(0)
  , tractiveVoltageOn(false)
  , brakeThrottleConflict(false)
  , bmsTemp(SENTINAL)
  , bmsCurrent(SENTINAL)
  , bmsVoltage(SENTINAL)
  , soc(SENTINAL)
{
  // No initialization required
}

template <typename First>
void logOne(const First& first) {
  if (!shouldLogAnything) {
    return;
  }
  Serial.print(String(first));
  Serial.print(", ");
  Serial.println(millis());

  Serial2.print(String(first));
  Serial2.print(", ");
  Serial2.println(millis());
}

template <typename First, typename Second>
void logTwo(const First& first, const Second& second) {
  if (!shouldLogAnything) {
    return;
  }
  Serial.print(String(first));
  Serial.print(", ");
  Serial.print(String(second));
  Serial.print(", ");
  Serial.println(millis());

  Serial2.print(String(first));
  Serial2.print(", ");
  Serial2.print(String(second));
  Serial2.print(", ");
  Serial2.println(millis());
}

template <typename First, typename Second, typename Third>
void logThree(const First& first, const Second& second, const Third& third) {
  if (!shouldLogAnything) {
    return;
  }
  Serial.print(String(first));
  Serial.print(", ");
  Serial.print(String(second));
  Serial.print(", ");
  Serial.print(String(third));
  Serial.print(", ");
  Serial.println(millis());

  Serial2.print(String(first));
  Serial2.print(", ");
  Serial2.print(String(second));
  Serial2.print(", ");
  Serial2.print(String(third));
  Serial2.print(", ");
  Serial2.println(millis());
}

template <typename First, typename Second, typename Third, typename Fourth>
void logFour(const First& first, const Second& second, const Third& third, const Fourth& fourth) {
  if (!shouldLogAnything) {
    return;
  }
  Serial.print(String(first));
  Serial.print(", ");
  Serial.print(String(second));
  Serial.print(", ");
  Serial.print(String(third));
  Serial.print(", ");
  Serial.print(String(fourth));
  Serial.print(", ");
  Serial.println(millis());

  Serial2.print(String(first));
  Serial2.print(", ");
  Serial2.print(String(second));
  Serial2.print(", ");
  Serial2.print(String(third));
  Serial2.print(", ");
  Serial2.print(String(fourth));
  Serial2.print(", ");
  Serial2.println(millis());
}

template <typename First, typename Second, typename Third, typename Fourth, typename Fifth>
void logFive(const First& first, const Second& second, const Third& third, const Fourth& fourth, const Fifth& fifth) {
  if (!shouldLogAnything) {
    return;
  }
  Serial.print(String(first));
  Serial.print(", ");
  Serial.print(String(second));
  Serial.print(", ");
  Serial.print(String(third));
  Serial.print(", ");
  Serial.print(String(fourth));
  Serial.print(", ");
  Serial.print(String(fifth));
  Serial.print(", ");
  Serial.println(millis());

  Serial2.print(String(first));
  Serial2.print(", ");
  Serial2.print(String(second));
  Serial2.print(", ");
  Serial2.print(String(third));
  Serial2.print(", ");
  Serial2.print(String(fourth));
  Serial2.print(", ");
  Serial2.print(String(fifth));
  Serial2.print(", ");
  Serial2.println(millis());
}

Store_Controller& Store_Controller::readInstance() {
  if(!instance) {
    instance = new Store_Controller();
  }
  return *instance;
}

Store_Controller& Store() {
  return Store_Controller::readInstance();
}

void Store_Controller::logSpeed(const Wheel wheel, const int16_t rpm) {
  speeds[wheel] = rpm;
  String wheelName = "";
  switch(wheel) {
    case FrontRightWheel:
      wheelName = "front_right";
      break;
    case FrontLeftWheel:
      wheelName = "front_left";
      break;
    case RearRightWheel:
      wheelName = "rear_right";
      break;
    case RearLeftWheel:
      wheelName = "rear_left";
      break;
    default:
      //Should never happen
      return;
  }
  logFour("wheel_speed", wheelName, rpm, "rpm");
}
int16_t Store_Controller::readSpeed(const Wheel wheel) {
  return speeds[wheel];
}

void Store_Controller::logAnalogThrottle(const uint8_t throttle) {
  analogThrottle = throttle;
  // logThree("analog_throttle", throttle, "uint8_units");
}
uint8_t Store_Controller::readAnalogThrottle() {
  return analogThrottle;
}

void Store_Controller::logAnalogBrake(const uint8_t brake) {
  analogBrake = brake;
}
uint8_t Store_Controller::readAnalogBrake() {
  return analogBrake;
}

void Store_Controller::logOutputTorque(const int16_t torque) {
  outputTorque = torque;
}
int16_t Store_Controller::readOutputTorque() {
  return outputTorque;
}

void Store_Controller::logBrakeThrottleConflict(const bool conflict) {
  brakeThrottleConflict = conflict;
}
bool Store_Controller::readBrakeThrottleConflict() {
  return brakeThrottleConflict;
}

void Store_Controller::logTractiveVoltage(bool _tractiveVoltageOn) {
  tractiveVoltageOn = _tractiveVoltageOn;
}
bool Store_Controller::readTractiveVoltage() {
  return tractiveVoltageOn;
}

void Store_Controller::logBmsTemp(const int16_t _bmsTemp) {
  bmsTemp = _bmsTemp;
  logThree("bms_temp", bmsTemp, "degrees");
}
int16_t Store_Controller::readBmsTemp() {
  return bmsTemp;
}

void Store_Controller::logBmsCurrent(const int16_t _bmsCurrent) {
  bmsCurrent = _bmsCurrent;
  logThree("bms_current", bmsCurrent, "amps");
}
int16_t Store_Controller::readBmsCurrent() {
  return bmsCurrent;
}

void Store_Controller::logBmsVoltage(const int16_t _bmsVoltage) {
  bmsVoltage = _bmsVoltage;
  logThree("bms_voltage", bmsVoltage, "volts");
}
int16_t Store_Controller::readBmsVoltage() {
  return bmsVoltage;
}

void Store_Controller::logSoc(const int16_t _soc) {
  soc = _soc;
}
int16_t Store_Controller::readSoc() {
  return soc;
}


void Store_Controller::logMotorCurrent(MotorController controller, int16_t current) {
  currents[controller] = current;
  String motor_name = (controller == RightMotor) ? "right" : "left";
  logFour("motor_current", motor_name, current, "int16_units");
}
int16_t Store_Controller::readMotorCurrent(MotorController controller) {
  return currents[controller];
}

void Store_Controller::logMotorResponse(MotorController dir) {
  responses[dir] = true;
}
bool Store_Controller::readMotorResponse(MotorController dir) {
  return responses[dir];
}

String motor_warnings[16] = {
  "parameter_damaged",
  "igbt_error",
  "should_never_happen",
  "can_timeout",
  "faulty_resolver",
  "under_voltage",
  "over_temp",
  "over_current",
  "current_out_of_tolerance",
  "3x_over_current",
  "raceaway",
  "can_hardware_error",
  "adc_error",
  "faulty_encoder",
  "software_error",
  "ballast_overload"
};
void Store_Controller::logMotorErrors(MotorController dir, uint16_t error_string) {
  String motor_name = (dir == RightMotor) ? "right" : "left";
  for(int i = 0; i < 16; i++) {
    if (bitRead(error_string, i)) {
      String error_name = motor_warnings[i];
      logThree("motor_warning", motor_name, error_name);
    }
  }
}

String bms_faults[8] = {
  "driving_off",
  "interlock_tripped",
  "comm_fault",
  "charge_overcurrent",
  "discharge_overcurrent",
  "over_temp",
  "under_voltage",
  "over_voltage"
};
void Store_Controller::logBmsFaults(uint8_t fault_string) {
  for(int i = 0; i < 8; i++) {
    if (bitRead(fault_string, i)) {
      String fault_name = bms_faults[i];
      logTwo("bms_fault", fault_name);
    }
  }
}

String bms_warnings[8] = {
  "low_voltage",
  "high_voltage",
  "charge_overcurrent",
  "discharge_overcurrent",
  "cold_temperature",
  "hot_temperature",
  "low_soh",
  "isolation_fault"
};
void Store_Controller::logBmsWarnings(uint8_t warning_string) {
  for(int i = 0; i < 8; i++) {
    if (bitRead(warning_string, i)) {
      String warning_name = bms_warnings[i];
      logTwo("bms_warning", warning_name);
    }
  }
}
