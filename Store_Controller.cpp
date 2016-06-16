#include "Store_Controller.h"
#include "Can_Ids.h"
#include "Dispatch_Controller.h"
#include "Logger.h"

// Value for indicating that there is not yet a reading
const int16_t SENTINAL = -32768;
const String NO_DIRECTION = "only";

Store_Controller::Store_Controller() 
  // Can node logging
  : analogThrottle(0)
  , analogBrake(0)
  , brakeThrottleConflict(false)
  , outputTorque(0)

  // Wheel logging
  , speeds{SENTINAL, SENTINAL, SENTINAL, SENTINAL}

  // Motor controller states
  , responses{false, false}
  , errors{false, false}
  , highVoltage{false, false}
  , lowVoltage{false, false}
  , motorRpm{SENTINAL, SENTINAL}

  // Motor controller readings
  , currents{SENTINAL, SENTINAL}
  , currentCommands{SENTINAL, SENTINAL}

  // BMS readings
  , bmsTemp(SENTINAL)
  , bmsAveragedCurrent(SENTINAL)
  , bmsInstantCurrent(SENTINAL)
  , bmsVoltage(SENTINAL)
  , soc(SENTINAL)
{
  // No initialization required
}

/********************* Can Node Logging **********************/

void Store_Controller::logAnalogThrottle(const uint8_t throttle) {
  analogThrottle = throttle;
}
uint8_t Store_Controller::readAnalogThrottle() {
  return analogThrottle;
}

void Store_Controller::logAnalogBrake(const uint8_t brake) {
  analogBrake = brake;
  Onboard().logFour("brake_pressure", NO_DIRECTION, brake, "uint8_t");
}
uint8_t Store_Controller::readAnalogBrake() {
  return analogBrake;
}

void Store_Controller::logBrakeThrottleConflict(const bool conflict) {
  brakeThrottleConflict = conflict;
  if (conflict) {
    // Breaks pattern because error that we want to catch and filter
    Onboard().logFive("brake_throttle_conflict", analogThrottle, "throttle", analogBrake, "brake");
    Xbee().logFive("brake_throttle_conflict", analogThrottle, "throttle", analogBrake, "brake");
  }
}
bool Store_Controller::readBrakeThrottleConflict() {
  return brakeThrottleConflict;
}

void Store_Controller::logOutputTorque(const int16_t torque) {
  outputTorque = torque;
  Onboard().logFour("torque_cmd", NO_DIRECTION, torque, "uint8_t");
}
int16_t Store_Controller::readOutputTorque() {
  return outputTorque;
}

/******************** Wheel Speed Logging *********************/
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
}
int16_t Store_Controller::readSpeed(const Wheel wheel) {
  return speeds[wheel];
}

/****************** Motor Controller Logging ******************/
void Store_Controller::logMotorResponse(Motor dir) {
  responses[dir] = true;
}
bool Store_Controller::readMotorResponse(Motor dir) {
  return responses[dir];
}
String motor_faults[16] = {
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
void Store_Controller::logMotorErrors(Motor dir, uint16_t error_string) {
  String motor_name = (dir == RightMotor) ? "right" : "left";
  bool hasFault = false;

  // First update the array of errors
  for(int i = 0; i < 16; i++) {
    if (bitRead(error_string, i)) {
      String error_name = motor_faults[i];
      if (error_name != "under_voltage") {
        hasFault = true;
        // Breaks pattern because error that we want to catch and filter
        Xbee().logThree("motor_fault", motor_name, error_name);
        Onboard().logThree("motor_fault", motor_name, error_name);
      }
    }
  }
  if (errors[dir] != hasFault) {
    // Record fault
    errors[dir] = hasFault;

    // Light goes on if either has fault
    bool lightState = errors[LeftMotor] || errors[RightMotor];

    // Write message
    uint8_t faultValue = lightState ? 4 : 3;
    Frame error_light_frame = {.id=VCU_ID, .body={faultValue}, .len=1};
    CAN().write(error_light_frame);
  }
}

void Store_Controller::logMotorHighVoltage(Motor dir, bool state) {
  highVoltage[dir] = state;
  String motor_name = (dir == RightMotor) ? "right" : "left";
  // For debugging
  Onboard().logThree("motor_high_voltage", motor_name, state);
}
bool Store_Controller::readMotorHighVoltage(Motor dir) {
  return highVoltage[dir];
}

void Store_Controller::logMotorLowVoltage(Motor dir, bool state) {
  lowVoltage[dir] = state;
  String motor_name = (dir == RightMotor) ? "right" : "left";
  // For debugging
  Onboard().logThree("motor_low_voltage", motor_name, state);
}
bool Store_Controller::readMotorLowVoltage(Motor dir) {
  return lowVoltage[dir];
}

void Store_Controller::logMotorCurrent(Motor dir, int16_t current) {
  currents[dir] = current;
  String motor_name = (dir == RightMotor) ? "right" : "left";
  if (Dispatcher().isEnabled()) {
    Onboard().logFour("motor_current", motor_name, current, "motor_units");
  }
}
int16_t Store_Controller::readMotorCurrent(Motor controller) {
  return currents[controller];
}


void Store_Controller::logMotorCurrentCommand(Motor dir, int16_t currentCommand) {
  currentCommands[dir] = currentCommand;
  String motor_name = (dir == RightMotor) ? "right" : "left";
  if (Dispatcher().isEnabled()) {
    Onboard().logFour("motor_current_cmd", motor_name, currentCommand, "motor_units");
  }
}
int16_t Store_Controller::readMotorCurrentCommand(Motor controller) {
  return currentCommands[controller];
}

void Store_Controller::logMotorRpm(Motor dir, int16_t rpm) {
  motorRpm[dir] = rpm;
  String motor_name = (dir == RightMotor) ? "right" : "left";
  if (Dispatcher().isEnabled()) {
    Onboard().logFour("motor_rpm", motor_name, rpm, "rpm");
  }
}
int16_t Store_Controller::readMotorRpm(Motor controller) {
  return motorRpm[controller];
}

Motor Store_Controller::toMotor(uint16_t id) {
  return id == RIGHT_MOTOR_ID ? RightMotor : LeftMotor;
}
Motor Store_Controller::otherMotor(Motor dir) {
  return dir == RightMotor ? LeftMotor : RightMotor;
}

/************************ BMS Logging *************************/

String bms_faults[8] = {
  "driving_off",
  "interlock_tripped",
  "communication_fault",
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
      // Breaks pattern because error that we want to catch and filter
      Xbee().logTwo("bms_fault", fault_name);
      Onboard().logTwo("bms_fault", fault_name);
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
      // Breaks pattern because error that we want to catch and filter
      Xbee().logTwo("bms_warning", warning_name);
      Onboard().logTwo("bms_warning", warning_name);
    }
  }
}

void Store_Controller::logBmsTemp(const int16_t _bmsTemp) {
  bmsTemp = _bmsTemp;
  Onboard().logFour("bms_temp", NO_DIRECTION, bmsTemp, "degrees");
}
int16_t Store_Controller::readBmsTemp() {
  return bmsTemp;
}

void Store_Controller::logBmsAveragedCurrent(int16_t _bmsAveragedCurrent) {
  bmsAveragedCurrent = _bmsAveragedCurrent;
  Onboard().logFour("bms_averaged_current", NO_DIRECTION, bmsAveragedCurrent, "amps");
}
int16_t Store_Controller::readBmsAveragedCurrent() {
  return bmsAveragedCurrent;
}

void Store_Controller::logBmsInstantCurrent(int16_t _bmsInstantCurrent) {
  bmsInstantCurrent = _bmsInstantCurrent;
  Onboard().logFour("bms_instant_current", NO_DIRECTION, bmsInstantCurrent, "amps");
}
int16_t Store_Controller::readBmsInstantCurrent() {
  return bmsInstantCurrent;
}

void Store_Controller::logBmsVoltage(const int16_t _bmsVoltage) {
  bmsVoltage = _bmsVoltage;
  Onboard().logFour("bms_voltage", NO_DIRECTION, bmsVoltage, "volts");
}
int16_t Store_Controller::readBmsVoltage() {
  return bmsVoltage;
}

void Store_Controller::logBmsSoc(const int16_t _soc) {
  soc = _soc;
  Onboard().logFour("bms_soc", NO_DIRECTION, soc, "percent");
}
int16_t Store_Controller::readBmsSoc() {
  return soc;
}






/********************* Private Methods **************************/
Store_Controller* Store_Controller::instance = NULL;

Store_Controller& Store_Controller::readInstance() {
  if(!instance) {
    instance = new Store_Controller();
  }
  return *instance;
}
Store_Controller& Store() {
  return Store_Controller::readInstance();
}
