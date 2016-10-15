#include "Store_Controller.h"
#include "Can_Ids.h"
#include "Dispatch_Controller.h"
#include "Logger.h"

// Value for indicating that there is not yet a reading
const int16_t SENTINAL = -32768;
const String NO_DIRECTION = "only";

Store_Controller::Store_Controller() 
  // Fault logging
  : hasFault(true)

  // Can node logging
  , analogThrottle(0)
  , analogBrake(0)
  , brakeThrottleConflict(false)

  // Wheel logging
  , speeds{SENTINAL, SENTINAL, SENTINAL, SENTINAL}

  // Motor controller states
  , responses{false, false}
  , errors{false, false}
  , motorRpm{SENTINAL, SENTINAL}

  // Motor controller readings
  , torqueCommands{SENTINAL, SENTINAL}

  // BMS readings
  , bmsTemp(SENTINAL)
  , bmsVoltage(SENTINAL)
  , soc(SENTINAL)
{
  // No initialization required
}

/********************* Fault Logging *************************/

void Store_Controller::logHasFault(bool fault) {
  hasFault = fault;
}
bool Store_Controller::readHasFault() {
  return hasFault;
}

/********************* Can Node Logging **********************/

void Store_Controller::logAnalogThrottle(const uint8_t throttle) {
  analogThrottle = throttle;
  Onboard().logFour("throttle_position", NO_DIRECTION, throttle, "uint8_t");
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
  }
}
bool Store_Controller::readBrakeThrottleConflict() {
  return brakeThrottleConflict;
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

String motor_warnings[16] = {
  "inconsistent_identification",
  "faulty_RUN_signal",
  "inactive_RFE",
  "should_never_happen",
  "should_never_happen",
  "missing_or_low_power_voltage",
  "Motor_temp>87%",
  "Device_temp>87%",
  "OverVoltage>1.5xUN",
  "2x_over_current",
  "should_never_happen",
  "should_never_happen",
  "Overload>87%",
  "should_never_happen",
  "should_never_happen",
  "Ballast_circuit_overload>87%"
};
void Store_Controller::logMotorWarnings(Motor dir, uint16_t warning_string){
    String motor_name = (dir == RightMotor) ? "right" : "left";
    for (int i = 0; i < 16; i++) {
      if (bitRead(warning_string, i)) {
        String warning_name = motor_warnings[i];
        Onboard().logThree("motor_warning", motor_name, warning_name);
        }
    }
}

void Store_Controller::logMotorState(Motor dir, uint32_t state_string){
  String motor_name = (dir == RightMotor) ? "right" : "left";
  for (int i = 0; i < 32; i++) {
    if (bitRead(state_string, i)){
      String state_name = "";
      if (i == 5) {
        // Current limited to continuous level
        state_name = "current_lim_contin";
      } else if (i == 21) {
        // Actual current limit reached
        state_name = "current_lim_reached";
      } else if (i == 22){
        // Current limiting via speed
        state_name = "current_lim_spd";
      } else if (i == 23){
        // Current limiting via output stage temp
        state_name = "current_lim_igbt_temp";
      } else if (i == 24){
        // Current reduced to continuous current via output stage temp
        state_name = "current_reduc_contin_igbt";
      } else if (i == 26){
        // Current limiting due to motor overtemp
        state_name = "current_lim_motor_temp";
      }
      if (state_name != ""){
        Onboard().logThree("motor_state", motor_name, state_name);
      }
    }
  }
}

void Store_Controller::logMotorTorqueCommand(Motor dir, int16_t torqueCommand) {
  torqueCommands[dir] = torqueCommand;
  String motor_name = (dir == RightMotor) ? "right" : "left";
  if (Dispatcher().isEnabled()) {
    Onboard().logFour("motor_torque_cmd", motor_name, torqueCommand, "uint16_t");
  }
}
int16_t Store_Controller::readMotorTorqueCommand(Motor controller) {
  return torqueCommands[controller];
}

void Store_Controller::logMotorRpm(Motor dir, int16_t rpm) {
  motorRpm[dir] = rpm;
  String motor_name = (dir == RightMotor) ? "right" : "left";
  if (Dispatcher().isEnabled()) {
    Onboard().logFour("motor_rpm", motor_name, rpm, "motor_units");
  }
}
int16_t Store_Controller::readMotorRpm(Motor controller) {
  return motorRpm[controller];
}

void Store_Controller::logMotorCurrent(Motor dir, int16_t current) {
  String motor_name = (dir == RightMotor) ? "right" : "left";
  if (Dispatcher().isEnabled()) {
    Onboard().logFour("motor_current", motor_name, current, "motor_units");
    Computer().logFour("motor_current", motor_name, current, "motor_units");
  }
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
