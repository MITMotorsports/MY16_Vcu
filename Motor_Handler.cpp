#include "Dispatch_Controller.h"
#include "Motor_Handler.h"

#include "Can_Controller.h"
#include "Rtd_Controller.h"
#include "Store_Controller.h"

#include "Pins.h"

const int REQUEST_PREFIX = 61; //0x3D
const int TORQUE_PREFIX = 144; //0x90

const int MOTOR_CURRENT_MODIFIER = 32; //0x20
const int MOTOR_SPEED_MODIFIER = 48; //0x30
const int MOTOR_ERRORS_MODIFIER = 143; //0x8F
const int MOTOR_VOLTAGE_MODIFIER = 224; //0xE0

void Motor_Handler::begin() {
  // No initialization needed
}

void Motor_Handler::requestSingleVoltageUpdate() {
  Frame frame = {.id=LEFT_MOTOR_REQUEST_ID, .body={REQUEST_PREFIX, MOTOR_VOLTAGE_MODIFIER, 0}, .len=3};
  CAN().write(frame);

  frame = {.id=RIGHT_MOTOR_REQUEST_ID, .body={REQUEST_PREFIX, MOTOR_VOLTAGE_MODIFIER, 0}, .len=3};
  CAN().write(frame);
}

void Motor_Handler::requestPermanentVoltageUpdate() {
  Frame frame = {.id=LEFT_MOTOR_REQUEST_ID, .body={REQUEST_PREFIX, MOTOR_VOLTAGE_MODIFIER, 100}, .len=3};
  CAN().write(frame);

  frame = {.id=RIGHT_MOTOR_REQUEST_ID, .body={REQUEST_PREFIX, MOTOR_VOLTAGE_MODIFIER, 100}, .len=3};
  CAN().write(frame);
}

void Motor_Handler::requestPermanentUpdates(uint16_t can_id) {
  requestPermanentUpdate(can_id, MOTOR_CURRENT_MODIFIER, 101);
  requestPermanentUpdate(can_id, MOTOR_SPEED_MODIFIER, 103);
  requestPermanentUpdate(can_id, MOTOR_ERRORS_MODIFIER, 105);
  requestPermanentUpdate(can_id, MOTOR_VOLTAGE_MODIFIER, 107);
  // requestPermanentUpdate(can_id, MOTOR_TEMP_MODIFIER, 109);
  // requestPermanentUpdate(can_id, MOTOR_POSITION_MODIFIER, 113);
}

void Motor_Handler::requestPermanentUpdate(uint16_t can_id, uint8_t msg_type, uint8_t time) {
  Frame frame = {.id=can_id, .body={REQUEST_PREFIX, msg_type, time}, .len=3};
  CAN().write(frame);
}

int16_t mergeBytesOfSignedInt(uint8_t low_byte, uint8_t high_byte) {
  int16_t result = (high_byte << 8) + low_byte;
  return result;
}

void Motor_Handler::handleMessage(Frame& message) {
  // Enable-only task
  if(!(message.id == RIGHT_MOTOR_ID || message.id == LEFT_MOTOR_ID)) {
    return;
  }
  if(message.id == RIGHT_MOTOR_ID) {
    Store().logMotorResponse(Store().RightMotor);
  }
  else {
    Store().logMotorResponse(Store().LeftMotor);
  }

  bool bothMcOn =
    Store().readMotorResponse(Store().RightMotor) &&
    Store().readMotorResponse(Store().LeftMotor);

  if(!bothMcOn) {
    // Invalid message since both motor controllers are not yet on
    return;
  }

  switch(message.body[0]) {
    case MOTOR_CURRENT_MODIFIER:
      handleCurrentMessage(message);
      break;
    case MOTOR_SPEED_MODIFIER:
      handleSpeedMessage(message);
      break;
    case MOTOR_VOLTAGE_MODIFIER:
      handleVoltageMessage(message);
      break;
    case MOTOR_ERRORS_MODIFIER:
      handleWarningMessage(message);
      break;
  }
}

int16_t makePositive(int16_t x) {
  if (x > 0) {
    return x;
  }
  else {
    return -x;
  }
}

void Motor_Handler::handleWarningMessage(Frame& message) {
  uint16_t warning_string = (message.body[2] << 8) + message.body[1];
  Store_Controller::MotorController motor;
  if (message.id == RIGHT_MOTOR_ID) {
    motor = Store_Controller::RightMotor;
  }
  else {
    motor = Store_Controller::LeftMotor;
  }
  Store().logMotorErrors(motor, warning_string);
}

void Motor_Handler::handleSpeedMessage(Frame& message) {
  Store_Controller::Wheel wheel;
  if (message.id == RIGHT_MOTOR_ID) {
    wheel = Store_Controller::RearRightWheel;
  }
  else {
    wheel = Store_Controller::RearLeftWheel;
  }

  int signed_speed_numeric = makePositive(
      mergeBytesOfSignedInt(message.body[1], message.body[2])
  );

  int rpm = motor_speed_to_wheel_rpm(signed_speed_numeric);
  Store().logSpeed(wheel, rpm);

  int kph = wheel_rpm_to_kph(rpm);
  // Evil logic that should go in Rtd_Controller...
  if (kph > 5) {
    digitalWrite(FAN_PIN, LOW);
  }
  else {
    digitalWrite(FAN_PIN, HIGH);
  }
}

int Motor_Handler::motor_speed_to_wheel_rpm(const int motor_speed) {
  const float MOTOR_REVS_PER_MOTOR_SPEED_INCREMENT = 1.0 / 8.2;
  const float WHEEL_REVS_PER_MOTOR_REV = 1.0 / 2.64;

  float motor_rev_per_min =
    motor_speed * MOTOR_REVS_PER_MOTOR_SPEED_INCREMENT;
  float wheel_rev_per_min =
    motor_rev_per_min * WHEEL_REVS_PER_MOTOR_REV;
  int rpm_rounded = round(wheel_rev_per_min);
  return rpm_rounded;
}

int Motor_Handler::wheel_rpm_to_kph(const int wheel_rpm) {
  const float WHEEL_DIAMETER_CM = 52.07;
  const float METERS_PER_CM = 0.01;
  const float METERS_PER_WHEEL_REV =
    WHEEL_DIAMETER_CM / METERS_PER_CM * PI;

  const float MIN_PER_HR = 60;
  const float KM_PER_METER = 0.001;
  const float WHEEL_RPM_TO_WHEEL_KPH =
    METERS_PER_WHEEL_REV * MIN_PER_HR * KM_PER_METER;

  float kilo_per_hr = wheel_rpm * WHEEL_RPM_TO_WHEEL_KPH;
  int kph_rounded = round(kilo_per_hr);
  return kph_rounded;
}

void Motor_Handler::handleCurrentMessage(Frame& message) {
  // TODO convert this to an actual reading instead of just percent
  int16_t signed_current = mergeBytesOfSignedInt(message.body[1], message.body[2]);
  int16_t current = makePositive(signed_current);
  Store().logMotorCurrent(message.id == RIGHT_MOTOR_ID ? Store_Controller::RightMotor : Store_Controller::LeftMotor, current);
}

void Motor_Handler::handleVoltageMessage(Frame& message) {
  // Don't worry about this message if car already enabled
  if (Dispatcher().isEnabled()) {
    return;
  }

  bool previousVoltageLive = Store().readTractiveVoltage();
  bool currentVoltageLive = (message.body[1] != 0);

  if(!currentVoltageLive && previousVoltageLive) {
    // Voltage was high and has become low, tell dash to shut off light
    Frame shutdownMessage = { .id=VCU_ID, .body={2}, .len=1};
    CAN().write(shutdownMessage);
    Store().logTractiveVoltage(false);
  }
  else if (currentVoltageLive && !previousVoltageLive) {
    // Voltage was low and has become high, tell dash to blink light
    Serial.println("VOLTAGE HIGH: CAR LIVE");
    Frame disableMessage = { .id=VCU_ID, .body={0}, .len=1};
    CAN().write(disableMessage);
    Store().logTractiveVoltage(true);
  }
}
