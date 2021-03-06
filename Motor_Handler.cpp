#include "Dispatch_Controller.h"
#include "Motor_Handler.h"

#include "Can_Controller.h"
#include "Rtd_Controller.h"
#include "Store_Controller.h"

#include "Pins.h"
#include "Logger.h"

/**
 * VERY IMPORTANT
 * Using a 32 bit union is the ONLY reliable way I have found
 * for unpacking > 2 bit CAN messages.
 * Please do not try anything else - you WILL lose your high-order bits.
 **/
union Reading
{
   uint32_t value;
   uint8_t bytes[4];
};

const int REQUEST_PREFIX = 61; //0x3D
const int TORQUE_PREFIX = 144; //0x90

const int MOTOR_HEARTBEAT_MODIFIER = 225; //0xE1

const int MOTOR_TORQUE_MODIFIER = TORQUE_PREFIX;
const int MOTOR_CURRENT_MODIFIER = 32; //0x20
const int MOTOR_CURRENT_AFTER_FILTER_MODIFIER = 95; //0x5f
const int MOTOR_SPEED_MODIFIER = 48; //0x30
const int MOTOR_ERRORS_MODIFIER = 143; //0x8F
const int MOTOR_STATE_MODIFIER = 64; //0x40
const int MOTOR_CURRENT_COMMAND_MODIFIER = 34; //0x22

const int MOTOR_CURRENT_LIMIT_MODIFIER = 72; //0x48
const int MOTOR_IGBT_TEMP_MODIFIER = 74; //0x4A
const int MOTOR_AIR_TEMP_MODIFIER = 75; //0x4B

void Motor_Handler::begin() {
  // No initialization needed
}

void Motor_Handler::requestHeartbeat() {
  Frame frame = {.id=LEFT_MOTOR_REQUEST_ID, .body={REQUEST_PREFIX, MOTOR_HEARTBEAT_MODIFIER, 0}, .len=3};
  CAN().write(frame);

  frame = {.id=RIGHT_MOTOR_REQUEST_ID, .body={REQUEST_PREFIX, MOTOR_HEARTBEAT_MODIFIER, 0}, .len=3};
  CAN().write(frame);
}

void Motor_Handler::requestPermanentUpdates(uint16_t can_id) {
  requestPermanentUpdate(can_id, MOTOR_TORQUE_MODIFIER, 101);
  requestPermanentUpdate(can_id, MOTOR_SPEED_MODIFIER, 103);
  requestPermanentUpdate(can_id, MOTOR_ERRORS_MODIFIER, 105);
  requestPermanentUpdate(can_id, MOTOR_CURRENT_MODIFIER, 109);
  requestPermanentUpdate(can_id, MOTOR_CURRENT_AFTER_FILTER_MODIFIER, 111);
  // Temp is less important
  //requestPermanentUpdate(can_id, MOTOR_AIR_TEMP_MODIFIER, 254);
  //requestPermanentUpdate(can_id, MOTOR_IGBT_TEMP_MODIFIER, 253);
  requestPermanentUpdate(can_id, MOTOR_STATE_MODIFIER, 249);
  requestPermanentUpdate(can_id, MOTOR_CURRENT_LIMIT_MODIFIER, 251);
  requestPermanentUpdate(can_id, MOTOR_CURRENT_COMMAND_MODIFIER, 253);
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
  Store().logMotorResponse(Store().toMotor(message.id));
  bool bothMcOn =
    Store().readMotorResponse(Store().RightMotor) &&
    Store().readMotorResponse(Store().LeftMotor);

  if(!bothMcOn) {
    // Invalid message since both motor controllers are not yet on
    return;
  }

  switch(message.body[0]) {
    case MOTOR_SPEED_MODIFIER:
      handleSpeedMessage(message);
      break;
    case MOTOR_ERRORS_MODIFIER:
      handleErrorMessage(message);
      break;
    case MOTOR_TORQUE_MODIFIER:
      handleTorqueMessage(message);
      break;
    case MOTOR_STATE_MODIFIER:
      handleStateMessage(message);
      break;
    case MOTOR_CURRENT_MODIFIER:
      handleCurrentMessage(message);
      break;
    case MOTOR_IGBT_TEMP_MODIFIER:
      handleIgbtTempMessage(message);
      break;
    case MOTOR_AIR_TEMP_MODIFIER:
      handleAirTempMessage(message);
      break;
    case MOTOR_CURRENT_LIMIT_MODIFIER:
      handleCurrentLimitMessage(message);
      break;
    case MOTOR_CURRENT_AFTER_FILTER_MODIFIER:
      handleCurrentAfterFilterMessage(message);
      break;
    case MOTOR_CURRENT_COMMAND_MODIFIER:
      handleCurrentCommandMessage(message);
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

void Motor_Handler::handleStateMessage(Frame& message){
  Reading currentReading;
  currentReading.bytes[3] = message.body[4];
  currentReading.bytes[2] = message.body[3];
  currentReading.bytes[1] = message.body[2];
  currentReading.bytes[0] = message.body[1];
  uint32_t state_string = currentReading.value;

  Motor motor = Store().toMotor(message.id);
  Store().logMotorState(motor, state_string);
}

void Motor_Handler::handleErrorMessage(Frame& message) {
  uint16_t error_string = (message.body[2] << 8) + message.body[1];
  uint16_t warning_string = (message.body[4] << 8) + message.body[3];
  Motor motor = Store().toMotor(message.id);
  Store().logMotorErrors(motor, error_string);
  Store().logMotorWarnings(motor, warning_string);
}

void Motor_Handler::handleSpeedMessage(Frame& message) {
  Motor motor = Store().toMotor(message.id);
  int signed_speed_numeric = makePositive(
      mergeBytesOfSignedInt(message.body[1], message.body[2])
  );

  // int rpm = motor_speed_to_motor_rpm(signed_speed_numeric);
  Store().logMotorRpm(motor, signed_speed_numeric);
}

void Motor_Handler::handleCurrentMessage(Frame& message) {
  Motor motor = Store().toMotor(message.id);
  int signed_current_numeric = makePositive(
      mergeBytesOfSignedInt(message.body[1], message.body[2])
  );

  Store().logMotorCurrent(motor, signed_current_numeric);
}
void Motor_Handler::handleCurrentAfterFilterMessage(Frame& message) {
  Motor motor = Store().toMotor(message.id);
  int signed_current_numeric = makePositive(
      mergeBytesOfSignedInt(message.body[1], message.body[2])
  );

  Store().logMotorCurrentAfterFilter(motor, signed_current_numeric);
}
void Motor_Handler::handleCurrentCommandMessage(Frame& message) {
  Motor motor = Store().toMotor(message.id);
  int signed_current_numeric = makePositive(
      mergeBytesOfSignedInt(message.body[1], message.body[2])
  );

  Store().logMotorCurrentCommand(motor, signed_current_numeric);
}
void Motor_Handler::handleTorqueMessage(Frame& message) {
  int16_t signed_torque = mergeBytesOfSignedInt(message.body[1], message.body[2]);
  int16_t torque = makePositive(signed_torque);
  Motor motor = Store().toMotor(message.id);
  Store().logMotorTorqueCommand(motor, torque);
}

void Motor_Handler::handleAirTempMessage(Frame& message) {
  int16_t signed_temp = mergeBytesOfSignedInt(message.body[1], message.body[2]);
  int16_t temp = makePositive(signed_temp);
  Motor motor = Store().toMotor(message.id);
  Store().logMotorAirTemp(motor, temp);
}

void Motor_Handler::handleIgbtTempMessage(Frame& message) {
  int16_t signed_temp = mergeBytesOfSignedInt(message.body[1], message.body[2]);
  int16_t temp = makePositive(signed_temp);
  Motor motor = Store().toMotor(message.id);
  Store().logMotorIgbtTemp(motor, temp);
}

void Motor_Handler::handleCurrentLimitMessage(Frame& message) {
  int16_t signed_limit = mergeBytesOfSignedInt(message.body[1], message.body[2]);
  int16_t limit = makePositive(signed_limit);
  Motor motor = Store().toMotor(message.id);
  Store().logMotorCurrentLimit(motor, limit);
}


int Motor_Handler::motor_rpm_to_wheel_rpm(const int motor_rev_per_min) {
  const float WHEEL_REVS_PER_MOTOR_REV = 1.0 / 2.64;
  float wheel_rev_per_min =
    motor_rev_per_min * WHEEL_REVS_PER_MOTOR_REV;
  int rpm_rounded = round(wheel_rev_per_min);
  return rpm_rounded;

}

int Motor_Handler::motor_speed_to_motor_rpm(const int motor_speed) {
  const float MOTOR_REVS_PER_MOTOR_SPEED_INCREMENT = 1.0 / 8.2;
  float motor_rev_per_min =
    motor_speed * MOTOR_REVS_PER_MOTOR_SPEED_INCREMENT;
  int rpm_rounded = round(motor_rev_per_min);
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
