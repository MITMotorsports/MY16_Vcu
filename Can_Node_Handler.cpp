#include <math.h>
#include "Can_Node_Handler.h"
#include "Dispatch_Controller.h"
#include "Rtd_Controller.h"
#include "Store_Controller.h"
#include "Logger.h"

const int STARBOARD_THROTTLE_IDX = 0;
const int PORT_THROTTLE_IDX = 1;
const int BRAKE_IDX = 2;
const int STEERING_IDX = 3;

const uint8_t THROTTLE_SCALING_PERCENTAGE = 100;

const uint8_t TORQUE_PREFIX = 144; //0x90

void Can_Node_Handler::brakeLightOn() {
  digitalWrite(BRAKE_LIGHT_PIN, HIGH);
}

void Can_Node_Handler::brakeLightOff() {
  digitalWrite(BRAKE_LIGHT_PIN, LOW);
}

void Can_Node_Handler::begin() {
  // No initialization needed
  pinMode(BRAKE_LIGHT_PIN, OUTPUT);
  brakeLightOff();
}

void Can_Node_Handler::handleMessage(Frame& message) {
  // Only execute if id matches something we care about
  if (message.id == CAN_NODE_ID) {
    handleCanNodeMessage(message);
  } else if (message.id == RPM_ID) {
    handleRpmMessage(message);
  } else {
    return;
  }
}

void Can_Node_Handler::handleRpmMessage(Frame& message) {
  uint16_t starboard_rpm = (message.body[1] << 8) + message.body[0];
  uint16_t port_rpm = (message.body[3] << 8) + message.body[2];
  Store().logSpeed(Store().FrontRightWheel, starboard_rpm);
  Store().logSpeed(Store().FrontLeftWheel, port_rpm);
}

void Can_Node_Handler::handleCanNodeMessage(Frame& message) {
  // Handle throttle messages
  bool plausible = true;
  uint8_t analogThrottle;
  if (isPlausible(message.body[STARBOARD_THROTTLE_IDX], message.body[PORT_THROTTLE_IDX])) {
    analogThrottle = min(
      message.body[STARBOARD_THROTTLE_IDX],
      message.body[PORT_THROTTLE_IDX]
    );
  }
  else {
    analogThrottle = 0;
    plausible = false;
  }

  // Handle brake messages
  uint8_t analogBrake = message.body[BRAKE_IDX];
  uint8_t analogSteering = message.body[STEERING_IDX];

  if(analogBrake < BRAKE_PUSHED_CUTOFF) {
    brakeLightOff();
  } else {
    brakeLightOn();
  }

  // Log analog sensors
  Store().logAnalogBrake(analogBrake);
  Store().logAnalogThrottle(analogThrottle);

  // Don't do torque commands if car is disabled!
  if(!Dispatcher().isEnabled()) {
    return;
  }

  // Zero torque if implausible
  if (!plausible) {
    writeThrottleMessages(0, 0);
    return;
  }

  // Also zero torque if brake-throttle conflict
  if (brakeThrottleConflict(analogThrottle, analogBrake)) {
    writeThrottleMessages(0, 0);
    return;
  }

  // Change from [0:255] to [0:32767]
  uint32_t throttleExtended = analogThrottle << 7;

  // We scale by multiplying by the numerator and then dividing by 100 (aka the denominator)
  //     where the numerator is the percentage from [0:100] that you want the torque to be scaled to.
  throttleExtended = throttleExtended * THROTTLE_SCALING_PERCENTAGE;
  throttleExtended = throttleExtended / 100;
  // Apply scaling factor and round
  const int16_t outputTorque = (int16_t) throttleExtended;
  Computer().logOne(outputTorque);

  // Write torque commands
  writeThrottleMessages(outputTorque, analogSteering);
}

bool Can_Node_Handler::isPlausible(uint8_t x, uint8_t y) {
  const uint8_t max = max(x, y);
  const uint8_t min = min(x, y);

  // 10% = 255/10
  const bool plausible = (max - min) < 25;
  return plausible;
}

bool Can_Node_Handler::brakeThrottleConflict(uint8_t analogThrottle, uint8_t analogBrake) {
  if (Store().readBrakeThrottleConflict()) {
    // We recently triggered a conflict: stay in conflict mode
    // unless throttle below 5%

    bool throttleReleased = analogThrottle < 13;

    if (throttleReleased) {
      // Remove conflict and log that it's cleared
      Store().logBrakeThrottleConflict(false);
    }
    else {
      // Don't remove conflict and log that it persists
      Store().logBrakeThrottleConflict(true);
    }
  }
  else {
    // We are not in conflict mode: only trigger conflict if
    // throttle above 25% and brake pressed

    bool throttlePushed = analogThrottle >= 64;

    // We raise the threshold for brake to 20% as well to prevent erroneous cutoff
    bool brakePushed = analogBrake >= 50;

    if (throttlePushed && brakePushed) {
      // Add conflict and log that it's added
      Store().logBrakeThrottleConflict(true);
    }
    else {
      // No conflict added, let's keep it that way
      Store().logBrakeThrottleConflict(false);
    }
  }
  return Store().readBrakeThrottleConflict();
}

// Right motor spins backwards
//   steering all right is 0, all left is 255 (https://github.com/MITMotorsports/MY16_Can_Node/blob/master/Can_Node.cpp)
void Can_Node_Handler::writeThrottleMessages(const int16_t throttle, uint8_t analogSteering) {

  // implements torque vectoring
  int32_t biased_steering = (int16_t) analogSteering;
  biased_steering = analogSteering - 128;

  uint32_t right_scale = (1000 + biased_steering*1000/128);
  uint32_t left_scale = (1000 - biased_steering*1000/128);
  right_scale = max(250, min(1750, right_scale));
  left_scale = max(250, min(1750, left_scale));

  uint16_t right_motor_throttle = max(0xFFFF, throttle*right_scale/1000);
  uint16_t left_motor_throttle = max(0xFFFF, throttle*left_scale/1000);

  Frame leftFrame = {
    .id=LEFT_MOTOR_REQUEST_ID,
    .body={
      TORQUE_PREFIX,
      lowByte(left_motor_throttle),
      highByte(left_motor_throttle)
    },
    .len=3
  };
  CAN().write(leftFrame);

  int16_t neg_throttle = -right_motor_throttle;
  Frame rightFrame = {
    .id=RIGHT_MOTOR_REQUEST_ID,
    .body={
      TORQUE_PREFIX,
      lowByte(neg_throttle),
      highByte(neg_throttle)
    },
    .len=3
  };
  CAN().write(rightFrame);
}
