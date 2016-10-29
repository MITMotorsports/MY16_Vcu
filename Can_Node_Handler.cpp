#include <math.h>
#include "Can_Node_Handler.h"
#include "Dispatch_Controller.h"
#include "Rtd_Controller.h"
#include "Store_Controller.h"

const int STARBOARD_THROTTLE_IDX = 0;
const int PORT_THROTTLE_IDX = 1;
const int BRAKE_IDX = 2;

const float THROTTLE_SCALING_FACTOR = 0.7;

const uint8_t TORQUE_PREFIX = 144; //0x90

enum braking_type{None, Coasting, Active};
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
  // Only execute if id matches
  if(message.id != CAN_NODE_ID) {
    return;
  }

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
    writeThrottleMessages(0);
    return;
  }

  // Also zero torque if brake-throttle conflict
  if (brakeThrottleConflict(analogThrottle, analogBrake)) {
    writeThrottleMessages(0);
    return;
  }

  int16_t raw_RPM = Store().readMotorRpm(STARBOARD_THROTTLE_IDX);
  int actual_motor_RPM = raw_RPM/4000; 
  float wheel_RPM = actual_motor_RPM*2.17;//using gear ratio to convert Motor RPM to Wheel RPM

  // Change from [0:255] to [0:32767]
  const int16_t throttleExtended = analogThrottle << 7;

  // Apply scaling factor and round
  const float throttleScaled = ((float)throttleExtended) * THROTTLE_SCALING_FACTOR;
  const int16_t outputTorque = (int16_t) (round(throttleScaled));

  if(analogThrottle>50){
    // Write torque commands
    writeThrottleMessages(outputTorque);
  }
  else if(wheel_RPM>2500&&analogBrake>100){ 
      writeThrottleMessages(-analogBrake);//need to scale braking value 
  }
  else if(wheel_RPM>2500){
    writeThrottleMessages(-50);
  }
  else{
    writeThrottleMessages(outputTorque);
  }
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

    bool throttleReleased = analogThrottle < (255 * 0.05);

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

    bool throttlePushed = analogThrottle >= (255 * 0.05);
    bool brakePushed = analogBrake >= BRAKE_PUSHED_CUTOFF;

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
void Can_Node_Handler::writeThrottleMessages(const int16_t throttle) {
  Frame leftFrame = {
    .id=LEFT_MOTOR_REQUEST_ID,
    .body={
      TORQUE_PREFIX,
      lowByte(throttle),
      highByte(throttle)
    },
    .len=3
  };
  CAN().write(leftFrame);

  int16_t neg_throttle = -throttle;
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
