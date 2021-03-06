#ifndef CAN_NODE_HANDLER_H
#define CAN_NODE_HANDLER_H

#include "Handler.h"

class Can_Node_Handler : public Handler {
  public:
    void begin();
    void handleMessage(Frame& message);
    void handleCanNodeMessage(Frame& message);
    void handleRpmMessage(Frame& message);
  private:
    void brakeLightOn();
    void brakeLightOff();
    void handleThrottleMessage(const uint8_t starboard, const uint8_t port);
    void handleBrakeMessage(const uint8_t starboard, const uint8_t port);
    bool brakeThrottleConflict(uint8_t analogThrottle, uint8_t analogBrake);

    void regenMotors(uint8_t throttle);
    void driveMotors(uint8_t throttle);
    void writeThrottleMessages(const int16_t throttle);

    bool isPlausible(uint8_t max, uint8_t min);
};

// Consider 12.5% of brake pressure a pushed brake
const uint8_t BRAKE_PUSHED_CUTOFF = 32;

#endif // CAN_NODE_HANDLER_H
