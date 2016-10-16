#ifndef MOTOR_HANDLER_H
#define MOTOR_HANDLER_H

#include "Handler.h"

class Motor_Handler : public Handler {
  public:
    void begin();
    void handleMessage(Frame& message);
    void requestHeartbeat();
    void requestPermanentUpdates(uint16_t can_id);
  private:
    void requestPermanentUpdate(uint16_t can_id, uint8_t msg_type, uint8_t time);
    void handleVoltageMessage(Frame& message);
    void handleTorqueMessage(Frame& message);
    void handleSpeedMessage(Frame& message);
    void handleErrorMessage(Frame& message);
    void handleStateMessage(Frame& message);
    void handleCurrentMessage(Frame& message);
    void handleAirTempMessage(Frame& message);
    void handleIgbtTempMessage(Frame& message);
    void handleCurrentLimitMessage(Frame& message);
    int motor_speed_to_motor_rpm(const int motor_speed);
    int motor_rpm_to_wheel_rpm(const int motor_speed);
    int wheel_rpm_to_kph(const int wheel_rpm);
};
#endif // MOTOR_HANDLER_H


