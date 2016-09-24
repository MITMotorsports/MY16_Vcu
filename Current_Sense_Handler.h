#ifndef CURRENT_SENSE_HANDLER_H
#define CURRENT_SENSE_HANDLER_H

#include "Handler.h"

class Current_Sense_Handler : public Handler {
  public:
    void begin();
    void handleMessage(Frame& message);
  private:
    int32_t mergeBytes(uint8_t low, uint8_t lowMid, uint8_t highMid, uint8_t high);
    void handleCurrentMessage(Frame& message);
    void handleVoltageMessage(Frame& message);
    void handlePowerMessage(Frame& message);
};

#endif // CURRENT_SENSE_HANDLER_H

