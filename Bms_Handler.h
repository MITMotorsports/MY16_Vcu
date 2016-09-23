#ifndef BMS_HANDLER_H
#define BMS_HANDLER_H

#include "Handler.h"

class Bms_Handler : public Handler {
  public:
    void begin();
    void handleMessage(Frame& message);
  private:
    int16_t mergeBytes(unsigned char low, unsigned char high);
    void handleFaultMessage(Frame& message);
    void handleVoltageMessage(Frame& message);
    void handleTempMessage(Frame& message);
    void handleSocMessage(Frame& message);
};

#endif // BMS_HANDLER_H

