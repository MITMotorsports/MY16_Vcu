#include "Bms_Handler.h"
#include "Store_Controller.h"
#include "Logger.h"

void Bms_Handler::begin() {
  // No initialization needed
}

void Bms_Handler::handleMessage(Frame& message) {
  switch(message.id) {
    case BMS_SUMMARY_ID:
      handleSummaryMessage(message);
      break;
    case BMS_FAULT_ID:
      handleFaultMessage(message);
      break;
    case BMS_VOLTAGE_ID:
      handleVoltageMessage(message);
      break;
    case BMS_CURRENT_ID:
      handleCurrentMessage(message);
      break;
    case BMS_TEMP_ID:
      handleTempMessage(message);
      break;
    case BMS_SOC_ID:
      handleSocMessage(message);
      break;
    case BMS_RESISTANCE_ID:
      break;
  }
}

uint16_t Bms_Handler::mergeBytes(unsigned char low, unsigned char high) {
  uint16_t low_ext = low;
  uint16_t high_ext = high << 8;
  return low_ext + high_ext;
}

void Bms_Handler::handleVoltageMessage(Frame& message) {
  uint16_t total_volts = mergeBytes(message.body[1], message.body[0]);
  Store().logBmsVoltage(total_volts);
  return;
}

void Bms_Handler::handleCurrentMessage(Frame& message) {
  uint16_t total_current = mergeBytes(message.body[1], message.body[0]);
  int16_t signed_current = (int16_t) total_current;
  Store().logBmsCurrent(signed_current);
  return;
}

void Bms_Handler::handleTempMessage(Frame& message) {
  unsigned char temp = message.body[0];
  Store().logBmsTemp(temp);
}

void Bms_Handler::handleSocMessage(Frame& message) {
  unsigned char soc = message.body[0];
  Store().logBmsSoc(soc);
}

void Bms_Handler::handleSummaryMessage(Frame& message) {
  (void)message;
  return;
}

void Bms_Handler::handleFaultMessage(Frame& message) {
  Store().logBmsFaults(message.body[5]);
  Store().logBmsWarnings(message.body[6]);
}
