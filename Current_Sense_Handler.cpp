#include "Current_Sense_Handler.h"
#include "Logger.h"

void Current_Sense_Handler::begin() {
  // No initialization needed
}

void Current_Sense_Handler::handleMessage(Frame& message) {
  switch(message.id) {
    case CS_CURRENT_ID:
      handleCurrentMessage(message);
      break;
    case CS_VOLTAGE_ID:
      handleVoltageMessage(message);
      break;
    case CS_POWER_ID:
      handlePowerMessage(message);
      break;
    default:
      break;
  }
}

int32_t Current_Sense_Handler::mergeBytes(uint8_t low, uint8_t lowMid, uint8_t highMid, uint8_t high) {
  int32_t highShift = (high << 24) & 0xFF000000;
  int32_t highMidShift = (highMid << 16) & 0x00FF0000;
  int32_t lowMidShift = (lowMid << 8) & 0x0000FF00;
  int32_t lowShift = low & 0x000000FF;
  return highShift | highMidShift | lowMidShift | lowShift;
}

void Current_Sense_Handler::handleCurrentMessage(Frame& message) {
  int32_t current = mergeBytes(message.body[5], message.body[4], message.body[3], message.body[2]);
  Onboard().logFour("cs_current", "only", current, "mA");
  Computer().logFour("cs_current", "only", current, "mA");
}

void Current_Sense_Handler::handleVoltageMessage(Frame& message) {
  int32_t voltage = mergeBytes(message.body[5], message.body[4], message.body[3], message.body[2]);
  Onboard().logFour("cs_voltage", "only", voltage, "mV");
  Computer().logFour("cs_voltage", "only", voltage, "mV");
}

void Current_Sense_Handler::handlePowerMessage(Frame& message) {
  int32_t power = mergeBytes(message.body[5], message.body[4], message.body[3], message.body[2]);
  Onboard().logFour("cs_power", "only", power, "W");
  Computer().logFour("cs_power", "only", power, "W");

}
