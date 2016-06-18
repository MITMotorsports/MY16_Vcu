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

int16_t Bms_Handler::mergeBytes(uint8_t low, uint8_t high) {
  uint16_t low_ext = low & 0x00FF;
  uint16_t high_ext = (high << 8);
  return high_ext | low_ext;
}

void Bms_Handler::handleVoltageMessage(Frame& message) {
  // If voltage ever goes above 32767 volts, use a different method
  uint16_t total_volts = mergeBytes(message.body[1], message.body[0]);
  Store().logBmsVoltage(total_volts);

  uint8_t min_cell_id  = message.body[3];
  String cell_id_string = "id_" + String(min_cell_id);
  uint8_t min_cell_voltage  = message.body[2];
  Xbee().logFour("min_cell_voltage", cell_id_string, min_cell_voltage, "100_millivolts");
  Onboard().logFour("min_cell_voltage", cell_id_string, min_cell_voltage, "100_millivolts");
  return;
}

void Bms_Handler::handleCurrentMessage(Frame& message) {
  int16_t signed_current = mergeBytes(message.body[1], message.body[0]);
  Store().logBmsAveragedCurrent(signed_current);
  return;
}

void Bms_Handler::handleTempMessage(Frame& message) {
  uint8_t temp = message.body[0];
  Store().logBmsTemp(temp);

  uint8_t max_cell_id  = message.body[5];
  String cell_id_string = "id_" + String(max_cell_id);
  uint8_t max_cell_temp  = message.body[4];
  Xbee().logFour("max_cell_temp", cell_id_string, max_cell_temp, "degrees");
  Onboard().logFour("max_cell_temp", cell_id_string, max_cell_temp, "degrees");
}

void Bms_Handler::handleSocMessage(Frame& message) {
  uint8_t soc = message.body[0];
  Store().logBmsSoc(soc);
}

void Bms_Handler::handleSummaryMessage(Frame& message) {

  uint8_t low_current_byte_in_100mA = message.body[3];
  uint8_t high_current_byte_in_100mA = message.body[2];
  int16_t signed_current_in_100mA = mergeBytes(
      low_current_byte_in_100mA,
      high_current_byte_in_100mA
  );
  int16_t signed_current = signed_current_in_100mA / 10;
  Store().logBmsInstantCurrent(signed_current);
  return;
}

void Bms_Handler::handleFaultMessage(Frame& message) {
  Store().logBmsFaults(message.body[5]);
  Store().logBmsWarnings(message.body[6]);
}
