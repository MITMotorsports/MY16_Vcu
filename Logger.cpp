#include "Logger.h"

Logger* Logger::computerInstance = NULL;
Logger* Logger::onboardInstance = NULL;
Logger* Logger::xbeeInstance = NULL;

Logger::Logger(HardwareSerial& _output, bool _enabled)
  : output(_output)
  , enabled(_enabled)
{
  // No initialization required
}

Logger& Logger::readComputerInstance() {
  if(!computerInstance) {
    Serial.begin(115200);
    computerInstance = new Logger(Serial, true);
  }
  return *computerInstance;
}
Logger& Computer() {
  return Logger::readComputerInstance();
}

Logger& Logger::readOnboardInstance() {
  if(!onboardInstance) {
    Serial1.begin(115200);
    onboardInstance = new Logger(Serial1, true);
  }
  return *onboardInstance;
}
Logger& Onboard() {
  return Logger::readOnboardInstance();
}

Logger& Logger::readXbeeInstance() {
  if(!xbeeInstance) {
    Serial2.begin(57600);
    xbeeInstance = new Logger(Serial2, true);
  }
  return *xbeeInstance;
}
Logger& Xbee() {
  return Logger::readXbeeInstance();
}

void Logger::enable() {
  enabled = true;
}
void Logger::disable() {
  enabled = false;
}
