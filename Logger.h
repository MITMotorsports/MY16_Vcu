#ifndef LOGGER_H
#define LOGGER_H

#include "Arduino.h"

class Logger {
  public:
    // Accessor methods
    static Logger& readComputerInstance();
    static Logger& readOnboardInstance();
    static Logger& readXbeeInstance();

    // Instance constructor
    Logger(HardwareSerial& _output, bool _enabled);

    // Regular method headers
    void enable();
    void disable();
    bool isEnabled();

    // Logging methods (MUST go in header)
    template <typename First>
      void logOne(const First& first) {
        if (!enabled) {
          return;
        }
        output.print(String(first));
        output.print(", ");
        output.println(millis());
      }

    template <typename First, typename Second>
      void logTwo(const First& first, const Second& second) {
        if (!enabled) {
          return;
        }
        output.print(String(first));
        output.print(", ");
        output.print(String(second));
        output.print(", ");
        output.println(millis());
      }

    template <typename First, typename Second, typename Third>
      void logThree(const First& first, const Second& second, const Third& third) {
        if (!enabled) {
          return;
        }
        output.print(String(first));
        output.print(", ");
        output.print(String(second));
        output.print(", ");
        output.print(String(third));
        output.print(", ");
        output.println(millis());
      }

    template <typename First, typename Second, typename Third, typename Fourth>
      void logFour(const First& first, const Second& second, const Third& third, const Fourth& fourth) {
        if (!enabled) {
          return;
        }
        output.print(String(first));
        output.print(", ");
        output.print(String(second));
        output.print(", ");
        output.print(String(third));
        output.print(", ");
        output.print(String(fourth));
        output.print(", ");
        output.println(millis());
      }

    template <typename First, typename Second, typename Third, typename Fourth, typename Fifth>
      void logFive(const First& first, const Second& second, const Third& third, const Fourth& fourth, const Fifth& fifth) {
        if (!enabled) {
          return;
        }
        output.print(String(first));
        output.print(", ");
        output.print(String(second));
        output.print(", ");
        output.print(String(third));
        output.print(", ");
        output.print(String(fourth));
        output.print(", ");
        output.print(String(fifth));
        output.print(", ");
        output.println(millis());
      }
  private:
    // Accessor variables
    static Logger *computerInstance;
    static Logger *onboardInstance;
    static Logger *xbeeInstance;

    // Instance variables
    HardwareSerial& output;
    bool enabled;
};

Logger& Computer();
Logger& Onboard();
Logger& Xbee();

#endif // LOGGER_H
