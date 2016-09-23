#ifndef STORE_CONTROLLER_H
#define STORE_CONTROLLER_H

#include <Arduino.h>

class Store_Controller {
  public:
    static Store_Controller& readInstance();

    // Directions for typechecking
    enum Wheel {FrontRightWheel, FrontLeftWheel, RearRightWheel, RearLeftWheel, WHEEL_LENGTH};
    enum Motor {LeftMotor, RightMotor, MOTOR_LENGTH};

    void logHasFault(bool hasFault);
    bool readHasFault();

    // Can node readings
    void logAnalogThrottle(uint8_t throttle);
    uint8_t readAnalogThrottle();
    void logAnalogBrake(uint8_t brake);
    uint8_t readAnalogBrake();
    void logBrakeThrottleConflict(bool conflict);
    bool readBrakeThrottleConflict();

    // Speeds
    void logSpeed(Wheel wheel, int16_t wheel_rpm);
    int16_t readSpeed(Wheel wheel);

    // Motor controller states
    void logMotorResponse(Motor dir);
    bool readMotorResponse(Motor dir);
    void logMotorErrors(Motor dir, uint16_t error_string);
    bool readMotorErrors(Motor dir);

    // Motor controller readings
    void logMotorTorqueCommand(Motor dir, int16_t torqueCommand);
    int16_t readMotorTorqueCommand(Motor dir);
    void logMotorRpm(Motor dir, int16_t motor_rpm);
    int16_t readMotorRpm(Motor dir);

    // Convenience methods for enum access
    Motor toMotor(uint16_t id);
    Motor otherMotor(Motor thisMotor);

    // BMS state
    void logBmsFaults(uint8_t fault_string);
    void logBmsWarnings(uint8_t warning_string);

    // BMS readings
    void logBmsTemp(int16_t temp);
    int16_t readBmsTemp();
    void logBmsVoltage(int16_t volts);
    int16_t readBmsVoltage();
    void logBmsSoc(int16_t percent);
    int16_t readBmsSoc();

  private:
    Store_Controller();
    static Store_Controller *instance;

    // Fault logging
    bool hasFault;

    // Can node logging
    uint8_t analogThrottle;
    uint8_t analogBrake;
    bool brakeThrottleConflict;

    // Wheel logging
    int16_t speeds[WHEEL_LENGTH];

    // Motor controller states
    bool responses[MOTOR_LENGTH];
    bool errors[MOTOR_LENGTH];
    int16_t motorRpm[MOTOR_LENGTH];

    // Motor controller readings
    int16_t torqueCommands[MOTOR_LENGTH];

    // BMS logging
    int16_t bmsTemp;
    int16_t bmsVoltage;
    int16_t soc;
};

Store_Controller& Store();
typedef enum Store_Controller::Motor Motor;
typedef enum Store_Controller::Wheel Wheel;

#endif // STORE_CONTROLLER_H
