#include "Dispatch_Controller.h"

//Magic timing library stuff
#include <SoftTimer.h>
#include <DelayRun.h>
#include <Debouncer.h>

#include "Can_Controller.h"
#include "Rtd_Controller.h"
#include "Store_Controller.h"
#include "Logger.h"

Dispatch_Controller::Dispatch_Controller()
: rtd_handler(Rtd_Handler()),
  can_node_handler(Can_Node_Handler()),
  bms_handler(Bms_Handler()),
  motor_handler(Motor_Handler()),
  begun(false),
  enabled(false)
{

}

// Slight hack: invoke static member accessor function in non-member function
// pointer to work with SoftTimer's bullshit.
// TODO: Tighten this loop if perf problems arise

// I deeply apologize for the next forty lines of code.
// May Stroustroup forgive me.
void dispatchPointer(Task*) {
  Dispatcher().dispatch();
}
Task stepTask(0, dispatchPointer);

void requestVoltage(Task*) {
  Dispatcher().requestMotorVoltage();
}
Task voltageTask(500, requestVoltage);

bool requestPermanentUpdatesLeft(Task*) {
  Dispatcher().requestLeftMotorUpdates();
  return false;
}
DelayRun requestLeftMotorUpdatesTask(50, requestPermanentUpdatesLeft);

bool requestPermanentUpdatesRight(Task*) {
  Dispatcher().requestRightMotorUpdates();
  return false;
}
DelayRun requestRightMotorUpdatesTask(100, requestPermanentUpdatesRight);

void Dispatch_Controller::requestMotorVoltage() {
  bool bothMcOn =
    Store().readMotorResponse(Store().RightMotor) &&
    Store().readMotorResponse(Store().LeftMotor);

  if (!bothMcOn) {
    motor_handler.requestSingleVoltageUpdate();
  }
  else {
    SoftTimer.remove(&voltageTask);
    SoftTimer.add(&requestLeftMotorUpdatesTask);
    SoftTimer.add(&requestRightMotorUpdatesTask);
  }
}

void Dispatch_Controller::requestLeftMotorUpdates() {
  motor_handler.requestPermanentUpdates(LEFT_MOTOR_REQUEST_ID);
}
void Dispatch_Controller::requestRightMotorUpdates() {
  motor_handler.requestPermanentUpdates(RIGHT_MOTOR_REQUEST_ID);
}

void Dispatch_Controller::begin() {
  //Make idempotent
  if(begun) {
    return;
  }
  begun = true;

  // Initialize controllers
  CAN().begin();
  RTD().begin();

  // Initialize handlers
  rtd_handler.begin();
  can_node_handler.begin();
  bms_handler.begin();
  motor_handler.begin();

  // Start event loop
  SoftTimer.add(&stepTask);
  // Start MC requests
  SoftTimer.add(&voltageTask);

  Computer().logOne("vehicle_power_on");
  Xbee().logOne("vehicle_power_on");
}

// Must define instance prior to use
Dispatch_Controller* Dispatch_Controller::instance = NULL;

Dispatch_Controller& Dispatch_Controller::getInstance() {
  if (!instance) {
    instance = new Dispatch_Controller();
    instance->begin();
  }
  return *instance;
}

Dispatch_Controller& Dispatcher() {
  return Dispatch_Controller::getInstance();
}

bool Dispatch_Controller::isEnabled() {
  return enabled;
}

void Dispatch_Controller::disable() {
  // Force idempotency
  if(!enabled) {
    return;
  }
  enabled = false;

  // Actually disable
  RTD().disable();

  Frame disableMessage = { .id=VCU_ID, .body={0}, .len=1};
  CAN().write(disableMessage);

  Xbee().logOne("vehicle_disabled");
  Computer().logOne("vehicle_disabled");
}

void Dispatch_Controller::enable() {
  // Force idempotency
  if(enabled) {
    return;
  }
  enabled = true;

  // Actually enable
  RTD().enable();

  // Notify listeners of enable
  Frame enableMessage = { .id=VCU_ID, .body={1}, .len=1};
  CAN().write(enableMessage);

  Xbee().logOne("vehicle_enabled");
  Computer().logOne("vehicle_enabled");
}

void Dispatch_Controller::dispatch() {
  // If no message, break early
  while(CAN().msgAvailable()) {
    Frame frame = CAN().read();

    // Send message to each handler
    rtd_handler.handleMessage(frame);
    bms_handler.handleMessage(frame);
    can_node_handler.handleMessage(frame);
    motor_handler.handleMessage(frame);
  }
}
