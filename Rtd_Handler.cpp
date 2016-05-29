#include <Debouncer.h>
#include <PciManager.h>

#include "Rtd_Handler.h"
#include "Dispatch_Controller.h"
#include "Store_Controller.h"

#include "Can_Node_Handler.h"

void Rtd_Handler::begin() {
  //No initialization necessary
}

void Rtd_Handler::handleMessage(Frame& frame) {
  if(frame.id != DASH_ID) {
    return;
  }
  else {
    // Check all three conditions for legal enable
    bool bothMotorsAlive =
      Store().readMotorHighVoltage(Store().LeftMotor) &&
      Store().readMotorHighVoltage(Store().RightMotor);
    if (!bothMotorsAlive) {
      // Car still in shutdown state, do nothing
      return;
    }

    bool brakePressed = Store().readAnalogBrake() >= BRAKE_PUSHED_CUTOFF;
    bool isEnableMessage = frame.body[0];
    if (isEnableMessage) {
      if (brakePressed) {
        // Legal enable
        Dispatcher().enable();
      }
      else {
        // Illegal enable; do nothing
      }
    }
    else {
      // Disable
      Dispatcher().disable();
    }
  }
}


