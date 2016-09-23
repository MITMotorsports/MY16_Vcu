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


