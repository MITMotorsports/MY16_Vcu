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
    if(!Store().readTractiveVoltage()) {
      // Ignore message; driver was dumb and pushed button
      // before HV was on
      Serial.println("NO Enable: Low Voltage");
      return;
    }
    if(frame.body[0]) {
      if(Store().readAnalogBrake() > BRAKE_PUSHED_CUTOFF) {
        Serial.println("Enable");
        Dispatcher().enable();
      }
      else {
        Serial.println("NO Enable: No Brake");
      }
    }
    else {
      Dispatcher().disable();
      Serial.println("NO Enable: Disable Pressed");
    }
  }
}


