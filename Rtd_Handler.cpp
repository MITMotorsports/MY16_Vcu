#include <Debouncer.h>
#include <PciManager.h>

#include "Rtd_Handler.h"
#include "Dispatch_Controller.h"
#include "Store_Controller.h"

void Rtd_Handler::begin() {
  //No initialization necessary
}

void Rtd_Handler::handleMessage(Frame& frame) {
  // Always set this to normal amount unless explicitly told to insane mode
  Store().logThrottleScalingFactor(0.2);
  if (frame.id == DASH_ID) {
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
  else if (frame.id == INSANE_MODE_ID) {
    if (!Dispatcher().isEnabled()) {
      // Only do insane mode things while enabled
      return;
    }
    Frame insane_mode_response = {.id = INSANE_MODE_RESPONSE_ID, .body={0}, .len=1};

    if (frame.body[0] == 0) {
      Store().logThrottleScalingFactor(0.2);
      insane_mode_response.body[0] = 0;
    }
    else {
      Store().logThrottleScalingFactor(0.4);
      insane_mode_response.body[0] = 1;
    }
    CAN().write(insane_mode_response);
  }
}


