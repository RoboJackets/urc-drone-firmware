#include <Arduino.h>
#include "open_loop_speed_control.h"

// (Optional) If you want non-default pins/address, change here before setup():
// Example:
// static OpenLoopSpeedController::Pins customPins = {8, 9, 10, 11};
// static constexpr uint8_t customAddr = 0x10;

void setup() {
  // If customizing:
//   OL() = OpenLoopSpeedController(customPins, customAddr); // <-- would require non-const singleton.
//   OL().setPwmScale(0.8f);
//   OL().setDelayBounds(6000, 80);

  OL().begin();
}

void loop() {
  OL().tick();
}
