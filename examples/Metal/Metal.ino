#include "uArm.h"

void setup() {
  Serial.begin(115200);  // start serial port at 115200 bps
  uArm.setup();


}

void loop() {
  uArm.run();
}
