// test by John Feng

//#define LATEST_HARDWARE
#include "uArm.h"

void setup() {
  Serial.begin(115200);

  uArm.setup();
  Serial.println("[READY]");
  uArm.moveTo(0, 150, 150, 10);


}

void loop() {
  uArm.run();

}
