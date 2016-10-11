// test by John Feng

//#define LATEST_HARDWARE
#include "uArm.h"

void setup() {
  Serial.begin(115200);

  uArm.setup();


}

void loop() {
  uArm.run();

}
