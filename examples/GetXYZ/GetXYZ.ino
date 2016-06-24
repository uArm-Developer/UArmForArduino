#include <uarm_library.h>

void setup(){
  Serial.begin(9600);
}

void loop(){
  uarm.get_current_xyz();
  Serial.print("X: ");
  Serial.println(uarm.get_current_x());
  Serial.print("Y: ");
  Serial.println(uarm.get_current_y());
  Serial.print("Z: ");
  Serial.println(uarm.get_current_z());
  Serial.println();
  delay(500);
}
