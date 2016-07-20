#include "uarm_library.h"

int value;        // value is the data recevied
unsigned char i=0,dat[10];
void setup() {
      uarm.arm_setup();

      Serial.begin(115200);  // start serial port at 9600 bps
      Serial.println("start");
      //Serial.println(uarm.move_to(100,150,100));    
}
double x,y,z;

void loop() {
  uarm.arm_process_commands();
/*  if(uarm.available()==true)
  {
    if(Serial.available())
    {
      message = Serial.readStringUntil(']') + ']';
      Serial.print(uarm.runCommand(message));         // Run the command and send back the response
    }
  }*/
}
