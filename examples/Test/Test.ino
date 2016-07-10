/******************************************************************************************
* File Name          : Test.ino
* Author             : Joey Song
* Version            : V1.0
* Date               : 26 Aug, 2014
* Modified Date      : 19 Nov, 2015
* Description        : This documents is for quick start with uArm Metal version
* Copyright(C) 2015 uArm Team. All right reserved.
*******************************************************************************************/

/*
 * Table of Content

 * Function 1 - 4 :    move to a certain point (f)
 * Fucntion 5 - 6 :    move a Rectangle or a curve (function 5-6)
 * Function 7 - 8 :    attach or detach all servos (function 7-8)
 * Function 9     :    uArm calibration
 * Function 10    :    read current coordinate x,y,z
 * Function 11    :    recording mode

*/

// headers should must include these four headers

#include "uarm_library.h"

int value;        // value is the data recevied
unsigned char i=0,dat[10];
void setup() {
      uarm.arm_setup();

      Serial.begin(115200);  // start serial port at 9600 bps
      Serial.println("start");
      //Serial.println(uarm.move_to(100,150,100));
      //uarm.get_current_xyz();
      
}
double x,y,z;
String message = "";
void loop() {
  uarm.arm_process_commands();
  /*if(uarm.available()==true)
  {
    Serial.println("available!");

  }*/
  if(Serial.available())
  {

   message = Serial.readStringUntil(']') + ']';
    Serial.print(uarm.runCommand(message));         // Run the command and send back the response

    /*char inChar = (char)Serial.read();
    dat[i]=inChar-48;
    if((dat[i]+48)=='e')
    {
      Serial.println("enter");
      i=0;
      x=dat[0]*100.0+dat[1]*10.0+dat[2];
      y=dat[3]*100.0+dat[4]*10.0+dat[5];
      z=dat[6]*100.0+dat[7]*10.0+dat[8];
      if(uarm.move_to(x,y,z)==IN_RANGE)
      {
        Serial.println("succeed");
        delay(200);
        //uarm.get_current_xyz();
      }
      else
      {
        Serial.println("fail");
      }
      //uarm.write_servo_angle(x,y,z);
        Serial.println("xyz\n");
        Serial.println(x,DEC);
        Serial.println(y,DEC);
        Serial.println(z,DEC);
    }
    else
    {
      i++;
    }*/

  }
  

}
