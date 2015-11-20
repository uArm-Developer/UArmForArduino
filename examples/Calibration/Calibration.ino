/******************************************************************************************
* File Name          : Calibration.ino
* Author             : Joey Song
* Version            : V1.0
* Date               : 26 Aug, 2014
* Modified Date      : 19 Nov, 2015
* Description        : This documents is for calibration with uArm Metal version
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

#include <EEPROM.h>
#include <Wire.h>
#include "uArm_library.h"
#include "uArm_calibration.h"
#include <Servo.h>

// define a uArm 
//uArmLibrary uArm;  
int value;        // value is the data recevied 

void setup() {
  
      Wire.begin();        // join i2c bus (address optional for master)
      Serial.begin(9600);  // start serial port at 9600 bps
      // uArm.init();
      
}


void loop() {

  if(Serial.available()>0)
  {

      char readSerial = Serial.read();
      Serial.println(readSerial);
      
      // Input c to start calibrate automatically
      if (readSerial == 'c') {
        calib.calibrations();
        delay(1000);
        uarm.moveTo(0,-15,6);
      }

      //----------------------------------  Test Function  ------------------------------------
      if (readSerial == '1') {
        uarm.moveTo(13,-13,6);
        delay(1000);
       }
       
     
      if (readSerial == '2') {
        uarm.moveTo(-13,-13,6);
        delay(1000);
       }

      // Do not need to use setOffset() function if you have already use calibrations();
      if (readSerial == 's') {
        calib.setOffset();
        delay(1000);
        uarm.moveTo(0,-15,6);
        delay(200);
       }
  
  } // close read available
}