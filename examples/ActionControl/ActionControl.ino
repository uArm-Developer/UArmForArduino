/******************************************************************************************
* File Name          : Calibration.ino
* Author             : Jerry Song
* Version            : V1.0
* Date               : 26 Aug, 2014
* Modified Date      : 26 Aug, 2015
* Description        : This documents is for quick start with uArm Metal version
* Copyright(C) 2015 uArm Team. All right reserved.
*******************************************************************************************/

#include <Servo.h>
#include <Wire.h>
#include <EEPROM.h>
#include "uArm_calibration.h"
#include "uArm_Library.h"

void setup() {
  
      Wire.begin();        // join i2c bus (address optional for master)
      Serial.begin(9600);  // start serial port at 9600 bps      
}

void loop() {

  if(Serial.available()>0)
  {

      char readSerial = Serial.read();
      Serial.println(readSerial);
      
      if (readSerial == 'c') {
        calib.calibrations();
      }
      if (readSerial == 'r') {
        Serial.println(uarm.readServoOffset(SERVO_ROT_NUM));
        Serial.println(uarm.readServoOffset(SERVO_LEFT_NUM));
        Serial.println(uarm.readServoOffset(SERVO_RIGHT_NUM));
      }
  } // close read available
}
