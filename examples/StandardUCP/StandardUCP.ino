
/******************************************************************************
* File Name          : Firmata-UCP.ino
* Author             : Alex
* Updated            : Alex
* Version            : V0.1
* Created Date       : 24 Aug, 2015
* Description        : Firmata-UCP(uArm Communication Protocol)
* License            : GNU
* Copyright(C) 2015 UFactory Team. All right reserved.
*******************************************************************************/

#include <Servo.h>
#include <Wire.h>

// Firmata-UCP
#include <EEPROM.h>
#include "uArm_Library_Metal.h"
#include "UCP.h"

// UCPClass UCP;

void setup()
{
	UCP.setFirmwareVersion(UCP_MAJOR_VERSION, UCP_MINOR_VERSION);
	UCP.attach(START_SYSEX, sysexCallback);

	UCP.begin(57600);

	// systemResetCallback();
}

void loop()
{

  /* STREAMREAD - processing incoming messagse as soon as possible, while still
   * checking digital inputs.  */	
  while (UCP.available())
    UCP.processInput();
}


void sysexCallback(byte command, byte argc, byte *argv)
{

	switch (command) {
  	case UARM_INIT:
  		uArm.init();
  		break;
    case ACTION_CONTROL:

        switch(argv[0]){
          case AC_COOR_REAL_XYZ:
          {
            short x = UCP.convertNumToCM(UCP.getValuesAsTwobitBytes(argv[1],argv[2]));
            short y = UCP.convertNumToCM(UCP.getValuesAsTwobitBytes(argv[3],argv[4]));
            short z = UCP.convertNumToCM(UCP.getValuesAsTwobitBytes(argv[5],argv[6]));
            short t = UCP.convertDMSToSec(UCP.getValuesAsTwobitBytes(argv[7],argv[8]));
            // int x = UCP.getValuesAsTwobitBytes(argv[1],argv[2]);
            // int y = UCP.getValuesAsTwobitBytes(argv[3],argv[4]);
            // int z = UCP.getValuesAsTwobitBytes(argv[5],argv[6]);
            // int t = UCP.getValuesAsTwobitBytes(argv[7],argv[8]);
            uArm.moveTo(x,y,z,RELATIVE,t);
           if(UCP.debugMode)
            {
              // UCP.sendString("Debug Mode Test");
              // Serial.print("UCP.getValuesAsTwobitBytes(argv[1],argv[2]):");
              // Serial.println(UCP.getValuesAsTwobitBytes(argv[1],argv[2]));
              Serial.print("X is: ");
              Serial.println(x);
              Serial.print("Y is: ");
              Serial.println(y);
              Serial.print("Z is: ");
              Serial.println(z);
              Serial.print("time is: ");
              Serial.println(t);
            }
            // if(UCP.debugMode)
            //   UCP.sendString("debugMode");
            //uArm.moveTo(UCP.convertNumToCM(argv[0],argv[1]),UCP.convertNumToCM(argv[2],argv[3]),UCP.convertNumToCM(argv[4],argv[5]),REALTIVE,UCP.convertDMSToSecond(argv[6],argv[7]));
            break;
          }
          case AC_COOR_ABS_XYZ:
          {
            // int x = UCP.getValuesAsTwobitBytes(argv[1],argv[2]);
            // int y = UCP.getValuesAsTwobitBytes(argv[3],argv[4]);
            // int z = UCP.getValuesAsTwobitBytes(argv[5],argv[6]);
            short x = UCP.convertNumToCM(UCP.getValuesAsTwobitBytes(argv[1],argv[2]));
            short y = UCP.convertNumToCM(UCP.getValuesAsTwobitBytes(argv[3],argv[4]));
            short z = UCP.convertNumToCM(UCP.getValuesAsTwobitBytes(argv[5],argv[6]));
            // double t = UCP.convertDMSToSec(UCP.getValuesAsTwobitBytes(argv[7],argv[8]));
            uArm.moveTo(x,y,z); //   
            //uArm.moveTo(x,y,z); //   
            //uArm.moveTo(12.99,12.99,2.99,ABSOLUTE,2);
            if(UCP.debugMode)
            {
               UCP.sendString("Debug Mode Test");
               Serial.print("X is: ");
               Serial.println(x);
               Serial.print("Y is: ");
               Serial.println(y);
               Serial.print("Z is: ");
               Serial.println(z);
               // Serial.print("time is: ");
               // Serial.println(t);
            }
            break;
          }
        }
        break;
    case REPORT_FIRMWARE:
      UCP.printFirmwareVersion();
      break;
    case DEBUG_MODE:
      switch(argv[0]){
        case UCP_SWITCH_ON:
          UCP.debugMode = true;
          break;
        case UCP_SWITCH_OFF:
          UCP.debugMode = false;
          break;
        }
      break;    
    case UCP_TEST:
        Serial.print("UCP TEST");
        break;
  	}
}


