/****************************************************************************************************
* File Name          : unpackage
* Author             : Joey Song
* Version            : V1.0
* Date               : 29 Aug, 2015
* Modified Date      : 29 Aug, 2015
* Description        : This documents is for unpackage a iphone box, only suitable for uArm metal
* Copyright(C) 2015 uArm Team. All right reserved.
*****************************************************************************************************/


#include <EEPROM.h>
#include <Wire.h>
#include <uArm_library.h>
#include <Servo.h>

// headers need to be loaded first


int valuel;
int buttonState;
int stopperPin = 2;                 // stopper is mounted under servo 4

int left_pos = 3;                   // left side of the center, move 3 cm left 
int right_pos = -3;                 // right side of the center, move 3 cm right 

int centerOfBox_x = 0;              // assume x of center of the box is 0
int centerOfBox_y = -18;            // assume y of center of the box is -18
int centerOfBox_z_case = 22;        // assume z of center of the box case is 22
int centerOfBox_z_others = 18;      // assume x of center of the other things is 18
int rotationDegreeLeft = 115;       // assume rotate object to left need 115 degree rotation
int rotationDegreeRight = 68;       // assume rotate object to right need 68 degree rotation

void setup() {
      
      Wire.begin();        // join i2c bus (address optional for master)
      Serial.begin(9600);  // start serial port at 9600 bps
//      uarm.init();
      pinMode(stopperPin,INPUT);
      
}

void loop(){
  
  if(Serial.available()>0)
  {
    
    char readSerial = Serial.read();
    Serial.println(readSerial);
    //read what you input to the console

    
    // first input a to the console to make sure uArm is located above the center of iphone box 
      if(readSerial == 'a')
    {
       uarm.moveTo(centerOfBox_x, centerOfBox_y, centerOfBox_z_case);
    }

    // then you can input b to execute main functions
    if(readSerial == 'b')
    {
      mainMove(centerOfBox_x, centerOfBox_y, centerOfBox_z_case, 0, -30, 19, 0);
        // Step 1: object 1 - big case 
        // initial position x,y,z = 0, -18, 22 (above the center of box)
        // destination position x = 0, y = -30, z = 19;
        // rotation angle = 0  means the object doesn't need to be rotated

      mainMove(centerOfBox_x, centerOfBox_y, centerOfBox_z_case, 0, -30, 19, 0);
        // Step 2: object 2 - iphone! 
        // initial position x,y,z = 0, -18, 18 (above the center of box)
        // destination position x = 0, y = -30, z = 19;
        // rotation angle = 0  means the object doesn't need to be rotated

      mainMove(right_pos, centerOfBox_y, centerOfBox_z_others, 14, -26, 18, rotationDegreeLeft);
        // Step 3: object 3 - iphone holder case
        // initial position x,y,z = 3, -18, 18 (left of the center of box, because there is a hold in the case)
        // destination position x = 14, y = -29, z = 18;
        // rotation angle != 0  means the object need to be rotated 
      
      mainMove(left_pos, centerOfBox_y, centerOfBox_z_others, -14, -26, 18, rotationDegreeRight);
        // Step 4: object 4 - user-manual 
        // initial position x,y,z = 0, -18, 18 (above the center of box)
        // destination position x = -14, y = -29, z = 18;
        // rotation angle != 0  means the object need to be rotated 
      
      mainMove(left_pos, centerOfBox_y-1, centerOfBox_z_others,10, -11, 17, 45);
        // Step 5: object 5 - earphone case
        // initial position x,y,z = 0, -18, 18 (above the center of box)
        // destination position x = 10, y = -11, z = 18;
        // rotation angle = 45  means the object need to be rotate 45 degree due to the rotation of base servo    
           
      mainMove(right_pos-2, centerOfBox_y-1, centerOfBox_z_others, -10, -11, 17, 45);
        // Step 6: object 6 - charger
        // initial position x,y,z = 0, -18, 18 (above the center of box)
        // destination position x = -10, y = -11, z = 18;
        // rotation angle = 45  means the object need to be rotate 45 degree due to the rotation of base servo  

      uarm.moveTo(centerOfBox_x, centerOfBox_y, centerOfBox_z_case);     // finally, move back to initial position
      digitalWrite(6,LOW);      // double check to disable the pump
      digitalWrite(5,LOW);
   }
   
   delay(50);
  } // end of reading
}


// main move procdure
void mainMove(double iniX, double iniY, double iniZ, double desX, double desY, double desZ, int rotDeg)
  // iniX, iniY and iniZ are the initial position of uArm
  // desX, desY, desZ are the desitination of uArm
  // rotDeg is the rotation degree of end-effector to rotate the object
{
  uarm.moveTo(iniX,iniY,iniZ);          // move the initial position
  absorbFcn(1,0, iniX, iniY , iniZ);    // move end-effector downwards until stopper hit something, 1 means begin to absorb 
  uarm.moveTo(0,0,10,1,6);              // move upwards for 10 cm in 6 seconds - slow upwards (relative = 1, means only move in the z axis)
  uarm.moveTo(desX,desY,desZ,0,2);      // move the destination position
  absorbFcn(2,rotDeg,desX,desY,desZ);   // move end-effector downwards until stopper hit something, 0 means disable the pump
  uarm.moveTo(0,0,5,1,4,rotDeg);        // move the upwards in 4 seconds - slow upwards
  uarm.moveTo(desX,desY,desZ);          // move to destination
}


// to control the pump to absorb stuff
void absorbFcn(int trigger, int rotDeg, double currentX, double currentY, double currentZ)
{
  buttonState = 1;                                // buttonState is the stopper state.
  while ( buttonState)
    {
      buttonState = digitalRead(stopperPin);      // if stopper hit anything, the reading would be LOW
      if(buttonState == HIGH)                     // while reading is HIGH, keep move downwards for 2 cm
      {
        uarm.moveTo(currentX,currentY,currentZ,0,0.5,rotDeg);
        currentZ = currentZ - 2;    // every time movedowards for 0.2 cm
        delay(50);
      }
      
    }
  
  // after stopper detects the end-effector hit something, pump will begin to work
  
  switch (trigger){
    case 1:                       //absorb
        digitalWrite(6,HIGH);     // begin to absorb
        digitalWrite(5,LOW);
        delay(500);               // wait 500 ms to make sure object is absorbed
      break;
    case 2:                       //release
        digitalWrite(6,LOW);      // begin to release quickly
        digitalWrite(5,HIGH);     
      break;
    default:
      break;
  }
  
}
