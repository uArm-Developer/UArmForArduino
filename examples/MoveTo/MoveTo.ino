// headers should must include these four headers
#include "uarm_library.h"  //name of uArm Library
#include <EEPROM.h>
#include <Wire.h>
#include <Servo.h>    

void setup() {

        Wire.begin();        // join i2c bus (address optional for master)

}

void loop() {

        uarm.moveTo(0,-20,20); //move uArm end effect to coordinate x=0, y=-20, z=20 from current location 
            exit(0);            //exit the loop

}
