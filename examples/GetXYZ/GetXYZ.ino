// headers should must include these four headers
#include "uarm_library.h"
#include <EEPROM.h>
#include <Wire.h>
#include <Servo.h>


int x ;
int y ;
int z ;

void setup() {

    Wire.begin();        // join i2c bus (address optional for master)
    Serial.begin(9600);  // start serial port at 9600 bps
}


void loop() {

    // detach uArm in case of attached, so you can manually move uArm
    uarm.detachAll();
    // assign robot coordinate to x,y,z.
    // unit is in centimeter
    x = uarm.getCalX();  // assign robot x - axis coordinate to value x
    y = uarm.getCalY();
    z = uarm.getCalZ();

    // print x,y,z into command console 
    Serial.print("The current location is ");
    Serial.print(x);
    Serial.print(" , ");
    Serial.print(y);
    Serial.print(" , ");
    Serial.print(z);
    Serial.println();

    delay(500);   // display x,y,z every 500 ms.
}