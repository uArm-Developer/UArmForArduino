/*!
   \example GettingStarted.ino
   \file GettingStarted.ino
   \brief GettingStarted Arduino Sketch
   \author Joe Song
   \update Alex Tan
   \license GNU
   \copyright(c) 2016 UFactory Team. All right reserved
 */

/*
 * Table of Content
   Function 1 - 4 :    move to a certain point (f)
   Fucntion d : detach servos
   Function o : pump on
   Function f : pump off
   Function g : read current coordinate
   Function 5 : Read Servo offset


 */


#include "uarm_library.h"

int value;        // value is the data recevied

void setup() {

        Wire.begin();      // join i2c bus (address optional for master)
        Serial.begin(9600); // start serial port at 9600 bps
}


void loop() {

        if(Serial.available()>0)
        {

                char readSerial = Serial.read();
                Serial.println(readSerial);

                //----------------------------------  function 1  ------------------------------------
                // function below is for move uArm from current position to absolute coordinate
                // x = 13, y = -13, z = 3

                if (readSerial == '1') {
                        uarm.move_to(13,-13,3);
                        delay(1000);
                }

                //----------------------------------  function 2  ------------------------------------
                // function below is for move uArm from current position to absolute coordinate
                // x = -13, y = -13, z = 3

                if (readSerial == '2') {
                        uarm.move_to(-13,-13,3);
                        delay(1000);
                }

                //----------------------------------  function 3  ------------------------------------
                // function below is for move uArm from current position to relatvie coordinate
                // (dot) dx = 4, dy = -3, dz = 2 in 5 seconds

                if (readSerial == '3') {
                        uarm.move_to(5,0,0,RELATIVE,2);
                        delay(1000);
                }

                //----------------------------------  function 4  ------------------------------------
                // function below is for move uArm from current position to relatvie coordinate
                // (dot) dx = -4, dy = 3, dz = -2 in 5 seconds

                if (readSerial == '4') {
                        uarm.move_to(-4,3,-2,RELATIVE,5);
                        delay(1000);
                }

                //----------------------------------  function d  ------------------------------------
                // Detach Servo

                if (readSerial == 'd') {
                    uarm.set_servo_status(false, SERVO_ROT_NUM);
                    uarm.set_servo_status(false, SERVO_LEFT_NUM);
                    uarm.set_servo_status(false, SERVO_RIGHT_NUM);
                    uarm.set_servo_status(false, SERVO_HAND_ROT_NUM);
                }
                //----------------------------------  function a  ------------------------------------
                // Detach Servo

                if (readSerial == 'a') {
                    uarm.set_servo_status(true, SERVO_ROT_NUM);
                    uarm.set_servo_status(true, SERVO_LEFT_NUM);
                    uarm.set_servo_status(true, SERVO_RIGHT_NUM);
                    uarm.set_servo_status(true, SERVO_HAND_ROT_NUM);
                }

                //----------------------------------  function o  ------------------------------------
                // Pump on
                if (readSerial == 'o') {
                        uarm.pump_on();
                }
                //----------------------------------  function f  ------------------------------------
                // Pump off
                if (readSerial == 'f') {
                        uarm.pump_off();
                }

                //----------------------------------  function g  ------------------------------------
                // function below is for print current x,y,z absolute location

                if (readSerial == 'g') {
                        uarm.get_current_xyz();
                        Serial.print("The current location is ");
                        Serial.print(uarm.get_current_x());
                        Serial.print(" , ");
                        Serial.print(uarm.get_current_y());
                        Serial.print(" , ");
                        Serial.print(uarm.get_current_z());
                        Serial.println();
                        delay(1000);
                }
                //----------------------------------  function 5  ------------------------------------
                // function below is for read servo offset

                if (readSerial == '5') {
                        Serial.print("SERVO_ROT_NUM offset:");
                        Serial.println(uarm.read_servo_offset(SERVO_ROT_NUM));
                        Serial.print("SERVO_LEFT_NUM offset:");
                        Serial.println(uarm.read_servo_offset(SERVO_LEFT_NUM));
                        Serial.print("SERVO_RIGHT_NUM offset:");
                        Serial.println(uarm.read_servo_offset(SERVO_RIGHT_NUM));
                        Serial.print("SERVO_HAND_ROT_NUM offset:");
                        Serial.println(uarm.read_servo_offset(SERVO_HAND_ROT_NUM));
                }



        } // close read available
}
