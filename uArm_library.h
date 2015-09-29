/******************************************************************************
* File Name          : uArm_Library_Metal.h
* Author             : Evan
* Updated            : Evan
* Version            : V0.0.1 (BATE)
* Created Date       : 12 Dec, 2014
* Modified Date      : 12 Dec, 2014
* Description        : 
* License            : 
* Copyright(C) 2014 UFactory Team. All right reserved.
*******************************************************************************/

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Servo.h>

#ifndef uArm_library_h
#define uArm_library_h

#define BUZZER                  				3
#define CALIBRATION_FLAG						0xEE

#define SERVO_ROT_NUM							1
#define SERVO_LEFT_NUM							2
#define SERVO_RIGHT_NUM							3
#define SERVO_HAND_ROT_NUM						4

class uArmClass 
{
public:
	uArmClass();
	

	double readServoOffset(byte servo_num);
    void detachServo(byte servo_num);
	void alert(byte times, byte runTime, byte stopTime);
    void saveDataToRom(double data, int addr);
    void attachAll();
    void detachAll();
	void writeAngle(byte servo_rot_angle, byte servo_left_angle, byte servo_right_angle, byte servo_hand_rot_angle);
	byte inputToReal(byte servo_num , byte input_angle);
	double readAngle(byte servo_num);
	double readToAngle(double input_angle, byte servo_num, byte trigger);
	void writeAngle(byte servo_rot_angle, byte servo_left_angle, byte servo_right_angle, byte servo_hand_rot_angle, byte trigger);
	// void writeAngle(byte servo_rot_angle, byte servo_left_angle, byte servo_right_angle);
	double readAngle(byte servo_num, byte trigger);

	const byte kAddrOffset      		= 90;
	const byte kAddrServo      			= 60;
	const byte kServoRotReadPin     	= 2;
	const byte kServoLeftReadPin    	= 0;
	const byte kServoRightReadPin    	= 1;
	const byte kServoHandRotReadPin     = 3;
private:
	/*****************  Define variables  *****************/
    unsigned int addr;

	Servo g_servo_rot;
	Servo g_servo_left;
	Servo g_servo_right;
	Servo g_servo_hand_rot;
	Servo g_servo_hand;
	
	double g_servo_offset;

};

extern uArmClass uarm;

#endif

