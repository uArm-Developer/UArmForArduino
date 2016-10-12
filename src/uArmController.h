/**
  ******************************************************************************
  * @file	uArmController.h
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-09-28
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#ifndef _UARMCONTROLLER_H_
#define _UARMCONTROLLER_H_

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include "UFServo.h"
#include "uArmConfig.h"

#define SERVO_ROT_NUM           0
#define SERVO_LEFT_NUM          1
#define SERVO_RIGHT_NUM         2
#define SERVO_HAND_ROT_NUM      3
#define SERVO_COUNT				4

#define SERVO_ROT_PIN           11
#define SERVO_LEFT_PIN          13
#define SERVO_RIGHT_PIN         12
#define SERVO_HAND_ROT_PIN      10

#define SERVO_ROT_ANALOG_PIN 		2
#define SERVO_LEFT_ANALOG_PIN 		0
#define SERVO_RIGHT_ANALOG_PIN 		1
#define SERVO_HAND_ROT_ANALOG_PIN 	3

#define DEFAULT_ANGLE			60

#define GRABBING        		0
#define WORKING         		1
#define STOP            		2
#define PUMP_GRABBING_CURRENT 	55


#define MATH_PI 			3.141592653589793238463
#define MATH_TRANS  		57.2958    
#define MATH_L1 			90.00	
#define MATH_L2 			21.17	
#define MATH_LOWER_ARM 		148.25	
#define MATH_UPPER_ARM 		160.2 	
#define MATH_FRONT_HEADER 	25.00// the distance between wrist to the front point we use
#define MATH_UPPER_LOWER 	MATH_UPPER_ARM/MATH_LOWER_ARM


#define IN_RANGE             		1
#define OUT_OF_RANGE_NO_SOLUTION 	2
#define OUT_OF_RANGE         		3


#define LOWER_ARM_MAX_ANGLE      120
#define LOWER_ARM_MIN_ANGLE      5
#define UPPER_ARM_MAX_ANGLE      120
#define UPPER_ARM_MIN_ANGLE      5
#define LOWER_UPPER_MAX_ANGLE    150
#define LOWER_UPPER_MIN_ANGLE    30

class uArmController
{
public:
	uArmController();

	void init();

	void attachAllServo();
	void attachServo(byte servoNum);
	void detachServo(byte servoNum);
	void detachAllServo();

	void writeServoAngle(double servoRotAngle, double servoLeftAngle, double servoRightAngle);
	void writeServoAngle(byte servoNum, double servoAngle, boolean writeWithOffset = true);
	double readServoAngle(byte servoNum, boolean withOffset = true);
	void updateAllServoAngle(boolean withOffset = true);

	//double getServoAngle(byte servoNum);
	double getServoAngles(double& servoRotAngle, double& servoLeftAngle, double& servoRightAngle);

	void gripperCatch();
	void gripperRelease();
	unsigned char gripperStatus();
	void pumpOn();
	void pumpOff();

	unsigned char moveTo(double x, double y, double z, boolean allowApproximate = true);
	unsigned char getCurrentXYZ(double& x, double& y, double& z);
	unsigned char getXYZFromPolar(double& x, double& y, double& z, double s, double r, double h);
	unsigned char getXYZFromAngle(double& x, double& y, double& z, double rot, double left, double right);

	unsigned int getServoAnalogData(byte servoNum);
	unsigned char coordianteToAngle(double x, double y, double z, double& angleRot, double& angleLeft, double& angleRight, boolean allowApproximate = true);
	unsigned char limitRange(double& angleRot, double& angleLeft, double& angleRight);

private:
	double readServoAngleOffset(byte servoNum);
	void attachServo(byte servoNum, byte pin, int valueMin);
	
	void moveToStartPos(byte servoNum);
	double analogToAngle(byte servoNum, int inputAnalog);
	void readLinearOffset(byte servoNum, double& interceptVal, double& slopeVal);

	void sort(unsigned int array[], unsigned int len);

protected:
	Servo mServo[SERVO_COUNT];

	double mCurAngle[SERVO_COUNT] = {90, 90, 90, 90};

    //offset of assembling
	double mServoAngleOffset[SERVO_COUNT];
	
	const byte SERVO_CONTROL_PIN[SERVO_COUNT] = {SERVO_ROT_PIN, SERVO_LEFT_PIN, SERVO_RIGHT_PIN, SERVO_HAND_ROT_PIN};
	const byte SERVO_ANALOG_PIN[SERVO_COUNT] = {SERVO_ROT_ANALOG_PIN, SERVO_LEFT_ANALOG_PIN, SERVO_RIGHT_ANALOG_PIN, SERVO_HAND_ROT_ANALOG_PIN};
	const byte SERVO_ANALOG_MIN_VALUE[SERVO_COUNT] = {50, 50, 50, 50};

	//double mCurrentX = 0;
	//double mCurrentY = 200;
	//double mCurrentZ = 100;


};

#endif // _UARMCONTROLLER_H_
