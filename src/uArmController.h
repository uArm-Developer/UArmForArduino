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
#include "uArmHWConfig.h"

#define SERVO_ROT_NUM           0
#define SERVO_LEFT_NUM          1
#define SERVO_RIGHT_NUM         2
#define SERVO_HAND_ROT_NUM      3
#define SERVO_COUNT				4



#define DEFAULT_ANGLE			60

#define GRABBING        		2
#define WORKING          		1
#define STOP            		0
#define PUMP_GRABBING_CURRENT 	55

#ifdef MKII

#define MATH_PI 			3.141592653589793238463
#define MATH_TRANS  		57.2958    
#define MATH_L1 			88.9	//90.00	
#define MATH_L2 			10		//21.17	
#define MATH_LOWER_ARM 		142.07	//148.25	
#define MATH_UPPER_ARM 		158.8	//160.2 	
#define MATH_FRONT_HEADER 	29.4	//25.00// the distance between wrist to the front point we use
#define MATH_UPPER_LOWER 	MATH_UPPER_ARM/MATH_LOWER_ARM

#elif defined(METAL)

#define MATH_PI 			3.141592653589793238463
#define MATH_TRANS  		57.2958    
#define MATH_L1 			107.45	
#define MATH_L2 			21.17	
#define MATH_LOWER_ARM 		148.25	
#define MATH_UPPER_ARM 		160.2 	
#define MATH_FRONT_HEADER 	25.00// the distance between wrist to the front point we use
#define MATH_UPPER_LOWER 	MATH_UPPER_ARM/MATH_LOWER_ARM

#endif

#define IN_RANGE             		1
#define OUT_OF_RANGE_NO_SOLUTION 	2
#define OUT_OF_RANGE         		3


#define LOWER_ARM_MAX_ANGLE      120
#define LOWER_ARM_MIN_ANGLE      5
#define UPPER_ARM_MAX_ANGLE      120
#define UPPER_ARM_MIN_ANGLE      5
#define LOWER_UPPER_MAX_ANGLE    150
#define LOWER_UPPER_MIN_ANGLE    30

#define LINEAR_INTERCEPT_START_ADDRESS      70
#define LINEAR_SLOPE_START_ADDRESS          50
#define MANUAL_OFFSET_ADDRESS               30
#define OFFSET_STRETCH_START_ADDRESS        20
#define SERIAL_NUMBER_ADDRESS               100

#define SERVO_9G_MAX    460
#define SERVO_9G_MIN    98

#define EXTERNAL_EEPROM_SYS_ADDRESS 0xA2

#define DATA_LENGTH  0x40
#define LEFT_SERVO_ADDRESS   0x0000
#define RIGHT_SERVO_ADDRESS  0x02D0
#define ROT_SERVO_ADDRESS    0x05A0

class uArmController
{
public:
	uArmController();

	void init();

	void attachAllServo();
	void attachServo(byte servoNum);
	void detachServo(byte servoNum);
	void detachAllServo();

	double getReverseServoAngle(byte servoNum, double servoAngle);
	void writeServoAngle(double servoRotAngle, double servoLeftAngle, double servoRightAngle);
	void writeServoAngle(byte servoNum, double servoAngle, boolean writeWithOffset = true);
	double readServoAngle(byte servoNum, boolean withOffset = true);
	double readServoAngles(double& servoRotAngle, double& servoLeftAngle, double& servoRightAngle, boolean withOffset = true);	
	void updateAllServoAngle(boolean withOffset = true);

	double getServoAngles(double& servoRotAngle, double& servoLeftAngle, double& servoRightAngle);
	double getServeAngle(byte servoNum);

	void gripperCatch();
	void gripperRelease();
	unsigned char gripperStatus();
	void pumpOn();
	void pumpOff();

	
	unsigned char pumpStatus();
	#ifdef MKII
	void readServoCalibrationData(unsigned int address, double& angle);	
	#endif

	unsigned char getCurrentXYZ(double& x, double& y, double& z);
	unsigned char getXYZFromPolar(double& x, double& y, double& z, double s, double r, double h);
	unsigned char getXYZFromAngle(double& x, double& y, double& z, double rot, double left, double right);

	unsigned int getAnalogData(byte pin);
	unsigned int getServoAnalogData(byte servoNum);
	unsigned char coordianteToAngle(double x, double y, double z, double& angleRot, double& angleLeft, double& angleRight, boolean allowApproximate = true);
	unsigned char limitRange(double& angleRot, double& angleLeft, double& angleRight);
	double analogToAngle(byte servoNum, int inputAnalog);
	
private:
	double readServoAngleOffset(byte servoNum);

	
	void readLinearOffset(byte servoNum, double& interceptVal, double& slopeVal);

	void sort(unsigned int array[], unsigned int len);

protected:
	Servo mServo[SERVO_COUNT];

	double mCurAngle[SERVO_COUNT] = {90, 90, 0, 90};

	unsigned int mMaxAdcPos[SERVO_COUNT] = {180};
    //offset of assembling
	double mServoAngleOffset[SERVO_COUNT];
	
	const byte SERVO_CONTROL_PIN[SERVO_COUNT] = {SERVO_ROT_PIN, SERVO_LEFT_PIN, SERVO_RIGHT_PIN, SERVO_HAND_ROT_PIN};
	const byte SERVO_ANALOG_PIN[SERVO_COUNT] = {SERVO_ROT_ANALOG_PIN, SERVO_LEFT_ANALOG_PIN, SERVO_RIGHT_ANALOG_PIN, SERVO_HAND_ROT_ANALOG_PIN};

};

#endif // _UARMCONTROLLER_H_
