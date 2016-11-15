/**
  ******************************************************************************
  * @file	uArm.h
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-09-28
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#ifndef _UARM_H_
#define _UARM_H_

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include "UFServo.h"
#include "uArmConfig.h"
#include "uArmHWConfig.h"
#include "uArmController.h"
#include "uArmBuzzer.h"
#include "uArmRecorder.h"
#include "uArmButton.h"
#include "uArmLed.h"

#define STEP_MAX	60

#define INTERP_EASE_INOUT_CUBIC 0  // original cubic ease in/out
#define INTERP_LINEAR           1
#define INTERP_EASE_INOUT       2  // quadratic easing methods
#define INTERP_EASE_IN          3
#define INTERP_EASE_OUT         4

#define NORMAL_MODE                 0
#define NORMAL_BT_CONNECTED_MODE    1
#define LEARNING_MODE               2
#define SINGLE_PLAY_MODE            3
#define LOOP_PLAY_MODE              4
#define LEARNING_MODE_STOP          5

#define STEP_MAX_TIME				20	// ms



#define OK                      0
#define ERR1                    1
#define ERR2                    2

#define SS   "[S]"
#define S0  "[S0]"
#define S1  "[S1]"
#define S2  "[S2]"
#define FF   "[F]"
#define F0  "[F0]"
#define F1  "[F1]"

// Calibration Flag & OFFSET EEPROM ADDRESS
#define CALIBRATION_FLAG                    10
#define CALIBRATION_LINEAR_FLAG             11
#define CALIBRATION_MANUAL_FLAG             12
#define CALIBRATION_STRETCH_FLAG            13



//#define TIME_PER_STEP 	10

#define SERVO_9G_MAX    460
#define SERVO_9G_MIN    98

#define CONFIRM_FLAG                        0x80


#define SUCCESS                 1
#define FAILED                  -1

#define DATA_TYPE_BYTE          1
#define DATA_TYPE_INTEGER       2
#define DATA_TYPE_FLOAT         4

#define EXTERNAL_EEPROM_SYS_ADDRESS 0xA2
#define EXTERNAL_EEPROM_USER_ADDRESS  0xA0

class uArmClass
{
public:
	uArmClass();

	void setup();
	void run();

	unsigned char moveTo(double x, double y, double z, double speed = 100);
	unsigned char moveToAngle(double x, double y, double z);

	bool isMoving();
 	void stopMove();
	void setReportInterval(unsigned int interval);
	void reportPos();

#ifdef MKII	
	bool isPowerPlugIn();
#endif

public:
	uArmController mController;
	uArmRecorder mRecorder;

private:
	void initHardware();
	char parseParam(String cmnd, const char *parameters, int parameterCount, double valueArray[]);
	void interpolate(double startVal, double endVal, double *interpVals, int steps, byte easeType);
	void interpolateEven(double startVal, double endVal, double *interpVals, int steps, byte easeType);

	void controllerRun();
	void systemRun();
	bool play();
	bool record();
	void tickTaskRun();
	void recorderTick();

	void btDetect();



private:
	int mCurStep;
	int mTotalSteps;
	unsigned int mTimePerStep;
	unsigned long mStartTime;

	double mPathX[STEP_MAX];
	double mPathY[STEP_MAX];
	double mPathZ[STEP_MAX];

	unsigned char mSysStatus = NORMAL_MODE;
	unsigned int mRecordAddr = 0;

	unsigned char mTime50ms;
	unsigned long mTickStartTime;

	uArmButton mButtonD4;
	uArmButton mButtonD7;

	unsigned int mReportInterval;	//  ms. 0 means no report
	unsigned int mTimeInterval;
	unsigned long mReportStartTime;

	unsigned long mTickRecorderTime;	
#ifdef MKII
	uArmLed mLed;
#endif
};
extern uArmClass uArm;




#endif // _UARM_H_
