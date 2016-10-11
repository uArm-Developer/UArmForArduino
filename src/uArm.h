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
#include "uArmController.h"
#include "uArmBuzzer.h"
#include "uArmRecorder.h"

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

class uArmClass
{
public:
	uArmClass();
	void setup();
	void run();

	void systemRun();

	unsigned char moveTo(double x, double y, double z, double speed = 100);
	bool isMoving();
 	void stopMove();

public:
	uArmController mController;
	uArmBuzzer mBuzzer;
	uArmRecorder mRecorder;

private:
	void initHardware();
	char parseParam(String cmnd, const char *parameters, int parameterCount, double valueArray[]);
	void interpolate(double startVal, double endVal, double *interpVals, int steps, byte easeType);
	void controllerRun();
	bool play();
	bool record();

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

};
extern uArmClass uArm;




#endif // _UARM_H_
