/**
  ******************************************************************************
  * @file	uArmConfig.h
  * @author	David.Long
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-09-28
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#ifndef _UARMCONFIG_H_
#define _UARMCONFIG_H_

#include <Arduino.h>

//#define DEBUG

#ifdef DEBUG
	#define debugPrint	dprint
#else
	#define debugPrint
#endif

#define current_ver         "[SH2-2.1.4]"

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

#define LINEAR_INTERCEPT_START_ADDRESS      70
#define LINEAR_SLOPE_START_ADDRESS          50
#define MANUAL_OFFSET_ADDRESS               30
#define OFFSET_STRETCH_START_ADDRESS        20
#define SERIAL_NUMBER_ADDRESS               100

#define LIMIT_SW                2    // LIMIT Switch Button

#define BTN_D4                  4    // LOW = Pressed
#define BTN_D7                  7    // LOW = Pressed

#define PUMP_EN                 6    // HIGH = Valve OPEN
#define VALVE_EN                5    // HIGH = Pump ON
#define GRIPPER                 9    // LOW = Catch
#define GRIPPER_FEEDBACK        A6

#define SERVO_9G_MAX    460
#define SERVO_9G_MIN    98

#define CONFIRM_FLAG                        0x80


#define SUCCESS                 1
#define FAILED                  -1

#define DATA_TYPE_BYTE          1
#define DATA_TYPE_INTEGER       2
#define DATA_TYPE_FLOAT         4


char* D(double value);
#ifdef DEBUG


void dprint(char *fmt, ...);


#ifdef F
void dprint(const __FlashStringHelper *fmt, ...);
#endif

#else

#endif // DEBUG



#endif // _UARMCONFIG_H_
