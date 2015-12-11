/******************************************************************************
* File Name          : calibration.h
* Author             : Jerry.song
* Updated            : Alex Tan
* Version            : V0.3
* Created Date       : 12 Dec, 2014
* Modified Date      : 15 Sep, 2015
* Description        : 
* License            : 
* Copyright(C) 2014 UFactory Team. All right reserved.
*******************************************************************************/


#include <EEPROM.h>
#include <Wire.h>
#include <Servo.h>
#include <Arduino.h>


#ifndef uarm_calibration_h
#define uarm_calibration_h

class CalibrationClass{
public:
		CalibrationClass();
	    void calibration();
		void setOffset();
		void calibrations();
		void calibrationServo(byte servo_num);
		void saveOffsetValue(double value, byte servo_num);
		void cleanEEPROM();
		void cleanOFFSETS();
};

extern CalibrationClass calib;

#endif
