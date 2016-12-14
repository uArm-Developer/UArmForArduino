/*!
   \file 	uArmAPI.h
   \brief 	uArm API for Arduino
   \author 	David Long
   \license GNU
   \copyright(c) 2016 UFactory Team. All right reserved
 */

#ifndef _UARMAPI_H_
#define _UARMAPI_H_

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include "UFServo.h"
#include "uArmConfig.h"
#include "uArmPin.h"
#include "uArmController.h"
#include "uArmBuzzer.h"
#include "uArmRecorder.h"
#include "uArmButton.h"
#include "uArmLed.h"
#include "uArmService.h"
#include "uArmTypes.h"


extern uArmButton buttonMenu;	// D4 in Metal
extern uArmButton buttonPlay;	// D7 in Metal

#ifdef MKII
    extern uArmLed ledRed;		// red led in MKII
#endif

extern void manage_inactivity(void);

/*!
   \brief init components
 */
void uArmInit();

/*!
   \brief move to pos(x, y, z)
   \param x, y, z in mm
   \param speed: 
   			[0]: move to destination directly
   			[1~99]: change the dutycycle of servo (1~99%)
   			[100~1000]: mm/min, will do interpolation to control the speed and block process util move done
   \return IN_RANGE if everything is OK
   \return OUT_OF_RANGE_NO_SOLUTION if cannot reach
   \return OUT_OF_RANGE will move to the closest pos
   \return NO_NEED_TO_MOVE if it is already there
 */
unsigned char moveTo(double x, double y, double z, double speed = 100);

/*!
   \brief move to pos of polor coordinates(s, r, h)
   \param s: stretch(mm)
   \param r: angle (0~180)
   \param h: height(mm)
   \param speed (mm/min)
   \return IN_RANGE if everything is OK
   \return OUT_OF_RANGE_NO_SOLUTION if cannot reach
   \return OUT_OF_RANGE will move to the closest pos
   \return NO_NEED_TO_MOVE if it is already there
 */
unsigned char moveToPol(double s, double r, double h, double speed);

/*!
   \brief move to pos(x, y, z) according to current pos
   \param x, y, z (mm)
   \param speed (mm/min)
   \return IN_RANGE if everything is OK
   \return OUT_OF_RANGE_NO_SOLUTION if cannot reach
   \return OUT_OF_RANGE will move to the closest pos
   \return NO_NEED_TO_MOVE if it is already there
 */
unsigned char relativeMove(double x, double y, double z, double speed);

/*!
   \brief attach servo(0~3)
   \param servoNumber: SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
   \return true or false
 */
bool attachServo(unsigned char servoNumber);

/*!
   \brief detach servo(0~3)
   \param servoNumber: SERVO_ROT_NUM, SERVO_LEFT_NUM, SERVO_RIGHT_NUM, SERVO_HAND_ROT_NUM
   \return true or false
 */
bool detachServo(unsigned char servoNumber);

/*!
   \brief set servo angle
   \param servoNumber(0~3)
   \param angle (0~180)
   \return OK if everything is OK
   \return ERR_SERVO_INDEX_EXCEED_LIMIT if servoNumber not in range(0~3)
   \return ERR_ANGLE_OUT_OF_RANGE if angle not in range(0~180)
 */
unsigned char setServoAngle(unsigned char servoNumber, double angle);

/*!
   \brief get servo angle
   \param servoNumber(0~3)
   \return value of angle
   \return -1 if servoNumber not in range(0~3)
 */
double getServoAngle(unsigned char servoNumber);

/*!
   \brief gripper work
 */
void gripperCatch();

/*!
   \brief gripper stop
 */
void gripperRelease();

/*!
   \brief get gripper status
   \return STOP if gripper is not working
   \return WORKING if gripper is working but not catched sth
   \return GRABBING if gripper got sth   
 */
unsigned char getGripperStatus();

/*!
   \brief pump working
 */
void pumpOn();

/*!
   \brief pump stop
 */
void pumpOff();

/*!
   \brief get pump status
   \return STOP if pump is not working
   \return WORKING if pump is working but not catched sth
   \return GRABBING if pump got sth   
 */
unsigned char getPumpStatus();

/*!
   \brief get tip status
   \return true if limit switch hit
 */
bool getTip();


/*!
   \brief check pos reachable
   \return IN_RANGE if everything is OK
   \return OUT_OF_RANGE_NO_SOLUTION if cannot reach
   \return OUT_OF_RANGE can move to the closest pos
 */
unsigned char validatePos(double x, double y, double z);

/*!
   \brief convert polor coordinates to Cartesian coordinate
   \param s(mm), r(0~180), h(mm)
   \output x, y, z(mm)
 */
void polToXYZ(double s, double r, double h, double& x, double& y, double& z);

/*!
   \brief get current pos
   \output x, y, z(mm)
 */
void getCurrentXYZ(double& x, double& y, double& z);

/*!
   \brief get current pos of polor coordinates
   \output s(mm), r(0~180), h(mm)
 */
void getCurrentPosPol(double& s, double& r, double& h);

/*!
   \brief get servo angles from pos(x, y, z)
   \param x, y, z(mm)
   \output angles of servo(0~180)
   \return IN_RANGE if everything is OK
   \return OUT_OF_RANGE_NO_SOLUTION if cannot reach
   \return OUT_OF_RANGE can move to the closest pos   
 */
unsigned char xyzToAngle(double x, double y, double z, double& angleRot, double& angleLeft, double& angleRight);

/*!
   \brief get  pos(x, y, z) from servo angles  
   \param angles of servo(0~180)
   \output x, y, z(mm)
   \return IN_RANGE if everything is OK
   \return OUT_OF_RANGE_NO_SOLUTION if cannot reach
   \return OUT_OF_RANGE can move to the closest pos   
 */
unsigned char angleToXYZ(double angleRot, double angleLeft, double angleRight, double& x, double& y, double& z);

/*!
   \brief get pin value
   \param pin of arduino
   \return HIGH or LOW
 */
int getDigitalPinValue(unsigned int pin);

/*!
   \brief set pin value
   \param pin of arduino
   \param value: HIGH or LOW
 */
void setDigitalPinValue(unsigned int pin, unsigned char value);

/*!
   \brief get analog value of pin
   \param pin of arduino
   \return value of analog data
 */
int getAnalogPinValue(unsigned int pin);


/*!
   \brief get e2prom data
   \param device:  EEPROM_ON_CHIP, EEPROM_EXTERN_USER, EEPROM_EXTERN_SYSTEM
   \param addr: 0~2047(EEPROM_ON_CHIP), 0~65535(EEPROM_EXTERN_USER, EEPROM_EXTERN_SYSTEM)
   \param type: DATA_TYPE_BYTE, DATA_TYPE_INTEGER, DATA_TYPE_FLOAT
 */
double getE2PROMData(unsigned char device, unsigned int addr, unsigned char type);

/*!
   \brief set e2prom data
   \param device:  EEPROM_ON_CHIP, EEPROM_EXTERN_USER, EEPROM_EXTERN_SYSTEM
   \param addr: 0~2047(EEPROM_ON_CHIP), 0~65535(EEPROM_EXTERN_USER, EEPROM_EXTERN_SYSTEM)
   \param type: DATA_TYPE_BYTE, DATA_TYPE_INTEGER, DATA_TYPE_FLOAT
   \param value: value to write
 */
double setE2PROMData(unsigned char device, unsigned int addr, unsigned char type, double value);

#ifdef MKII	
/*!
   \brief stop move immediately
 */
void stopMove();

/*!
   \brief is moving now
 */
bool isMoving();


/*!
   \brief check if power plug in
 */
	bool isPowerPlugIn();
#endif

#endif // _UARMAPI_H_
