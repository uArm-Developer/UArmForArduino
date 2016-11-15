/**
  ******************************************************************************
  * @file	  uArmComm.h
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	  2016-10-08
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#ifndef _UARMCOMM_H_
#define _UARMCOMM_H_

#include <Arduino.h>
#include "uArm.h"
#include "uArmBuzzer.h"

#define COM_LEN_MAX   60

enum CommState
{
  IDLE,
  START,
  CMD,
  END,

  STATE_COUNT
};


#define OUT_OF_RANGE      10
#define NO_SUCH_CMD       20
#define PARAMETER_ERROR   21
#define ADDRESS_ERROR     22

#define REPORT_POS    3

class uArmComm
{



public:
  uArmComm();

 	unsigned char cmdMove(int serialNum, int parameterCount, double value[4]);
 	unsigned char cmdMovePol(int serialNum, int parameterCount, double value[4]);
 	unsigned char cmdSetAttachServo(int serialNum, int parameterCount, double value[4]);
	unsigned char cmdSetDetachServo(int serialNum, int parameterCount, double value[4]);
 	unsigned char cmdSetServoAngle(int serialNum, int parameterCount, double value[4]);

 	 unsigned char cmdSetServoAngleWithOffset(int serialNum, int parameterCount, double value[4]);
 	 unsigned char cmdSetPump(int serialNum, int parameterCount, double value[4]);
 	 unsigned char cmdSetGripper(int serialNum, int parameterCount, double value[4]);
 	 unsigned char cmdSetBuzz(int serialNum, int parameterCount, double value[4]);
 	 unsigned char cmdStopMove(int serialNum, int parameterCount, double value[4]);

	 unsigned char cmdGetHWVersion(int serialNum, int parameterCount, double value[4]);
   unsigned char cmdGetSWVersion(int serialNum, int parameterCount, double value[4]);
	 unsigned char cmdSimulatePos(int serialNum, int parameterCount, double value[4]);
	 unsigned char cmdGetCurrentXYZ(int serialNum, int parameterCount, double value[4]);
 	 unsigned char cmdGetCurrentPosPol(int serialNum, int parameterCount, double value[4]); 
 	 unsigned char cmdGetCurrentAngle(int serialNum, int parameterCount, double value[4]);  	

 	 unsigned char cmdGetServoAngle(int serialNum, int parameterCount, double value[4]);  	
 	 unsigned char cmdCoordinateToAngle(int serialNum, int parameterCount, double value[4]);  	
 	 unsigned char cmdAngleToXYZ(int serialNum, int parameterCount, double value[4]);  	
 	 unsigned char cmdIsMoving(int serialNum, int parameterCount, double value[4]);  	
 	 unsigned char cmdGetTip(int serialNum, int parameterCount, double value[4]);  	

 	 unsigned char cmdGetDigitValue(int serialNum, int parameterCount, double value[4]);  	
  	 unsigned char cmdSetDigitValue(int serialNum, int parameterCount, double value[4]);  	
  	 unsigned char cmdGetAnalogValue(int serialNum, int parameterCount, double value[4]);  	
  	 unsigned char cmdGetE2PROMData(int serialNum, int parameterCount, double value[4]);  	
  	 unsigned char cmdSetE2PROMData(int serialNum, int parameterCount, double value[4]); 

     unsigned char cmdGetGripperStatus(int serialNum, int parameterCount, double value[4]);

   
     unsigned char cmdGetPumpStatus(int serialNum, int parameterCount, double value[4]);
#ifdef MKII      
     unsigned char cmdGetPowerStatus(int serialNum, int parameterCount, double value[4]);
#endif

     unsigned char cmdGetServoAnalogData(int serialNum, int parameterCount, double value[4]); 

     unsigned char cmdRelativeMove(int serialNum, int parameterCount, double value[4]);

     unsigned char cmdSetReportInterval(int serialNum, int parameterCount, double value[4]);
     unsigned char cmdGetDeviceName(int serialNum, int parameterCount, double value[4]);
     unsigned char cmdGetAPIVersion(int serialNum, int parameterCount, double value[4]);
     unsigned char cmdGetDeviceUUID(int serialNum, int parameterCount, double value[4]);

     void reportPos();

  	 void run();	

   bool parseCommand(char *message);
	 char parseParam(String cmnd, const char *parameters, int parameterCount, double valueArray[]);
	 void runCommand(String message);
	 void SerialCmdRun();
   void handleSerialData(char data);

 	 void printf(bool success, double *dat, char *letters, unsigned char num);
	 void printf(bool success, double dat);
	 void printf(bool success, int dat); 	

   void HandleMoveCmd(int cmdCode, int serialNum, int parameterCount, double value[4]);
   void HandleSettingCmd(int cmdCode, int serialNum, int parameterCount, double value[4]);
   void HandleQueryCmd(int cmdCode, int serialNum, int parameterCount, double value[4]);

   void replyOK(int serialNum);
   void replyError(int serialNum, unsigned int errorCode);
   void replyResult(int serialNum, String result);
   void reportResult(int reportCode, String result);
private:
   CommState mState;
   unsigned char cmdReceived[COM_LEN_MAX];
   unsigned char cmdIndex;
};

#endif // _UARMCOMM_H_
