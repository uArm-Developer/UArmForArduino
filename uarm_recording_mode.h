/******************************************************************************
* File Name          : uarm_recording_mode.h
* Author             : Alex
* Updated            : Alex
* Version            : V0.1
* Date               : 18 Feb, 2016
* Modified Date      : 20 Feb, 2016
* Description        :
* License            : 
* Copyright(C) 2016 UFactory Team. All right reserved.
*******************************************************************************/

#include <Arduino.h>
//#include "VarSpeedServo.h"
#include <Servo.h>

#ifndef uarm_recording_mode_h
#define uarm_recording_mode_h

// PIN
#define BTN_D4                  4     //
#define BTN_D7                  7     //
#define BUZZER                  3     //
#define LIMIT_SW                2     // Limit Switch
#define SERVO_HAND              9     //
#define SERVO_HAND_ROT          10    //
#define SERVO_ROT               11    //
#define SERVO_R                 12    //
#define SERVO_L                 13    //
#define BUFFER_OUTPUT					555
#define BTN_TIMEOUT_1000        1000
#define BTN_TIMEOUT_3000        3000

//SERVO FACTORY CONFIG
#define D090M_SERVO_MIN_PUL     500
#define D090M_SERVO_MAX_PUL     2500
#define D009A_SERVO_MIN_PUL     600
#define D009A_SERVO_MAX_PUL     2550
#define SERVO_MAX					605
#define SERVO_MIN					80

class uArmRecordClass
{
public:
	uArmRecordClass();
	void init();
	// void play(unsigned char buttonPin);
	void record(unsigned char recordDelay);
	void alert(int _times, int _runTime, int _stopTime);
	// void setServoSpeed(char _servoNum, unsigned char _servoSpeed); // 0=full speed, 1-255 slower to faster
	void play(unsigned char leftServo, unsigned char rightServo, unsigned char rotServo, unsigned char handrot, unsigned char recordDelay);
	void servoBufOutL(unsigned char _lastDt, unsigned char _dt);
	void servoBufOutR(unsigned char _lastDt, unsigned char _dt);
	void servoBufOutRot(unsigned char _lastDt, unsigned char _dt);
    void servoBufOutHandRot(unsigned char _lastDt, unsigned char _dt);	
    int  readAngle(char _servoNum);
    void attachAll();
private:
	boolean isRecording;
	boolean isPause;
	boolean isPlaying;
	Servo servoR;
	Servo servoL;
	Servo servoRot;
	Servo servoHand;
	Servo servoHandRot;
	unsigned char servoSpdR;
	unsigned char servoSpdL;
	unsigned char servoSpdRot;
	unsigned char servoSpdHand;
	unsigned char servoSpdHandRot;	
	unsigned char leftServoLast;
    unsigned char rightServoLast;
    unsigned char rotServoLast;	
    unsigned char handrotLast;
};

extern uArmRecordClass uarmRecord;
#endif