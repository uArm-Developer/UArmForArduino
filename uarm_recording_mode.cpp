/******************************************************************************
* File Name          : uarm_recording_mode.cpp
* Author             : Alex
* Updated            : Alex
* Version            : V0.1
* Date               : 18 Feb, 2016
* Modified Date      : 20 Feb, 2016
* Description        :
* License            : 
* Copyright(C) 2016 UFactory Team. All right reserved.
*******************************************************************************/


#include "uarm_recording_mode.h"

uArmRecordClass uarmRecord;

uArmRecordClass::uArmRecordClass(){
	boolean isRecording = false;
	boolean isPause = true;	
	boolean isPlaying = false;
}

void uArmRecordClass::init(){
	pinMode(LIMIT_SW, INPUT_PULLUP);  
	pinMode(BTN_D4,   INPUT_PULLUP);  
	pinMode(BTN_D7,   INPUT_PULLUP);  
	pinMode(BUZZER,   OUTPUT);
	delay(1000);
	// attachAll();    
}

void uArmRecordClass::record(unsigned char recordDelay){
	// // if(!digitalRead(BTN_D4))
	// if(recordFlag)
	// {
	// 	delay(20);
	// 	// buzzer alert
	// 	alert(2, 50, 100);
	// 	isRecording = true;
	// }

	if(!digitalRead(LIMIT_SW)){
		Serial.print("[");
		Serial.print("M1");  
		Serial.println("]");
		// Serial.println("Recording Start");
		servoL.detach();
		servoR.detach();
		servoRot.detach();
		servoHandRot.detach();
	    // while(digitalRead(BTN_D7)){
	    while(!digitalRead(LIMIT_SW)){
			// if(!digitalRead(LIMIT_SW)){
				unsigned int leftServo  = map(constrain(readAngle(SERVO_L), SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX, 0, 180);
				unsigned int rightServo = map(constrain(readAngle(SERVO_R), SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX, 0, 180);
				unsigned int rotServo   = map(constrain(readAngle(SERVO_ROT), SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX, 0, 180);
	    		unsigned int handrot    = map(constrain(readAngle(SERVO_HAND_ROT), SERVO_MIN , SERVO_MAX), SERVO_MIN, SERVO_MAX, 0, 180);        		
				Serial.print("[");
				Serial.print("M1,");
				Serial.print("X");
				Serial.print(leftServo);
				Serial.print(",Y");		
				Serial.print(rightServo);
				Serial.print(",Z");		
				Serial.print(rotServo);
				Serial.print(",H");		
				Serial.print(handrot);
				Serial.println("]");        		
				isPause = false;
	    	// }
	   //  	else{
	   //  		if(!isPause){ //only send one time
	 		// 		Serial.print("[");
				// 	Serial.print("M2");  
				// 	Serial.println("]");
				// }
				// isPause = true;     		
	   //  	}
	    	delay(recordDelay);
		}
		Serial.print("[");
		Serial.print("M4");  
		Serial.println("]");		
		// Serial.println("Recording end");
		// isRecording = false;
		// alert(1, 500, 0);
		// attachAll();
	}

}

void uArmRecordClass::attachAll(){
	// servoL.attach(SERVO_L, D090M_SERVO_MIN_PUL, D090M_SERVO_MAX_PUL);
	// servoR.attach(SERVO_R, D090M_SERVO_MIN_PUL, D090M_SERVO_MAX_PUL);
	// servoRot.attach(SERVO_ROT, D090M_SERVO_MIN_PUL, D090M_SERVO_MAX_PUL);
	// servoHandRot.attach(SERVO_HAND_ROT, D090M_SERVO_MIN_PUL, D090M_SERVO_MAX_PUL);
	servoL.attach(SERVO_L );
	servoR.attach(SERVO_R );
	servoRot.attach(SERVO_ROT );
	servoHandRot.attach(SERVO_HAND_ROT);
	
}

void uArmRecordClass::play(unsigned char leftServo, unsigned char rightServo, unsigned char rotServo, unsigned char handrot, unsigned char recordDelay){
	// if(!isRecording){
		// if(!digitalRead(BTN_D7))
		// {
		// 	delay(20);
		// 	// buzzer alert
		// 	alert(2, 50, 100);
		// 	isPlaying = true;
		// }	
		attachAll();
	    servoBufOutL(leftServoLast,  leftServo);
	    servoBufOutR(rightServoLast, rightServo);
	    servoBufOutRot(rotServoLast, rotServo);
	    servoBufOutHandRot(handrotLast, handrot);		
	    leftServoLast  = leftServo;
	    rightServoLast = rightServo;
	    rotServoLast   = rotServo;
	    handrotLast    = handrot;
	    delay(recordDelay);	    
	// }
}

int uArmRecordClass::readAngle(char servoNum)
{
	int portAd;
	switch(servoNum)
	{
		case SERVO_L:
			portAd = A0;
			break;
		case SERVO_R:
			portAd = A1;
			break;
		case SERVO_ROT:
			portAd = A2;
			break;
		case SERVO_HAND_ROT:
			portAd = A3;
			break;
		case SERVO_HAND:
			portAd = A6;
			break;
		default: return 0; break;
	}
	int adAdd = 0;
	for(char i=0; i<5; i++)
		adAdd += analogRead(portAd);
	return adAdd/5;
}

// void uArmRecordClass::setServoSpeed(char _servoNum, unsigned char _servoSpeed) // 0=full speed, 1-255 slower to faster
// {
// 	switch(_servoNum)
// 	{
// 		case SERVO_L:
// 			servoSpdR = _servoSpeed;
// 			break;
// 		case SERVO_R:
// 			servoSpdL = _servoSpeed;
// 			break;
// 		case SERVO_ROT:
// 			servoSpdRot = _servoSpeed;
// 			break;
// 		case SERVO_HAND_ROT:
// 			servoSpdHand = _servoSpeed;
// 			break;
// 		case SERVO_HAND:
// 			servoSpdHandRot = _servoSpeed;
// 			break;
// 		default: break;
// 	}
// }

void uArmRecordClass::servoBufOutL(unsigned char _lastDt, unsigned char _dt)
{
	if(abs(_dt - _lastDt) > BUFFER_OUTPUT)     // goes from Min degrees to Max degrees 
	  // servoL.write(_dt, 40, true);
	  servoL.write(_dt);
	else 
	servoL.write(_dt); 
}

void uArmRecordClass::servoBufOutR(unsigned char _lastDt, unsigned char _dt)
{
	if(abs(_dt - _lastDt) > BUFFER_OUTPUT)
		servoR.write(_dt);
	else 
	servoR.write(_dt);
}

void uArmRecordClass::servoBufOutRot(unsigned char _lastDt, unsigned char _dt)
{
	if(abs(_dt - _lastDt) > BUFFER_OUTPUT)
		servoRot.write(_dt);
	else 
	servoRot.write(_dt);
}
void uArmRecordClass::servoBufOutHandRot(unsigned char _lastDt, unsigned char _dt)
{
	if(abs(_dt - _lastDt) > BUFFER_OUTPUT)
		servoHandRot.write(_dt);
	else 
	servoHandRot.write(_dt);
}

void uArmRecordClass::alert(int _times, int _runTime, int _stopTime)
{
	for(int _ct=0; _ct < _times; _ct++)
	{
		delay(_stopTime);
		digitalWrite(BUZZER, HIGH);
		delay(_runTime);
		digitalWrite(BUZZER, LOW);
	}
}