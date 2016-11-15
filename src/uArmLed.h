/**
  ******************************************************************************
  * @file	uArmLed.h
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-10-17
  ******************************************************************************
  */

#ifndef _UARMLED_H_
#define _UARMLED_H_

#include <Arduino.h>
#include "uArmConfig.h"

class uArmLed
{
	typedef enum 
	{
		INIT,
		OFF,
		ON,
		BLINK_OFF,
		BLINK_ON,

		STATE_COUNT
		
	} LEDState;

public:
	uArmLed();

	void setPin(unsigned char pin);

	void on();
	void off();
	void blink();

	void tick();

private:
	void tickInc();

private:
	unsigned char mPin;
	LEDState mState;
	unsigned int mTicks;
};

#endif // _UARMLED_H_
