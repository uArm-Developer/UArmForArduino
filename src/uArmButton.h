/**
  ******************************************************************************
  * @file	uArmButton.h
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-10-17
  ******************************************************************************
  */

#ifndef _UARMBUTTON_H_
#define _UARMBUTTON_H_

#include <Arduino.h>

#include "uArmConfig.h"
#include "uArmPin.h"
#include "uArmBuzzer.h"

#define EVENT_NONE			0
#define EVENT_CLICK			1
#define EVENT_LONG_PRESS	2

class uArmButton
{

	typedef enum {
		INIT,
		IDLE,
		HALF_PRESSED,
		PRESSED,
		RELEASE,
		LONGPRESSED,

		STATE_COUNT

	} BUTTON_STATE;

public:
	uArmButton();

	void setPin(unsigned char pin);

	bool clicked();
	bool longPressed();

	void clearEvent();
	void tick();

private:
	bool isPressed();

private:
	unsigned char mPin;
	unsigned int mTicks;

	unsigned char mEvent;

	BUTTON_STATE mState;
};



#endif // _UARMBUTTON_H_
