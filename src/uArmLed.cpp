/**
  ******************************************************************************
  * @file	uArmLed.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-10-17
  ******************************************************************************
  */

#include "uArmLed.h" 

uArmLed::uArmLed()
{
	mPin = 0xff;
	mState = INIT;
	mTicks = 0;
}

void uArmLed::setPin(unsigned char pin)
{
	mPin = pin;
	mState = OFF;
	mTicks = 0;
}

void uArmLed::on()
{
	digitalWrite(mPin, LOW); 
	mState = ON;
}

void uArmLed::off()
{
	digitalWrite(mPin, HIGH); 
	mState = OFF;
}

void uArmLed::blink()
{
	digitalWrite(mPin, LOW); 
	mState = BLINK_ON;
	mTicks = 0;
}

void uArmLed::tickInc()
{
	mTicks++;

	if (mTicks >= (500/TICK_INTERVAL))
	{
		mTicks = 0;

		if (mState == BLINK_ON)
		{	
			digitalWrite(mPin, HIGH); 
			mState = BLINK_OFF;
		}
		else
		{
			digitalWrite(mPin, LOW); 
			mState = BLINK_ON;
		}
	}
}

void uArmLed::tick()
{
	switch (mState)
	{	
	case BLINK_OFF:
	case BLINK_ON:
		tickInc();
		break;

	default:
		break;

	}
}