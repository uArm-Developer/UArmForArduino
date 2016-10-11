/**
  ******************************************************************************
  * @file	uArmBuzzer.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-09-30
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#include "uArmBuzzer.h" 

uArmBuzzer::uArmBuzzer()
{
	mStopTime = 0;
}

void uArmBuzzer::buzz(unsigned int frequency, unsigned int duration)
{
	if (duration <= 0)
		return;

	mStopTime = millis() + duration;

	tone(BUZZER, frequency);
}

void uArmBuzzer::run()
{
	if (mStopTime > 0)
	{
		if (millis() >= mStopTime)
		{
			noTone(BUZZER);
			mStopTime = 0;
		}
	}
}

void uArmBuzzer::stop()
{
	noTone(BUZZER);
	mStopTime = 0;	
}