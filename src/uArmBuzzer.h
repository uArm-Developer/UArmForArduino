/**
  ******************************************************************************
  * @file	uArmBuzzer.h
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-09-30
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#ifndef _UARMBUZZER_H_
#define _UARMBUZZER_H_

#include <Arduino.h>
#include "uArmConfig.h"



class uArmBuzzer
{
public:
    uArmBuzzer();

    void setPin(unsigned char pin);
    void buzz(unsigned int frequency, unsigned long duration);
    void stop();

    void run();

    bool on();

private:
    unsigned char mPin;
    bool mOn;
    unsigned long mStartTime;
    unsigned long mDuration;
};

extern uArmBuzzer gBuzzer;

#endif // _UARMBUZZER_H_
