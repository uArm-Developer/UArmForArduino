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

#define BUZZER                  3    // HIGH = ON

class uArmBuzzer
{
public:
    uArmBuzzer();

    void buzz(unsigned int frequency, unsigned long duration);
    void stop();

};

#endif // _UARMBUZZER_H_
