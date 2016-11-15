/**
  ******************************************************************************
  * @file	uArmRecorder.h
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-09-30
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#ifndef _UARMRECORDER_H_
#define _UARMRECORDER_H_

#include <Arduino.h>
#include "uArmConfig.h"
#include "uArmHWConfig.h"
#include "uArmIIC.h"




class uArmRecorder
{
public:
  	uArmRecorder();
	void write(unsigned int addr, unsigned char data[], int num);
	void read(unsigned int addr, unsigned char data[], int num);

private:


};

extern uArmRecorder gRecorder;


#endif // _UARMRECORDER_H_
