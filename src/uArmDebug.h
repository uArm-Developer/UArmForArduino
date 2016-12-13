/**
  ******************************************************************************
  * @file	uArmDebug.h
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-12-02
  ******************************************************************************
  */

#ifndef _UARMDEBUG_H_
#define _UARMDEBUG_H_

#include <Arduino.h>
#include "uArmConfig.h"



#ifdef DEBUG
	#define debugPrint	mprint
#else
	#define debugPrint
#endif

#define PrintSerial		Serial

#define PRINT_BUF 	128


void mprint(char *fmt, ...);

int msprintf(char *result, char *str, ...);

#endif // _UARMDEBUG_H_
