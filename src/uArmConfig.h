/**
  ******************************************************************************
  * @file	uArmConfig.h
  * @author	David.Long
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-09-28
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#ifndef _UARMCONFIG_H_
#define _UARMCONFIG_H_

#include <Arduino.h>

int ardprintf(char *result, char *str, ...);
// #define MKII
#define METAL

//#define DEBUG


//#define METAL_MOTOR

#ifdef DEBUG
	#define debugPrint	dprint
#else
	#define debugPrint
#endif


#ifdef MKII
  #define HW_VER  "3.1"
  #define SW_VER  "2.2.1"
#elif defined(METAL)
  #define HW_VER  "2.1"
  #define SW_VER  "2.2.1"
#endif

#ifdef METAL
    #define DEVICE_NAME "Metal"
#elif defined(MKII)
    #define DEVICE_NAME "MKII"
#else
    #define DEVICE_NAME "UNKNOWN"
#endif

#define TICK_INTERVAL    50    // ms


// conver double value to string
char* D(double value);

#ifdef DEBUG

void dprint(char *fmt, ...);

#ifdef F
void dprint(const __FlashStringHelper *fmt, ...);
#endif

#else

#endif // DEBUG


#endif // _UARMCONFIG_H_
