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


//#define MKII
#define METAL

//#define DEBUG                 // uncomment if you want to print debug info
#define DebugSerial   Serail    // usr serial0 as debug port

//#define METAL_MOTOR           // use servos made of metal




#ifdef MKII
  #define HW_VER  "3.1"
  #define SW_VER  "2.2.3"
#elif defined(METAL)
  #define HW_VER  "2.1"
  #define SW_VER  "2.2.3"
#else
  #error "NO machine model defined(METAL, MKII)"
#endif

#ifdef METAL
    #define DEVICE_NAME "Metal"
#elif defined(MKII)
    #define DEVICE_NAME "MKII"
#else
    #define DEVICE_NAME "UNKNOWN"
#endif

#define TICK_INTERVAL    50    // ms





#endif // _UARMCONFIG_H_
