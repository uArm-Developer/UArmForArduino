/**
  ******************************************************************************
  * @file	uArmConfig.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-09-28
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#include "uArmConfig.h" 

#ifdef DEBUG

#include <stdarg.h>

#define PRINT_BUF 	128



char* D(double value)
{

	static char d_str[5][7] = {0};

	static unsigned char d_index = 0;

	d_index++;
	if (d_index >= 5)
		d_index = 0;

	dtostrf(value, 4, 2, d_str[d_index]);
	return d_str[d_index];
}



void dprint(char *fmt, ...) 
{
	char buf[PRINT_BUF];

	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, PRINT_BUF, fmt, args);
	va_end(args);
	Serial.println(buf);
}


#ifdef F 

void dprint(const __FlashStringHelper *fmt, ...) 
{
	char buf[PRINT_BUF];

	va_list args;
	va_start(args, fmt);
#ifdef __AVR__
  	vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args); // progmem for AVR
#else
  	vsnprintf(buf, sizeof(buf), (const char *)fmt, args); // for the rest of the world
#endif
	va_end(args);
	Serial.println(buf);	
}

#endif // F


#else
char* D(double value)
{

	return 0; 
}

#endif // DEBUG