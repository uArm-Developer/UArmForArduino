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
#include <stdarg.h>

#define ARDBUFFER 50




int ardprintf(char *result, char *str, ...)
{
  int i, count=0, j=0, flag=0;
  char temp[ARDBUFFER+1];
  char num[ARDBUFFER+1];
  for(i=0; str[i]!='\0';i++)  if(str[i]=='%')  count++;

    result[0] = '\0';
  va_list argv;
  va_start(argv, count);
  for(i=0,j=0; str[i]!='\0';i++)
  {
    if(str[i]=='%')
    {
      temp[j] = '\0';
      strcat(result, temp);
      j=0;
      temp[0] = '\0';

      switch(str[++i])
      {
        case 'd': 
        {

                  itoa(va_arg(argv, int), num, 10);
                  strcat(result, num);
                  break;
        }

        case 'l': ltoa(va_arg(argv, long), num, 10);
                  strcat(result, num);
                  break;

        case 'f': 
        {
          char d_str[10];
          dtostrf(va_arg(argv, double), 4, 2, d_str);
          strcat(result, d_str);
          break;
        }

        case 'c': num[0] = (char)va_arg(argv, int);
                  num[1] = '\0';
                  strcat(result, num);
                  break;
        case 's': strcat(result, va_arg(argv, char *));
                  break;
        default:  ;
      };
    }
    else 
    {
      temp[j] = str[i];
      j = (j+1)%ARDBUFFER;
      if(j==0) 
      {
        temp[ARDBUFFER] = '\0';
        strcat(result, temp);
        temp[0]='\0';
      }
    }
  };

  return count + 1;
}



#ifdef DEBUG



#define PRINT_BUF 	128



// convert double value to string
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

  
  return NULL;
}

#endif // DEBUG