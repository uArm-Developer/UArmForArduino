/**
  ******************************************************************************
  * @file	uArmDebug.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-12-02
  ******************************************************************************
  */

#include "uArmDebug.h" 
#include <stdarg.h>


#define ARDBUFFER 50

int msprintf(char *result, char *str, ...)
{
  int i, count=0, j=0, flag=0;
  char temp[ARDBUFFER+1];
  char num[ARDBUFFER+1];
  for(i=0; str[i]!='\0';i++)  if(str[i]=='%')  count++;

    result[0] = '\0';

  if (count == 0)
  {
    for (i = 0; str[i] != '\0'; i++)
    {
      result[i] = str[i];
    }
    result[i] = '\0';
  }
  else
  {

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
    }
  }
  temp[j] = '\0';
  strcat(result, temp);

  return count + 1;
}








void mprint(char *fmt, ...) 
{
	char buf[PRINT_BUF];

  int i, count=0, j=0, flag=0;
  char temp[ARDBUFFER+1];
  char num[ARDBUFFER+1];
  for(i=0; fmt[i]!='\0';i++) 
  {
    if(fmt[i]=='%')  
      count++;
  }

    if (count == 0)
    {
      for (i = 0; fmt[i] != '\0'; i++)
      {
        buf[i] = fmt[i];
      }
      buf[i] = '\0';
    }
    else
    {
      buf[0] = '\0';
      va_list argv;
      va_start(argv, count);
      for(i=0,j=0; fmt[i]!='\0';i++)
      { 
        if(fmt[i]=='%')
        {
          temp[j] = '\0';
          strcat(buf, temp);
          j=0;
          temp[0] = '\0';

          switch(fmt[++i])
          {
            case 'd': 
            {

                      itoa(va_arg(argv, int), num, 10);
                      strcat(buf, num);
                      break;
            }

            case 'l': ltoa(va_arg(argv, long), num, 10);
                      strcat(buf, num);
                      break;

            case 'f': 
            {
              char d_str[10];
              dtostrf(va_arg(argv, double), 4, 2, d_str);
              strcat(buf, d_str);
              break;
            }

            case 'c': num[0] = (char)va_arg(argv, int);
                      num[1] = '\0';
                      strcat(buf, num);
                      break;
            case 's': strcat(buf, va_arg(argv, char *));
                      break;
            default:  ;
          };
        }
        else 
        {
          temp[j] = fmt[i];
          j = (j+1)%ARDBUFFER;
          if(j==0) 
          {
            temp[ARDBUFFER] = '\0';
            strcat(buf, temp);
            temp[0]='\0';
          }
        }
      }
    }


  temp[j] = '\0';
  strcat(buf, temp);

	PrintSerial.println(buf);
}


