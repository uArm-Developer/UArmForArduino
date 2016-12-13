/**
  ******************************************************************************
  * @file	uArmIIC.h
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-10-18
  ******************************************************************************
  */

#ifndef _UARMIIC_H_
#define _UARMIIC_H_

#include <Arduino.h>
#include "uArmConfig.h"
#include "uArmPin.h"


#ifdef MKII



#define SDA_SET		PORTB |= 0x02
#define SDA_CLEAR	PORTB &= 0xFD
#define SCL_SET		PORTB |= 0x01	
#define SCL_CLEAR	PORTB &= 0xFE

#define SDA_INPUT	DDRB &= 0xFD
#define SDA_OUTPUT	DDRB |= 0x02

#define SCL_INPUT	DDRB &= 0xFE
#define SCL_OUTPUT	DDRB |= 0x01

#define SDA_READ	PINB & 0x02

#define PORT_DDR	DDRB


#elif defined(METAL)


#define SDA_SET		PORTC |= 0x10
#define SDA_CLEAR	PORTC &= 0xEF
#define SCL_SET		PORTC |= 0x20	
#define SCL_CLEAR	PORTC &= 0xDF

#define SDA_INPUT	DDRC &= 0xEF
#define SDA_OUTPUT	DDRC |= 0x10

#define SCL_INPUT	DDRC &= 0xDF
#define SCL_OUTPUT	DDRC |= 0x20

#define SDA_READ	PINC & 0x10

#define PORT_DDR	DDRC

#endif



void delay_us();

void iic_start();
void iic_stop();

//return 0:ACK=0
//return 1:NACK=1
unsigned char read_ack();

//ack=0:send ack
//ack=1:do not send ack
void send_ack();

void iic_sendbyte(unsigned char dat);

unsigned char iic_receivebyte();

unsigned char iic_writebuf(unsigned char *buf,unsigned char device_addr,unsigned int addr,unsigned char len);

unsigned char iic_readbuf(unsigned char *buf,unsigned char device_addr,unsigned int addr,unsigned char len);





#endif // _UARMIIC_H_
