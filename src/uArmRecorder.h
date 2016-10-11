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

#define EXTERNAL_EEPROM_USER_ADDRESS  0xA0

class uArmRecorder
{
public:
  	uArmRecorder();
	void write(unsigned int addr, unsigned char data[], int num);
	void read(unsigned int addr, unsigned char data[], int num);

private:
	void delay_us();
	void iic_start();

	void uArmRecorder::iic_stop();

	//return 0:ACK=0
	//return 1:NACK=1
	unsigned char uArmRecorder::read_ack();

	//ack=0:send ack
	//ack=1:do not send ack
	void send_ack();

	void iic_sendbyte(unsigned char dat);

	unsigned char iic_receivebyte();
	unsigned char iic_writebuf(unsigned char *buf,unsigned char device_addr,unsigned int addr,unsigned char len);
	unsigned char iic_readbuf(unsigned char *buf,unsigned char device_addr,unsigned int addr,unsigned char len);

};


#endif // _UARMRECORDER_H_
