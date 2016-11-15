/**
  ******************************************************************************
  * @file	uArmRecorder.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-09-30
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#include "uArmRecorder.h" 

uArmRecorder gRecorder;

uArmRecorder::uArmRecorder()
{
	
}

void uArmRecorder::write(unsigned int addr, unsigned char data[], int num)
{
    unsigned char i=0;
    i = (addr % 128);
    // Since the eeprom's sector is 128 byte, if we want to write 5 bytes per cycle we need to care about when there's less than 5 bytes left
    if((i >= 124) && (num == 5))
    {
        i = 128 - i;
        iic_writebuf(data, EXTERNAL_EEPROM_USER_ADDRESS, addr, i);// write data
        delay(5);
        iic_writebuf(data + i, EXTERNAL_EEPROM_USER_ADDRESS, addr + i, num - i);// write data
    }
    //if the left bytes are greater than 5, just do it
    else
    {
        iic_writebuf(data, EXTERNAL_EEPROM_USER_ADDRESS, addr, num);// write data
    }
}

void uArmRecorder::read(unsigned int addr, unsigned char data[], int num)
{
    unsigned char i=0;
    i= (addr % 128);
    // Since the eeprom's sector is 128 byte, if we want to write 5 bytes per cycle we need to care about when there's less than 5 bytes left
    if( (i >= 124) && (num == 5))
    {
        i = 128 - i;
        iic_readbuf(data, EXTERNAL_EEPROM_USER_ADDRESS, addr, i);// write data
        delay(5);
        iic_readbuf(data + i, EXTERNAL_EEPROM_USER_ADDRESS, addr + i, num - i);// write data
    }
    //if the left bytes are greater than 5, just do it
    else
    {
        iic_readbuf(data, EXTERNAL_EEPROM_USER_ADDRESS, addr, num);// write data
    }
}


