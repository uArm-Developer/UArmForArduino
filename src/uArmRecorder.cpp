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

void uArmRecorder::delay_us()
{


}

void uArmRecorder::iic_start()
{
        PORTC |= 0x20;//  SCL=1
        delay_us();
        PORTC |= 0x10;//  SDA=1
        delay_us();
        PORTC &= 0xEF;//  SDA=0
        delay_us();
        PORTC &= 0xDF;//  SCL=0
        delay_us();
}

void uArmRecorder::iic_stop()
{
        PORTC &= 0xDF;//  SCL=0
        delay_us();
        PORTC &= 0xEF;//  SDA=0
        delay_us();
        PORTC |= 0x20;//  SCL=1
        delay_us();
        PORTC |= 0x10;//  SDA=1
        delay_us();
}

//return 0:ACK=0
//return 1:NACK=1
unsigned char uArmRecorder::read_ack()
{
        unsigned char old_state;
        old_state = DDRC;
        DDRC = DDRC & 0xEF;//SDA INPUT
        PORTC |= 0x10;//  SDA = 1;
        delay_us();
        PORTC |= 0x20;//  SCL=1
        delay_us();
        if((PINC&0x10) == 0x10) // if(SDA)
        {
                PORTC &= 0xDF;//  SCL=0
                iic_stop();
                return 1;
        }
        else{
                PORTC &= 0xDF;//  SCL=0
                DDRC = old_state;
                return 0;
        }
}

//ack=0:send ack
//ack=1:do not send ack
void uArmRecorder::send_ack()
{
        unsigned char old_state;
        old_state = DDRC;
        DDRC = DDRC | 0x10;//SDA OUTPUT
        PORTC &= 0xEF;//  SDA=0
        delay_us();
        PORTC |= 0x20;//  SCL=1
        delay_us();
        PORTC &= 0xDF;//  SCL=0
        delay_us();
        DDRC = old_state;
        PORTC |= 0x10;//  SDA=1
        delay_us();
}

void uArmRecorder::iic_sendbyte(unsigned char dat)
{
        unsigned char i;
        for(i = 0; i < 8; i++)
        {
                if(dat & 0x80)
                        PORTC |= 0x10; //  SDA = 1;
                else
                        PORTC &= 0xEF; //  SDA = 0;
                dat <<= 1;
                delay_us();
                PORTC |= 0x20;//  SCL=1
                delay_us();
                PORTC &= 0xDF;//  SCL=0
        }
}

unsigned char uArmRecorder::iic_receivebyte()
{
        unsigned char i,byte = 0;
        unsigned char old_state;
        old_state = DDRC;
        DDRC = DDRC & 0xEF;//SDA INPUT
        for(i = 0; i < 8; i++)
        {
                PORTC |= 0x20;//  SCL=1
                delay_us();
                byte <<= 1;
                if((PINC&0x10) == 0x10) // if(SDA)
                        byte |= 0x01;
                delay_us();
                PORTC &= 0xDF;//  SCL=0
                DDRC = old_state;
                delay_us();
        }
        return byte;
}

unsigned char uArmRecorder::iic_writebuf(unsigned char *buf,unsigned char device_addr,unsigned int addr,unsigned char len)
{
        DDRC = DDRC | 0x30;
        PORTC = PORTC | 0x30;
        unsigned char length_of_data=0;//page write
        length_of_data = len;
        iic_start();
        iic_sendbyte(device_addr);
        if(read_ack()) return 1;
        iic_sendbyte(addr>>8);
        if(read_ack()) return 1;
        iic_sendbyte(addr%256);
        if(read_ack()) return 1;
        while(len != 0)
        {
                iic_sendbyte(*(buf + length_of_data - len));
                len--;
                if(read_ack()) return 1;
                delay(5);
        }
        iic_stop();

        return 0;
}

unsigned char uArmRecorder::iic_readbuf(unsigned char *buf,unsigned char device_addr,unsigned int addr,unsigned char len)
{
        DDRC = DDRC | 0x30;
        PORTC = PORTC | 0x30;
        unsigned char length_of_data=0;
        length_of_data = len;
        iic_start();
        iic_sendbyte(device_addr);
        if(read_ack()) return 1;
        iic_sendbyte(addr>>8);
        if(read_ack()) return 1;
        iic_sendbyte(addr%256);
        if(read_ack()) return 1;
        iic_start();
        iic_sendbyte(device_addr+1);
        if(read_ack()) return 1;

        while(len != 0)
        {
                *(buf + length_of_data - len) = iic_receivebyte();

                len--;
                if(len != 0) {
                        send_ack();
                }
        }
        iic_stop();
        return 0;
}