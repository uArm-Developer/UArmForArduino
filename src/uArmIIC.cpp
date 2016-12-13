/**
  ******************************************************************************
  * @file	uArmIIC.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-10-18
  ******************************************************************************
  */

#include "uArmIIC.h" 
void delay_us()
{


}

void iic_start()
{
        SCL_SET;//  SCL=1
        delay_us();
        SDA_SET;//  SDA=1
        delay_us();
        SDA_CLEAR;//  SDA=0
        delay_us();
        SCL_CLEAR;//  SCL=0
        delay_us();
}

void iic_stop()
{
        SCL_CLEAR;//  SCL=0
        delay_us();
        SDA_CLEAR;//  SDA=0
        delay_us();
        SCL_SET;//  SCL=1
        delay_us();
        SDA_SET;//  SDA=1
        delay_us();
}

//return 0:ACK=0
//return 1:NACK=1
unsigned char read_ack()
{
        unsigned char old_state;
        old_state = PORT_DDR;
        SDA_INPUT;//SDA INPUT
        SDA_SET;//  SDA = 1;
        delay_us();
        SCL_SET;//  SCL=1
        delay_us();
        if(SDA_READ) // if(SDA)
        {
                SCL_CLEAR;//  SCL=0
                iic_stop();
                PORT_DDR = old_state;
                return 1;
        }
        else{
                SCL_CLEAR;//  SCL=0
                PORT_DDR = old_state;
                return 0;
        }
}

//ack=0:send ack
//ack=1:do not send ack
void send_ack()
{
        unsigned char old_state;
        old_state = PORT_DDR;
        SDA_OUTPUT;//SDA OUTPUT
        SDA_CLEAR;//  SDA=0
        delay_us();
        SCL_SET;//  SCL=1
        delay_us();
        SCL_CLEAR;//  SCL=0
        delay_us();
        PORT_DDR = old_state;
        SDA_SET;//  SDA=1
        delay_us();
}

void iic_sendbyte(unsigned char dat)
{
        unsigned char i;
        for(i = 0; i < 8; i++)
        {
                if(dat & 0x80)
                        SDA_SET; //  SDA = 1;
                else
                        SDA_CLEAR; //  SDA = 0;
                dat <<= 1;
                delay_us();
                SCL_SET;//  SCL=1
                delay_us();
                SCL_CLEAR;//  SCL=0
        }
}

unsigned char iic_receivebyte()
{
        unsigned char i,byte = 0;
        unsigned char old_state;
        old_state = PORT_DDR;
        SDA_INPUT;//SDA INPUT
        for(i = 0; i < 8; i++)
        {
                SCL_SET;//  SCL=1
                delay_us();
                byte <<= 1;
                if(SDA_READ) // if(SDA)
                        byte |= 0x01;
                delay_us();
                SCL_CLEAR;//  SCL=0
               
                delay_us();
        }
        PORT_DDR = old_state;
        return byte;
}

unsigned char iic_writebuf(unsigned char *buf,unsigned char device_addr,unsigned int addr,unsigned char len)
{
        SCL_OUTPUT;
        SDA_OUTPUT;
        SCL_SET;
        SDA_SET;

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

unsigned char iic_readbuf(unsigned char *buf,unsigned char device_addr,unsigned int addr,unsigned char len)
{
        SCL_OUTPUT;
        SDA_OUTPUT;
        SCL_SET;
        SDA_SET;
        
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