/**
  ******************************************************************************
  * @file	uArmRingBuffer.h
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-12-08
  ******************************************************************************
  */

#ifndef _UARMRINGBUFFER_H_
#define _UARMRINGBUFFER_H_

#include <Arduino.h>

class uArmRingBuffer
{

public:
	uArmRingBuffer();
	void init(uint8_t *data_buf, uint32_t buf_size);
	uint32_t put(uint8_t value);
	uint32_t get(uint8_t *value);
	uint32_t isFull();
	uint32_t isEmpty();

private:
	uint32_t head;
	uint32_t tail;
	uint8_t *data;
	uint32_t buffer_size;


};


#endif // _UARMRINGBUFFER_H_
