/**
  ******************************************************************************
  * @file	uArmRingBuffer.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-12-08
  ******************************************************************************
  */

#include "uArmRingBuffer.h" 

uArmRingBuffer::uArmRingBuffer()
{

}

void uArmRingBuffer::init(uint8_t *data_buf, uint32_t buf_size)
{
	head = 0;
	tail = 0;
	data = data_buf;
	buffer_size = buf_size;
}


uint32_t uArmRingBuffer::put(uint8_t value)
{

	if (isFull())
	{
		return 0;
	}

	data[tail] = value;

	tail = (tail + 1) % buffer_size;
	
	return 1;

}

uint32_t uArmRingBuffer::get(uint8_t *value)
{

	if (isEmpty())
		return 0;
	
	*value = data[head];
	
	head = (head + 1) % buffer_size;

	
	return 1;
	
}

uint32_t uArmRingBuffer::isFull()
{
	return ((tail + 1) % buffer_size) == head ? 1 : 0;
}

uint32_t uArmRingBuffer::isEmpty()
{
	return head == tail ? 1 : 0;
}