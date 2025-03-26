/*
 * FC_RC/RingBuffer.c
 *
 * Created: 2018-04-14 오후 3:12:06
 *  Author: dauera
 */

#include <FC_Basic/RingBuffer.h>

#include <assert.h>
#include <string.h>
#include <stdlib.h>
   
/**
  * @brief  ring buffer init.
  * @param  ptRB		
  					size 		
  * @retval unsigned int 0 성공
  */
unsigned char RB_init(RingFifo_t * ptRB, unsigned short size)
{

	if(size & (size-1))
	{
		assert(0);
		return 1;
	}

	ptRB->size = size;
	ptRB->wrIdx= 0;
	ptRB->rdIdx= 0;
	ptRB->data = malloc(size);

	assert(ptRB->data);

	return 0;
}

void RB_clear(RingFifo_t * ptRB)
{
		ptRB->wrIdx= 0;
		ptRB->rdIdx= 0;
		memset(ptRB->data, 0, ptRB->size);
}

void RB_write(RingFifo_t * ptRB, unsigned char data)
{
  if(RB_isfull(ptRB))
    return;
    
	ptRB->data[ptRB->wrIdx] = data;
	ptRB->wrIdx = (ptRB->size-1) & (ptRB->wrIdx+1);	
}

unsigned char RB_read(RingFifo_t * ptRB)
{

	unsigned char val = ptRB->data[ptRB->rdIdx];
	ptRB->rdIdx = (ptRB->size-1) & (ptRB->rdIdx+1);	

	return val;
}

unsigned char RB_isempty(RingFifo_t * ptRB)
{
	return (ptRB->rdIdx == ptRB->wrIdx);
}

unsigned char RB_isfull(RingFifo_t * ptRB)
{
	return ((ptRB->size-1) & ptRB->rdIdx) == ((ptRB->size-1) & (ptRB->wrIdx+1));
}

unsigned short RB_count(RingFifo_t * ptRB)
{
	return ((ptRB->size-1) & (ptRB->wrIdx - ptRB->rdIdx));
}

