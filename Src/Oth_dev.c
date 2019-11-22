#include "Oth_Dev.h"
#include "string.h"

ATT Attitude;	
uint8_t buffer[BUFFER_SIZE];	/**< fifo buffer for received data. */
volatile uint8_t tail;					/**< index for writing to buffer. */
volatile uint8_t head;					/**< index for reading from buffer. */
uint8_t transfer_compl;
void InBuffer(uint8_t byte)
{
	if(tail == BUFFER_SIZE - 1)
		tail = 0;
    buffer[tail++] = byte;
}

/**
  * @brief  Function returns data from buffer.
  * @param  None.
  * @retval unsigned char current data.
  */
unsigned char OutBufferStr()
{
	if(head == BUFFER_SIZE - 1)
		head = 0;
	unsigned char data;
		data = buffer[head++];
	return data;
}

/**
  * @brief  Function check if new data was received.
  * @param  None.
  * @retval uint8_t result status.
  */
uint8_t IsNewData()
{
	if(head == tail)
			return 0;
	return 1;
}
	
/**
  * @brief  Function wipes off the buffer.
  * @param  None.
  * @retval None.
  */
void clearBuf()
{
	for(uint16_t i = 0; i < BUFFER_SIZE; i++)
			  		 	 buffer[i] = 0;
	head = 0;
	tail = 0;
}
	
uint8_t read_buf()
{
	uint8_t temp_arr[40];
	uint8_t byte = OutBufferStr();
	if(byte == 0xAA)
	{
		temp_arr[0] = byte;
		for(uint8_t i = 1; i < 40; i++)
		temp_arr[i] = OutBufferStr();
		
		memcpy(&Attitude, &temp_arr, 40);
	}
	else
		return 0;
	return 1;
}
		