#ifndef _OTH_DEV_H
#define _OTH_DEV_H

#include "stdint.h"
#include "MotionFX_Manager.h"
#include "tm_stm32_ahrs_imu.h"

#define BUFFER_SIZE 256
#define BUFFER_MASK (BUFFER_SIZE - 1)

typedef struct 
	{
		uint8_t preamble; 
		float RollDeg;
		float PitchDeg;
		float YawDeg;
		float Gx;
		float Gy;
		float Gz;
		float q0;
		float q1;
		float q2;
		float q3;
	}  ATT;

	
void InBuffer(uint8_t);
uint8_t OutBufferStr();
uint8_t IsNewData();
void clearBuf();
uint8_t read_buf(void);
void calculate_attitude(void);
	
#endif

