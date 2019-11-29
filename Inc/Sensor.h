#ifndef _SENSOR_H
#define _SENSOR_H

#include "stdint.h"
#include "main.h"
#include "MotionFX_Manager.h"
#include "iks01a2_motion_sensors.h"
#include "tm_stm32_ahrs_imu.h"

#define ALGO_FREQ    200U /* Algorithm frequency [Hz] */
#define ALGO_PERIOD  50   /* Algorithm period [ms] */
#define MOTIONFX_ENGINE_DELTATIME  0.049f

#define FROM_MG_TO_G                    0.001f
#define FROM_G_TO_MG                    1000.0f
#define FROM_MDPS_TO_DPS                0.001f
#define FROM_DPS_TO_MDPS                1000.0f
#define FROM_MGAUSS_TO_UT50             (0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS             500.0f
#define ACC_GYRO_ADDR 0xD7


 void Init_Sensors(void);
 void Lib_Process(void);
 void lib_init(TIM_HandleTypeDef* AlgoTimHandle);
 void Lib_ReadRequest(void);
 void FX_Data_Handler(void);
 void Accelero_Sensor_Handler(uint32_t Instance);
 void Gyro_Sensor_Handler(uint32_t Instance);
 void Magneto_Sensor_Handler(uint32_t Instance);
 void IMU_proc(void);
 
#endif
