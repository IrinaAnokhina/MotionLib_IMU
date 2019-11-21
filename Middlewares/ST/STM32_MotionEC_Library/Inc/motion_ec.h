/**
  ******************************************************************************
  * @file    motion_ec.h
  * @author  MEMS Application Team
  * @version V1.1.1
  * @date    15-February-2019
  * @brief   Header for motion_ec module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MOTION_EC_H_
#define _MOTION_EC_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup MIDDLEWARES
  * @{
  */

/** @defgroup MOTION_EC MOTION_EC
  * @{
  */

/** @defgroup MOTION_EC_Exported_Types MOTION_EC_Exported_Types
  * @{
  */

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  float acc[3];        /* Accelerometer data [g] */
  float mag[3];        /* Magnetometer data [uT / 50] */
  float deltatime_s;   /* Time between 2 library calls in [s] */
} MEC_input_t;

typedef struct
{
  float quaternion[4]; /* Quaternion [x, y, z, w] */
  float euler[3];      /* Yaw, Pitch, Roll [deg] */
  float i_gyro[3];     /* Virtual gyroscope [dps] */
  float gravity[3];    /* Gravity [g] */
  float linear[3];     /* Linear acceleration [g] */
} MEC_output_t;

typedef enum
{
  MEC_DISABLE = 0,
  MEC_ENABLE  = 1
} MEC_state_t;

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup MOTION_EC_Exported_Functions MOTION_EC_Exported_Functions
  * @{
  */

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  Initialize and reset the MotionEC engine
  * @param  freq  Sensors sampling frequency [Hz]
  * @retval None
  */
void MotionEC_Initialize(float *freq);

/**
  * @brief  Run E-Compass algorithm (accelerometer and magnetometer data fusion)
  * @param  data_in  Structure containing input data
  * @param  data_out  Structure containing output data
  * @retval None
  */
void MotionEC_Run(MEC_input_t *data_in, MEC_output_t *data_out);

/**
  * @brief  Get enable/disable state of the Euler angles calculation
  * @param  state  Current enable/disable state
  * @retval None
  */
void MotionEC_GetOrientationEnable(MEC_state_t *state);

/**
  * @brief  Set enable/disable state of the Euler angles calculation
  * @param  state  New enable/disable state to be set
  * @retval None
  */
void MotionEC_SetOrientationEnable(MEC_state_t state);

/**
  * @brief  Get enable/disable state of the virtual gyroscope calculation
  * @param  state  Current enable/disable state
  * @retval None
  */
void MotionEC_GetVirtualGyroEnable(MEC_state_t *state);

/**
  * @brief  Set enable/disable state of the virtual gyroscope calculation
  * @param  state  New enable/disable state to be set
  * @retval None
  */
void MotionEC_SetVirtualGyroEnable(MEC_state_t state);

/**
  * @brief  Get enable/disable state of the gravity vector calculation
  * @param  state  Current enable/disable state
  * @retval None
  */
void MotionEC_GetGravityEnable(MEC_state_t *state);

/**
  * @brief  Set enable/disable state of the gravity vector calculation
  * @param  state  New enable/disable state to be set
  * @retval None
  */
void MotionEC_SetGravityEnable(MEC_state_t state);

/**
  * @brief  Get enable/disable state of the linear acceleration calculation
  * @param  state  Current enable/disable state
  * @retval None
  */
void MotionEC_GetLinearAccEnable(MEC_state_t *state);

/**
  * @brief  Set enable/disable state of the linear acceleration calculation
  * @param  state  New enable/disable state to be set
  * @retval None
  */
void MotionEC_SetLinearAccEnable(MEC_state_t state);

/**
  * @brief  Set sampling frequency (modify filtering parameters)
  * @param  freq  New sensors sampling frequency [Hz]
  * @retval None
  */
void MotionEC_SetFrequency(float freq);

/**
  * @brief  Get the library version
  * @param  version  pointer to an array of 35 char
  * @retval Number of characters in the version string
  */
uint8_t MotionEC_GetLibVersion(char *version);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* _MOTION_EC_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
