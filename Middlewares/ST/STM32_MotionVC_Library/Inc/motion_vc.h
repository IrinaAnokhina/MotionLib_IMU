/**
  ******************************************************************************
  * @file    motion_vc.h
  * @author  MEMS Application Team
  * @version V1.0.1
  * @date    15-February-2019
  * @brief   Header for motion_vc module
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
  ********************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MOTION_VC_H_
#define _MOTION_VC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup MIDDLEWARES
  * @{
  */

/** @defgroup MOTION_VC MOTION_VC
  * @{
  */


/** @defgroup MOTION_VC_Exported_Types MOTION_VC_Exported_Types
 * @{
 */
/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
 typedef struct
{
  float AccX;           /* Acceleration in X axis in [g] */
  float AccY;           /* Acceleration in Y axis in [g] */
  float AccZ;           /* Acceleration in Z axis in [g] */
  float Press;          /* Atmospheric pressure in [hPa] */
} MVC_input_t;

typedef enum 
{
  MVC_UNKNOWN,          /* Unknown - In case of no pressure sensor data or reliable data */
  MVC_FLOOR,            /* OnFloor - When walking on flat surface */
  MVC_UPDOWN,           /* UpDown - If significant change observed in height but not sufficinet to declare as stairs or elevator */
  MVC_STAIRS,           /* Stairs - Detection of stairs */
  MVC_ELEVATOR,         /* Elevator */
  MVC_ESCALATOR         /* Escalator - Detect Escalator provided no walking on escalator */
} MVC_context_t;

typedef enum 
{
  MVC_CONFIDENCE_UNKNOWN,
  MVC_CONFIDENCE_POOR,
  MVC_CONFIDENCE_MED,
  MVC_CONFIDENCE_HIGH
} MVC_confidence_t;

typedef struct
{
  int16_t Speed;        /* Vertical speed in [cm/s] */
  uint16_t Speed_Error; /* Vertical speed error in [cm/s] */
} MVC_speed_t;

typedef struct
{
  uint32_t Timestamp;           /* Timestamp */
  uint8_t Valid;                /* Indicates if results is valid or not */
  int32_t Baro_Altitude;        /* Computed altitude with standard formula in [cm]*/
  int32_t Cal_Altitude;         /* Calibrated altitude with drift correction in [cm] */
  MVC_speed_t Speed;            /* Vertical speed */
  MVC_context_t Context;        /* Vertical context */
  MVC_confidence_t Confidence;  /* Condfidence in context */
  uint32_t NSteps;              /* Number of steps */
} MVC_output_t;

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup MOTION_VC_Exported_Functions MOTION_VC_Exported_Functions
 * @{
 */

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Initialize the MotionVC engine
 * @param  none
 * @retval none
 */
void MotionVC_Initialize(void);

/**
 * @brief  Run vertical context algorithm
 * @param  data_in  pointer to accaleration in [g] and atmospheric pressure in [hPa]
 * @param  data_out  pointer to vertical context output structure
 * @retval none
 */
void MotionVC_Update(MVC_input_t *data_in, MVC_output_t *data_out);

/**
 * @brief  Get the library version
 * @param  version pointer to an array of 35 char
 * @retval Number of characters in the version string
 */
uint8_t MotionVC_GetLibVersion(char *version);


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

#endif /* _MOTION_VC_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
