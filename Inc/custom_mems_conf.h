/**
 ******************************************************************************
 * @file    custom_mems_conf.h
 * @author  MEMS Application Team
 * @version V6.0.0
 * @date    16-January-2019
 * @brief   This file contains definitions of the MEMS components bus interfaces for custom boards 
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
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
#ifndef __CUSTOM_MEMS_CONF_H__
#define __CUSTOM_MEMS_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo_bus.h"
#include "stm32f4xx_nucleo_errno.h"

/* USER CODE BEGIN 1 */
#define USE_CUSTOM_MOTION_SENSOR_LSM6DSL_0        1U

#define USE_CUSTOM_MOTION_SENSOR_LSM303AGR_ACC_0  1U
#define USE_CUSTOM_MOTION_SENSOR_LSM303AGR_MAG_0  1U

/* USER CODE END 1 */

#define CUSTOM_LSM6DSL_0_I2C_Init BSP_I2C1_Init
#define CUSTOM_LSM6DSL_0_I2C_DeInit BSP_I2C1_DeInit
#define CUSTOM_LSM6DSL_0_I2C_ReadReg BSP_I2C1_ReadReg
#define CUSTOM_LSM6DSL_0_I2C_WriteReg BSP_I2C1_WriteReg

#define CUSTOM_LSM303AGR_0_I2C_Init BSP_I2C1_Init
#define CUSTOM_LSM303AGR_0_I2C_DeInit BSP_I2C1_DeInit
#define CUSTOM_LSM303AGR_0_I2C_ReadReg BSP_I2C1_ReadReg
#define CUSTOM_LSM303AGR_0_I2C_WriteReg BSP_I2C1_WriteReg

#define CUSTOM_HTS221_0_I2C_Init BSP_I2C1_Init
#define CUSTOM_HTS221_0_I2C_DeInit BSP_I2C1_DeInit
#define CUSTOM_HTS221_0_I2C_ReadReg BSP_I2C1_ReadReg
#define CUSTOM_HTS221_0_I2C_WriteReg BSP_I2C1_WriteReg

#define CUSTOM_LPS22HB_0_I2C_Init BSP_I2C1_Init
#define CUSTOM_LPS22HB_0_I2C_DeInit BSP_I2C1_DeInit
#define CUSTOM_LPS22HB_0_I2C_ReadReg BSP_I2C1_ReadReg
#define CUSTOM_LPS22HB_0_I2C_WriteReg BSP_I2C1_WriteReg

#ifdef __cplusplus
}
#endif

#endif /* __CUSTOM_MEMS_CONF_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
