#include "Sensor.h"
//#include "MahonyAHRS.h"
#include "Oth_Dev.h"

static IKS01A2_MOTION_SENSOR_Axes_t AccValue;
static IKS01A2_MOTION_SENSOR_Axes_t GyrValue;
static IKS01A2_MOTION_SENSOR_Axes_t MagValue;
static IKS01A2_MOTION_SENSOR_Axes_t MagOffset;

extern TM_AHRSIMU_t IMU0;
extern ATT Attitude;	
extern UART_HandleTypeDef huart1;
extern uint8_t transfer_compl;
 MFX_output_t data_out;
 MFX_output_t data_out0;
 MFX_output_t data_out2;
uint8_t send_count = 0;
TM_AHRSIMU_t IMU;
static volatile uint8_t MagCalRequest = 0;
HAL_StatusTypeDef status;
extern I2C_HandleTypeDef hi2c1;
static uint32_t MagTimeStamp = 0;
static uint8_t MagCalStatus = 0;
float Ax, Ay, Az, Gx, Gy, Gz;
static volatile uint8_t SensorReadRequest = 0;

char lib_version[35];
int lib_version_len;
	
 void Init_Sensors(void)
{
  (void)IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO);
//  (void)IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);

//  (void)IKS01A2_ENV_SENSOR_Init(IKS01A2_HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY);
//  (void)IKS01A2_ENV_SENSOR_Init(IKS01A2_LPS22HB_0, ENV_PRESSURE);

}

	void lib_init(TIM_HandleTypeDef* AlgoTimHandle)
	{
		/* Sensor Fusion API initialization function */
  MotionFX_manager_init();

  /* OPTIONAL */
  /* Get library version */
  MotionFX_manager_get_version(lib_version, &lib_version_len);
		 /* Enable magnetometer calibration */
  MotionFX_manager_MagCal_start(ALGO_PERIOD);
//		TM_AHRSIMU_Init(&IMU, 200, 0.1, 0);
	/*Start timer */
	HAL_TIM_Base_Start_IT(AlgoTimHandle);
	/*Start fusion calibration */
	 MotionFX_manager_start_6X();
		
		}
	
void Lib_Process(void)
{
	uint8_t arr[200];
	uint8_t data0_sz;
	 for (;;)
  {
		 if (SensorReadRequest == 1U)
    {
      SensorReadRequest = 0;
//			BSP_LED_On(LED2);
			Accelero_Sensor_Handler(IKS01A2_LSM6DSL_0);
      Gyro_Sensor_Handler(IKS01A2_LSM6DSL_0);
//      Magneto_Sensor_Handler(IKS01A2_LSM303AGR_MAG_0);
			 /* Sensor Fusion specific part */
      FX_Data_Handler();
			calculate_attitude();
//			IMU_proc();
//				TM_AHRSIMU_UpdateIMU(&IMU, Gx, Gy, Gz, Ax, Ay, Az);
//			 BSP_LED_Off(LED2);
		}
		
//		if(transfer_compl)
//		{
//			transfer_compl = 0;
//			if(read_buf())
//			{
//				calculate_attitude();
//				if(++send_count >= 10)
//				{
//					sprintf((char*)arr, "R: %8.3f", Attitude.RollDeg);	
//		data0_sz = sprintf((char*)arr, "R: %d  R: %d P: %d P: %d Y: %d Y: %d \r\n", 
//			(int16_t)data_out.rotation_6X[2], (int16_t)Attitude.RollDeg, (int16_t)data_out.rotation_6X[1], (int16_t)Attitude.PitchDeg, 
//		(int16_t)data_out.rotation_6X[0],(int16_t)Attitude.YawDeg );
		
//			data0_sz = sprintf((char*)arr, "R: %d  R: %d P: %d P: %d Y: %d Y: %d \r\n", 
//			(int16_t)IMU0.Roll, (int16_t)Attitude.RollDeg, (int16_t)IMU0.Pitch, (int16_t)Attitude.PitchDeg, 
//		(int16_t)IMU0.Yaw,(int16_t)Attitude.YawDeg );
			
//			data0_sz = sprintf((char*)arr, "q0: %8.3f  q0: %8.3f q1: %8.3f q1: %8.3f q2: %8.3f q2: %8.3f, q3: %8.3f, q3: %8.3f \r\n", 
//			data_out0.quaternion_6X[0], Attitude.q0, data_out0.quaternion_6X[1], Attitude.q1, 
//		data_out0.quaternion_6X[2], Attitude.q2, data_out0.quaternion_6X[3], Attitude.q3 );	
		//if(data0_sz)
//		HAL_UART_Transmit(&huart1, arr, data0_sz,10);

//					send_count = 0;
//	}
//		}
//}	
	}
}

void Lib_ReadRequest()
{
	SensorReadRequest = 1;
}

void IMU_proc(void)
{
	Ax = (float)AccValue.x * FROM_MG_TO_G;
        Ay = (float)AccValue.y * FROM_MG_TO_G;
        Az = (float)AccValue.z * FROM_MG_TO_G;
	
	Gx = AHRSIMU_DEG2RAD((float)GyrValue.x * FROM_MDPS_TO_DPS);        
				Gy	= AHRSIMU_DEG2RAD((float)GyrValue.y * FROM_MDPS_TO_DPS);
        Gz = AHRSIMU_DEG2RAD((float)GyrValue.z * FROM_MDPS_TO_DPS);
}

 void FX_Data_Handler()
{
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
  MFX_input_t data_in;
  MFX_input_t *pdata_in = &data_in;
 
  MFX_output_t *pdata_out = &data_out;
#elif (defined (USE_STM32L0XX_NUCLEO))
  MFX_CM0P_input_t data_in;
  MFX_CM0P_input_t *pdata_in = &data_in;
  MFX_CM0P_output_t data_out;
  MFX_CM0P_output_t *pdata_out = &data_out;
#else
#error Not supported platform
#endif

//				data_in.gyro[0] = 0.0;
//        data_in.gyro[1] = 0.0;
//        data_in.gyro[2] = 0.0;
	
        data_in.gyro[0] = (float)GyrValue.x * FROM_MDPS_TO_DPS;
        data_in.gyro[1] = (float)GyrValue.y * FROM_MDPS_TO_DPS;
        data_in.gyro[2] = (float)GyrValue.z * FROM_MDPS_TO_DPS;

        data_in.acc[0] = (float)AccValue.x * FROM_MG_TO_G;
        data_in.acc[1] = (float)AccValue.y * FROM_MG_TO_G;
        data_in.acc[2] = (float)AccValue.z * FROM_MG_TO_G;

//				data_in.mag[0] = (float)MagValue.x * FROM_MGAUSS_TO_UT50;
//				data_in.mag[1] = (float)MagValue.y * FROM_MGAUSS_TO_UT50;
//				data_in.mag[2] = (float)MagValue.z * FROM_MGAUSS_TO_UT50;

        /* Run Sensor Fusion algorithm */
        BSP_LED_On(LED2);
        MotionFX_manager_run(pdata_in, pdata_out, MOTIONFX_ENGINE_DELTATIME);
        BSP_LED_Off(LED2);


}

/**
 * @brief  Handles the ACC axes data getting/sending
 * @param  Msg the ACC part of the stream
 * @param  Instance the device instance
 * @retval None
 */
 void Accelero_Sensor_Handler(uint32_t Instance)
{
    (void)IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &AccValue);

}

/**
 * @brief  Handles the GYR axes data getting/sending
 * @param  Msg the GYR part of the stream
 * @param  Instance the device instance
 * @retval None
 */
 void Gyro_Sensor_Handler(uint32_t Instance)
{
    (void)IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &GyrValue);

}

/**
 * @brief  Handles the MAG axes data getting/sending
 * @param  Msg the MAG part of the stream
 * @param  Instance the device instance
 * @retval None
 */
 void Magneto_Sensor_Handler(uint32_t Instance)
{
  float ans_float;
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
  MFX_MagCal_input_t mag_data_in;
  MFX_MagCal_output_t mag_data_out;
#elif (defined (USE_STM32L0XX_NUCLEO))
  MFX_CM0P_MagCal_input_t mag_data_in;
  MFX_CM0P_MagCal_output_t mag_data_out;
#else
#error Not supported platform
#endif

    (void)IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_MAGNETO, &MagValue);


    if (MagCalStatus == 0U)
    {
      mag_data_in.mag[0] = (float)MagValue.x * FROM_MGAUSS_TO_UT50;
      mag_data_in.mag[1] = (float)MagValue.y * FROM_MGAUSS_TO_UT50;
      mag_data_in.mag[2] = (float)MagValue.z * FROM_MGAUSS_TO_UT50;

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
      mag_data_in.time_stamp = (int)MagTimeStamp;
      MagTimeStamp += (uint32_t)ALGO_PERIOD;
#endif

      MotionFX_manager_MagCal_run(&mag_data_in, &mag_data_out);

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
      if (mag_data_out.cal_quality == MFX_MAGCALGOOD)
#elif (defined (USE_STM32L0XX_NUCLEO))
      if (mag_data_out.cal_quality == MFX_CM0P_MAGCALGOOD)
#else
#error Not supported platform
#endif
      {
        MagCalStatus = 1;

        ans_float = (mag_data_out.hi_bias[0] * FROM_UT50_TO_MGAUSS);
        MagOffset.x = (int32_t)ans_float;
        ans_float = (mag_data_out.hi_bias[1] * FROM_UT50_TO_MGAUSS);
        MagOffset.y = (int32_t)ans_float;
        ans_float = (mag_data_out.hi_bias[2] * FROM_UT50_TO_MGAUSS);
        MagOffset.z = (int32_t)ans_float;

        /* Disable magnetometer calibration */
        MotionFX_manager_MagCal_stop(ALGO_PERIOD);
      }
    }

    MagValue.x = (int32_t)(MagValue.x - MagOffset.x);
    MagValue.y = (int32_t)(MagValue.y - MagOffset.y);
    MagValue.z = (int32_t)(MagValue.z - MagOffset.z);


}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    Lib_ReadRequest();
  }
}

