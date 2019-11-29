#include "Oth_Dev.h"
#include "string.h"

ATT Attitude;	
TM_AHRSIMU_t IMU0;
uint8_t buffer[BUFFER_SIZE];	/**< fifo buffer for received data. */
volatile uint8_t tail;					/**< index for writing to buffer. */
volatile uint8_t head;					/**< index for reading from buffer. */
uint8_t transfer_compl;
extern MFX_output_t data_out;
extern MFX_output_t data_out0;
extern MFX_output_t data_out2;
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

float normalize(const TM_AHRSIMU_t* IMU0)
{
	return oneOverSqrt(pow(IMU0->_q0, 2) + pow(IMU0->_q1, 2) + pow(IMU0->_q2, 2) + pow(IMU0->_q3, 2));
}

static 
	void Quaternion_Addition(MFX_output_t* ourDev, MFX_output_t* othDev)
	{
		float pow_norma = pow(sqrt(pow(othDev->quaternion_6X[0], 2) + pow(othDev->quaternion_6X[1], 2) + 
			pow(othDev->quaternion_6X[2], 2) + pow(othDev->quaternion_6X[3], 2)), 2);
		
		othDev->quaternion_6X[0] = -(othDev->quaternion_6X[0]) / pow_norma;
		othDev->quaternion_6X[1] = -(othDev->quaternion_6X[1]) / pow_norma;
		othDev->quaternion_6X[2] = -(othDev->quaternion_6X[2]) / pow_norma;
		othDev->quaternion_6X[2] /= pow_norma;
		
		IMU0._q0 = ourDev->quaternion_6X[0] + othDev->quaternion_6X[0]; 
		IMU0._q1 = ourDev->quaternion_6X[1] + othDev->quaternion_6X[1]; 
		IMU0._q2 = ourDev->quaternion_6X[2] + othDev->quaternion_6X[2]; 
		IMU0._q3 = ourDev->quaternion_6X[3] + othDev->quaternion_6X[3];
		
//	float norm = normalize(&IMU0);
//	IMU0._q0 *= norm;
//	IMU0._q1 *= norm;
//	IMU0._q2 *= norm;
//	IMU0._q3 *= norm;
		
	}

	void Quaternion2Angles(TM_AHRSIMU_t* IMU1)
{
	float Norm, test, xx,yy,zz;
		// q[0] = w, q[1] = vx, q[2] = vy, q[3] = vz / 
		// Assuming the angles are in radians.

	Norm = oneOverSqrt(IMU1->_q0 * IMU1->_q0+IMU1->_q1*IMU1->_q1+IMU1->_q2*IMU1->_q2+IMU1->_q3*IMU1->_q3);

	IMU1->_q0*=Norm;	
	IMU1->_q1*=Norm;
	IMU1->_q2*=Norm;
	IMU1->_q3*=Norm;
	
//	test = Att->q1*Att->q2 + Att->q3*Att->q0;
//	
//	if (test > 0.499) // singularity at north pole
//	{ 
//		Att->YawDeg = AHRSIMU_RAD2DEG(2 * atan2(Att->q1,Att->q0));
//		Att->PitchDeg = AHRSIMU_RAD2DEG(AHRSIMU_PI/2);
//		Att->RollDeg = 0;
//		return;
//	}
//	if (test < -0.499) // singularity at south pole
//	{ 
//		Att->YawDeg = -2.0f * AHRSIMU_RAD2DEG(atan2(Att->q1,Att->q0));
//		Att->PitchDeg = -1.0f * AHRSIMU_RAD2DEG(AHRSIMU_PI/2);
//		Att->RollDeg = 0;
//		return;
//	}
//  xx = Att->q1*Att->q1;
//  yy = Att->q2*Att->q2;
//  zz = Att->q3*Att->q3;
//	
//  Att->YawDeg = AHRSIMU_RAD2DEG(1) * atan2(2*Att->q3*Att->q0 - 2*Att->q1*Att->q3 , 1 - 2*yy - 2*zz);
//	Att->PitchDeg = AHRSIMU_RAD2DEG(1) * asin(2*test);
//	Att->RollDeg = AHRSIMU_RAD2DEG(1) * atan2(2*Att->q1*Att->q0-2*Att->q2*Att->q3 , 1 - 2*xx - 2*zz);	



    // roll (x-axis rotation)
   float sinr_cosp = 2 * (IMU1->_q0 * IMU1->_q1 + IMU1->_q2 * IMU1->_q3);
   float cosr_cosp = 1 - 2  *(IMU1->_q1 * IMU1->_q1 + IMU1->_q2 * IMU1->_q2);
		
		IMU1->Roll = AHRSIMU_RAD2DEG(1) * atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2  *(IMU1->_q0 * IMU1->_q2 - IMU1->_q3 * IMU1->_q1);
    
		if (sinp >= 1)
      IMU1->Pitch = 90; // use 90 degrees if out of range
    else if (sinp <= -1)
			IMU1->Pitch = -90;
		else
      IMU1->Pitch = AHRSIMU_RAD2DEG(1) * asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (IMU1->_q0 * IMU1->_q3 + IMU1->_q1 * IMU1->_q2);
    float cosy_cosp = 1 - 2 * (IMU1->_q2 * IMU1->_q2 + IMU1->_q3 * IMU1->_q3);
    
		IMU1->Yaw = AHRSIMU_RAD2DEG(1) * atan2(siny_cosp, cosy_cosp);
}	


static 
	void Quaternion_product(MFX_output_t* ourDev, ATT* othDev)
	{
		float q0, q1, q2, q3, recipNorm;
	
		
	/*Quaternion production. Q3 = AxB*/
	
	IMU0._q0 = ourDev->quaternion_6X[0]*othDev->q0 - ourDev->quaternion_6X[1]*othDev->q1 - 
						 ourDev->quaternion_6X[2]*othDev->q2 - ourDev->quaternion_6X[3]*othDev->q3;
		
	IMU0._q1 = ourDev->quaternion_6X[0]*othDev->q1 + ourDev->quaternion_6X[1]*othDev->q0 + 
						 ourDev->quaternion_6X[2]*othDev->q3 - ourDev->quaternion_6X[3]*othDev->q2;
		
	IMU0._q2 = ourDev->quaternion_6X[0]*othDev->q2 - ourDev->quaternion_6X[1]*othDev->q3 + 
						 ourDev->quaternion_6X[2]*othDev->q0 + ourDev->quaternion_6X[3]*othDev->q1;
		
	IMU0._q3 = ourDev->quaternion_6X[0]*othDev->q3 + ourDev->quaternion_6X[1]*othDev->q2 - 
						 ourDev->quaternion_6X[2]*othDev->q1 + ourDev->quaternion_6X[3]*othDev->q0;
	}
	
	static 
	void Quaternion_Product(MFX_output_t* ourDev, MFX_output_t* othDev)
	{
		float q0, q1, q2, q3, recipNorm;
	
		//////////////Sergey`s ///////////////////////////////////////////////
		
		float pow_norma = pow(sqrt(pow(ourDev->quaternion_6X[0], 2) + pow(ourDev->quaternion_6X[1], 2) + 
			pow(ourDev->quaternion_6X[2], 2) + pow(ourDev->quaternion_6X[3], 2)), 2);
		
		ourDev->quaternion_6X[0] = -(ourDev->quaternion_6X[0]); // / pow_norma;
		ourDev->quaternion_6X[1] = -(ourDev->quaternion_6X[1]); // / pow_norma;
		ourDev->quaternion_6X[2] = -(ourDev->quaternion_6X[2]); // / pow_norma;
	//	ourDev->quaternion_6X[2] /= pow_norma;
		
		
		
	/*Quaternion production. Q3 = AxB*/
	
	IMU0._q0 = ourDev->quaternion_6X[0]*othDev->quaternion_6X[3] + ourDev->quaternion_6X[1]*othDev->quaternion_6X[2] - 
						 ourDev->quaternion_6X[2]*othDev->quaternion_6X[1] + ourDev->quaternion_6X[3]*othDev->quaternion_6X[0];
		
	IMU0._q1 = -ourDev->quaternion_6X[0]*othDev->quaternion_6X[2] + ourDev->quaternion_6X[1]*othDev->quaternion_6X[3] + 
						 ourDev->quaternion_6X[2]*othDev->quaternion_6X[0] + ourDev->quaternion_6X[3]*othDev->quaternion_6X[1];
		
	IMU0._q2 = ourDev->quaternion_6X[0]*othDev->quaternion_6X[1] - ourDev->quaternion_6X[1]*othDev->quaternion_6X[0] + 
						 ourDev->quaternion_6X[2]*othDev->quaternion_6X[3] + ourDev->quaternion_6X[3]*othDev->quaternion_6X[2];
		
	IMU0._q3 = -ourDev->quaternion_6X[0]*othDev->quaternion_6X[0] - ourDev->quaternion_6X[1]*othDev->quaternion_6X[1] - 
						 ourDev->quaternion_6X[2]*othDev->quaternion_6X[2] + ourDev->quaternion_6X[3]*othDev->quaternion_6X[3];
		
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
//	IMU0._q3 = 	ourDev->quaternion_6X[3]*othDev->quaternion_6X[3] - ourDev->quaternion_6X[0]*othDev->quaternion_6X[0] - 
//							ourDev->quaternion_6X[1]*othDev->quaternion_6X[1] - ourDev->quaternion_6X[2]*othDev->quaternion_6X[2];
//		
//		IMU0._q0 = ourDev->quaternion_6X[3]*othDev->quaternion_6X[0] + ourDev->quaternion_6X[0]*othDev->quaternion_6X[3] + 
//						 ourDev->quaternion_6X[1]*othDev->quaternion_6X[2] - ourDev->quaternion_6X[2]*othDev->quaternion_6X[1];
//						 
//	IMU0._q1 = ourDev->quaternion_6X[3]*othDev->quaternion_6X[1] - ourDev->quaternion_6X[0]*othDev->quaternion_6X[2] + 
//						 ourDev->quaternion_6X[1]*othDev->quaternion_6X[3] + ourDev->quaternion_6X[2]*othDev->quaternion_6X[0];
//		
//	IMU0._q2 = ourDev->quaternion_6X[3]*othDev->quaternion_6X[2] + ourDev->quaternion_6X[0]*othDev->quaternion_6X[1] + 
//						 ourDev->quaternion_6X[1]*othDev->quaternion_6X[0] + ourDev->quaternion_6X[2]*othDev->quaternion_6X[3];					 
	
}
	
	void Euler2Quaternionrotate(MFX_output_t* ourDev) 
	{
		/* q[0] = vx, q[1] = vy, q[2] = vz, q[3] = w */ 
		// Assuming the angles are in radians.
    
		
		double c1 = cos(AHRSIMU_DEG2RAD(ourDev->rotation_6X[0])/2);
    double s1 = sin(AHRSIMU_DEG2RAD(ourDev->rotation_6X[0])/2);
    double c2 = cos(AHRSIMU_DEG2RAD(ourDev->rotation_6X[1])/2.0f);
    double s2 = sin(AHRSIMU_DEG2RAD(ourDev->rotation_6X[1])/2.0f);
    double c3 = cos(AHRSIMU_DEG2RAD(ourDev->rotation_6X[2])/2.0f);
    double s3 = sin(AHRSIMU_DEG2RAD(ourDev->rotation_6X[2])/2.0f);
    double c1c2 = c1*c2;
    double s1s2 = s1*s2;
		
   ourDev->quaternion_6X[3] = c1c2*c3 + s1s2*s3;
  	ourDev->quaternion_6X[0] = c1c2*s3 - s1s2*c3;
	  ourDev->quaternion_6X[1] = s1*c2*s3 + c1*s2*c3;	
	  ourDev->quaternion_6X[2] = s1*c2*c3 - c1*s2*s3;
  }
	
	void Euler2Quaternion_rotate(ATT* othDev) 
	{
		/* q[0] = vx, q[1] = vy, q[2] = vz, q[3] = w */ 
		// Assuming the angles are in radians.
    
		
		double c1 = cos(AHRSIMU_DEG2RAD(othDev->YawDeg)/2);
    double s1 = sin(AHRSIMU_DEG2RAD(othDev->YawDeg)/2);
    double c2 = cos(AHRSIMU_DEG2RAD(othDev->PitchDeg)/2.0f);
    double s2 = sin(AHRSIMU_DEG2RAD(othDev->PitchDeg)/2.0f);
    double c3 = cos(AHRSIMU_DEG2RAD(othDev->RollDeg)/2.0f);
    double s3 = sin(AHRSIMU_DEG2RAD(othDev->RollDeg)/2.0f);
    double c1c2 = c1*c2;
    double s1s2 = s1*s2;
		
   othDev->q3 = c1c2*c3 + s1s2*s3;
  	othDev->q0 = c1c2*s3 - s1s2*c3;
	  othDev->q1 = s1*c2*s3 + c1*s2*c3;	
	  othDev->q2 = s1*c2*c3 - c1*s2*s3;
  }
	
	void calculate_difference(MFX_output_t* ourDev, MFX_output_t* othDev)
	{
		IMU0.Yaw = ourDev->rotation_6X[0] - othDev->rotation_6X[0];
//		if(IMU0.Yaw < 0.0)
//			IMU0.Yaw = 360.0 + IMU0.Yaw;
		 if(IMU0.Yaw > 360.0)
			IMU0.Yaw -= 360.0;
		
		IMU0.Pitch = ourDev->rotation_6X[1] - othDev->rotation_6X[1];
			if(IMU0.Pitch < -180.0)
			IMU0.Pitch = 180.0 -(-180.0 - IMU0.Pitch);
		else if(IMU0.Pitch > 180.0)
			IMU0.Pitch = -180.0 +(IMU0.Pitch - 180.0);
		
		IMU0.Roll = ourDev->rotation_6X[2] - othDev->rotation_6X[2];
			if(IMU0.Roll < -180.0)
			IMU0.Roll = 180.0 -(-180.0 - IMU0.Roll);
		else if(IMU0.Roll > 180.0)
			IMU0.Roll = -180.0 +(IMU0.Roll - 180.0);
		
	}
	
	void calculate_Difference(MFX_output_t* ourDev, ATT* othDev)
	{
		IMU0.Yaw = ourDev->rotation_6X[0] - othDev->YawDeg;
//		if(IMU0.Yaw < 0.0)
//			IMU0.Yaw = 360.0 + IMU0.Yaw;
		 if(IMU0.Yaw > 360.0)
			IMU0.Yaw -= 360.0;
		
		IMU0.Pitch = ourDev->rotation_6X[1] - othDev->PitchDeg;
			if(IMU0.Pitch < -180.0)
			IMU0.Pitch = 180.0 -(-180.0 - IMU0.Pitch);
		else if(IMU0.Pitch > 180.0)
			IMU0.Pitch = -180.0 +(IMU0.Pitch - 180.0);
		
		IMU0.Roll = ourDev->rotation_6X[2] - othDev->RollDeg;
			if(IMU0.Roll < -180.0)
			IMU0.Roll = 180.0 -(-180.0 - IMU0.Roll);
		else if(IMU0.Roll > 180.0)
			IMU0.Roll = -180.0 +(IMU0.Roll - 180.0);
		
	}
	
void calculate_attitude()
{
	data_out0.rotation_6X[0] = data_out.rotation_6X[0];
	data_out0.rotation_6X[1] = data_out.rotation_6X[1];
	data_out0.rotation_6X[2] = data_out.rotation_6X[2];
	
	Euler2Quaternionrotate(&data_out0);
	
	data_out2.rotation_6X[0] = data_out0.rotation_6X[0]; // + 30.0;
//	if(data_out2.rotation_6X[0] > 360.0)
//		data_out2.rotation_6X[0] = data_out2.rotation_6X[0] - 360.0;
	
	data_out2.rotation_6X[1] = data_out0.rotation_6X[1] + 30.0;
	if(data_out2.rotation_6X[1] > 180.0)
		data_out2.rotation_6X[1] = -180.0 + data_out2.rotation_6X[1] - 180.0;
	
	data_out2.rotation_6X[2] = data_out0.rotation_6X[2]; // + 30.0;
//	if(data_out2.rotation_6X[2] > 180.0)
//		data_out2.rotation_6X[2] = -180.0 + data_out2.rotation_6X[2] - 180.0;
	
//	calculate_Difference(&data_out0, &Attitude);
	
//	calculate_difference(&data_out0, &data_out2);
	Euler2Quaternionrotate(&data_out2);
	Quaternion_Product(&data_out0, &data_out2);
	
//	Euler2Quaternion_rotate(&Attitude);
//	Quaternion_product(&data_out0, &Attitude);
	float temp = IMU0._q0;
	float temp2 = IMU0._q1;
	IMU0._q0 = IMU0._q3;
	IMU0._q1 = temp;
	temp = IMU0._q2;
	IMU0._q2 = temp2;
	IMU0._q3 = temp;
	Quaternion2Angles(&IMU0);
}


	