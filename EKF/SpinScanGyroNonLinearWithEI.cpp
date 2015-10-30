#include "algorithms.h"
#include <math.h>
#include <fstream>

#define GYRO_SMOOTH 11
#define GYRO_WINDOW 125
#define GYRO_DELAY 49
extern uint32_t global_counter;

extern std::ofstream f1, f2, f3, f4, f5, f6, f6_1, f6_2, X, Y, f8, fte;
extern uint8_t  cadence_acc;
extern int cadence_gyro;
extern int GyroQuadS;
extern int GyroQuadE;
extern int GyroHamS;
extern int GyroHamE;
extern int fuckyou;
/*FTE stuff*/
extern float EIRMSBuf[750];
extern int EIsize;
extern int EIseg;

void SpinScanGyro(uint8_t* rawData, int init)
{
	RAW_DATA raw_z;
	static int sz;
	static int sz_prev = 0;
	static float smoothBuff[GYRO_SMOOTH];																				// The buffer to hold quad RMS values																		// The buffer to hold ham RMS values
	static float sum_z = 0;																								// Variables to hold the sum of the values inside the sliding window
	static int ptr1 = 0;																										// Pointer to the place inside the buffer to add the new data
	static int GyroBuff[GYRO_WINDOW];
	static int ptr2 = 0;
	static short delayBuff[GYRO_DELAY];
	static int ptr3 = 0;
	int ptr4;
	/////////////////////////////
	static int length = 0;
	static int current_length = 0;
	static int current_length_copy = 0;
	static int current_length_copy2;
	static int quadStartLength;
	static int quadEndLength;
	static int hamStartLength;
	static int hamEndLength;
	static int averageQ_s[3];
	static int averageQ_e[3];
	static int averageH_s[3];
	static int averageH_e[3];
	static int RepCount_Q = 0;
	static int RepCount_H = 0;
	static int zerocross = 0;
	static int downsampler = 0;
	static int quad_detected_delayed = 0;
	static int ham_detected_delayed = 0;
	int SegmentGyro = 0;
	static int state = 0;
	float angle;
	float max, min;
	int i;
	float g;
	static float gyroAngle = 0;
	static float maxAngle = 0, minAngle = 0;
	static float currentRange = 1;
	static int half_point = 0;
	int startAngle, endAngle;
	int EI;
	int EIstart, EIend, fullcircle, RMSBuffstart, RMSBuffend, EIsum;
	float anglestep, f;
	if (init == 0 && downsampler == 0)
	{
		raw_z.raw_data = 0;

		raw_z.data_byte[0] = rawData[4];	
		raw_z.data_byte[1] = rawData[5];
		if ((rawData[5] & 0x80) == 0x00)																							// Input data is positive, set the sign to 0
		{
			raw_z.data_byte[2] = 0;
			raw_z.data_byte[3] = 0;
		}
		else																																					// Input data is negative, set the sign to 1
		{
			raw_z.data_byte[2] = 0xFF;
			raw_z.data_byte[3] = 0xFF;
		}
		////////////////smoothing and segmentation/////////////////////
		sum_z = sum_z - smoothBuff[ptr1] + raw_z.raw_data;																			// Oldest value is subtracted from the sum and new value is added
		smoothBuff[ptr1] = raw_z.raw_data;																										// New value replaes the oldest value in the buffer																																
		sz = (float)(sum_z)/GYRO_SMOOTH;																	// Outout is the average of the window
		(ptr1)++;																															// Increment the pointers to implement the sliding effect and set them to zero if they reach the end of the buffer
		if ( ptr1 == GYRO_SMOOTH )
			ptr1 = 0;				
		delayBuff[ptr3] = sz;
		ptr4 = ptr3 + 1;
		if (ptr4 == GYRO_DELAY)
			ptr4 = 0;
		ptr3++;
		if (ptr3 == GYRO_DELAY)
			ptr3 = 0;
		GyroBuff[ptr2] = delayBuff[ptr4];
		ptr2++;
		if (ptr2 == GYRO_WINDOW)
			ptr2 = 0;
		max = GyroBuff[0];
		min = GyroBuff[0];
		for (i = 0; i < GYRO_WINDOW; i++)
		{
			if (max < GyroBuff[i])
				max = GyroBuff[i];
			if (min > GyroBuff[i])
				min = GyroBuff[i];
		}
		//X <<  delayBuff[ptr4] << std::endl;
		SegmentGyro = 0;
		if (delayBuff[ptr4] > min + (max - min)/5)
			SegmentGyro = 1;
		//Y <<  SegmentGyro << std::endl;
		////////////////////////////////////////////////////////////////
		{
			if (GyroQuadS == 1)
			{
				if (half_point == 0)
					quadStartLength = gyroAngle;
				else
					quadStartLength = -gyroAngle;
			}
			if (GyroQuadE == 1)
			{
				if (half_point == 0)
					quadEndLength = gyroAngle;
				else
					quadEndLength = -gyroAngle;
				quad_detected_delayed = 1;
			}
			if (GyroHamS == 1)
			{
				if (half_point == 0)
					hamStartLength = gyroAngle;
				else
					hamStartLength = -gyroAngle;
			}
			if (GyroHamE == 1)
			{
				if (half_point == 0)
					hamEndLength = gyroAngle;
				else
					hamEndLength = -gyroAngle;
				ham_detected_delayed = 1;
			}
			/*if (sz_prev < 0 && DelayBuff[ptr4] > 0)
			{
				current_length = length;
				length = 0;
			}*/
			if (sz_prev > 0 && delayBuff[ptr4] < 0)
			{
				half_point = 1;
			}
			switch(state)
			{
				case 0:
					if (SegmentGyro == 1)
					{
						state = 1;
						//current_length_copy = current_length;
						zerocross = 0;
					}
					break;
				case 1:
					if (sz_prev < 0 && delayBuff[ptr4] > 0 && zerocross == 0)
					{
						zerocross = 1;
						half_point = 0;
						current_length = length;
						currentRange = maxAngle - minAngle;
						length = 0;
						maxAngle = 0;
						minAngle = 0;
						gyroAngle = 0;
					}
					if (SegmentGyro == 0)
					{
						state = 0;
						if (cadence_gyro == 0)
							fuckyou = 0;
						f5 << fuckyou << std::endl;
					}
					break;
			}
			
		  if (quad_detected_delayed == 1)
		  {
			if (length != 0)
			{
				angle  = (float)quadStartLength*(180/currentRange);
				if (angle >= 360)
					angle = 0;
				if (angle < 0)
					angle += 360;
			}
			else
				angle = 0;
			startAngle = angle; //for EI
			if (RepCount_Q > 2)
				{
					averageQ_s[0] = averageQ_s[1];
					averageQ_s[1] = averageQ_s[2];
					averageQ_s[2] = angle;
				}
				if (RepCount_Q > 4)
				{
					if (abs(averageQ_s[2] - averageQ_s[1]) >= 180)
						 averageQ_s[2] -= 360;
					angle = (averageQ_s[1] +  averageQ_s[2])/2;
					if (angle < 0)
						 angle += 360;
					if (abs(angle - averageQ_s[0]) >= 180)
						angle -= 360;
					angle = (averageQ_s[0] +  angle)/2;
					if (angle < 0)
						angle += 360;
				}
			if (RepCount_Q > 2)
				f6 << angle << "\t\t";
			if (length != 0)
			{
				angle  = (float)quadEndLength*(180/currentRange);
				if (angle >= 360)
					angle = 0;
				if (angle < 0)
					angle += 360;
			}
			else
				angle = 0;
			endAngle = angle;	//for EI

			if (RepCount_Q > 2)
			{
				averageQ_e[0] = averageQ_e[1];
				averageQ_e[1] = averageQ_e[2];
				averageQ_e[2] = angle;
			}
			if (RepCount_Q > 4)
			{
				
				if (abs(averageQ_e[0] - averageQ_e[1]) >= 180)
						averageQ_e[1] -= 360;
				angle = (averageQ_e[0] +  averageQ_e[1])/2;
				if (angle < 0)
						angle += 360;
				if (abs(angle - averageQ_e[2]) >= 180)
					angle -= 360;
				angle = (averageQ_e[2] +  angle)/2;
				if (angle < 0)
					angle += 360;
			 }
			 RepCount_Q++;
			 if (RepCount_Q > 2)
				f6 << angle << std::endl;
			 quad_detected_delayed = 0;

			 /*EI*/
			 /*EIstart = 100;
			 EIend = 160;
			 fullcircle = 0;
			 if (endAngle < startAngle)
			 {
				 endAngle += 360;
				 EIstart += 360;
				 EIend += 360;
				 fullcircle = 1;
			 }
			 if (fullcircle == 1)
			 {
				 EIsum = 0;
				 if (endAngle <= EIstart)
					 EI = 0;
				 else
				 {
					 RMSBuffstart = (EIstart - startAngle)/(endAngle - startAngle)*FTEsize;
					 if (endAngle <= EIend)
						 RMSBuffstart = FTEsize - 1;
					 else
						 RMSBuffend = (EIend - startAngle)/(endAngle - startAngle)*EIsize;
					 if (RMSBuffend > EIsize - 1)
						 RMSBuffend = EIsize - 1;
					 for (i = RMSBuffstart; i < RMSBuffend; i++)
						 EIsum += EIRMSBuf[i];
					 EI = (float)EIsum/EIseg*100;
				 }
			 }
			 else
			 {
				 if ( (startAngle >= 160 && startAngle <= 360) && (endAngle >= 160 && endAngle <= 360) )
				 {
					 EI = 0;
				 }
				 else
				 {
					 RMSBuffstart = (EIstart - startAngle)/(endAngle - startAngle)*EIsize;
					 if (endAngle <= EIend)
						 RMSBuffstart = EIsize - 1;
					 else
						 RMSBuffend = (EIend - startAngle)/(endAngle - startAngle)*EIsize;
					 if (RMSBuffend > EIsize - 1)
						 RMSBuffend = EIsize - 1;
					 for (i = RMSBuffstart; i < RMSBuffend; i++)
						 EIsum += EIRMSBuf[i];
					 EI = (float)EIsum/EIseg*100;
				 }
			 }*/
			 //much easier method
			 EIsum = 0;
			 if (endAngle < startAngle)
				endAngle += 360;
			 anglestep = (endAngle - startAngle)/(float)EIsize;
			 f = startAngle;
			 for (i = 0; i < EIsize; i++)
			 {
				 if (f > 100 && f < 160)
					 EIsum += EIRMSBuf[i];
				 f += anglestep;
				 if (f >= 360)
					 f -= 360;
			 }
			 EI = (float)EIsum/EIseg*100;
			 fte << EI << std::endl;
			 /*END of EI*/
		  }
		  if (ham_detected_delayed == 1)
		  {
			if (length != 0)
			{
				angle  = (float)hamStartLength*(180/currentRange);
				if (angle >= 360)
					angle = 0;
				if (angle < 0)
					angle += 360;
			}
			else
				angle = 0;
			if (RepCount_H > 2)
			{
				averageH_s[0] = averageH_s[1];
				averageH_s[1] = averageH_s[2];
				averageH_s[2] = angle;
			}
			if (RepCount_H > 4)
			{
				if (abs(averageH_s[2] - averageH_s[1]) >= 180)
						averageH_s[2] -= 360;
				angle = (averageH_s[1] +  averageH_s[2])/2;
				if (angle < 0)
						angle += 360;
				if (abs(angle - averageH_s[0]) >= 180)
					angle -= 360;
				angle = (averageH_s[0] +  angle)/2;
				if (angle < 0)
					angle += 360;
			}
			if (RepCount_H > 2)
				f6_2 << angle << "\t\t";
			if (length != 0)
			{
				angle  = (float)hamEndLength*(180/currentRange);
				if (angle >= 360)
					angle = 0;
				if (angle < 0)
					angle += 360;
			}
			else
				angle = 0;
			if (RepCount_H > 2)
			{
				averageH_e[0] = averageH_e[1];
				averageH_e[1] = averageH_e[2];
				averageH_e[2] = angle;
			}
			if (RepCount_H > 4)
			{
				if (abs(averageH_e[2] - averageH_e[1]) >= 180)
						averageH_e[2] -= 360;
				angle = (averageH_e[1] +  averageH_e[2])/2;
				if (angle < 0)
						angle += 360;
				if (abs(angle - averageH_e[0]) >= 180)
					angle -= 360;
				angle = (averageH_e[0] +  angle)/2;
				if (angle < 0)
					angle += 360;
			}
			RepCount_H++;
			if (RepCount_H > 2)
				f6_2 << angle << std::endl;
			 ham_detected_delayed = 0;
		  }
		/*if (cadence_acc == 0)
		  {
				averageH_e[0] = 0; averageH_e[1] = 0; averageH_e[2] = 0;
				averageH_s[0] = 0; averageH_s[1] = 0; averageH_s[2] = 0;
				averageQ_e[0] = 0; averageQ_e[1] = 0; averageQ_e[2] = 0;
				averageQ_s[0] = 0; averageQ_s[1] = 0; averageQ_s[2] = 0;
				RepCount_Q = 0;
				RepCount_H = 0;
		  }*/
		  g = (float)(delayBuff[ptr4]-sz_prev)*(delayBuff[ptr4]-sz_prev) + (float)(6.4e-5);
		  //gyroAngle = (float)0.98*((float)delayBuff[ptr4]/32.8*(float)0.008 + gyroAngle);
		  gyroAngle += ((float)delayBuff[ptr4]/32.8*(float)0.008);
		  //X <<  delayBuff[ptr4]<< std::endl;
		  //Y << delayBuff[ptr4] << std::endl;
		  if (gyroAngle > maxAngle)
			  maxAngle = gyroAngle;
		  if (gyroAngle < minAngle)
			  minAngle = gyroAngle;
		  length += sqrt(g);
		  sz_prev = delayBuff[ptr4];
		}
		downsampler++;
		if (downsampler == 4)
			downsampler = 0;
	}
	else if (init == 0 && downsampler != 0)
	{
		downsampler++;
		if (downsampler == 4)
			downsampler = 0;
		if (GyroQuadS == 1)
		{
			if (half_point == 0)
				quadStartLength = gyroAngle;
			else
				quadStartLength = -gyroAngle;
		}
		if (GyroQuadE == 1)
		{
			if (half_point == 0)
				quadEndLength = gyroAngle;
			else
				quadEndLength = -gyroAngle;
			quad_detected_delayed = 1;
		}
		if (GyroHamS == 1)
		{
			if (half_point == 0)
				hamStartLength = gyroAngle;
			else
				hamStartLength = -gyroAngle;
		}
		if (GyroHamE == 1)
		{
			if (half_point == 0)
				hamEndLength = gyroAngle;
			else
				hamEndLength = -gyroAngle;
			ham_detected_delayed = 1;
		}
	}
	else
	{
		state = 0;
		downsampler = 0;
		current_length = 0;
		current_length_copy = 0;
	}
}
