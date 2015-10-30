#include "algorithms.h"
#include <math.h>
#include <fstream>

#define GYRO_SMOOTH 11
#define GYRO_WINDOW 125
#define GYRO_DELAY 54
extern uint32_t global_counter;

extern std::ofstream f1, f2, f3, f4, f5, f6, f6_1, f6_2, X, Y, f8, ei;
extern uint8_t  cadence_acc;
extern int cadence_gyro;
extern int GyroQuadS;
extern int GyroQuadE;
extern int GyroHamS;
extern int GyroHamE;
extern float fuckyou;
extern unsigned char EIRMSBuf[750];
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
	static int quad_start_delayed = 0;
	static int quad_end_delayed = 0;
	static int ham_start_delayed = 0;
	static int ham_end_delayed = 0;
	static int ham_detected_delayed = 0;
	int SegmentGyro = 0;
	static int state = 0;
	float angle;
	float max, min;
	int i;
	float g;
	static int half_cycle = 0;
	static int QuadStart;
	static int HamStart;
	static float fml = 0;
	static int k1, k11, k2, k22, k3, k4;
	static int EI;
	int startAngle = 0;
	int endAngle = 0; 
	int EIstart, EIend, fullcircle, RMSBuffstart, RMSBuffend, EIsum;
	float f, anglestep;
	static int anotherTemp1, anotherTemp2;
	static int save = 0;
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
		delayBuff[ptr3] = raw_z.raw_data;//sz;
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
		X <<  delayBuff[ptr4] << std::endl;
		SegmentGyro = 0;
		if (delayBuff[ptr4] > min + (max - min)/5)
			SegmentGyro = 1;
		Y <<  SegmentGyro << std::endl;
		////////////////////////////////////////////////////////////////
		{
			if (GyroQuadS == 1)
			{
				quadStartLength = length;
				quad_start_delayed = 1;
			}
			if (GyroQuadE == 1)
			{
				quadEndLength = length;
				quad_detected_delayed = 1;
				quad_end_delayed = 1;
				fml = fuckyou;
			}
			if (GyroHamS == 1)
			{
				hamStartLength = length;
				ham_start_delayed = 1;
			}
			if (GyroHamE == 1)
			{
				hamEndLength = length;
				ham_detected_delayed = 1;
				ham_end_delayed = 1;
			}

			switch(state)
			{
				case 0:
					if (sz_prev < 0 && delayBuff[ptr4] > 0)
					{
						state = 1;
						current_length = length;
						anotherTemp2 = length*4;
						length = 0;
						half_cycle = 2;
						//if (cadence_gyro == 0)
						//	fml = 0;
						save = 1;
					}
					break;
				case 1:
					if (sz_prev > 0 && delayBuff[ptr4] < 0)
					{
						state = 0;
						current_length = length;
						anotherTemp1 = length*4;
						length = 0;
						half_cycle = 1;
					}
					break;
			}
			
			if (quad_start_delayed == 1 && half_cycle != 0)
			{
				angle  = (float)quadStartLength/current_length*180;
				if (half_cycle == 2)
					angle += 180;
				if (angle >= 360)
					angle = 0;
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
				{
					QuadStart = angle;
					k1 = angle;
				}
				k11 = k1;
				quad_start_delayed = 0;
			}
			if (quad_end_delayed == 1 && half_cycle != 0)
			{
				angle  = (float)quadEndLength/current_length*180;
				if (half_cycle == 2)
					angle += 180;
				if (angle >= 360)
					angle = 0;
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
				{
					//f6 << QuadStart << "\t\t" << angle << std::endl;
					k2 = angle;
				}
				k22 = k2;
				quad_detected_delayed = 0;
				quad_end_delayed = 0;

				//EI//
				endAngle = k2;
				startAngle = k1;
				EIsum = 0;
				if (endAngle < startAngle)
					endAngle += 360;
				anglestep = (endAngle - startAngle)/(float)EIsize;
				f = startAngle;
				//if (f >= 180 && f <= 360)
				//	anglestep = (float)180/anotherTemp2;
				//else
				//	anglestep = (float)180/anotherTemp1;
				for (i = 0; i < EIsize; i++)
				{
					if (f > 100 && f < 160)
						EIsum += EIRMSBuf[i];
					f += anglestep;
					if (f >= 360)
						f -= 360;
					//if (f >= 180 && f <= 360)
					//	anglestep = (float)180/anotherTemp2;
					//else
					//	anglestep = (float)180/anotherTemp1;
				}
				EI = (float)EIsum/EIseg*100;
			}
			if (ham_start_delayed == 1 && half_cycle != 0)
			{
				angle  = (float)hamStartLength/current_length*180;
				if (half_cycle == 2)
					angle += 180;
				if (angle >= 360)
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
					HamStart = angle;
				ham_start_delayed = 0;
			}
			if (ham_end_delayed == 1 && half_cycle != 0)
			{
				
				angle  = (float)hamEndLength/current_length*180;
				if (half_cycle == 2)
					angle += 180;
				if (angle >= 360)
					angle = 0;
			
				if (RepCount_H > 2)
				{
					averageH_e[0] = averageH_e[1];
					averageH_e[1] = averageH_e[2];
					averageH_e[2] = angle;
				}
				if (RepCount_H > 4)
				{
				
					if (abs(averageH_e[0] - averageH_e[1]) >= 180)
							averageH_e[1] -= 360;
					angle = (averageH_e[0] +  averageH_e[1])/2;
					if (angle < 0)
							angle += 360;
					if (abs(angle - averageH_e[2]) >= 180)
						angle -= 360;
					angle = (averageH_e[2] +  angle)/2;
					if (angle < 0)
						angle += 360;
					}
					RepCount_H++;
					if (RepCount_H > 2)
						f6_2 << HamStart << "\t\t" << angle << std::endl;
					ham_detected_delayed = 0;
					ham_end_delayed = 0;
			}
			if (half_cycle != 0)
				half_cycle = 0;
			if (cadence_gyro == 0)
			{
				averageH_e[0] = 0; averageH_e[1] = 0; averageH_e[2] = 0;
				averageH_s[0] = 0; averageH_s[1] = 0; averageH_s[2] = 0;
				averageQ_e[0] = 0; averageQ_e[1] = 0; averageQ_e[2] = 0;
				averageQ_s[0] = 0; averageQ_s[1] = 0; averageQ_s[2] = 0;
				RepCount_Q = 0;
				RepCount_H = 0;
				k1 = 0;
				k2 = 0;
			}
			g = (float)(delayBuff[ptr4]-sz_prev)*(delayBuff[ptr4]-sz_prev) + (float)(6.4e-5);
			length++;// += sqrt(g);
			sz_prev = delayBuff[ptr4];
			if (save == 1)
			{
				save = 0;
				f4 << cadence_gyro << std::endl;
				f5 << fml << std::endl;
				f6 << k11 << "\t" << k22 << std::endl;
				f6_2 << k3 << "\t" << k4 << std::endl;
				ei << EI << std::endl;
			}
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
			quadStartLength = length;
			quad_start_delayed = 1;
		}
		if (GyroQuadE == 1)
		{
			quadEndLength = length;
			quad_detected_delayed = 1;
			quad_end_delayed = 1;
			fml = fuckyou;
		}
		if (GyroHamS == 1)
		{
			hamStartLength = length;
			ham_start_delayed = 1;
		}
		if (GyroHamE == 1)
		{
			hamEndLength = length;
			ham_detected_delayed = 1;
			ham_end_delayed = 1;
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