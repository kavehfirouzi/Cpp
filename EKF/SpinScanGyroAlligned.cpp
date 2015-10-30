#include "algorithms.h"
#include <math.h>
#include "Matrix.h"
#include <fstream>

#define GYRO_SMOOTH 11
#define GYRO_WINDOW 125
#define GYRO_DELAY 49
extern uint32_t global_counter;

extern std::ofstream f1, f2, f3, f4, f5, f6, f6_1, f6_2, X, Y, f8, allign, unalligned;
extern uint8_t  cadence_acc;
extern int cadence_gyro;
extern int GyroQuadS;
extern int GyroQuadE;
extern int GyroHamS;
extern int GyroHamE;
extern int fuckyou;

void SpinScanGyroAlligned(uint32_t* rawData, int init)
{
	RAW_DATA raw_z, raw_y, raw_x;
	static int sz;
	static int sz_prev = 0;
	static int szangle_prev = 0;
	static float smoothBuff[GYRO_SMOOTH];																				// The buffer to hold quad RMS values																		// The buffer to hold ham RMS values
	static float sum_z = 0;																								// Variables to hold the sum of the values inside the sliding window
	static int ptr1 = 0;																										// Pointer to the place inside the buffer to add the new data
	static int GyroBuff[GYRO_WINDOW];
	static int ptr2 = 0;
	static short delayBuff_z[GYRO_DELAY];
	static short delayBuff_x[GYRO_DELAY];
	static short delayBuff_y[GYRO_DELAY];
	static float delayBuffAngle_z[GYRO_DELAY];
	static float delayBuffAngle_x[GYRO_DELAY];
	static float delayBuffAngle_y[GYRO_DELAY];
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
	static int state2 = 0;
	static int stateangle = 0;
	float angle;
	float max, min;
	int i;
	float f, g;
	/*new stuff*/
	static float gyroSVDBuff[600];
	static float angleSVDBuff[600];
	float alligned[600];
	static int allignptr = 0;
	static int allignangleptr = 0;
	static int alligncounter = 0;
	static int alligncounterangle = 0;
	static int start = 0;
	int repstart, repend;
	static int alaki = 0;
	float alakibuff[200];
	static float gyroAngleZ = 0;
	static float gyroAngleY = 0;
	static float gyroAngleX = 0;
	static int mid_point = 0;
	static float eigenvecs[9];
	///
	if (init == 0 && downsampler == 0)
	{
		alaki++;
		raw_z.raw_data = rawData[2];
		raw_y.raw_data = rawData[1];
		raw_x.raw_data = rawData[0];
		////////////////smoothing and segmentation/////////////////////
		sum_z = sum_z - smoothBuff[ptr1] + raw_z.raw_data;																			// Oldest value is subtracted from the sum and new value is added
		smoothBuff[ptr1] = raw_z.raw_data;																										// New value replaes the oldest value in the buffer																																
		sz = (float)(sum_z)/GYRO_SMOOTH;																	// Outout is the average of the window
		(ptr1)++;																															// Increment the pointers to implement the sliding effect and set them to zero if they reach the end of the buffer
		if ( ptr1 == GYRO_SMOOTH )
			ptr1 = 0;
		///////////////////////////////////
		delayBuff_z[ptr3] = raw_z.raw_data;
		delayBuff_x[ptr3] = raw_x.raw_data;
		delayBuff_y[ptr3] = raw_y.raw_data;
		///////////////////////////////////
		ptr4 = ptr3 + 1;
		if (ptr4 == GYRO_DELAY)
			ptr4 = 0;
		//////////////////////////////////
		gyroAngleZ = 0.98*((float)raw_z.raw_data/32.8*(float)0.008 + gyroAngleZ);	
		gyroAngleY = 0.98*((float)raw_y.raw_data/32.8*(float)0.008 + gyroAngleY);			
		gyroAngleX = 0.98*((float)raw_x.raw_data/32.8*(float)0.008 + gyroAngleX);			
		delayBuffAngle_z[ptr3] = gyroAngleZ;
		delayBuffAngle_x[ptr3] = gyroAngleX;
		delayBuffAngle_y[ptr3] = gyroAngleY;
		//////////////////////////////////
		ptr3++;
		if (ptr3 == GYRO_DELAY)
			ptr3 = 0;
		GyroBuff[ptr2] = delayBuff_z[ptr4];
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
		//X <<  gyroAngleZ << std::endl;
		if (alaki == 2000)
			alaki = alaki;
		SegmentGyro = 0;
		if (delayBuff_z[ptr4] > min + (max - min)/5)
			SegmentGyro = 1;
		//Y <<  SegmentGyro << std::endl;
		////////////////////////////////////////////////////////////////
		{
			if (GyroQuadS == 1)
			{
				quadStartLength = length;
			}
			if (GyroQuadE == 1)
			{
				quadEndLength = length;
				quad_detected_delayed = 1;
			}
			if (GyroHamS == 1)
			{
				hamStartLength = length;
			}
			if (GyroHamE == 1)
			{
				hamEndLength = length;
				ham_detected_delayed = 1;
			}
			
			/*switch(stateangle)
			{
				case 0:
					if (szangle_prev < 0 && delayBuffAngle[ptr4] > 0)
					{
						stateangle = 1;
						allignangleptr = 0;
						stateangle = 1;
					}
					break;
				case 1:
					angleSVDBuff[allignangleptr*3] = delayBuffAngle_x[ptr4];
					angleSVDBuff[allignangleptr*3 + 1] = delayBuffAngle_y[ptr4];
					angleSVDBuff[allignangleptr*3 + 2] = delayBuffAngle[ptr4];
					allignangleptr++;
					alligncounterangle++;
					if (allignangleptr == 200)
						allignangleptr = 0;
					if (szangle_prev < 0 && delayBuffAngle[ptr4] > 0)
					{
						if (alligncounterangle < 200)
						{
							AllignGyroAcc(gyroSVDBuff, angleSVDBuff, alligned, eigenvecs, alligncounterangle);
						}
						allignangleptr = 0;
						alligncounterangle = 0;
					}
					break;
			}*/

			switch(state)
			{
				case 0:
					if (sz_prev < 0 && delayBuff_z[ptr4] > 0)
					{
						length = 0;
						state = 1;
						alligncounter = 0;
						allignptr = 0;
					}
					break;
				case 1:
					gyroSVDBuff[allignptr*3] = delayBuff_x[ptr4];
					gyroSVDBuff[allignptr*3 + 1] = delayBuff_y[ptr4];
					gyroSVDBuff[allignptr*3 + 2] = delayBuff_z[ptr4];
					angleSVDBuff[allignptr*3] = delayBuffAngle_x[ptr4];
					angleSVDBuff[allignptr*3 + 1] = delayBuffAngle_y[ptr4];
					angleSVDBuff[allignptr*3 + 2] = delayBuffAngle_z[ptr4];
					allignptr++;
					if (allignptr == 200)
						allignptr = 0;
					alligncounter++;
					if (sz_prev < 0 && delayBuff_z[ptr4] > 0)
					{
						if (alligncounter < 200)
						{
							AllignGyroAcc(gyroSVDBuff, angleSVDBuff, alligned, eigenvecs, alligncounter);
							//Matrix_Multiply(gyroSVDBuff, eigenvecs, alligned, alligncounter, 3, 3);
							for (i = 0; i < alligncounter; i++)
							{
								allign << alligned[i*3] << std::endl;
								unalligned << gyroSVDBuff[i*3] << "\t" << gyroSVDBuff[i*3 + 1] << "\t" << gyroSVDBuff[i*3 + 2] << std::endl;
								alakibuff[i] = alligned[i*3];
							}
						}
						start = 1;
					}
					break;
			}
			if (start == 1)
			{
			  start = 0;
			  f = 0;
			  state2 = 0;
			  for (i = 0; i < alligncounter; i++)
				  f += gyroSVDBuff[i*3 + 2]*alligned[i*3];
			  repstart = 0;
			  repend = alligncounter;
			  mid_point = 0;
			  if (f > 0)
			  {
				  for (i = 1; i < alligncounter-1; i++)
				  {
					   if (alligned[(i-1)*3] > 0 && alligned[(i)*3] < 0)
							  mid_point = 1;
					  if (state2 == 0)
					  {
						  if (alligned[(i-1)*3] < 0 && alligned[(i)*3] > 0 && mid_point == 0)
						  {
							  repstart = i;
							  state2 = 1;
						  }
						  else if (alligned[(i-1)*3] < 0 && alligned[(i)*3] > 0 && mid_point == 1)
						  {
							  repend = i;
							  break;
						  }
					  }
					  else if (state2 == 1)
					  {
						  if (alligned[(i-1)*3] < 0 && alligned[(i)*3] > 0)
						  {
							  repend = i;
							  break;
						  }
					  }
				  }
			  }
			  else
			  {
				  for (i = 1; i < alligncounter-1; i++)
				  {
					  if (state2 == 0)
					  {
						  if (alligned[(i-1)*3] > 0 && alligned[(i)*3] < 0)
						  {
							  repstart = i;
							  state2 = 1;
						  }
					  }
					  else if (state2 == 1)
					  {
						  if (alligned[(i-1)*3] > 0 && alligned[(i)*3] < 0)
						  {
							  repend = i;
							  break;
						  }
					  }
				  }
			  }
			  alligncounter = 0;
			  allignptr = 0;
			  current_length = repend - repstart;

			  if (quadStartLength < repend)
				 quadStartLength = abs(quadStartLength - repstart);
			  else
				 quadStartLength = abs(quadStartLength - repend);

			  if (quadEndLength < repend)
				 quadEndLength = abs(quadEndLength - repstart);
			  else
				 quadEndLength = abs(quadEndLength - repend);

			  if (quad_detected_delayed == 1)
			  {
				if (length != 0)
				{
					angle  = (float)quadStartLength/current_length*360;
					if (angle >= 360)
						angle = 0;
				}
				else
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
				//if (RepCount_Q > 2)
				//	f6 << angle << "\t\t";
				if (length != 0)
				{
					angle  = (float)quadEndLength/current_length*360;
					if (angle >= 360)
						angle = 0;
				}
				else
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
				 //if (RepCount_Q > 2)
				//	f6 << angle << std::endl;
				 quad_detected_delayed = 0;
			  }
			  if (ham_detected_delayed == 1)
			  {
				if (length != 0)
				{
					angle  = (float)hamStartLength/current_length*360;
					if (angle >= 360)
						angle = 0;
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
				//if (RepCount_H > 2)
				//	f6_2 << angle << "\t\t";
				if (length != 0)
				{
					angle  = (float)hamEndLength/current_length*360;
					if (angle >= 360)
						angle = 0;
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
				//if (RepCount_H > 2)
				//	f6_2 << angle << std::endl;
				 ham_detected_delayed = 0;
			  }
			  length = 0;
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
		  //g = (float)(delayBuff[ptr4]-sz_prev)*(delayBuff[ptr4]-sz_prev) + (float)(6.4e-5);
		  length++;// += sqrt(g);
		  sz_prev = delayBuff_z[ptr4];
		  szangle_prev = delayBuffAngle_z[ptr4];
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
		}
		if (GyroQuadE == 1)
		{
			quadEndLength = length;
			quad_detected_delayed = 1;
		}
		if (GyroHamS == 1)
		{
			hamStartLength = length;
		}
		if (GyroHamE == 1)
		{
			hamEndLength = length;
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
