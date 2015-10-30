#include "algorithms.h"
#include <math.h>
#include <fstream>
extern std::ofstream GyroAngle, f4;
#define pi (float)3.1415
#define MaxMinLength 125
#define AccSmoothWin 125
#define GyroDelay (AccSmoothWin-1)/2 + 1
#define THRESH 15
#define GyroDivFact 5

void Peakdetection_Gyro(int in, int* out);
int cadence_gyro;
void cadenceGyro(uint8_t* rawData, uint8_t* rawDataacc, uint8_t *Results, int init)
{
	static int32_t GyroDelay_Buf[GyroDelay];
	static int32_t gyroPnt = 0;
	static int32_t gyroPnt_delay;
	static int32_t AccX_Buf[AccSmoothWin];
	static int32_t AccY_Buf[AccSmoothWin];
	static int32_t ptrAcc = 0;
	static int Sum_X = 0;
	static int Sum_Y = 0;
	int X, Y, gyroZ;
	float gyroZ_delayed, angleAcc;
	static float Angle = 0;
	RAW_DATA raw_x, raw_y, raw_Gyro;
	int Max = -1000000;
	int Min = 1000000;
	static float MaxMin_Buf[125];
	static int MaxMin_pnt = 0;
	////////////////
	static float S[3] = {1, 3, 1};
	float s1, s2, s3;
	//static int counter = 0;
	int i;
	int Segment;
	static int cadence;
	////////////////
	if (init == 0)
	{
		raw_x.raw_data = 0;
		raw_y.raw_data = 0;

		raw_x.data_byte[0] = rawData[0];	
		raw_x.data_byte[1] = rawData[1];
		if ((rawData[1] & 0x80) == 0x00)																							// Input data is positive, set the sign to 0
		{
			raw_x.data_byte[2] = 0;
			raw_x.data_byte[3] = 0;
		}
		else																																					// Input data is negative, set the sign to 1
		{
			raw_x.data_byte[2] = 0xFF;
			raw_x.data_byte[3] = 0xFF;
		}

		raw_y.data_byte[0] = rawData[2];	
		raw_y.data_byte[1] = rawData[3];
		if ((rawData[3] & 0x80) == 0x00)																							// Input data is positive, set the sign to 0
		{
			raw_y.data_byte[2] = 0;
			raw_y.data_byte[3] = 0;
		}
		else																																					// Input data is negative, set the sign to 1
		{
			raw_y.data_byte[2] = 0xFF;
			raw_y.data_byte[3] = 0xFF;
		}

		raw_Gyro.raw_data = 0;
		/****don't forget to change this*****/
		raw_Gyro.data_byte[0] = rawData[4];	
		raw_Gyro.data_byte[1] = rawData[5];
		if ((rawData[5] & 0x80) == 0x00)																							// Input data is positive, set the sign to 0
		{
			raw_Gyro.data_byte[2] = 0;
			raw_Gyro.data_byte[3] = 0;
		}
		else																																					// Input data is negative, set the sign to 1
		{
			raw_Gyro.data_byte[2] = 0xFF;
			raw_Gyro.data_byte[3] = 0xFF;
		}

		X = raw_x.raw_data;
		Y = raw_y.raw_data;
		gyroZ = raw_Gyro.raw_data;
		////////////////
		//Smoothing ACC and delaying Gyro//
		Sum_X = Sum_X - AccX_Buf[ptrAcc] +  X;
		Sum_Y = Sum_Y - AccY_Buf[ptrAcc] +  Y;
		AccX_Buf[ptrAcc] = X;
		AccY_Buf[ptrAcc] = Y;
		X = Sum_X / AccSmoothWin;
		Y = Sum_Y / AccSmoothWin;
		ptrAcc++;
		if (ptrAcc == AccSmoothWin)
			ptrAcc = 0;
		GyroDelay_Buf[gyroPnt] = gyroZ;
		gyroPnt_delay = gyroPnt + 1;
		if (gyroPnt_delay == 63)
			gyroPnt_delay = 0;
		gyroZ_delayed = (float)GyroDelay_Buf[gyroPnt_delay]/32.8;
		gyroPnt++;
		if (gyroPnt == 63)
			gyroPnt = 0;
		//if (counter >= 0)
		//{
		angleAcc = 180*atan2((float)Y, (float)X)/pi;
		s1 = S[0] +  (float)0.576610843847976*(gyroZ_delayed - S[0] - S[1]);
		s2 = S[1] +  (float)0.057661084384681*(gyroZ_delayed - S[0] - S[1]);
		//s3 = S[2] +  (float)0.954451150103322*(angleAcc - S[2]);
		S[0] = s1;
		S[1] = s2;
		//S[2] = s3;
		Angle = (float)0.98*(gyroZ_delayed*(float)0.008 + Angle);// + (float)(0.05)*angleAcc;
		//GyroAngle << Angle  << std::endl;
		//}
		//counter++;
		MaxMin_Buf[MaxMin_pnt] = Angle;
		for (i = 0; i <MaxMinLength; i++)
		{
			if (Max < MaxMin_Buf[i])
				Max = MaxMin_Buf[i];
			if (Min > MaxMin_Buf[i])
				Min = MaxMin_Buf[i];
		}
		MaxMin_pnt++;
		if (MaxMin_pnt == MaxMinLength)
			MaxMin_pnt = 0;
		if (Angle > (Max-Min)/GyroDivFact + Min)// && Max - Min > THRESH)   																		// Classification of accelerometer values
			Segment = 1;
		else
			Segment = 0;
		Peakdetection_Gyro(Segment, &cadence);
		if (Max-Min < THRESH)
			cadence = 0;
		cadence_gyro = cadence;
		GyroAngle << Segment  << std::endl;
		//f4 << cadence_gyro << std::endl;
	}
	else
	{
		Angle = 0;
		//counter = 0;
	}
}

void Peakdetection_Gyro(int in, int* out)
{
	static int state = 0;																					// State machine
	static int counter = 0;																				// Counter to keep track of distance between segments
	static int dists[2] = {0,0};																	// Buffer to hold distances for averaging. Everytime a new distance is calculated, oldest one is tossed out and new one replaces it
	static int pointer = 0;																				
	static int peak_counter = 0;                                  // Keeps track of the numbner of segments detected so far.
	static int stop_counter = 0;																	// Counter to keep track of time no segment has been detected
	static int stop_counter_enable = 0;                           // Enables and disable the above counter
	int i;
	static int StartStop = 0;
	static long long max , min;
	float f;
	//
	static int mainCounter = 0;
	static int start = 0;
	static int stop = 0;
	static int currentRep = 0;
	static int prevRep = 0;
	if (stop_counter == 375)                                      // If no segment has been detected for the pase 250 samples (2 seconds), reset all buffers and variables
	{
		*out = 0;
		state = 0;
		counter = 0;
		pointer = 0;
		peak_counter = 0;
		stop_counter = 0;
		stop_counter_enable = 0;
		for (i = 0; i < 2; i++)
		{
			dists[i] = 0;
		}
	}
	if (stop_counter_enable == 1)																	
				stop_counter++;
	mainCounter++;
	switch(state)
	{
		case 0:
			counter++;
			if (in == 1)
			{
				StartStop = 0;
				start = mainCounter;
				state = 1;
			}
			break;
		case 1:
			StartStop++;
			counter++;
			if (in == 0)
			{
				stop = mainCounter;
				if (peak_counter > 0)																		// Calculate cadence if more than 1 segment have been detected so far
				{
					stop_counter = 0;			
					currentRep = (start+stop)/2;// Reset the stop_counter to zero
					stop_counter_enable = 1;                              // Reset counter only should start counting when at least two segments have been detected
					dists[pointer] = counter - StartStop / 2;															// Putting the current distance in the buffer
					//dists[pointer] = currentRep - prevRep;
					pointer++;
					if (pointer == 2)
						pointer = 0;
					if (peak_counter < 2)																	// If only one distance has been detected so far, output it as cadence
					{
						if (dists[0] != 0)
						{
							*out = 7500  / (dists[0]);
							f4 << *out << std::endl;
						}
					}
					else																									// If more than one distances have been detected, start averaging over the last two each time
					{
						if ((dists[0] + dists[1]) != 0)
						{
							*out = 7500 * 2 / (dists[0] + dists[1]);
							f4 << *out << std::endl;
						}
					}
				}
				peak_counter++;
				counter = StartStop / 2;
				prevRep = currentRep;
				state = 0;
			}
			break;
	}
}