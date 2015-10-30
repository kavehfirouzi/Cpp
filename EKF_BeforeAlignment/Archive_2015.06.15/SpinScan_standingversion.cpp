#include "algorithms.h"
#include <math.h>
#define ACTIVITY_WIN 125
#include <fstream>
extern std::ofstream f1, f2, f3, f4, f5, ftest;
extern unsigned int CurrentQuadPeak;															// Holds the value of the current quad peak
extern unsigned int CurrentHamPeak;     
extern uint32_t global_counter;
int Segmentation(int Pos);
int AngleQuad;
int AngleHam;
#include <fstream>
extern std::ofstream f1, f2, f3, f4, f5, f6, f6_1, f6_2, X, Y;
void velocityX(int x, int *v, int init);
void PositionX(int x, int *pos, int init);
void velocityY(int y, int *v, int init);
void PositionY(int y, int *pos, int init);
extern float Arctan(float X, float Y);
int posx, posy; 
float reference = 0;
float slope = 0;
int TopX = 0;
int TopY = 0;
int BottX, BottY;
extern uint32_t quad_detected;
extern uint32_t ham_detected;
int posptrH = 0;
extern uint8_t  cadence_acc;
int length_1 = 0;
int length_2 = 0;
int length = 0;
int current_length = 0;
long long diff_y;
long long diff_x;
extern unsigned int quadlength;
int posptrQ_s = 0;
int posptrQ_e = 0;
int posptrH_s = 0;
int posptrH_e = 0;
extern int Quad_Start;
extern int Quad_End;
extern int Ham_Start;
extern int Ham_End;
extern int SegmentCad;
void SpinScan(uint8_t* rawData, int init)
{
	RAW_DATA raw_x, raw_y;
	static float buffer[101];
	static int x_buffer[101];
	static int y_buffer[101];
	static int ptr1 = 0;
	static int ptr2;
	static int counter = 0;
	static int sx, sy, sz, vx, vy;
	float k;
	//////////////////////////////////
	static float delay_buffer_x[63];
	static float delay_buffer_y[63];
	static int delayptr1 = 0;
	static int delayptr2;
	//////////////////////////////////

	static float dummy_1[ACTIVITY_WIN];																				// The buffer to hold quad RMS values
	static float dummy_2[ACTIVITY_WIN];																				// The buffer to hold ham RMS values
	static float sum_x = 0;																								// Variables to hold the sum of the values inside the sliding window
	static float sum_y  = 0;
	static int ptr3 = 0;																										// Pointer to the place inside the buffer to add the new data

	/////////////////////////////

	static int PosQBuff_start[65];
	static int PosQBuff_end[65];
	static int PosHBuff_start[65];
	static int PosHBuff_end[65];
	static int PosHBuffX[65];
	static int PosHBuffY[65];
	static int PosQBuffVel[65];
	static int PosHBuffVel[65];
	static int PQ = 0, PH = 0;
	float sum;
	float max, min;
	int i;
	float f, g;
	int shift_x, shift_y;
	static int state = 0;
	static int zero_crossing_q = 0;
	static int zero_crossing_h = 0;
	float angle;
	static int RepCount_Q = 0;
	static int averageQ_e[3];

	static int current_length_copy = 0;
	static int current_length_copy2;
	static int v_x_prev = 0;
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

		delay_buffer_x[delayptr1] = raw_x.raw_data;
		delay_buffer_y[delayptr1] = raw_y.raw_data;
		delayptr2 = delayptr1+1;
		if (delayptr2 == 35)
			delayptr2 = 0;
		delayptr1++;
		if (delayptr1 == 35)
			delayptr1 = 0;
		max = -1e30;
		min = 1e30;
		////////////////smoothing/////////////////////
		/*sum_x = sum_x - dummy_1[ptr3] + raw_x.raw_data;																			// Oldest value is subtracted from the sum and new value is added
		dummy_1[ptr3] = raw_x.raw_data;																										// New value replaes the oldest value in the buffer
		if (counter < 21-1)
			sx = 0;
		else																																		
			sx = (float)(sum_x)/21;																	// Outout is the average of the window
		
		sum_y = sum_y - dummy_2[ptr3] + raw_y.raw_data;																			// Oldest value is subtracted from the sum and new value is added
		dummy_2[ptr3] = raw_y.raw_data;																										// New value replaes the oldest value in the buffer
		if (counter < 21-1)
			sy = 0;
		else																																		
			sy = (float)(sum_y)/21;	

		(ptr3)++;																															// Increment the pointers to implement the sliding effect and set them to zero if they reach the end of the buffer
		if ( ptr3 == 21 )
			ptr3 = 0;*/

		counter++;				

		//f6 << sx << std::endl;
		//f6_2 << sy << std::endl;
		/////////////////////////////////////////////
		velocityX(raw_x.raw_data, &vx, 0);
		velocityY(-raw_y.raw_data, &vy, 0);
		
		if ( (counter & 0x1) != 0 || counter >= 125)
		{
			//X << vx << std::endl;
			Y << vy << std::endl;
			if (Quad_Start == 1 && posptrQ_s < 15)
			{
				PosQBuff_start[posptrQ_s] = current_length;
				posptrQ_s++;
			}
			if (Quad_End == 1 && posptrQ_e < 15)
			{
				PosQBuff_end[posptrQ_e] = current_length;
				posptrQ_e++;
			}
			if (Ham_Start == 1 && posptrH_s < 15)
			{
				PosHBuff_start[posptrH_s] = current_length;
				posptrH_s++;
			}
			if (Ham_End == 1 && posptrH_e < 15)
			{
				PosHBuff_end[posptrH_e] = current_length;
				posptrH_e++;
			}
			switch(state)
			{
				case 0:
					if (SegmentCad == 1)
					{
						state = 1;
						current_length_copy = current_length;
					}
					break;
				case 1:
					if (v_x_prev > 0 && vx < 0)
					{
						current_length = 0;
						current_length_copy2 = current_length_copy;
					}
					if (SegmentCad == 0)
					{
						state = 0;
						length = current_length_copy2;
					}
					break;
			}
			v_x_prev = vx;
			/*switch(state)
			{
				case 0:
					if (vx < 0)
					{
						state = 1;
						length = current_length;
						zero_crossing_q = 1;
						zero_crossing_h = 1;
						current_length = 0;
					}
					break;
				case 1:
					if (vx > 0)
						state = 0;
					break;
			}*/
		  if (posptrQ_e == 15)
		  {
			if (length != 0)
			{
				angle  = (float)PosQBuff_start[14]/length*360;
				if (angle > 360)
					angle = 0;
			}
			else
				angle = 0;
			f6 << angle << "\t\t";
			if (length != 0)
			{
				angle  = (float)PosQBuff_end[14]/length*360;
				if (angle > 360)
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
				
				if (abs(averageQ_e[2] - averageQ_e[1]) >= 180)
						averageQ_e[2] -= 360;
				angle = (averageQ_e[1] +  averageQ_e[2])/2;
				if (angle < 0)
						angle += 360;
				if (abs(angle - averageQ_e[0]) >= 180)
					angle -= 360;
				angle = (averageQ_e[0] +  angle)/2;
				if (angle < 0)
					angle += 360;
			 }
			 RepCount_Q++;
			 f6 << angle << std::endl;
			 zero_crossing_q = 0;
			 quad_detected = 0;
			 Quad_Start = 0;
			 Quad_End = 0;
			 posptrQ_e = 0;
			 posptrQ_s = 0;
		  }
		  if (posptrH_e == 15)
		  {
			if (length != 0)
			{
				angle  = (float)PosHBuff_start[14]/length*360;
				if (angle > 360)
					angle = 180;
			}
			else
				angle = 180;
			f6_2 << angle << "\t\t";
			if (length != 0)
			{
				angle  = (float)PosHBuff_end[14]/length*360;
				if (angle > 360)
					angle = 180;
			}
			else
				angle = 180;
			 f6_2 << angle << std::endl;
			 zero_crossing_h = 0;
			 Ham_Start = 0;
			 Ham_End = 0;
			 posptrH_e = 0;
			 posptrH_s = 0;
		  }
		  g = (float)vx*vx + (float)vy*vy;
		  current_length += sqrt(g);
		  current_length_copy += sqrt(g);
		}
	}
	else
	{
		state = 0;
		current_length = 0;
		current_length_copy = 0;
		//counter = 0;
		//velocityX(0, &i, 1);
		PositionX(0, &i, 1);
		//velocityY(0, &i, 1);
		PositionY(0, &i, 1);
		
	}
}

//void Position(int x, int *pos, int init);
//void SpinScan(uint8_t* rawData, int init)
//{
//	RAW_DATA raw_x;
//	static int posx;
//	if (init == 0)
//	{
//		raw_x.raw_data = 0;
//
//		raw_x.data_byte[0] = rawData[0];	
//		raw_x.data_byte[1] = rawData[1];
//		if ((rawData[1] & 0x80) == 0x00)																							// Input data is positive, set the sign to 0
//		{
//			raw_x.data_byte[2] = 0;
//			raw_x.data_byte[3] = 0;
//		}
//		else																																					// Input data is negative, set the sign to 1
//		{
//			raw_x.data_byte[2] = 0xFF;
//			raw_x.data_byte[3] = 0xFF;
//		}
//		Position(raw_x.raw_data, &posx, 0);
//		Segmentation(posx);
//	}
//	else
//	{
//		Position(0, &posx, 1);
//	}
//}

void Position(int x, int *pos, int init)
{
	static int Buffer_1[ACTIVITY_WIN];
	static int sum_1 = 0;
	static int high_1;
	static int ptr1_1 = 0;
	static int ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;

	static int Buffer_2[ACTIVITY_WIN];
	static int sum_2 = 0;
	static int high_2;
	static int ptr2_1 = 0;
	static int ptr2_2 = (ACTIVITY_WIN - 1)/2 + 1;

	static int cumsum_1 = 0;
	static int cumsum_2 = 0;

	if (init == 0)
	{
		sum_1 = sum_1 - Buffer_1[ptr1_1] + x;											// Oldest value is subtracted from the sum and new value is added
		Buffer_1[ptr1_1] = x;																		// new value replaes the oldest value in the buffer
		high_1 = Buffer_1[ptr1_2] - (sum_1)/ACTIVITY_WIN; // The average of the window is subtracted from the middle point of the window
		cumsum_1 += high_1;
		(ptr1_1)++;																							// Increment the pointers to implement the sliding effect and set them to zero if they reach the end of the buffer
		(ptr1_2)++;
		if ( (ptr1_1) == ACTIVITY_WIN )																
			ptr1_1 = 0;
		if ( (ptr1_2) == ACTIVITY_WIN )
			ptr1_2 = 0;

		sum_2 = sum_2 - Buffer_2[ptr2_1] + cumsum_1;											// Oldest value is subtracted from the sum and new value is added
		Buffer_2[ptr2_1] = cumsum_1;																		// new value replaes the oldest value in the buffer
		high_2 = Buffer_2[ptr2_2] - (sum_2)/ACTIVITY_WIN; // The average of the window is subtracted from the middle point of the window
		cumsum_2 += high_2;
		(ptr2_1)++;																							// Increment the pointers to implement the sliding effect and set them to zero if they reach the end of the buffer
		(ptr2_2)++;
		if ( (ptr2_1) == ACTIVITY_WIN )																
			ptr2_1 = 0;
		if ( (ptr2_2) == ACTIVITY_WIN )
			ptr2_2 = 0;

		*pos = cumsum_2;
		//f6 << cumsum_2 << std::endl;
	}
	else
	{
		cumsum_1 = 0;
		cumsum_2 = 0;
	}
}

int Segmentation(int Pos)
{
		static int MaxMinBuff[ACTIVITY_WIN];	
	static int pnt = 0;
	int j;
	static int QuadAngles[2];
	static int HamAngles[2];
	float max_1 = -10000000;
	float min_1 =  10000000;
	float max_2 = -10000000;
	float min_2 =  10000000;
  static int state = 0;
	int Seg1, Seg2;
	static int prevPos = 0;
	static int currentPos = 0;
	int quadpeak, hampeak;
	int i;
	static int maxpos;
	MaxMinBuff[pnt] = Pos;	
	pnt++;
	if (pnt == ACTIVITY_WIN)
		pnt = 0;
	for (j=0; j < ACTIVITY_WIN ; j++)														// Shifting the values inside the buffer and finding the maximum and minimum 
	{
		if (max_1 < -MaxMinBuff[j])
				max_1 = -MaxMinBuff[j];
		if (min_1 > -MaxMinBuff[j])
				min_1 = -MaxMinBuff[j];
		
		if (max_2 < MaxMinBuff[j])
				max_2 = MaxMinBuff[j];
		if (min_2 > MaxMinBuff[j])
				min_2 = MaxMinBuff[j];
	}
	if (-Pos > (max_1-min_1)/5 + min_1 && (max_1-min_1 > 100000))   																		// Classification of accelerometer values
	{
		Seg1 =  1;
	}
	else
	{
		Seg1 =  0;
	}
	//if (Pos > (max_2-min_2)/4 + min_2 && (max_2-min_2> 100000))   																		// Classification of accelerometer values
	//{
	//	Seg2 =  1;
	//}
	//else
	//{
	//	Seg2 =  0;
	//}
	//switch (state)
	//{
	//	case 0:
	//		maxpos = 0;
	//		if (Seg2 == 1)
	//		{
	//			state = 1;
	//			maxpos = Pos;
	//		}
	//		break;
	//	case 1:
	//		if (Pos > maxpos)
	//		{
	//			maxpos = Pos;
	//			currentPos = global_counter - 4*125;                                               //125: total delay for position algorithm
	//		}
	//		if (Seg2 == 0)
	//		{
	//			quadpeak = 0;
	//			hampeak = 0;
	//			for (i = 0 ; i < 5; i++)
	//			{
	//				if ( (quad_peaks[i] - 190) <= currentPos && (quad_peaks[i] - 190) >= prevPos )   //190: total delay for ekf algorithm
	//				{
	//					quadpeak = quad_peaks[i] - 190;
	//					//break;
	//				}
	//			}
	//			for (i = 0 ; i < 5; i++)
	//			{
	//				if ( (h_s[i].point - 190) <= currentPos && (h_s[i].point - 190) >= prevPos )
	//				{
	//					hampeak = (h_s[i].point - 190);
	//					//break;
	//				}
	//			}
	//			if (prevPos > 0)
	//			{
	//				if (quadpeak != 0)
	//				{
	//					QuadAngles[0] = QuadAngles[1];
	//					QuadAngles[1] = (float)(quadpeak - prevPos)/(currentPos - prevPos) * 360;
	//					//AngleQuad = (float)(quadpeak - prevPos)/(currentPos - prevPos) * 360;
	//					
	//					//f6 <<  ((QuadAngles[0] + QuadAngles[1]) >> 2) << std::endl;;
	//				}
	//				if (hampeak != 0)
	//				{
	//					/*AngleHam = (float)(hampeak - prevPos)/(currentPos - prevPos) * 360;
	//					Results->Ham_Angle = AngleHam >> 1;*/
	//					HamAngles[0] = HamAngles[1];
	//					HamAngles[1] = (float)(hampeak - prevPos)/(currentPos - prevPos) * 360;
	//					//Results->Ham_Angle = (HamAngles[0] + HamAngles[1]) >> 2;
	//				}
	//			}
	//			prevPos = currentPos;
	//			state = 0;
	//		}
	//		break;
	//}
	return Seg1;
}

void velocityX(int x, int *v, int init)
{
	static int Buffer_1[ACTIVITY_WIN];
	static long long sum_1 = 0;
	static int high_1;
	static int ptr1_1 = 0;
	static int ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
	static int cumsum_1 = 0;
	static int counter = 0;
	static int state = 0;
	int i;
	if (init == 0)
	{

		/*switch(state)
		{
		case 0:
			if (high_1 < 0)
				state = 1;
			break;
		case 1:
			if (high_1 > 0)
				state = 2;
			break;
		}*/
		if (counter < ACTIVITY_WIN)
		{
			//sum_1 += x;
			cumsum_1 += x;
			sum_1 += cumsum_1;
			Buffer_1[ptr1_1] = cumsum_1;
			counter++;
			if ( (counter & 0x1) != 0)
			{
				high_1 = Buffer_1[counter/2] - (sum_1)/counter;
				//if (state == 2)
					//cumsum_1 += high_1;
				*v = high_1;
			}
			(ptr1_1)++;
			(ptr1_2)++;
			if ( (ptr1_1) == ACTIVITY_WIN )																
				ptr1_1 = 0;
			if ( (ptr1_2) == ACTIVITY_WIN )
				ptr1_2 = 0;
			
		}
		else
		{
			cumsum_1 += x;
			sum_1 = sum_1 - Buffer_1[ptr1_1] + cumsum_1;											// Oldest value is subtracted from the sum and new value is added
			/*if (sum_1 > 1e9)
			{
				sum_1 = 0;
				cumsum_1 = 0;
				ptr1_1 = 0;
				ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
				cumsum_1 += x;
				sum_1 = sum_1 - Buffer_1[ptr1_1] + cumsum_1;
			}*/
			Buffer_1[ptr1_1] = cumsum_1;																		// new value replaes the oldest value in the buffer
			if (counter < ACTIVITY_WIN - 1)
				high_1 = 0;
			else
				high_1 = Buffer_1[ptr1_2] - (sum_1)/ACTIVITY_WIN; // The average of the window is subtracted from the middle point of the window
			//if (state == 2)
			//	cumsum_1 += high_1;
			(ptr1_1)++;																							// Increment the pointers to implement the sliding effect and set them to zero if they reach the end of the buffer
			(ptr1_2)++;
			if ( (ptr1_1) == ACTIVITY_WIN )																
				ptr1_1 = 0;
			if ( (ptr1_2) == ACTIVITY_WIN )
				ptr1_2 = 0;
			if (high_1 > 34e6)
				sum_1 = sum_1*1;
			*v = high_1;
		}
		//counter++;
		
	}
	else
	{
		for (i = 0; i < ACTIVITY_WIN; i++)
			Buffer_1[i] = 0;
		state = 0;
		counter = 0;
		cumsum_1 = 0;
		ptr1_1 = 0;
		ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
		sum_1 = 0;
	}
}

void PositionX(int x, int *pos, int init)
{
	static int Buffer_1[ACTIVITY_WIN];
	static int MaxMinBuff[ACTIVITY_WIN];
	static int pnt = 0;
	static int sum_1 = 0;
	static int high_1;
	static int ptr1_1 = 0;
	static int ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
	static int cumsum_1 = 0;
	static int counter = 0;
	static int state = 0;
	int max = -10000000;
	int min = +10000000;
	int j, i;
	float f;
	static float test_alaki = 0;
	if (init == 0)
	{
		switch(state)
		{
			case 0:
				//X << x << std::endl;
				if (x < 0)
				{
					state = 1;
					length = current_length;
					current_length = 0;
				}
				break;
			case 1:
				
				if (x > 0)
				{
					state = 0;
					//length_1 = current_length;
					//current_length = 0;
				}
				break;
		}
		
		if (state == 1)
			cumsum_1 += x;
		*pos = cumsum_1;
		f = diff_x + diff_y;
		if (f >= 0)
			current_length += sqrt(f);
		else
			current_length = 0;
		
	}
	else
	{
		state = 0;
		/*for (i = 0; i < ACTIVITY_WIN; i++)
		{
			Buffer_1[i] = 0;
			MaxMinBuff[i] = 0;
		}
		counter = 0;*/
		cumsum_1 = 0;
		/*ptr1_1 = 0;
		ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
		sum_1 = 0;*/
	}
}

void velocityY(int y, int *v, int init)
{
	static int Buffer_1[ACTIVITY_WIN];
	static long long sum_1 = 0;
	static int high_1;
	static int ptr1_1 = 0;
	static int ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
	static int cumsum_1 = 0;
	static int counter = 0;
	static int state = 0;
	int i;
	if (init == 0)
	{

		/*switch(state)
		{
		case 0:
			if (high_1 < 0)
				state = 1;
			break;
		case 1:
			if (high_1 > 0)
				state = 2;
			break;
		}*/
		if (counter < ACTIVITY_WIN)
		{
			//sum_1 += x;
			cumsum_1 += y;
			sum_1 += cumsum_1;
			Buffer_1[ptr1_1] = cumsum_1;
			counter++;
			if ( (counter & 0x1) != 0)
			{
				high_1 = Buffer_1[counter/2] - (sum_1)/counter;
				//if (state == 2)
					//cumsum_1 += high_1;
				*v = high_1;
			}
			(ptr1_1)++;
			(ptr1_2)++;
			if ( (ptr1_1) == ACTIVITY_WIN )																
				ptr1_1 = 0;
			if ( (ptr1_2) == ACTIVITY_WIN )
				ptr1_2 = 0;
			
		}
		else
		{
			cumsum_1 += y;
			sum_1 = sum_1 - Buffer_1[ptr1_1] + cumsum_1;											// Oldest value is subtracted from the sum and new value is added
			Buffer_1[ptr1_1] = cumsum_1;																		// new value replaes the oldest value in the buffer
			if (counter < ACTIVITY_WIN - 1)
				high_1 = 0;
			else
				high_1 = Buffer_1[ptr1_2] - (sum_1)/ACTIVITY_WIN; // The average of the window is subtracted from the middle point of the window
			//if (state == 2)
			//	cumsum_1 += high_1;
			(ptr1_1)++;																							// Increment the pointers to implement the sliding effect and set them to zero if they reach the end of the buffer
			(ptr1_2)++;
			if ( (ptr1_1) == ACTIVITY_WIN )																
				ptr1_1 = 0;
			if ( (ptr1_2) == ACTIVITY_WIN )
				ptr1_2 = 0;
			*v = high_1;
		}
		//counter++;
		
	}
	else
	{
		for (i = 0; i < ACTIVITY_WIN; i++)
			Buffer_1[i] = 0;
		state = 0;
		counter = 0;
		cumsum_1 = 0;
		ptr1_1 = 0;
		ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
		sum_1 = 0;
	}
}

void PositionY(int y, int *pos, int init)
{
	static int Buffer_1[ACTIVITY_WIN];
	static int MaxMinBuff[ACTIVITY_WIN];
	static int pnt = 0;
	static int sum_1 = 0;
	static int high_1;
	static int ptr1_1 = 0;
	static int ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
	static int cumsum_1 = 0;
	static int counter = 0;
	static int state = 0;
	int max = -10000000;
	int min = +10000000;
	int j, i;
	int prev_pos = 0;
	if (init == 0)
	{
		MaxMinBuff[pnt] = y;	
		pnt++;
		if (pnt == ACTIVITY_WIN)
			pnt = 0;
		for (j=0; j < ACTIVITY_WIN ; j++)														// Shifting the values inside the buffer and finding the maximum and minimum 
		{
			if (max < MaxMinBuff[j])
					max = MaxMinBuff[j];
			if (min > MaxMinBuff[j])
				min = MaxMinBuff[j];
		}
		switch(state)
		{
			case 0:
				if (y > 0)
					state = 1;
				break;
			case 1:
				if (y < 0)
					state = 2;
				break;
		}
	
		{
		    if (state == 2)
				cumsum_1 += y;
			*pos = cumsum_1;
			//diff_y = y*y;
		}
		
	}
	else
	{
		state = 0;
		/*for (i = 0; i < ACTIVITY_WIN; i++)
		{
			Buffer_1[i] = 0;
			MaxMinBuff[i] = 0;
		}
		counter = 0;*/
		cumsum_1 = 0;
		/*ptr1_1 = 0;
		ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
		sum_1 = 0;*/
	}
}