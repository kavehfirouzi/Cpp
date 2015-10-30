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
extern std::ofstream f1, f2, f3, f4, f5, f6, f6_1, f6_2;
void velocityX(int x, int *v, int init);
void PositionX(int x, int *pos, int init);
void velocityY(int y, int *v, int init);
void PositionY(int y, int *pos, int init);
extern float Arctan(float X, float Y);
int posx, posy; 
int reference;
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
	///////centring position/////
	static int centerBuffX[ACTIVITY_WIN];
	static int centerBuffY[ACTIVITY_WIN];
	static int sum_1 = 0;
	static int high_1;
	static int ptr1_1 = 0;
	static int ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
	static int sum_2 = 0;
	static int high_2;
	static int ptr2_1 = 0;
	static int ptr2_2 = (ACTIVITY_WIN - 1)/2 + 1;
	/////////////////////////////
	float sum;
	float max;
	int i;
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
		max = -1e30;
		counter++;
		velocityX(raw_x.raw_data, &vx, 0);
		velocityY(raw_y.raw_data, &vy, 0);
		if ( (counter & 0x1) != 0 || counter >= 125)
		{
			PositionX(vx, &posx, 0);
			PositionY(vy, &posy, 0);
			//f6 << posx << std::endl;
			//f6_2 << vy << std::endl;
			sum = (float)posx*posx + (float)posy*posy;
			//ftest << vx << std::endl;
			//f6_2 << sum << std::endl;
			buffer[ptr1] = sum;
			x_buffer[ptr1] = posx;
			y_buffer[ptr1] = posy;
			for (i = 0; i<51; i++)
			{
				if (buffer[i] > max)
					max = buffer[i];
			}
			ptr2 = ptr1 + 25;
			if (ptr2 > 50)
				ptr2 -= 51;
			if (buffer[ptr2] == max && x_buffer[ptr2] > 0)
			{	
				//f6 << 1 << std::endl;
				reference = Arctan(x_buffer[ptr2],  y_buffer[ptr2]);
				//f6 << reference << std::endl;
				//ftest << global_counter << std::endl;
			}
			//else
				//f6 << 0 << std::endl;
			ptr1++;
			if (ptr1 == 51)
				ptr1 = 0;
			//Segmentation(posx);
		}
	}
	else
	{
		counter = 0;
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
	static int sum_1 = 0;
	static int high_1;
	static int ptr1_1 = 0;
	static int ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
	static int cumsum_1 = 0;
	static int counter = 0;
	static int state = 0;
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
			sum_1 += x;
			Buffer_1[ptr1_1] = x;
			counter++;
			if ( (counter & 0x1) != 0)
			{
				high_1 = Buffer_1[counter/2] - (sum_1)/counter;
				//if (state == 2)
					cumsum_1 += high_1;
				*v = cumsum_1;
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
			sum_1 = sum_1 - Buffer_1[ptr1_1] + x;											// Oldest value is subtracted from the sum and new value is added
			Buffer_1[ptr1_1] = x;																		// new value replaes the oldest value in the buffer
			high_1 = Buffer_1[ptr1_2] - (sum_1)/ACTIVITY_WIN; // The average of the window is subtracted from the middle point of the window
			//if (state == 2)
				cumsum_1 += high_1;
			(ptr1_1)++;																							// Increment the pointers to implement the sliding effect and set them to zero if they reach the end of the buffer
			(ptr1_2)++;
			if ( (ptr1_1) == ACTIVITY_WIN )																
				ptr1_1 = 0;
			if ( (ptr1_2) == ACTIVITY_WIN )
				ptr1_2 = 0;
			*v = cumsum_1;
		}
		
	}
	else
	{
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
	static int sum_1 = 0;
	static int high_1;
	static int ptr1_1 = 0;
	static int ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
	static int cumsum_1 = 0;
	static int counter = 0;
	if (init == 0)
	{
		if (counter < ACTIVITY_WIN)
		{
			sum_1 += x;
			Buffer_1[ptr1_1] = x;
			counter++;
			if ( (counter & 0x1) != 0)
			{
				high_1 = Buffer_1[counter/2] - (sum_1)/counter;
				cumsum_1 += high_1;
				*pos = cumsum_1;
				ftest << high_1 << std::endl;
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
			sum_1 = sum_1 - Buffer_1[ptr1_1] + x;											// Oldest value is subtracted from the sum and new value is added
			Buffer_1[ptr1_1] = x;																		// new value replaes the oldest value in the buffer
			high_1 = Buffer_1[ptr1_2] - (sum_1)/ACTIVITY_WIN; // The average of the window is subtracted from the middle point of the window
			cumsum_1 += high_1;
			ftest << high_1 << std::endl;
			(ptr1_1)++;																							// Increment the pointers to implement the sliding effect and set them to zero if they reach the end of the buffer
			(ptr1_2)++;
			if ( (ptr1_1) == ACTIVITY_WIN )																
				ptr1_1 = 0;
			if ( (ptr1_2) == ACTIVITY_WIN )
				ptr1_2 = 0;
			*pos = cumsum_1;
		}
		
	}
	else
	{
		counter = 0;
		cumsum_1 = 0;
		ptr1_1 = 0;
		ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
		sum_1 = 0;
	}
}

void velocityY(int y, int *v, int init)
{
	static int Buffer_1[ACTIVITY_WIN];
	static int sum_1 = 0;
	static int high_1;
	static int ptr1_1 = 0;
	static int ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
	static int cumsum_1 = 0;
	static int counter = 0;
	static int state = 0;
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
			sum_1 += y;
			Buffer_1[ptr1_1] = y;
			counter++;
			if ( (counter & 0x1) != 0)
			{
				high_1 = Buffer_1[counter/2] - (sum_1)/counter;
				//if (state == 2)
					cumsum_1 += high_1;
				*v = cumsum_1;
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
			sum_1 = sum_1 - Buffer_1[ptr1_1] + y;											// Oldest value is subtracted from the sum and new value is added
			Buffer_1[ptr1_1] = y;																		// new value replaes the oldest value in the buffer
			high_1 = Buffer_1[ptr1_2] - (sum_1)/ACTIVITY_WIN; // The average of the window is subtracted from the middle point of the window
			//if (state == 2)
				cumsum_1 += high_1;
			(ptr1_1)++;																							// Increment the pointers to implement the sliding effect and set them to zero if they reach the end of the buffer
			(ptr1_2)++;
			if ( (ptr1_1) == ACTIVITY_WIN )																
				ptr1_1 = 0;
			if ( (ptr1_2) == ACTIVITY_WIN )
				ptr1_2 = 0;
			*v = cumsum_1;
		}
		
	}
	else
	{
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
	static int sum_1 = 0;
	static int high_1;
	static int ptr1_1 = 0;
	static int ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
	static int cumsum_1 = 0;
	static int counter = 0;
	if (init == 0)
	{
		if (counter < ACTIVITY_WIN)
		{
			sum_1 += y;
			Buffer_1[ptr1_1] = y;
			counter++;
			if ( (counter & 0x1) != 0)
			{
				high_1 = Buffer_1[counter/2] - (sum_1)/counter;
				cumsum_1 += high_1;
				*pos = cumsum_1;
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
			sum_1 = sum_1 - Buffer_1[ptr1_1] + y;											// Oldest value is subtracted from the sum and new value is added
			Buffer_1[ptr1_1] = y;																		// new value replaes the oldest value in the buffer
			high_1 = Buffer_1[ptr1_2] - (sum_1)/ACTIVITY_WIN; // The average of the window is subtracted from the middle point of the window
			cumsum_1 += high_1;
			(ptr1_1)++;																							// Increment the pointers to implement the sliding effect and set them to zero if they reach the end of the buffer
			(ptr1_2)++;
			if ( (ptr1_1) == ACTIVITY_WIN )																
				ptr1_1 = 0;
			if ( (ptr1_2) == ACTIVITY_WIN )
				ptr1_2 = 0;
			*pos = cumsum_1;
		}
		
	}
	else
	{
		counter = 0;
		cumsum_1 = 0;
		ptr1_1 = 0;
		ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
		sum_1 = 0;
	}
}