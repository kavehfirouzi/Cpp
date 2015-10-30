#include "algorithms.h"
#include <math.h>
#include <fstream>
extern std::ofstream f1, f2, f3, f4, f5, f6, f6_1, f6_2;
extern uint32_t global_counter;
#define WIN_SIZE 250																				// Size of the classification window																			// Size of the sliding window size used for smoothing
extern uint8_t cadence_acc;
extern uint32_t quad_detected;
/**
 * @brief	 Segmenting the accelerometer data, for real time purposes, data is fed to the algorithm one at time.
 * @param  rawdata	   : raw accelerometer data
 * @param	 Results     : pointer to the buffer to puy the results in (Bluetooth buffer)
 * @parami init  	     : if init = 0, initialize, else run the algorithm
 * @return	nothing
 * @detail             : Cadence is based on segmenting the accelerometer data based on a simple classification. A sliding window moves over smoothed acc data.
 *                     : The Maximum and Minimum inside the window are calculated each time. The latest data inside the window is classified based on it's value
 *										 : compared to the average of the Maximum and Minimum value.
 */	

extern int posx, posy; 
extern unsigned int quad_peaks[5];
extern struct Ham_Seg h_s[5];

void UnsupervisedQuad(float Sigma, int *Class);
void UnsupervisedHam(float Sigma, int *Class);
float Arctan(float X, float Y);
extern float reference;	
extern float slope;
extern int TopX, TopY;
extern int BottX, BottY;
void Unsupervised(float Sigma, int *Class, int sel)
{
	 if (sel == 0)
		UnsupervisedQuad(Sigma, Class);
	 else
		UnsupervisedHam(Sigma, Class);
}

void UnsupervisedQuad(float Sigma, int *Class)
{  

	static float SigmaBuff[500];															// Buffer to hold smoothed accelerometer values					
	static int pnt = 0;	
	static int QuadAngles[2];
	//////////////////
	float max = -10000000;
	float min =  10000000;
	int j;
	float f;
	global_counter++;
	//for (j = 0 ; j < 5; j++)
	//{
	//	if ( quad_peaks[j] == global_counter - 310 )   //190: total delay for ekf algorithm
	//	{
	//		/*QuadAngles[0] = QuadAngles[1];
	//		QuadAngles[1] = Arctan((float)posx, (float)(posy));
	//		if ( reference > 90 && reference < 180 &&  QuadAngles[1] < -90 && QuadAngles[1] > -180)
	//		{
	//			QuadAngles[1] += 360;
	//		}
	//		QuadAngles[1]  = reference - QuadAngles[1];
	//		if (QuadAngles[1] < 0)
	//			QuadAngles[1] += 360;
	//		f6 << QuadAngles[1] << std::endl;*/
	//		f = (posx - TopX)*slope;
	//		f *= f;
	//		f += (posx - TopX)^2;
	//		if (reference != 0 )
	//			QuadAngles[1] = (f)/reference*180;
	//		f6 << QuadAngles[1] << std::endl;
	//		break;
	//	}
	//}
	SigmaBuff[pnt++] = Sigma;
	if (pnt == WIN_SIZE)
		pnt = 0;
	for (j=0; j < WIN_SIZE ; j++)														// Shifting the values inside the buffer and finding the maximum and minimum 
	{
		if (max < SigmaBuff[j])
				max = SigmaBuff[j];
		if (min > SigmaBuff[j])
				min = SigmaBuff[j];
	}
	if (Sigma > (max-min)/3+ min )   																		// Classification of accelerometer values
	{
		*Class = 1;
	}
	else
	{
		*Class = 0;
	}
}

void UnsupervisedHam(float Sigma, int *Class)
{  
	static float SigmaBuff[500];															// Buffer to hold smoothed accelerometer values					
	 static int pnt = 0;	
	 static int HamAngles[2];
	//////////////////
	float max = -10000000;
	float min =  10000000;
	int j;
	for (int i = 0 ; i < 5; i++)
	{
		//if ( h_s[i].point_mid  == global_counter - 310  )   //190: total delay for ekf algorithm
		//{
		//	HamAngles[0] = HamAngles[1];
		//	HamAngles[1] = Arctan((float)posx, (float)(posy));
		//	//QuadHams[1] -= (reference + 90);
		//	if ( reference > 90 && reference < 180 &&  HamAngles[1] < -90 && HamAngles[1] > -180)
		//	{
		//		HamAngles[1] += 360;
		//	}
		//	HamAngles[1] = reference - HamAngles[1];
		//	if (HamAngles[1] < 0)
		//		HamAngles[1] += 360;
		//	f6_2 << HamAngles[1] << std::endl;
		//	break;
		//}
	}
	SigmaBuff[pnt++] = Sigma;
	if (pnt == WIN_SIZE)
		pnt = 0;
	for (j=0; j < WIN_SIZE ; j++)														// Shifting the values inside the buffer and finding the maximum and minimum 
	{
		if (max < SigmaBuff[j])
				max = SigmaBuff[j];
		if (min > SigmaBuff[j])
				min = SigmaBuff[j];
	}
	if (Sigma > (max-min)/3+ min )   																		// Classification of accelerometer values
	{
		*Class = 1;
	}
	else
	{
		*Class = 0;
	}
	/*static float SigmaBuff[25];															// Buffer to hold smoothed accelerometer values		
	static float MaxBuff[500];
	static float MinBuff[500];
    static int pnt = 0;	
	static int pnt2 = 0;
	float max = -50000;
	float min =  50000;
	float Maximum;
	float Minimum;
	int j, s;
	
	SigmaBuff[pnt++] = Sigma;
	if (pnt == WIN_SIZE)
		pnt = 0;
	for (j=0; j < WIN_SIZE ; j++)														// Shifting the values inside the buffer and finding the maximum and minimum 
	{
		if (max < SigmaBuff[j])
				max = SigmaBuff[j];
		if (min > SigmaBuff[j])
				min = SigmaBuff[j];
	}
	MaxBuff[pnt2] = max;
	MinBuff[pnt2] = min;
	s = pnt2;
	pnt2++;
	if (pnt2 == 500)
		pnt2 = 0;
	Maximum = max;
	Minimum = min;
	for (j = 0; j<20; j++)
	{
		if (MaxBuff[s] > Maximum)
			Maximum = MaxBuff[s];
		if (MinBuff[s] < Minimum)
			Minimum = MinBuff[s];
		s -= 25;
		if (s<0)
			s += 500;
	}
	if (Sigma > ((Maximum+Minimum)/2+Minimum)/2)   																		// Classification of accelerometer values
	{
		*Class = 1;
	}
	else
	{
		*Class = 0;
	}*/
}

float Arctan(float X, float Y)
{
	float R;
	float R2;
	float out;
	float div[19] = {0.16667,0.075,0.044643,0.030382,0.022372,0.017353,0.013965,0.011552,0.0097616,0.0083903,0.0073125,0.0064472,0.00574,0.0051533,0.0046601,0.0042409,0.003881,0.0035692,0.0032971};
	int i;

	/*if (X == 0 && Y > 0)
		return 0;
	else if (X == 0 && Y < 0)
		return 180;*/

	if (Y == 0 && X > 0)
		return 90;
	else if (Y == 0 && X < 0)
		return -90;

	//R = abs(Y/X);
	//R = Y/X;
	R = X/Y;
	R = R / sqrt(1+R*R);
	out = R;
	R2 = R*R;
	
	for (i = 0; i < 19; i++)
	{
		R = R*R2;
		out += R*div[i];
	}
	

	if (X > 0 && Y < 0)
		return out*57.2958 + 180;
	else if (X < 0 && Y < 0)
		return out*57.2958 - 180;
	else
		return out*57.2958;
}