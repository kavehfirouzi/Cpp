#include "algorithms.h"
#include <math.h>
#include <fstream>
extern std::ofstream f1, f2, f3, f4, f5, f6, f6_1, f6_2, shake;
extern uint32_t global_counter;
#define WIN_SIZE 1200																				// Size of the classification window																			// Size of the sliding window size used for smoothing
#define DEVFACT 7
uint8_t cadence_zeroer = 0;
extern uint32_t cadence_acc;
extern uint32_t quad_detected;
extern int CadAdapt;
extern int cadence_gyro;
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

void UnsupervisedQuad(unsigned short Sigma, int *Class);
void UnsupervisedHam(unsigned short Sigma, int *Class);
void ShakeRemoval(float Sigma, int Class, int init);
float Arctan(float X, float Y);
extern float reference;	
extern float slope;
extern int TopX, TopY;
extern int BottX, BottY;
void Unsupervised(unsigned short Sigma, int *Class, int sel)
{
	 if (sel == 0)
		UnsupervisedQuad(Sigma, Class);
	 else
		UnsupervisedHam(Sigma, Class);
}

void UnsupervisedQuad(unsigned short Sigma, int *Class)
{  

	static unsigned short SigmaBuff[1200];															// Buffer to hold smoothed accelerometer values					
	static int pnt = 0;
	static int QuadAngles[2];
	static short Segs[1000];
	static short NonSegs[1000];
	static short SigmaBuff_2[50];															// Buffer to hold smoothed accelerometer values		
	static short MaxBuff[1000];
	static float sumSegs = 0;
	static float sumNonSegs = 0;
	static int pnt_2 = 0;
	static int pnt_3 = 0;
	//////////////////
	float max = -10000000;
	float min =  10000000;
	int j;
	float f;
	int s;
	int Maximum;
	int W, start, finish_1, finish_2;
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
	SigmaBuff[pnt] = Sigma;
	if (cadence_gyro == 0)
		W = 0;
	else
		W = (60*400/cadence_gyro);
	if (W > 1200)
		W = 1200;
	if (W < 100)
		W = 100;
	start = pnt - W;
	
	if (start < 0)
	{
		start += 1200;
		finish_1 = 1200;
		finish_2 = pnt;
	}
	else
	{
		finish_1 = pnt;
		finish_2 = 0;
	}
	pnt++;
	if (pnt == WIN_SIZE)
		pnt = 0;
	for (j=start; j < finish_1 ; j++)														// Shifting the values inside the buffer and finding the maximum and minimum 
	{
		if (max < SigmaBuff[j])
				max = SigmaBuff[j];
		if (min > SigmaBuff[j])
				min = SigmaBuff[j];
	}
	for (j=0; j < finish_2 ; j++)														// Shifting the values inside the buffer and finding the maximum and minimum 
	{
		if (max < SigmaBuff[j])
				max = SigmaBuff[j];
		if (min > SigmaBuff[j])
				min = SigmaBuff[j];
	}
	if (Sigma > (max-min)/DEVFACT + min)   																		// Classification of accelerometer values
	{
		*Class = 1;
	}
	else
	{
		*Class = 0;
	}
	//ShakeRemoval(Sigma, *Class, 0);
	/*max = -10000000;
	///Shake fix////
	if (*Class == 1)
	{
		sumSegs = sumSegs - Segs[pnt_2] + Sigma;
		Segs[pnt_2] = Sigma;
		
		sumNonSegs = sumNonSegs - NonSegs[pnt_2];
		NonSegs[pnt_2] = 0;
	}
	else
	{
		sumSegs = sumSegs - Segs[pnt_2];
		Segs[pnt_2] = 0;

		sumNonSegs = sumNonSegs - NonSegs[pnt_2] + Sigma;
		NonSegs[pnt_2] = Sigma;
	}

	SigmaBuff_2[pnt_3++] = Segs[pnt_2];
	
	if (pnt_3 == 50)
		pnt_3 = 0;
	for (j=0; j < 50 ; j++)														// Shifting the values inside the buffer and finding the maximum and minimum 
	{
		if (max < SigmaBuff_2[j])
				max = SigmaBuff_2[j];
	}
	MaxBuff[pnt_2] = max;
	s = pnt_2;
	pnt_2++;
	if (pnt_2 == 1000)
		pnt_2 = 0;
	Maximum = max;
	for (j = 0; j<20; j++)
	{
		if (MaxBuff[s] > Maximum)
			Maximum = MaxBuff[s];
		s -= 50;
		if (s<0)
			s += 1000;
	}
	shake << Maximum << std::endl;
	////////////////////////////////
	cadence_zeroer = 0;
	if (sumSegs == 0)
		cadence_zeroer = 1;
	else if (sumSegs/sumNonSegs < 3 && Maximum < 800)
		cadence_zeroer = 1;
	else
		cadence_zeroer = 0;
	*/
}

void UnsupervisedHam(unsigned short Sigma, int *Class)
{  
	static unsigned short SigmaBuff[1200];															// Buffer to hold smoothed accelerometer values					
	static int pnt = 0;
	static int QuadAngles[2];
	static short Segs[1000];
	static short NonSegs[1000];
	static short SigmaBuff_2[50];															// Buffer to hold smoothed accelerometer values		
	static short MaxBuff[1000];
	static float sumSegs = 0;
	static float sumNonSegs = 0;
	static int pnt_2 = 0;
	static int pnt_3 = 0;
	//////////////////
	float max = -10000000;
	float min =  10000000;
	int j;
	float f;
	int s;
	int Maximum;
	int W, start, finish_1, finish_2;
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
	SigmaBuff[pnt] = Sigma;
	if (cadence_gyro == 0)
		W = 0;
	else
		W = (60*400/cadence_gyro);
	if (W > 1200)
		W = 1200;
	if (W < 100)
		W = 100;
	start = pnt - W;
	if (start < 0)
	{
		start += 1200;
		finish_1 = 1200;
		finish_2 = pnt;
	}
	else
	{
		finish_1 = pnt;
		finish_2 = 0;
	}
	pnt++;
	if (pnt == WIN_SIZE)
		pnt = 0;
	for (j=start; j < finish_1 ; j++)														// Shifting the values inside the buffer and finding the maximum and minimum 
	{
		if (max < SigmaBuff[j])
				max = SigmaBuff[j];
		if (min > SigmaBuff[j])
				min = SigmaBuff[j];
	}
	for (j=0; j < finish_2 ; j++)														// Shifting the values inside the buffer and finding the maximum and minimum 
	{
		if (max < SigmaBuff[j])
				max = SigmaBuff[j];
		if (min > SigmaBuff[j])
				min = SigmaBuff[j];
	}
	if (Sigma > (max-min)/DEVFACT + min)   																		// Classification of accelerometer values
	{
		*Class = 1;
	}
	else
	{
		*Class = 0;
	}
}

void ShakeRemoval(float Sigma, int Class, int init)
{
	static short Segs[1000];
	static short NonSegs[1000];
	static short SigmaBuff_2[50];
	static short SigmaBuff_3[50];
	static short MaxBuff[1000];
	static short MinBuff[1000];
	static float sumSegs = 0;
	static float sumNonSegs = 0;
	static int pnt_2 = 0;
	static int pnt_3 = 0;
	static int counter = 0;
	int j, s;
	float f;
	//////////////////
	float max = -10000000;
	float min =  10000000;
	int Maximum, Minimum;
	if (init == 0)
	{
		max = -10000000;
		min =  10000000;
		///Shake fix////
		if (Class == 1)
		{
			counter++;
			if (Segs[pnt_2] != 0)
				counter--;
			sumSegs = sumSegs - Segs[pnt_2] + Sigma;
			Segs[pnt_2] = Sigma;
		
			sumNonSegs = sumNonSegs - NonSegs[pnt_2];
			NonSegs[pnt_2] = 0;
		}
		else
		{
			if (Segs[pnt_2] != 0)
				counter--;
			sumSegs = sumSegs - Segs[pnt_2];
			Segs[pnt_2] = 0;

			sumNonSegs = sumNonSegs - NonSegs[pnt_2] + Sigma;
			NonSegs[pnt_2] = Sigma;
		}
		
		SigmaBuff_2[pnt_3++] = Segs[pnt_2];
	    SigmaBuff_3[pnt_3++] = NonSegs[pnt_2];
		if (pnt_3 == 50)
			pnt_3 = 0;
		for (j=0; j < 50 ; j++)														// Shifting the values inside the buffer and finding the maximum and minimum 
		{
			if (max < SigmaBuff_2[j])
					max = SigmaBuff_2[j];
			if (min > SigmaBuff_3[j])
					min = SigmaBuff_3[j];
		}
		MaxBuff[pnt_2] = max;
		MinBuff[pnt_2] = min;
		s = pnt_2;
		pnt_2++;
		if (pnt_2 == 1000)
			pnt_2 = 0;
		Maximum = max;
		Minimum = min;
		for (j = 0; j<20; j++)
		{
			if (MaxBuff[s] > Maximum)
				Maximum = MaxBuff[s];
			if (MinBuff[s] < Minimum)
				Minimum = MinBuff[s];
			s -= 50;
			if (s<0)
				s += 1000;
		}
		cadence_zeroer = 0;
		if (Maximum - Minimum < 500)
			cadence_zeroer = 1;
		shake << (int)cadence_zeroer << std::endl;
	}
	else
	{
		for (j = 0; j < 1000; j++)
			Segs[j] = 0;
		counter = 0;
	}
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