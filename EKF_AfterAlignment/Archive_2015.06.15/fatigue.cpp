#include "algorithms.h"
#include <fstream>
extern std::ofstream f7;
#define FATIGUE_BUFFER 184
/* Hamming window */
const float Hamming[64] = {0.08,0.082286,0.089121,0.10044,0.11612,0.13602,0.15993,0.18762,0.21881,0.25319,0.29043,0.33014,0.37194,0.41541,0.46012,0.50562,0.55147,0.5972,0.64236,0.6865,0.72919,0.77,0.80852,0.84438,0.8772,0.90668,0.93251,0.95445,0.97226,0.98578,0.99486,0.99943,0.99943,0.99486,0.98578,0.97226,0.95445,0.93251,0.90668,0.8772,0.84438,0.80852,0.77,0.72919,0.6865,0.64236,0.5972,0.55147,0.50562,0.46012,0.41541,0.37194,0.33014,0.29043,0.25319,0.21881,0.18762,0.15993,0.13602,0.11612,0.10044,0.089121,0.082286,0.08
};
/* sinusoid look-up table */
const float Sin_LUT[501] = {0,0.0031416,0.0062831,0.0094246,0.012566,0.015707,0.018848,0.021989,0.02513,0.028271,0.031411,0.034551,0.03769,0.040829,0.043968,0.047106,0.050244,0.053382,0.056519,0.059655,0.062791,0.065926,0.06906,0.072194,0.075327,0.078459,0.081591,0.084721,0.087851,0.09098,0.094108,0.097235,0.10036,0.10349,0.10661,0.10973,0.11286,0.11598,0.1191,0.12222,0.12533,0.12845,0.13156,0.13468,0.13779,0.1409,0.14401,0.14712,0.15023,0.15333,0.15643,0.15954,0.16264,0.16574,0.16883,0.17193,0.17502,0.17812,0.18121,0.18429,0.18738,0.19047,0.19355,0.19663,0.19971,0.20279,0.20586,0.20894,0.21201,0.21508,0.21814,0.22121,0.22427,0.22733,0.23039,0.23345,0.2365,0.23955,0.2426,0.24565,0.24869,0.25173,0.25477,0.25781,0.26084,0.26387,0.2669,0.26993,0.27295,0.27597,0.27899,0.28201,0.28502,0.28803,0.29104,0.29404,0.29704,0.30004,0.30304,0.30603,0.30902,0.312,0.31499,0.31797,0.32094,0.32392,0.32689,0.32986,0.33282,0.33578,0.33874,0.34169,0.34464,0.34759,0.35053,0.35347,0.35641,0.35935,0.36228,0.3652,0.36812,0.37104,0.37396,0.37687,0.37978,0.38268,0.38558,0.38848,0.39137,0.39426,0.39715,0.40003,0.40291,0.40578,0.40865,0.41151,0.41438,0.41723,0.42009,0.42293,0.42578,0.42862,0.43146,0.43429,0.43712,0.43994,0.44276,0.44557,0.44838,0.45119,0.45399,0.45679,0.45958,0.46237,0.46515,0.46793,0.4707,0.47347,0.47624,0.479,0.48175,0.4845,0.48725,0.48999,0.49273,0.49546,0.49819,0.50091,0.50362,0.50633,0.50904,0.51174,0.51444,0.51713,0.51982,0.5225,0.52517,0.52785,0.53051,0.53317,0.53583,0.53848,0.54112,0.54376,0.54639,0.54902,0.55165,0.55426,0.55688,0.55948,0.56208,0.56468,0.56727,0.56985,0.57243,0.57501,0.57757,0.58013,0.58269,0.58524,0.58779,0.59032,0.59286,0.59538,0.5979,0.60042,0.60293,0.60543,0.60793,0.61042,0.61291,0.61539,0.61786,0.62033,0.62279,0.62524,0.62769,0.63013,0.63257,0.635,0.63742,0.63984,0.64225,0.64466,0.64706,0.64945,0.65183,0.65421,0.65659,0.65895,0.66131,0.66367,0.66601,0.66835,0.67069,0.67301,0.67533,0.67765,0.67995,0.68225,0.68455,0.68683,0.68911,0.69139,0.69365,0.69591,0.69817,0.70041,0.70265,0.70488,0.70711,0.70932,0.71154,0.71374,0.71594,0.71813,0.72031,0.72248,0.72465,0.72681,0.72897,0.73112,0.73326,0.73539,0.73751,0.73963,0.74174,0.74385,0.74594,0.74803,0.75011,0.75218,0.75425,0.75631,0.75836,0.76041,0.76244,0.76447,0.76649,0.76851,0.77051,0.77251,0.7745,0.77649,0.77846,0.78043,0.78239,0.78434,0.78629,0.78823,0.79016,0.79208,0.79399,0.7959,0.79779,0.79968,0.80157,0.80344,0.80531,0.80717,0.80902,0.81086,0.81269,0.81452,0.81634,0.81815,0.81995,0.82175,0.82353,0.82531,0.82708,0.82884,0.8306,0.83234,0.83408,0.83581,0.83753,0.83924,0.84094,0.84264,0.84433,0.84601,0.84768,0.84934,0.85099,0.85264,0.85428,0.85591,0.85753,0.85914,0.86074,0.86234,0.86392,0.8655,0.86707,0.86863,0.87018,0.87173,0.87326,0.87479,0.87631,0.87782,0.87932,0.88081,0.88229,0.88377,0.88523,0.88669,0.88814,0.88958,0.89101,0.89243,0.89384,0.89525,0.89664,0.89803,0.89941,0.90077,0.90213,0.90348,0.90483,0.90616,0.90748,0.9088,0.91011,0.9114,0.91269,0.91397,0.91524,0.9165,0.91775,0.919,0.92023,0.92146,0.92267,0.92388,0.92508,0.92627,0.92745,0.92862,0.92978,0.93093,0.93207,0.9332,0.93433,0.93544,0.93655,0.93765,0.93873,0.93981,0.94088,0.94194,0.94299,0.94403,0.94506,0.94609,0.9471,0.9481,0.9491,0.95008,0.95106,0.95202,0.95298,0.95393,0.95486,0.95579,0.95671,0.95762,0.95852,0.95941,0.96029,0.96117,0.96203,0.96288,0.96372,0.96456,0.96538,0.9662,0.967,0.9678,0.96858,0.96936,0.97013,0.97088,0.97163,0.97237,0.9731,0.97382,0.97453,0.97523,0.97592,0.9766,0.97727,0.97793,0.97858,0.97922,0.97986,0.98048,0.98109,0.98169,0.98229,0.98287,0.98345,0.98401,0.98456,0.98511,0.98564,0.98617,0.98669,0.98719,0.98769,0.98817,0.98865,0.98912,0.98958,0.99002,0.99046,0.99089,0.99131,0.99172,0.99211,0.9925,0.99288,0.99325,0.99361,0.99396,0.9943,0.99463,0.99495,0.99526,0.99556,0.99585,0.99613,0.9964,0.99667,0.99692,0.99716,0.99739,0.99761,0.99782,0.99803,0.99822,0.9984,0.99857,0.99874,0.99889,0.99903,0.99917,0.99929,0.9994,0.99951,0.9996,0.99968,0.99976,0.99982,0.99988,0.99992,0.99996,0.99998,1,1
};

float Sinus(float Input);
float Cosin(float Input);

float32_t EMG_Buffer_1[FATIGUE_BUFFER];														 // Quad Emg buffer to delay the it to take care of shifts for fatigue
float32_t EMG_Buffer_2[FATIGUE_BUFFER];														 // Ham Emg buffer to delay the it to take care of shifts for fatigue

#define nfft 40																							// Number of coefficients to calculate
#define SIZE 64																							// Window size to perform the pwelch algorithm on
#define overlap 32																					// Overlap size between the windows
#define PI	  (float)3.141592653589
#define average_size 30																			// Number of mean frequencies to average
#define center 15
#define const_val 15

/**
 * @brief	Performs pwelch power spectrum estimation and mean frequency calculation
 * @param newData	  : new filtered EMG data
 * @param mean_freq : pointer to the mean frequency
 * @param start   	: marks the start and stop of the calculation of the power spectrum and mean frequency
 * @return	nothing
 * @deteiled         : This function works as a state machine. when it is at reset state and start is 0, it does nothing. when start becomed 1 at reset state, 
 *                     calculation is started and it goes to the running state. In the running state calculation keeps running while start = 1. when start becomes 0,
 *                     calculation is stopped. Power spectrum is finalized, mean frequency is calculated and it goes back to reset state and waites for the next start.
 *                     Pwelch is done by calculatin a 256-fft on overlapping 64-sample wide windows. The amopunt of overlapping is 32 samples. Only 40 frequencies are 
 * 										 calculated according to the fatigue algorithms. execept for the first 32 samples, two overlapping windows are processed at the same time;
 * 										 To take care of the shift between filtered EMG and segmenatation, filtered EMG is stored in fifo first, and after a deley of 432 samples, fed to the fatigue algorithm
 * 										 EMGB_ptrx_1 and EMGB_ptrx_2 are pointers inside the buffer. The former is where we put the new EMG value, the latter is where we read the value to feed the fatigue algorithm
 * 										 They are incremented at every step, when they reach the end of the buffer, they go back to the start. EMGB_ptrx_2 is always one step ahead of EMGB_ptrx_1
 */	

void pWelch_quad(float newData, uint32_t* mean_freq, int start, int init)
{
	static int EMGB_ptr1_1 = 0;																																// The two buffer used to make a delay
	static int EMGB_ptr1_2 = 1;
	static int welch_state;																																		// Variable to hold the state
	static int mid_point;                                                                     // After averaging over a certain amount of mean frequencies, that values is preserved as point for scaling
	static int counter = 0;																																		// This countetr keeps track of how many windows were processed from start to finish and used for averaging
	static float buff[30];																																		// Buffer to hold mean frequencies for averaging
	static float sum; 																																				// Holds the sum of mean frequencies
	static uint32_t averager = 0;
	static int pnt = 0;
	static float coeffs_real_1[nfft];																													// Buffer to hold real part of the coefficients for the first FFT window
	static float coeffs_img_1[nfft];																													// Buffer to hold imaginary part of the coefficients for the first FFT window
	static float coeffs_real_2[nfft];																													// Buffer to hold real part of the coefficients for the second FFT window
	static float coeffs_img_2[nfft];																													// Buffer to hold real part of the coefficients for the second FFT window
	static float absolute_sum[nfft];																													// Buffer to hold the sum of absolute value of the coefficients 
	float p = 2*PI/256;
	static int c_1 = 0;																																				// Counter the indicates how many samples inside a 64-sample window has been received and processed so far
	static int c_2 = 0;																																			  // Counter is incremented with each new data, when it reaches 64, one window is complete, Counter is reset to zero and next window is started.
	static int flag = 0;
	float y;
	int i, ii;
	float df = 1.9380;																																				// 250 divided by 129 (256/2 + 1)
  float f, pow;
	float alaki, alaki2;
	float *p1, *p2;
	if (init == 0)                                                                            // Not initializing
	{
			EMG_Buffer_1[EMGB_ptr1_1] = newData;																			
			EMGB_ptr1_1++;
			EMGB_ptr1_2++;
			if (EMGB_ptr1_1 == FATIGUE_BUFFER-1)
				EMGB_ptr1_1 = 0;
			if (EMGB_ptr1_2 == FATIGUE_BUFFER-1)
					EMGB_ptr1_2 = 0;
			switch (welch_state)
			{
				case 0:																																							// reset state
					if (start == 1)																																		// if start becomes 1, go to running state (start of a segment)
						welch_state = 1;
					break;
				case 1:
					alaki = EMG_Buffer_1[EMGB_ptr1_2]*Hamming[c_1];
					alaki2 = p*c_1;
					p1 = coeffs_real_1;
				    p2 = coeffs_img_1;
					ii = 0;
					for (i = 0; i < nfft/4; i++)																												// with each incoming data, update all the coefficients
					{
								p1[0] +=  alaki*Cosin(alaki2*ii);	// Real part of the coefficient
								p2[0]  -= -alaki*Sinus(alaki2*ii);	// Imaginary part of the coefficient
								p1[1] +=  alaki*Cosin(alaki2*(ii+1));	// Real part of the coefficient
								p2[1]  -= -alaki*Sinus(alaki2*(ii+1));	// Imaginary part of the coefficient
								p1[2] +=  alaki*Cosin(alaki2*(ii+2));	// Real part of the coefficient
								p2[2]  -= -alaki*Sinus(alaki2*(ii+2));	// Imaginary part of the coefficient
								p1[3] +=  alaki*Cosin(alaki2*(ii+3));	// Real part of the coefficient
								p2[3]  -= -alaki*Sinus(alaki2*(ii+3));	// Imaginary part of the coefficient
								p1 += 4;
								p2 += 4;
								ii += 4;
					}
					c_1++;																																						// Increment the counter
					if (c_1 == overlap + 1)
						flag =  1;																																			// This means we have received 32 samples, it's time to start the concurrent calculation for overlaping windows
					if (c_1 == SIZE)																																	// we have reached the end of the window, coefficient calculation is complete, calculate absolute values and reset the counter
					{
						for (i = 0; i < nfft; i++)
						{
								absolute_sum[i] += coeffs_real_1[i]*coeffs_real_1[i] + coeffs_img_1[i]*coeffs_img_1[i]; // Finding the absoulte value for each coefficient
								coeffs_real_1[i] = 0;																												// Setting to zero for the next window
								coeffs_img_1[i] = 0;																												// Setting to zero for the next window
						}
						c_1 = 0;
						counter++;                                                                      // This countetr keeps track of how many windows were processed from start to finish and used for averaging
					}
					if (flag == 1) 																																		// Starting concurrent calculation for overlaping windows
					{
						alaki = EMG_Buffer_1[EMGB_ptr1_2]*Hamming[c_2];
						alaki2 = p*c_2;
						p1 = coeffs_real_2;
						p2 = coeffs_img_2;
						ii = 0;
						for (i = 0; i < nfft/4; i++)																												// with each incoming data, update all the coefficients
						{
									p1[0] +=  alaki*Cosin(alaki2*ii);	// Real part of the coefficient
									p2[0]  -= -alaki*Sinus(alaki2*ii);	// Imaginary part of the coefficient
									p1[1] +=  alaki*Cosin(alaki2*(ii+1));	// Real part of the coefficient
									p2[1]  -= -alaki*Sinus(alaki2*(ii+1));	// Imaginary part of the coefficient
									p1[2] +=  alaki*Cosin(alaki2*(ii+2));	// Real part of the coefficient
									p2[2]  -= -alaki*Sinus(alaki2*(ii+2));	// Imaginary part of the coefficient
									p1[3] +=  alaki*Cosin(alaki2*(ii+3));	// Real part of the coefficient
									p2[3]  -= -alaki*Sinus(alaki2*(ii+3));	// Imaginary part of the coefficient
							    p1 += 4;
									p2 += 4;
									ii += 4;
						}
						c_2++;																																					// Increment the counter
						if (c_2 == SIZE)
						{
							for (i = 0; i < nfft; i++)
							{
								absolute_sum[i] += coeffs_real_2[i]*coeffs_real_2[i] + coeffs_img_2[i]*coeffs_img_2[i];  // Finding the absoulte value for each coefficient
								coeffs_real_2[i] = 0;																												// Setting to zero for the next window
								coeffs_img_2[i] = 0;																												// Setting to zero for the next window
							}
							c_2 = 0;
							counter++;
						}
					}
					if (start == 0)																																		// End of segment, average the power spectrums, calculate mean frequency and reset the state machine parameters
					{
						y = 0;
						f = 1;
						pow = 0;
						for (i = 0; i < nfft; i++)                                                      // for each coefficient, add the final two absolute values to the sum and find the average
						{
							c_1 = 0;
							c_2 = 0;
							flag = 0;
							absolute_sum[i] += coeffs_real_1[i]*coeffs_real_1[i] + coeffs_img_1[i]*coeffs_img_1[i];
							absolute_sum[i] += coeffs_real_2[i]*coeffs_real_2[i] + coeffs_img_2[i]*coeffs_img_2[i];
							absolute_sum[i] /= (counter + 2);
							pow += absolute_sum[i];
							y += absolute_sum[i] * f;
							f += df;
							absolute_sum[i] = 0;																													// Reseting the buffers
							coeffs_real_1[i] = 0;
							coeffs_img_1[i] = 0;
							coeffs_real_2[i] = 0;
							coeffs_img_2[i] = 0;
						}
						sum = sum - buff[pnt] +  y / pow;																								// A moving average window of size 30 over the calculated mean frequencies
						buff[pnt] = y / pow;
						//*mean_freq = y / pow;																													// For debug
						pnt++;
						if (pnt == average_size)																												// Reseting the pointer inside the buffer used for moving average
							pnt = 0;
						averager++;
						if (averager < average_size)																										// If less than average_size number of mean frequencies have been calculated so far, send 255 over bluetooth
							*mean_freq = const_val;
						else if (averager == average_size)																							// If 30 mean frequencies have been calculated, calculate their average
							 mid_point = sum / average_size;																							// This average is used as a point of compariosn fron bow on
						else
							*mean_freq = (sum / average_size) - mid_point + center;                       // Center the averaged mean frequencies around 15, based on the mid_point value calculated above
						welch_state = 0;		
						f7 << y / pow << std::endl;// Go to reset state
					}
					break;
			}
	}
	else																																											// Initializing
	{
		welch_state = 0;
		for (i = 0; i < nfft; i++)																															// Setting the buffers and pararmeters to zero
		{
			coeffs_real_1[i] = 0;
			coeffs_img_1[i] = 0;
			coeffs_real_2[i] = 0;
			coeffs_img_2[i] = 0;
			absolute_sum[i] = 0;
		}
		sum = 0;
		averager = 0;
		pnt = 0;
		for (i = 0; i < 30; i++)
		{
			buff[i] = 0;
		}
		
	}
}

/**
 * @brief	Exactly the same as pWelch_quad, just using different buffers. Performs pwelch power spectrum estimation and mean frequency calculation
 * @param newData	  : new filtered EMG data
 * @param mean_freq : pointer to the mean frequency
 * @param start   	: marks the start and stop of the calculation of the power spectrum and mean frequency
 * @return	nothing
 * @deteiled         : This function works as a state machine. when it is at reset state and start is 0, it does nothing. when start becomed 1 at reset state, 
 *                     calculation is started and it goes to the running state. In the running state calculation keeps running while start = 1. when start becomes 0,
 *                     calculation is stopped. Power spectrum is finalized, mean frequency is calculated and it goes back to reset state and waites for the next start.
 *                     Pwelch is done by calculatin a 256-fft on overlapping 64-sample wide windows. The amopunt of overlapping is 32 samples. Only 40 frequencies are 
 * 										 calculated according to the fatigue algorithms. execept for the first 32 samples, two overlapping windows are processed at the same time;
 * 										 To take care of the shift between filtered EMG and segmenatation, filtered EMG is stored in fifo first, and after a deley of 432 samples, fed to the fatigue algorithm
 * 										 EMGB_ptrx_1 and EMGB_ptrx_2 are pointers inside the buffer. The former is where we put the new EMG value, the latter is where we read the value to feed the fatigue algorithm
 * 										 They are incremented at every step, when they reach the end of the buffer, they go back to the start. EMGB_ptrx_2 is always one step ahead of EMGB_ptrx_1
 */	
void pWelch_ham(float newData, uint32_t* mean_freq, int start, int init)
{
	static int EMGB_ptr1_1 = 0;
	static int EMGB_ptr1_2 = 1;
	static int welch_state;																																		// Variable to hold the state
	static int mid_point;                                                                     // After averaging over a certain amount of mean frequencies, that values is preserved as point for scaling
	static int counter = 0;																																		// This countetr keeps track of how many windows were processed from start to finish and used for averaging
	static float buff[30];																																		// Buffer to hold mean frequencies for averaging
	static float sum; 																																				// Holds the sum of mean frequencies
	static uint32_t averager = 0;
	static int pnt = 0;
	static float coeffs_real_1[nfft];																													// Buffer to hold real part of the coefficients for the first FFT window
	static float coeffs_img_1[nfft];																													// Buffer to hold imaginary part of the coefficients for the first FFT window
	static float coeffs_real_2[nfft];																													// Buffer to hold real part of the coefficients for the second FFT window
	static float coeffs_img_2[nfft];																													// Buffer to hold real part of the coefficients for the second FFT window
	static float absolute_sum[nfft];																													// Buffer to hold the sum of absolute value of the coefficients 
	float p = 2*PI/256;
	static int c_1 = 0;																																				// Counter the indicates how many samples inside a 64-sample window has been received and processed so far
	static int c_2 = 0;																																			  // Counter is incremented with each new data, when it reaches 64, one window is complete, Counter is reset to zero and next window is started.
	static int flag = 0;
	float y;
	int i;
	float df = 1.9380;																																				// 250 divided by 129 (256/2 + 1)
  float f, pow;
	if (init == 0)                                                                            // Not initializing
	{
			EMG_Buffer_2[EMGB_ptr1_1] = newData;																			
			EMGB_ptr1_1++;
			EMGB_ptr1_2++;
			if (EMGB_ptr1_1 == FATIGUE_BUFFER-1)
				EMGB_ptr1_1 = 0;
			if (EMGB_ptr1_2 == FATIGUE_BUFFER-1)
				EMGB_ptr1_2 = 0;
			switch (welch_state)
			{
				case 0:																																							// reset state
					if (start == 1)																																		// if start becomes 1, go to running state (start of a segment)
						welch_state = 1;
					break;
				case 1:
					for (i = 0; i < nfft; i++)																												// with each incoming data, update all the coefficients
					{
								coeffs_real_1[i] +=  EMG_Buffer_1[EMGB_ptr1_2]*Hamming[c_1]*Cosin(p*i*c_1);	// Real part of the coefficient
								coeffs_img_1[i]  -= -EMG_Buffer_1[EMGB_ptr1_2]*Hamming[c_1]*Sinus(p*i*c_1);	// Imaginary part of the coefficient
					}
					c_1++;																																						// Increment the counter
					if (c_1 == overlap + 1)
						flag =  1;																																			// This means we have received 32 samples, it's time to start the concurrent calculation for overlaping windows
					if (c_1 == SIZE)																																	// we have reached the end of the window, coefficient calculation is complete, calculate absolute values and reset the counter
					{
						for (i = 0; i < nfft; i++)
						{
								absolute_sum[i] += coeffs_real_1[i]*coeffs_real_1[i] + coeffs_img_1[i]*coeffs_img_1[i]; // Finding the absoulte value for each coefficient
								coeffs_real_1[i] = 0;																												// Setting to zero for the next window
								coeffs_img_1[i] = 0;																												// Setting to zero for the next window
						}
						c_1 = 0;
						counter++;                                                                      // This countetr keeps track of how many windows were processed from start to finish and used for averaging
					}
					if (flag == 1) 																																		// Starting concurrent calculation for overlaping windows
					{
						for (i = 0; i < nfft; i++)
						{
							coeffs_real_2[i] +=  EMG_Buffer_1[EMGB_ptr1_2]*Hamming[c_2]*Cosin(p*i*c_2);		// Real part of the coefficient
							coeffs_img_2[i]  -= -EMG_Buffer_1[EMGB_ptr1_2]*Hamming[c_2]*Sinus(p*i*c_2);		// Imaginary part of the coefficient
						}
						c_2++;																																					// Increment the counter
						if (c_2 == SIZE)
						{
							for (i = 0; i < nfft; i++)
							{
								absolute_sum[i] += coeffs_real_2[i]*coeffs_real_2[i] + coeffs_img_2[i]*coeffs_img_2[i];  // Finding the absoulte value for each coefficient
								coeffs_real_2[i] = 0;																												// Setting to zero for the next window
								coeffs_img_2[i] = 0;																												// Setting to zero for the next window
							}
							c_2 = 0;
							counter++;
						}
					}
					if (start == 0)																																		// End of segment, average the power spectrums, calculate mean frequency and reset the state machine parameters
					{
						y = 0;
						f = 1;
						pow = 0;
						for (i = 0; i < nfft; i++)                                                      // for each coefficient, add the final two absolute values to the sum and find the average
						{
							c_1 = 0;
							c_2 = 0;
							flag = 0;
							absolute_sum[i] += coeffs_real_1[i]*coeffs_real_1[i] + coeffs_img_1[i]*coeffs_img_1[i];
							absolute_sum[i] += coeffs_real_2[i]*coeffs_real_2[i] + coeffs_img_2[i]*coeffs_img_2[i];
							absolute_sum[i] /= (counter + 2);
							pow += absolute_sum[i];
							y += absolute_sum[i] * f;
							f += df;
							absolute_sum[i] = 0;																													// Reseting the buffers
							coeffs_real_1[i] = 0;
							coeffs_img_1[i] = 0;
							coeffs_real_2[i] = 0;
							coeffs_img_2[i] = 0;
						}
						sum = sum - buff[pnt] +  y / pow;																								// A moving average window of size 30 over the calculated mean frequencies
						buff[pnt] = y / pow;
						//*mean_freq = y / pow;																													// For debug
						pnt++;
						if (pnt == average_size)																												// Reseting the pointer inside the buffer used for moving average
							pnt = 0;
						averager++;
						if (averager < average_size)																										// If less than average_size number of mean frequencies have been calculated so far, send 255 over bluetooth
							*mean_freq = const_val;
						else if (averager == average_size)																							// If 30 mean frequencies have been calculated, calculate their average
							 mid_point = sum / average_size;																							// This average is used as a point of compariosn fron bow on
						else
							*mean_freq = (sum / average_size) - mid_point + center;                        // Center the averaged mean frequencies around 15, based on the mid_point value calculated above
						welch_state = 0;																																// Go to reset state
					}
					break;
			}
	}
	else																																											// Initializing
	{
		welch_state = 0;
		for (i = 0; i < nfft; i++)																															// Setting the buffers and pararmeters to zero
		{
			coeffs_real_1[i] = 0;
			coeffs_img_1[i] = 0;
			coeffs_real_2[i] = 0;
			coeffs_img_2[i] = 0;
			absolute_sum[i] = 0;
		}
		sum = 0;
		averager = 0;
		pnt = 0;
		for (i = 0; i < 30; i++)
		{
			buff[i] = 0;
		}
		
	}
}



/**
 * @brief	finds the sin value of an input in degrees
 */	
float Sinus(float Input)
{
	int r;
	float reminder;
	r = (Input/(2*PI));
	reminder = Input - (float)r*2*PI;
	if (reminder <= PI/2)
	{
		r = (reminder/PI)*1000;
		return Sin_LUT[r];
	}
	else if (reminder <= PI)
	{
		reminder -= PI/2;
		r = (reminder/PI)*1000;
		return Sin_LUT[500-r];
	}
	else if (reminder <= (float)1.5*PI)
	{
		reminder -= PI;
		r = (reminder/PI)*1000;
		return -1*Sin_LUT[r];
	}
	else
	{
		reminder -= (float)1.5*PI;
		r = (reminder/PI)*1000;
		return -1*Sin_LUT[500-r];
	}
}

/**
 * @brief	finds the cos value of an input in degrees
 */
float Cosin(float Input)
{
	int r;
	float reminder;
	r = (Input/(2*PI));
	reminder = Input - (float)r*2*PI;
	if (reminder <= PI/2)
	{
		r = (reminder/PI)*1000;
		return Sin_LUT[500-r];
	}
	else if (reminder <= PI)
	{
		reminder -= PI/2;
		r = (reminder/PI)*1000;
		return -1*Sin_LUT[r];
	}
	else if (reminder <= (float)1.5*PI)
	{
		reminder -= PI;
		r = (reminder/PI)*1000;
		return -1*Sin_LUT[500-r];
	}
	else
	{
		reminder -= (float)1.5*PI;
		r = (reminder/PI)*1000;
		return Sin_LUT[r];
	}
}
