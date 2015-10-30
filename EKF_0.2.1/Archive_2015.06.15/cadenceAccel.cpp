#include "algorithms.h"
#include <math.h>
#include <fstream>
extern std::ofstream f1, f2, f3, f4, f5, f6, ftest, X, velseg, Y;

#define THRESH     2500
#define Accel_size 125																					// Size of the classification window
#define SMOOTH_SIZE 25																					// Size of the sliding window size used for smoothing
extern uint8_t cadence_acc;
extern uint8_t cadence_zeroer;
void PostBuffer_acc_1(int in, int* out);												// post processing to remove small segments
void PostBuffer_acc_2(int in, int* out);												// post processing to remove small gaps
void Peakdetection_acc(int in, long long pos, int* out);
void Smooth_1(int32_t input, float32_t *out, int init);
void Smooth_2(int32_t input, float32_t *out, int init);
int SegmentCad;
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

void cadenceAccel(uint8_t* rawData, uint8_t *Results, int init)
{  
	static float smoothedAccBuff[SMOOTH_SIZE];															// Buffer to hold smoothed accelerometer values
	static float sum_low = 0;
	static int ptr3 = 0;
	static int ptr4 = 0;
	static int ptr5;
	float low;
	static float HighPassAccBuff[Accel_size];	
	static long long sum_high = 0;
	static int ptr1 = 0;
	static int ptr2 = (Accel_size - 1)/2 + 1;
	static int count = 0;
	float high, high2;
	static long long cumsum = 0;
	static long long cumsum2 = 0;
	static float MaxMinBuff[Accel_size];	
	static long long DelayBuffer[13];
	//////////////////
//	float Sigma;
	float max = -10000000;
	float min =  10000000;
	RAW_DATA raw_x;
	int Class;
  int j, z;
  static int cadence;
	if (init == 0)																									// If init = 0, run the algorithms
	{
			raw_x.raw_data = 0;
			raw_x.data_byte[0] = rawData[0];															// Converting the 2-byte, big-endian accelerometer value to 2-byte small-endian
			raw_x.data_byte[1] = rawData[1];
			if ((rawData[1] & 0x80) == 0x00)
			{
				raw_x.data_byte[2] = 0;
				raw_x.data_byte[3] = 0;
			}
			else
			{
				raw_x.data_byte[2] = 0xFF;
				raw_x.data_byte[3] = 0xFF;
			}
			/////////////High Pass
			cumsum += raw_x.raw_data;
			sum_high = sum_high - HighPassAccBuff[ptr1] + cumsum;											// Oldest value is subtracted from the sum and new value is added
			HighPassAccBuff[ptr1] = cumsum;																		// new value replaes the oldest value in the buffer
			if (count < Accel_size-1)
				high = 0;
			else
				high = (float)HighPassAccBuff[ptr2] - (float)(sum_high)/Accel_size; // The average of the window is subtracted from the middle point of the window
			(ptr1)++;																							// Increment the pointers to implement the sliding effect and set them to zero if they reach the end of the buffer
			(ptr2)++;
			if ( (ptr1) == Accel_size )																
				ptr1 = 0;
			if ( (ptr2) == Accel_size )
				ptr2 = 0;
			(count)++;
			cumsum2 += high;
			//cumsum += high;
			/////////////Low pass
			sum_low = sum_low - smoothedAccBuff[ptr3] + high;
			smoothedAccBuff[ptr3] = high;
			low = (float)(sum_low)/SMOOTH_SIZE;
			DelayBuffer[ptr4] = cumsum2;
			ptr5 = ptr4 + 1;
			if (ptr5 == 13)
				ptr5 = 0;
			ptr4++;
			if (ptr4 == 13)
				ptr4 = 0;
			(ptr3)++;
			if (ptr3 == SMOOTH_SIZE)
				ptr3 = 0;
			//X << low << std::endl;
			//Y << DelayBuffer[ptr5] << std::endl;
			///////////////////////
			
			for (j=0; j < Accel_size-1 ; j++)														// Shifting the values inside the buffer and finding the maximum and minimum 
			{
				MaxMinBuff[j] = MaxMinBuff[j+1];
				if (max < MaxMinBuff[j])
						max = MaxMinBuff[j];
				if (min > MaxMinBuff[j])
						min = MaxMinBuff[j];
			}
			MaxMinBuff[Accel_size-1] = low;	
			if (max < MaxMinBuff[Accel_size-1])
						max = MaxMinBuff[Accel_size-1];
			if (min > MaxMinBuff[Accel_size-1])
						min = MaxMinBuff[Accel_size-1];
			if (low > (max-min)/3 + min && max - min > THRESH)   																		// Classification of accelerometer values
			{
				Class = 1;
			}
			else
			{
				Class = 0;
			}
			SegmentCad = Class;
			velseg << Class << std::endl;
			//f1 << Class << std::endl;
			//f4 << low << std::endl;
			//PostBuffer_acc_1(Class, &dummy1);														// First round of post-processing. Segments that have a length smaller than a threshod are eliminated
			//PostBuffer_acc_2(dummy1, &dummy2);													// Second round of post-processing. Gaps between two segments that have a length smaller than a threshod are eliminated
			Peakdetection_acc(Class, DelayBuffer[ptr5], &cadence);
			if ( max - min < THRESH || cadence_zeroer == 1)																			// By using this threshold, we get rid of small movments
			{
				cadence = 0;
			}
			//f4 << cadence << std::endl;
			cadence_acc = cadence;
			//ftest << (int)cadence_acc << std::endl;
			z = 0;
			//memcpy(Results, &z, 1);
			//memcpy((Results+1), &cadence_acc, 1);																	// putting cadence value in output (bluetooth) buffer
	}
	else																														// Initializing
	{		
		ptr1 = 0;
	    ptr2 = (Accel_size - 1)/2 + 1;
		ptr3 = 0;
		ptr4 = 0;
		sum_low = 0;
		sum_high = 0;
		count = 0;
		cumsum = 0;
		cumsum2 = 0;
		for (j = 0; j < Accel_size; j++)
		{
			HighPassAccBuff[j] = 0;
		}
	}
	
}

/*void cadenceAccel(uint8_t* rawData, uint8_t *Results, int init)
{  
	static float smoothedAccBuff[250];															// Buffer to hold smoothed accelerometer values
	static float s_1 = 0;																						
	static float s_2 = 0;
	static float s_3 = 0;
	static float s_4 = 0;
	static float s_5 = 0;
	////////Filtetring stuff
	static float dummy_1[SMOOTH_SIZE];
	static float dummy_2[SMOOTH_SIZE];
	static float dummy_3[SMOOTH_SIZE];
	static float dummy_4[SMOOTH_SIZE];
	static float dummy_5[SMOOTH_SIZE];
	static int ptr_1 = 0;
	static int ptr_2 = 0;
	static int ptr_3 = 0;
	static int ptr_4 = 0;
	static int ptr_5 = 0;
	static float sum_1 = 0;
	static float sum_2 = 0;
	static float sum_3 = 0;
	static float sum_4 = 0;
	static float sum_5 = 0;
	static int count_1 = 0;
	static int count_2 = 0;
	static int count_3 = 0;
	static int count_4 = 0;
	static int count_5 = 0;
	//////////////////
	float det;
	float max = -10000000;
	float min =  10000000;
	RAW_DATA raw_x, raw_y, raw_z;
	int Class;
  int dummy1, dummy2,j;
  static int cadence;
	float f;
	if (init == 0)																									// If init = 0, run the algorithms
	{
			raw_x.raw_data = 0;
			raw_x.data_byte[0] = rawData[0];															// Converting the 2-byte, big-endian accelerometer value to 2-byte small-endian
			raw_x.data_byte[1] = rawData[1];
			if ((rawData[1] & 0x80) == 0x00)
			{
				raw_x.data_byte[2] = 0;
				raw_x.data_byte[3] = 0;
			}
			else
			{
				raw_x.data_byte[2] = 0xFF;
				raw_x.data_byte[3] = 0xFF;
			}
			
			//raw_y.raw_data = 0;
			//raw_y.data_byte[0] = rawData[2];															// Converting the 2-byte, big-endian accelerometer value to 2-byte small-endian
			//raw_y.data_byte[1] = rawData[3];
			//if ((rawData[3] & 0x80) == 0x00)
			//{
			//	raw_y.data_byte[2] = 0;
			//	raw_y.data_byte[3] = 0;
			//}
			//else
			//{
			//	raw_y.data_byte[2] = 0xFF;
			//	raw_y.data_byte[3] = 0xFF;
			//}
			//
			//raw_z.raw_data = 0;
			//raw_z.data_byte[0] = rawData[4];															// Converting the 2-byte, big-endian accelerometer value to 2-byte small-endian
			//raw_z.data_byte[1] = rawData[5];
			//if ((rawData[5] & 0x80) == 0x00)
			//{
			//	raw_z.data_byte[2] = 0;
			//	raw_z.data_byte[3] = 0;
			//}
			//else
			//{
			//	raw_z.data_byte[2] = 0xFF;
			//	raw_z.data_byte[3] = 0xFF;
			//}
			
			/////////////Filtering
			 // f = raw_x.raw_data*raw_x.raw_data+ raw_y.raw_data*raw_y.raw_data + raw_z.raw_data*raw_z.raw_data;
			 // f = sqrt(f);
				//sum_1 = sum_1 - dummy_1[ptr_1] + f;
				//dummy_1[ptr_1] = f;
				//if (count_1 < SMOOTH_SIZE-1)
				//	s_1 = 0;
				//else
				//	s_1 = (float)(sum_1)/SMOOTH_SIZE;
				//(ptr_1)++;
				//if (ptr_1 == SMOOTH_SIZE)
				//	ptr_1 = 0;
				//(count_1)++;
				/////
				//sum_2 = sum_2 - dummy_2[ptr_2] + s_1;
				//dummy_2[ptr_2] = s_1;
				//if (count_2 < SMOOTH_SIZE-1)
				//	s_2 = 0;
				//else
				//	s_2 = (float)(sum_2)/SMOOTH_SIZE;
				//(ptr_2)++;
				//if (ptr_2 == SMOOTH_SIZE)
				//	ptr_2 = 0;
				//(count_2)++;
				/////
				//sum_3 = sum_3 - dummy_3[ptr_3] + s_2;
				//dummy_3[ptr_3] = s_2;
				//if (count_3 < SMOOTH_SIZE-1)
				//	s_3 = 0;
				//else
				//	s_3 = (float)(sum_3)/SMOOTH_SIZE;
				//(ptr_3)++;
				//if (ptr_3 == SMOOTH_SIZE)
				//	ptr_3 = 0;
				//(count_3)++;
				/////
				//sum_4 = sum_4 - dummy_4[ptr_4] + s_3;
				//dummy_4[ptr_4] = s_3;
				//if (count_4 < SMOOTH_SIZE-1)
				//	s_4 = 0;
				//else
				//	s_4 = (float)(sum_4)/SMOOTH_SIZE;
				//(ptr_4)++;
				//if (ptr_4 == SMOOTH_SIZE)
				//	ptr_4 = 0;
				//(count_4)++;
				/////
				//sum_5 = sum_5 - dummy_5[ptr_5] + s_4;
				//dummy_5[ptr_5] = s_4;
				//if (count_5 < SMOOTH_SIZE-1)
				//	s_5 = 0;
				//else
				//	s_5 = (float)(sum_5)/SMOOTH_SIZE;
				//(ptr_5)++;
				//if (ptr_5 == SMOOTH_SIZE)
				//	ptr_5 = 0;
				//(count_5)++;
				//////
				//Detrending(&s_5, &det, 60, 0, 3);
			EKF_acc(raw_x.raw_data, &det, 0);
			///////////////////////
			
			for (j=0; j < Accel_size-1 ; j++)														// Shifting the values inside the buffer and finding the maximum and minimum 
			{
				smoothedAccBuff[j] = smoothedAccBuff[j+1];
				if (max < smoothedAccBuff[j])
						max = smoothedAccBuff[j];
				if (min > smoothedAccBuff[j])
						min = smoothedAccBuff[j];
			}
			smoothedAccBuff[Accel_size-1] = det;												// Adding new smoothed accelerometer to the buffer
			if (det > (max+min)/2)   																		// Classification of accelerometer values
			{
				Class = 1;
			}
			else
			{
				Class = 0;
			}
			PostBuffer_acc_1(Class, &dummy1);														// First round of post-processing. Segments that have a length smaller than a threshod are eliminated
			PostBuffer_acc_2(dummy1, &dummy2);													// Second round of post-processing. Gaps between two segments that have a length smaller than a threshod are eliminated
			Peakdetection_acc(dummy2, &cadence);
			if (max - min < THRESH)																			// By using this threshold, we get rid of small movments
			{
				cadence = 0;
			}
			f1  << Class << std::endl;
			f2 << det << std::endl;
			cadence_acc = cadence;
			f6 << cadence << std::endl;
			//memcpy((Results), &cadence_acc, 1);																	// putting cadence value in output (bluetooth) buffer
	}
	else																														// Initializing
	{		
			for (j = 0; j<SMOOTH_SIZE; j++)
			{
				dummy_1[j] = 0;
				dummy_2[j] = 0;
				dummy_3[j] = 0;
				dummy_4[j] = 0;
				dummy_5[j] = 0;
			}
				sum_1 = 0;
				sum_2 = 0;
			  sum_3 = 0;
			  sum_4 = 0;
			  sum_5 = 0;
				ptr_1 = 0;
				ptr_2 = 0;
				ptr_3 = 0;
				ptr_4 = 0;
				ptr_5 = 0;
	}
	
}*/

/**
 * @brief	 Calculating cadence based on segmentation data, for real time purposes data is fed one at a time to the algorithm
 * @param  in	         : segmentation data
 * @param	 out         : cadence value
 * @detail             : Distance betweebn to consecutive segments is calculated. (From start of segment to start of the next segment). Cadence is the average over
 *                     : over two of such distatnces. The function works as a state machine. It has two states. state 0 and state 1. When it's at 0 state and no segment has been detected, nothing happens.
 *                     : Otherwise if the input is 1, it means the beggining of a segmentation and it changes state to state 1 and starts counting. In this state a counter increments everytime with each 
 *                     : input with value 1 to keep track of the distance. If the input goes to zero in this state, it means the segment has ended. We go back to state 0. Now, since a segment has been detected, 
 *                     : state 0 acts differenty. It keeps incrementing the counter until input is 1, which means the start of another segment. Value of the counter is stored in the buffer as the distance. Counter is reset
 *										 : and the process repeats by going to state 1.
 */	

void Peakdetection_acc_middletomiddle(int in, int* out)
{
	static int state = 0;																					// State machine
	static int counter = 0;																				// Counter to keep track of distance between segments
	static int dists[2] = {0,0};																	// Buffer to hold distances for averaging. Everytime a new distance is calculated, oldest one is tossed out and new one replaces it
	static int pointer = 0;																				
	static int peak_counter = 0;                                  // Keeps track of the numbner of segments detected so far.
	static int stop_counter = 0;																	// Counter to keep track of time no segment has been detected
	static int stop_counter_enable = 0;                           // Enables and disable the above counter
	int i;
	static int global_acc_counter = 0;
	static int prev_peak = 0;
	static int current_peak = 0;
	static int start = 0;
	global_acc_counter++;
	float f;
	if (stop_counter == 375)                                      // If no segment has been detected for the pase 250 samples (2 seconds), reset all buffers and variables
	{
		*out = 0;
		state = 0;
		counter = 0;
		pointer = 0;
		peak_counter = 0;
		stop_counter = 0;
		stop_counter_enable = 0;
		EKF_quad(0, &f, &f, 1);
		EKF_ham(0, &f, &f, 1);
		for (i = 0; i < 2; i++)
		{
			dists[i] = 0;
		}
	}
	if (stop_counter_enable == 1)																	
				stop_counter++;
	switch(state)																									// State machine
	{
		case 0:																											// State 0
			if (in == 1)																							// A new segment has been detected, time to put the new distance in the buffer and calculate cadence
			{
				start = global_acc_counter;
				state = 1;
			}
			break;
		case 1:																							// As long as the input is 1, keep incrementing the counter
			if (in == 0)	
			{
				current_peak = (start + global_acc_counter) >> 1;
				if (peak_counter > 0)																		// Calculate cadence if more than 1 segment have been detected so far
				{
					stop_counter = 0;																			// Reset the stop_counter to zero
					stop_counter_enable = 1;                              // Reset counter only should start counting when at least two segments have been detected
					dists[pointer] = current_peak - prev_peak;															// Putting the current distance in the buffer
					pointer++;
					if (pointer == 2)
						pointer = 0;
					if (peak_counter < 2)																	// If only one distance has been detected so far, output it as cadence
					{
						if (dists[0] != 0)
						{
							*out = 7500  / (dists[0]);
							//if (cadence_zeroer == 0)
								f4 << *out << std::endl;
							//else
							//	f4 << 0 << std::endl;
						}
						//f6 << *out << std::endl;
					}
					else																									// If more than one distances have been detected, start averaging over the last two each time
					{
						if ((dists[0] + dists[1]) != 0)
						{
							*out = 7500 * 2 / (dists[0] + dists[1]);
							//if (cadence_zeroer == 0)
								f4 << *out << std::endl;
							//else
							//	f4 << 0 << std::endl;
						}
						//f6 << *out << std::endl;
					}
				}			
				prev_peak = current_peak;// Set the counter back to zero
				peak_counter++;
				state = 0;
			}
			break;
	}
}

void Peakdetection_acc(int in, long long pos, int* out)
{
	static int state = 0;																					// State machine
	static int counter = 0;																				// Counter to keep track of distance between segments
	static int dists[2] = {0,0};																	// Buffer to hold distances for averaging. Everytime a new distance is calculated, oldest one is tossed out and new one replaces it
	static int pointer = 0;																				
	static int peak_counter = 0;                                  // Keeps track of the numbner of segments detected so far.
	static int stop_counter = 0;																	// Counter to keep track of time no segment has been detected
	static int stop_counter_enable = 0;                           // Enables and disable the above counter
	int i;
	static long long max , min;
	float f;

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
		EKF_quad(0, &f, &f, 1);
		EKF_ham(0, &f, &f, 1);
	}
	if (stop_counter_enable == 1)																	
				stop_counter++;
	switch(state)																									// State machine
	{
		case 0:																											// State 0
			counter++;
			if (in == 1)																							// A new segment has been detected, time to put the new distance in the buffer and calculate cadence
			{
				state = 1;
				if (peak_counter > 0)																		// Calculate cadence if more than 1 segment have been detected so far
				{
					stop_counter = 0;																			// Reset the stop_counter to zero
					stop_counter_enable = 1;                              // Reset counter only should start counting when at least two segments have been detected
					dists[pointer] = counter;															// Putting the current distance in the buffer
					pointer++;
					if (pointer == 2)
						pointer = 0;
					if (peak_counter < 2)																	// If only one distance has been detected so far, output it as cadence
					{
						if (dists[0] != 0)
						{
							*out = 7500  / (dists[0]);
							if (max - min > 2.5e5)
								f4 << *out << std::endl;
							else
								f4 << 0 << std::endl;
						}
					}
					else																									// If more than one distances have been detected, start averaging over the last two each time
					{
						if ((dists[0] + dists[1]) != 0)
						{
							*out = 7500 * 2 / (dists[0] + dists[1]);
							if (max - min > 2.5e5)
								f4 << *out << std::endl;
							else
								f4 << 0 << std::endl;
						}
					}
				}
				counter = 0;		
				max = -1e60;
				min = +1e60;// Set the counter back to zero
				peak_counter++;
			}
			break;
		case 1:
			counter++;																								// As long as the input is 1, keep incrementing the counter
			if (pos > max)
				max = pos;
			if (pos < min)
				min = pos;
			if (in == 0)		
			{																													// Segment has ended, go back to state 0
				state = 0;
			}
			break;
	}
}

/**
 * @brief	Post processing operation to get rid of segments samller than a certain size, for real-time practice, input data is fed one at a time.
 * @param in	      : input segmentation data, which could be either 0 or 1
 * @param out       : processed input
 * @detail          : The function works as a state machine. It has two states. Reset state and Running state. When it's at reset state and the input is 0, it does nothing.
 *                  : Otherwise if the input is 1, it means the beggining of a segmentation and it changes state to running state. In this state a counter increments everytime with each 
 *                  : input wuth value 1 to keep track of the length of the segment. If the input goes to zero in this state, it means the segment has ended. If the length of the segment is
 *                  : smaller than a threshold, this segment is eliminated and set to zero.
 */	

void PostBuffer_acc_1(int in, int* out)
{
	static int state = 0;																										// State machine
	static int counter = 0;																									// Counter to keep track of the length of the segment
	static int Buffer[12];																										// Buffer to hold the incoming segmentation values
	int i;
	for (i = 0; i < 11; i++)																									// Shifting the values inside the buffer
	{
		Buffer[i] = Buffer[i+1];
	}
	Buffer[11] = in;																													// Adding the latest data to the buffer
	switch(state)
	{
		case 0:																																// Reset state
			counter = 0;																												// Reset the counter to zero
			if (in == 1)																												// If input is one, segment has started, start counting and go to the running state
			{
				(counter)++;
				state = 1;
			}
			break;
		case 1:
			if (in == 1)																											  // While Input is 1, keep counting to keep track of the length
				(counter)++;
			if (in == 0)																												// If input is zero, segment has ended, check the length
			{
				if (counter < 10)																									// If length is smaller than a threshold, eliminate it by zeroing it out
				{	
					for (i = (11 - counter); i<11; i++)
						Buffer[i] = 0;
				}
				state = 0;																												// Go back to reset state
			}
			break;
	}
	*out = Buffer[0];
}

/**
 * @brief	Post processing operation to get rid of gaps between segments that are samller than a certain size, for real-time practice, input data is fed one at a time.
 * @param in	      : input segmentation data, which could be either 0 or 1
 * @param out       : processed input
 * @detail          : The function works as a state machine. It has two states. Reset state and Running state. When it's at reset state and the input is 1, it does nothing.
 *                  : Otherwise if the input is 0, it means the beggining of a gap and it changes state to running state. In this state a counter increments everytime with each 
 *                  : input wuth value 0 to keep track of the length of the gap. If the input goes to 1 in this state, it means the gap has ended. If the length of the gap is
 *                  : smaller than a threshold, this gap is eliminated and set to 1.
 */	
void PostBuffer_acc_2(int in, int* out)
{
	static int state = 0;																										// State machine
	static int counter = 0;																									// Counter to keep track of the length of the segment
	static int Buffer[7];																										// Buffer to hold the incoming segmentation values
	int i;
	for (i = 0; i < 6; i++)																									// Shifting the values inside the buffer
	{
		Buffer[i] = Buffer[i+1];
	}
	Buffer[6] = in;																													// Adding the latest data to the buffer
	switch(state)
	{
		case 0:																																// Reset state
			counter = 0;																												// Reset the counter to zero
			if (in == 0)																												// If input is one, segment has started, start counting and go to the running state
			{
				(counter)++;
				state = 1;
			}
			break;
		case 1:
			if (in == 0)																											  // While Input is 1, keep counting to keep track of the length
				(counter)++;
			if (in == 1)																												// If input is zero, segment has ended, check the length
			{
				if (counter < 5)																									// If length is smaller than a threshold, eliminate it by zeroing it out
				{	
					for (i = (6 - counter); i<6; i++)
						Buffer[i] = 1;
				}
				state = 0;																												// Go back to reset state
			}
			break;
	}
	*out = Buffer[0];
}

void Smooth_1(int32_t input, float32_t *out, int init)
{
	static int dummy[SMOOTH_SIZE];                   							         // Buffer used for first round of smoothing on the acc values
	static int ptr = 0;																					 	         // Pointer to the place inside the smoothing buffer to add the new data
	static float sum = 0;	
  static int count = 0;	
  int j;	
  if (init == 0)																								         // If init = 0, run the algorithm
	{
		sum = sum- dummy[ptr] + input;            										
		dummy[ptr] = input;
		if (count < SMOOTH_SIZE-1)																	         // If the Buffer is not filled yet, set the outout at zero
				*out = 0;
		else
				*out = (float)(sum)/SMOOTH_SIZE;
		ptr++;																										         	 // Increment the poiter, if it reaches the end, set it back to zero
		if (ptr == SMOOTH_SIZE)
				ptr = 0;
		(count)++;
  }
	else																												        	 // Initializing
	{
		for (j = 0; j<SMOOTH_SIZE; j++)															         // Setting buffers to zero
			dummy[j] = 0;
		sum= 0;																								               // Initializing variables
		ptr = 0;
	}
}

void Smooth_2(int32_t input, float32_t *out, int init)
{
	static int dummy[SMOOTH_SIZE];                   							 // Buffer used for first round of smoothing on the acc values
	static int ptr = 0;																					 	 // Pointer to the place inside the smoothing buffer to add the new data
	static float sum = 0;	
  static int count = 0;	
  int j;	
  if (init == 0)																								 // If init = 0, run the algorithm
	{
		sum = sum- dummy[ptr] + input;            										
		dummy[ptr] = input;
		if (count < SMOOTH_SIZE-1)																	 // If the Buffer is not filled yet, set the outout at zero
				*out = 0;
		else
				*out = (float)(sum)/SMOOTH_SIZE;
		ptr++;																											 // Increment the poiter, if it reaches the end, set it back to zero
		if (ptr == SMOOTH_SIZE)
				ptr = 0;
		(count)++;
  }
	else																													 // Initializing
	{
		for (j = 0; j<SMOOTH_SIZE; j++)															 // Setting buffers to zero
			dummy[j] = 0;
		sum= 0;																								       // Initializing variables
		ptr = 0;
	}
}
