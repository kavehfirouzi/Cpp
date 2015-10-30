#include "algorithms.h"
#include <math.h>
/**
 * @brief	Finding the RMS values of the the values inside a sliding window. New data is fed to the algorithm one at a time for real-time purposes.
 * @param Data	    : new data
 * @param MAout     : RMS output
 * @parami init  	  : if init = 0, initialize, else run the algorithm
 * @parami sel  	  : if sel = 0, run for quad, otherwise run for ham
 * @return	nothing
 * @detatil         : instead of shifting the values inside the buffers to implement the sliding window effect, this function uses circulating pointers. The pointer is incremented
 *                  : everytime and is set to zero when it reaches the end of the buffer. Once the buffer is filled, pointer points to the oldest value inside the buffer and the new data
 *                  : would replace the old data.
 */	
 
void RMS_f32(float32_t *Data, float32_t *RMSOut, uint32_t init, uint32_t sel, int size)
{
	int i;
	static float dummy_1[RMS_SIZE_RUN_HAM];																				// Buffer to hold quad values
	static float dummy_2[RMS_SIZE_RUN_HAM];																				// Buffer to hold ham values
	static float RMS_1 = 0;																										// Variable to hold the sum of squared values inside the sliding window
	static int pointer_1 = 0;																									// Pointer to the place inside the buffer that holds the odest value
	static float RMS_2 = 0;
	static int pointer_2 = 0;
	static int count1 = 0;																										// A counter that keeps the output at zero before the buffer is filled
	static int count2 = 0;
	float *dummy, *RMS;
	int *pointer;
	float newData = *Data;
	int *count;
	if (sel == 1)																															// If sel = 1, set the pointers to Ham buffer and variables
	{
		dummy = dummy_1;
		RMS = &RMS_1;
		pointer = &pointer_1;
		count = &count1;
	}
	else																																			// If sel = 0, set the pointers to Ham buffer and variables
	{
		dummy = dummy_2;
		RMS = &RMS_2;
		pointer = &pointer_2;
		count = &count2;
	}
	if (init == 0)																														// If init = 0, Run the algorithms
	{
		*RMS = *RMS - (dummy[*pointer]) + newData*newData;											// subtract the oldest value in the buffer and add the square of the latest input data
		if (*RMS < 0)																														// Just a caution to avoid negative RMS values in the beggining, before fillin the buffer, most likely unnecessary
			(*RMS = 0);
		dummy[*pointer] = newData*newData;																			// Replace the oldest value with the square of the latest input data
		(*pointer)++;																														// Increment the pointer, set it to zero if it reaches the end
		if (*pointer>size-1)
			(*pointer) = 0;
		if ((*count) < MA_SIZE+size+1)                                          // Before the buffer is filled, set the output to zero
			*RMSOut = 0;
		else
			*RMSOut = sqrt(*RMS/size);
		(*count)++;
	}
	else																																			// If init = 1, initialize the buffers and variables
	{
		for (i=0; i<RMS_SIZE_RUN_HAM; i++)
		{
		  dummy_1[i] = 0;																												// Set the buffers to zero
			dummy_2[i] = 0;
		}
			count1 = 0;																														// Set the variabled to initial values
	    count2 = 0;
			RMS_1 = 0;
			pointer_1 = 0;
			RMS_2 = 0;
			pointer_2 = 0;
	}
}
