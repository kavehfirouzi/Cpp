#include "algorithms.h"


/**
 * @brief	Pulling the RMS signal down over vertical axis withot shifting over horizantal axis. New data is fed to the algorithm one at a time for real-time purposes.
 * @param Data	    : new RMS data
 * @param MAout     : detrended output
 * @parami init  	  : if init = 0, initialize, else run the algorithm
 * @parami sel  	  : if sel = 0, run for quad, otherwise run for ham
 * @return	nothing
 * @detailed        : For each sample, the average value over a window before that point is subtracted from the value of that point.
 *                    Unlike other sliding window operations, the calculated value for each point replace the old value and is used for future calculations.
 *                  : instead of shifting the values inside the buffers to implement the sliding window effect, this function uses circulating pointers. The pointer is incremented
 *                  : everytime and is set to zero when it reaches the end of the buffer. Once the buffer is filled, pointer points to the oldest value inside the buffer and the new data
 *                  : would replace the old data.
 */	
 
void Detrending(float *Data, float32_t *MAout, uint32_t det_size, uint32_t init, uint32_t sel)
{
	int i;
	static float dummy_1[DET_MA_SIZE];                                  // Buffer to hold values for quad
	static float dummy_2[DET_MA_SIZE];																	// Buffer to hold values for ham
	static float sum_quad = 0;																					// variable to hold the sum of values inside the sliding window
	static float sum_ham  = 0;
	static int ptr1_1 = 0;
	static int ptr2_1 = 0;
	static int count1 = 0;                                              // A counter to keep the output at the original value of the inputs before the buffer is filled
	static int count2 = 0;
	static float dummy_3[DET_MA_SIZE];                                  // Buffer to hold values for quad
	static float sum_acc  = 0;
	static int ptr3_1 = 0;                                         // A counter to keep the output at the original value of the inputs before the buffer is filled
	static int count3 = 0;
	static float dummy_4[DET_MA_SIZE];                                  // Buffer to hold values for quad
	static float sum_cad  = 0;
	static int ptr4_1 = 0;                                         // A counter to keep the output at the original value of the inputs before the buffer is filled
	static int count4 = 0;
	float *dummy;
	float *sum;
	int *ptr1;
	int *count;
	if (sel == 0)																												// Setting to pointers for quad buffers and variables
	{
		dummy = dummy_1;
		sum   = &sum_quad;
		ptr1  = &ptr1_1;
		count = &count1;
	}
	else if (sel == 1)																															// Setting to pointers for ham buffers and variables
	{
		dummy = dummy_2;
		sum   = &sum_ham;
		ptr1  = &ptr2_1;
		count = &count2;
	}
	else if (sel == 2)
	{
		dummy = dummy_3;
		sum   = &sum_acc;
		ptr1  = &ptr3_1;
		count = &count3;
	}
	else
	{
		dummy = dummy_4;
		sum   = &sum_cad;
		ptr1  = &ptr4_1;
		count = &count4;
	}
	if (init == 0)																											// Run the algorithm
	{
		*sum = *sum - dummy[*ptr1] + *Data;																// Oldest value in the buffer is subtracted from sum and the new value is added
		if (*(count) < det_size-1)																				// Before the buffer is fileld, output is the same as input.
			*MAout = *Data;
		else
			*MAout = *Data - (*sum) /  (det_size);													// Subtract the average value of the window from the input
		*sum = *sum - *Data + *MAout;																			// As mentioned before, the calculated value for each sample replaces it's oroginal value
		dummy[*ptr1] = *MAout;																						// Put the new calculated value in the buffer

		(*ptr1)++;																												// Increment the pointer to make the window slide by one point. Reset pointer to zero when it reaches the end of the buffer
		if ( (*ptr1) == det_size )
			*ptr1 = 0;
		(*count)++;
	}
	else																																// Initialize the buffers and variabels
	{
		for (i=0; i< DET_MA_SIZE; i++)
		{
			dummy_1[i] = 0;																									// Setting the buffers to zero
			dummy_2[i] = 0;
			dummy_3[i] = 0;
		}
		sum_quad = 0;
		sum_ham  = 0;
		sum_acc = 0;
		count1 = 0;
		count2 = 0;
		count3 = 0;
		ptr1_1 = 0;
		ptr2_1 = 0;
		ptr3_1 = 0;
	}
}
