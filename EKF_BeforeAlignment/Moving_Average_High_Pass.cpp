#include "algorithms.h"

/**
 * @brief	High-pass filter by means of subtracting the average value of a sliding window from each sample. New data is fed to the algorithm one at a time for real-time purposes.
 * @param Data	    : new raw EMG data
 * @param MAout     : High-pass filter output
 * @parami init  	  : if init = 0, initialize, else run the algorithm
 * @parami sel  	  : if sel = 0, run for quad, otherwise run for ham
 * @return	nothing
 * @detatil         : instead of shifting the values inside the buffers to implement the sliding window effect, this function uses circulating pointers. The pointer is incremented
 *                  : everytime and is set to zero when it reaches the end of the buffer. Once the buffer is filled, pointer points to the oldest value inside the buffer and the new data
 *                  : would replace the old data. Another pointer which also increments everytime has the effect of pointing to the middle of the sliding window at all time.
 */	
 
void Moving_Average_HIGH(int32_t *Data, float32_t *MAout, uint32_t init, uint32_t sel)
{
	int i;
	static float dummy_1[MA_SIZE];                             // The buffer to hold quad raw EMG quad values
	static float dummy_2[MA_SIZE];														 // The buffer to hold quad raw EMG ham values
	static float dummy_3[MA_SIZE];	
	static float sum_quad = 0;																 // Variables to hold the sum of the values inside the sliding window
	static float sum_ham  = 0;
	static float sum_vl  = 0;
	static int ptr1_1 = 0;																		 // Pointer to the place inside the buffer to add the new data
	static int ptr1_2 = (MA_SIZE - 1)/2 + 1;									 // Pointer to the sample inside the buffer (the miffle point) in which we're subtracting the average from
	static int ptr2_1 = 0;
	static int ptr2_2 = (MA_SIZE - 1)/2 + 1;
	static int ptr3_1 = 0;
	static int ptr3_2 = (MA_SIZE - 1)/2 + 1;
	static int count1 = 0;																		 // A counter to keep the output at zero brfore the buffer is full
	static int count2 = 0;
	static int count3 = 0;
	float *dummy;
	float *sum;
	int *ptr1;
	int *ptr2;
	int *count;
	if (sel == 0)																							 // If sel = 1, set the pointers to the buffers and variables for Ham
	{
		dummy = dummy_1;
		sum   = &sum_quad;
		ptr1  = &ptr1_1;
		ptr2  = &ptr1_2;
		count = &count1;
	}
	else if (sel == 1)																											// If sel = 0, set the pointers to the buffers and variables for Ham
	{
		dummy = dummy_2;
		sum   = &sum_ham;
		ptr1  = &ptr2_1;
		ptr2  = &ptr2_2;
		count = &count2;
	}
	else
	{
		dummy = dummy_3;
		sum   = &sum_vl;
		ptr1  = &ptr3_1;
		ptr2  = &ptr3_2;
		count = &count3;
	}
	if (init == 0)																						// If init = 0, run the algorithm
	{
		*sum = *sum - dummy[*ptr1] + *Data;											// Oldest value is subtracted from the sum and new value is added
		dummy[*ptr1] = *Data;																		// new value replaes the oldest value in the buffer
		if (*(count) < MA_SIZE-1)
			*MAout = 0;
		else
			*MAout = (float)dummy[*ptr2] - (float)(*sum)/MA_SIZE; // The average of the window is subtracted from the middle point of the window
		(*ptr1)++;																							// Increment the pointers to implement the sliding effect and set them to zero if they reach the end of the buffer
		(*ptr2)++;
		if ( (*ptr1) == MA_SIZE )																
			*ptr1 = 0;
		if ( (*ptr2) == MA_SIZE )
			*ptr2 = 0;
		(*count)++;
	}
	else																											// Initialize the buffers and variables
	{
		for (i=0; i< MA_SIZE; i++)
		{
		  dummy_1[i] = 0;																				// Setting the buffers to zero
			dummy_2[i] = 0;
		}
		sum_quad = 0;
		sum_ham  = 0;
		count1 = 0;
	  count2 = 0;
		ptr1_1 = 0;
		ptr1_2 = (MA_SIZE - 1)/2 + 1;                           // Reset to the middle point
		ptr2_1 = 0;
		ptr2_2 = (MA_SIZE - 1)/2 + 1;
	}
	
}
