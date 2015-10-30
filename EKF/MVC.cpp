#include "algorithms.h"
#define BUFFER_SIZE 1500																					// Size of the classification window
#define SMOOTH_SIZE 501																					// Size of the sliding window size used for smoothing
extern uint8_t cadence_acc;
extern uint32_t quad_detected;
extern int32_t  posproc;	
extern int32_t	 Rawfiltered1, Rawfiltered2;	
#include <fstream>
extern std::ofstream f1_1, f2, f3, f4, f5;
extern uint32_t global_counter;


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
void SegmetnAnalysisMVC(float in, float RMS, float *intensity, int sel, int init);
void MVCQuad(uint8_t* rawData, uint8_t *Results, int init);
void MVCHam(uint8_t* rawData, uint8_t *Results, int init);
void MVC(uint8_t* rawData, uint8_t *Results, int init)
{
	 float dummy;
	 if (init == 1)
		 SegmetnAnalysisMVC(0, 0, &dummy, 0, 1);
	 MVCQuad(rawData,Results, init);
	 MVCHam(rawData,Results, init);
}
void MVCQuad(uint8_t* rawData, uint8_t *Results, int init)
{  
	static float smoothedAccBuff[1500];// = (float*)(0x10002000);															// Buffer to hold smoothed accelerometer values
	static float s_1 = 0;																																										
	////////Filtetring stuff
	static int dummy_1[SMOOTH_SIZE];
	static int ptr_1 = 0;
	static int ptr_2 = 0;
	static int ptr_3 = 0;
	static float sum_1 = 0;
	static int count_1 = 0;
	static float filter_out;
	static float LOW2_out;
	static float detrend_out;
	volatile static float Max_Intensity = 0;
	volatile static float intensity = 0;
	static int sample_counter = 0;
	uint32_t dummy_2, dummy;
	//////////////////
	float max = -10000000;
	float min =  10000000;
	RAW_DATA raw, raw2;
	int Class, j;
	int pnt3 = 0;
	if (init == 0)																									// If init = 0, run the algorithms
	{
			if (sample_counter > 0)
			{
				raw.raw_data = 0;
				raw2.raw_data = 0;
				/*raw.data_byte[0] = rawData[pnt3+2];																																						
				raw.data_byte[1] = rawData[pnt3+1];
				raw.data_byte[2] = rawData[pnt3+0];
				if ((rawData[pnt3+0] & 0x80) == 0x00)*/
				raw.data_byte[0] = rawData[pnt3+0];     																   // uncommnet use this for Debuging while reading from memory, no conversion needed.
				raw.data_byte[1] = rawData[pnt3+1];																			 
				raw.data_byte[2] = rawData[pnt3+2];
				if ((rawData[pnt3+2] & 0x80) == 0x00)
					raw.data_byte[3] = 0;																											 // Input data is positive, set the sign to zero.
				else
					raw.data_byte[3] = 0xFF;	
				
				raw2.data_byte[0] = rawData[pnt3+6];     																   // uncommnet use this for Debuging while reading from memory, no conversion needed.
				raw2.data_byte[1] = rawData[pnt3+7];																			 
				raw2.data_byte[2] = rawData[pnt3+8];
				if ((rawData[pnt3+2] & 0x80) == 0x00)
					raw2.data_byte[3] = 0;																											 // Input data is positive, set the sign to zero.
				else
					raw2.data_byte[3] = 0xFF;

				PreProcessing((raw.raw_data), (raw2.raw_data), &filter_out, &LOW2_out, &detrend_out, 0, 0);		
				//f2 << filter_out << std::endl;// Putting Free-mode filtered EMG values in the bluetooth buffer 
				/////////////Filtering
				sum_1 = sum_1 - dummy_1[ptr_1] + LOW2_out;
				dummy_1[ptr_1] = LOW2_out;
				if (count_1 < SMOOTH_SIZE-1)
					s_1 = 0;
				else
					s_1 = (float)(sum_1)/SMOOTH_SIZE;
				(ptr_1)++;
				if (ptr_1 == SMOOTH_SIZE)
					ptr_1 = 0;
				(count_1)++;
				if (s_1 < 300)
					s_1 = 0;
				ptr_3 = ptr_2 - BUFFER_SIZE/2;
				if (ptr_3 < 0)
					ptr_3 += BUFFER_SIZE;
				for (j=0; j < BUFFER_SIZE ; j++)														// Shifting the values inside the buffer and finding the maximum and minimum 
				{
					if (max < smoothedAccBuff[j])
							max = smoothedAccBuff[j];
					if (min > smoothedAccBuff[j])
							min = smoothedAccBuff[j];
				}
				smoothedAccBuff[ptr_2] = s_1;												// Adding new smoothed accelerometer to the buffer
				(ptr_2)++;
				if (ptr_2 == BUFFER_SIZE)
					ptr_2 = 0;
				if (s_1 > (max+min)/2)   																		// Classification of accelerometer values
				{
					Class = 1;
				}
				else
				{
					Class = 0;
				}
				SegmetnAnalysisMVC(Class, s_1, (float*)&intensity, 0, 0);
				if (intensity >  Max_Intensity)
				{
					Max_Intensity = intensity;
					//f5 << Max_Intensity << std::endl;
				}
				//f1_1 << Class << std::endl;
				//f2 << smoothedAccBuff[ptr_3] << std::endl;
				dummy_2 = Max_Intensity;
				dummy = 0;
				dummy |= ((dummy_2 & 0xff) << 8) | ((dummy_2 & 0xff00) >> 8);// Sending a 2-byte unsigned integer MVC value to bluetooth buffer. Changing from big-endian to small-endian.
				//memcpy((Results + 108), (uint8_t*)&dummy, 2);
		}
		sample_counter++;
	}
	else																														// Initializing
	{	
			for (j = 0; j<SMOOTH_SIZE; j++)
			{
				dummy_1[j] = 0;
			}
			for (j = 0; j<BUFFER_SIZE; j++)
			{
				smoothedAccBuff[j] = 0;
			}
				sum_1 = 0;
				ptr_1 = 0;
			  ptr_2 = 0;
				Max_Intensity = 0;
				count_1= 0;
				sample_counter = 0;
		  	intensity = 0;
	}
	
}

void MVCHam(uint8_t* rawData, uint8_t *Results, int init)
{  
	static float smoothedAccBuff[1500];// = (float*)(0x10002FA0);															// Buffer to hold smoothed accelerometer values
	static float s_1 = 0;																																									
	////////Filtetring stuff
	static int dummy_1[SMOOTH_SIZE];
	static int ptr_1 = 0;
	static int ptr_2 = 0;
	static float sum_1 = 0;
	static int count_1 = 0;
	static float filter_out;
	static float LOW2_out;
	static float detrend_out;
	static int sample_counter = 0;
	//////////////////
	float max = -10000000;
	float min =  10000000;
	RAW_DATA raw;
	int Class, j;
	int pnt3 = 3;
	volatile static float Max_Intensity = 0;
	volatile static float intensity = 0;
	uint32_t dummy_2, dummy;
	if (init == 0)																									// If init = 0, run the algorithms
	{
			if (sample_counter > 200)
			{
				raw.raw_data = 0;
				raw.data_byte[0] = rawData[pnt3+2];																																						
				raw.data_byte[1] = rawData[pnt3+1];
				raw.data_byte[2] = rawData[pnt3+0];
				if ((rawData[pnt3+0] & 0x80) == 0x00)
				/*raw.data_byte[0] = rawData[pnt3+0];     																   // uncommnet use this for Debuging while reading from memory, no conversion needed.
				raw.data_byte[1] = rawData[pnt3+1];																			 
				raw.data_byte[2] = rawData[pnt3+2];
				if ((rawData[pnt3+2] & 0x80) == 0x00)*/
					raw.data_byte[3] = 0;																											 // Input data is positive, set the sign to zero.
				else
					raw.data_byte[3] = 0xFF;	
				
				PreProcessing((raw.raw_data), (raw.raw_data), &filter_out, &LOW2_out, &detrend_out, 1, 0);	
				/////////////Filtering
				sum_1 = sum_1 - dummy_1[ptr_1] + LOW2_out;
				dummy_1[ptr_1] = LOW2_out;
				if (count_1 < SMOOTH_SIZE-1)
						s_1 = 0;
				else
						s_1 = (float)(sum_1)/SMOOTH_SIZE;
				(ptr_1)++;
				if (ptr_1 == SMOOTH_SIZE)
						ptr_1 = 0;
				(count_1)++;
				if (s_1 < 300)
					s_1 = 0;
				for (j=0; j < BUFFER_SIZE ; j++)														// Shifting the values inside the buffer and finding the maximum and minimum 
				{
					if (max < smoothedAccBuff[j])
							max = smoothedAccBuff[j];
					if (min > smoothedAccBuff[j])
							min = smoothedAccBuff[j];
				}
				smoothedAccBuff[ptr_2] = s_1;												// Adding new smoothed accelerometer to the buffer
				(ptr_2)++;
				if (ptr_2 == BUFFER_SIZE)
					ptr_2 = 0;
				if (s_1 > (max+min)/2)   																		// Classification of accelerometer values
				{
					Class = 1;
				}
				else
				{
					Class = 0;
				}
				SegmetnAnalysisMVC(Class, s_1, (float*)&intensity, 1, 0);
				if (intensity >  Max_Intensity)
					Max_Intensity = intensity;
				dummy_2 = Max_Intensity;
				dummy = 0;
				dummy |= ((dummy_2 & 0xff) << 8) | ((dummy_2 & 0xff00) >> 8);// Sending a 2-byte unsigned integer MVC value to bluetooth buffer. Changing from big-endian to small-endian.
				//memcpy((Results + 110), (uint8_t*)&dummy, 2);
		}
		sample_counter++;
	}
	else																														// Initializing
	{	
			for (j = 0; j<SMOOTH_SIZE; j++)
			{
				dummy_1[j] = 0;
			}
			for (j = 0; j<BUFFER_SIZE; j++)
			{
				smoothedAccBuff[j] = 0;
			}
				sum_1 = 0;
				ptr_1 = 0;
				ptr_2 = 0;
				Max_Intensity = 0;
				count_1 = 0;
				sample_counter = 0;
				intensity = 0;
	}
	
}

void SegmetnAnalysisMVC(float in, float RMS, float *intensity, int sel, int init)
{
	static int state_q = 0;
	static float RMS_Ave_q = 0;
	static int counter_q = 0;
	static int state_h = 0;
	static float RMS_Ave_h = 0;
	static int counter_h = 0;
	int *state;
	int *counter;
	float *RMS_Ave;
	if (init == 1)
	{
		state_q = 0;
		state_h = 0;
	}
	else
	{
			if (sel == 0)
			{
				state = &state_q;
				counter = &counter_q;
				RMS_Ave = &RMS_Ave_q;
			}
			else
			{
				state = &state_h;
				counter = &counter_h;
				RMS_Ave = &RMS_Ave_h;
			}
			switch(*state)
			{
				case 0:
					*RMS_Ave = 0;
					*counter = 0;
					if (in == 1)
						*state = 1;
					break;
				case 1:
					(*counter)++;
					*RMS_Ave += RMS;
					if (in == 0)
					{
							*state = 0;
							*intensity = (*RMS_Ave / *counter);
					}
					break;
			}
	}
}
