#include "algorithms.h"

#include <fstream>
extern std::ofstream f1, f2, f3, f4, f5, f6, f6_1, f6_2, X, Y;
#define STOP 250
#define CAD_COEFF 7500																						// If the distance between two peaks is x, the cadence is 60/(x/125) or 7500/x (125 is 500 / 4)

extern unsigned char cadence;
extern unsigned int PrevQuadPeak;																	// Holds the value of the previous quad peak
extern unsigned int CurrentQuadPeak;															// Holds the value of the current quad peak
extern unsigned int CurrentHamPick;                               // This global counter increments with every new RMS values. It's for calculating the distance between peaks for coordination.
extern unsigned int p2p; 																					// Holds the distance between current quad peak and previous quad peak
extern unsigned int H2Q;																					// Holds the distance between quad peak and Ham peak
extern unsigned int ham_detected;
extern unsigned int quad_detected;
extern unsigned int ham_counter;
extern unsigned int pseudo_ham;
float targeting_ham;
extern unsigned int CurrentHamPeak;
float targeting_quad;

struct Ham_Seg h_s[5];
struct Ham_Seg h_s_Intensity[8];
int h_s_count;
unsigned int quad_peaks[5];
extern int Normalize_quad;
extern int Normalize_ham;
extern int posptrQ;
extern int posptrH;

extern int length;
extern int current_length;
extern uint8_t cadence_acc;

/**
 * @brief	processing the segments to find the peak in each segment, find the intensity (average RMS value in the segment). Find coordination and targetting values. 
 *        For real time purposes, data is fed to the algorithm one at a time.
 * @param  in	         : input segmentation data, which could be either 0 or 1
 * @parami RMS         : Input RMS value
 * @param  cad         : Cadence value
 * @paramo RMSout_quad : delayed hamd RMS
 * @parami intensity   : intensity value inside each segment
 * @parami init  	     : if init = 0, initialize, else run the algorithm
 * @return	nothing
 * @detail         
 */	
 
void SegmetnAnalysisQuad(float in, float RMS, uint8_t* cad, uint8_t *intensity, float *rawIntensity, uint8_t *targeting, uint8_t *coordination, uint8_t* IntensityHam, uint32_t global_counter, int init)
{
	static int state = 0;																						 // state machine
	static float RMS_Ave = 0;																				 // Variable to hold the sum of normalized RMS valiues inside the segment
	static int max = 0;
	static int counter = 0;                                          // The countet to keep track of the length of the segment																		 // Buffer to hold the last two peak-to-peak values.
	static float RMS_Aves[2] = {0,0};																 // Buffer to hold the last two intensity values													 // Enables the the incrementing of the above counter. If it's zero, the counter won't increment.
	int i;
	static int point_ham = 0;
	static float intensity_ham = 0;
	static float Ham_Ave[2] = {0,0};
	static float Targeting_Ave[2] = {0,0};
	float f;
	float max_ham;
	static int start, stop;
	static float coord;
	static unsigned char AngleBuff[500];
	static int ptr = 0;
	int angle = 0;
	int mid;
	if (init == 1)																									 // If init = 1, initialize the parameters
	{
		state = 0;																										
		RMS_Aves[0] = 0;
		RMS_Aves[1] = 0;
	}
	else																														 // start Running the algorithms 
	{
		    if (length != 0 && cadence_acc != 0)
			{
				angle  = (float)current_length/length*180;
				if (angle > 180)
					angle = 0;
			}
			else
				angle = 0;
			switch(state)																								 // State Machine
			{
				case 0:																										 // Reset state
					RMS_Ave = 0;																						 // Sum of RMS values is set to 0
					targeting_quad = 0;
					counter = 0;
					max = 0;
					if (in == 1)																						 // If input is 1, a segment has started, go to running state, set the quad_detected flag to 1 so ham intensity can get updated
					{
						ptr = 1;
						AngleBuff[0] = angle;
						max = RMS;
						state = 1;
						start = global_counter;
					}
					break;
				case 1:																										 // Running state
					counter++;				
																			 // If the input is 1, keep incrementing the counter to keep track of the length of the segment
					AngleBuff[ptr] = angle;
					ptr++;
					if (ptr == 500)
						ptr = 0;
					RMS_Ave += RMS / Normalize_quad;												 // Accumulate the normalized RMS values
					targeting_quad += RMS;																	 // Same as above, but non-normalized values
					if (RMS > max)                                           // As new RMS comes in, compare it with previous max, update if it's bigger
					{
						max = RMS;
						CurrentQuadPeak = global_counter;											 // Keeping track of the peak
						if (global_counter - 190 == 19864)
							global_counter = 19864 + 190;
						posptrQ = 0;
					}
					if (in == 0)																						 // If input is zero, segment has ended, next state will be the reset state.
					{
							state = 0;
							if (counter > 50)
							{
							   *rawIntensity = targeting_quad / counter;
								max_ham = 0;
								for (i = 0; i<5; i++)
								{
									if (h_s[i].point >= PrevQuadPeak && h_s[i].point <= CurrentQuadPeak)
									{
										if (h_s[i].Ave != 0 && h_s[i].Ave > max_ham)
										{
											max_ham = h_s[i].Ave;
											intensity_ham = h_s[i].Ave;											// This one is for intensity		
											point_ham  = h_s[i].point;											// This one is for Coordination	
											targeting_ham = h_s[i].Raw_RMS;									// This one is for Ratio
										}
									}
								}
								Ham_Ave[0] = Ham_Ave[1];
								Ham_Ave[1] = intensity_ham;
								f = (Ham_Ave[0] + Ham_Ave[1])*125;
								if (f > 255)																					// We don't want the intensity to reach 100%. If it's over 100%, cut it to 90%
									f = 255;
								*IntensityHam = (uint8_t)f;
								//////////////////////////////
								f = (RMS_Ave / counter);                       				 // Average the sum of RMS values by dividing it by the length of the segment and scale it to 255
								RMS_Aves[0] = RMS_Aves[1];
								RMS_Aves[1] = f;																			// Put the latest intensity value inside the averaging buffer
								f = (RMS_Aves[0] + RMS_Aves[1])*125;									// If more than one segments have been detected, average the latest two values
								if (f > 255)																					// We don't want the intensity to reach 100%. If it's over 100%, cut it to 90%
									f = 255;
								*intensity = (uint8_t)f;		
																																			// Find the distance between the current peak and the previous one
																																			// Update the previous peak, the current peak is now the previous peak in the next cycle
								Targeting_Ave[0] = Targeting_Ave[1];
								if (targeting_quad != 0  &&  targeting_ham != 0)
									Targeting_Ave[1] = (targeting_quad/(targeting_quad + targeting_ham)*100);
								*targeting = (uint8_t)(Targeting_Ave[0] + Targeting_Ave[1])/2;
								//f5 << CurrentQuadPeak << std::endl;; // (float)*targeting << std::endl;
								f5 << (float)*targeting << std::endl;
								p2p = CurrentQuadPeak - PrevQuadPeak;	
								H2Q = abs((int)CurrentQuadPeak - (int)point_ham);     // Now that we've calculated coordination for the previous peak, update the distance between current quad peak and the largest of current ham peaks
								if (p2p !=0)
									coord = ((float)H2Q / (p2p) * 180);
								if (coord <= 180)
								{
									if (coord > 90)
										coord = 180 - coord;
									*coordination = (uint8_t)coord;
								}
								for (i = 0; i < 4; i++)
									quad_peaks[i] = quad_peaks[i+1];
								quad_peaks[4] = CurrentQuadPeak;
								PrevQuadPeak = CurrentQuadPeak;
								mid = counter/2 + 68; //68 = 4*62 - 190;
								f6 << (float)AngleBuff[mid] << std::endl;
							}
							quad_detected = 1;
					}
					break;
			}
	}
}

/**
 * @brief	 Calculating cadence based on segmentation data, and intensity based on RMS of quad and ham. for real time purposes data is fed one at a time to the algorithm
 * @param  in	         : segmentation data
 * @param	 RMS_quad    : quad RMS
 * @param	 RMS_quad    : ham RMS
 * @param  cad				 : Pointer to cadence variable
 * @param  intensity_quad		: Pointer to quad intensity variable
 * @param  intensity_ham		: Pointer to ham intensity variable
 * @detail             : Distance between to consecutive segments is calculated. (From start of segment to start of the next segment). Cadence is the average over
 *                     : two of such distances. The function works as a state machine. It has two states. state 0 and state 1. When it's at 0 state and no segment has been detected, nothing happens.
 *                     : Otherwise if the input is 1, it means the beggining of a segmentation and it changes state to state 1 and starts counting. In this state a counter increments everytime with each 
 *                     : input with value 1 to keep track of the distance. If the input goes to zero in this state, it means the segment has ended. We go back to state 0. Now, since a segment has been detected, 
 *                     : state 0 acts differenty. It keeps incrementing the counter until input is 1, which means the start of another segment. Value of the counter is stored in the buffer as the distance. Counter is reset
 *										 : and the process repeats by going to state 1. Intensity is calculated as the average of RMS between to segments. (Start of one segment to the start of next). Latest
 *										 : two intensities are averaged for final result
 */	
void SegmetnAnalysisWalk(float in, float RMS_quad, float RMS_ham, uint8_t* cad, unsigned char *intensity_quad, unsigned char *intensity_ham, unsigned char *targeting, int init)
{
	static int state = 0;																								// State machine
	static float RMS_Ave_1 = 0;																					// Variable to hold the sum of normalized RMS values
	static float RMS_Ave_2 = 0;
	static int counter = 0;																							// Counter to keep track of distance between segments
	static int dists[2] = {0,0};																				// Buffer to hold the latest two distances
	static float RMS_Aves_1[2] = {0,0};																	// Buffers to hold the latest two normalized RMS values
	static float RMS_Aves_2[2] = {0,0};
	static int pointer = 0;
	static int peak_counter = 0;																				// Keeps track of the number of segments detected so far.
	static int stop_counter = 0;																				// Counter to keep track of time no segment has been detected
	static int stop_counter_enable = 0;																	// Enables and disable the above counter
	static int i = 0;
//	static int counter_2 = 0;
//	static unsigned char Buffer_1[200];
//	static unsigned char Buffer_2[200];
//	static unsigned char Buffer_RMS_1[200];
//	static unsigned char Buffer_RMS_2[200];
//	static int var_1;
//	static float var_3, var_2;
	if (init == 1)																											// If init == 0, go to 0 state
		state = 0;
	if (stop_counter == STOP)                                            // If no segment has been detected for the pase 250 samples (2 seconds), reset all buffers and variables
	{
		*cad = 0;
		state = 0;
		counter = 0;
		pointer = 0;
		peak_counter = 0;
		stop_counter = 0;
		stop_counter_enable = 0;
		RMS_Ave_1 = 0;
		RMS_Ave_2 = 0;
		targeting_quad = 0;
		for (i = 0; i < 2; i++)
		{
			dists[i] = 0;
			RMS_Aves_1[i] = 0;
			RMS_Aves_2[i] = 0;
		}
	}
	if (stop_counter_enable == 1)
				stop_counter++;
	switch(state)																											   // State machine
	{
		case 0:
			counter++;
			if (in == 1)																										 // A new segment has been detected, time to put the new distance and intensities in the buffer and calculate cadence
			{
				  /////////Cadence////////////////////////
					if (peak_counter > 0)																				 // Calculate cadence if more than 1 segment have been detected so far
					{
						stop_counter = 0;																			     // Reset the stop_counter to zero
					  stop_counter_enable = 1;                                   // Reset counter only should start counting when at least two segments have been detected
					  dists[pointer] = counter;															     // Putting the current distance in the buffer
						pointer++;
						if (pointer == 2)
							pointer = 0;
						if (peak_counter < 2)
						{
							if (dists[0] != 0)																			 // If only one distance has been detected so far, output it as cadence
								*cad = CAD_COEFF  / (dists[0]);
						}
						else
						{
							if ((dists[0] + dists[1]) != 0)													 // If more than one distances have been detected, start averaging over the last two each time
								*cad = CAD_COEFF * 2 / (dists[0] + dists[1]);
						}
					}
					////////itensity///////////////////
				/////////////////////////////////////
				peak_counter++;																								// Increment the pointer to keep track of how many peaks have been detected
				counter = 0;																									// Reset the counter for next round
				state = 1;																										
			}
			break;
		case 1:
			counter++;	
			RMS_Ave_1 += RMS_quad / Normalize_quad;													 // Accumulate normalized RMS values
			RMS_Ave_2 += RMS_ham /  Normalize_ham;
		  // intensity
			if (in == 0)																										// If input is 0, segment has ended, go back to state 0
			{
				RMS_Aves_1[0] = RMS_Aves_1[1];
				RMS_Aves_1[1] = RMS_Ave_1/counter;
				RMS_Aves_2[0] = RMS_Aves_2[1];
				RMS_Aves_2[1] = RMS_Ave_2/counter;
				*intensity_quad = (RMS_Aves_1[0] + RMS_Aves_1[1])*125;
				*intensity_ham = (RMS_Aves_2[0] + RMS_Aves_2[1])*125;
				if (targeting_quad != 0 &&  targeting_ham != 0)
						*targeting = 100 * ((float)*intensity_quad * Normalize_quad) / ((float)*intensity_quad * Normalize_quad + (float)*intensity_ham * Normalize_ham);
				state = 0;
				RMS_Ave_1 = 0;													 // Accumulate normalized RMS values
				RMS_Ave_2 =0;
			}
			break;
	}
}

void SegmetnAnalysisHam(float in, float RMS, float *rawIntensity, uint32_t global_counter, int init)
{
	static int state = 0;
	static float max = 0;
	static float RMS_Ave = 0;
	static int counter = 0;
	static float t_ham = 0;
	static int start, stop;
	int i;
	static unsigned short AngleBuff[500];
	static int ptr = 0;
	int angle = 0;
	int mid;
	if (init == 1)
	{
		state = 0;
	}
	else
	{
			if (length != 0 && cadence_acc != 0)
			{
				angle  = (float)current_length/length*360;
				if (angle > 360)
					angle = 0;
			}
			else
				angle = 180;
			switch(state)
			{
				case 0:
					RMS_Ave = 0;
					t_ham = 0;
					counter = 0;
					max = 0;
					if (in == 1)
					{
						ptr = 1;
						AngleBuff[0] = angle;
						max = RMS;
						state = 1;
						start = global_counter;
					}
					break;
				case 1:
					counter++;
					AngleBuff[ptr] = angle;
					ptr++;
					if (ptr == 500)
						ptr = 0;
					RMS_Ave += RMS / Normalize_ham;
					t_ham += RMS;
					if (RMS > max)
					{
						max = RMS;
						CurrentHamPeak = global_counter;
						posptrH = 0;
					}
					if (in == 0)
					{
							state = 0;
						    *rawIntensity = t_ham / counter;
							///////////////////////
							for (i = 0; i<4; i++)
							{
								h_s[i].Ave = h_s[i+1].Ave;
								h_s[i].point = h_s[i+1].point;
								h_s[i].point_mid = h_s[i+1].point_mid;
								h_s[i].Raw_RMS = h_s[i+1].Raw_RMS;
							}
							if (counter > 25)
								h_s[4].Ave = RMS_Ave / counter;
							else
								h_s[4].Ave = 0;
							h_s[4].Raw_RMS = t_ham;
							h_s[4].point = CurrentHamPeak;
							h_s[4].point_mid = CurrentHamPeak;
							ham_detected = 1;
							mid = counter/2 + 68; //68 = 4*62 - 190;
							f6_2 << AngleBuff[mid] << std::endl;
							//h_s[4].point_mid = (global_counter + start) >> 1;
							//ftest << h_s[4].point_mid << std::endl;
					}
					break;
			}
	}
}
