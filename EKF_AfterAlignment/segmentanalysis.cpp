#include "algorithms.h"
#include <fstream>
extern std::ofstream f1_1, f2, f3, f4, f5, f6, f6_1, f6_2, X, Y;

#define STOP 250
#define CAD_COEFF 7500																						// If the distance between two peaks is x, the cadence is 60/(x/125) or 7500/x (125 is 500 / 4)
#define EMGACC 78
extern unsigned char cadence;
extern unsigned int PrevQuadPeak;																	// Holds the value of the previous quad peak
extern unsigned int CurrentQuadPeak;															// Holds the value of the current quad peak
extern unsigned int p2p; 																					// Holds the distance between current quad peak and previous quad peak
extern unsigned int H2Q;																					// Holds the distance between quad peak and Ham peak
extern unsigned int ham_detected;
extern unsigned int quad_detected;
extern unsigned int ham_counter;
extern unsigned int pseudo_ham;
float targeting_ham;
extern unsigned int CurrentHamPeak;
float targeting_quad;
extern uint32_t cadence_acc;
struct Ham_Seg h_s[5];
unsigned int quad_peaks[5];
struct Ham_Seg h_s_Intensity[8];
int h_s_count;
extern int cadence_gyro;
extern int Normalize_quad;
extern int Normalize_ham;
extern int my_counter;

extern uint32_t cadence_acc;
unsigned int quadlength = 0;
int Quad_Start = 0;
int Quad_End = 0;
int Ham_Start = 0;
int Ham_End = 0;
int tbuff_quadS[200];
int tbuff_quadE[200];
int tptr = 0;
int tptr2;
int tbuff_hamS[200];
int tbuff_hamE[200];
int tptr3 = 0;
int tptr4;
int GyroQuadS;
int GyroQuadE;
int GyroHamS;
int GyroHamE;
int fuckyou;

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
	static float RMS_Aves[3] = {0,0,0};																 // Buffer to hold the last two intensity values													 // Enables the the incrementing of the above counter. If it's zero, the counter won't increment.
	int i;
	static int point_ham = 0;
	static float intensity_ham = 0;
	static float Ham_Ave[3] = {0,0,0};
	static float Targeting_Ave[3] = {0,0,0};
	float f;
	float max_ham;
	static float coord;
	static unsigned char AngleBuff[500];
	static unsigned int lengthBuff[500];
	static int ptr = 0;
	int angle = 0;
	int mid;
	int testqs, testqe;
	//static int start;
	if (init == 1)																									 // If init = 1, initialize the parameters
	{
		state = 0;																										
		RMS_Aves[0] = 0;
		RMS_Aves[1] = 0;
	}
	else																														 // start Running the algorithms 
	{
			/*if (length != 0 && cadence_acc != 0)
			{
				angle  = (float)current_length/length*180;
				if (angle > 180)
					angle = 0;
			}
			else
				angle = 0;*/
		    testqs = 0;
			testqe = 0;
			GyroQuadS = 0;
			GyroQuadE = 0;
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
						//quad_detected = 0;
						Quad_Start = 1;
						testqs = 1;
						GyroQuadS = 1;
						//start = Very_Global_Counter;
					}
					break;
				case 1:																										 // Running state
					counter++;																							 // If the input is 1, keep incrementing the counter to keep track of the length of the segment
					AngleBuff[ptr] = angle;
					ptr++;
					if (ptr == 500)
						ptr = 0;	
					//RMS_Ave += RMS / Normalize_quad;												 // Accumulate the normalized RMS values
					targeting_quad += RMS;																	 // Same as above, but non-normalized values
					if (RMS > max)                                           // As new RMS comes in, compare it with previous max, update if it's bigger
					{
						max = RMS;
						CurrentQuadPeak = global_counter;											 // Keeping track of the peak
					}
					if (in == 0)																						 // If input is zero, segment has ended, next state will be the reset state.
					{
							testqe = 1;
							GyroQuadE = 1;
							state = 0;
							Quad_End = 1;
							RMS_Ave = targeting_quad / Normalize_quad;
						
							//if (counter > 50)
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
								Ham_Ave[1] = Ham_Ave[2];
								Ham_Ave[2] = intensity_ham;
								f = (Ham_Ave[0] + Ham_Ave[1] + Ham_Ave[2])*255/3;
								//fuckyou =  (float)(Ham_Ave[0] + Ham_Ave[1] + Ham_Ave[2])*100/3 ;
								//f5 << fuckyou << std::endl;
								//if (f > 255)																					// We don't want the intensity to reach 100%. If it's over 100%, cut it to 90%
								//	f = 255;
								*IntensityHam = (uint8_t)f;
								//////////////////////////////
								f = (RMS_Ave / counter);                       				 // Average the sum of RMS values by dividing it by the length of the segment and scale it to 255
								RMS_Aves[0] = RMS_Aves[1];
								RMS_Aves[1] = RMS_Aves[2];
								RMS_Aves[2] = f;																			// Put the latest intensity value inside the averaging buffer
								f = (RMS_Aves[0] + RMS_Aves[1] + RMS_Aves[2])*255/3;									// If more than one segments have been detected, average the latest two values
								if (f > 255)																					// We don't want the intensity to reach 100%. If it's over 100%, cut it to 90%
									f = 255;
								*intensity = (uint8_t)f;		
								//f5 << int((RMS_Aves[0] + RMS_Aves[1] + RMS_Aves[2])*100/3) << std::endl;																											// Find the distance between the current peak and the previous one
								//fuckyou =  (float)(RMS_Aves[0] + RMS_Aves[1] + RMS_Aves[2])*100/3 ;																								// Update the previous peak, the current peak is now the previous peak in the next cycle
								Targeting_Ave[0] = Targeting_Ave[1];
								Targeting_Ave[1] = Targeting_Ave[2];
								targeting_quad /= counter;
								if (targeting_quad != 0  &&  targeting_ham != 0)
									Targeting_Ave[2] = (targeting_quad/(targeting_quad + targeting_ham)*100);
								*targeting = (uint8_t)(Targeting_Ave[0] + Targeting_Ave[1] + Targeting_Ave[2])/3;
								fuckyou = (float)(Targeting_Ave[0] + Targeting_Ave[1] + Targeting_Ave[2])/3;
								if (cadence_gyro == 0)
									*targeting = 0;
								//f5 << (int)fuckyou << std::endl;
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
								//quad_peaks[4] = (Very_Global_Counter + start) >> 1;
								quad_peaks[4] = CurrentQuadPeak;
								PrevQuadPeak = CurrentQuadPeak;
								PrevQuadPeak = CurrentQuadPeak;
								mid = counter/2 + 68; //68 = 4*62 - 190;
								//f6 <<  (float)AngleBuff[mid]*2 << std::endl;
								//quadlength = lengthBuff[mid];
								//quad_detected = 1;
							}
							
					}
					break;
			}
			tbuff_quadS[tptr] = testqs;
			tbuff_quadE[tptr] = testqe;
			tptr2 = tptr + 1;
			if (tptr2 == EMGACC)
				tptr2 = 0;
			tptr++;
			if (tptr == EMGACC)
				tptr = 0;
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

void SegmetnAnalysisHam(float in, float RMS, float *rawIntensity, uint32_t global_counter, int init)
{
	static int state = 0;
	static float max = 0;
	static float RMS_Ave = 0;
	static int counter = 0;
	static float t_ham = 0;
	int i;
	static unsigned char AngleBuff[500];
	static int ptr = 0;
	int angle = 0;
	int mid;
	int tesths, testhe;
//	static int start;
	if (init == 1)
	{
		state = 0;
	}
	else
	{
		    tesths = 0;
			testhe = 0;
			GyroHamS = 0;
			GyroHamE = 0;
			switch(state)
			{
				case 0:
					RMS_Ave = 0;
					t_ham = 0;
					counter = 0;
					max = 0;
					if (in == 1)
					{
						tesths = 1;
						GyroHamS = 1;
						ptr = 1;
						AngleBuff[0] = angle;
						max = RMS;
						state = 1;
						Ham_Start = 1;
						//start = Very_Global_Counter;
					}
					break;
				case 1:
					counter++;
					AngleBuff[ptr] = angle;
					ptr++;
					if (ptr == 500)
						ptr = 0;
					//RMS_Ave += RMS / Normalize_ham;
					t_ham += RMS;
					if (RMS > max)
					{
						max = RMS;
						CurrentHamPeak = global_counter;
					}
					if (in == 0)
					{
							testhe = 1;
							GyroHamE = 1;
							state = 0;
							Ham_End = 1;
						    *rawIntensity = t_ham / counter;
							RMS_Ave = t_ham / 31347;
							//targeting_ham = t_ham;
							///////////////////////
							for (i = 0; i<4; i++)
							{
								h_s[i].Ave = h_s[i+1].Ave;
								h_s[i].point = h_s[i+1].point;
								h_s[i].Raw_RMS = h_s[i+1].Raw_RMS;
								h_s[i].point_mid = h_s[i+1].point_mid;
							}
							if (counter > 25)
								h_s[4].Ave = RMS_Ave / counter;
							else
								h_s[4].Ave = 0;
							h_s[4].Raw_RMS = t_ham/counter;
							h_s[4].point = CurrentHamPeak;
							h_s[4].point_mid = CurrentHamPeak;
							mid = counter/2 + 68; //68 = 4*62 - 190;
							//f6_2 << (float)AngleBuff[mid] << std::endl;
							//ham_detected = 1;
							//h_s[4].point_mid = (Very_Global_Counter + start) >> 1;
					}
					break;
			}
			tbuff_hamS[tptr3] = tesths;
			tbuff_hamE[tptr3] = testhe;
			tptr4 = tptr3 + 1;
			if (tptr4 == EMGACC)
				tptr4 = 0;
			tptr3++;
			if (tptr3 == EMGACC)
				tptr3 = 0;
	}
}
