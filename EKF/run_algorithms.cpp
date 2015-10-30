#include "algorithms.h"
#include "ekfemg.h"
extern int test_data_counter;
#include <fstream>
extern std::ofstream f1_1, f1_2, f2, f3, f4_1, f4_2, f5;
uint32_t  cadence_acc = 0;                                // Cadence calculated from accelerometer
uint8_t  cadence = 0;																		 // Cadence calculated from EMG
uint32_t global_counter = 0;														 // A global counter that counts the number of samples processed by naive bayes
uint32_t PrevQuadPeak = 0;															 // Location of previous quad peak in time (number of samples from the start)
uint32_t CurrentQuadPeak = 0;														 // Location of latest quad peak in time (number of samples from the start)
uint32_t CurrentHamPeak;																 // Location of previous ham peak in time (number of samples from the start)
uint32_t p2p = 0; 																			 // Distance (number of samples) between the curren quad peak and previous peak
uint32_t H2Q = 0;																				 // The distance between the quad peak and closest Ham peak to it
uint8_t  Intens_Quad = 0;																 // Quad intensity
uint8_t  Intens_Ham = 0;																 // Ham intensity
uint8_t  targetingQ = 0;																 // Targeting (quad as the numerator)
uint8_t  coordinate = 0;                                 // Coordination 
uint32_t ham_detected = 0;															 // A ham segment was detected
uint32_t quad_detected = 0;															 // A quad was detected
uint32_t ham_counter = 0;																 // Number of ham segments detected between two quads
int32_t  posproc;																				 // Window size for post_processing
int32_t	 Rawfiltered1, Rawfiltered2;										 // Filtered EMG for free mode																		 
uint8_t  Normalizers[8] = {0,0,0,0,0,0,0,0};						 // Buffer to read the two 2-byte MVC values from flash
int32_t  Normalize_quad = 0;																 // MVC for quad for normalizing RMS values
int32_t  Normalize_ham = 0;																	 // MVC for ham to normalize RMS values												
int32_t c_1, c_2;																				 // Counters used for downsampling

uint32_t *EKF_Ptr_Q;
uint32_t *EKF_Ptr_H;
extern struct EKF_Buffer *EKF_Quad;
extern struct EKF_Buffer *EKF_Ham;
extern int cadence_gyro;
extern float fuckyou;

/**
 * @brief	  Reset the algorithms to initial states and sets the buffers and other parameters to initial values
 * 				  The only relavant parameter here is the init which should be set to 1 to initialize each method. 
 *				  Other values and pointers are don't-care and dummy data is used
 * @return	nothing
 */				

void Algorithm_Reset(void)
{
	  int init = 1;
	  
		int32_t dummy_int;
		float32_t dummy_float;
		//EKF_quad(0,0,0,1);
		uint8_t dummy_char;
		Moving_Average_HIGH(&dummy_int, &dummy_float, init, 1);
		RMS_f32(&dummy_float, &dummy_float, init, 1, 0);
		 SegmetnAnalysisQuad(dummy_float, dummy_float, &dummy_char, &dummy_char, &dummy_float, &dummy_char, &dummy_char, &dummy_char, dummy_int, init);
		SegmetnAnalysisHam(dummy_float, dummy_float, &dummy_float, dummy_int, init);
		Detrending(&dummy_float, &dummy_float, dummy_int, init, 0);
		cadenceAccel(&dummy_char, &dummy_char, init);
		pWelch_quad(dummy_float, (uint32_t*)&dummy_int, dummy_int, init);
		pWelch_ham(dummy_float, (uint32_t*)&dummy_int, dummy_int, init);
		MVC( &dummy_char, &dummy_char, 1);
		//test_data_counter = 0;
		/**********************
		* Reading the MVC values from Flash memory and changing from big-endian to small-endian
		**********************/
	
		Normalize_quad = 19708;
		cadence = 0;																																// Setting the global pararmeters used by the functions to 0.
		PrevQuadPeak = 0;
		CurrentQuadPeak = 0;
		p2p = 0; 
		H2Q = 0;
		coordinate = 0;
		ham_detected = 0;
		ham_counter = 0;
		c_1 = 0;
		c_2 = 0;

		///////
		EKF_Ptr_Q = new uint32_t[581];//(uint32_t*)malloc(581*4);
		EKF_Ptr_H = new uint32_t[581];//(uint32_t*)malloc(581*4);
		EKF_Quad = (struct EKF_Buffer*)EKF_Ptr_Q;
		EKF_Ham = (struct EKF_Buffer*)EKF_Ptr_H;
		EKF_Quad->X[0] = alpha0; EKF_Quad->X[1] = d_sigma0; EKF_Quad->X[2] = shift0;
		EKF_Quad->S[0] = Sk_1; EKF_Quad->S[1] = Sk_2; EKF_Quad->S[2] = Sk_3; EKF_Quad->S[3] = Sk_4; EKF_Quad->S[4] = Sk_5;
		EKF_Quad->S[5] = Sk_6; EKF_Quad->S[6] = Sk_7; EKF_Quad->S[7] = Sk_8; EKF_Quad->S[8] = Sk_9;
		EKF_Quad->C[0] = C0_1;  EKF_Quad->C[1] = C0_2; EKF_Quad->C[2] = C0_3;
		EKF_Quad->p1 = 0;
		EKF_Quad->p2 = 1;
		EKF_Quad->ps = 0;
		EKF_Quad->sums = 0;
		EKF_Quad->ptr3 = 0;
		EKF_Quad->ptr4 = 1;
		EKF_Quad->ptr2 = 0;
		EKF_Quad->ptr = 0;
		EKF_Quad->sum = 0;
		memset(EKF_Quad->RMSBuff, 0, 61*4);
		memset(EKF_Quad->MaxMin, 0, 251*4);
		memset(EKF_Quad->Delay, 0, 95*4);
		memset(EKF_Quad->delay2, 0, 50*4);
		memset(EKF_Quad->smoothBuff, 0, 100*4);
		///////
}	

/**
 * @brief	Running the Segmentation algorithms for Cycling and Running
 * @param rawData	: pointer to the buffer holding the raw EMG values
 * @param Results : pointer to the bluetooth buffer
 * @param action	: selects between different actions. 0 for Cycling, 1 for Running
 * @return	nothing
 */	
void ExecuteAlgorithms(uint8_t* rawData, uint8_t *Results)
{
	  static float RawIntensityq = 0;
		static float RawIntensityh = 0;
		static int seg_quad = 0; 
	  static int seg_ham = 0; 
		static uint32_t mean_freq_quad = 15;
	  static uint32_t mean_freq_ham = 15;
		uint32_t dummy_2, dummy; 
		float32_t Sigma, filter_out, detrend_out, RMS_out, LOW2_out;
		uint8_t sel;
		uint32_t pnt3;
		RAW_DATA raw, raw2;
	  //float f;
		pnt3 = 0;																																			 // pointer to the quad and ham values in the rawData buffer. Reading Quad value first

	  for (sel = 0; sel<2; sel++)  																									 // for loop to run Quad and Ham algorithms. sel = 0: Quad, sel = 1: Ham
		{
			raw.raw_data = 0;																														 // Conveting big-endian ADC 3-byte input data to small-endian 4-byte data.
			raw2.raw_data = 0;
			raw.data_byte[0] = rawData[pnt3+0];     																   // uncommnet use this for Debuging while reading from memory, no conversion needed.
			raw.data_byte[1] = rawData[pnt3+1];																			 
			raw.data_byte[2] = rawData[pnt3+2];
			if ((rawData[pnt3+2] & 0x80) == 0x00)
				raw.data_byte[3] = 0;																											 // Input data is positive, set the sign to zero.
			else
				raw.data_byte[3] = 0xFF;																									 // Input data is negative, set the sign to oe
			raw2.data_byte[0] = rawData[6];     																   // uncommnet use this for Debuging while reading from memory, no conversion needed.
			raw2.data_byte[1] = rawData[7];																			 
			raw2.data_byte[2] = rawData[8];
			if ((rawData[8] & 0x80) == 0x00)
				raw2.data_byte[3] = 0;																											 // Input data is positive, set the sign to zero.
			else
				raw2.data_byte[3] = 0xFF;	
			PreProcessing((raw.raw_data), (raw2.raw_data), &filter_out, &LOW2_out, &detrend_out, sel, 0);
	
			if (sel == 0)																																	// Fatigue for Quad
			{																			
				//pWelch_quad(filter_out, &mean_freq_quad, seg_quad, 0);	              // Putting the quad fatigue value in the bluethooth buffer
				//memcpy(&Results->QuadHAM_Fatigue[0], (uint8_t*)&mean_freq_quad, 1);                      // Fatigue
			}
			
			if (sel == 1)																																	// Fatigue for Ham
			{
				pWelch_ham(filter_out, &mean_freq_ham, seg_ham, 0);	                  // Putting the ham fatigue value in the bluethooth buffer
				//memcpy(&Results->QuadHAM_Fatigue[1], (uint8_t*)&mean_freq_ham, 1);                       //fatigue
			}
																		
			if (sel == 0) 																																// Calling naive-Bayes segmentation algorithm for Quad
			{
				//EKF_quad(LOW2_out, &Sigma, &RMS_out, 0);
				HammmingSmooth_Quad(LOW2_out, &Sigma, &RMS_out, 0);
				//f4_1 << Sigma << std::endl;
				Unsupervised(Sigma/2, &seg_quad, 0);	
				PostProcessing(seg_quad, &seg_quad, RMS_out, &RMS_out, 0, 0, 0);
				f4_1 << Sigma << std::endl;
				pWelch_quad(filter_out, &mean_freq_quad, seg_quad, 0);	              // Putting the quad fatigue value in the bluethooth buffer
				pWelch_quad2(filter_out, &mean_freq_quad, seg_quad, 0);	              // Putting the quad fatigue value in the bluethooth buffer
				f1_1 << seg_quad << std::endl;
				SegmetnAnalysisQuad(seg_quad, RMS_out, &cadence, &Intens_Quad, &RawIntensityq, &targetingQ, &coordinate, &Intens_Ham, global_counter, 0);		// Finding intensity, targetting and coordination
				//f5 << (float)targetingQ << std::endl;
				if (RawIntensityq > Normalize_quad)																					// Updating MVC
				{
						Normalize_quad = RawIntensityq;
						dummy_2 = Normalize_quad;
						dummy = 0;
						dummy |= ((dummy_2 & 0xff) << 8) | ((dummy_2 & 0xff00) >> 8);           // Sending a 2-byte unsigned integer MVC value to bluetooth buffer. Change from big-endian to small-endian.
						//memcpy(&Results->MVC_IO[0], (uint8_t*)&dummy, 2);
						dummy_2 = Normalize_ham;
						dummy = 0;
						dummy |= ((dummy_2 & 0xff) << 8) | ((dummy_2 & 0xff00) >> 8);           // Sending a 2-byte unsigned integer MVC value to bluetooth buffer. Change from big-endian to small-endian.
						//memcpy(&Results->MVC_IO[2], (uint8_t*)&dummy, 2);
				}
				//f4 << (float)RMS_out << std::endl;
				if (cadence_gyro == 0) 																											// If cadence is 0, set the intesity and targeting values to 0
				{
					fuckyou = 0;
					Intens_Quad = 0;
					Intens_Ham = 0;
					targetingQ = 0;
				//	//SpinScan(rawData, 1);
				}

//				f = (float)(0.01)*cadence_acc*(Intens_Quad+Intens_Ham);
//				Intens_Quad = (uint8_t)f;
				
			}
			if (sel == 1) 																																// Calling naive-Bayes segmentation algorithm for Ham
			{
				//EKF_ham(LOW2_out, &Sigma, &RMS_out, 0);	
			    HammmingSmooth_Ham(LOW2_out, &Sigma, &RMS_out, 0);
				Unsupervised(Sigma/2, &seg_ham, 1);	
				//PostProcessSeg(seg_ham, &seg_ham, 0, 0, RMS_out, &RMS_out, 1, 0);
				//PostProcessGap(seg_ham, &seg_ham, 0, 0, RMS_out, &RMS_out, 1, 0);
				PostProcessing(seg_ham, &seg_ham, 0, 0, RMS_out, &RMS_out, 1);
				f1_2 << seg_ham << std::endl;
				f4_2 << Sigma << std::endl;
				//f1 << RMS_out << std::endl;
				SegmetnAnalysisHam(seg_ham, RMS_out, &RawIntensityh, global_counter, 0);
				if (RawIntensityh > Normalize_ham)																					// Updating MVC
				{
					Normalize_ham = RawIntensityh;
					dummy_2 = Normalize_quad;
					dummy = 0;
					dummy |= ((dummy_2 & 0xff) << 8) | ((dummy_2 & 0xff00) >> 8);             // Sending a 2-byte unsigned integer MVC value to bluetooth buffer. Change from big-endian to small-endian.
					//memcpy(&Results->MVC_IO[0], (uint8_t*)&dummy, 2);
					dummy_2 = Normalize_ham;
					dummy = 0;
					dummy |= ((dummy_2 & 0xff) << 8) | ((dummy_2 & 0xff00) >> 8);             // Sending a 2-byte unsigned integer MVC value to bluetooth buffer. Change from big-endian to small-endian.
					//memcpy(&Results->MVC_IO[2], (uint8_t*)&dummy, 2);
				}
			}
			
			pnt3 += 3;  																																  // Updating the pointer to read Ham raw values from the rawData buffer
			
			
//			dummy_int = filter_out; 																											// Putting Free-mode filtered EMG values in the bluetooth buffer 
//			if (sel == 0)																																	// Quad EMG
//			{
//					Rawfiltered1 = 0;
//					Rawfiltered1 |= ((dummy_int & 0xff) << 8) | ((dummy_int & 0xff00) >> 8);  // Changing small-endian value to big-endian for bluetooth
//			}
//			else																																					// Ham EMG
//			{
//					Rawfiltered2 = 0;
//					Rawfiltered2 |= ((dummy_int & 0xff) << 8) | ((dummy_int & 0xff00) >> 8);	// Changing small-endian value to big-endian for bluetooth
//			}
		}
}

