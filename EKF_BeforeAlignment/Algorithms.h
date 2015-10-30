#ifndef __ALGORITHMS__
#define __ALGORITHMS__

#include <stdint.h>
#include <stdlib.h>
#include "string.h"

#define ZENSAH   //switch between Zensah and old band

#define RMS_SIZE		19                             // Size of RMS sliding window
#define RMS_SIZE_RUN_HAM		67												 // Size of RMS sliding window for Running Hs,
#define MA_SIZE 9																	 // Size of High-Pass sliding window for raw EMG
#define MA_SIZE_LOW1 19														 // Size of first round of smoothing on RMS
#define MA_SIZE_LOW2 19														 // Size of second round of smoothing on RMS
#define MA_SIZE_LOW3 19														 // Size of smoothing for Accelerometer Walk 
#define DET_MA_SIZE 125														 // Maximum size needed for detrending window
#define LUT1 201
#define LUT2 1000



typedef   float float32_t;
extern const	int32_t test[5000]; //test[1568];  for debugging
extern const int32_t test_run[908];
extern const int32_t test_run_ham[908];
extern const int32_t acc_test[1250];

typedef union                                      // Structure to read raw EMG and accelerometer values from sensors
{
	uint8_t 	data_byte[4];
	int32_t		raw_data;
} RAW_DATA;


struct Ham_Seg																		// A structure used for the Coordination algorithm, explained at coordination function
{
	float Ave;
	float Raw_RMS;
	int point;
	int point_mid;
};

struct EKF_Buffer  //581*4 bytes
{
	float RMSBuff[61];
	float smoothBuff[100];
	float delay2[50];
	int p1;
	int p2;
	int ps;
	float sums;
	float MaxMin[251];
	float Delay[95];
	int ptr3;
	int ptr4;
	int ptr2;
	int ptr;
	float sum;
	float S[9];								// S
	float C[3];                        // initial value of Ck
	float X[3];								// initial state values
};

void SpinScan(uint8_t* rawData, int init);
void SpinScanGyro(uint8_t* rawData, int init);
float sqrt_f32(float f);
void EKF_quad(float RMS, float *Sigma, float *RMS_out, int init);
void EKF_ham(float RMS, float *Sigma, float *RMS_out, int init);
void Unsupervised(unsigned short Sigma, int *Class, int sel);
void PreProcessing(int RawEMG, int RawEMG2, float* FilteredEMG, float* SmoothedRMS, float* Detrended, int sel, int action);
void ExecuteAlgorithms(uint8_t* rawData, uint8_t *Results);
void ExecuteAlgorithmsWalk(uint8_t* rawData, uint8_t* rawDataEMG, uint8_t *Results, int acc);
void Algorithm_Reset(void);
void Moving_Average_HIGH(int32_t *Data, float32_t *MAout, uint32_t init, uint32_t sel);
//////////////////////////////////////////////////////////////////////////////////////////
void Moving_Average_LOW_1(float32_t *Data, float32_t *MAout, uint32_t size, uint32_t init, uint32_t sel);
void Moving_Average_LOW_2(float32_t *Data, float32_t *MAout, uint32_t size, uint32_t init, uint32_t sel);
void Moving_Average_LOW_3(float32_t *Data, float32_t *MAout, uint32_t init, uint32_t sel);
//////////////////////////////////////////////////////////////////////////////////////////
void MVC(uint8_t* rawData, uint8_t *Results, int init);
void RMS_f32(float32_t *Data, float32_t *RMSOut, uint32_t init, uint32_t sel, int size);
void Detrending(float *Data, float32_t *MAout, uint32_t det_size, uint32_t init, uint32_t sel);
void Bayes_all(float32_t in, float32_t RMS, float32_t *RMS_out, int action, int *seg, int init);
void Bayes_Walk(float32_t in, int *seg, int init);
void SegmetnAnalysisQuad(float in, float RMS, uint8_t* cad, uint8_t *intensity, float *rawIntensity, uint8_t *targeting, uint8_t *coordination, uint8_t* IntensityHam, uint32_t global_counter, int init);
void SegmetnAnalysisHam(float in, float RMS, float *rawIntensity, uint32_t global_counter, int init);
void SegmetnAnalysisWalk(float in, float RMS_quad, float RMS_ham, uint8_t* cad, unsigned char *intensity_quad, unsigned char *intensity_ham, unsigned char *targeting, int init);
void PostProcessSeg(int in, int* out, float RMS_quad, float *RMSout_quad, float RMS_ham, float *RMSout_ham, int sel, int init);
void PostProcessGap(int in, int* out, float RMS_quad, float *RMSout_quad, float RMS_ham, float *RMSout_ham, int sel, int init);
void cadenceAccel(uint8_t* rawData, uint8_t *Results, int init);
void cadenceGyro(uint8_t* rawData, uint8_t* rawDataacc, uint8_t *Results, int init);
void PostProcessing(int32_t SegIn, int32_t *SegProcessed, float32_t RMS1_in, float32_t *RMS1_out, float32_t RMS2_in, float32_t *RMS2_out, int sel);
void pWelch_quad(float newData, uint32_t* mean_freq, int start, int init);
void pWelch_quad2(float newData, uint32_t* mean_freq, int start, int init);
void pWelch_ham(float newData, uint32_t* mean_freq, int start, int init);
void pWelch_ham2(float newData, uint32_t* mean_freq, int start, int init);
int AvtiviyRec(uint8_t* rawData);
void EKF_acc(int ACC, float *Sigma, int init);
void ButterQ(double newData, double *Butterout);
void ButterH(double newData, double *Butterout);
void ButterVL(double newData, double *Butterout);
void HammmingSmooth_Quad(float RMS, float *Filtered, float *RMS_out, int init);
void HammmingSmooth_Ham(float RMS, float *Filtered, float *RMS_out, int init);
extern const float32_t exp_lookup_frac[LUT2];
extern const float32_t exp_lookup_int[LUT1];

#endif
