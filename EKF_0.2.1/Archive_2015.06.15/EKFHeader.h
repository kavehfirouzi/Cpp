#ifndef __EKF_H__
#define __EKF_H__

#include <stdint.h>
#include <stdlib.h>
#include "string.h"

struct Ham_Seg																		// A structure used for the Coordination algorithm, explained at coordination function
{
	float Ave;
	int point;
};
typedef   float float32_t;
#define RMS_SIZE 19
#define MA_SIZE 9
void Matrix_Multiply(float *matrix_1, float *matrix_2, float *output, int row, int com, int col);
void MatrixInverse(float *ptrIn, float *ptrOut, int size);
void QR(float *matrix, float *Q, float *R, int row, int col);
void RMS_f32(float32_t *Data, float32_t *RMSOut, uint32_t init, uint32_t sel, int size);
void Moving_Average_HIGH(int32_t *Data, float32_t *MAout, uint32_t init, uint32_t sel);
void EKF(float RMS, float *EKFout);
void SegmetnAnalysisQuad(float in, float RMS, uint8_t* cad, uint8_t *intensity, uint8_t *targeting, uint8_t *coordination, uint32_t global_counter, int init);
void SegmetnAnalysisHam(float in, float RMS, unsigned char *intensity, uint32_t global_counter, int init);
void Moving_Average_HIGH(int32_t *Data, float32_t *MAout, uint32_t init, uint32_t sel);
void RMS_f32(float32_t *Data, float32_t *RMSOut, uint32_t init, uint32_t sel, int size);
void Unsupervised(float Sigma, float RMS, uint8_t *Results, int action, int init);

#endif