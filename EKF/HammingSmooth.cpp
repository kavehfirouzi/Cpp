#include "algorithms.h"

#define HammDelay 103 //76 + 25
#define HammLength 151
#define SmoothWin 51

const float HammingWin[HammLength] = {0.00034186,0.000359,0.0003801,0.00040563,0.00043603,0.00047173,0.00051315,0.00056072,0.00061482,0.00067583,0.00074413,0.00082005,0.00090391,0.000996,0.0010966,0.0012059,0.0013242,0.0014516,0.0015883,0.0017344,0.00189,0.0020551,0.0022297,0.0024138,0.0026073,0.0028101,0.0030222,0.0032432,0.003473,0.0037114,0.0039579,0.0042124,0.0044743,0.0047434,0.0050192,0.0053012,0.0055889,0.0058818,0.0061792,0.0064807,0.0067856,0.0070932,0.007403,0.0077141,0.008026,0.0083378,0.0086489,0.0089586,0.009266,0.0095705,0.0098713,0.010168,0.010459,0.010744,0.011022,0.011293,0.011556,0.01181,0.012055,0.012289,0.012513,0.012725,0.012925,0.013113,0.013288,0.013449,0.013596,0.013729,0.013847,0.013951,0.014038,0.01411,0.014167,0.014207,0.014231,0.014239,0.014231,0.014207,0.014167,0.01411,0.014038,0.013951,0.013847,0.013729,0.013596,0.013449,0.013288,0.013113,0.012925,0.012725,0.012513,0.012289,0.012055,0.01181,0.011556,0.011293,0.011022,0.010744,0.010459,0.010168,0.0098713,0.0095705,0.009266,0.0089586,0.0086489,0.0083378,0.008026,0.0077141,0.007403,0.0070932,0.0067856,0.0064807,0.0061792,0.0058818,0.0055889,0.0053012,0.0050192,0.0047434,0.0044743,0.0042124,0.0039579,0.0037114,0.003473,0.0032432,0.0030222,0.0028101,0.0026073,0.0024138,0.0022297,0.0020551,0.00189,0.0017344,0.0015883,0.0014516,0.0013242,0.0012059,0.0010966,0.000996,0.00090391,0.00082005,0.00074413,0.00067583,0.00061482,0.00056072,0.00051315,0.00047173,0.00043603,0.00040563,0.0003801,0.000359,0.00034186
};

void HammmingSmooth_Quad(float RMS, float *Filtered, float *RMS_out, int init)
{
	static float delay[HammDelay];
	static float SmoothBuf[51];
	static int ptrSmooth = 0;
	static float SumSmooth = 0;
	static int ptr1 = 0;
	int ptr1_d;
	static float HammingBuf[HammLength];
	static int ptr2 = 0;
	float *ptrHamm;
	int i;
	float sum;
	
	HammingBuf[ptr2] = RMS;
	ptr2++;
	if (ptr2 == HammLength)
		ptr2 = 0;
	
	sum = 0;
	ptrHamm = (float*)&HammingBuf[ptr2];
	sum = 0;
	for (i = 0; i < HammLength-ptr2; i++)
		sum += ptrHamm[i]*HammingWin[i];
	
	ptrHamm = (float*)&HammingWin[HammLength - ptr2];
	for (i = 0; i<ptr2; i++)
		sum += HammingBuf[i]*ptrHamm[i];
	SumSmooth  = SumSmooth - SmoothBuf[ptrSmooth] + sum;
	SmoothBuf[ptrSmooth] = sum;
	ptrSmooth++;
	if (ptrSmooth == SmoothWin)
		ptrSmooth = 0;
	*Filtered = SumSmooth / SmoothWin;
	
	delay[ptr1] = RMS;
	ptr1_d = ptr1+1;
	ptr1++;
	if (ptr1 == HammDelay)
		ptr1 = 0;
	if (ptr1_d == HammDelay)
		ptr1_d = 0;
	*RMS_out = delay[ptr1_d];
}

void HammmingSmooth_Ham(float RMS, float *Filtered, float *RMS_out, int init)
{
	static float delay[HammDelay];
	static float SmoothBuf[51];
	static int ptrSmooth = 0;
	static float SumSmooth = 0;
	static int ptr1 = 0;
	int ptr1_d;
	static float HammingBuf[HammLength];
	static int ptr2 = 0;
	float *ptrHamm;
	int i;
	float sum;
	
	HammingBuf[ptr2] = RMS;
	ptr2++;
	if (ptr2 == HammLength)
		ptr2 = 0;
	
	sum = 0;
	ptrHamm = (float*)&HammingBuf[ptr2];
	sum = 0;
	for (i = 0; i < HammLength-ptr2; i++)
		sum += ptrHamm[i]*HammingWin[i];
	
	ptrHamm = (float*)&HammingWin[HammLength - ptr2];
	for (i = 0; i<ptr2; i++)
		sum += HammingBuf[i]*ptrHamm[i];
	SumSmooth  = SumSmooth - SmoothBuf[ptrSmooth] + sum;
	SmoothBuf[ptrSmooth] = sum;
	ptrSmooth++;
	if (ptrSmooth == SmoothWin)
		ptrSmooth = 0;
	*Filtered = SumSmooth / SmoothWin;
	
	delay[ptr1] = RMS;
	ptr1_d = ptr1+1;
	ptr1++;
	if (ptr1 == HammDelay)
		ptr1 = 0;
	if (ptr1_d == HammDelay)
		ptr1_d = 0;
	*RMS_out = delay[ptr1_d];
}
