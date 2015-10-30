#include "Algorithms.h"
#include "matrix.h"
#include <math.h>
#include <fstream>
#define SMOOTHWINDOW 61
#define SMOOTHWINDOW_acc 41
#define MAXMIN 251
float exponent(float f);
extern std::ofstream f1, f2, f3, f4, f5;
const float exp_lookup_frac[1000] = {1.001,1.002,1.003,1.004,1.005,1.006,1.007,1.008,1.009,1.0101,1.0111,1.0121,1.0131,1.0141,1.0151,1.0161,1.0171,1.0182,1.0192,1.0202,1.0212,1.0222,1.0233,1.0243,1.0253,1.0263,1.0274,1.0284,1.0294,1.0305,1.0315,1.0325,1.0336,1.0346,1.0356,1.0367,1.0377,1.0387,1.0398,1.0408,1.0419,1.0429,1.0439,1.045,1.046,1.0471,1.0481,1.0492,1.0502,1.0513,1.0523,1.0534,1.0544,1.0555,1.0565,1.0576,1.0587,1.0597,1.0608,1.0618,1.0629,1.064,1.065,1.0661,1.0672,1.0682,1.0693,1.0704,1.0714,1.0725,1.0736,1.0747,1.0757,1.0768,1.0779,1.079,1.08,1.0811,1.0822,1.0833,1.0844,1.0855,1.0865,1.0876,1.0887,1.0898,1.0909,1.092,1.0931,1.0942,1.0953,1.0964,1.0975,1.0986,1.0997,1.1008,1.1019,1.103,1.1041,1.1052,1.1063,1.1074,1.1085,1.1096,1.1107,1.1118,1.1129,1.114,1.1152,1.1163,1.1174,1.1185,1.1196,1.1208,1.1219,1.123,1.1241,1.1252,1.1264,1.1275,1.1286,1.1298,1.1309,1.132,1.1331,1.1343,1.1354,1.1366,1.1377,1.1388,1.14,1.1411,1.1422,1.1434,1.1445,1.1457,1.1468,1.148,1.1491,1.1503,1.1514,1.1526,1.1537,1.1549,1.156,1.1572,1.1584,1.1595,1.1607,1.1618,1.163,1.1642,1.1653,1.1665,1.1677,1.1688,1.17,1.1712,1.1723,1.1735,1.1747,1.1759,1.177,1.1782,1.1794,1.1806,1.1818,1.1829,1.1841,1.1853,1.1865,1.1877,1.1889,1.1901,1.1912,1.1924,1.1936,1.1948,1.196,1.1972,1.1984,1.1996,1.2008,1.202,1.2032,1.2044,1.2056,1.2068,1.208,1.2092,1.2105,1.2117,1.2129,1.2141,1.2153,1.2165,1.2177,1.219,1.2202,1.2214,1.2226,1.2238,1.2251,1.2263,1.2275,1.2288,1.23,1.2312,1.2324,1.2337,1.2349,1.2361,1.2374,1.2386,1.2399,1.2411,1.2423,1.2436,1.2448,1.2461,1.2473,1.2486,1.2498,1.2511,1.2523,1.2536,1.2548,1.2561,1.2573,1.2586,1.2599,1.2611,1.2624,1.2636,1.2649,1.2662,1.2674,1.2687,1.27,1.2712,1.2725,1.2738,1.2751,1.2763,1.2776,1.2789,1.2802,1.2815,1.2827,1.284,1.2853,1.2866,1.2879,1.2892,1.2905,1.2918,1.293,1.2943,1.2956,1.2969,1.2982,1.2995,1.3008,1.3021,1.3034,1.3047,1.306,1.3073,1.3087,1.31,1.3113,1.3126,1.3139,1.3152,1.3165,1.3178,1.3192,1.3205,1.3218,1.3231,1.3245,1.3258,1.3271,1.3284,1.3298,1.3311,1.3324,1.3338,1.3351,1.3364,1.3378,1.3391,1.3404,1.3418,1.3431,1.3445,1.3458,1.3472,1.3485,1.3499,1.3512,1.3526,1.3539,1.3553,1.3566,1.358,1.3593,1.3607,1.3621,1.3634,1.3648,1.3662,1.3675,1.3689,1.3703,1.3716,1.373,1.3744,1.3758,1.3771,1.3785,1.3799,1.3813,1.3826,1.384,1.3854,1.3868,1.3882,1.3896,1.391,1.3924,1.3938,1.3951,1.3965,1.3979,1.3993,1.4007,1.4021,1.4035,1.4049,1.4064,1.4078,1.4092,1.4106,1.412,1.4134,1.4148,1.4162,1.4176,1.4191,1.4205,1.4219,1.4233,1.4248,1.4262,1.4276,1.429,1.4305,1.4319,1.4333,1.4348,1.4362,1.4376,1.4391,1.4405,1.442,1.4434,1.4448,1.4463,1.4477,1.4492,1.4506,1.4521,1.4535,1.455,1.4564,1.4579,1.4594,1.4608,1.4623,1.4637,1.4652,1.4667,1.4681,1.4696,1.4711,1.4726,1.474,1.4755,1.477,1.4785,1.4799,1.4814,1.4829,1.4844,1.4859,1.4874,1.4888,1.4903,1.4918,1.4933,1.4948,1.4963,1.4978,1.4993,1.5008,1.5023,1.5038,1.5053,1.5068,1.5083,1.5098,1.5113,1.5129,1.5144,1.5159,1.5174,1.5189,1.5204,1.522,1.5235,1.525,1.5265,1.5281,1.5296,1.5311,1.5327,1.5342,1.5357,1.5373,1.5388,1.5403,1.5419,1.5434,1.545,1.5465,1.5481,1.5496,1.5512,1.5527,1.5543,1.5558,1.5574,1.5589,1.5605,1.5621,1.5636,1.5652,1.5667,1.5683,1.5699,1.5715,1.573,1.5746,1.5762,1.5778,1.5793,1.5809,1.5825,1.5841,1.5857,1.5872,1.5888,1.5904,1.592,1.5936,1.5952,1.5968,1.5984,1.6,1.6016,1.6032,1.6048,1.6064,1.608,1.6096,1.6112,1.6128,1.6145,1.6161,1.6177,1.6193,1.6209,1.6226,1.6242,1.6258,1.6274,1.6291,1.6307,1.6323,1.6339,1.6356,1.6372,1.6389,1.6405,1.6421,1.6438,1.6454,1.6471,1.6487,1.6504,1.652,1.6537,1.6553,1.657,1.6586,1.6603,1.662,1.6636,1.6653,1.667,1.6686,1.6703,1.672,1.6736,1.6753,1.677,1.6787,1.6803,1.682,1.6837,1.6854,1.6871,1.6888,1.6905,1.6922,1.6938,1.6955,1.6972,1.6989,1.7006,1.7023,1.704,1.7057,1.7074,1.7092,1.7109,1.7126,1.7143,1.716,1.7177,1.7194,1.7212,1.7229,1.7246,1.7263,1.7281,1.7298,1.7315,1.7333,1.735,1.7367,1.7385,1.7402,1.7419,1.7437,1.7454,1.7472,1.7489,1.7507,1.7524,1.7542,1.7559,1.7577,1.7594,1.7612,1.763,1.7647,1.7665,1.7683,1.77,1.7718,1.7736,1.7754,1.7771,1.7789,1.7807,1.7825,1.7843,1.786,1.7878,1.7896,1.7914,1.7932,1.795,1.7968,1.7986,1.8004,1.8022,1.804,1.8058,1.8076,1.8094,1.8112,1.813,1.8148,1.8167,1.8185,1.8203,1.8221,1.8239,1.8258,1.8276,1.8294,1.8313,1.8331,1.8349,1.8368,1.8386,1.8404,1.8423,1.8441,1.846,1.8478,1.8497,1.8515,1.8534,1.8552,1.8571,1.8589,1.8608,1.8626,1.8645,1.8664,1.8682,1.8701,1.872,1.8739,1.8757,1.8776,1.8795,1.8814,1.8833,1.8851,1.887,1.8889,1.8908,1.8927,1.8946,1.8965,1.8984,1.9003,1.9022,1.9041,1.906,1.9079,1.9098,1.9117,1.9136,1.9155,1.9175,1.9194,1.9213,1.9232,1.9251,1.9271,1.929,1.9309,1.9329,1.9348,1.9367,1.9387,1.9406,1.9425,1.9445,1.9464,1.9484,1.9503,1.9523,1.9542,1.9562,1.9581,1.9601,1.9621,1.964,1.966,1.968,1.9699,1.9719,1.9739,1.9759,1.9778,1.9798,1.9818,1.9838,1.9858,1.9877,1.9897,1.9917,1.9937,1.9957,1.9977,1.9997,2.0017,2.0037,2.0057,2.0077,2.0097,2.0117,2.0138,2.0158,2.0178,2.0198,2.0218,2.0238,2.0259,2.0279,2.0299,2.032,2.034,2.036,2.0381,2.0401,2.0421,2.0442,2.0462,2.0483,2.0503,2.0524,2.0544,2.0565,2.0585,2.0606,2.0627,2.0647,2.0668,2.0689,2.0709,2.073,2.0751,2.0772,2.0792,2.0813,2.0834,2.0855,2.0876,2.0897,2.0917,2.0938,2.0959,2.098,2.1001,2.1022,2.1043,2.1064,2.1085,2.1107,2.1128,2.1149,2.117,2.1191,2.1212,2.1234,2.1255,2.1276,2.1297,2.1319,2.134,2.1361,2.1383,2.1404,2.1426,2.1447,2.1468,2.149,2.1511,2.1533,2.1555,2.1576,2.1598,2.1619,2.1641,2.1663,2.1684,2.1706,2.1728,2.1749,2.1771,2.1793,2.1815,2.1837,2.1858,2.188,2.1902,2.1924,2.1946,2.1968,2.199,2.2012,2.2034,2.2056,2.2078,2.21,2.2122,2.2144,2.2167,2.2189,2.2211,2.2233,2.2255,2.2278,2.23,2.2322,2.2345,2.2367,2.2389,2.2412,2.2434,2.2457,2.2479,2.2502,2.2524,2.2547,2.2569,2.2592,2.2614,2.2637,2.266,2.2682,2.2705,2.2728,2.275,2.2773,2.2796,2.2819,2.2842,2.2864,2.2887,2.291,2.2933,2.2956,2.2979,2.3002,2.3025,2.3048,2.3071,2.3094,2.3117,2.3141,2.3164,2.3187,2.321,2.3233,2.3257,2.328,2.3303,2.3326,2.335,2.3373,2.3396,2.342,2.3443,2.3467,2.349,2.3514,2.3537,2.3561,2.3584,2.3608,2.3632,2.3655,2.3679,2.3703,2.3726,2.375,2.3774,2.3798,2.3821,2.3845,2.3869,2.3893,2.3917,2.3941,2.3965,2.3989,2.4013,2.4037,2.4061,2.4085,2.4109,2.4133,2.4157,2.4181,2.4206,2.423,2.4254,2.4278,2.4303,2.4327,2.4351,2.4376,2.44,2.4424,2.4449,2.4473,2.4498,2.4522,2.4547,2.4571,2.4596,2.4621,2.4645,2.467,2.4695,2.4719,2.4744,2.4769,2.4794,2.4818,2.4843,2.4868,2.4893,2.4918,2.4943,2.4968,2.4993,2.5018,2.5043,2.5068,2.5093,2.5118,2.5143,2.5168,2.5193,2.5219,2.5244,2.5269,2.5294,2.532,2.5345,2.537,2.5396,2.5421,2.5447,2.5472,2.5498,2.5523,2.5549,2.5574,2.56,2.5625,2.5651,2.5677,2.5702,2.5728,2.5754,2.578,2.5805,2.5831,2.5857,2.5883,2.5909,2.5935,2.5961,2.5987,2.6013,2.6039,2.6065,2.6091,2.6117,2.6143,2.6169,2.6195,2.6222,2.6248,2.6274,2.63,2.6327,2.6353,2.6379,2.6406,2.6432,2.6459,2.6485,2.6512,2.6538,2.6565,2.6591,2.6618,2.6645,2.6671,2.6698,2.6725,2.6751,2.6778,2.6805,2.6832,2.6859,2.6885,2.6912,2.6939,2.6966,2.6993,2.702,2.7047,2.7074,2.7101,2.7129,2.7156,2.7183
};
const float exp_lookup_int[201] = {3.78351e-044,1.00893e-043,2.74654e-043,7.46892e-043,2.03048e-042,5.52112e-042,1.50079e-041,4.0796e-041,1.1089e-040,3.0144e-040,8.19399e-040,2.2274e-039,6.0546e-039,1.6458e-038,4.4738e-038,1.2161e-037,3.3057e-037,8.9858e-037,2.4426e-036,6.6397e-036,1.8049e-035,4.9061e-035,1.3336e-034,3.6251e-034,9.8542e-034,2.6786e-033,7.2813e-033,1.9793e-032,5.3802e-032,1.4625e-031,3.9754e-031,1.0806e-030,2.9375e-030,7.9849e-030,2.1705e-029,5.9001e-029,1.6038e-028,4.3596e-028,1.1851e-027,3.2213e-027,8.7565e-027,2.3803e-026,6.4702e-026,1.7588e-025,4.7809e-025,1.2996e-024,3.5326e-024,9.6027e-024,2.6103e-023,7.0955e-023,1.9287e-022,5.2429e-022,1.4252e-021,3.874e-021,1.0531e-020,2.8625e-020,7.7811e-020,2.1151e-019,5.7495e-019,1.5629e-018,4.2484e-018,1.1548e-017,3.1391e-017,8.533e-017,2.3195e-016,6.3051e-016,1.7139e-015,4.6589e-015,1.2664e-014,3.4425e-014,9.3576e-014,2.5437e-013,6.9144e-013,1.8795e-012,5.1091e-012,1.3888e-011,3.7751e-011,1.0262e-010,2.7895e-010,7.5826e-010,2.0612e-009,5.6028e-009,1.523e-008,4.1399e-008,1.1254e-007,3.059e-007,8.3153e-007,2.2603e-006,6.1442e-006,1.6702e-005,4.54e-005,0.00012341,0.00033546,0.00091188,0.0024788,0.0067379,0.018316,0.049787,0.13534,0.36788,1,2.7183,7.3891,20.086,54.598,148.41,403.43,1096.6,2981,8103.1,22026,59874,162750,442410,1.2026e+006,3.269e+006,8.8861e+006,2.4155e+007,6.566e+007,1.7848e+008,4.8517e+008,1.3188e+009,3.5849e+009,9.7448e+009,2.6489e+010,7.2005e+010,1.9573e+011,5.3205e+011,1.4463e+012,3.9313e+012,1.0686e+013,2.9049e+013,7.8963e+013,2.1464e+014,5.8346e+014,1.586e+015,4.3112e+015,1.1719e+016,3.1856e+016,8.6593e+016,2.3539e+017,6.3984e+017,1.7393e+018,4.7278e+018,1.2852e+019,3.4934e+019,9.4961e+019,2.5813e+020,7.0167e+020,1.9073e+021,5.1847e+021,1.4093e+022,3.831e+022,1.0414e+023,2.8308e+023,7.6948e+023,2.0917e+024,5.6857e+024,1.5455e+025,4.2012e+025,1.142e+026,3.1043e+026,8.4384e+026,2.2938e+027,6.2351e+027,1.6949e+028,4.6072e+028,1.2524e+029,3.4043e+029,9.2538e+029,2.5154e+030,6.8377e+030,1.8587e+031,5.0524e+031,1.3734e+032,3.7332e+032,1.0148e+033,2.7585e+033,7.4984e+033,2.0383e+034,5.5406e+034,1.5061e+035,4.094e+035,1.1129e+036,3.0251e+036,8.223e+036,2.2352e+037,6.076e+037,1.6516e+038,1.6516e+038,1.6516e+038,1.6516e+038,1.6516e+038,1.6516e+038,1.6516e+038,1.6516e+038,1.6516e+038,
1.6516e+038,1.6516e+038,1.6516e+038,1.6516e+038};

void ColumnConcat(float *matrix_1, float *matrix_2, float *matrix_out, int row, int col_1, int col_2);
void RowConcat(float *matrix_1, float *matrix_2, float *matrix_out, int row_1, int col, int row_2);
void Matrixtranspose(float *psrc, float *pout, int srcRow, int srcCol);

void EKF_quad(float RMS, float *Sigma, float *RMS_out, int init)
{
	static float RMSBuff[SMOOTHWINDOW];
	static float smoothBuff[100]; //= (float*)(0x10000FA0);
	static float delay2[50]; // = (float*)(0x10001130);
	static int p1 = 0;
	static int p2 = 1;
	static int ps = 0;
	static float sums = 0;
	static float MaxMin[251];
	static float Delay[95];
	static int ptr3 = 0;
	static int ptr4 = 1;
	static int ptr2 = 0;
	static int ptr5;
	static int ptr = 0;
	static float sum = 0;
	float mean;
	const float A[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};								// A matrix
	float Bneg[9] = {-100, 0, 0, 0, -0.01, 0, 0, 0, -0.1};					// -B
	static float S[9]= {1, 0, 0, 0, 1, 0, 0, 0, 1};								// Sk
	static float tempMAT_1[18];
	static float tempMAT_2[18];
	static float tempMAT_3[18];
	static float Q[36];
	static float R[18];
	static float Rinv_1[9];
	static float Rinv_2[9];
	static float C[3] = {9.607894e-01 , -7.686316e+02 , 1};                        // initial value of Ck
	float Dinv  = 1/5000.0;					 
	static float X[3] = {2000, 2.000000e-01, 100};								// initial state values
	const float zeros_1[3] = {0,0,0};
	float zeros_2[4] = {0,0,0,1/5000.0};
	float max = -1000000;
	float min = +1000000;
	static float observ;
	int i, k;
	if (init == 0)
	{
		Delay[ptr3] = RMS;
		sum = sum - RMSBuff[ptr] + Delay[ptr4];											                      // Finding the mea
		mean = sum / SMOOTHWINDOW;
		RMSBuff[ptr] =  Delay[ptr4];
		ptr3++;
		if (ptr3 == 95)
			ptr3 = 0;
		ptr4++;
		if (ptr4 == 95)
			ptr4 = 0;
		k = ptr + (SMOOTHWINDOW-1)/2;
		if (k>SMOOTHWINDOW-1)
			k -= SMOOTHWINDOW;
		ptr++;
		if (ptr == SMOOTHWINDOW)
			ptr = 0;

		MaxMin[ptr2] = RMS;
		ptr5 = ptr2 + 125;
		if (ptr5 >= 251)
			ptr5 -= 251;
		ptr2++;
		if (ptr2 == 251)
			ptr2 = 0;
		for (i =0; i < 251 ; i++)														// Shifting the values inside the buffer and finding the maximum and minimum 
		{
			if (max < MaxMin[i])
				max = MaxMin[i];
			if (min > MaxMin[i])
				min = MaxMin[i];
		}
		if (MaxMin[ptr5] >  ((max+min)/2+min)/2)
		{
			Bneg[4] = -0.0001;
			Dinv  = 1/6000.0;		
			zeros_2[3] = Dinv;
		}
		else
		{
			Bneg[4] = -0.5;
			Dinv  = 1/5000.0;	
			zeros_2[3] = Dinv;
		}
		//////////////////////////////
		Matrix_Multiply((float*)A, S, tempMAT_1, 3, 3, 3);						            // tempMAT_1, 3 = A*Sk;
		ColumnConcat(tempMAT_1, (float*)Bneg, tempMAT_2, 3, 3, 3);					      // tempMAT_2 = tempmat = [A*Sk, -B];
		Matrixtranspose(tempMAT_2, tempMAT_3, 3, 6);							              	// tempMAT_3 = tempmat = tempmat';
		QR(tempMAT_3, Q, R, 6, 3);												                        // [Q R] = [Qb, Rb] = QR(tempmat);
		MatrixInverse(R, Rinv_2, 3);										                      		// R_inv2 = Rb^-1;
		for (i = 0; i<3; i++)												                              // D^1*Ck;
			C[i] *= Dinv;
		RowConcat(Rinv_2, C, tempMAT_1, 3, 3, 1);								                  // tempMAT_1 = tempmat[Rb^-1;-D^1*Ck];
		for (i = 0; i<16; i++)													                          // Initializing Q
		{
			if ( i % 5 == 0)
				Q[i] = 1;
			else
				Q[i] = 0;
		}
		QR(tempMAT_1, Q, R, 4, 3);												                        // [Qa, Ra] = QR[tempmat];
		MatrixInverse(R, Rinv_1, 3);											                        // Rinv_1 = Ra^-1;
		ColumnConcat(Rinv_1, (float*)zeros_1, tempMAT_1, 3, 3, 1);					      // tempMAT_1 = [Ra^-1,zeros(3,3)]
		Matrixtranspose(Q, tempMAT_3, 4, 4);										                  // tempMAT_3 = Q'
		Matrix_Multiply(tempMAT_1, tempMAT_3, tempMAT_2, 3, 4, 4);					      // tempMAT_2 = [Ra^-1,zeros(3,3)]*Q'
		ColumnConcat(Rinv_2, (float*)zeros_1, tempMAT_1, 3, 3, 1);					      // tempMAT_1 = [Rb^-1,zeros(3,3)]
		RowConcat(tempMAT_1, (float*)zeros_2, tempMAT_3, 3, 4, 1);					      // tempMAT_3 = [(Rb^-1,zeros(3,3));(zeros(1,3),D^-1)]
		Matrix_Multiply(tempMAT_2, tempMAT_3, tempMAT_1, 3, 4, 4);				        // tempMAT_1 = M
		observ = mean - 2*X[0]*(X[1]*X[1])*exponent(-1*X[1]*X[1]);                // observ = y = mean - -2*alpha*d_sigma^2*exp(-d_sigma^2);
		Matrix_Multiply((float*)A, X, tempMAT_2, 3, 3, 1);						           	// tempMAT_2 = A*Xk';
		RowConcat(tempMAT_2, &observ, tempMAT_3, 3, 1, 1);							          // tempMAT_3 = [A*Xk';y];
		Matrix_Multiply(tempMAT_1, tempMAT_3, tempMAT_2, 3, 4, 1);				      	// tempMAT_2 = M*[A*Xk';y];

		X[0] = tempMAT_2[0];														                          // Updating states
		X[1] = tempMAT_2[1];
		X[2] = tempMAT_2[2];


		for (i = 0; i < 9 ; i++)												                          // Sk = Ra^-1
			S[i] = Rinv_1[i];

		C[0] = exponent(-1*X[1]*X[1]);
		C[1] = -2*X[0]*X[1]*exponent(-1*X[1]*X[1]);
		C[2] = 1;
		
		//*Sigma = X[0]*exponent(-1*X[1]*X[1]) + X[2];
		sums = sums - smoothBuff[ps] + X[0]*exponent(-1*X[1]*X[1]) + X[2];		
		*Sigma = sums / 100;
		 smoothBuff[ps] = X[0]*exponent(-1*X[1]*X[1]) + X[2];		
		ps++;
		if (ps == 100)
			ps = 0;
		delay2[p1] = RMSBuff[k];
		*RMS_out = delay2[p2];
		p1++;
		if (p1 == 50)
			p1 = 0;
		p2++;
		if (p2 == 50)
			p2 = 0;
		//f4 << *Sigma;
	}
}

void EKF_ham(float RMS, float *Sigma, float *RMS_out, int init)
{
	static float RMSBuff[SMOOTHWINDOW];
	static float smoothBuff[100]; //= (float*)(0x10000FA0);
	static float delay2[50]; // = (float*)(0x10001130);
	static int p1 = 0;
	static int p2 = 1;
	static int ps = 0;
	static float sums = 0;
	static float MaxMin[251];
	static float Delay[85];
	static int ptr3 = 0;
	static int ptr4 = 1;
	static int ptr2 = 0;
	static int ptr5;
	static int ptr = 0;
	static float sum = 0;
	float mean;
	const float A[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};								// A matrix
	float Bneg[9] = {-100, 0, 0, 0, -0.01, 0, 0, 0, -0.1};					// -B
	static float S[9]= {1, 0, 0, 0, 1, 0, 0, 0, 1};								// Sk
	static float tempMAT_1[18];
	static float tempMAT_2[18];
	static float tempMAT_3[18];
	static float Q[36];
	static float R[18];
	static float Rinv_1[9];
	static float Rinv_2[9];
	static float C[3] = {9.607894e-01 , -7.686316e+02 , 1};                        // initial value of Ck
	float Dinv  = 1/5000.0;					 
	static float X[3] = {2000, 2.000000e-01, 100};								// initial state values
	const float zeros_1[3] = {0,0,0};
	float zeros_2[4] = {0,0,0,1/5000.0};
	float max = -1000000;
	float min = +1000000;
	static float observ;
	int i, k;
	if (init == 0)
	{
		Delay[ptr3] = RMS;
		sum = sum - RMSBuff[ptr] + Delay[ptr4];											                      // Finding the mea
		mean = sum / SMOOTHWINDOW;
		RMSBuff[ptr] =  Delay[ptr4];
		ptr3++;
		if (ptr3 == 85)
			ptr3 = 0;
		ptr4++;
		if (ptr4 == 85)
			ptr4 = 0;
		k = ptr + (SMOOTHWINDOW-1)/2;
		if (k>SMOOTHWINDOW-1)
			k -= SMOOTHWINDOW;
		ptr++;
		if (ptr == SMOOTHWINDOW)
			ptr = 0;

		MaxMin[ptr2] = RMS;
		ptr5 = ptr2 + 125;
		if (ptr5 >= 251)
			ptr5 -= 251;
		ptr2++;
		if (ptr2 == 251)
			ptr2 = 0;
		for (i =0; i < 251 ; i++)														// Shifting the values inside the buffer and finding the maximum and minimum 
		{
			if (max < MaxMin[i])
				max = MaxMin[i];
			if (min > MaxMin[i])
				min = MaxMin[i];
		}
		if (MaxMin[ptr5] >  ((max+min)/2+min)/2)
		{
			Bneg[4] = -0.0001;
			Dinv  = 1/6000.0;		
			zeros_2[3] = Dinv;
		}
		else
		{
			Bneg[4] = -0.5;
			Dinv  = 1/5000.0;	
			zeros_2[3] = Dinv;
		}
		//////////////////////////////
		Matrix_Multiply((float*)A, S, tempMAT_1, 3, 3, 3);						            // tempMAT_1, 3 = A*Sk;
		ColumnConcat(tempMAT_1, (float*)Bneg, tempMAT_2, 3, 3, 3);					      // tempMAT_2 = tempmat = [A*Sk, -B];
		Matrixtranspose(tempMAT_2, tempMAT_3, 3, 6);							              	// tempMAT_3 = tempmat = tempmat';
		QR(tempMAT_3, Q, R, 6, 3);												                        // [Q R] = [Qb, Rb] = QR(tempmat);
		MatrixInverse(R, Rinv_2, 3);										                      		// R_inv2 = Rb^-1;
		for (i = 0; i<3; i++)												                              // D^1*Ck;
			C[i] *= Dinv;
		RowConcat(Rinv_2, C, tempMAT_1, 3, 3, 1);								                  // tempMAT_1 = tempmat[Rb^-1;-D^1*Ck];
		for (i = 0; i<16; i++)													                          // Initializing Q
		{
			if ( i % 5 == 0)
				Q[i] = 1;
			else
				Q[i] = 0;
		}
		QR(tempMAT_1, Q, R, 4, 3);												                        // [Qa, Ra] = QR[tempmat];
		MatrixInverse(R, Rinv_1, 3);											                        // Rinv_1 = Ra^-1;
		ColumnConcat(Rinv_1, (float*)zeros_1, tempMAT_1, 3, 3, 1);					      // tempMAT_1 = [Ra^-1,zeros(3,3)]
		Matrixtranspose(Q, tempMAT_3, 4, 4);										                  // tempMAT_3 = Q'
		Matrix_Multiply(tempMAT_1, tempMAT_3, tempMAT_2, 3, 4, 4);					      // tempMAT_2 = [Ra^-1,zeros(3,3)]*Q'
		ColumnConcat(Rinv_2, (float*)zeros_1, tempMAT_1, 3, 3, 1);					      // tempMAT_1 = [Rb^-1,zeros(3,3)]
		RowConcat(tempMAT_1, (float*)zeros_2, tempMAT_3, 3, 4, 1);					      // tempMAT_3 = [(Rb^-1,zeros(3,3));(zeros(1,3),D^-1)]
		Matrix_Multiply(tempMAT_2, tempMAT_3, tempMAT_1, 3, 4, 4);				        // tempMAT_1 = M
		observ = mean - 2*X[0]*(X[1]*X[1])*exponent(-1*X[1]*X[1]);                // observ = y = mean - -2*alpha*d_sigma^2*exp(-d_sigma^2);
		Matrix_Multiply((float*)A, X, tempMAT_2, 3, 3, 1);						           	// tempMAT_2 = A*Xk';
		RowConcat(tempMAT_2, &observ, tempMAT_3, 3, 1, 1);							          // tempMAT_3 = [A*Xk';y];
		Matrix_Multiply(tempMAT_1, tempMAT_3, tempMAT_2, 3, 4, 1);				      	// tempMAT_2 = M*[A*Xk';y];

		X[0] = tempMAT_2[0];														                          // Updating states
		X[1] = tempMAT_2[1];
		X[2] = tempMAT_2[2];


		for (i = 0; i < 9 ; i++)												                          // Sk = Ra^-1
			S[i] = Rinv_1[i];

		C[0] = exponent(-1*X[1]*X[1]);
		C[1] = -2*X[0]*X[1]*exponent(-1*X[1]*X[1]);
		C[2] = 1;
		
		//*Sigma = X[0]*exponent(-1*X[1]*X[1]) + X[2];
		sums = sums - smoothBuff[ps] + X[0]*exponent(-1*X[1]*X[1]) + X[2];		
		*Sigma = sums / 100;
		 smoothBuff[ps] = X[0]*exponent(-1*X[1]*X[1]) + X[2];		
		ps++;
		if (ps == 100)
			ps = 0;
		delay2[p1] = RMSBuff[k];
		*RMS_out = delay2[p2];
		p1++;
		if (p1 == 50)
			p1 = 0;
		p2++;
		if (p2 == 50)
			p2 = 0;
	}
}

void EKF_acc(int ACC, float *Sigma, int init)
{
	static float ACCBuff[SMOOTHWINDOW_acc];
	static int ptr = 0;
	static float sum = 0;
	float mean;
	const float A[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};								// A matrix
	float Bneg[9] = {-100, 0, 0, 0, -0.01, 0, 0, 0, -0.1};					// -B
	static float S[9]= {1, 0, 0, 0, 1, 0, 0, 0, 1};								// Sk
	static float tempMAT_1[18];
	static float tempMAT_2[18];
	static float tempMAT_3[18];
	static float Q[36];
	static float R[18];
	static float Rinv_1[9];
	static float Rinv_2[9];
	static float C[3] = {9.139312e-01, -2.741794e+03, 1};                        // initial value of Ck
	float Dinv  = 1/5000.0;					 
	static float X[3] = {5000, 3.000000e-01, 200};								// initial state values
	const float zeros_1[3] = {0,0,0};
	float zeros_2[4] = {0,0,0,1/5000.0};
	float max = -1000000;
	float min = +1000000;
	static float observ;
	int i;
	if (init == 0)
	{
		sum = sum - ACCBuff[ptr] + ACC;											                      // Finding the mea
		mean = sum / SMOOTHWINDOW_acc;
		ACCBuff[ptr] =  ACC;
		for (i =0; i < SMOOTHWINDOW_acc ; i++)														// Shifting the values inside the buffer and finding the maximum and minimum 
		{
			if (max < ACCBuff[i])
				max = ACCBuff[i];
			if (min > ACCBuff[i])
				min = ACCBuff[i];
		}
		if (ACCBuff[ptr] >  ((max+min)/2+min)/2)
		{
			Bneg[4] = -0.0001;
			Dinv  = 1/5000.0;		
			zeros_2[3] = Dinv;
		}
		else
		{
			Bneg[4] = -0.5;
			Dinv  = 1/3000.0;	
			zeros_2[3] = Dinv;
		}
		ptr++;
		if (ptr == SMOOTHWINDOW_acc)
			ptr = 0;
		//////////////////////////////
		Matrix_Multiply((float*)A, S, tempMAT_1, 3, 3, 3);						            // tempMAT_1, 3 = A*Sk;
		ColumnConcat(tempMAT_1, (float*)Bneg, tempMAT_2, 3, 3, 3);					      // tempMAT_2 = tempmat = [A*Sk, -B];
		Matrixtranspose(tempMAT_2, tempMAT_3, 3, 6);							              	// tempMAT_3 = tempmat = tempmat';
		QR(tempMAT_3, Q, R, 6, 3);												                        // [Q R] = [Qb, Rb] = QR(tempmat);
		MatrixInverse(R, Rinv_2, 3);										                      		// R_inv2 = Rb^-1;
		for (i = 0; i<3; i++)												                              // D^1*Ck;
			C[i] *= Dinv;
		RowConcat(Rinv_2, C, tempMAT_1, 3, 3, 1);								                  // tempMAT_1 = tempmat[Rb^-1;-D^1*Ck];
		for (i = 0; i<16; i++)													                          // Initializing Q
		{
			if ( i % 5 == 0)
				Q[i] = 1;
			else
				Q[i] = 0;
		}
		QR(tempMAT_1, Q, R, 4, 3);												                        // [Qa, Ra] = QR[tempmat];
		MatrixInverse(R, Rinv_1, 3);											                        // Rinv_1 = Ra^-1;
		ColumnConcat(Rinv_1, (float*)zeros_1, tempMAT_1, 3, 3, 1);					      // tempMAT_1 = [Ra^-1,zeros(3,3)]
		Matrixtranspose(Q, tempMAT_3, 4, 4);										                  // tempMAT_3 = Q'
		Matrix_Multiply(tempMAT_1, tempMAT_3, tempMAT_2, 3, 4, 4);					      // tempMAT_2 = [Ra^-1,zeros(3,3)]*Q'
		ColumnConcat(Rinv_2, (float*)zeros_1, tempMAT_1, 3, 3, 1);					      // tempMAT_1 = [Rb^-1,zeros(3,3)]
		RowConcat(tempMAT_1, (float*)zeros_2, tempMAT_3, 3, 4, 1);					      // tempMAT_3 = [(Rb^-1,zeros(3,3));(zeros(1,3),D^-1)]
		Matrix_Multiply(tempMAT_2, tempMAT_3, tempMAT_1, 3, 4, 4);				        // tempMAT_1 = M
		observ = mean - 2*X[0]*(X[1]*X[1])*exponent(-1*X[1]*X[1]);                // observ = y = mean - -2*alpha*d_sigma^2*exp(-d_sigma^2);
		Matrix_Multiply((float*)A, X, tempMAT_2, 3, 3, 1);						           	// tempMAT_2 = A*Xk';
		RowConcat(tempMAT_2, &observ, tempMAT_3, 3, 1, 1);							          // tempMAT_3 = [A*Xk';y];
		Matrix_Multiply(tempMAT_1, tempMAT_3, tempMAT_2, 3, 4, 1);				      	// tempMAT_2 = M*[A*Xk';y];

		X[0] = tempMAT_2[0];														                          // Updating states
		X[1] = tempMAT_2[1];
		X[2] = tempMAT_2[2];


		for (i = 0; i < 9 ; i++)												                          // Sk = Ra^-1
			S[i] = Rinv_1[i];

		C[0] = exponent(-1*X[1]*X[1]);
		C[1] = -2*X[0]*X[1]*exponent(-1*X[1]*X[1]);
		C[2] = 1;
		
		*Sigma = X[0]*exponent(-1*X[1]*X[1]) + X[2];		
		f4 << *Sigma << std::endl;
		//f2 << ACC << std::endl;
		//f2 << ACC << std::endl;

	}
}



float exponent(float f)
{
	static int integer,frac;
	static float dummy;
	static float a;
	static float b;
	if (abs(f) < 0.001)
		return 1.0;
	if (f>0 )
	{
		integer = f;
		dummy = abs(f-(float)integer);
		frac = dummy*1000;
	}
	else
	{
		integer = f;
		dummy = 1.0-abs(f-(float)integer);
		frac = dummy*1000;
	}
	integer += 100;
	if (integer > 200)
		integer = 200;
	if (integer < 0)
		integer = 0;
	if (frac > 999)
		frac = 999;
	if (frac < 0)
		frac = 0;
	if (f>0)
		a = exp_lookup_int[integer];
	else
		a = exp_lookup_int[integer]/(float)2.718281828459046;
	b = exp_lookup_frac[frac];
	return a*b;
}
