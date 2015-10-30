#include <iostream>
#include <fstream>
#include <string>
#include <fstream>
#include "algorithms.h"
using namespace std;
extern void ColumnConcat(float *matrix_1, float *matrix_2, float *matrix_out, int row, int col_1, int col_2);
extern void RowConcat(float *matrix_1, float *matrix_2, float *matrix_out, int col, int row_1, int row_2);
extern int read_file(float* data, string filename);
ofstream f1_1, f1_2, f2, f3, f4_1, f4_2, f5, f6, f6_1, f7, f6_2, ftest, X, Y, f1, f4, shake, f8, velseg, GyroAngle, allign, unalligned;
void test_function(float alaki[4][3]);
float exponent(float f);
extern int32_t Normalize_quad;

float max_test(float in);
extern uint8_t  cadence_acc;
void main()
{
   
	int size;
	float *data = new float[500000];
	float *data2 = new float[500000];
	float *data3 = new float[500000];
	float *data4 = new float[500000];
	float *data5 = new float[500000];
	float *data6 = new float[500000];
	float *data7 = new float[500000];
	float *data8 = new float[500000];

	string filename= "newratiotest_q.txt";
	size = read_file(data, filename);

	filename= "newratiotest_h.txt";
	read_file(data2, filename);

	filename= "newratiotest_x.txt";
	read_file(data3, filename);

	filename= "newratiotest_y.txt";
	read_file(data4, filename);

	filename= "newratiotest_gz.txt";
	read_file(data5, filename);

	filename= "newratiotest_gy.txt";
	read_file(data8, filename);

	filename= "newratiotest_gx.txt";
	read_file(data7, filename);

	filename= "newratiotest_vl.txt";
	read_file(data6, filename);

	f1_1.open("segq.txt");
	f1_2.open("segh.txt");
	f2.open("butter_2.txt");
	f1.open("butter_1.txt");
	f3.open("MVC.txt");
	f4_1.open("EKFq.txt");
	f4_2.open("EKFh.txt");
	f5.open("intensity.txt");
	f6.open("Spin_quad.txt");
	f6_2.open("Spin_ham.txt");
	f6_1.open("SpinSeg.txt");
	f7.open("fatigue.txt");
	ftest.open("test.txt");
	X.open("pos_x.txt");
	Y.open("pos_y.txt");
	f4.open("cadence.txt");
	shake.open("shake.txt");
	f8.open("AxisAngle.txt");
	velseg.open("velseg.txt");
	GyroAngle.open("GyroAngle.txt");
	allign.open("alligned.txt");
	unalligned.open("unalligned.txt");
	float out;
	float Sigma, RMS_out;
	int seg_quad;
	unsigned char dummy;
	int alaki = 0;
	int k,z;
	unsigned char Results[100];
	unsigned char rawData[9];
	unsigned int  GyroData[3];
	Algorithm_Reset();

	for (int j = 0; j <1; j++)
	{   
		int ii = 0;
		for (int i = 0; i < 20000; i++)
		{
			
			k = data[i];
			memcpy(rawData, &k, 3);
			k = data2[i];
			memcpy(rawData + 3, &k, 3);
			k = data6[i];
			memcpy(rawData + 6, &k, 3);
			//MVC(rawData, Results, 0);
			ExecuteAlgorithms(rawData, Results);

			if ( (i % 4)  == 0)
			{	
				k = data3[ii+0];
				memcpy(rawData, &k, 2);
				k = data4[ii+0];
				memcpy(rawData+2, &k, 2);
				k = data5[ii+0];
				memcpy(rawData+4, &k, 2);
				cadenceAccel(rawData, &dummy, 0);
				cadenceGyro(rawData, rawData, &dummy, 0);
				//SpinScan(rawData, 0);
				GyroData[0] = data7[ii];
				GyroData[1] = data8[ii];
				GyroData[2] = data5[ii];
				SpinScanGyro(rawData, 0);
				SpinScanGyroAlligned(GyroData, 0);
				//if (cadence_acc == 0)
				//	SpinScan(rawData, 2);
				ii++;
			}
			else
			{
				SpinScanGyro(rawData, 0);
				SpinScanGyroAlligned(GyroData, 0);
			}
				//SpinScan(rawData, 0);
			
		}
	}

	/*float a[9] = {  0.2664638e4,   0.1518687e4,  -0.6949826e4,
					0.1518687e4,   0.1263740e4,  -0.4271110e4,
					-0.6949826e4,  -0.4271110e4,   2.2401793e4};
	float a[9] = {  0.092988926870678,   0.915025911735790,   0.030385272494918,
					0.463489247762243,   0.642741739133104,   0.208470223305320,
					0.009332512027769,   0.001419058202403,   0.454966145002097};
	float b[3];
	float c[9];
	EigenTest(a, b, c);

	for (int i = 0; i < 3; i++)
		cout << b[i] << endl;
	cout << endl;
	for (int i = 0; i < 9; i++)
		cout << c[i] << endl;

	MatrixMultiplyTrans(a, a, c, 3, 3);
	cout << endl;
	for (int i = 0; i < 9; i++)
		cout << c[i] << endl;*/

	f1_1.close();
	f1_2.close();
	f2.close();
	f3.close();
	f4_1.close();
	f4_2.close();
	f5.close();
	f6.close();
	f6_1.close();
	f6_2.close();
	f7.close();
	ftest.close();
	f4.close();
	shake.close();
	f8.close();
	velseg.close();
	GyroAngle.close();
	allign.close();
	unalligned.close();
}

float max_test(float in)
{
	static float SigmaBuff[4];															// Buffer to hold smoothed accelerometer values		
	static float MaxBuff[12];
    static int pnt = 0;	
	static int pnt2 = 0;
	float max = -10000000;
	float min =  10000000;
	float Maximum;
	float Minimum;
	int j, s;
	
	SigmaBuff[pnt++] = in;
	if (pnt == 4)
		pnt = 0;
	for (j=0; j < 4 ; j++)														// Shifting the values inside the buffer and finding the maximum and minimum 
	{
		if (max < SigmaBuff[j])
				max = SigmaBuff[j];
		if (min > SigmaBuff[j])
				min = SigmaBuff[j];
	}
	MaxBuff[pnt2] = max;
	s = pnt2;
	pnt2++;
	if (pnt2 == 12)
		pnt2 = 0;
	Maximum = max;
	for (j = 0; j<3; j++)
	{
		if (MaxBuff[s] > Maximum)
			Maximum = MaxBuff[s];
		s -= 4;
		if (s<0)
			s += 12;
	}
	return Maximum;

}
