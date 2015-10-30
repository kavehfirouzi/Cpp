#include "algorithms.h"
#include <fstream>
extern std::ofstream f1, f2, f3, f4, f5;
extern int32_t  Normalize_quad;																 // MVC for quad for normalizing RMS values
extern int32_t  Normalize_ham;																 // MVC for ham to normalize RMS values						
extern uint8_t  Normalizers[8];						                     // Buffer to read the two 2-byte MVC values from flash
extern int32_t  posproc;																			 // Window size for post_processing
extern int32_t c_1;
extern int32_t c_2;
int my_counter = 0;

double Butter_b[5] = {0.965080986344736,  -2.814944037358806,   3.982816114046526,  -2.814944037358806, 0.965080986344736};

double Butter_a[4] = {-2.864980440387021,   3.981596404609088,  -2.764907634330585, 0.931381682126905};

double Butter_b_20[11] = { 0.6689830880565,  -6.6898308805653,   30.1042389625437,  -80.2779705667832,   140.4864484918705,
	-168.5837381902446,   140.4864484918705,  -80.2779705667832,   30.1042389625437,  -6.6898308805653, 0.6689830880565};
double Butter_a_20[10] = {-9.1967361675507,  38.0910588369597,  -93.5627808579634,   150.9311663652504,
	-167.0769732545968,   128.5295680551100,  -67.8479756954957,   23.5200094558486,  -4.8348751090022,   0.4475383721056};

void Butter(double newData, double *Butterout);

void PreProcessing(int RawEMG, float* FilteredEMG, float* SmoothedRMS, float* Detrended, int sel, int action)
{
		float32_t filter_out, RMS_out, LOW2_out, detrend_out, butter_out;
	    float f;
		double alaki;
			detrend_out = 0;
			if (action != 4)
			{
				
			
				Moving_Average_HIGH(&(RawEMG), &filter_out, 0, sel);
				if (sel == 0)
				{
					//Butter(filter_out, &alaki);
					
					ButterQ(filter_out, &alaki);
					butter_out = alaki;
				}
				else
				{
					
					ButterH(filter_out, &alaki);
					butter_out = alaki;
				}
				RMS_f32(&butter_out, &RMS_out, 0, sel, RMS_SIZE);
				
			}
			else
			{
				f = RawEMG;
				//Moving_Average_LOW_1(&f, &LOW2_out, MA_SIZE_LOW3, 0, 1);							// Smoothing the ACC data five times
			}
			*FilteredEMG = butter_out;
			*SmoothedRMS = RMS_out;
			*Detrended = detrend_out;
}

void ButterQ(double newData, double *Butterout)
{
	static double DataBuff[5];
	static double OutBuff[4];
	double sum;
	int j;
	double Data = newData;
	for (j=0 ; j< 4; j++)
	{
		DataBuff[j] = DataBuff[j+1];
	}
	DataBuff[4] = Data;
	sum = 0;
	for (j = 0; j<5; j++)
	{
		sum += DataBuff[j] * Butter_b[j];
	}
	for (j = 0; j<4; j++)
	{
		sum -= OutBuff[j] * Butter_a[j];
	}
	for (j = 4 ; j > 0; j--)
	{
		OutBuff[j] = OutBuff[j-1];
	}
	OutBuff[0] = sum;
	*Butterout =  sum;
}

void ButterH(double newData, double *Butterout)
{
	static double DataBuff[5];
	static double OutBuff[4];
	double sum;
	int j;
	double Data = newData;
	for (j=0 ; j< 4; j++)
	{
		DataBuff[j] = DataBuff[j+1];
	}
	DataBuff[4] = Data;
	sum = 0;
	for (j = 0; j<5; j++)
	{
		sum += DataBuff[j] * Butter_b[j];
	}
	for (j = 0; j<4; j++)
	{
		sum -= OutBuff[j] * Butter_a[j];
	}
	for (j = 4 ; j > 0; j--)
	{
		OutBuff[j] = OutBuff[j-1];
	}
	OutBuff[0] = sum;
	*Butterout =  sum;
}

void Butter(double newData, double *Butterout)
{
	static double DataBuff[11];
	static double OutBuff[10];
	double sum;
	for ( int j=0 ; j< 10; j++)
	{
		DataBuff[j] = DataBuff[j+1];
	}
	DataBuff[10] = (float)newData;
	sum = 0;
	for (int k=0; k<11; k++)
	{
		sum += DataBuff[k] * Butter_b_20[k];
	}
	for (int k=0; k<10; k++)
	{
		sum -= OutBuff[k] * Butter_a_20[k];
	}
	for ( int j=10 ; j > 0; j--)
	{
		OutBuff[j] = OutBuff[j-1];
	}
	OutBuff[0] =  sum;
	*Butterout =  sum;
}