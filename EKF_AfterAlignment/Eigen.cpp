#include "algorithms.h"
#include "Matrix.h"
#include <math.h>
#include <fstream>

void EigenValues(float *matrix, float* EigenValues);
void EigenVectors(float *matrix, float *EigenVals, float *EigenVectors);
void GaussJordan(float *ptrIn, float *EigenVector);
void QRDecomposition(float *matrix, float* Q, float* R);
void MatrixMultiplyTrans(float *matrix_1, float *matrix_2, float *output, int row, int col); // This is matrix_1'*matrix_2, not matrix_1*matrix_2

void AllignGyroAcc(float *input_acc, float *input_gyro, float *alligned, int blockSize)
{
	float covar[9];
	float Eigens[3];
	float EigenVecs[9];

	MatrixMultiplyTrans(input_gyro, input_gyro, covar, blockSize, 3); // input'*input
	EigenValues(covar, Eigens);
	EigenVectors(covar, Eigens, EigenVecs);
	Matrix_Multiply(input_acc, EigenVecs, alligned, blockSize, 3, 3);
}

void EigenTest(float *input, float *eigens, float* eigenvecs)
{
	EigenValues(input, eigens);
	EigenVectors(input, eigens, eigenvecs);
}

void EigenValues(float *matrix, float* EigenValues)
{
	float Q[9];
	float R[9];
	float matrix_next[9];
    int i, j, h;
    float f;
    float *p1, *p2, *p3;

	for (i = 0; i<9; i++)
		matrix_next[i] = matrix[i];
        
	for (i = 0; i<40; i++)                // 40 iterationss is fast and accurate enough 
	{
		QRDecomposition(matrix_next, Q, R);
        f = 0;
        p1 = &R[0];
        p3 = &matrix_next[0];
        for (h = 0; h < 3; h++)
        {
                p2 = &Q[0];
                for (j = 0; j < 3; j++)  
                {
                        f = p1[0]*p2[0] +p1[1]*p2[1] + p1[2]*p2[2];
                        *p3 = f;
                        p3++;
                        p2 += 3;
                }
                p1 += 3;
        }
	}
	for (i = 0 ; i<3; i++)
		EigenValues[i] = matrix_next[i*3+i];
}

void QRDecomposition(float *matrix, float* Q, float* R)
{
	float *eVectors;
	float *matrixR;
	float vector[3];
	float projected[3];
	int i,j;
    float f = 0;
    float *p1, *p2, *p3, dot1, dot2;

	eVectors = Q;
	matrixR = R;

	for (i = 0; i<3; i++)
	{
        vector[0] = matrix[i+0];
        vector[1] = matrix[i+3];
        vector[2] = matrix[i+6];

		for (j = 0; j<i; j++)
		{
			dot1 = 0; 
			dot2 = 0;
			dot1 += eVectors[j*3]*vector[0] + eVectors[j*3 + 1]*vector[1] + eVectors[j*3 + 2]*vector[2];
			dot2 += eVectors[j*3]*eVectors[j*3] + eVectors[j*3 + 1]*eVectors[j*3 + 1] + eVectors[j*3 + 2]*eVectors[j*3 + 2];
			f = dot1/dot2;
			projected[0] = f*eVectors[j*3];
			projected[1] = f*eVectors[j*3 + 1];
			projected[2] = f*eVectors[j*3 + 2];

            vector[0] -= projected[0];
            vector[1] -= projected[1];
            vector[2] -= projected[2];
		}

        f = 0;
        f += vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2];
        f = sqrtf(f);
        eVectors[i*3] = vector[0]/f;
        eVectors[i*3 + 1] = vector[1]/f;
        eVectors[i*3 + 2] = vector[2]/f;
	}

    p1 = &eVectors[0];
    p3 = &matrixR[0];
    for (i = 0; i < 3; i++)
    {
        p2 = &matrix[0];
        for (j = 0; j < 3; j++)  
        {
            f = p1[0]*p2[0] +p1[1]*p2[3] + p1[2]*p2[6];
            *p3 = f;
            p3++;
            p2++;
        }
        p1 += 3;
    }
}

void EigenVectors(float *matrix, float *EigenVals, float *EigenVectors)
{
	float temp[9];
	float EigenVector[3];
	int i,j,h;

	for (i = 0; i<9; i++)				 // loads into temp matrix
	{
		temp[i] = matrix[i];
	}

	for (i = 0; i<3 ; i++)
	{
		j = 0;
		for (h = 0; h<3; h++)				
		{
			temp[j] -= EigenVals[i];
			j += 4;
		}
		GaussJordan(temp, EigenVector);  // calculate EigenVector
		j = i;
		for (h = 0; h<3; h++)			 // put EigenVector in EigenVetors Matrix
		{
			EigenVectors[j] = EigenVector[h];
			j += 3;
		}
		j = 0;
		for (h = 0;h<3; h++)			 // reset dummy
		{
			temp[j] = matrix[j];
			j += 3+1;
		}
	}
}

void GaussJordan(float *ptrIn, float *EigenVector)
{
	float temp[9];
	float f;
	int i,j,k,ii,iii,row;

	EigenVector[3-1] = 1;

	for (i = 0; i<9; i++)
		temp[i] = ptrIn[i];

	for (i = 1; i<3; i++)				// forward pass
	{
		row = i*3;
		for (j = 0; j<i; j++)
		{
			ii = row+j;
			iii = j*3+j;
			f = -1*temp[ii]/temp[iii];
			for (k = 0; k<3-j; k++)
			{
				temp[ii+k] += f*temp[iii+k];
			}
		}
	}

	for (i = 3-2; i>-1; i--)			// backward pass
	{
		f = 0;
		ii = i*3;
		for (int j = i+1; j<3 ; j++)
		{
			f += temp[ii+j]*EigenVector[j];
		}
		f *= -1;
		EigenVector[i] = f/temp[ii+i];
	}

	f = 0;
	for (i = 0; i<3; i++)				// normalize
		f += EigenVector[i]*EigenVector[i];
	for (i = 0; i<3; i++)
		EigenVector[i] /= sqrtf(f);
}

void MatrixMultiplyTrans(float *matrix_1, float *matrix_2, float *output, int row, int col) // This is A'*B not A*B
{
	int i, j, k;
	float f;

	for (i = 0; i < col; i++)
	{
		for (j = 0; j < col; j++)  
		{
			f = 0;
			for (k = 0; k < row; k++)
			{
				f += matrix_1[i + k*col]*matrix_2[j + k*col];
			}
			output[j + i*col] = f;
		}
	}
}