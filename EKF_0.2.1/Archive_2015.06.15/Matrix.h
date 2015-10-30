#ifndef __MATRIX_H__
#define __MATRIX_H__

void QR(float *matrix, float *Q, float *R, int row, int col);
void Matrix_Multiply(float *matrix_1, float *matrix_2, float *output, int row, int com, int col);
void Matrix_Sub(float *matrix_1, float *matrix_2, float *output, int row, int col);
void MatrixInverse(float *ptrIn, float *ptrOut, int size);
void ColumnConcat(float *matrix_1, float *matrix_2, float *matrix_out, int row, int col_1, int col_2);
void RowConcat(float *matrix_1, float *matrix_2, float *matrix_out, int row_1, int col, int row_2);
void Matrixtranspose(float *psrc, float *pout, int srcRow, int srcCol);
void Matrix_Sub(float *matrix_1, float *matrix_2, float *output, int row, int col);
	
#endif
