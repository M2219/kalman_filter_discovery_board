/* mathlib.h */

#ifndef __MATHLIB_H__
#define __MATHLIB_H__
#define PI 3.14159265
#define INV_MAX_DIM 6

void print_mat(int row, int col, float a[row][col]);

void sliceColArray(int row, int col, float A[row][col], float out[row][1], int target_col);
void sliceRowArray(int row, int col, float A[row][col], float out[1][col], int target_row);
void mulMat(int row1, int col1, float mat1[row1][col1], int row2, int col2, float mat2[row2][col2], float out[row1][col2]);
void addMat(int row, int col, float mat1[row][col], float mat2[row][col], float out[row][col]);
void subMat(int row, int col, float mat1[row][col], float mat2[row][col], float out[row][col]);
void transposeMat(int row, int col, float mat[row][col], float out[row][col]);

float norm(float mean, float std_dev);
float rand_val(int seed);
void randn(int num_samples, float mean, float std_dev, float norm_rv[num_samples]);


void identityMat(int num, float out[num][num]);
float determinant(float [][INV_MAX_DIM], float);
void transpose_inv(float num[INV_MAX_DIM][INV_MAX_DIM], float fac[INV_MAX_DIM][INV_MAX_DIM], int r, float inverse[r][r]);
void cofactor(float num[INV_MAX_DIM][INV_MAX_DIM], int f, float inverse_out[f][f]);
void inverseMat(int k, float mat[k][k], float out[k][k]);


#endif /* __MATHLIB_H__ */
