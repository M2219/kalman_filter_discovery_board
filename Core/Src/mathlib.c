/* mathlib.c */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "mathlib.h"

void print_mat(int row, int col, float a[row][col])
{
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
             printf("%f " , a[i][j]);
        }
        printf("\n");
    }
}

void sliceColArray(int row, int col, float A[row][col], float out[row][1], int target_col)
{
    for (int r = 0; r < row; r++)
    {
        out[r][0] = A[r][target_col - 1];
    }
}

void sliceRowArray(int row, int col, float A[row][col], float out[1][col], int target_row)
{
    for (int c = 0; c < col; c++)
    {
        out[0][c] = A[target_row - 1][c];
    }
}

void mulMat(int row1, int col1, float mat1[row1][col1], int row2, int col2, float mat2[row2][col2], float out[row1][col2])
{

    if (col1 != row2)
    {
        printf("The number of columns in matrix-1 must be equal to the number of rows in matrix-2");
    }
    for (int i = 0; i < row1; i++)
    {
        for (int j = 0; j < col2; j++)
        {
            out[i][j] = 0;
            for (int k = 0; k < row2; k++)
            {
                out[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}

void addMat(int row, int col, float mat1[row][col], float mat2[row][col], float out[row][col])
{
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            out[i][j] = mat1[i][j] + mat2[i][j];
        }
    }
}

void subMat(int row, int col, float mat1[row][col], float mat2[row][col], float out[row][col])
{
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            out[i][j] = mat1[i][j] - mat2[i][j];
        }
    }
}

void transposeMat(int row, int col, float mat[row][col], float out[row][col])
{
    for (int i = 0; i < row; ++i)
    {
        for (int j = 0; j < col; ++j)
        {
            out[j][i] = mat[i][j];
        }
    }
}

void identityMat(int num, float out[num][num])
{
    for (int i = 0; i < num; i++)
    {
        for (int j = 0; j < num; j++)
        {
            if (i == j)
                out[i][j] = 1.0;
            else
                out[i][j] = 0.0;
        }
    }
}

/********************************************************/
/***************** random normal ************************/
/********************************************************/
void randn(int num_samples, float mean, float std_dev, float norm_rv[num_samples])
{
    rand_val((int) 10);

    // Output message and generate interarrival times
    for (int i = 0; i < num_samples; i++)
    {
        norm_rv[i] = norm(mean, std_dev);
    }
}

float norm(float mean, float std_dev)
{
  float   u, r, theta;           // Variables for Box-Muller method
  float   x;                     // Normal(0, 1) rv
  float   norm_rv;               // The adjusted normal rv

  // Generate u
  u = 0.0;
  while (u == 0.0)
    u = rand_val(0);

  r = sqrt(-2.0 * log(u));

  // Generate theta
  theta = 0.0;
  while (theta == 0.0)
    theta = 2.0 * PI * rand_val(0);

  // Generate x value
  x = r * cos(theta);

  // Adjust x value for specified mean and variance
  norm_rv = (x * std_dev) + mean;

  // Return the normally distributed RV value
  return(norm_rv);
}

float rand_val(int seed)
{
  const long  a =      16807;  // Multiplier
  // const long  m = 2147483647;  // Modulus
  const long  q =     127773;  // m div a
  const long  r =       2836;  // m mod a
  // static long x;               // Random int value
  long        x_div_q;         // x divided by q
  long        x_mod_q;         // x modulo q
  long        x_new;           // New x value

  long x = rand();               // Random int value

  // Set the seed if argument is non-zero and then return zero
  if (seed > 0)
  {
    x = seed;
    return(0.0);
  }

  // RNG using integer arithmetic
  x_div_q = x / q;
  x_mod_q = x % q;
  x_new = (a * x_mod_q) - (r * x_div_q);
  if (x_new > 0)
    x = x_new;
  else
    x = x_new + RAND_MAX;

  // Return a random value between 0.0 and 1.0
  return((float) x / RAND_MAX);
}

/********************************************************/
/***************** inverse matrix ***********************/
/********************************************************/

void inverseMat(int k, float mat[k][k], float out[k][k])
{

    float a[INV_MAX_DIM][INV_MAX_DIM], d;
    int i, j;

    for (i = 0; i < k; i++)
    {
        for (j = 0; j < k; j++)
        {
            a[i][j] = mat[i][j];
        }
    }

    d = determinant(a, k);

    if (d == 0)
        printf("\nInverse of Entered Matrix is not possible\n");
    else

    cofactor(a, k, out);

}

float determinant(float a[INV_MAX_DIM][INV_MAX_DIM], float k)
{
    float s = 1, det = 0, b[INV_MAX_DIM][INV_MAX_DIM];
    int i, j, m, n, c;
    if (k == 1)
    {
        return (a[0][0]);
    }

    else
    {
        det = 0;
        for (c = 0; c < k; c++)
        {
            m = 0;
            n = 0;
            for (i = 0; i < k; i++)
            {
                for (j = 0; j < k; j++)
                {
                    b[i][j] = 0;

                    if (i != 0 && j != c)
                    {
                        b[m][n] = a[i][j];

                        if (n < (k - 2))
                            n++;
                        else
                        {
                            n = 0;
                            m++;
                        }
                    }
                }

            }

            det = det + s * (a[0][c] * determinant(b, k - 1));
            s = -1 * s;

        }

    }
    return (det);
}


void transpose_inv(float num[INV_MAX_DIM][INV_MAX_DIM], float fac[INV_MAX_DIM][INV_MAX_DIM], int r, float inverse[r][r])
{
    int i, j;

    float b[INV_MAX_DIM][INV_MAX_DIM], d;

    for (i = 0; i < r; i++)
    {
        for (j = 0; j < r; j++)
        {
            b[i][j] = fac[j][i];
        }
    }

    d = determinant(num, r);

    for (i = 0; i < r; i++)
    {
       for (j = 0; j < r; j++)
       {
           inverse[i][j] = b[i][j] / d;
       }
    }
}

void cofactor(float num[INV_MAX_DIM][INV_MAX_DIM], int f, float inverse_out[f][f])
{
    float b[INV_MAX_DIM][INV_MAX_DIM], fac[INV_MAX_DIM][INV_MAX_DIM];
    int p, q, m, n, i, j;

    for (q = 0; q < f; q++)
    {
        for (p = 0; p < f; p++)
        {
            m = 0;
            n = 0;
            for (i = 0; i < f; i++)
            {
                for (j = 0; j < f; j++)
                {
                    if (i != q && j != p)
                    {
                        b[m][n] = num[i][j];

                        if (n < (f - 2))
                            n++;

                        else
                        {
                            n = 0;
                            m++;
                        }
                    }

                }

            }

            fac[q][p] = pow(-1, q + p) * determinant(b, f - 1);
        }
    }
    transpose_inv(num, fac, f, inverse_out);
}

