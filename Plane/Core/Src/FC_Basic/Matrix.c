/*
 * Matrix.c
 *
 *  Created on: Jul 27, 2025
 *      Author: twwawy
 *      Email : twwawy37@gmail.com
 */


/* Includes ------------------------------------------------------------------*/
#include <math.h>

#include <FC_Basic/Matrix/Matrix.h>



/* Functions -----------------------------------------------------------------*/
/*
 * Matrix addition function
 * Matrix C is automatically initialized
 * C = A + B
 */
void mat_add(const float *A, const float *B, float *C, int rows, int cols)
{
  for (int i = 0; i < rows; ++i)
  {
    for (int j = 0; j < cols; ++j)
    {
      // C[i][j] → C[i*cols + j]
      C[i*cols + j] = A[i*cols + j] + B[i*cols + j];
    }
  }
}


/**
 * Matrix subtraction function
 * Matrix C is automatically initialized
 * C = A − B
 */
void mat_sub(const float *A, const float *B, float *C, int rows, int cols)
{
  for (int i = 0; i < rows; ++i)
  {
    for (int j = 0; j < cols; ++j)
    {
      // C[i][j] → C[i*cols + j]
      C[i*cols + j] = A[i*cols + j] - B[i*cols + j];
    }
  }
}


/*
 * Matrix multiplication function
 * Matrix C is automatically initialized
 * C = A × B
 */
void mat_mul(const float *A, const float *B, float *C, int rowsA, int colsA, int colsB)
{
  for (int i = 0; i < rowsA; ++i)
  {
    for (int j = 0; j < colsB; ++j)
    {
      float sum = 0.0f;

      for (int k = 0; k < colsA; ++k)
      {
        // A[i][k] → A[i*colsA + k],  B[k][j] → B[k*colsB + j]
        sum += A[i*colsA + k] * B[k*colsB + j];
      }

      // C[i][j] → C[i*colsB + j]
      C[i*colsB + j] = sum;
    }
  }
}


/*
 * Matrix copy function
 * Matrix C is automatically initialized
 * C = A
 */
void mat_copy(const float *A, float *C, int rows, int cols)
{
  for (int i = 0; i < rows; ++i)
  {
    for (int j = 0; j < cols; ++j)
    {
      // A[i][j] → A[i*cols + j]
      // C[i][j] → C[i*cols + j]
      C[i*cols + j] = A[i*cols + j];
    }
  }
}


/*
 * Matrix transpose function
 * Matrix C is automatically initialized
 * C = Aᵀ
 */
void mat_trans(const float *A, float *C, int rowsA, int colsA)
{
  for (int i = 0; i < rowsA; ++i)
  {
    for (int j = 0; j < colsA; ++j)
    {
      // A[i][j] → A[i*cols + j]
      // C[j][i] → C[j*rows + i]
      C[j*rowsA + i] = A[i*colsA + j];
    }
  }
}


/*
 * Matrix inverse function (Gauss–Jordan elimination)
 * Matrix C is automatically initialized
 * C = A⁻¹
 * A: n × n (must be square)
 * C: n × n
 * @retval  1  : success
 * @retval 0  : singular (no inverse)
 */
int mat_inverse(const float *A, float *C, int n)
{
    // 1) working copy of A
    float tmp[n * n];

    // 2) init tmp = A, C = I
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            tmp[i * n + j] = A[i * n + j];
            C  [i * n + j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // 3) Gauss–Jordan elimination
    for (int k = 0; k < n; ++k)
    {
        float pivot = tmp[k * n + k];
        if (fabsf(pivot) < 1e-6f)
            return 0;  // singular

        // normalize pivot row
        for (int j = 0; j < n; ++j)
        {
            tmp[k * n + j] /= pivot;
            C  [k * n + j] /= pivot;
        }

        // eliminate other rows
        for (int i = 0; i < n; ++i)
        {
            if (i == k)
                continue;

            float factor = tmp[i * n + k];
            for (int j = 0; j < n; ++j)
            {
                tmp[i * n + j] -= factor * tmp[k * n + j];
                C  [i * n + j] -= factor * C  [k * n + j];
            }
        }
    }

    return 1;
}
