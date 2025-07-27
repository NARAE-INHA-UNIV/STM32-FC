/*
 * Matrix.h
 *
 *  Created on: Jul 27, 2025
 *      Author: twwawy
 *      Email : twwawy37@gmail.com
 */

#ifndef INC_FC_BASIC_MATRIX_MATRIX_H_
#define INC_FC_BASIC_MATRIX_MATRIX_H_


/* Includes ------------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
void mat_copy(const float *A, float *C, int rows, int cols);

void mat_add(const float *A, const float *B, float *C, int rows, int cols);
void mat_sub(const float *A, const float *B, float *C, int rows, int cols);
void mat_mul(const float *A, const float *B, float *C, int rowsA, int colsA, int colsB);
void mat_trans(const float *A, float *C, int rowsA, int colsA);
int mat_inverse(const float *A, float *C, int n);


#endif /* INC_FC_BASIC_MATRIX_MATRIX_H_ */
