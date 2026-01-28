/**
  ******************************************************************************
  * @file		matrix.h
  * @brief		Matrix API Header
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/

#ifndef MATRIX_H_
#define MATRIX_H_



void matrix_init(void);
void matrix_set_pixel(int x, int y);
void matrix_clear_pixel(int x, int y);
void matrix_clear(void);
void matrix_update(void);

#endif /* MATRIX_H_ */
