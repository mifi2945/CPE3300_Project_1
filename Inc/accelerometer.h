/**
  ******************************************************************************
  * @file		accelerometer.h
  * @brief		ADXL345 Header
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/

#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_



void accelerometer_init(void);
void get_xyz(int *x, int *y, int *z);


#endif /* ACCELEROMETER_H_ */
