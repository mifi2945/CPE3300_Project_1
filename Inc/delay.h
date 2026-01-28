/**
  ******************************************************************************
  * @file		delay.h
  * @brief		Delay header
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/

#ifndef DELAY_H_
#define DELAY_H_

#include <stdint.h>

typedef struct {
	volatile uint32_t CTRL;
	volatile uint32_t LOAD;
	volatile uint32_t VAL;
	volatile uint32_t CALIB;
}SYSTICK;

#define SYSTICK_BASE	0xE000E010

#define ENABLE			(1<<0)
#define DISABLE			0
#define PROC_CLKSOURCE	(1<<2)
#define COUNTFLAG		(1<<16)

#define MAX_US			1000000
#define F_CPU 			16000000UL

void delay_us(unsigned int);
void delay_ms(unsigned int);

#endif /* DELAY_H_ */
