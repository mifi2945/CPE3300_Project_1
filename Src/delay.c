/**
  ******************************************************************************
  * @file		delay.c
  * @brief		Delay API structure
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/


#include "delay.h"

/**
 * @brief delays the program in microseconds using SysTick
 * @param delay - the number of microseconds to delay by
 */
void delay_us(unsigned int delay) {
	SYSTICK *systick = (SYSTICK*)SYSTICK_BASE;

	systick->CTRL = 0;

	systick->LOAD = (F_CPU/1000000 * delay - 1);
	systick->VAL = 0;
	systick->CTRL = PROC_CLKSOURCE | ENABLE;

	while (!(systick->CTRL&COUNTFLAG)) {} // busy wait

	systick->CTRL = DISABLE;
}

/**
 * @brief delays the program in milliseconds using SysTick
 * @param delay - the number of milliseconds to delay by
 */
void delay_ms(unsigned int delay) {
	for (int i = 0; i < delay; ++i) {
		delay_us(1000);
	}
}
