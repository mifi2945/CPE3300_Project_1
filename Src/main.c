/**
  ******************************************************************************
  * @file		main.c
  * @brief		Use the
  *
  * Course:		CPE 2610
  * Section:	131
  * Assignment:	Lab Week x
  * Name:		Mikhail Filippov
  * Summary:
  ******************************************************************************
*/



#include <stdio.h>
#include <math.h>
#include "uart_driver.h"
#include "include.h"

#define F_CPU 16000000UL

// main
int main(void){
	init_usart2(57600,F_CPU);

	// never return
	for(;;){

	}
	return 0;
}

