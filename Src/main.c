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
	init_protocol();

	int length = 0;
	char string[255];
	// never return
	for(;;){
		char c = getchar();
		if (c != '\n') {
			string[length++] = c;
		} else {
			transmit(length, string);
			length = 0;
		}

	}
	return 0;
}

