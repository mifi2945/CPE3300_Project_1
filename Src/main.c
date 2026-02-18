/**
  ******************************************************************************
  * @file      main.c
  * @students: Mikhail Filippov, Lizbeth Acosta
  * @section:  111
  * @instr:    Dr. Varnell
  * @course    CPE 3300
  * @assign    Project 1
  * @brief
  ******************************************************************************
  */


#include <stdio.h>
#include "uart_driver.h"
#include "include.h"

extern char rx_buffer[258];

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
		printf("%d", rx_buffer[1]);
		if (c != '\n') {
			string[length++] = c;
		} else {
			// TODO can add logic to make sure string !> 255
			// but no reason if it is a precondition of project
			transmit(length, string);
			length = 0;
		}

	}
	return 0;
}

