/**
/*
 * protocol.c
 *
 *  Created on: Jan 28, 2026
 *      Author: filippovm, acostal
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
			// TODO can add logic to make sure string !> 255
			// but no reason if it is a precondition of project
			transmit(length, string);
			length = 0;
		}

	}
	return 0;
}

