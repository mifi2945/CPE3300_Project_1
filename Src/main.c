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
#include <string.h>
#include "uart_driver.h"
#include "include.h"


#define F_CPU 16000000UL

#define DEBUG 0				// 1 for yes, 0 for no


static int nonblocking_getchar(void) {
	if((*(USART_SR)&(1<<RXNE)) == (1<<RXNE)) {
		char c = ((char) *USART_DR);  // Read character from usart
		usart2_putch(c);  // Echo back

		if (c == '\r'){  // If character is CR
			usart2_putch('\n');  // send it
			c = '\n';   // Return LF. fgets is terminated by LF
		}
		return (int)c;
	}
	return -1;
}

// main
int main(void){
	init_usart2(57600,F_CPU);
	init_protocol();

	int length = 0;
	char string[255];
	char incoming[257];
	// never return
	for(;;){
		// use nonblocking USART get_char() so we can poll for RX
		int ch = nonblocking_getchar();
		int incoming_length = print_rx(&incoming);

		if (incoming_length > -1) {
			printf("Received message: ");
			printf("%s\n", incoming + (DEBUG ? 0 : 2));
			memset(incoming, 0, sizeof(incoming));
		}

		if (ch != -1) {
			char c = (char)ch;
			if (c != '\n') {
				string[length++] = c;
			} else {
				// TODO can add logic to make sure string !> 255
				// but no reason if it is a precondition of project
				transmit(length, string);
				length = 0;
			}
		}
	}
	return 0;
}

