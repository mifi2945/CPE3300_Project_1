/**
  ******************************************************************************
  * @file		keypad.c
  * @brief		Keypad API implementation
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/


#include <stdint.h>
#include "keypad.h"
#include "gpio.h"
#include "delay.h"


//volatile uint32_t *gpioc = (uint32_t*) GPIOC;
static unsigned int keypad[4][4] = {
			{1, 2, 3, 4},
			{5, 6, 7, 8},
			{9, 10, 11, 12},
			{13, 14, 15, 16}
};
static char char_codes[4][4] = {
			{'1', '2', '3', 'A'},
			{'4', '5', '6', 'B'},
			{'7', '8', '9', 'C'},
			{'E', '0', 'F', 'D'}
};

/**
 * @brief initializes keypad to be used
 */
void keypad_init(void) {
	volatile uint32_t *rcc_enr = (uint32_t*) RCC_AHB1_ENR;
	*rcc_enr |= GPIOCEN;

	// set to general output mode for low and high method scanning
	/*
	gpioc->MODER_INDEX] |= (01<<0*2)|(01<<1*2)|(01<<2*2)|(01<<3*2)|(01<<4*2)|
			(01<<5*2)|(01<<6*2)|(01<<7*2);
			*/
	// configure pull up resistors for method 2
	gpioc->PUPDR |= (01<<0*2)|(01<<1*2)|(01<<2*2)|(01<<3*2)|(01<<4*2)|
			(01<<5*2)|(01<<6*2)|(01<<7*2);
}

/**
 * @brief Scans the keypad once (non-blocking) to check if any keys
 * are pressed
 * @return The numeric code representing the button on the keypad
 * that was pressed, 1-16; otherwise 0 if no button was pressed
 */
unsigned int keypad_getkey_noblock(void) {
	// variable to store numeric position
	int row = -1;
	int col = -1;

	// first, set cols to ODR 0
	// clear bits first as well for row read
	gpioc->MODER &= (KEY_MODER_RESET);
	gpioc->MODER |= (01<<0*2)|(01<<1*2)|(01<<2*2)|(01<<3*2);
	gpioc->ODR &= ~((1<<0)|(1<<1)|(1<<2)|(1<<3));

	// read rows IDR
	for (int i = 0; i < 4; ++i) {
		if(!(gpioc->IDR & (1<<(i+4)))) {
			row = i;
			break;
		}
	}

	// set rows to ODR 0
	// clear bits first as well for col read
	gpioc->MODER &= (KEY_MODER_RESET);
	gpioc->MODER |= (01<<4*2)|(01<<5*2)|(01<<6*2)|(01<<7*2);
	gpioc->ODR &= ~((1<<4)|(1<<5)|(1<<6)|(1<<7));

	// read cols IDR
		for (int i = 0; i < 4; ++i) {
			if(!(gpioc->IDR & (1<<i))) {
				col = i;
				break;
			}
		}

	return (row == -1 || col == -1) ? 0 : keypad[row][col];
}

/**
 * @brief Scans the keypad continuously, waiting for an input (blocking)
 * @return The numeric code representing the button that was pressed
 */
unsigned int keypad_getkey(void) {
	int code;
	while ((code = keypad_getkey_noblock()) == 0) {}
	delay_ms(250);
	return code;
}

/**
 * @brief Gets the ASCII code representation of the key pressed
 * in a non-blocking fashion (scan once)
 * @return ASCII code of the key pressed; -1 if none
 */
uint8_t keypad_getchar_noblock(void) {
	int index = keypad_getkey_noblock();
	if (index == 0) {
		return 0;
	} else {
		return char_codes[(index-1)/4][(index-1)%4];
	}
}

/**
 * @brief Gets the ASCII code representation of the key pressed
 * in a blocking fashion, waiting for an input
 * @return ASCII code of the key pressed
 */
uint8_t keypad_getchar(void) {
	int index = keypad_getkey();
	return char_codes[(index-1)/4][(index-1)%4];
}
