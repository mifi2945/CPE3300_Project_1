/**
  ******************************************************************************
  * @file		keypad.h
  * @brief		Keypad Header File
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/

#ifndef KEYPAD_H_
#define KEYPAD_H_

#include <stdint.h>

#define KEY_MODER_RESET	(~0xFFFF)

void keypad_init(void);
unsigned int keypad_getkey_noblock(void);
unsigned int keypad_getkey(void);
uint8_t keypad_getchar_noblock(void);
uint8_t keypad_getchar(void);

#endif /* KEYPAD_H_ */
