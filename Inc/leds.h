/**
  ******************************************************************************
  * @file		leds.h
  * @brief		Led + GPIO control header
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/

#ifndef LEDS_H_
#define LEDS_H_


void led_init(void);
void led_light(unsigned int);
void led_bright(unsigned int);

#endif /* LEDS_H_ */
