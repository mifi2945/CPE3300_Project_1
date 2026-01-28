/**
  ******************************************************************************
  * @file		gpio.h
  * @brief		GPIO addresses
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/



#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>

#define F_CPU 16000000UL
#define NULL			((void*)0)

#define GPIOA			0x40020000
#define GPIOB			0x40020400
#define GPIOC			0x40020800
#define GPIOD			0x40020C00

#define RCC_AHB1_ENR	0x40023830
#define RCC_AHB2_ENR	0x40023834
#define RCC_APB1_ENR	0x40023840
#define RCC_APB2_ENR	0x40023844


#define GPIOAEN			(1<<0) //this is in the uart_driver.h file
#define GPIOBEN			(1<<1)
#define GPIOCEN			(1<<2)
#define GPIODEN			(1<<3)

//#define MODER_INDEX		0
//#define OTYPER_INDEX		1
//#define SPEED_INDEX		2
//#define PUPDR_INDEX		3
//#define IDR_INDEX			4
//#define ODR_INDEX			5
//#define BSRR_INDEX		6
//#define LCKR_INDEX		7
//#define AFRL_INDEX		8
//#define AFRH_INDEX		9

typedef struct {
	uint32_t MODER;
	uint32_t OTYPER;
	uint32_t SPEED;
	uint32_t PUPDR;
	uint32_t IDR;
	uint32_t ODR;
	uint32_t BSRR;
	uint32_t LCKR;
	uint32_t AFRL;
	uint32_t AFRH;
} GPIO;

extern volatile GPIO *gpioa;
extern volatile GPIO *gpiob;
extern volatile GPIO *gpioc;
extern volatile GPIO *gpiod;

#endif /* GPIO_H_ */
