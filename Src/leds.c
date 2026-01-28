/**
  ******************************************************************************
  * @file		leds.c
  * @brief		Led + GPIO control structure
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/


#include <stdint.h>
#include "leds.h"
#include "gpio.h"
#include "interrupt.h"
#include "timer.h"


unsigned int led_number = 0;
unsigned int led_duty = 0;

static void tim_init(void) {

	// Use TIM2_CH2 on PB3, Alternate Mode 1
	volatile uint32_t *rcc_apb1enr = (uint32_t*) RCC_APB1_ENR;
	*rcc_apb1enr |= TIM2_EN;

	gpiob->MODER &= ~(0b11<<3*2); //clear
	gpiob->MODER |= (0b10<<3*2); //PB1 alternate function mode
	gpiob->AFRL &= ~(0b1111<<3*4);
	gpiob->AFRL |= (0b0001<<3*4); // AF1


	tim2->CCR2 = 0; //turn off
	tim2->ARR = (F_CPU / 1000) - 1; // 1KHz period ok for lights
	tim2->CCMR1 |= (0b110<<12); // OC2M, PWM mode 1

	tim2->DIER |= 0b101;

	nvic[ISER0] = 1<<28; //TIM3 is IRQ 28

	tim2->CR1 = 1;
}

/**
 * @brief Initializes light block on board
 */
void led_init(void) {
	// Use TIM3_CH4 on PB1, Alternate Mode 2
	volatile uint32_t *rcc_ahb1enr = (uint32_t*) RCC_AHB1_ENR;

	*rcc_ahb1enr |= GPIOBEN;
	gpiob->MODER |= (01<<10)|(01<<12)|(01<<14)|(01<<16)|(01<<18)|(01<<20)|
			(01<<24)|(01<<26)|(01<<28)|(01<<30);

	// init interrupt and timer
	//tim_init();
}

void internal_light(unsigned int num) {
	gpiob->ODR &= ~(0b11110111111<<5);
	gpiob->ODR |= (((num & (0b1111<<6))<<1)<<5) | ((num & (0b111111))<<5);
}

/**
 * @brief Lights up the lights on light block in binary
 * @param number - the number to be displayed in binary
 */
void led_light(unsigned int number) {
	led_number = number;
	gpiob->ODR &= ~(0b11110111111<<5);
	gpiob->ODR |= (((number & (0b1111<<6))<<1)<<5) | ((number & (0b111111))<<5);
}

void led_bright(unsigned int duty) {
	led_duty = duty;

	uint32_t period = tim2->ARR + 1;
	tim2->CCR2 = period*duty/100;
}

/*
void TIM2_IRQHandler(void) {
	if (tim2->SR & (1<<2)) {
		tim2->SR = ~(1<<2);
		internal_light(0);
	}

	if (tim2->SR & 0b1) {
		tim2->SR = ~0b1;
		if (led_duty != 0) {
			internal_light(led_number);
		}
	}
}
*/
