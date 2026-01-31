/*
 * protocol.c
 *
 *  Created on: Jan 28, 2026
 *      Author: filippovm, acostal
 */


#include "gpio.h"
#include "interrupt.h"
#include "timer.h"
#include <stdio.h>

static uint8_t PREAMBLE = 0x55;
static char MESSAGE[255 + 2];
static int curr_char = 0;
static int curr_bit = 7;
static int transmitting = 0; // false

void init_protocol(void) {
	MESSAGE[0] = PREAMBLE;

	volatile uint32_t *rcc_ahb1enr = (uint32_t*) RCC_AHB1_ENR;
	*rcc_ahb1enr |= GPIOBEN;

	gpiob->MODER |= (0b01<<12);
	gpiob->BSRR = 1<<6; // set high


	// Use TIM2_CH2 on PB3, Alternate Mode 1
	volatile uint32_t *rcc_apb1enr = (uint32_t*) RCC_APB1_ENR;
	*rcc_apb1enr |= TIM2_EN;

	gpiob->MODER &= ~(0b11<<3*2); // clear
	gpiob->MODER |= (0b10<<3*2); // PB3 alternate function mode
	gpiob->AFRL &= ~(0b1111<<3*4); // clear
	gpiob->AFRL |= (0b0001<<3*4); // AF1

	tim2->ARR = F_CPU / 1000 - 1; // millisecond
	tim2->DIER |= (1<<2) | 1; // capture/compare and update interrupt enable 2
	tim2->CCMR1 |= (0b001<<12); // OC2M, active level on match
	tim2->CCR2 = F_CPU / 1000 / 2 - 1;

	nvic[ISER0] = 1<<28; //TIM2 is IRQ 28
	tim2->CR1 = 1; // timer enable
}

void transmit(uint8_t length, char* message) {
	MESSAGE[0] = PREAMBLE;
	MESSAGE[1] = length;

	// message
	for(int i = 0; i < length; ++i) {
		MESSAGE[i+2] = message[i];
	}

	curr_char = 0;
	curr_bit = 7;
	transmitting = (length + 2) * 8 * 2 + 1; // +1 for going back to idle
}

void TIM2_IRQHandler(void) {
	uint16_t sr = tim2->SR;
	tim2->SR = ~(111);
	if (transmitting > 1) {
		uint8_t c = MESSAGE[curr_char];
		uint8_t bit = c & (1<<curr_bit);

		// capture event, first bit
		if (sr & (1<<2)) {
			if (bit) {
				gpiob->BSRR = 1<<(6+16); // reset
			} else {
				gpiob->BSRR = 1<<(6); // set
			}
		}
		// update event
		else if (sr & 1) {
			if (bit) {
				gpiob->BSRR = 1<<(6); // set
			} else {
				gpiob->BSRR = 1<<(6 + 16); // reset
			}
			curr_bit--;
		}

		transmitting--;

		// move character
		if (curr_bit == -1) {
			curr_bit = 7;
			curr_char++;
		}
	} else if (transmitting == 1) {
		gpiob->BSRR = 1<<(6); // set idle
		transmitting--;
	}

}
