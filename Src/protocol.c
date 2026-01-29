/*
 * protocol.c
 *
 *  Created on: Jan 28, 2026
 *      Author: filippovm, acostal
 */


#include "gpio.h"
#include "interrupt.h"
#include "timer.h"

static uint8_t PREAMBLE = 0x55;

void init_protocol(void) {
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

//	tim2->ARR = ; //TODO
	tim2->DIER |= (1<<2); // capture/compare interrupt enable 2
	tim2->CCMR1 |= (0b001<<12); // OC2M, active level on match
//	tim2->CCR2 =

	nvic[ISER0] = 1<<28; //TIM2 is IRQ 28
	tim2->CR1 = 1;
}

void transmit(int length, char* message) {
	// preamble
	for(int size = 7; size >= 0; --size) {
		if (PREAMBLE & (1<<size)) {
			gpiob->BSRR = 1<<(6+16); // reset
			delay_us(500);
			gpiob->BSRR = 1<<(6); // set
			delay_us(500);
		} else {
			gpiob->BSRR = 1<<(6); // set
			delay_us(500);
			gpiob->BSRR = 1<<(6+16); // reset
			delay_us(500);
		}
	}

	// length
	for(int size = 7; size >= 0; --size) {
		if (length & (1<<size)) {
			gpiob->BSRR = 1<<(6+16); // reset
			delay_us(500);
			gpiob->BSRR = 1<<(6); // set
			delay_us(500);
		} else {
			gpiob->BSRR = 1<<(6); // set
			delay_us(500);
			gpiob->BSRR = 1<<(6+16); // reset
			delay_us(500);
		}
	}

	// message
	for(int i = 0; i < length; ++i) {
		uint8_t c = message[i];
		for(int size = 7; size >= 0; --size) {
			// if bit is 1
			if (c & (1<<size)) {
				gpiob->BSRR = 1<<(6+16); // reset
				delay_us(500);
				gpiob->BSRR = 1<<(6); // set
				delay_us(500);
			} else {
				gpiob->BSRR = 1<<(6); // set
				delay_us(500);
				gpiob->BSRR = 1<<(6+16); // reset
				delay_us(500);
			}
		}
	}
	gpiob->BSRR = 1<<6; // set high
}

void TIM2_IRQHandler(void) {

}
