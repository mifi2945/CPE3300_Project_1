/*
 * protocol.c
 *
 *  Created on: Jan 28, 2026
 *      Author: filippovm, acostal
 */

#include "protocol.h"
#include "gpio.h"
#include "interrupt.h"
#include "timer.h"
#include "stm32f411.h"
#include <stdio.h>

static uint8_t PREAMBLE = 0x55;
static char MESSAGE[255 + 2];
static int curr_char = 0;
static int curr_bit = 7;
static int transmitting = 0; // false

static enum Rx_State curr_state = IDLE;
static void set_state(enum Rx_State state);
static uint8_t rx_bit = 1;

static void init_transmit(void) {
	MESSAGE[0] = PREAMBLE;

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

static void init_monitor(void) {
	// set pins PB0, 1, 2 as IDLE, BUSY, COLISSION outputs
	gpiob->MODER &= ~(0b11<<(0*2));
	gpiob->MODER |= 0b01<<0*2;

	gpiob->MODER &= ~(0b11<<(1*2));
	gpiob->MODER |= 0b01<<1*2;

	gpiob->MODER &= ~(0b11<<(2*2));
	gpiob->MODER |= 0b01<<2*2;

	rx_bit = gpiob->IDR & (1<<4);
	set_state(rx_bit ? IDLE : COLLISION);


	//pb4 for rx alternate function mode
	gpiob->MODER &= ~(0b11<<(4*2));
	gpiob->MODER |= (0b10<<(4*2));

	// AF02
	gpiob->AFRL &= ~(0b1111<<(4*4));
	gpiob->AFRL |= (0b0010<<(4*4));

	//init timer 3 channel 1 for RX
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	tim3->DIER |= (1<<1); // interrupt enable
	tim3->CCMR1 |= 0b01; // input capture
	tim3->CCER |= (1<<0) | (0b1<<1) | (1<<3); // set CC1P = 1 non-inverted both edges set CC1NP to 1

	NVIC->ISER[0] = 1 << (TIM3_IRQn);
	NVIC->IP[TIM3_IRQn] |= 0b0011<<4; // set tim3 less priority than tim4

	tim3->CR1 = 1; // enable timer 3

	//////////////////////////////////////////////////////////
	// init timer 4 for monitor
	// PB7 for TIM4_CH2 alternate function 02
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	gpiob->MODER &= ~(0b11<<7*2); // clear
	gpiob->MODER |= (0b10<<7*2);

	// AF02
	gpiob->AFRL &= ~(0b1111<<(7*4));
	gpiob->AFRL |= (0b0010<<(7*4));

//	tim4->ARR = F_CPU / 1000 - 1; // millisecond
	tim4->DIER |= (1<<2); // capture/compare interrupt enable ch2
	tim4->CCMR1 |= (0b001<<12); // OC2M, active level on match
	tim4->CCR2 = (F_CPU / 10000) * 11 - 1; // 1.1 millisecond

	NVIC->ISER[0] = 1 << (TIM4_IRQn); //TIM4 is IRQ 30
//	tim4->CR1 = 1; // timer
}

void init_protocol(void) {
	volatile uint32_t *rcc_ahb1enr = (uint32_t*) RCC_AHB1_ENR;
	*rcc_ahb1enr |= GPIOBEN;

	init_transmit();
	init_monitor();
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
				// length + 2 bytes in MESSAGE
				// * 8 bits per byte
				// * 2 bits per bit for Manchester
	transmitting = (length + 2) * 8 * 2 + 1; // +1 for going back to idle
}

static void set_state(enum Rx_State state) {
	// set pins
	// PB0 = IDLE, PB1 = BUSY, PB2 = COLISSION
	gpiob->BSRR = 1 << (0 + (state==IDLE ? 0 : 16));
	gpiob->BSRR = 1 << (1 + (state==BUSY ? 0 : 16));
	gpiob->BSRR = 1 << (2 + (state==COLLISION ? 0 : 16));

	// set state variable
	curr_state = state;
}

// transmit handler
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

void TIM3_IRQHandler(void){

	switch (curr_state) {
	case IDLE:
		set_state(BUSY);
		// start timer to count for timeout
		tim4->CNT = 0;
		tim4->CR1 = 1;
		break;
	case BUSY:
		// reset counter since new edge arrived early enough
		tim4->CNT = 0;
		break;
	case COLLISION:

		set_state(BUSY);
		// start timer to count for timeout
		tim4->CNT = 0;
		tim4->CR1 = 1;
		break;
	}
	rx_bit = gpiob->IDR & (1<<4);//need to set it at the end bc tim4 is higher priority
	tim3->SR = ~(1<<1);
}

void TIM4_IRQHandler(void){
	switch (curr_state) {
	case IDLE:
		// we don't care about interrupt if we are idle
		tim4->CR1 = 0;
		break;
	case BUSY:
		// check if rx pb4 are currently high or low
		// if high, this is idle
		if (rx_bit) {
			set_state(IDLE);
		} else {
			set_state(COLLISION);
		}
		break;
	case COLLISION:
		// we don't care if about interrupt we are collision
		tim4->CR1 = 0;
		break;
	}

	// check if we need to regenerate edge event if edge and timer interrupt
	// happened at the same time / really close together
	if (tim3->SR & (1<<1)) {
		tim3->SR = ~(1<<1);
		tim3->EGR = 1<<1;
	}
	tim4->SR = ~(1<<2);
}
