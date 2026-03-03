/**
  ******************************************************************************
  * @file      protocol.c
  * @students: Mikhail Filippov, Lizbeth Acosta
  * @section:  111
  * @instr:    Dr. Varnell
  * @course    CPE 3300
  * @assign    Project 1
  * @brief
  ******************************************************************************
  */

#include "protocol.h"
#include "gpio.h"
#include "interrupt.h"
#include "timer.h"
#include "stm32f411.h"
#include <stdio.h>
#include "monitor.h"

static uint8_t PREAMBLE = 0x55;
static char MESSAGE[255 + 2];
static int curr_char = 0;
static int curr_bit = 7;
static int transmitting = 0; // false

extern enum Rx_State curr_state;
uint8_t tx_ok = 1;
int request_retransmit = 0;

static void init_transmit(void) {
	MESSAGE[0] = PREAMBLE;

	// PB6
	gpiob->MODER |= (0b01<<12);
	gpiob->PUPDR &= ~(0b11<<(6*2));
	gpiob->PUPDR |= 0b01<<(6*2);
	gpiob->OTYPER |= 1<<6;
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


void init_protocol(void) {
	volatile uint32_t *rcc_ahb1enr = (uint32_t*) RCC_AHB1_ENR;
	*rcc_ahb1enr |= GPIOBEN;

	init_transmit();
	init_monitor();
}

void allow_tx() {
	tx_ok = 1;
}

void block_tx() {
	if (transmitting) {
		// if we were previously transmitting something, make sure to flag this
		request_retransmit = 1;
	}
	tx_ok = 0;
	transmitting = 0; // block TX
	gpiob->BSRR = 1<<(6); // set idle
}

void retransmit(void) {
	if (request_retransmit) {
		curr_char = 0;
		curr_bit = 7;
		transmitting = (MESSAGE[1] + 2) * 8 * 2 + 1;
		request_retransmit = 0;
	}
}

int transmit(uint8_t length, char* message) {
	// ensure no message is overwritten
	if (!transmitting && tx_ok && curr_state == IDLE) {
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
		return 0;
	}
	// this function will keep getting called until message is successfully transmitted
	return -1;
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
