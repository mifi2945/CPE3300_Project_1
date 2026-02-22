/**
  ******************************************************************************
  * @file      monitor.c
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
#include "monitor.h"
#include <stdio.h>
#include <string.h>

static enum Rx_State curr_state = IDLE;
static uint8_t rx_bit = 1;

char rx_buffer[258];
static int rx_length = -5;
static const int micro = F_CPU / 1000000;
static uint8_t received = 0;

void init_monitor(void) {
	memset(rx_buffer, 0, sizeof(rx_buffer));

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

int print_rx(char *buffer) {
	// check for received flag AND that the whole message has transmitted
	// (that is, no errors from transmission device)
	if (received && rx_buffer[rx_length + 2 - 1] != "\0") {
		received = 0;
		strncpy(buffer, rx_buffer, rx_length + 2);
		memset(rx_buffer, 0, sizeof(rx_buffer));
		return rx_length + 2;
	}
	return -1;
}

void set_state(enum Rx_State state) {
	// set pins
	// PB0 = IDLE, PB1 = BUSY, PB2 = COLISSION
	gpiob->BSRR = 1 << (0 + (state==IDLE ? 0 : 16));
	gpiob->BSRR = 1 << (1 + (state==BUSY ? 0 : 16));
	gpiob->BSRR = 1 << (2 + (state==COLLISION ? 0 : 16));

	// set state variable
	curr_state = state;
}

static void read_bit(uint8_t bit_state, uint8_t *curr_char, int8_t *curr_bit) {
	// clear bit
	if (bit_state == 0) {
		rx_buffer[*curr_char] &= ~(1<<(*curr_bit));
	}
	// set 1 bit
	else {
		rx_buffer[*curr_char] |= (1<<(*curr_bit));
	}

	(*curr_bit)--;
	if (*curr_bit == -1) {
		*curr_bit = 7;
		(*curr_char)++;
	}
}

void TIM3_IRQHandler(void){
	static volatile uint8_t curr_char = 0;
	static volatile int8_t curr_bit = 7;
	static volatile uint8_t bit = 0;
	static volatile uint8_t ignore = 0; // if the edge is on the start of the period

	static volatile uint16_t time = 0;

	// TIM 3 CH 1
	switch (curr_state) {
	case IDLE:
		time = tim3->CCR1;

		// start timer to count for timeout
		tim4->CNT = 0;
		tim4->CR1 = 1;
		set_state(BUSY);

		// we always start with a 0 bit, reset all tracking vars
		bit = 0;
		curr_char = 0;
		curr_bit = 7;
		ignore = 0;
		rx_length = -5;
		read_bit(bit, &curr_char, &curr_bit);
		break;
	case BUSY:
		// reset counter since new edge arrived early enough
		tim4->CNT = 0;

		uint16_t curr_time = tim3->CCR1;
		uint16_t width = curr_time - time;

		// opposite bit
		if (width >= 900 * micro && width <= 1100 * micro) {
			bit ^= 1;
			read_bit(bit, &curr_char, &curr_bit);
		} else if (width >= 450 * micro && width <= 550 * micro) {
			ignore ^= 1;
			if (!ignore) {
				read_bit(bit, &curr_char, &curr_bit);
			}
		}

		time = curr_time;

		// check if we can read length now
		if (curr_char == 2 && curr_bit == 7) {
			rx_length = rx_buffer[1];
		}

		// message complete
		if (rx_length > 0 && curr_char-2 == rx_length) {
			received = 1;
			bit = 0;
			curr_char = 0;
			curr_bit = 7;
			ignore = 0;
		}
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
