/**
  ******************************************************************************
  * @file		ir.c
  * @brief		IR API
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/

#include <stdint.h>
#include <stdio.h>
#include "ir.h"
#include "gpio.h"
#include "timer.h"
#include "interrupt.h"
#include "delay.h"

static volatile uint16_t ir_command = 0;
static volatile int ir_ready_flag = 0;


static void tim_init(void){
	// Use TIM2_CH2 on PB3, AF01
	volatile uint32_t *rcc_apb1enr = (uint32_t*) RCC_APB1_ENR;
	*rcc_apb1enr |= TIM2_EN;

	gpiob->MODER &= ~(0b11<<(3*2));
	gpiob->MODER |= (0b10<<(3*2));

	gpiob->PUPDR &= ~(0b11<<(3*2));

	gpiob->AFRL &= ~(0b1111<<(3*4));
	gpiob->AFRL |= (0b0001<<(3*4));

	// input compare TI2
	tim2->CCMR1 |= (0b01<<8);
	// falling edge CH2: 0b01
	tim2->CCER |= (0<<7) | (1<<5);
	// enable capture
	tim2->CCER |= 1<<4;
	// interrupt enable 2
	tim2->DIER |= (1<<2);
	// set ARR max; it's already at max
	//tim2[ARR] |= (1<<31) - 1;

	// position 28
	nvic[ISER0] = 1<<(28);


	// enable
	tim2->CR1 = 1;

}

void ir_init(void){
	// Use TIM3_CH4 on PA3
	volatile uint32_t *rcc_ahb1enr = (uint32_t*) RCC_AHB1_ENR;
	*rcc_ahb1enr |= GPIOBEN;

	// PB2 input mode
	gpiob->MODER &= ~(0b11<<(2*2));
	// clear; no pull up or pull down
	gpiob->PUPDR &= ~(0b11<<(2*2));

	tim_init();
}

void read_ready(void) {
	ir_ready_flag = 0;
}

uint16_t ir_get_command(void) {
	uint16_t ret = 0;
	if (ir_ready_flag == 1) {
		ret = ir_command;
		ir_command = 0;
	}
	return ret;
}

void TIM2_IRQHandler(void) {
	static enum State state = WAIT_START;
	static volatile int i = 0;
	static volatile uint16_t time = 0;
	static volatile const int milli = F_CPU / 1000;
	// interrupt flag is cleared by reading CCR2

	tim2->SR = ~(1<<2);


	if (ir_ready_flag == 0) {
		uint16_t current_time = tim2->CCR2;
		if (state == WAIT_START) {
			time = current_time;
			state = WAIT_END_OF_START;
		} else if (state == WAIT_END_OF_START) {
			uint16_t width = current_time - time;
			if (width > 5*milli/2 && width <= 7*milli/2){
				time = current_time;
				i = 0;
				state = WAIT_BITS;
			} else {
				// error
				state = WAIT_START;
			}
		} else if (state == WAIT_BITS) {
			uint16_t width = current_time - time;
			if (width > milli/4 && width < 3*milli/2) {
				// 0 interpreted
				time = current_time;
				//ir_command |= (0<<i);
				i = i + 1;
			} else if (width >= 3*milli/2 && width < 5*milli/2) {
				// 1 interpreted
				time = current_time;
				ir_command |= (1<<i);
				i = i + 1;
			} else {
				// error
				state = WAIT_START;
			}
			if (i == 11) {
				ir_ready_flag = 1;
				//printf("%X\n",ir_command);
				state = WAIT_START;
			}
		}
	}
}
