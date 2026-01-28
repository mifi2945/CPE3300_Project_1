/**
  ******************************************************************************
  * @file		speaker.c
  * @brief		Speaker API
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/



#include <stdint.h>
#include "speaker.h"
#include "timer.h"
#include "gpio.h"
#include "delay.h"

#define F_CPU 16000000UL


/**
 * @brief initialize the speaker through the timer and
 * all required GPIO
 */
void speaker_init(void) {
	volatile uint32_t *rcc = (uint32_t*) RCC_AHB1_ENR;
	*rcc |= GPIOBEN;

	gpiob->MODER &= ~(0b11<<(4*2)); //clear
	gpiob->MODER |= (0b10<<(4*2)); //alternate mode

	gpiob->AFRL &= ~(0b1111<<(4*4));
	gpiob->AFRL |= (0b0010<<(4*4)); // TIM3_CH1

	volatile uint32_t *rcc_tim = (uint32_t*) RCC_APB1_ENR;
	*rcc_tim |= TIM3_EN;

	tim3->CCR1 = 0; //reset CCR1
	tim3->CCMR1 |= (0b110<<4); //set PWM mode 1 (OCM1)
	tim3->CCER |= 1; // enable output
}

/**
 * @brief start playing a tone at given frequency
 * @param freq - the frequency given to play
 */
void speaker_tone(int freq) {
	int period = F_CPU / freq;
	tim3->ARR = period-1;
	tim3->CR1 = 1;
}

/**
 * @brief stop speaker (in actuality TIM)
 */
void speaker_stop(void) {
	tim3->CR1 = 0;
}

/**
 * @brief adjust the speaker volume; needs to be called
 * to even hear a sound in the first place
 * @param duty - the "volume" to be set to
 */
void speaker_volume(unsigned int duty) {
	uint32_t period = tim3->ARR + 1;

	// set duty cycle for PWM (volume)
	tim3->CCR1 = (period*duty) / 5000;
}
