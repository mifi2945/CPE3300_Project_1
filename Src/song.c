/**
  ******************************************************************************
  * @file		song.c
  * @brief		Song API
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/


#include <stdio.h>
#include "song.h"
#include "speaker.h"
#include "delay.h"
#include "timer.h"
#include "gpio.h"
#include "interrupt.h"

static volatile const note *song;
static volatile unsigned int duration = 0;

// start variable ensures the first note plays on init
static volatile unsigned int start = 0;

void song_init(void) {
	speaker_init();
	// GPIOB already enabled

	volatile uint32_t *rcc_apb1enr = (uint32_t*) RCC_APB1_ENR;
	*rcc_apb1enr |= TIM5_EN;

	// YOU DON'T NEED GPIO T-T

	tim5->ARR = F_CPU;
	tim5->CCR2 = F_CPU;

	tim5->CCMR1 |= (0b110<<12); // OC2M, PWM mode 1
	tim5->DIER |= (1<<2) | 1; // update event enabled and channel event

	nvic[ISER1] = 1<<(50-32); //TIM5 is IRQ 50

	tim5->CR1 = 1;

	speaker_volume(50);
}

int get_duration(void) {
	return duration;
}

void play_song(const note *music, int length) {
	song = music;
	duration = length;
	start = 1;
}

void stop_song(void) {
	duration = 0;
	speaker_stop();
}

void TIM5_IRQHandler(void) {
	if (tim5->SR & 1) {
		// update event flagged; ARR
		// clear flag
		tim5->SR = ~0b1;

		if (duration > 0) {
			// set delay by adjusting how often handler is called
			// through update event
			tim5->ARR = F_CPU / 1000 * song->duration;
			// for channel event, space between notes
			tim5->CCR2 = F_CPU / 1000 * (song->duration - 10);

			speaker_tone(song->freq);
			song++;
			duration--;
		}
	}
	if (tim5->SR & (1<<2)) {
		// channel event; CCRx
		// space between notes
		tim5->SR = ~(1<<2);
		if (start == 0) {
			speaker_stop();
		}
		start = 0;
	}
}
