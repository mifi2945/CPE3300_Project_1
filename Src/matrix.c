/**
  ******************************************************************************
  * @file		matrix.c
  * @brief		Matrix API File
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/


#include <string.h>
#include "matrix.h"
#include "spi.h"
#include "gpio.h"

static uint8_t matrix_buffer[8];

static void matrix_write(uint16_t frame) {
	uint8_t send[] = {frame>>8, frame&0xFF};
	spi2_wr(gpiob, 12, 2, send, 0, NULL);
}

void matrix_init(void) {
	spi2_init();

	matrix_write(0x0C01); //normal mode
	matrix_write(0x0B07); //scan limit max
	matrix_write(0x0F00); // no display test / normal operation
	matrix_write(0x0900); // no decode mode
	matrix_write(0x0A01); // brightness

	matrix_clear();
	matrix_update();
}

void matrix_set_pixel(int x, int y) {
	if (x >= 0 && x < 8 && y >=0 && y < 8) {
		matrix_buffer[x] |= (1<<y);
	}
}
void matrix_clear_pixel(int x, int y) {
	if (x >= 0 && x < 8 && y >=0 && y < 8) {
		matrix_buffer[x] &= ~(1<<y);
	}
}
void matrix_clear(void) {
	memset(matrix_buffer, 0, 8);
}
void matrix_update(void) {
	for (int i = 0; i < 8; ++i) {
		matrix_write((i+1)<<8 | matrix_buffer[i]);
	}
}
