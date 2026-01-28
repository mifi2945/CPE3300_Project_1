/**
  ******************************************************************************
  * @file		accelerometer.c
  * @brief		ADXL345 API
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/


#include "gpio.h"
#include "accelerometer.h"
#include "spi.h"

static void clear_registers(void) {
	uint8_t send[] = {0x31, 0x00};
	spi2_wr(gpiob, ACC_SS, 2, send, 0, NULL);

	uint8_t offset_send[] = {0x1E, 0x00};
	spi2_wr(gpiob, ACC_SS, 2, offset_send, 0, NULL);
	offset_send[0] = 0x1F;
	offset_send[1] = 0x00;
	spi2_wr(gpiob, ACC_SS, 2, offset_send, 0, NULL);
	offset_send[0] = 0x20;
	offset_send[1] = 0x00;
	spi2_wr(gpiob, ACC_SS, 2, offset_send, 0, NULL);

	send[0] = 0x2D;
	send[1] = 0x00;
	spi2_wr(gpiob, ACC_SS, 2, send, 0, NULL);
}

void accelerometer_init(void) {
	spi2_init();
	clear_registers();

	// address, data
	uint8_t send[] = {0x31, 0x0B};
	spi2_wr(gpiob, ACC_SS, 2, send, 0, NULL);

	// offset data
	// offset calculated as: data to send = raw change / 4
	uint8_t offset_send[] = {0x1E, 0xFE};
	spi2_wr(gpiob, ACC_SS, 2, offset_send, 0, NULL);
	offset_send[0] = 0x1F;
	offset_send[1] = 0xFE;
	spi2_wr(gpiob, ACC_SS, 2, offset_send, 0, NULL);
	offset_send[0] = 0x20;
	offset_send[1] = 0x05;
	spi2_wr(gpiob, ACC_SS, 2, offset_send, 0, NULL);

	// enable
	send[0] = 0x2D;
	send[1] = 0x08;
	spi2_wr(gpiob, ACC_SS, 2, send, 0, NULL);
}

void get_xyz(int *x, int *y, int *z) {
	// address of X0, multi-frame, read
	uint8_t send = 0x32 | 1<<6 | 1<<7;
	uint8_t read[6];

	spi2_wr(gpiob, ACC_SS, 1, &send, 6, read);

	*x = (int16_t)((read[1]<<8) | read[0]);
	*y = (int16_t)((read[3]<<8) | read[2]);
	*z = (int16_t)((read[5]<<8) | read[4]);
}
