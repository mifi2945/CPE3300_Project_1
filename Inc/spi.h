/**
  ******************************************************************************
  * @file		spi.h
  * @brief		SPI Address Header
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include "gpio.h"

#define SPI1		0x40013000
#define SPI2		0x40003800
#define SPI3		0x40003C00
#define SPI4		0x40013400
#define SPI5		0x40015000

#define SPI1_EN		(1<<12) //APB2
#define SPI2_EN		(1<<14) //APB1
#define SPI3_EN		(1<<15) //APB1
#define SPI4_EN		(1<<13) //APB2
#define SPI5_EN		(1<<20) //APB2

//bits in CR1
#define CPHA		0
#define CPOL		1
#define MSTR		2
#define BR			3
//#define BR		4;
//#define BR		5;
#define SPE			6
#define LSB			7
#define SSI			8
#define SSM			9
#define RX_ONLY		10
#define DFF			11

//flags in SR
#define SPI_RXNE	(1<<0)
#define SPI_TXE		(1<<1)
#define SPI_BSY		(1<<7)

#define SPI_RXNEIE	(1<<6)
#define SPI_TXEIE	(1<<7)

#define ACC_SS		5
#define MATRIX_SS	12

typedef struct {
	uint32_t CR1;
	uint32_t CR2;
	uint32_t SR;
	uint32_t DR;
} SPI;

extern volatile SPI *spi2;

void spi2_init(void);
void spi2_wr(volatile GPIO* ssport, int ssbit,
		int write_length, uint8_t* write_data,
		int read_length, uint8_t* read_data);

#endif /* SPI_H_ */
