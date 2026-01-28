/**
  ******************************************************************************
  * @file		spi.c
  * @brief		SPI Address File
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/


#include "spi.h"
#include "gpio.h"


volatile SPI *spi2 = (SPI*) SPI2;
//static uint16_t spi2_frame = 0;
//static int sending = 0;

void spi2_init(void) {
	// PB5 -> Accelerameter CS
	// PB12 -> Matrix CS

	// PB13 -> SCK
	// PB14 -> MISO
	// PB15 -> MOSI

	volatile uint32_t *rcc_apb1enr = (uint32_t*) RCC_APB1_ENR;
	*rcc_apb1enr |= SPI2_EN;
	volatile uint32_t *rcc_ahb1enr = (uint32_t*) RCC_AHB1_ENR;
	*rcc_ahb1enr |= GPIOBEN;

	//clear and set AF5
	gpiob->MODER &= ~(0b11<<(ACC_SS*2)) & ~(0b11<<(MATRIX_SS*2)) &
			~(0b11<<(13*2)) & ~(0b11<<(14*2)) & ~(0b11<<(15*2));
	gpiob->MODER |= (0b01<<(ACC_SS*2)) | (0b01<<(MATRIX_SS*2)) |
			(0b10<<(13*2)) | (0b10<<(14*2)) | (0b10<<(15*2));
	gpiob->AFRH &= ~(0xF<<(20)) & ~(0xF<<(24)) & ~(0xF<<(28));
	gpiob->AFRH |= (0b0101<<(20)) | (0b0101<<(24)) | (0b0101<<(28));

	// 8 bit frame, clk/4, CPOL 1, CPHA 1 (second edge, rising), master
	spi2->CR1 |= (0<<DFF) | (001<<BR) | (1<<CPHA) | (1<<CPOL) |
			(1<<MSTR) | (1<<SSI) | (1<<SSM);

	// bring acc CS high
	//gpiob->BSRR = 1<<ACC_SS;
}

void spi2_wr(volatile GPIO* ssport, int ssbit,
		int write_length, uint8_t* write_data,
		int read_length, uint8_t* read_data) {

	spi2->CR1 |= (1<<SPE);
	// slave/chip select
	ssport->BSRR = 1<<(ssbit+16);


	//TX loop
	for (int i = 0; i < write_length; ++i) {
		while (!(spi2->SR & SPI_TXE)) {}
		spi2->DR = write_data[i];
	}

	int _ = spi2->DR;
	_ = spi2->SR;
	while (!(spi2->SR & SPI_RXNE)) {}
	_ = spi2->DR;

	for (int i = 0; i < read_length; ++i) {
		// initiate read
		spi2->DR = 0x00;

		while (!(spi2->SR & SPI_RXNE)) {}
		read_data[i] = spi2->DR;
		while (!(spi2->SR & SPI_TXE)) {}
	}

	while (spi2->SR & SPI_BSY) {}

	//shut down
	ssport->BSRR = 1<<ssbit;
	spi2->CR1 &= ~(1<<SPE);


}

//void SPI2_IRQHandler(void) {
//	uint32_t status = spi2->SR;
//	// TXE
//	if ((status & SPI_TXE) && (spi2->CR2 & SPI_TXEIE)) {
//		spi2->DR = spi2_frame;
//		sending = 0;
//		spi2->CR2 &= ~SPI_TXEIE;
//		//spi2->CR1 |= (1<<8);
//	}
//}
