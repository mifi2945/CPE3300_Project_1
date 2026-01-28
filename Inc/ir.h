/**
  ******************************************************************************
  * @file		ir.h
  * @brief		IR API
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/

#ifndef IR_H_
#define IR_H_


#define POWER		0x95
#define VOL_UP		0x92
#define VOL_DOWN	0x93
#define MUTE		0x94
#define CH_UP		0x90
#define CH_DOWN		0x91
#define NUM_1		0x80
#define NUM_2		0x81
#define NUM_3		0x82
#define NUM_4		0x83
#define NUM_5		0x84
#define NUM_6		0x85
#define NUM_7		0x86
#define NUM_8		0x87
#define NUM_9		0x88
#define NUM_0		0x89
#define EXIT		0xBB


enum State {
	WAIT_START,
	WAIT_END_OF_START,
	WAIT_BITS
};

void ir_init(void);
void read_ready(void);
uint16_t ir_get_command(void);

#endif /* IR_H_ */
