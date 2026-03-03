/**
  ******************************************************************************
  * @file      protocol.h
  * @students: Mikhail Filippov, Lizbeth Acosta
  * @section:  111
  * @instr:    Dr. Varnell
  * @course    CPE 3300
  * @assign    Project 1
  * @brief
  ******************************************************************************
  */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <stdint.h>

enum Rx_State {
	IDLE,
	BUSY,
	COLLISION
};

void init_protocol(void);
void allow_tx();
void block_tx();
void retransmit(void);
int transmit(uint8_t length, char* message);

#endif /* PROTOCOL_H_ */
