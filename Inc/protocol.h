/*
 * protocol.h
 *
 *  Created on: Jan 28, 2026
 *      Author: filippovm, acostal
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

enum Rx_State {
	IDLE,
	BUSY,
	COLLISION
};

void init_protocol(void);
void transmit(uint8_t length, char* message);

#endif /* PROTOCOL_H_ */
