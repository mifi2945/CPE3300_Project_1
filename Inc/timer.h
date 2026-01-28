/**
  ******************************************************************************
  * @file		timer.h
  * @brief		Timer Header file for TIM constants
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/

#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>


#define TIM2			0x40000000
#define TIM3			0x40000400
#define TIM4			0x40000800
#define TIM5			0x40000C00

// RCC_APB1
#define TIM2_EN			(1<<0)
#define TIM3_EN			(1<<1)
#define TIM4_EN			(1<<2)
#define TIM5_EN			(1<<3)

// RCC_APB2
#define TIM1_EN			(1<<0)

//#define CR1		0
//#define CR2		1
//#define SMCR	2
//#define DIER	3
//#define SR		4
//#define EGR		5
//#define CCMR1	6
//#define CCMR2	7
//#define CCER	8
//#define CNT		9
//#define PSC		10
//#define ARR		11
////#define RESERVED 12
//#define CCR1	13
//#define	CCR2	14
//#define CCR3	15
//#define CCR4	16
////#define RESERVED 17
//#define DCR		18
//#define DMAR	19
//#define OR		20

typedef struct {
	uint32_t CR1;
	uint32_t CR2;
	uint32_t SMCR;
	uint32_t DIER;
	uint32_t SR;
	uint32_t EGR;
	uint32_t CCMR1;
	uint32_t CCMR2;
	uint32_t CCER;
	uint32_t CNT;
	uint32_t PCS;
	uint32_t ARR;
	uint32_t reserved1;
	uint32_t CCR1;
	uint32_t CCR2;
	uint32_t CCR3;
	uint32_t CCR4;
	uint32_t reserved2;
	uint32_t DCR;
	uint32_t DMAR;
	uint32_t OR;
} TIM;

extern volatile TIM *tim2;
extern volatile TIM *tim3;
extern volatile TIM *tim5;

#endif /* TIMER_H_ */
