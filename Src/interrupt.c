/**
  ******************************************************************************
  * @file		interrupt.c
  * @brief		Interrupt API
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/


#include "interrupt.h"


volatile uint32_t *exti = (uint32_t*) EXTI;
volatile uint32_t *nvic = (uint32_t*) NVIC;
