/**
  ******************************************************************************
  * @file		gpio.c
  * @brief		GPIO addresses
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/

#include "gpio.h"

volatile GPIO *gpioa = (GPIO*) GPIOA;
volatile GPIO *gpiob = (GPIO*) GPIOB;
volatile GPIO *gpioc = (GPIO*) GPIOC;
volatile GPIO *gpiod = (GPIO*) GPIOD;
