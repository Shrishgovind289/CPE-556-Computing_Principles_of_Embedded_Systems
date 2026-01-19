#ifndef _SERIAL_DISPLAY_H
#define _SERIAL_DISPLAY_H

#include "stm32l476xx.h"

#define TX_PIN 2 //port A

void USART2_Transmit(char c);

void USART2_GPIO_Init(void);

void USART2_Init(void);

void USART2_TransmitData(const char* str);

#endif