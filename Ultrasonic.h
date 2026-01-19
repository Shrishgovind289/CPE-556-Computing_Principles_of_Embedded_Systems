#ifndef _ULTRASONIC_H
#define _ULTRASONIC_H

#include "stm32l476xx.h" 

#define Trig_Pin  5
#define Echo_Pin  6

void UltraSonic_Init();

void TIM2_uS();

void delay_uS(uint32_t dur);

void Trig_Pulse();

#endif