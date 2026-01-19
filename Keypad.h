#ifndef _KEYPAD_H
#define _KEYPAD_H

#include "stm32l476xx.h"

void KEYPAD_Init(void);

char read_keypad(void);

#endif