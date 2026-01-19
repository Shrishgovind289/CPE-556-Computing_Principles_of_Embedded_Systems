#include "stm32l476xx.h"
#include "Keypad.h"

char pad[4][4] = {
    {'1','2','3','A'},  //1,*,7,4
    {'4','5','6','B'},  //2,0,8,5
    {'7','8','9','C'},
    {'*','0','#','D'}
};

int row_pin[4] = {10, 13, 14, 15};  // PB10, PB13, PB14, PB15
int col_pin[4] = {12, 13, 14, 15};  // PC12, PC13, PC14, PC15

void KEYPAD_Init(void) 
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;

    // ROWS: PB10, PB13–15 as output
    for (int i = 0; i < 4; i++) {
        GPIOB->MODER &= ~(3 << (2 * row_pin[i]));
        GPIOB->MODER |=  (1 << (2 * row_pin[i]));
    }

    // COLS: PC12–15 as input with pull-up
    for (int i = 0; i < 4; i++) {
        GPIOC->MODER &= ~(3 << (2 * col_pin[i]));
        GPIOC->PUPDR &= ~(3 << (2 * col_pin[i]));
        GPIOC->PUPDR |=  (1 << (2 * col_pin[i]));
    }
}

char read_keypad(void) 
{
		
    for (int row = 0; row < 4; row++) {
        // Set all rows HIGH
        for (int r = 0; r < 4; r++)
            GPIOB->ODR |= (1 << row_pin[r]);

        // Set current row LOW
        GPIOB->ODR &= ~(1 << row_pin[row]);
        for(int del = 0; del < 10; del++);

        for (int col = 0; col < 4; col++) {
            if (!(GPIOC->IDR & (1 << col_pin[col]))) {
                for(int del = 0; del < 2000; del++); // debounce
                if (!(GPIOC->IDR & (1 << col_pin[col]))) {
                    return pad[row][col];
                }
            }
        }
    }
    return 0;
}