#include "stm32l476xx.h"
#include "Serial_Display.h"


void USART2_Transmit(char c) 
{
    while (!(USART2->ISR & USART_ISR_TXE)); // Wait until transmit buffer empty
    USART2->TDR = c;
    while (!(USART2->ISR & USART_ISR_TC));  // Wait until transmission complete
}

void USART2_TransmitData(const char* str)
{
		while (*str) 
		{
        USART2_Transmit(*str++);
    }
}

void USART2_GPIO_Init(void) 
{
    // Enable GPIOA 
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
   
    // Set PA2 and PA3 to alternate function (AF7 = USART2)
    //setting TX pin to alternate function (10)
	 GPIOA->MODER &= ~(3UL<<(2*TX_PIN));
	 GPIOA->MODER |= (2UL<<(2*TX_PIN));
	 //pin 2, tx
	 GPIOA->AFR[0] &= ~(15 << 4*TX_PIN); //clearing pin 2 AltFunc bits.
	 GPIOA->AFR[0] |= (7 << 4*TX_PIN); //setting pin 2 altfunc to Altfunc 7.
}

void USART2_Init(void) 
{
	 
	  // Baud: 115200, Data: 8 bits, Parity: None, Stop: 1 bit, Flow Control: None
	  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; //ENABLE USART2 clocks 
    
    USART2->CR1 &= ~USART_CR1_UE; // Disable USART2 before configuration

    // Set baud rate: 4 MHz / 115200 = 34.72 ? rounded to 35
    USART2->BRR = 35;
    
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE; // Enable Transmitter and Receiver
	
    USART2->CR1 |= USART_CR1_UE; // Enable USART2
}