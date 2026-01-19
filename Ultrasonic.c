#include "stm32l476xx.h" 
#include "Ultrasonic.h"

void UltraSonic_Init()
{
		//Enable GPIO Port B
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	
		//GPIO Mode for Trigger Pin
		GPIOB->MODER &= ~(3UL << (2 * Trig_Pin));
		GPIOB->MODER |= (1UL << (2 * Trig_Pin));
	
		//GPIO Speed for Trigger Pin
		GPIOB->OSPEEDR &= ~(3UL << (2 * Trig_Pin));
		GPIOB->OSPEEDR |= (3UL << (2 * Trig_Pin));
	
		//GPIO Output Type for Trigger Pin
		GPIOB->OTYPER &= ~(1UL << Trig_Pin);
	
		//GPIO Mode for Echo Pin
		GPIOB->MODER &= ~(3UL << (2 * Echo_Pin));
	
		//GPIO Push-Pull for Echo Pin
		GPIOB->PUPDR &= ~(3UL << (2 * Echo_Pin));
}

void TIM2_uS()  
{ 
     RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; // Enable TIM2 clock 
 
     // Set prescaler and ARR for 1s overflow (for 0.5Hz toggle) 
     TIM2->PSC = 3;        // 4 MHz / (3999 + 1) = 1 kHz timer 
     TIM2->ARR = 0xFFFF;         // Overflow every 1000 ticks = 1s 
 
     TIM2->CNT = 0;           // Reset counter 
     TIM2->CR1 |= TIM_CR1_CEN; // Enable counter 
} 

void delay_uS(uint32_t dur)
{
		for (uint32_t i = 0; i < dur; i++) 
		{
        TIM2->SR &= ~TIM_SR_UIF;             // Clear update flag
        while (!(TIM2->SR & TIM_SR_UIF));    // Wait until UIF is set
    }
}

void Trig_Pulse()
{
		uint32_t start;
	
		GPIOB->ODR &= ~(1U << Trig_Pin);
		start = TIM2->CNT;
	
		while((TIM2->CNT - start) < 2);
	
		GPIOB->ODR |= (1U << Trig_Pin);
		start = TIM2->CNT;
		
		while((TIM2->CNT - start) < 10);
	
		GPIOB->ODR &= ~(1U << Trig_Pin);
}