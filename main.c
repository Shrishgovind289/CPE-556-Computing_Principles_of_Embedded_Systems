#include "stm32l476xx.h" 
#include "Ultrasonic.h"
#include "OLED.h"
#include <stdio.h>
#include "Serial_Display.h"
#include "Keypad.h"
#include <string.h>
 
int main(void)  
{ 
     UltraSonic_Init(); 
     TIM2_uS(); 
	
		 I2C1_GPIO_Init();
     I2C1_Init();
     SSD1306_Init();
	   SSD1306_Fill(0x00);  // Clear screen
	
		 USART2_GPIO_Init();
		 USART2_Init();
	
		 KEYPAD_Init();
	
     while (1) 
		 {  
			  
				Trig_Pulse();
			  
			  uint32_t timeout = TIM2->CNT;
			  while((GPIOB->IDR & (1 << Echo_Pin)) == 0)
				{
						if((TIM2->CNT - timeout) > 500000)
							break;
				}
				
				uint32_t star = TIM2->CNT;
				
				while((GPIOB->IDR & (1 << Echo_Pin)) != 0);
				
				uint32_t end = TIM2->CNT;
				
				uint32_t time = (end >= star) ? (end - star) : (0xFFFF - star + end);
				
				unsigned long distance = time/58;
				
				char distance_string[10];
				sprintf(distance_string, "%lu cm", distance);
				
				USART2_TransmitData("\r\nDistance: ");
				USART2_TransmitData(distance_string);
				
				if(distance <= 150)
				{
						char key[5];
						int flag = 0;
						USART2_TransmitData("Presence Detected\r\n");
						USART2_TransmitData("Activate Keypad\r\n");
						SSD1306_PrintString(3, 5, "Enter 4 digit Key");
						SSD1306_PrintString(4, 5, "_ _ _ _");
						
						int read = 0;						
					  while(read < 4)
						{
								for(int del = 0; del < 20000; del++); // debounce
								key[read] = read_keypad();
							
								if((key[0] > 0) && flag == 0)
								{
										SSD1306_PrintString(4, 5, "# _ _ _");
										USART2_Transmit(key[0]);
										read = 1;
										flag = 1;
								}
								else if((key[1]) && flag == 1)
								{
										SSD1306_PrintString(4, 5, "# # _ _");
										USART2_Transmit(key[1]);
										read = 2;
										flag = 2;
								}
								else if((key[2]) && flag == 2)
								{
										SSD1306_PrintString(4, 5, "# # # _");
										USART2_Transmit(key[2]);
										read = 3;
										flag = 3;
								}
								else if((key[3]) && flag == 3)
								{
										SSD1306_PrintString(4, 5, "# # # #");
										USART2_Transmit(key[3]);
										read = 4;
										flag = 4;
								}
						}
						
						if((key[0] == '1') && (key[1] == '0') && (key[2] == '4') && (key[3] == '8')) 
						{
								USART2_TransmitData("Key is Correct\r\n");
								USART2_TransmitData("Person May Enter\r\n");
								SSD1306_Fill(0x00);  // Clear screen
								SSD1306_PrintString(3, 5, "Pass Key is Correct\r\n");
							  for(int del = 0; del < 500000; del++);
								SSD1306_Fill(0x00);  // Clear screen
								SSD1306_PrintString(3, 5, "Welcome To The Other Side\r\n");
								for(int del = 0; del < 500000; del++);
								SSD1306_Fill(0x00);  // Clear screen
							
						}
						else
						{
								USART2_TransmitData("Key is Incorrect\r\n");
								USART2_TransmitData("Intruder Alert\r\n");
								SSD1306_Fill(0x00);  // Clear screen
								SSD1306_PrintString(3, 3, "Pass Key is Incorrect\r\n");
							  for(int del = 0; del < 500000; del++);
								SSD1306_Fill(0x00);  // Clear screen
								SSD1306_PrintString(3, 5, "IMPOSTER\r\n");
								for(int del = 0; del < 500000; del++);
								SSD1306_Fill(0x00);  // Clear screen
						}
				}
								
				star = TIM2->CNT;
				while((TIM2->CNT - star) < 500000);
     } 
} 



