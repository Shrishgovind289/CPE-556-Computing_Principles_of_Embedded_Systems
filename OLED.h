#ifndef _OLED_H
#define _OLED_H

#include "stm32l476xx.h"

// I2C and SSD1306 definitions
#define SSD1306_I2C_ADDR  0x3C
#define SSD1306_COMMAND   0x00
#define SSD1306_DATA      0x40

#define SDA_PIN  9
#define SCL_PIN  8

// === I2C GPIO Setup (PB8 = SCL, PB9 = SDA) ===
void I2C1_GPIO_Init(void);

// === I2C Peripheral Initialization ===
void I2C1_Init(void);

// === Send a Single Command to SSD1306 ===
void SSD1306_WriteCommand(uint8_t cmd);

// === Send One Data Byte ===
void SSD1306_WriteData(uint8_t data);

// === SSD1306 Initialization Sequence ===
void SSD1306_Init(void);

// === Fill Display with a Pattern ===
void SSD1306_Fill(uint8_t pattern);

void SSD1306_SetCursor(uint8_t page, uint8_t col);

void SSD1306_PrintChar(char c);

void SSD1306_PrintString(uint8_t page, uint8_t col, const char* str);

void SSD1306_ClearLine(uint8_t page);


#endif