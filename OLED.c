#include "stm32l476xx.h"
#include "OLED.h"
#include "fonts.h"

// === I2C GPIO Setup (PB8 = SCL, PB9 = SDA) ===
void I2C1_GPIO_Init(void) 
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    // Alternate function for PB8, PB9 (AF4)
    GPIOB->MODER &= ~((3 << (SCL_PIN * 2)) | (3 << (SDA_PIN * 2)));
    GPIOB->MODER |=  ((2 << (SCL_PIN * 2)) | (2 << (SDA_PIN * 2)));

    GPIOB->OTYPER |= (1 << SCL_PIN) | (1 << SDA_PIN); // Open-drain

    GPIOB->OSPEEDR |= (3 << (SCL_PIN * 2)) | (3 << (SDA_PIN * 2)); // High speed

    GPIOB->PUPDR &= ~((3 << (SCL_PIN * 2)) | (3 << (SDA_PIN * 2)));
    GPIOB->PUPDR |=  ((1 << (SCL_PIN * 2)) | (1 << (SDA_PIN * 2))); // Pull-up

    GPIOB->AFR[1] &= ~((0xF << 0) | (0xF << 4));
    GPIOB->AFR[1] |=  ((4 << 0) | (4 << 4)); // AF4 = I2C1
}

// === I2C Peripheral Initialization ===
void I2C1_Init(void) 
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

    RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C1RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C1RST;

    I2C1->CR1 &= ~I2C_CR1_PE;

    // TIMING for 100kHz @ 4 MHz input (from STM32 timing tool)
    I2C1->TIMINGR = 0x00303D5B;

    I2C1->CR1 |= I2C_CR1_PE;
}

// === Send a Single Command to SSD1306 ===
void SSD1306_WriteCommand(uint8_t cmd) 
{
    while (I2C1->ISR & I2C_ISR_BUSY);

    I2C1->CR2 = (SSD1306_I2C_ADDR << 1) | (2 << 16) | I2C_CR2_AUTOEND | I2C_CR2_START;

    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = SSD1306_COMMAND;

    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = cmd;

    while (!(I2C1->ISR & I2C_ISR_STOPF));
    I2C1->ICR |= I2C_ICR_STOPCF;
}

// === Send One Data Byte ===
void SSD1306_WriteData(uint8_t data) 
{
    while (I2C1->ISR & I2C_ISR_BUSY);

    I2C1->CR2 = (SSD1306_I2C_ADDR << 1) | (2 << 16) | I2C_CR2_AUTOEND | I2C_CR2_START;

    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = SSD1306_DATA;

    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = data;

    while (!(I2C1->ISR & I2C_ISR_STOPF));
    I2C1->ICR |= I2C_ICR_STOPCF;
}

// === SSD1306 Initialization Sequence ===
void SSD1306_Init(void) 
{
    SSD1306_WriteCommand(0xAE); // Display OFF
    SSD1306_WriteCommand(0xD5); SSD1306_WriteCommand(0x80);
    SSD1306_WriteCommand(0xA8); SSD1306_WriteCommand(0x3F);
    SSD1306_WriteCommand(0xD3); SSD1306_WriteCommand(0x00);
    SSD1306_WriteCommand(0x40);
    SSD1306_WriteCommand(0x8D); SSD1306_WriteCommand(0x14);
    SSD1306_WriteCommand(0x20); SSD1306_WriteCommand(0x00);
    SSD1306_WriteCommand(0xA1);
    SSD1306_WriteCommand(0xC8);
    SSD1306_WriteCommand(0xDA); SSD1306_WriteCommand(0x12);
    SSD1306_WriteCommand(0x81); SSD1306_WriteCommand(0xCF);
    SSD1306_WriteCommand(0xD9); SSD1306_WriteCommand(0xF1);
    SSD1306_WriteCommand(0xDB); SSD1306_WriteCommand(0x40);
    SSD1306_WriteCommand(0xA4);
    SSD1306_WriteCommand(0xA6);
    SSD1306_WriteCommand(0xAF); // Display ON
}

// === Fill Display with a Pattern ===
void SSD1306_Fill(uint8_t pattern) 
{
    for (uint8_t page = 0; page < 8; page++) 
	  {
        SSD1306_WriteCommand(0xB0 + page); // Set page address
        SSD1306_WriteCommand(0x00);        // Lower column
        SSD1306_WriteCommand(0x10);        // Upper column

        for (uint8_t col = 0; col < 128; col++) 
			  {
            SSD1306_WriteData(pattern);
        }
    }
}

void SSD1306_SetCursor(uint8_t page, uint8_t col) {
    SSD1306_WriteCommand(0xB0 + page);         // Page address (0–7)
    SSD1306_WriteCommand(0x00 + (col & 0x0F)); // Lower nibble of column
    SSD1306_WriteCommand(0x10 + (col >> 4));   // Upper nibble of column
}

void SSD1306_PrintChar(char c) {
    if (c < 32 || c > 126) return;  // Only printable ASCII (0x20 to 0x7E)
    const char* chr_bitmap = font[c - 32];
    for (int i = 0; i < 6; i++) {
        SSD1306_WriteData(chr_bitmap[i]);
    }
}

void SSD1306_PrintString(uint8_t page, uint8_t col, const char* str) {
    SSD1306_SetCursor(page, col);
    while (*str) {
        SSD1306_PrintChar(*str++);
    }
}

void SSD1306_ClearLine(uint8_t page) {
    SSD1306_SetCursor(page, 0);  // Start at column 0 of the page
    for (uint8_t col = 0; col < 128; col++) {
        SSD1306_WriteData(0x00); // Blank pixel column
    }
}

