#include "main.h"
#include "stm32u585xx.h"
#include <stdio.h>
#include <string.h>

#define I2C2_SCL  4
#define I2C2_SDA  5

char rx_buffer[64];
char response[64];
int rx_index = 0;
int rx_ready = 0;
int cmp_res = 69;

volatile char data;

float temp, humidity;
float pressure = 0.0f;
float temperature = 0.0f;
char c;

void delay(volatile uint32_t d) 
{
    while (d--) __NOP();
}

//////////////////////////////////I2C Protocol START///////////////////////////////////////////////////////
void I2C2_Init()
{
	//I2C GPIO Config
	RCC->AHB2ENR1 |= RCC_AHB2ENR1_GPIOHEN;
	
	// Set mode to Alternate Function (10)
	GPIOH->MODER &= ~((3 << (2 * I2C2_SCL)) | (3 << (2 * I2C2_SDA)));   // Clear MODER4 and MODER5
	GPIOH->MODER |=  ((2 << (2 * I2C2_SCL)) | (2 << (2 * I2C2_SDA)));   // Set to AF

	// Set AF4 for both pins
	GPIOH->AFR[0] &= ~((0xF << (4 * I2C2_SCL)) | (0xF << (4 * I2C2_SDA)));
	GPIOH->AFR[0] |=  ((4 << (4 * I2C2_SCL)) | (4 << (4 * I2C2_SDA)));

	// Set output type to open-drain
	GPIOH->OTYPER |= (1 << I2C2_SCL) | (1 << I2C2_SDA);

	// Set pull-up (recommended for I2C lines)
	GPIOH->PUPDR &= ~((3 << (2 * I2C2_SCL)) | (3 << (2 * I2C2_SDA)));
	GPIOH->PUPDR |=  ((1 << (2 * I2C2_SCL)) | (1 << (2 * I2C2_SDA)));

	// Set speed to very high
	GPIOH->OSPEEDR |= ((3 << (2 * I2C2_SCL)) | (3 << (2 * I2C2_SDA)));
	
	//I2C Configuration
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
		
	// Reset I2C2
	RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C2RST;
	RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C2RST;

	// Disable I2C2 before configuration
	I2C2->CR1 &= ~I2C_CR1_PE;

	// Set timing register for 100kHz standard mode (assuming 16 MHz clock)
	I2C2->TIMINGR = 0x20303E5D;  // You can recalculate using STM32 timing tool

	// Enable I2C2
	I2C2->CR1 |= I2C_CR1_PE;

}

uint8_t I2C2_ReadRegister(uint8_t devAddr, uint8_t regAddr)
{
    	uint32_t timeout;

    	// --------- Write phase: send register address ---------
    	I2C2->CR2 = (devAddr << 1) | (1 << 16) | I2C_CR2_START;

    	// Wait for TXIS or NACK
    	timeout = 100000;
    	while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)) && --timeout);
    	if (I2C2->ISR & I2C_ISR_NACKF) 
	{
        	I2C2->ICR |= I2C_ICR_NACKCF;
        	return 0xFF;
    	}

    	I2C2->TXDR = regAddr;

    	// Wait for TC or NACK
    	timeout = 100000;
    	while (!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)) && --timeout);
    	if (I2C2->ISR & I2C_ISR_NACKF) 
		{
        	I2C2->ICR |= I2C_ICR_NACKCF;
        	return 0xFF;
    	}

    	// --------- Read phase ---------
    	I2C2->CR2 = (devAddr << 1) | (1 << 16) | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND | I2C_CR2_START;

    	// Wait for RXNE or NACK
    	timeout = 100000;
    	while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)) && --timeout);
    	if (I2C2->ISR & I2C_ISR_NACKF) 
		{
        	I2C2->ICR |= I2C_ICR_NACKCF;
        	return 0xFF;
    	}

    	return I2C2->RXDR;
}


void I2C2_WriteRegister(uint8_t devAddr, uint8_t regAddr, uint8_t data) 
{
    // 1. Configure I2C2 CR2 for 2 bytes to send (reg + data)
	I2C2->CR2 = (devAddr << 1) | (2 << 16) | I2C_CR2_AUTOEND | I2C_CR2_START;           // Generate START condition
	
	// 2. Wait for TXIS (ready to transmit)
    while (!(I2C2->ISR & I2C_ISR_TXIS));
    I2C2->TXDR = regAddr;  // First byte = Register address

	while (!(I2C2->ISR & I2C_ISR_TXIS));
    I2C2->TXDR = data;     // Second byte = data to write

    // 3. Wait for STOP flag
    while (!(I2C2->ISR & I2C_ISR_STOPF));

    // 4. Clear STOP flag by writing to ICR
    I2C2->ICR |= I2C_ICR_STOPCF;
}

//////////////////////////////////I2C Protocol STOP///////////////////////////////////////////////////////

//////////////////////////////////HTS221 Temp & Humidity START////////////////////////////////////////////

void HTS221_Enable(void) 
{
    // CTRL_REG1 (0x20)
    // Power-on (PD = 1), BDU = 1 (block update), ODR = 1 Hz (01 << 0)
    I2C2_WriteRegister(0x5F, 0x20, 0b10000101);  // PD=1, BDU=1, ODR1
}

uint8_t HTS221_DataAvailable(void) 
{
	HTS221_Enable();
   	uint8_t status = I2C2_ReadRegister(0x5F, 0x27); // STATUS_REG
   	return (status & 0x03); // Bit 1: Temp available, Bit 0: Humid available
}

int16_t HTS221_ReadRawHumidity(void) 
{
   	uint8_t H_OUT_L = I2C2_ReadRegister(0x5F, 0x28);
   	uint8_t H_OUT_H = I2C2_ReadRegister(0x5F, 0x29);
   	return (int16_t)((H_OUT_H << 8) | H_OUT_L);
}


int16_t HTS221_ReadRawTemperature(void) 
{
   	uint8_t T_OUT_L = I2C2_ReadRegister(0x5F, 0x2A);
   	uint8_t T_OUT_H = I2C2_ReadRegister(0x5F, 0x2B);
   	return (int16_t)((T_OUT_H << 8) | T_OUT_L);
}

float HTS221_ReadHumidity(void) 
{
	int16_t H0_rH = I2C2_ReadRegister(0x5F, 0x30) >> 1; // in %
    int16_t H1_rH = I2C2_ReadRegister(0x5F, 0x31) >> 1; // in %
    int16_t H0_T0_OUT = (int16_t)((I2C2_ReadRegister(0x5F, 0x36) << 8) | I2C2_ReadRegister(0x5F, 0x37));
    int16_t H1_T0_OUT = (int16_t)((I2C2_ReadRegister(0x5F, 0x3A) << 8) | I2C2_ReadRegister(0x5F, 0x3B));
    int16_t H_OUT = HTS221_ReadRawHumidity();

	return ((H_OUT - H0_T0_OUT) * (H1_rH - H0_rH) / (H1_T0_OUT - H0_T0_OUT)) + H0_rH;
}

float HTS221_ReadTemperature(void) 
{
    uint8_t T0_degC_x8 = I2C2_ReadRegister(0x5F, 0x32);
    uint8_t T1_degC_x8 = I2C2_ReadRegister(0x5F, 0x33);
    uint8_t T1_T0_msb = I2C2_ReadRegister(0x5F, 0x35);
    int16_t T0_degC = ((T1_T0_msb & 0x03) << 8 | T0_degC_x8) >> 3;
    int16_t T1_degC = ((T1_T0_msb & 0x0C) << 6 | T1_degC_x8) >> 3;

    int16_t T0_OUT = (int16_t)((I2C2_ReadRegister(0x5F, 0x3C) << 8) | I2C2_ReadRegister(0x5F, 0x3D));
    int16_t T1_OUT = (int16_t)((I2C2_ReadRegister(0x5F, 0x3E) << 8) | I2C2_ReadRegister(0x5F, 0x3F));
    int16_t T_OUT = HTS221_ReadRawTemperature();

    return ((T_OUT - T0_OUT) * (T1_degC - T0_degC) / (T1_OUT - T0_OUT)) + T0_degC;
}

//////////////////////////////////HTS221 Temp & Humidity STOP////////////////////////////////////////////

///////////////////////////////////////////UART 3 Protocol START/////////////////////////////////////////////////////////////////

void USART3_Init(void) 
{
    // Enable GPIOD and USART3 clocks
    RCC->AHB2ENR1 |= RCC_AHB2ENR1_GPIODEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;

    // Set PD8 (RX) and PD9 (TX) to Alternate Function mode (AF7)
    GPIOD->MODER &= ~((3 << (2 * 8)) | (3 << (2 * 9)));
    GPIOD->MODER |=  ((2 << (2 * 8)) | (2 << (2 * 9)));

    GPIOD->AFR[1] &= ~((0xF << (0)) | (0xF << (4)));   // PD8=AFRH0, PD9=AFRH1
    GPIOD->AFR[1] |=  ((7 << (0)) | (7 << (4)));       // AF7 = USART3

    GPIOD->OSPEEDR |= (3 << (2 * 9));  // Optional: very high speed

    // Disable USART3 before config
    USART3->CR1 &= ~USART_CR1_UE;

    // Set baud rate
    USART3->BRR = SystemCoreClock / 115200;

    // Enable transmitter and receiver
    USART3->CR1 |= USART_CR1_TE | USART_CR1_RE;
		
    // Enable USART3
    USART3->CR1 |= USART_CR1_UE;
}

void USART3_SendChar(char c) 
{
    while (!(USART3->ISR & USART_ISR_TXE));
    USART3->TDR = c;
}

void USART3_SendString(const char* str)
{
    while (*str) USART3_SendChar(*str++);
}

void USART3_ReceiveString(char *buffer, int max_len) 
{
    int i = 0;
    char c;

    while (i < max_len - 1) 
    {
        // Wait until RXNE (Receive Not Empty) is set
        while (!(USART3->ISR & USART_ISR_RXNE));
        c = USART3->RDR;  // Read the received character
        if (c == '\r' || c == '\n') 
        {
            break;  // End of input
        }
        buffer[i++] = c;
    }
    buffer[i] = '\0';  // Null-terminate the string
}

//////////////////////////////////UART 3 Protocol STOP//////////////////////////////////////////////////////

//////////////////////////////////WiFi Protcol START/////////////////////////////////////////////////////////

/*int wait_for_response(const char *expected, int max_attempts) 
{
    char response[128];

    for (int i = 0; i < max_attempts; i++) 
    {
        USART3_ReceiveString(response, sizeof(response));

        if (strstr(response, expected)) 
        {
            return 1;
        }

        if (strstr(response, "ERROR") || strstr(response, "FAIL")) 
        {
            return 0;
        }

        delay(1000);  // Optional delay before next attempt
    }
    return 0;
}*/


void Send_TempHumidity_AT(float tempC, float humRH) 
{
    char payload[128];
    snprintf(payload, sizeof(payload), "GET /update?api_key=YOUR_API_KEY&field1=%.2f&field2=%.2f\r\n", tempC, humRH);

    /*char cip_cmd[32];
    snprintf(cip_cmd, sizeof(cip_cmd), "AT+CIPSEND=%d\r\n", strlen(payload));

    // Step 1: Start TCP connection
    USART3_SendString("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");
    if (!wait_for_response("OK", 10000)) 
    {
        USART3_SendString("CIPSTART failed\r\n");
        return;
    }

    // Step 2: Tell ESP we are sending X bytes
    USART3_SendString(cip_cmd);
    if (!wait_for_response(">", 10000)) 
    {
        USART3_SendString("CIPSEND prompt failed\r\n");
        return;
    }

    // Step 3: Send actual payload (GET request)
    USART3_SendString(payload);
    if (!wait_for_response("SEND OK", 10000)) 
    {
        USART3_SendString("SEND failed\r\n");
        return;
    }*/
}

//////////////////////////////////WiFi Protcol STOP/////////////////////////////////////////////////////////

static GPIO_InitTypeDef  GPIO_InitStruct;

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    // Init communication peripherals
    USART3_Init();     // USART3 = UART to ESP8266
    I2C2_Init();       // I2C2 = HTS221
    HTS221_Enable();   // Power-on HTS221

    // Connect to Wi-Fi using AT command
    //USART3_SendString("AT+CWJAP=\"235-SS_2-APT_5\",\"StevensDucks@235\"\r\n");
    /*if (!wait_for_response("WIFI CONNECTED", 10000)) 
    {
        USART3_SendString("Wi-Fi connection failed\r\n");
        while (1); // Stop execution
    }*/

    while (1)
    {
        // Read HTS221 sensor values
        temperature = HTS221_ReadTemperature();
        humidity    = HTS221_ReadHumidity();

        // Send values to ESP8266 (which forwards to ThingSpeak)
        Send_TempHumidity_AT(temperature, humidity);
        delay(5000000);  // ~5s delay (adjust as needed)
    }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler()
{
	while(1);
}
