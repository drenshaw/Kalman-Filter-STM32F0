/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include <stdarg.h>
#include "kalman.h"

/* Defines */
#define RED 6
#define BLUE 7
#define ORANGE 8
#define GREEN 9
#define ALL 0x3C0
#define OFF 0
#define ON 1
#define TOGGLE 2
#define WRITE 0
#define READ 1
#define THRESHOLD 2500

/* Private variables ---------------------------------------------------------*/
char inputChar;
int inputFlag;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void LED_setup(void);
void usart3_init(void);
void i2c2_init(void);
void USART3_4_IRQHandler(void);
void sendChar(char output);
void sendString(char* output);
int compare(char* left, char* right, int length);
int LED_ctrl(int ctrl, int count, ...);
void i2c2_send(uint16_t addr,uint8_t num_bytes,uint8_t rw);
uint8_t i2c2_read(uint16_t addr,int tx_reg,uint8_t num_bytes,uint8_t WHO_AM_I);
void gyro_setup(uint16_t ctrl_reg, uint16_t axes);
int16_t axis(uint8_t addr, uint8_t axis_addr);

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
	HAL_Init();
  SystemClock_Config();

	/* LED initialization */
  LED_setup();
	
	/* USART3 initialization */
//************** ENABLE WHEN COMMUNICATING WITH COMPUTER!!**********//
	//usart3_init();
  
	/* I2C2 initialization */
	i2c2_init();
	
  /* Prepare gyro to read WHO_AM_I value */
	i2c2_send(0x6B,1,WRITE);
	
	/* I2C read and check for WHO_AM_I register value */
	i2c2_read(0x6B,0x0F,1,0xD4);
  
	/* Enable write for axis access on gyro */
	i2c2_send(0x6B,2,WRITE);
  
	/* Set up gyro ctrl_reg and axes */
	/**	Gyroscope axis enable register CTRL_REG1 layout	**/
	/*	x		|		x		|		x		|		x		|		x		|		x		|		x		|		x		
			DR1	|		DR0	|		BW1	|		BW0	|		PD	|		ZEn	|		XEn	|		YEn	
	Settings for L3GD20 CTRL_REG1 (8 bits)
	DR[1:0] : 00 = 095 Hz
						01 = 190 Hz
						10 = 380 Hz
						11 = 760 Hz
	BW[1:0]	:	00 = 12.5	Hz
						01 = 25.0 Hz
						10 = 50.0 Hz
						11 = 70.0 Hz
	PD = power down pin
	*/
	// Default speed (95Hz) and bandwidth (12.5Hz)
	// Enable x,y,z; enable power-down
	gyro_setup(0x20, 0x0F);	
	
	while(1)	// Begin infinite while loop (main program loop)
	{
		/* Wait 100 ms to ensure gyro completion of previous task */
		HAL_Delay(100);
		
		/* Get L3GD20 axis data */
		int16_t xaxis = axis(0x6B,0xA8);	// A8 to read both hi and low
		int16_t yaxis = axis(0x6B,0xAA);	// AA to read both hi and low
		//int16_t zaxis = axis(0x6B,0xAC);// AC to read both hi and low
		
		// Turn off all LEDs to eliminate previous visual values
		GPIOC->BSRR |= (ALL<<16);		// bitshift 16 to the reset part of BSRR
		
		// If magnitude of x-axis change exceeds threshold, light Green/Orange LEDs
		if (xaxis > THRESHOLD) 
			GPIOC->BSRR |= (1 << GREEN);
		else if (xaxis < -THRESHOLD)
			GPIOC->BSRR |= (1 << ORANGE);
		
		// If magnitude of y-axis change exceeds threshold, light Red/Blue LEDs
		if (yaxis > THRESHOLD)
			GPIOC->BSRR |= (1 << RED);
		else if (yaxis < -THRESHOLD)
			GPIOC->BSRR |= (1 << BLUE);
		
	}	// End infinite while loop
}	// End main loop (should never reach!!)

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/*************************************************/
/**************** USER Functions *****************/
/*************************************************/
/* LED setup code */
void LED_setup(void)
{
	/* LED Setup */
	// RCC enable
	RCC->AHBENR |=	RCC_AHBENR_GPIOCEN;
	
	// PC6, PC7, PC8, PC9 to output mode
	GPIOC->MODER 		|=	GPIO_MODER_MODER6_0	|	GPIO_MODER_MODER7_0	|
											GPIO_MODER_MODER8_0	|	GPIO_MODER_MODER9_0;	//(1 << 12) | (1 << 14) | (1 << 16) | (1 << 18);
	GPIOC->MODER 		&=~(GPIO_MODER_MODER6_1 |	GPIO_MODER_MODER7_1 |
											GPIO_MODER_MODER8_1	|	GPIO_MODER_MODER9_1);
	// PC6, PC7, PC8, PC9 to push-pull output type
	GPIOC->OTYPER 	&= ~(GPIO_OTYPER_OT_6	|	GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);
	// PC6, PC7, PC8, PC9 to low speed
	GPIOC->OSPEEDR 	&= ~(	GPIO_OSPEEDR_OSPEEDR6_1 | GPIO_OSPEEDR_OSPEEDR7_1 |
												GPIO_OSPEEDR_OSPEEDR8_1	|	GPIO_OSPEEDR_OSPEEDR9_1);
	// PC6, PC7, PC8, PC9 to no pull-up/pull-down
	GPIOC->PUPDR		&= ~(	GPIO_PUPDR_PUPDR6	|	GPIO_PUPDR_PUPDR7	|
												GPIO_PUPDR_PUPDR8	|	GPIO_PUPDR_PUPDR9);
}

/* Enable USART3 */
void usart3_init(void)
{
	/*UART setup*/
	// RCC init for USART3 and GPIOC for pins PC4/PC5
	RCC->APB1ENR |=	RCC_APB1ENR_USART3EN;
	RCC->AHBENR |=	RCC_AHBENR_GPIOCEN;
	
	// Alternate function mode for PC4, PC5
  GPIOC->MODER |= (1 << 9) | (1 << 11);
  //GPIOB->MODER &= ~((1 << 20) | (1 << 22));
	
	// Set to AF1 for PC4 (UART3_tx), PC5 (UART3_rx).
	GPIOC->AFR[0] |= (1 << 20) | (1 << 16);
	
	// Set the baud rate of USART3 to 115200
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;
	
	// Enable RX and TX in USART3, TX/RX enable, RXNE interrupt enable
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
	
	// Enable USART3
	USART3->CR1 |= USART_CR1_UE;
	
	// Enable USART3/4 interrupt in NVIC
	NVIC_EnableIRQ(USART3_4_IRQn);
}	

void i2c2_init(void)
{
	/*I2C Setup*/
	// RCC setup for I2C2 and GPIOB/GPIOC for pins PB11/PB13/PB14/PC0
	RCC->APB1ENR |=	RCC_APB1ENR_I2C2EN;
	RCC->AHBENR |=	RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
	// Set PB11 to alt func mode, open-drain, and select I2C2_SDA as alternate function
	// Set PB13 to alt func mode, open-drain, and select I2C2_SCL as alternate function
	// PB11, PB13 to alternatate function mode
	GPIOB->MODER |= (1 << 23) | (1 << 27);
	
	// AF1 to PB11 (I2C_SDA), AF5 to PB13 (I2C_SCL)
	GPIOB->AFR[1] |= (1 << 12) | ((1 << 20) | (1 << 22));
	
	// PB11, PB13 to open drain type
	GPIOB->OTYPER |= (1 << 13) | (1 << 11);
	
	// Set PB14 to output mode, push-pull output type, and set the pin high
	// PB14 to output mode
	GPIOB->MODER |= (1 << 28);
	
	// Set PB14 output to high
	GPIOB->ODR |= (1 << 14);
	
	// PC0 to output mode
	GPIOC->MODER |= (1 << 0);

	// PC0 output to high
	GPIOC->ODR |= (1 << 0);
	
	// Setup timing register
	I2C2->TIMINGR |= (1 << 28) | (0x13) | (0xF << 8) | (0x2 << 16) | (0x4 << 20);
	
	// Enable the I2C peripheral using the PE bit in CR1 register
  I2C2->CR1 |= I2C_CR1_PE;
}

void USART3_4_IRQHandler()
{
	inputChar = USART3->RDR;
	inputFlag = 1;
}

void sendChar(char output)
{
	while(!(USART3->ISR & USART_ISR_TXE)) {	}
	USART3->TDR = output;
}

void sendString(char* output)
{
	int i = 0;
	while (output[i] != '\0') {
		sendChar(output[i]);
		i++;
	}
}

int compare(char* left, char* right, int length)
{
	int i;
	for (i = 0; i < length; i++) {
		if (left[i] != right[i]) {
			return 0;
		}
	}
	return 1;
}


int LED_ctrl(int ctrl, int count, ...)
{
	va_list list;
	va_start(list, count); 
	int i = va_arg(list, int);
	int j = 0;
	int val = 0;
	for(j=0; j < count; j++)
	{
		val += va_arg(list, int);
		if (ctrl == ON)
			GPIOC->ODR |= (1<<(va_arg(list, int)));
		else if(ctrl == OFF)
			GPIOC->ODR &= ~(1<<(va_arg(list, int)));
		else if(ctrl == TOGGLE)
			GPIOC->ODR ^= (1<<(va_arg(list, int)));
		else
			return i;
	}

	va_end(list);
	return 0;
}

void i2c2_send(uint16_t addr,uint8_t num_bytes,uint8_t rw)
{
	// Set the address to the gyro (L3GD20) peripheral
	// Shift addr (0x6B) by 1 as this is 7-bit addr
	I2C2->CR2 = (addr << 1);		

	// Set number of bytes to send to 1
	I2C2->CR2 |= (num_bytes << I2C_CR2_NBYTES_Pos);

	if (rw)// Enable the read operation
		I2C2->CR2 |= (I2C_CR2_RD_WRN);
	else
		I2C2->CR2 &= ~(I2C_CR2_RD_WRN);		// Write is 0, read is 1
	
	// Set the start bit 
	I2C2->CR2 |= I2C_CR2_START;  
	
	while (!((I2C2->ISR & I2C_ISR_TXIS) | (I2C2->ISR & I2C_ISR_NACKF))) 
	{	// Wait for either transmit interrupt or NACKF status
	}
	
	if ((I2C2->ISR & I2C_ISR_NACKF))
	{	// NACKF
		GPIOC->ODR &= ~(ALL);
		HAL_Delay(500);
		GPIOC->ODR |= ALL;
		HAL_Delay(500);
	}
	if ((I2C2->ISR & I2C_ISR_TXIS))
	{
		//GPIOC->ODR |= ALL;
	}
}

uint8_t i2c2_read(uint16_t addr,int tx_reg,uint8_t num_bytes,uint8_t WHO_AM_I)
{
	// Transmit WHO_AM_I register info
  I2C2->TXDR = tx_reg;
  
  // Wait for TC flag to set
  while (!(I2C2->ISR & I2C_ISR_TC))
	{ // Wait for transfer complete
	}
  
  // Set the address to the correct peripheral.
  I2C2->CR2 = (addr << 1);
  
  // Set number of bytes to read to 1
  I2C2->CR2 |= (num_bytes << I2C_CR2_NBYTES_Pos);
  
	// Enable the read operation
	I2C2->CR2 |= (I2C_CR2_RD_WRN);
  
  // Set the start bit 
  I2C2->CR2 |= (I2C_CR2_START);
  
  while (!((I2C2->ISR & I2C_ISR_RXNE) | (I2C2->ISR & I2C_ISR_NACKF)))
	{	// Wait for either receive data reg not empty or NACKF status
	}
  
  uint8_t who = I2C2->RXDR;
	if (who != WHO_AM_I)
	{
		GPIOC->ODR |= (1<<ORANGE);
		return 1;
	}
  
  // Wait for TC flag to set
  while (!(I2C2->ISR & I2C_ISR_TC))
	{ // Wait for transfer complete
	}
	
	return 0;
}

void gyro_setup(uint16_t ctrl_reg, uint16_t axes)
{
	// Set the address to CTRL_REG1
  I2C2->TXDR = ctrl_reg;
  
  // Wait for TXIS flag to set
  while(!((I2C2->ISR & I2C_ISR_TXIS) | (I2C2->ISR & I2C_ISR_NACKF)))
	{	// Wait for either transmit interrupt or NACKF status
	}

  // Set the XEN, YEN, and PEN
  I2C2->TXDR = axes;
  
  // Wait for TC flag to set
  while (!(I2C2->ISR & I2C_ISR_TC))
	{ // Wait for transfer complete
	}
}

int16_t axis(uint8_t peripheral_addr, uint8_t axis_addr)
{
	// Clear bits for address, number of bytes, and read/write
	I2C2->CR2 &= ~((0x7F << I2C_CR2_NBYTES_Pos) | (0x3FF << I2C_CR2_SADD_Pos));
	I2C2->CR2 &= ~I2C_CR2_RD_WRN;
	
	// Set address and number of bytes
	I2C2->CR2 |= (peripheral_addr << 1) | (1 << I2C_CR2_NBYTES_Pos);
	
	// Start I2C transaction
	I2C2->CR2 |= I2C_CR2_START;
	
	while (!((I2C2->ISR & I2C_ISR_TXIS) | (I2C2->ISR & I2C_ISR_NACKF))) 
	{	// Wait for either transmit interrupt or NACKF status
	}
	if (I2C2->ISR & I2C_ISR_NACKF) 
	{// ERROR: NACK
	}
	
	// Send gyro address
	I2C2->TXDR = axis_addr;  // gyro n-axis address (n = x, y, or z)
	
	while (!(I2C2->ISR & I2C_ISR_TC))
	{ // Wait for transfer complete
	}
	
	// Clear bits for address and number of bytes
	I2C2->CR2 &= ~((I2C_CR2_NBYTES) | (I2C_CR2_SADD));
	
	// Set address, number of bytes, and read condition
	I2C2->CR2 |= (peripheral_addr << 1) | (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_RD_WRN;
	
	// Start I2C transaction
	I2C2->CR2 |= (I2C_CR2_START);
	
	while (!((I2C2->ISR & I2C_ISR_RXNE) | (I2C2->ISR & I2C_ISR_NACKF)))
	{	// Wait for either receive data reg not empty or NACKF status
	}
	if (I2C2->ISR & I2C_ISR_NACKF) 
	{// ERROR: NACK
	}
	
	//Store lower bits of axis
	int16_t axis = I2C2->RXDR;
	while (!((I2C2->ISR & I2C_ISR_RXNE) | (I2C2->ISR & I2C_ISR_NACKF)))
	{	// Wait for either receive data reg not empty or NACKF status
	}
	
	if (I2C2->ISR & I2C_ISR_NACKF)
	{// ERROR: NACK
	}
	
	//Store upper bits of axis
	axis |= (I2C2->RXDR << 8);
	while (!(I2C2->ISR & I2C_ISR_TC))
	{	// Wait for transfer complete
	}
	
	//Release I2C bus and return axis value
	I2C2->CR2 |= I2C_CR2_STOP;
	return axis;
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
		LED_ctrl(4, ON, RED,BLUE,GREEN,ORANGE);
		HAL_Delay(250);
		LED_ctrl(4, OFF, RED,BLUE,GREEN,ORANGE);
		HAL_Delay(250);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
