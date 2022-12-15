/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RESET_CPU 			SCB->AIRCR = 0x05FA0004
#define UART_PZEM 			&huart1
#define UART_MQTT			&huart2
#define GPIO_PORT_MUX_A		GPIOC
#define GPIO_PIN_MUX_A		GPIO_PIN_0
#define GPIO_PORT_MUX_B		GPIOC
#define GPIO_PIN_MUX_B		GPIO_PIN_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t str[150]={0};

uint8_t V[4][3] =  {{0}};
uint8_t A[4][3] =  {{0}};
uint8_t W[4][3] =  {{0}};
uint8_t Wh[4][3] = {{0}};

uint8_t buf[7] = {0};

uint8_t sendVoltage[7] = 	{0xB0,0xC0,0xA8,0x01,0x01,0x00,0x1A};	// Read the current voltage
uint8_t sendCurrent[7] = 	{0xB1,0xC0,0xA8,0x01,0x01,0x00,0x1B};	// Read the current current
uint8_t sendPower[7] = 		{0xB2,0xC0,0xA8,0x01,0x01,0x00,0x1C};	// Read the current power
uint8_t sendEnergy[7] = 	{0xB3,0xC0,0xA8,0x01,0x01,0x00,0x1D};	// Read the energy

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void cntUART(uint8_t i);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Очистка переменных перед опросом
	  memset(V,0, sizeof(V));	memset(A,0, sizeof(A));		memset(W,0, sizeof(W));		memset(Wh,0, sizeof(Wh));
	  // Запрашиваем данные
	  // Напряжение
	  for(uint8_t i = 0; i < 4; i++){
		  HAL_UART_Abort(UART_PZEM);
		  cntUART(i);
		  if(HAL_UART_Transmit(UART_PZEM, (uint8_t *)sendVoltage, 7, 100) == HAL_OK){
			  if(HAL_UART_Receive(UART_PZEM, (uint8_t *)buf, 7, 100) == HAL_OK){
				  if(buf[1]+buf[2] == 160){
					  RESET_CPU; // Если сбился порядок ресет контроллера
				  }else{
					  if(buf[0] == 0xA0){ V[i][0] = buf[1]; V[i][1] = buf[2]; V[i][2] = buf[3]; }
				  }
			  }
		  }
		  memset(buf,0,7);
		  HAL_Delay(100);
	  }
	  HAL_Delay(1000);
	  // Ток
	  for(uint8_t i = 0; i < 4; i++){
		  HAL_UART_Abort(UART_PZEM);
		  cntUART(i);
		  if(HAL_UART_Transmit(UART_PZEM, (uint8_t *)sendCurrent, 7, 100) == HAL_OK){
			  if(HAL_UART_Receive(UART_PZEM, (uint8_t *)buf, 7, 100) == HAL_OK){
				  if(buf[0] == 0xA1){ A[i][0] = buf[2]; A[i][1] = buf[3]; }
			  }
		  }
		  memset(buf,0,7);
		  HAL_Delay(5);
	  }
	  HAL_Delay(1000);
	  // Мощьность
	  for(uint8_t i = 0; i < 4; i++){
		  HAL_UART_Abort(UART_PZEM);
		  cntUART(i);
		  if(HAL_UART_Transmit(UART_PZEM, (uint8_t *)sendPower, 7, 100) == HAL_OK){
			  if(HAL_UART_Receive(UART_PZEM, (uint8_t *)buf, 7, 100) == HAL_OK){
				if(buf[0] == 0xA2){ W[i][0] = buf[1]; W[i][1] = buf[2]; }
			  }
		  }
		  memset(buf,0,7);
		  HAL_Delay(5);
	  }
  	  HAL_Delay(1000);
	  // Потреб. мощьность
	  for(uint8_t i = 0; i < 4; i++){
	  		  HAL_UART_Abort(UART_PZEM);
	  		  cntUART(i);
	  			  if(HAL_UART_Transmit(UART_PZEM, (uint8_t *)sendEnergy, 7, 100) == HAL_OK){
	  				  if(HAL_UART_Receive(UART_PZEM, (uint8_t *)buf, 7, 100) == HAL_OK){
	  					  if(buf[0] == 0xA3){ Wh[i][0] = buf[1]; Wh[i][1] = buf[2]; Wh[i][2] = buf[3]; }
	  				  }
	  			  }
	  		  memset(buf,0,7);
	  		  HAL_Delay(5);
	  	  }


  // Формируем данные в UART
	  sprintf(str, "PZEM-004T value: \r\n"
			  " 1 - %d.%dV / %d.%02dA / %dW / %dWh\r\n"
			  " 2 - %d.%dV / %d.%02dA / %dW / %dWh\r\n"
			  " 3 - %d.%dV / %d.%02dA / %dW / %dWh\r\n"
			  " 4 - %d.%dV / %d.%02dA / %dW / %dWh\r\n\r\n",
			  V[0][0]+V[0][1], V[0][2], A[0][0], A[0][1], W[0][0] + W[0][1], Wh[0][0]+Wh[0][1]+Wh[0][2],
			  V[1][0]+V[1][1], V[1][2], A[1][0], A[1][1], W[1][0] + W[1][1], Wh[1][0]+Wh[1][1]+Wh[1][2],
			  V[2][0]+V[2][1], V[2][2], A[2][0], A[2][1], W[2][0] + W[2][1], Wh[2][0]+Wh[2][1]+Wh[2][2],
			  V[3][0]+V[3][1], V[3][2], A[3][0], A[3][1], W[3][0] + W[3][1], Wh[3][0]+Wh[3][1]+Wh[3][2]); // Формируем данные для отправки в UART

	  HAL_UART_Transmit(UART_MQTT, str, strlen(str), 100); // Отправляем данные в UART
	  HAL_Delay(1500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void cntUART(uint8_t i){
	  if(i == 0) { HAL_GPIO_WritePin(GPIO_PORT_MUX_A, GPIO_PIN_MUX_A, 0); HAL_GPIO_WritePin(GPIO_PORT_MUX_B, GPIO_PIN_MUX_B, 0); }
	  if(i == 1) { HAL_GPIO_WritePin(GPIO_PORT_MUX_A, GPIO_PIN_MUX_A, 1); HAL_GPIO_WritePin(GPIO_PORT_MUX_B, GPIO_PIN_MUX_B, 0); }
	  if(i == 2) { HAL_GPIO_WritePin(GPIO_PORT_MUX_A, GPIO_PIN_MUX_A, 0); HAL_GPIO_WritePin(GPIO_PORT_MUX_B, GPIO_PIN_MUX_B, 1); }
	  if(i == 3) { HAL_GPIO_WritePin(GPIO_PORT_MUX_A, GPIO_PIN_MUX_A, 1); HAL_GPIO_WritePin(GPIO_PORT_MUX_B, GPIO_PIN_MUX_B, 1); }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
