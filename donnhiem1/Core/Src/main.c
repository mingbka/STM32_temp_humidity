/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "CLCD_I2C.h"
#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	DISPLAY_TEMP,
	DISPLAY_HUMID,
	DISPLAY_ALL
} DisplayMode_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int p = 5000;
volatile DisplayMode_t DisplayMode = DISPLAY_ALL;
CLCD_I2C_Name LCD1;

/* shtc3 -----------------------------------------------*/
#define SHTC3_ADDRESS (0x70<<1)
uint8_t rev_buffer[6];				// buffer giao tiep voi cam bien
uint8_t wakeup_cmd[2] = {0x35,0x17};
uint8_t measure_cmd[2] = {0x7C,0xA2};
uint8_t sleep_cmd[2] = {0xB0,0x98};
float temperature = 0.0f;
float humidity = 0.0f;

uint8_t rxData[5];
uint32_t start_time_SHTC3, end_time_SHTC3, elapsed_time_SHTC3;
uint32_t start_time_CLCD, end_time_CLCD, elapsed_time_CLCD;
uint32_t start_time_UART, end_time_UART, elapsed_time_UART;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void measure();
void write_clcd();
void send_uart();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
__weak void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  if(huart->Instance == USART1){
	 // rxData[sizeof(rxData) - 1] = '\0';
	if ((rxData[0] == 't') && (rxData[1] == 'e') && (rxData[2] == 'm') && (rxData[3] == 'p'))
	{
		DisplayMode = DISPLAY_TEMP;
		printf("Change Display Mode to DISPLAY_TEMP\r\n\n");
		//memset(rxData, 0, sizeof(rxData));
	}
	else if ((rxData[0] == 'h') && (rxData[1] == 'u') && (rxData[2] == 'm') && (rxData[3] == 'i'))
	{
		DisplayMode = DISPLAY_HUMID;
		printf("Change Display Mode to DISPLAY_HUMI\r\n\n");
		//memset(rxData, 0, sizeof(rxData));
	}
	else if ((rxData[0] == 'b') && (rxData[1] == 'o') && (rxData[2] == 't') && (rxData[3] == 'h'))
	{
		DisplayMode = DISPLAY_ALL;
		printf("Change Display Mode to DISPLAY_ALL\r\n\n");
		//memset(rxData, 0, sizeof(rxData));
	}
	else
	{
		printf("Error Command Syntax\r\n\n");
	}
	HAL_UART_Receive_IT(&huart1, rxData, sizeof(rxData) - 1);
  }

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
}
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  CLCD_I2C_Init(&LCD1, &hi2c2, 0X4e, 20, 4);
  CLCD_I2C_SetCursor(&LCD1,0,0);
  CLCD_I2C_WriteString(&LCD1,"Start");
  HAL_UART_Receive_IT(&huart1, rxData, sizeof(rxData) - 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
  if(uwTick%p == 0){
	  start_time_SHTC3 = HAL_GetTick();
	  measure();
	  end_time_SHTC3 = HAL_GetTick();
	  elapsed_time_SHTC3 = -(start_time_SHTC3 - end_time_SHTC3);

	  start_time_CLCD = HAL_GetTick();
	  write_clcd();
	  end_time_CLCD = HAL_GetTick();
	  elapsed_time_CLCD = -(start_time_CLCD - end_time_CLCD);

	  start_time_UART = HAL_GetTick();
	  send_uart();
	  end_time_UART = HAL_GetTick();
	  elapsed_time_UART = -(start_time_UART - end_time_UART);

	  //HAL_Delay(2000);
		/* USER CODE BEGIN 3 */
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void measure(){
	uint16_t hex_ther;
	uint16_t hex_moisture;
	HAL_I2C_Master_Transmit(&hi2c1,SHTC3_ADDRESS, wakeup_cmd ,2, 500);
	HAL_Delay(1);
	HAL_I2C_Master_Transmit(&hi2c1,SHTC3_ADDRESS, measure_cmd ,2, 500);
	HAL_Delay(15);
	HAL_I2C_Master_Receive(&hi2c1,SHTC3_ADDRESS, rev_buffer ,6,500);
	HAL_I2C_Master_Transmit(&hi2c1,SHTC3_ADDRESS, sleep_cmd ,2, 500);
	hex_ther = (rev_buffer[0]<<8)|rev_buffer[1];
	hex_moisture = (rev_buffer[3]<<8)|rev_buffer[4];
	temperature = -45.0f + 175.0f * (float)hex_ther / 65535.0f;
	humidity = 100.0f * (float)hex_moisture / 65535.0f;
}

void write_clcd(){
// char temp_str[16];
  char lcd_str[16];
  switch(DisplayMode){
  case DISPLAY_ALL:
	sprintf(lcd_str, "Humidity: %.2f%%", humidity);
	CLCD_I2C_SetCursor(&LCD1,0,1);
	CLCD_I2C_WriteString(&LCD1,lcd_str);
	sprintf(lcd_str, "Temp: %.2f C   ",temperature);
	CLCD_I2C_SetCursor(&LCD1,0,0);
	CLCD_I2C_WriteString(&LCD1,lcd_str);
	break;
  case DISPLAY_HUMID:
	sprintf(lcd_str, "Humidity: %.2f%%", humidity);
	CLCD_I2C_SetCursor(&LCD1,0,0);
	CLCD_I2C_WriteString(&LCD1,lcd_str);
	CLCD_I2C_SetCursor(&LCD1,0,1);
	CLCD_I2C_WriteString(&LCD1,"                ");
	break;
  case DISPLAY_TEMP:
	sprintf(lcd_str, "Temp: %.2f C   ",temperature);
	CLCD_I2C_SetCursor(&LCD1,0,0);
	CLCD_I2C_WriteString(&LCD1,lcd_str);
	CLCD_I2C_SetCursor(&LCD1,0,1);
	CLCD_I2C_WriteString(&LCD1,"                ");
	break;
  }
}
void send_uart(){
  char data[64];
  sprintf(data, "Temperature: %.2f C, Humidity: %.2f%%\r\n", temperature, humidity);
  HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen(data), 1000);
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
