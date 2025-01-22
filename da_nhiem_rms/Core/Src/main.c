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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "CLCD_I2C.h"
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
volatile DisplayMode_t DisplayMode = DISPLAY_ALL;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* Definitions for measure */
osThreadId_t measureHandle;
const osThreadAttr_t measure_attributes = {
  .name = "measure",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for write_clcd */
osThreadId_t write_clcdHandle;
const osThreadAttr_t write_clcd_attributes = {
  .name = "write_clcd",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for send_uart */
osThreadId_t send_uartHandle;
const osThreadAttr_t send_uart_attributes = {
  .name = "send_uart",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for bin_sem */
osSemaphoreId_t bin_semHandle;
const osSemaphoreAttr_t bin_sem_attributes = {
  .name = "bin_sem"
};
/* USER CODE BEGIN PV */
int a,b,c;
uint8_t rxData[5];
float temperature = 0.0f;
float humidity = 0.0f;
CLCD_I2C_Name LCD1;

#define SHTC3_ADDRESS (0x70<<1)
uint8_t rev_buffer[6];
uint8_t wakeup_cmd[2] = {0x35, 0x17};
uint8_t measure_cmd[2] = {0x7C, 0xA2};
uint8_t sleep_cmd[2] = {0xB0, 0x98};

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
void measure_task(void *argument);
void write_clcd_task(void *argument);
void send_uart_task(void *argument);

/* USER CODE BEGIN PFP */

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
	}
	else if ((rxData[0] == 'h') && (rxData[1] == 'u') && (rxData[2] == 'm') && (rxData[3] == 'i'))
	{
		DisplayMode = DISPLAY_HUMID;
		printf("Change Display Mode to DISPLAY_HUMI\r\n\n");
	}
	else if ((rxData[0] == 'b') && (rxData[1] == 'o') && (rxData[2] == 't') && (rxData[3] == 'h'))
	{
		DisplayMode = DISPLAY_ALL;
		printf("Change Display Mode to DISPLAY_ALL\r\n\n");
	}
	else
	{
		printf("Error Command Syntax\r\n\n");
	}
	HAL_UART_Receive_IT(&huart1, rxData, sizeof(rxData) - 1);
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

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of bin_sem */
  bin_semHandle = osSemaphoreNew(1, 1, &bin_sem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of measure */
  measureHandle = osThreadNew(measure_task, NULL, &measure_attributes);

  /* creation of write_clcd */
  write_clcdHandle = osThreadNew(write_clcd_task, NULL, &write_clcd_attributes);

  /* creation of send_uart */
  send_uartHandle = osThreadNew(send_uart_task, NULL, &send_uart_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_measure_task */
/**
  * @brief  Function implementing the measure thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_measure_task */
void measure_task(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_write_clcd_task */
/**
* @brief Function implementing the write_clcd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_write_clcd_task */
void write_clcd_task(void *argument)
{
  /* USER CODE BEGIN write_clcd_task */
  /* Infinite loop */
  for(;;)
  {
	b++;
	start_time_CLCD = HAL_GetTick();
	char lcd_line1[17];
	char lcd_line2[17];
	if (osSemaphoreAcquire(bin_semHandle, osWaitForever) == osOK) {
		switch(DisplayMode){
			case DISPLAY_ALL:
				sprintf(lcd_line2, "Humidity: %.2f%%", humidity);
				sprintf(lcd_line1, "Temp: %.2f C   ",temperature);
				break;
			case DISPLAY_HUMID:
				sprintf(lcd_line1, "Humidity: %.2f%%", humidity);
				sprintf(lcd_line2, "                ");
				break;
			case DISPLAY_TEMP:
				sprintf(lcd_line1, "Temp: %.2f C   ",temperature);
				sprintf(lcd_line2, "                ");
				break;
		}
		// Release Semaphore
		osSemaphoreRelease(bin_semHandle);
	}
	CLCD_I2C_SetCursor(&LCD1,0,0);
	CLCD_I2C_WriteString(&LCD1,lcd_line1);
	CLCD_I2C_SetCursor(&LCD1,0,1);
	CLCD_I2C_WriteString(&LCD1,lcd_line2);
	end_time_CLCD = HAL_GetTick();
	elapsed_time_CLCD = -(start_time_CLCD - end_time_CLCD);
	osDelayUntil(start_time_CLCD+1000);
  }
  /* USER CODE END write_clcd_task */
}

/* USER CODE BEGIN Header_send_uart_task */
/**
* @brief Function implementing the send_uart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_send_uart_task */
void send_uart_task(void *argument)
{
  /* USER CODE BEGIN send_uart_task */
  /* Infinite loop */
  for(;;)
  {
	c++;
	start_time_UART = HAL_GetTick();
	if (osSemaphoreAcquire(bin_semHandle, osWaitForever) == osOK) {
	char data[64];
		sprintf(data, "Temperature: %.2f C, Humidity: %.2f%%\r\n", temperature, humidity);
		HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen(data), 1000);

		// Release Semaphore
		osSemaphoreRelease(bin_semHandle);
	}
	end_time_UART = HAL_GetTick();
	elapsed_time_UART = -(start_time_UART - end_time_UART);
	osDelayUntil(start_time_UART+1000);
  }
  /* USER CODE END send_uart_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
