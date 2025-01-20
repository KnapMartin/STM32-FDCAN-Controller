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

#include "../../Project/fdcan_controller.h"

#include <stdio.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef huart2;

/* Definitions for task0 */
osThreadId_t task0Handle;
const osThreadAttr_t task0_attributes =
{ .name = "task0", .stack_size = 256 * 4, .priority =
		(osPriority_t) osPriorityNormal };
/* Definitions for task1 */
osThreadId_t task1Handle;
const osThreadAttr_t task1_attributes =
{ .name = "task1", .stack_size = 256 * 4, .priority =
		(osPriority_t) osPriorityNormal };
/* Definitions for task2 */
osThreadId_t task2Handle;
const osThreadAttr_t task2_attributes =
{ .name = "task2", .stack_size = 256 * 4, .priority =
		(osPriority_t) osPriorityNormal };
/* Definitions for task2 */
osThreadId_t task3Handle;
const osThreadAttr_t task3_attributes =
{ .name = "task3", .stack_size = 256 * 4, .priority =
		(osPriority_t) osPriorityNormal };
/* Definitions for queueCan */
osMessageQueueId_t queueCanFifo0Handle;
const osMessageQueueAttr_t queueCanFifo0_attributes =
{ .name = "queueCanFifo0" };
/* Definitions for queueCan */
osMessageQueueId_t queueCanFifo1Handle;
const osMessageQueueAttr_t queueCanFifo1_attributes =
{ .name = "queueCanFifo1" };
/* Definitions for mutexCan */
osMutexId_t mutexCanHandle;
const osMutexAttr_t mutexCan_attributes =
{ .name = "mutexCan" };
/* Definitions for semCan */
osSemaphoreId_t semCanHandle;
const osSemaphoreAttr_t semCan_attributes =
{ .name = "semCan" };
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART2_UART_Init(void);
void startTask0(void *argument);
void startTask1(void *argument);
void startTask2(void *argument);
void startTask3(void *argument);

/* USER CODE BEGIN PFP */

FdcanController can;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern "C" int _write(int file, char *data, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t*) data, len, 100);
	return len;
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
	MX_FDCAN1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	printf("Can Test\r\n");

	can.setHandleFdcan(&hfdcan1);
	can.setHandleMutex(&mutexCanHandle);
	can.setHandleQueue(&queueCanFifo0Handle, FdcanController::Buffer::Fifo0);
	can.setHandleQueue(&queueCanFifo1Handle, FdcanController::Buffer::Fifo1);
	can.setHandleSem(&semCanHandle);

	FDCAN_FilterTypeDef filterFifo0;
	filterFifo0.IdType = FDCAN_STANDARD_ID;
	filterFifo0.FilterIndex = 0;
	filterFifo0.FilterType = FDCAN_FILTER_RANGE;
	filterFifo0.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	filterFifo0.FilterID1 = 0x001;
	filterFifo0.FilterID2 = 0x00A;
	if (can.setFilter(filterFifo0) != FdcanController::State::Ok)
	{
		printf("error\r\n");
	}

	FDCAN_FilterTypeDef filterFifo1;
	filterFifo1.IdType = FDCAN_STANDARD_ID;
	filterFifo1.FilterIndex = 1;
	filterFifo1.FilterType = FDCAN_FILTER_RANGE;
	filterFifo1.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	filterFifo1.FilterID1 = 0x00B;
	filterFifo1.FilterID2 = 0x016;
	if (can.setFilter(filterFifo1) != FdcanController::State::Ok)
	{
		printf("error\r\n");
	}

	if (can.init() != FdcanController::State::Ok)
	{
		printf("error\r\n");
	}

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();
	/* Create the mutex(es) */
	/* creation of mutexCan */
	mutexCanHandle = osMutexNew(&mutexCan_attributes);

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of semCan */
	semCanHandle = osSemaphoreNew(1, 0, &semCan_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of queueCan */
	queueCanFifo0Handle = osMessageQueueNew(16, sizeof(FdcanMsg),
			&queueCanFifo0_attributes);
	queueCanFifo1Handle = osMessageQueueNew(16, sizeof(FdcanMsg),
			&queueCanFifo1_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of task0 */
	task0Handle = osThreadNew(startTask0, NULL, &task0_attributes);

	/* creation of task1 */
	task1Handle = osThreadNew(startTask1, NULL, &task1_attributes);

	/* creation of task2 */
	task2Handle = osThreadNew(startTask2, NULL, &task2_attributes);

	/* creation of task3 */
	task3Handle = osThreadNew(startTask3, NULL, &task3_attributes);

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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN1_Init(void)
{

	/* USER CODE BEGIN FDCAN1_Init 0 */

	/* USER CODE END FDCAN1_Init 0 */

	/* USER CODE BEGIN FDCAN1_Init 1 */

	/* USER CODE END FDCAN1_Init 1 */
	hfdcan1.Instance = FDCAN1;
	hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV2;
	hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan1.Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
	hfdcan1.Init.AutoRetransmission = DISABLE;
	hfdcan1.Init.TransmitPause = DISABLE;
	hfdcan1.Init.ProtocolException = DISABLE;
	hfdcan1.Init.NominalPrescaler = 283;
	hfdcan1.Init.NominalSyncJumpWidth = 1;
	hfdcan1.Init.NominalTimeSeg1 = 1;
	hfdcan1.Init.NominalTimeSeg2 = 1;
	hfdcan1.Init.DataPrescaler = 1;
	hfdcan1.Init.DataSyncJumpWidth = 1;
	hfdcan1.Init.DataTimeSeg1 = 1;
	hfdcan1.Init.DataTimeSeg2 = 1;
	hfdcan1.Init.StdFiltersNbr = 0;
	hfdcan1.Init.ExtFiltersNbr = 0;
	hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN FDCAN1_Init 2 */

	/* USER CODE END FDCAN1_Init 2 */

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
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_GREEN_Pin */
	GPIO_InitStruct.Pin = LED_GREEN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan,
		uint32_t BufferIndexes)
{
	if (can.updateInterruptTx(hfdcan) != FdcanController::State::Ok)
	{
		printf("error\r\n");
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if (can.updateInterruptRx(hfdcan, RxFifo0ITs) != FdcanController::State::Ok)
	{
		printf("error\r\n");
	}
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	if (can.updateInterruptRx(hfdcan, RxFifo1ITs) != FdcanController::State::Ok)
	{
		printf("error\r\n");
	}
}

void task_action(char command)
{
	ITM_SendChar(command);
	ITM_SendChar('\n');
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startTask0 */
/**
 * @brief  Function implementing the task0 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_startTask0 */
void startTask0(void *argument)
{
	/* USER CODE BEGIN 5 */

	FdcanMsg msg;
	msg.txHeader.Identifier = 0x006;
	msg.txHeader.IdType = FDCAN_STANDARD_ID;
	msg.txHeader.TxFrameType = FDCAN_DATA_FRAME;
	msg.txHeader.DataLength = FDCAN_DLC_BYTES_8;
	msg.txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	msg.txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	msg.txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	msg.txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	msg.txHeader.MessageMarker = 0;

	msg.data[0] = 'h';
	msg.data[1] = 'e';
	msg.data[2] = 'y';
	msg.data[3] = 'a';
	msg.data[4] = '0';
	msg.data[5] = '\0';

	/* Infinite loop */
	for (;;)
	{
		osDelay(1e3);
		task_action('1');
		can.send(msg);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startTask1 */
/**
 * @brief Function implementing the task1 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startTask1 */
void startTask1(void *argument)
{
	/* USER CODE BEGIN startTask1 */
	/* Infinite loop */
	for (;;)
	{
		FdcanMsg msg;
		can.receive(&msg, FdcanController::Buffer::Fifo0);
		printf("fifo0 got: %s\r\n", reinterpret_cast<char*>(msg.data));
	}
	/* USER CODE END startTask1 */
}

void startTask2(void *argument)
{
	/* USER CODE BEGIN startTask1 */
	FdcanMsg msg;
	msg.txHeader.Identifier = 0x00D;
	msg.txHeader.IdType = FDCAN_STANDARD_ID;
	msg.txHeader.TxFrameType = FDCAN_DATA_FRAME;
	msg.txHeader.DataLength = FDCAN_DLC_BYTES_8;
	msg.txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	msg.txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	msg.txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	msg.txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	msg.txHeader.MessageMarker = 0;

	msg.data[0] = 'h';
	msg.data[1] = 'e';
	msg.data[2] = 'y';
	msg.data[3] = 'a';
	msg.data[4] = '2';
	msg.data[5] = '\0';
	/* Infinite loop */
	for (;;)
	{
		osDelay(1e3);
		task_action('2');
		can.send(msg);
	}
	/* USER CODE END startTask1 */
}

void startTask3(void *argument)
{
	/* USER CODE BEGIN startTask1 */

	/* Infinite loop */
	for (;;)
	{
		FdcanMsg msg;
		can.receive(&msg, FdcanController::Buffer::Fifo1);
		printf("fifo1 got: %s\r\n", reinterpret_cast<char*>(msg.data));
	}
	/* USER CODE END startTask1 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6)
	{
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
