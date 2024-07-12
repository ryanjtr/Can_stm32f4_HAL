/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t checkTx = 0;
uint8_t checkRx = 0;

uint8_t success_msg[30] = "Receive successfully\n";
uint8_t fail_msg[30] = "Receive fail\n";
char msg[30];
char trans_fail[30] = "Transmit fail\n";
uint8_t rx_msg[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void CAN_Tx();
void CAN_Rx();
void CAN_Filter_Config();
// void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
//{
//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3); //neu nhan duoc byte tin hieu thi led 2 sang
//
// }
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  CAN_Filter_Config();

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    CAN_Tx();

    // Watchdog
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
    HAL_Delay(1000);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  HAL_NVIC_SetPriority(CAN1_TX_IRQn, 15, 0);
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 15, 0);
  HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 15, 0);
  HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 15, 0);

  HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE END CAN1_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Anti_WDG_GPIO_Port, Anti_WDG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Anti_WDG_Pin */
  GPIO_InitStruct.Pin = Anti_WDG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Anti_WDG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void CAN_Tx()
{
  CAN_TxHeaderTypeDef txHeader;
  uint8_t my_msg[8] = {0, 1, 2, 3, 4, 5, 6, 7};
  uint32_t canMailbox; // CAN Bus Mail box variable
  txHeader.DLC = 8;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.StdId = 0x123;
  if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, my_msg, &canMailbox) != HAL_OK)
  {

    Error_Handler();
  }
  //  while (HAL_CAN_IsTxMessagePending(&hcan1, canMailbox))
  //    ; // polling
}

void CAN_Rx()
{
  CAN_RxHeaderTypeDef rxHeader;

  while (!HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0))
  {
    HAL_UART_Transmit(&huart2, fail_msg, 30, 1000);
    HAL_Delay(2000);
  }
  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rx_msg) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_UART_Transmit(&huart2, success_msg, 30, 1000);
}

void CAN_Filter_Config()
{
  CAN_FilterTypeDef can1_filter_init;
  can1_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
  can1_filter_init.FilterBank = 0;
  can1_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;
  can1_filter_init.FilterIdHigh = 0x0000;
  can1_filter_init.FilterIdLow = 0;
  can1_filter_init.FilterMaskIdHigh = 0;
  can1_filter_init.FilterMaskIdLow = 0;
  can1_filter_init.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can1_filter_init.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &can1_filter_init);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
  sprintf(msg, "Transmit successfully M0\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, 30, 1000);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
  sprintf(msg, "Transmit successfully M1\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, 30, 1000);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
  sprintf(msg, "Transmit successfully M2\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, 30, 1000);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rxHeader;

  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rx_msg) != HAL_OK)
  {
    sprintf(msg, "Receive Fail\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, 30, 1000);
  }
  else
  {
    sprintf(msg, "Receive Successfully\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, 30, 1000);
  }
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
}
 void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	sprintf(msg, "Error happen\n");
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, 30, 1000);
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
