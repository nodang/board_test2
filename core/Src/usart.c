/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "tim.h"

#include <memory.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define TX_LED_ON	HAL_GPIO_WritePin(transmit_led_GPIO_Port,transmit_led_Pin,RESET)
#define TX_LED_OFF	HAL_GPIO_WritePin(transmit_led_GPIO_Port,transmit_led_Pin,SET)
#define RX_LED_ON	HAL_GPIO_WritePin(receive__led_GPIO_Port,receive__led_Pin,RESET)
#define RX_LED_OFF	HAL_GPIO_WritePin(receive__led_GPIO_Port,receive__led_Pin,SET)

#define BUF_SIZE 128
static uint8_t buf;
static uint8_t rxBuffer[BUF_SIZE];
static uint8_t txBuffer[BUF_SIZE];
uint8_t txERR[] = "Something's wrong\r\n";
uint32_t tim_cnt = 0;

char* CR = "\r";

void USARTx_TxString(volatile UART_HandleTypeDef *USARTx, char *Str)
{
	uint16_t strCnt = 0;
	while(*Str)
	{
		if(*Str == '\n') {
			strcat((char*)txBuffer, CR);
			strCnt++;
		}
		strncat((char*)txBuffer, Str++, 1);
		strCnt++;
	}
	if(strCnt == 0)
		return;

	//HAL_UART_Transmit(&huart1, txBuffer, ++strCnt, 15);
	HAL_UART_Transmit(&huart1, (uint8_t*)txBuffer, ++strCnt, 15);

	memset((void*)txBuffer, 0x00, sizeof(uint8_t)*BUF_SIZE);
}


void TxPrintf(char *Form, ... )
{
	TX_LED_ON;
	
	tim_cnt = htim1.Instance->CNT;
	static char Buff[BUF_SIZE];
	va_list ArgPtr;
	va_start(ArgPtr,Form);
	vsprintf(Buff, Form, ArgPtr);
	va_end(ArgPtr);
	USARTx_TxString(&huart1, Buff);
	tim_cnt -= htim1.Instance->CNT;
	
	TX_LED_OFF;
}

void RxBuffer(void)
{
	if(buf == '\r' || buf == '\n') {
		TxPrintf("%s\n", rxBuffer);
		//TxPrintf(" | tm :%d\n", tim_cnt);
		memset((void*)rxBuffer, 0x00, sizeof(uint8_t)*BUF_SIZE);
	}
	else {
		strcat((char*)rxBuffer, (char*)&buf);
		//TxPrintf("%s\n", rxBuffer);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	RX_LED_ON;
	
	if(huart->Instance == USART1) {
		RxBuffer();
	}
	HAL_UART_Receive_IT(&huart1, &buf, 1);
	
	RX_LED_OFF;
}

void Receive_DMA(void)
{
	HAL_UART_Receive_IT(&huart1, &buf, 1);
	//HAL_UART_Receive_DMA(&huart1, rxBuffer, DMA_BUF_SIZE);
}

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
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

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
