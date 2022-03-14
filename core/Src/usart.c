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

#define BUF_SIZE 128
uint8_t buf;
uint8_t rxBuffer[BUF_SIZE];
uint8_t txBuffer[] = "Something's wrong\r\n";

uint32_t tim3_cnt = 0;


char USARTx_RxChar(volatile UART_HandleTypeDef *USARTx)
{
    while((USARTx->Instance->SR & UART_FLAG_RXNE) == RESET);   //wait for Rx Not Empty flag
    return (uint16_t)(USARTx->Instance->DR & (uint16_t)0x01FF); //read from DR register
}

void USARTx_TxChar(volatile UART_HandleTypeDef *USARTx, char Data)
{
    USARTx->Instance->DR = (Data & (uint16_t)0x01FF);         //write to DR register
    while((USARTx->Instance->SR & UART_FLAG_TXE) == RESET);  //wait for Tx Empty flag
}

void USARTx_TxString(volatile UART_HandleTypeDef *USARTx, char *Str)
{
    while(*Str)
    {
        if(*Str == '\n'){
            USARTx_TxChar(USARTx, '\r');
        }

        USARTx_TxChar(USARTx, *Str++);
    }
}

void TxPrintf(char *Form, ... )
{
	huart1.gState |= HAL_UART_STATE_BUSY_TX;
	static char Buff[BUF_SIZE];
	va_list ArgPtr;
	va_start(ArgPtr,Form);
	vsprintf(Buff, Form, ArgPtr);
    va_end(ArgPtr);
    USARTx_TxString(&huart1, Buff);
	huart1.gState = HAL_UART_STATE_READY;
}

void USARTx_TxString_R(volatile UART_HandleTypeDef *USARTx, char *Str)
{
	static char buff[BUF_SIZE] = {0,};
	static char CR = '\r';
	uint16_t str_cnt = 0;

	while(*Str)
	{
		if(*Str == '\n'){
			strncat((char*)buff, (char*)&CR, 1);
			str_cnt++;
		}
		strncat((char*)buff, (char*)Str++, 1);
		str_cnt++;
	}
	HAL_UART_Transmit(&huart1, (uint8_t*)buff , str_cnt, 5);
}

void TxPrintf_R(char *Form, ... )
{
	tim3_cnt = htim1.Instance->CNT;
	static char Buff[BUF_SIZE] = {0,};
	va_list ArgPtr;
	va_start(ArgPtr,Form);
	vsprintf(Buff, Form, ArgPtr);
	va_end(ArgPtr);
	USARTx_TxString(&huart1, Buff);
	tim3_cnt -= htim1.Instance->CNT;
}

void RxBuffer(void)
{
	if(buf == '\r' || buf == '\n') {
		TxPrintf_R("fdbk :%s |", rxBuffer);
		TxPrintf("tm :%d\n", tim3_cnt);
		memset((void*)rxBuffer, 0x00, sizeof(uint8_t)*BUF_SIZE);
	}
	else {
		strncat((char*)rxBuffer, (char*)&buf, 1);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_UART_Transmit_DMA(&huart1, &buf, 1);
	if(huart->Instance == USART1) {
		RxBuffer();

		HAL_UART_Receive_IT(&huart1, &buf, 1);
	}
}

void Receive_DMA(void)
{
	HAL_UART_Receive_IT(&huart1, &buf, 1);
	//HAL_UART_Receive_DMA(&huart1, rxBuffer, DMA_BUF_SIZE);
}

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

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

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

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

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
