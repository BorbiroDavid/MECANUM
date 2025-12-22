/*******************************************************************************
 * File Name:	UART2_itcbuf.c
 * Description:	Driver program for handling UART2 communication using
 *				circular RX and TX buffers with IT
 *******************************************************************************
 *
 * Copyright(c) 2020 SZTAKI
 *
 *******************************************************************************
 */

/*******************************************************************************
 * @file UART2_itcbuf.c
 * @author Alexandros Soumelidis
 * @date 28 June 2020
 * @brief Driver program for handling UART2 communication.
 *
 * UART2 RX and TX using circular buffers with IT.
 *
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32f4xx_hal.h"

/* Constants & Macros --------------------------------------------------------*/

#define	RXBUF2_SIZE		4096		// Receive Buffer Size
#define	TXBUF2_SIZE		4096		// Transmit Buffer Size

/* External variables --------------------------------------------------------*/

extern UART_HandleTypeDef huart2;

/* Private variables ---------------------------------------------------------*/

static uint8_t RxBuf[RXBUF2_SIZE];	// Receive Buffer
static uint32_t RxIpnt = 0;			// Receive Input Pointer
static uint32_t RxOpnt = 0;			// Receive Output Pointer
static uint32_t RxCnt = 0;			// Receive Counter

static uint8_t TxBuf[TXBUF2_SIZE];	// Transmit Buffer
static uint32_t TxIpnt = 0;			// Transmit Input Pointer
static uint32_t TxOpnt = 0;			// Transmit Output Pointer
static uint32_t TxCnt = 0;			// Transmit Counter

/* Local function prototypes -------------------------------------------------*/

/* IT Callback functions -----------------------------------------------------*/

/**
  * @brief  UART2 Rx Transfer completed callback.
  * @retval None
  */
void UART2_RxCpltCallback()
{
	// New data in the RxBuf + RxIpnt position
	RxCnt++;
	if (RxCnt < RXBUF2_SIZE)
	  {
		RxIpnt++;
		if (RxIpnt >= RXBUF2_SIZE) RxIpnt = 0;
		if (HAL_UART_Receive_IT(&huart2,RxBuf + RxIpnt,1) != HAL_OK)
		  {
		    Error_Handler();
		  }
	  }
	else
	  {
		// Receive process blocked
		SendErrorLEDSignal();
	  }
}

/**
  * @brief  UART2 Tx Transfer completed callback.
  * @retval None
  */
 void UART2_TxCpltCallback()
{
	// Transmission of 1 character is completed
	TxCnt--;
	TxOpnt++;
	if (TxOpnt >= TXBUF2_SIZE) TxOpnt = 0;
	if (TxCnt > 0)
	  {
		// Initiating new transmission: next character in the Buffer
		if (HAL_UART_Transmit_IT(&huart2,TxBuf + TxOpnt, 1) != HAL_OK)
		  {
		    Error_Handler();
		  }
	  }
}

/* Global functions ----------------------------------------------------------*/

/**
  * @brief  Starting UART2 Receive Process.
  * @param  None
  * @retval None
  */
void StartUART2Communication()
{
	RxIpnt = 0;
	RxOpnt = 0;
	RxCnt = 0;
	if (HAL_UART_Receive_IT(&huart2,RxBuf,1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	TxIpnt = 0;
	TxOpnt = 0;
	TxCnt = 0;
}

/**
  * @brief  Testing whether there is any character in the UART2 Receive Buffer.
  * @param  None
  * @retval RESET - empty, SET - there is at least one character in the buffer
  */
uint32_t TestUART2RxData()
{
	if (RxCnt > 0) return(SET);
	return(RESET);
}

/**
  * @brief  Getting a character from the UART2 Receive Buffer.
  * @param  None
  * @retval character (null if there is no character in the buffer)
  */
uint8_t GetcUART2RxData()
{
  uint8_t data = 0;
  uint32_t prim;

	if (RxCnt > 0)
	  {
	  	prim = __get_PRIMASK();
	    __disable_irq();
		data = RxBuf[RxOpnt];
		RxOpnt++;
		if (RxOpnt >= RXBUF2_SIZE) RxOpnt = 0;
		if (RxCnt >= RXBUF2_SIZE)
		  {
			// Restarting receive process
			RxIpnt++;
			if (RxIpnt >= RXBUF2_SIZE) RxIpnt = 0;
			if (HAL_UART_Receive_IT(&huart2,RxBuf + RxIpnt,1) != HAL_OK)
			  {
			    Error_Handler();
			  }
		  }
		RxCnt--;
	    if (!prim) __enable_irq();
	  }
	return(data);
}

/**
  * @brief  Putting a character to the UART2 Transmit Buffer.
  * @param  data	the character to be transferred
  * @retval SUCCESS / ERROR
  */
uint32_t PutcUART2TxData(uint8_t data)
{
  uint32_t prim;

	if (TxCnt < TXBUF2_SIZE)
	  {
	  	prim = __get_PRIMASK();
	    __disable_irq();
		TxBuf[TxIpnt] = data;
		TxIpnt++;
		if (TxIpnt >= TXBUF2_SIZE) TxIpnt = 0;
		TxCnt++;
		if (TxCnt == 1)
		  {
			// Starting Transmit process
			if (HAL_UART_Transmit_IT(&huart2,TxBuf + TxOpnt,1) != HAL_OK)
			  {
			    Error_Handler();
			  }
		  }
		if (!prim) __enable_irq();
		return(SUCCESS);
	  }
	else
	  {
		// TX Buffer full: character is discarded
		SendErrorLEDSignal();
		return(ERROR);
	  }
}

/**
  * @brief  Putting a a message to the UART2 Transmit Buffer
  * 		without considering NUL characters.
  * @param  message	the string to be transferred
  * @param  size	size of the message buffer
  * @retval SUCCESS / ERROR
  */
uint32_t PutcccUART2TxData(uint8_t *message, int size)
{
	uint32_t index = 0, status = SUCCESS;

	while (index < size)
	  {
		status = PutcUART2TxData(message[index]);
		if (status != SUCCESS) break;
		index++;
	  }
	return(status);
}

/**
  * @brief  Putting a message (NUL-terminated string) to the UART2 Transmit Buffer.
  * @param  message	the string to be transferred
  * @param  maxsize	maximal size of the message buffer
  * @retval SUCCESS / ERROR
  */
uint32_t PutsUART2TxData(uint8_t *message, int maxsize)
{
	uint32_t index = 0, status = SUCCESS;

	while (index < maxsize && message[index] != NUL_C)
	  {
		status = PutcUART2TxData(message[index]);
		if (status != SUCCESS) break;
		index++;
	  }
	return(status);
}

 /************************ (C) COPYRIGHT SZTAKI *****END OF FILE***************/
