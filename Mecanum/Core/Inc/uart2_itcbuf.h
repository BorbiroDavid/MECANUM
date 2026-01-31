/*******************************************************************************
 * File Name:	UART2_itcbuf.h
 * Description:	Driver program for handling UART2 communication using
 *				circular RX and TX buffers with IT
 *******************************************************************************
 *
 * Copyright(c) 2020 SZTAKI
 *
 *******************************************************************************
 */

/*******************************************************************************
 * @file UART2_itcbuf.h
 * @author Alexandros Soumelidis
 * @date 28 June 2020
 * @brief Driver program for handling UART2 communication.
 *
 * UART2 RX and TX using circular buffers with IT.
 *                
 *******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART2_ITCBUF_H
#define __UART2_ITCBUF_H

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32f4xx_hal.h"

/* Global function prototypes ------------------------------------------------*/

/**
  * @brief  UART2 Rx Transfer completed callback.
  * @retval None
  */
void UART2_RxCpltCallback(void);

/**
  * @brief  UART2 Tx Transfer completed callback.
  * @retval None
  */
 void UART2_TxCpltCallback(void);

/**
  * @brief  Starting UART2 Receive Process.
  * @param  None
  * @retval None
  */
void StartUART2Communication(void);

/**
  * @brief  Testing whether there is any character in the UART2 Receive Buffer.
  * @param  None
  * @retval RESET - empty, SET - there is at least one character in the buffer
  */
uint32_t TestUART2RxData(void);

/**
  * @brief  Getting a character from the UART2 Receive Buffer.
  * @param  None
  * @retval character (null if there is no character in the buffer)
  */
uint8_t GetcUART2RxData(void);

/**
  * @brief  Putting a character to the UART2 Transmit Buffer.
  * @param  data	the character to be transferred
  * @retval SUCCESS / ERROR
  */
uint32_t PutcUART2TxData(uint8_t data);

/**
  * @brief  Putting a a message to the UART2 Transmit Buffer
  * 		without considering NUL characters.
  * @param  message	the string to be transferred
  * @param  size	size of the message buffer
  * @retval SUCCESS / ERROR
  */
uint32_t PutcccUART2TxData(uint8_t *message, int size);

/**
  * @brief  Putting a message (NUL-terminated string) to the UART2 Transmit Buffer.
  * @param  message	the string to be transferred
  * @param  maxsize	maximal size of the message buffer
  * @retval SUCCESS / ERROR
  */
uint32_t PutsUART2TxData(uint8_t *message, int maxsize);

#endif /* __UART2_ITCBUF_H */

/************************ (C) COPYRIGHT SZTAKI *****END OF FILE****************/

