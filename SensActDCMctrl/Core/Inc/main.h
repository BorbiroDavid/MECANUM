/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define	NUL_C				(0x00)			// Character constant
#define	CR_C				(0x0D)			// Character constant
#define	LF_C				(0x0A)			// Character constant
#define	SP_C				(0x20)			// Character constant
#define	LIFE_C				(0x07)			// Character constant

#define	NUL_PTR				((void *)0)		// NUL pointer constant

typedef enum _STATE							// State specifier
	{ OFF = 0,								//		 OFF State
	  ON									//		 ON State
	} STATE;

/*-----	Error Codes ----------------------------------------------------------*/

#define ERR_MCOVR			0x8000			// Main Cycle Overflow
#define ERR_HAL				0x4000			// HAL Error
#define ERR_UART2			0x0001			// UART2 Error
#define ERR_ICMD			0x0002			// Command Error
#define ERR_DNLD			0x0004			// Download Error
#define ERR_SPI1			0x0008			// SPI Error

/*-----	Error Info Codes: UART -----------------------------------------------*/

#define ERI_UART_HAL		0x0001			// UART HAL Error
#define ERI_UART_TRE		0x0002			// UART Transmission Error
#define ERI_UART_RXO		0x0004			// UART Receive Overflow
#define ERI_UART_TXO		0x0008			// UART Receive Overflow

/*-----	Error Info Codes: Input Command Reception ----------------------------*/

#define ERI_ICMD_UNN		0x0001			// Unknown Command
#define ERI_ICMD_ILL		0x0002			// Error Info Code: Illegal Command
#define ERI_ICMD_ARN		0x0004			// Error Info Code: Arguments Number
#define ERI_ICMD_BOV		0x0008			// Error Info Code: Buffer Overflow
#define ERI_ICMD_ILT		0x0010			// Error Info Code: Illegal Item

	/*-----	Error Info Codes: Download Signal --------------------------------*/

#define ERI_DNLD_SIZE		0x0001			// Error Info Code: Illegal Item

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/**
  * @brief  Sending Error Signal only to Red LED.
  *
  * @param  noine
  *
  * @retval None
  */
void SendErrorLEDSignal(void);

/**
  * @brief  Sending Error Signal to Red LED and
  * 		to the Config Interface.
  *
  * @param  sigid	 signal (Red LED) identifier
  * @param  info	 auxiliary information
  *
  * @retval None
  */
void SendErrorSignal(uint16_t sigid, uint32_t info);

/**
  * @brief	Sending a Message Line to VCP.
  * @param  message		a NUL terminated string
  * @retval None
  */
void SendMessageLine (char *message);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define YEL4_LED_Pin GPIO_PIN_13
#define YEL4_LED_GPIO_Port GPIOC
#define RPMS_NSS_Pin GPIO_PIN_0
#define RPMS_NSS_GPIO_Port GPIOC
#define DCMC_EN_B_Pin GPIO_PIN_1
#define DCMC_EN_B_GPIO_Port GPIOC
#define ANGS_MISO_Pin GPIO_PIN_2
#define ANGS_MISO_GPIO_Port GPIOC
#define ANGS_MOSI_Pin GPIO_PIN_3
#define ANGS_MOSI_GPIO_Port GPIOC
#define DCMC_IN1_B_Pin GPIO_PIN_0
#define DCMC_IN1_B_GPIO_Port GPIOA
#define DCMC_IN2_B_Pin GPIO_PIN_1
#define DCMC_IN2_B_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_3
#define VCP_RX_GPIO_Port GPIOA
#define RPSM_QB_Pin GPIO_PIN_4
#define RPSM_QB_GPIO_Port GPIOA
#define ANGS_NSS_Pin GPIO_PIN_6
#define ANGS_NSS_GPIO_Port GPIOA
#define PSENSE_AIN_Pin GPIO_PIN_7
#define PSENSE_AIN_GPIO_Port GPIOA
#define VPOT1_AIN_Pin GPIO_PIN_4
#define VPOT1_AIN_GPIO_Port GPIOC
#define VPOT2_AIN_Pin GPIO_PIN_5
#define VPOT2_AIN_GPIO_Port GPIOC
#define PS_LED_R_Pin GPIO_PIN_0
#define PS_LED_R_GPIO_Port GPIOB
#define YEL1_LED_Pin GPIO_PIN_1
#define YEL1_LED_GPIO_Port GPIOB
#define YEL2_LED_Pin GPIO_PIN_2
#define YEL2_LED_GPIO_Port GPIOB
#define ASENS_SCK_Pin GPIO_PIN_10
#define ASENS_SCK_GPIO_Port GPIOB
#define USR_SW3_Pin GPIO_PIN_12
#define USR_SW3_GPIO_Port GPIOB
#define USR_SW3_EXTI_IRQn EXTI15_10_IRQn
#define USR_SW6_Pin GPIO_PIN_13
#define USR_SW6_GPIO_Port GPIOB
#define USR_SW6_EXTI_IRQn EXTI15_10_IRQn
#define USR_SW5_Pin GPIO_PIN_14
#define USR_SW5_GPIO_Port GPIOB
#define USR_SW5_EXTI_IRQn EXTI15_10_IRQn
#define USR_SW4_Pin GPIO_PIN_15
#define USR_SW4_GPIO_Port GPIOB
#define USR_SW4_EXTI_IRQn EXTI15_10_IRQn
#define RED_LED_Pin GPIO_PIN_6
#define RED_LED_GPIO_Port GPIOC
#define GREEN_LED_Pin GPIO_PIN_7
#define GREEN_LED_GPIO_Port GPIOC
#define YEL7_LED_Pin GPIO_PIN_8
#define YEL7_LED_GPIO_Port GPIOC
#define USR_SW1_Pin GPIO_PIN_9
#define USR_SW1_GPIO_Port GPIOC
#define USR_SW1_EXTI_IRQn EXTI9_5_IRQn
#define YEL5_LED_Pin GPIO_PIN_8
#define YEL5_LED_GPIO_Port GPIOA
#define YEL6_LED_Pin GPIO_PIN_9
#define YEL6_LED_GPIO_Port GPIOA
#define DCMC_EN_A_Pin GPIO_PIN_10
#define DCMC_EN_A_GPIO_Port GPIOA
#define USR_SW2_Pin GPIO_PIN_11
#define USR_SW2_GPIO_Port GPIOA
#define USR_SW2_EXTI_IRQn EXTI15_10_IRQn
#define YEL3_LED_Pin GPIO_PIN_12
#define YEL3_LED_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define RPSM_QA_Pin GPIO_PIN_15
#define RPSM_QA_GPIO_Port GPIOA
#define PS_LED_G_Pin GPIO_PIN_3
#define PS_LED_G_GPIO_Port GPIOB
#define DCMC_IN1_A_Pin GPIO_PIN_4
#define DCMC_IN1_A_GPIO_Port GPIOB
#define DCMC_IN2_A_Pin GPIO_PIN_5
#define DCMC_IN2_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
