/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
void SendErrorLEDSignal(void);
void SendErrorSignal(uint16_t sigid, uint32_t info);
void SendMessageLine (char *message);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BLUE_SW_Pin GPIO_PIN_13
#define BLUE_SW_GPIO_Port GPIOC
#define BLUE_SW_EXTI_IRQn EXTI15_10_IRQn
#define MWC_RPSB_RL_Pin GPIO_PIN_1
#define MWC_RPSB_RL_GPIO_Port GPIOC
#define RPSPI_MISO_Pin GPIO_PIN_2
#define RPSPI_MISO_GPIO_Port GPIOC
#define RPSPI_MOSI_Pin GPIO_PIN_3
#define RPSPI_MOSI_GPIO_Port GPIOC
#define MWC_RPSA_FL_Pin GPIO_PIN_0
#define MWC_RPSA_FL_GPIO_Port GPIOA
#define MWC_RPSA_FR_Pin GPIO_PIN_1
#define MWC_RPSA_FR_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_3
#define VCP_RX_GPIO_Port GPIOA
#define MWC_RPSB_FL_Pin GPIO_PIN_4
#define MWC_RPSB_FL_GPIO_Port GPIOA
#define GREEN_LED_Pin GPIO_PIN_5
#define GREEN_LED_GPIO_Port GPIOA
#define IMU_TC1_Pin GPIO_PIN_6
#define IMU_TC1_GPIO_Port GPIOA
#define PSENS_AIN_Pin GPIO_PIN_7
#define PSENS_AIN_GPIO_Port GPIOA
#define IMU_INT_Pin GPIO_PIN_4
#define IMU_INT_GPIO_Port GPIOC
#define IMU_INT_EXTI_IRQn EXTI4_IRQn
#define IMU_RST_Pin GPIO_PIN_5
#define IMU_RST_GPIO_Port GPIOC
#define MWC_RPSB_FR_Pin GPIO_PIN_0
#define MWC_RPSB_FR_GPIO_Port GPIOB
#define RPSPI_NSS_FL_Pin GPIO_PIN_1
#define RPSPI_NSS_FL_GPIO_Port GPIOB
#define YEL_LED_Pin GPIO_PIN_2
#define YEL_LED_GPIO_Port GPIOB
#define RPSPI_SCK_Pin GPIO_PIN_10
#define RPSPI_SCK_GPIO_Port GPIOB
#define RED_LED_Pin GPIO_PIN_12
#define RED_LED_GPIO_Port GPIOB
#define RPSPI_NSS_FR_Pin GPIO_PIN_13
#define RPSPI_NSS_FR_GPIO_Port GPIOB
#define RPSPI_NSS_RL_Pin GPIO_PIN_14
#define RPSPI_NSS_RL_GPIO_Port GPIOB
#define RPSPI_NSS_RR_Pin GPIO_PIN_15
#define RPSPI_NSS_RR_GPIO_Port GPIOB
#define RFCOM_TX_Pin GPIO_PIN_6
#define RFCOM_TX_GPIO_Port GPIOC
#define IMU_TC2_Pin GPIO_PIN_7
#define IMU_TC2_GPIO_Port GPIOC
#define MWC_EN_RR_Pin GPIO_PIN_8
#define MWC_EN_RR_GPIO_Port GPIOC
#define MWC_EN_RL_Pin GPIO_PIN_9
#define MWC_EN_RL_GPIO_Port GPIOC
#define MWC_IN1_FL_Pin GPIO_PIN_8
#define MWC_IN1_FL_GPIO_Port GPIOA
#define MWC_IN2_FL_Pin GPIO_PIN_9
#define MWC_IN2_FL_GPIO_Port GPIOA
#define MWC_IN1_FR_Pin GPIO_PIN_10
#define MWC_IN1_FR_GPIO_Port GPIOA
#define MWC_IN1_FRA11_Pin GPIO_PIN_11
#define MWC_IN1_FRA11_GPIO_Port GPIOA
#define RFCOM_RX_Pin GPIO_PIN_12
#define RFCOM_RX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define MWC_RPSA_RL_Pin GPIO_PIN_15
#define MWC_RPSA_RL_GPIO_Port GPIOA
#define IMU_SCK_Pin GPIO_PIN_10
#define IMU_SCK_GPIO_Port GPIOC
#define IMU_MISO_Pin GPIO_PIN_11
#define IMU_MISO_GPIO_Port GPIOC
#define IMU_MOSI_Pin GPIO_PIN_12
#define IMU_MOSI_GPIO_Port GPIOC
#define IMU_NSS_Pin GPIO_PIN_2
#define IMU_NSS_GPIO_Port GPIOD
#define MWC_RPSA_RR_Pin GPIO_PIN_3
#define MWC_RPSA_RR_GPIO_Port GPIOB
#define MWCL_EN_FL_Pin GPIO_PIN_4
#define MWCL_EN_FL_GPIO_Port GPIOB
#define MWC_EN_FR_Pin GPIO_PIN_5
#define MWC_EN_FR_GPIO_Port GPIOB
#define MWC_IN1_RL_Pin GPIO_PIN_6
#define MWC_IN1_RL_GPIO_Port GPIOB
#define MWC_IN2_RL_Pin GPIO_PIN_7
#define MWC_IN2_RL_GPIO_Port GPIOB
#define MWC_IN1_RR_Pin GPIO_PIN_8
#define MWC_IN1_RR_GPIO_Port GPIOB
#define MWC_IN2_RR_Pin GPIO_PIN_9
#define MWC_IN2_RR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
