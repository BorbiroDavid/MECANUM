/*******************************************************************************
 * File Name:	adc1_itinj.h
 * Description:	Driver header for handling multiple channels on ADC1 by IT.
 *******************************************************************************
 *
 * Copyright(c) 2021 MTA SZTAKI
 *
 *******************************************************************************
 */

/*******************************************************************************
 * @file adc1_itinj.h
 * @author Alexandros Soumelidis
 * @date 7 Sep 2021
 * @brief Driver header for handling multiple channels on ADC1 by IT.
 *
 * Settings: Scanned Injected Conversion on multiple analog channels
 * Trigger:  Timer 4 Update Event
 *
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32f4xx_hal.h"

/* Constants & Macros --------------------------------------------------------*/

#define	ADCH_R1			1			// ADC Channel Rank 1
#define	ADCH_R2			2			// ADC Channel Rank 2
#define	ADCH_R3			3			// ADC Channel Rank 3
//#define	ADCH_R4			4			// ADC Channel Rank 3

/* Global function prototypes ------------------------------------------------*/

/**
  * @brief  Initializing / starting ADC Channels
  * @param  none
  * @retval none
  */

void InitADCmodule(void);

/**
  * @brief  Initializing / starting ADC Channels
  * @param  none
  * @retval none
  */

void StartADCmeasurement(void);

/**
  * @brief  Checking ADC Channel state
  *
  *	Checking whether there is any measurement available
  *	on the selected channel.
  *
  * @param  ch - channel index
  *
  * @retval 1 if measurement is available, 0 otherwise
  */

int CheckADCChannelForDataAvailable(uint8_t ch);

/**
  * @brief  Reading measurement data associated with an ADC Channel.
  *
  *	Reading measurement value associated with the ADC Channel
  * if it is available, and the measured value is returned.
  *
  * @param  ch - channel index
  * @param  value [output by ref] - measurement value if ready,
  * 								unchanged otherwise
  *
  * @retval 1 if measurement is available, 0 otherwise
  */

int GetADCChannelData(uint8_t ch, uint16_t *value);

/************************ (C) COPYRIGHT MTA SZTAKI *****END OF FILE************/
