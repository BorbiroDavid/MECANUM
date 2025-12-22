/*******************************************************************************
 * File Name:	adc1_itinj.c
 * Description:	Driver program for handling multiple channels (max. 4)
 * 				on ADC1 by IT.
 *******************************************************************************
 *
 * Copyright(c) 2021 SZTAKI
 *
 *******************************************************************************
 */

/*******************************************************************************
 * @file adc1_itinj.c
 * @author Alexandros Soumelidis
 * @date 28 June 2020
 * @brief Driver program for handling multiple (max.4) channels on ADC1 by IT.
 *
 * Settings: Scanned Injected Conversion on multiple analog channels
 * Trigger:  Timer 4 Update Event
 *                
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc1_itinj.h"

/* Constants & Macros --------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

extern ADC_HandleTypeDef hadc1;

/* Private variables ---------------------------------------------------------*/

volatile uint32_t adcCh1Acc = 0;		// ADC Channel 1 accumulator
volatile uint32_t adcCh1Cnt = 0;		// ADC Channel 1 counter
volatile uint32_t adcCh2Acc = 0;		// ADC Channel 2 accumulator
volatile uint32_t adcCh2Cnt = 0;		// ADC Channel 2 counter
volatile uint32_t adcCh3Acc = 0;		// ADC Channel 3 accumulator
volatile uint32_t adcCh3Cnt = 0;		// ADC Channel 3 counter
//volatile uint32_t adcCh4Acc = 0;		// ADC Channel 4 accumulator
//volatile uint32_t adcCh4Cnt = 0;		// ADC Channel 4 counter


/* Local function prototypes -------------------------------------------------*/

/* Callback functions --------------------------------------------------------*/

/**
  * @brief  Injected conversion complete callback in non blocking mode
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADCEx_InjectedConvCpltCallback (ADC_HandleTypeDef* hadc)
{
	adcCh1Acc += HAL_ADCEx_InjectedGetValue(hadc,ADC_INJECTED_RANK_1);
	adcCh1Cnt++;
	adcCh2Acc += HAL_ADCEx_InjectedGetValue(hadc,ADC_INJECTED_RANK_2);
	adcCh2Cnt++;
	adcCh3Acc += HAL_ADCEx_InjectedGetValue(hadc,ADC_INJECTED_RANK_3);
	adcCh3Cnt++;
	//	adcCh4Acc += HAL_ADCEx_InjectedGetValue(hadc,ADC_INJECTED_RANK_4);
	//	adcCh4Cnt++;
}

/* Global functions ----------------------------------------------------------*/

/**
  * @brief  Initializing / starting ADC Channels
  * @param  none
  * @retval none
  */

void InitADCmodule ()
{
	adcCh1Acc = 0;
	adcCh1Cnt = 0;
	adcCh2Acc = 0;
	adcCh2Cnt = 0;
	adcCh3Acc = 0;
	adcCh3Cnt = 0;
//	adcCh4Acc = 0;
//  adcCh4Cnt = 0;
   HAL_ADCEx_InjectedStart_IT(&hadc1);
}

/**
  * @brief  Initializing / starting ADC Channels
  * @param  none
  * @retval none
  */

void StartADCmeasurement ()
{
   HAL_ADCEx_InjectedStart_IT(&hadc1);
}

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

int CheckADCChannelForDataAvailable(uint8_t ch)
{
  int status = 0;

	switch (ch)
	  {
		case ADCH_R1:
			if (adcCh1Cnt > 0) status = 1;
			break;
		case ADCH_R2:
			if (adcCh2Cnt > 0) status = 1;
			break;
		case ADCH_R3:
			if (adcCh3Cnt > 0) status = 1;
			break;
/*		case ADCH_R4:
			if (adcCh4Cnt > 0) status = 1;
			break;
*/
		default:;
	  }
	return(status);
}

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

int GetADCChannelData(uint8_t ch, uint16_t *value)
{
  int status = 0;
  float n;
  uint32_t prim;

	switch (ch)
	  {
		case ADCH_R1:
			if (adcCh1Cnt > 0)
			  {
				// Disabling Global Interrupt
				prim = __get_PRIMASK();
			    __disable_irq();
				n = (float)adcCh1Cnt / 8.0;
				*value = (uint16_t)((float)adcCh1Acc / n);
				adcCh1Acc = 0;
				adcCh1Cnt = 0;
				status = 1;
				// Enabling Global Interrupt
				if (!prim) __enable_irq();
			  }
			break;
		case ADCH_R2:
			if (adcCh2Cnt > 0)
			  {
				// Disabling Global Interrupt
				prim = __get_PRIMASK();
			    __disable_irq();
				n = (float)adcCh2Cnt / 8.0;
				*value = (uint16_t)((float)adcCh2Acc / n);
				adcCh2Acc = 0;
				adcCh2Cnt = 0;
				status = 1;
				// Enabling Global Interrupt
				if (!prim) __enable_irq();
			  }
			break;
		case ADCH_R3:
			if (adcCh3Cnt > 0)
			  {
				// Disabling Global Interrupt
				prim = __get_PRIMASK();
			    __disable_irq();
				if (adcCh3Cnt > 1)
					*value = (uint16_t)(adcCh3Acc / adcCh3Cnt);
				else
					*value = (uint16_t)adcCh3Acc;
				adcCh3Acc = 0;
				adcCh3Cnt = 0;
				status = 1;
				// Enabling Global Interrupt
				if (!prim) __enable_irq();
			  }
			break;
/*		case ADCH_R4:
			if (adcCh4Cnt > 0)
	  	  	  {
				// Disabling Global Interrupt
				prim = __get_PRIMASK();
	    		__disable_irq();
				if (adcCh4Cnt > 1)
 	 	 	 		*value = (uint16_t)(adcCh4Acc / adcCh4Cnt);
				else
					*value = (uint16_t)adcCh4Acc;
				adcCh4Acc = 0;
				adcCh4Cnt = 0;
				status = 1;
				// Enabling Global Interrupt
				if (!prim) __enable_irq();
	  	  	  }
			break;
*/
		default:;
	  }
	return(status);
}

/************************ (C) COPYRIGHT SZTAKI *****END OF FILE****************/
