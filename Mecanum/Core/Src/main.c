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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "uart2_itcbuf.h"
#include "AS5047U.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	PROGRAM_ID		"Mechanum_project" // Program Version ID

#define	LEDSIGN_PER		200				// LED Signal Duration (ms)
#define	SWBLOCK_PER		50				// Switch Block Period (SysTick-s nbr.)
#define	CTRLOOP_PER		10				// Control Loop Period (x TIM10 T)
#define	STATTRF_PER		50				// State Transfer Period (x TIM10 T)
#define	VBAT_SMP_PER	2				// Vbat Sampling Period (secTick nbr.)
#define samplePeriod    50

#define SWB_STATE		0x0001			// Bit Mask for Switch Blue State
#define SW1_STATE		0x0002			// Bit Mask for Switch 1 State
#define SW2_STATE		0x0004			// Bit Mask for Switch 2 State
#define SW3_STATE		0x0008			// Bit Mask for Switch 3 State
#define SW4_STATE		0x0010			// Bit Mask for Switch 4 State
#define SW5_STATE		0x0020			// Bit Mask for Switch 5 State
#define SW6_STATE		0x0040			// Bit Mask for Switch 6 State

#define LEDY1_STATE		0x0001			// Bit Mask for LED YEL0 State
#define LEDY2_STATE		0x0002			// Bit Mask for LED YEL1 State
#define LEDY3_STATE		0x0004			// Bit Mask for LED YEL2 State
#define LEDY4_STATE		0x0008			// Bit Mask for LED YEL3 State
#define LEDY5_STATE		0x0010			// Bit Mask for LED YEL4 State
#define LEDY6_STATE		0x0020			// Bit Mask for LED YEL5 State
#define LEDY7_STATE		0x0040			// Bit Mask for LED YEL6 State
#define LEDR_STATE		0x0080			// Bit Mask for LED RED State
#define LEDG_STATE		0x0100			// Bit Mask for LED GREEN State

#define	ADCH_VBAT		ADCH_R3			// ADC Channel for Battery Voltage

#define	MPWMPER_MAX		3000			// Maximal value for Motor PWM TOP value
#define	MPWMPER_MIN		2048			// Minimal value for Motor PWM TOP value
#define	MPWMRED_FACT	8				// Motor PWM reduction factor
#define	MPWMPER_CTRF	(30.69 / 43.2)	// Motor PWM Period control factor

//#define	DCMCTST_IDLE	0x00			// Motor Ctrl State: Idle
#define	DCMCTST_IGN		0x01			// Motor Ctrl State: Ignition bit mask
//#define	DCMCTST_BRAKE	0x02			// Motor Ctrl State: Brake bit mask

#define	LOCAL_MODE		0x00			// Local Mode identifier (default)
#define	REMOTE_MODE		0x01			// Remote Mode  bit mask
#define	AUTOTEST_MODE	0x04			// Automatic Test Sequence Mode bit mask
#define	AUTOTEST_TSMASK	0x8000			// Tstamp Test Mode bit mask

#define	TSTAMP_MAX		32768			// Upper limit for Time Stamp

#define	BATST_OK		16.8			// Battery State Ok
#define	BATST_DISCH		13.6			// Battery State Discharged
#define	BATST_DDISCH	12.0			// Battery State Deep Discharged

#define	ACTVAL_MAX		16368			// Maximal Actuator Value
#define	ACTVAL_MIN		-16368			// Minimal Actuator Value

#define	MCTRL_DIRECT	0				// Motor Control Type: Direct (Open Loop)
#define	MCTRL_PROP		1				// Motor Control Type: Proportional
#define	MCTRL_PI		2				// Motor Control Type: PI
#define	MSERVO_PROP		3				// Motor Servo Control Type: Proportional
#define	MSERVO_PIPROP	4				// Motor Servo Control Type: Proportional with PI RPM
#define MSERVO_LQG		5				// Motor Servo Control Type: LQG Optimal Control

//#define MC_RPM_PROP_GAIN	2.5			// Motor RPM Control Proportional Gain
#define MC_RPM_PI_GAIN		10.0		// Motor RPM Control PI Gain
#define MC_RPM_PI_TI		16.0		// Motor RPM Control PI Integration Time
//#define MC_SRV_PROP_GAIN	10.0		// Motor Servo Control Proportional Gain
//#define MC_SRV_PIRPM_GAIN	10.0		// Motor Servo Control PIRPM Gain
//#define MC_SRV_PIRPM_TI		16.0		// Motor Servo Control PIRPM Integration Time

#define WINDUP_MAX		 32767.0		// Positive limit for PI anti-windup
#define WINDUP_MIN		-32767.0		// Negative limit for PI anti-windup

#define	RPS_FACTOR		13440000		// Scale factor between rps and period

#define FL 0
#define FR 1
#define RL 2
#define RR 3


#define	INMSG_SIZE		256				// Input Message Size (chars' nbr.)

#define	TESTSIGN_SIZE	32768			// Test Signal size (max. 32768)
#define	TESTDNLD_BLRATE	100				// Test Signal Blink Rate


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

// Soma

volatile uint32_t rpsmCapt1, rpsmCapt2, rpsmCapt3, rpsmCapt4 = 0;			// Motor Captured value of RPS
volatile int32_t rpsmVal1, rpsmVal2, rpsmVal3, rpsmVal4 = 0;				// Motor RPS measured value
volatile int32_t nperAcc1, nperAcc2, nperAcc3, nperAcc4 = 0;				// Motor Period Number accumulator
volatile uint32_t nperCnt1, nperCnt2, nperCnt3, nperCnt4 = 0;				// Motor Period Number counter
volatile int16_t rpsNoPulse1, rpsNoPulse2, rpsNoPulse3, rpsNoPulse4 = 0;	// Motor Number of periods without pulses



// Barni

static uint16_t motorCtrlSt = 0;


static int16_t motorActVal1 = 0;
static int16_t motorActVal2 = 0;
static int16_t motorActVal3 = 0;
static int16_t motorActVal4 = 0;

static int16_t motorActValBk = 0x7FFF;
static int16_t motorActValBk1 = 0x7FFF;
static int16_t motorActValBk2 = 0x7FFF;
static int16_t motorActValBk3 = 0x7FFF;
static int16_t motorActValBk4 = 0x7FFF;

static int16_t motorActValPW1 = 0;
static int16_t motorActValPW2 = 0;
static int16_t motorActValPW3 = 0;
static int16_t motorActValPW4 = 0;

static uint16_t motorPWMtop = MPWMPER_MAX;
static uint16_t motorPWMtop1 = MPWMPER_MAX;
static uint16_t motorPWMtop2 = MPWMPER_MAX;

static uint16_t motorPWMtopBk = MPWMPER_MAX;
static uint16_t motorPWMtopBk1 = MPWMPER_MAX;
static uint16_t motorPWMtopBk2 = MPWMPER_MAX;


static uint16_t dcmCtrlSetp1 = 0;
volatile uint16_t motorSpeed1 = 0;

static uint16_t dcmCtrlSetp2 = 0;
volatile uint16_t motorSpeed2 = 0;

static uint16_t dcmCtrlSetp3 = 0;
volatile uint16_t motorSpeed3 = 0;

static uint16_t dcmCtrlSetp4 = 0;
volatile uint16_t motorSpeed4 = 0;

static float mcRPMPIGain = MC_RPM_PI_GAIN;
static float mcRPMPITi = MC_RPM_PI_TI;

static double xvalPI1 = 0.0;
static double yvalPI1 = 0.0;
static double xvalPI2 = 0.0;
static double yvalPI2 = 0.0;
static double xvalPI3 = 0.0;
static double yvalPI3 = 0.0;
static double xvalPI4 = 0.0;
static double yvalPI4 = 0.0;

//David

/* Constant strings ----------------------------------------------------------*/

const char programId[] = PROGRAM_ID; 		// Program Version ID string
const char horLine28[]   = "----------------------------";   // Horizontal line for communication

/* Private variables ----------------------------------------------------------*/
volatile uint16_t secCnt = 0;				// Second Counter for SysTick
volatile uint16_t secTick = 0;				// Second Tick signal
volatile uint16_t mloopTick = 0;			// Main Loop Tick signal
static uint16_t ctrlLoopCnt = 0;			// Control Loop Period counter
static uint16_t statTrfCnt = STATTRF_PER - 1;// State Transfer Period counter

static uint16_t lockDataTrf = OFF; 			// Data Transfer Lock On/Off

static uint8_t inmsgBuf2[INMSG_SIZE];		// UART2 Receive Command Buffer
static uint32_t inmsgPnt2 = 0;				// UART2 Receive Command Pointer

static uint8_t inmsgBuf6[INMSG_SIZE];		// UART6 Receive Command Buffer
static uint32_t inmsgPnt6 = 0;				// UART6 Receive Command Pointer

volatile uint8_t testMode = 0;				// Test Mode state - Local Mode is default
volatile int16_t testSignal[TESTSIGN_SIZE];	// Input Test Signal Data array
static uint16_t testSigInPnt = 0;			// Test Signal Input Pointer - size of the actual Signal
static uint16_t testSigOutPnt = 0;			// Test Signal Output Pointer
static uint16_t testSigReset = OFF;			// Test Signal Reset / Cleared flag
static uint16_t testStart = OFF;			// Test Start flag
static uint16_t testStop = OFF;

uint8_t counter = 0;
char tx_buffer[50];

//uint16_t N1 = 10, N2 = 0, N3 = 0, N4 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

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

	float npaval = 0.0;
	uint32_t prim;

	//PI variables
	uint16_t pulse1, pulse2, pulse3, pulse4;
	double xval1, xtmp1, yval1;
	double xval2, xtmp2, yval2;
	double xval3, xtmp3, yval3;
	double xval4, xtmp4, yval4;

	//int status;
	char message[33];
	memset(message, 0, sizeof(message));
	uint8_t rch;



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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_TIM10_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  //for (uint16_t k = 0; k < TESTSIGN_SIZE; k++) testSignal[k] = 0;

  //Signing in to VCP
	StartUART2Communication();
	SendMessageLine ((char *)horLine28);
	SendMessageLine ((char *)programId);
	SendMessageLine ((char *)horLine28);
	strcpy(message,"$I\r");
	PutsUART2TxData((uint8_t *)message,strlen(message));

/****** Initializing TIMERs ***********************************************/

	/* ????????????????????????????????????????
	HAL_TIM_Base_Start_IT(&htim10);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim4);
	??????????????????????????????  */

/******	Initializing Motor PWM ********************************************/

    // Left side
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    // Right side
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //----------------- TESZTELŐ SZÁMOLÓ CIKLUS ------------------------------
	  	/*sprintf(tx_buffer, "Szamlalo: %d\r\n", N1);
		PutsUART2TxData((uint8_t *)tx_buffer, sizeof(tx_buffer));

		counter++;

		if (counter > 10)
		{
			counter = 0;
		}

		HAL_Delay(1000);  // 1000 ms várakozás
		*/

	/*while (mloopTick == 0) continue;
	if (mloopTick > 1)
		SendErrorSignal(ERR_MCOVR,0);
	mloopTick = 0; */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /*** Motor Control ***********************************/

	  // Soma kezdet

	  // RPM Sampling

	  ctrlLoopCnt++;

	  if (ctrlLoopCnt >= samplePeriod)
	  {

		  ctrlLoopCnt = 0;
	      uint32_t n_cnt;
	      int32_t n_acc;
		// 1.
		if (nperCnt1 > 0)
		{
			prim = __get_PRIMASK();
			__disable_irq();
			n_cnt = nperCnt1;
			n_acc = nperAcc1;
			nperAcc1 = 0; nperCnt1 = 0;
			if (!prim) __enable_irq();

			npaval = (n_cnt > 1) ? (float)n_acc / (float)n_cnt : (float)n_acc;
			rpsmVal1 = (int32_t)(RPS_FACTOR / npaval);
			rpsNoPulse1 = 0;
		}
		else
		{
			if (rpsmVal1 != 0) {
				rpsNoPulse1++;
				if (rpsNoPulse1 > 2) rpsmVal1 /= 2;
			}
		}
		motorSpeed1 = (int16_t)rpsmVal1;

		// 2.
		if (nperCnt2 > 0)
		{
			prim = __get_PRIMASK();
			__disable_irq();
			n_cnt = nperCnt2;
			n_acc = nperAcc2;
			nperAcc2 = 0; nperCnt2 = 0;
			if (!prim) __enable_irq();

			npaval = (n_cnt > 1) ? (float)n_acc / (float)n_cnt : (float)n_acc;
			rpsmVal2 = (int32_t)(RPS_FACTOR / npaval);
			rpsNoPulse2 = 0;
		}
		else
		{
			if (rpsmVal2 != 0) {
				rpsNoPulse2++;
				if (rpsNoPulse2 > 2) rpsmVal2 /= 2;
			}
		}
		motorSpeed2 = (int16_t)rpsmVal2;

		// 3.
		if (nperCnt3 > 0)
		{
			prim = __get_PRIMASK();
			__disable_irq();
			n_cnt = nperCnt3;
			n_acc = nperAcc3;
			nperAcc3 = 0; nperCnt3 = 0;
			if (!prim) __enable_irq();

			npaval = (n_cnt > 1) ? (float)n_acc / (float)n_cnt : (float)n_acc;
			rpsmVal3 = (int32_t)(RPS_FACTOR / npaval);
			rpsNoPulse3 = 0;
		}
		else
		{
			if (rpsmVal3 != 0) {
				rpsNoPulse3++;
				if (rpsNoPulse3 > 2) rpsmVal3 /= 2;
			}
		}
		motorSpeed3 = (int16_t)rpsmVal3;

		// 4.
		if (nperCnt4 > 0)
		{
			prim = __get_PRIMASK();
			__disable_irq();
			n_cnt = nperCnt4;
			n_acc = nperAcc4;
			nperAcc4 = 0; nperCnt4 = 0;
			if (!prim) __enable_irq();

			npaval = (n_cnt > 1) ? (float)n_acc / (float)n_cnt : (float)n_acc;
			rpsmVal4 = (int32_t)(RPS_FACTOR / npaval);
			rpsNoPulse4 = 0;
		}
		else
		{
			if (rpsmVal4 != 0) {
				rpsNoPulse4++;
				if (rpsNoPulse4 > 2) rpsmVal4 /= 2;
			}
		}
		motorSpeed4 = (int16_t)rpsmVal4;

		// Soma vége


	  // Enable - Ignition ON - kell!!!!
	  if ((motorCtrlSt & DCMCTST_IGN) > 0)
	   { // Ignition ON
		 HAL_GPIO_WritePin(MWC_EN_RL_GPIO_Port,MWC_EN_RL_Pin,GPIO_PIN_SET);
		 HAL_GPIO_WritePin(MWCL_EN_FL_GPIO_Port,MWCL_EN_FL_Pin,GPIO_PIN_SET);
		 HAL_GPIO_WritePin(MWC_EN_RR_GPIO_Port,MWC_EN_RR_Pin,GPIO_PIN_SET);
		 HAL_GPIO_WritePin(MWC_EN_FR_GPIO_Port,MWC_EN_FR_Pin,GPIO_PIN_SET);

		 // MCTRL_PI
		 // MOTOR1
		 xval1 = (double)(dcmCtrlSetp1 - motorSpeed1);
		 xtmp1 = xval1 + xval1 / mcRPMPITi - xvalPI1;
		 yval1 = yvalPI1 + mcRPMPIGain * xtmp1;
		 if (yval1 > WINDUP_MAX)
			 yval1 = WINDUP_MAX;
		 else if (yval1 < WINDUP_MIN)
			 yval1 = WINDUP_MIN;
		 if (yval1 > (double)ACTVAL_MAX)
			 motorActVal1 = ACTVAL_MAX;
		 else if (yval1 < (double)ACTVAL_MIN)
			 motorActVal1 = ACTVAL_MIN;
		 else
			 motorActVal1 = (int16_t)yval1;
		 yvalPI1 = yval1;
		 xvalPI1 = xval1;

		 // MOTOR2
		 xval2 = (double)(dcmCtrlSetp2 - motorSpeed2);
		 xtmp2 = xval2 + xval2 / mcRPMPITi - xvalPI2;
		 yval2 = yvalPI2 + mcRPMPIGain * xtmp2;
		 if (yval2 > WINDUP_MAX)
			 yval2 = WINDUP_MAX;
		 else if (yval2 < WINDUP_MIN)
			 yval2 = WINDUP_MIN;
		 if (yval2 > (double)ACTVAL_MAX)
			 motorActVal2 = ACTVAL_MAX;
		 else if (yval2 < (double)ACTVAL_MIN)
			 motorActVal2 = ACTVAL_MIN;
		 else
			 motorActVal2 = (int16_t)yval2;
		 yvalPI2 = yval2;
		 xvalPI2 = xval2;

		 // MOTOR3
		 xval3 = (double)(dcmCtrlSetp3 - motorSpeed3);
		 xtmp3 = xval3 + xval3 / mcRPMPITi - xvalPI3;
		 yval3 = yvalPI3 + mcRPMPIGain * xtmp3;
		 if(yval3 > WINDUP_MAX)
			yval3 = WINDUP_MAX;
		 else if (yval3 < WINDUP_MIN)
			 yval3 = WINDUP_MIN;
		 if (yval3 > (double)ACTVAL_MAX)
			 motorActVal3 = ACTVAL_MAX;
		 else if (yval3 < (double)ACTVAL_MIN)
			 motorActVal3 = ACTVAL_MIN;
		 else
			 motorActVal3 = (int16_t)yval3;
		 yvalPI3 = yval3;
		 xvalPI3 = xval3;

		 // MOTOR4
		 xval4 = (double)(dcmCtrlSetp4 - motorSpeed4);
		 xtmp4 = xval4 + xval4 / mcRPMPITi - xvalPI4;
		 yval4 = yvalPI4 + mcRPMPIGain * xtmp4;
		 if(yval4 > WINDUP_MAX)
			yval4 = WINDUP_MAX;
		 else if (yval4 < WINDUP_MIN)
			 yval4 = WINDUP_MIN;
		 if (yval4 > (double)ACTVAL_MAX)
			 motorActVal4 = ACTVAL_MAX;
		 else if (yval4 < (double)ACTVAL_MIN)
			 motorActVal4 = ACTVAL_MIN;
		 else
			 motorActVal4 = (int16_t)yval4;
		 yvalPI4 = yval4;
		 xvalPI4 = xval4;


		 //Szétbontva 4 külön egységre
		 // MOTOR1
		 if (motorActVal1 != motorActValBk1)
		   {
			 motorActValPW1 = motorActVal1 / MPWMRED_FACT;
			 if (motorPWMtop1 != motorPWMtopBk1)
			   { // Action for PS Voltage correction
				 __HAL_TIM_SET_AUTORELOAD(&htim1, motorPWMtop1 - 1);
				 motorPWMtopBk1 = motorPWMtop1;
			   }
			 if (motorActValPW1 >= 0)
			   {
				 pulse1 = (uint16_t)motorActValPW1;
				 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse1);
				 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			   }
			 else
			   {
				 pulse1 = (uint16_t)(-motorActValPW1);
				 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
				 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse1);
			   }
			 motorActValBk1 = motorActVal1;
		   }

		 // MOTOR2
		 if (motorActVal2 != motorActValBk2)
		   {
			 motorActValPW2 = motorActVal2 / MPWMRED_FACT;
			 if (motorPWMtop1 != motorPWMtopBk1)
			   { // Action for PS Voltage correction
				 __HAL_TIM_SET_AUTORELOAD(&htim1, motorPWMtop1 - 1);
				 motorPWMtopBk1 = motorPWMtop1;
			   }
			 if (motorActValPW2 >= 0)
			   {
				 pulse2 = (uint16_t)motorActValPW2;
				 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse2);
				 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
			   }
			 else
			   {
				 pulse2 = (uint16_t)(-motorActValPW2);
				 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
				 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse2);
			   }
			 motorActValBk2 = motorActVal2;
		   }

		 // MOTOR3
		 if (motorActVal3 != motorActValBk3)
		   {
			 motorActValPW3 = motorActVal3 / MPWMRED_FACT;
			 if (motorPWMtop2 != motorPWMtopBk2)
			   { // Action for PS Voltage correction
				 __HAL_TIM_SET_AUTORELOAD(&htim4, motorPWMtop2 - 1);
				 motorPWMtopBk2 = motorPWMtop2;
			   }
			 if (motorActValPW3 >= 0)
			   {
				 pulse3 = (uint16_t)motorActValPW3;
				 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse3);
				 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
			   }
			 else
			   {
				 pulse3 = (uint16_t)(-motorActValPW3);
				 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
				 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pulse3);
			   }
			 motorActValBk3 = motorActVal3;
		   }

		 // MOTOR4
		 if (motorActVal4 != motorActValBk4)
		   {
			 motorActValPW4 = motorActVal4 / MPWMRED_FACT;
			 if (motorPWMtop2 != motorPWMtopBk2)
			   { // Action for PS Voltage correction
				 __HAL_TIM_SET_AUTORELOAD(&htim4, motorPWMtop2 - 1);
				 motorPWMtopBk2 = motorPWMtop2;
			   }
			 if (motorActValPW4 >= 0)
			   {
				 pulse4 = (uint16_t)motorActValPW4;
				 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse4);
				 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
			   }
			 else
			   {
				 pulse4 = (uint16_t)(-motorActValPW4);
				 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
				 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pulse4);
			   }
			 motorActValBk4 = motorActVal4;
		   }
	   }
	  else
	   { // Ignition OFF
		 HAL_GPIO_WritePin(MWC_EN_RL_GPIO_Port,MWC_EN_RL_Pin,GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(MWCL_EN_FL_GPIO_Port,MWCL_EN_FL_Pin,GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(MWC_EN_RR_GPIO_Port,MWC_EN_RR_Pin,GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(MWC_EN_FR_GPIO_Port,MWC_EN_FR_Pin,GPIO_PIN_RESET);
		 if ((motorActVal1 || motorActVal2 || motorActVal3 || motorActVal4) != 0 )
		   {
			 motorActVal1 = 0;
			 motorActVal2 = 0;
			 motorActVal3 = 0;
			 motorActVal4 = 0;
			 motorActValBk1 = 0x7FFF;
			 motorActValBk2 = 0x7FFF;
			 motorActValBk3 = 0x7FFF;
			 motorActValBk4 = 0x7FFF;
			 motorActValPW1 = 0;
			 motorActValPW2 = 0;
			 motorActValPW3 = 0;
			 motorActValPW4 = 0;
			 xvalPI1 = 0.0;
			 xvalPI2 = 0.0;
			 xvalPI3 = 0.0;
			 xvalPI4 = 0.0;
			 yvalPI1 = 0.0;
			 yvalPI2 = 0.0;
			 yvalPI3 = 0.0;
			 yvalPI4 = 0.0;
			 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
			 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

			 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
			 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
			 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
			 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
		   }
		 if (motorCtrlSt != 0)
		   {
			 motorCtrlSt = 0;
		   }
	   }

	  } // ctrlloopcnt if vége

	  // Green led test
	  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin,GPIO_PIN_SET);
	  HAL_Delay(1000);
	  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin,GPIO_PIN_RESET);
	  HAL_Delay(1000);

// Sending RPM to VCP
	if (lockDataTrf == OFF)
	  {
		sprintf(message,"$C%04X%04X%04X%04X\r",dcmCtrlSetp1,dcmCtrlSetp2,dcmCtrlSetp3,dcmCtrlSetp4);
		PutsUART2TxData((uint8_t *)message,strlen(message));
		HAL_Delay(1000);

		/*sprintf(message,"$CD%04X%04X%04X%04X%04X\r",
					*(uint16_t *)&dcmCtrlSetp,*(uint16_t *)&motorSpeed,
					*(uint16_t *)&motorExtAngle,*(uint16_t *)&motorActVal,
					remTstamp);

		PutsUART2TxData((uint8_t *)message,strlen(message));*/
	  }
	ctrlLoopCnt = 0;
	/****** Handling Test Start ************************************************/

	if (testStart == ON)
	  {
		strcpy(message,"$TS\r");
		PutsUART2TxData((uint8_t *)message,strlen(message));
		testStart = OFF;
	  }

	/****** Handling Test Stop *************************************************/

	if (testStop == ON)
	  {
		strcpy(message,"$TE\r");
		PutsUART2TxData((uint8_t *)message,strlen(message));
		testStop = OFF;
	  }


/******	Receiving Messages from VCP *******************************/

	while (TestUART2RxData() == SET)
	  {

		rch = GetcUART2RxData();
		if (inmsgPnt2 > 0)
		  {

			if (rch == CR_C)
			  {
				// A complete command is in the buffer
				inmsgBuf2[inmsgPnt2] = NUL_C;
				// Processing the received command
				switch (inmsgBuf2[1])
				  {
					case 'C':{
						if (inmsgPnt2 >= 18)
						{
							sscanf((char *)&inmsgBuf2[2], "%4hx%4hx%4hx%4hx", &dcmCtrlSetp1, &dcmCtrlSetp2, &dcmCtrlSetp3, &dcmCtrlSetp4);
						}
						else
						{
							SendErrorSignal(ERR_ICMD, ERI_ICMD_ILL);
						}

						inmsgPnt2 = 0;
						break;
					}

				    default:inmsgPnt2 = 0;
					break;
				  }
			  }
		    else
			{
			  // Character belonging to Command
			  inmsgBuf2[inmsgPnt2] = rch;
			  inmsgPnt2++;
			  if (inmsgPnt2 >= INMSG_SIZE)
			  {
				 // Invalid command: too long
				 inmsgPnt2 = 0;
				 SendErrorSignal(ERR_ICMD,ERI_ICMD_ILL);
			  }
			}
		}

		else
		{ // Waiting for Command Start character
			if (rch == '$')
			  {
				// Command Start character found
				inmsgBuf2[0] = rch;
				inmsgPnt2++;
			  }
			// else: done nothing - data is dropped
	    }
	 }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config()
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_7;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 2048-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim3.Init.Period = 2048-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim4.Init.Period = 2048-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 84-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 1000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IMU_RST_Pin|MWC_EN_RR_Pin|MWC_EN_RL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RPSPI_NSS_FL_Pin|RPSPI_NSS_FR_Pin|RPSPI_NSS_RL_Pin|RPSPI_NSS_RR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, YEL_LED_Pin|RED_LED_Pin|MWCL_EN_FL_Pin|MWC_EN_FR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_NSS_GPIO_Port, IMU_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BLUE_SW_Pin */
  GPIO_InitStruct.Pin = BLUE_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLUE_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 MWC_RPSB_RL_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|MWC_RPSB_RL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MWC_RPSB_FL_Pin */
  GPIO_InitStruct.Pin = MWC_RPSB_FL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MWC_RPSB_FL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_INT_Pin */
  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU_RST_Pin MWC_EN_RR_Pin MWC_EN_RL_Pin */
  GPIO_InitStruct.Pin = IMU_RST_Pin|MWC_EN_RR_Pin|MWC_EN_RL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MWC_RPSB_FR_Pin */
  GPIO_InitStruct.Pin = MWC_RPSB_FR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MWC_RPSB_FR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RPSPI_NSS_FL_Pin RPSPI_NSS_FR_Pin RPSPI_NSS_RL_Pin RPSPI_NSS_RR_Pin */
  GPIO_InitStruct.Pin = RPSPI_NSS_FL_Pin|RPSPI_NSS_FR_Pin|RPSPI_NSS_RL_Pin|RPSPI_NSS_RR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : YEL_LED_Pin RED_LED_Pin MWCL_EN_FL_Pin MWC_EN_FR_Pin */
  GPIO_InitStruct.Pin = YEL_LED_Pin|RED_LED_Pin|MWCL_EN_FL_Pin|MWC_EN_FR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_NSS_Pin */
  GPIO_InitStruct.Pin = IMU_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(IMU_NSS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Soma kezdet

// 1. IDŐZÍTŐ CALLBACK (1 ms-os ütemezés a TIM10-ből)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10)
    {
        mloopTick++;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint32_t captval;
    int32_t npincr;
    uint8_t rpsmdir;

    // --- TIM2: 1. és 2. Motor (FL és FR) ---
    if (htim->Instance == TIM2)
    {
        // 1. Motor (FL - Front Left) -> TIM2 Channel 1
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            captval = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);

            rpsmdir = (uint8_t)HAL_GPIO_ReadPin(MWC_RPSB_FL_GPIO_Port, MWC_RPSB_FL_Pin);

            if (captval < rpsmCapt1)
                npincr = (int32_t)(0xFFFFFFFF - rpsmCapt1 + captval + 1);
            else
                npincr = (int32_t)(captval - rpsmCapt1);

            if (rpsmdir == 0) nperAcc1 += npincr; else nperAcc1 -= npincr;
            nperCnt1++;
            rpsmCapt1 = captval;
        }
        // 2. Motor (FR - Front Right) -> TIM2 Channel 2
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        {
            captval = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);

            rpsmdir = (uint8_t)HAL_GPIO_ReadPin(MWC_RPSB_FR_GPIO_Port, MWC_RPSB_FR_Pin);

            if (captval < rpsmCapt2)
                npincr = (int32_t)(0xFFFFFFFF - rpsmCapt2 + captval + 1);
            else
                npincr = (int32_t)(captval - rpsmCapt2);

            if (rpsmdir == 0) nperAcc2 += npincr; else nperAcc2 -= npincr;
            nperCnt2++;
            rpsmCapt2 = captval;
        }
    }

    // --- TIM5: 3. és 4. Motor (RL és RR) ---
    else if (htim->Instance == TIM5)
    {
        // 3. Motor (RL - Rear Left) -> TIM5 Channel 1
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            captval = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);

            rpsmdir = (uint8_t)HAL_GPIO_ReadPin(MWC_RPSB_RL_GPIO_Port, MWC_RPSB_RL_Pin);

            if (captval < rpsmCapt3)
                npincr = (int32_t)(0xFFFFFFFF - rpsmCapt3 + captval + 1);
            else
                npincr = (int32_t)(captval - rpsmCapt3);

            if (rpsmdir == 0) nperAcc3 += npincr; else nperAcc3 -= npincr;
            nperCnt3++;
            rpsmCapt3 = captval;
        }
        // 4. Motor (RR - Rear Right) -> TIM5 Channel 2
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        {
            captval = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);

            rpsmdir = (uint8_t)HAL_GPIO_ReadPin(MWC_RPSB_RR_GPIO_Port, MWC_RPSB_RR_Pin);

            if (captval < rpsmCapt4)
                npincr = (int32_t)(0xFFFFFFFF - rpsmCapt4 + captval + 1);
            else
                npincr = (int32_t)(captval - rpsmCapt4);

            if (rpsmdir == 0) nperAcc4 += npincr; else nperAcc4 -= npincr;
            nperCnt4++;
            rpsmCapt4 = captval;
        }
    }
}

// Soma vége

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
	  UART2_RxCpltCallback();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
	  UART2_TxCpltCallback();
  }
}

void SendMessageLine (char *message)
{
  char outmsg[68];
    sprintf(outmsg,"$%s\r",message);
    PutsUART2TxData((uint8_t *)outmsg,strlen(outmsg));
    PutcUART2TxData('\0');
}

void SendErrorSignal(uint16_t sigid, uint32_t info)
{
	char message[32];
	sprintf(message,"$DE%04X,%08lX\r",sigid,info);
	PutsUART2TxData((uint8_t *)message,strlen(message));
}

void SendErrorLEDSignal()
{
	//HAL_GPIO_WritePin(GPIOC, RED_LED_Pin,GPIO_PIN_SET);
	//ledState |= LEDR_STATE;
	//ledRsign = LEDSIGN_PER;
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
#ifdef USE_FULL_ASSERT
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
