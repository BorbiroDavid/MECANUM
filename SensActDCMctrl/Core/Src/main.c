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

#include <math.h>
#include "uart2_itcbuf.h"
#include "adc1_itinj.h"
#include "AS5047U.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define	PROGRAM_ID		"SensAct-DCMctrl" // Program Version ID

#define	LEDSIGN_PER		200				// LED Signal Duration (ms)
#define	SWBLOCK_PER		50				// Switch Block Period (SysTick-s nbr.)
#define	CTRLOOP_PER		10				// Control Loop Period (x TIM10 T)
#define	STATTRF_PER		50				// State Transfer Period (x TIM10 T)
#define	VBAT_SMP_PER	2				// Vbat Sampling Period (secTick nbr.)

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

#define	ADCH_POT1		ADCH_R1			// ADC Channel for Potentiometer 1
#define	ADCH_POT2		ADCH_R2			// ADC Channel for Potentiometer 2
#define	ADCH_VBAT		ADCH_R3			// ADC Channel for Battery Voltage

#define	MPWMPER_MAX		3000			// Maximal value for Motor PWM TOP value
#define	MPWMPER_MIN		2048			// Minimal value for Motor PWM TOP value
#define	MPWMRED_FACT	8				// Motor PWM reduction factor
#define	MPWMPER_CTRF	(30.69 / 43.2)	// Motor PWM Period control factor

#define	POTSEL_ST		0x01			// Potmeter Select State bit mask
#define	POTSEL_CHG		0x02			// Potmeter Select Changed mask

#define	DCMCTST_IDLE	0x00			// Motor Ctrl State: Idle
#define	DCMCTST_IGN		0x01			// Motor Ctrl State: Ignition bit mask
#define	DCMCTST_BRAKE	0x02			// Motor Ctrl State: Brake bit mask

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

#define MC_RPM_PROP_GAIN	2.5			// Motor RPM Control Proportional Gain
#define MC_RPM_PI_GAIN		10.0		// Motor RPM Control PI Gain
#define MC_RPM_PI_TI		16.0		// Motor RPM Control PI Integration Time
#define MC_SRV_PROP_GAIN	10.0		// Motor Servo Control Proportional Gain
#define MC_SRV_PIRPM_GAIN	10.0		// Motor Servo Control PIRPM Gain
#define MC_SRV_PIRPM_TI		16.0		// Motor Servo Control PIRPM Integration Time

#define WINDUP_MAX		 32767.0		// Positive limit for PI anti-windup
#define WINDUP_MIN		-32767.0		// Negative limit for PI anti-windup

#define	RPS_FACTOR		13440000		// Scale factor between rps and period

#define FL 0
#define FR 1
#define RL 2
#define RR 3

#define SANGLE_MAX		8192			// Positive limit for Servo Angle
#define SANGLE_MIN		-8192			// Negative limit for Servo Angle

#define	INMSG_SIZE		256				// Input Message Size (chars' nbr.)

#define	TESTSIGN_SIZE	32768			// Test Signal size (max. 32768)
#define	TESTDNLD_BLRATE	100				// Test Signal Blink Rate


#ifndef RPSPI_NSS_FL_Pin
  #define RPSPI_NSS_FL_Pin GPIO_PIN_1
#endif
#ifndef RPSPI_NSS_FR_Pin
  #define RPSPI_NSS_FR_Pin GPIO_PIN_13
#endif
#ifndef RPSPI_NSS_RL_Pin
  #define RPSPI_NSS_RL_Pin GPIO_PIN_14
#endif
#ifndef RPSPI_NSS_RR_Pin
  #define RPSPI_NSS_RR_Pin GPIO_PIN_15
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Constant strings ----------------------------------------------------------*/

const char programId[] = PROGRAM_ID; 		// Program Version ID string
const char horLine28[]   = "----------------------------";
											// Horizontal line
/* Private Variables -------------------------------------------------------- */

volatile uint16_t secCnt = 0;				// Second Counter for SysTick
volatile uint16_t secTick = 0;				// Second Tick signal
volatile uint16_t mloopTick = 0;			// Main Loop Tick signal
static uint16_t ctrlLoopCnt = 0;			// Control Loop Period counter
static uint16_t statTrfCnt = STATTRF_PER - 1;// State Transfer Period counter

volatile uint16_t ledY0sign = 0;			// Yellow 1 LED Signal
volatile uint16_t ledY1sign = 0;			// Yellow 2 LED Signal
volatile uint16_t ledY2sign = 0;			// Yellow 3 LED Signal
volatile uint16_t ledY3sign = 0;			// Yellow 4 LED Signal
volatile uint16_t ledY4sign = 0;			// Yellow 5 LED Signal
volatile uint16_t ledY5sign = 0;			// Yellow 6 LED Signal
volatile uint16_t ledY6sign = 0;			// Yellow 7 LED Signal
volatile uint16_t ledRsign = 0;				// Red LED Signal
volatile uint16_t ledGsign = 0;				// Green LED Signal

volatile uint8_t usrSw1Block = 0;			// Blocking Period Cnt for User Sw1
volatile uint8_t usrSw2Block = 0;			// Blocking Period Cnt for User Sw2
volatile uint8_t usrSw3Block = 0;			// Blocking Period Cnt for User Sw3
volatile uint8_t usrSw4Block = 0;			// Blocking Period Cnt for User Sw4
volatile uint8_t usrSw5Block = 0;			// Blocking Period Cnt for User Sw5
volatile uint8_t usrSw6Block = 0;			// Blocking Period Cnt for User Sw6
volatile uint8_t blueSwBlock = 0;			// Blocking Period Cnt for Blue Sw

volatile uint16_t swsState = 0;				// Switches State
volatile uint16_t swsStateBk = 0;			// Switches State Backup
volatile uint16_t ledState = 0;				// LED State
volatile uint16_t ledStateBk = 0;			// LED State Backup

volatile uint8_t potSel = 0;				// Potentiometer Select flag: 0 / 1
static int16_t pot1Val = 0;					// Actual value of Potentiometer 1
static int16_t pot1ValBk = 32767;			// Backup value of Potentiometer 1
static int16_t pot2Val = 0;					// Actual value of Potentiometer 2
static int16_t pot2ValBk = 32767;			// Backup value of Potentiometer 2
static int16_t potSVal = 0;					// Selected Potentiometer value
static int16_t potSValBk = 32767;			// Selected Potentiometer value backup
static int16_t remCtrlVal = 0;				// Remote Motor Control value

static int16_t asSensANG = 0;				// AS ANG Sensor Measurement
static int16_t asSensANGbk = 0;				// AS ANG Sensor Measurement backup
static int16_t angTurnsNbr = 0;				// Angle Full Turns Number
static uint8_t asANGsensErr = 0;			// AS ANG Sensor Error
static uint8_t asANGsensErrBk = 0xFF;		// AS ANG Sensor Error backup

volatile uint32_t rpsmCapt = 0;				// Motor Captured value of RPS
volatile int32_t rpsmVal = 0;				// Motor RPS measured value
volatile int32_t nperAcc = 0;				// Motor Period Number accumulator
volatile uint32_t nperCnt = 0;				// Motor Period Number counter
volatile int16_t rpsNoPulse = 0;			// Motor Number of periods without pulses

static uint16_t vbatVal = 0;				// Battery Voltage value
static uint16_t vbatValBk = 0xFFFF;			// Battery Voltage backup value
static uint8_t vbatSmpCnt = VBAT_SMP_PER - 1; // Vbat Sampling counter

TIM_OC_InitTypeDef speedPWMconf;			// Configuration Data for Speed PWM
TIM_OC_InitTypeDef rgbPWMconf;				// Configuration Data for RGBled PWM

static uint16_t motorCtrlSt = 0;			// Motor Control State
static uint16_t ctrlTstamp = 0;				// Control Time Stamp
static uint16_t remTstamp = 0;				// Time Stamp to send to remote system
static uint16_t lockDataTrf = OFF; 			// Data Transfer Lock On/Off

static int16_t motorActVal = 0;				// Motor Actuator value
static int16_t motorActValBk = 0x7FFF;		// Motor Actuator value backup
static int16_t motorActValPW = 0;			// Motor Speed Pulse Width (signed)
static uint16_t motorPWMtop = MPWMPER_MAX;	// PWM Counter Top value
static uint16_t motorPWMtopBk = MPWMPER_MAX;	// PWM Counter Top value backup

//static int16_t dcmCtrlSetp = 0;				// Motor Control SetPoint
//volatile int16_t motorSpeed = 0;			// Motor RPS sampled value
static int16_t motorAngle = 0;				// Motor Angle
static int16_t motorExtAngle = 0;			// Motor Extended Angle

static uint16_t motorCtrlType = MCTRL_DIRECT; // Motor Control Type

static uint8_t inmsgBuf[INMSG_SIZE];		// UART Receive Command Buffer
static uint32_t inmsgPnt = 0;				// UART Receive Command Pointer

static uint16_t samplePeriod = CTRLOOP_PER;			// Sample Period for Motor Control
static float mcRPMPropGain = MC_RPM_PROP_GAIN;		// Motor RPM Control Prop Gain
static uint16_t mcRPMPropGainCorr = OFF;			// Motor RPM Control Prop Gain Correction On/Off
static float mcRPMPIGain = MC_RPM_PI_GAIN;			// Motor RPM Control PI Gain
static float mcRPMPITi = MC_RPM_PI_TI;				// Motor RPM Control PI Integration Time
static float mcServoPropGain = MC_SRV_PROP_GAIN;	// Motor Servo Control Prop Gain
static uint16_t mcServoEmbPIRPM = OFF;				// Embedded PI RPM Control for Motor Servo On/Off
static float mcServoPIRPMGain = MC_SRV_PIRPM_GAIN;	// Motor Servo Control PIRPM Gain
static float mcServoPIRPMTi = MC_SRV_PIRPM_TI;		// Motor Servo Control PIRPM Integration Time

static double mcglAc[3][3] = {{0.9569, 0.004903, 3.874e-06},
							  {-0.2873, 0.9541, 0.0009297},
							  {-26.51, -11.32,- 0.009454}};
													// Motor General Linear Control A matrix
static double mcglBc[3] = {0.04291 ,  0.1781, -0.4477};
													// Motor General Linear Control B vector
static double mcglCc[3] = {-4.691, -1.54, -0.002401};
													// Motor General Linear Control C vector
static double mcglDc = 0.0;							// Motor General Linear Control D value
static double mcglPx0 = 0.0;						// Motor General Linear Control x0 past value
static double mcglPx1 = 0.0;						// Motor General Linear Control x1 past value
static double mcglPx2 = 0.0;						// Motor General Linear Control x2 past value

static double xvalPI = 0.0;					// PI Controller: x value
static double yvalPI = 0.0;					// PI Controller: y value

volatile uint8_t testMode = 0;				// Test Mode state - Local Mode is default
volatile int16_t testSignal[TESTSIGN_SIZE];	// Input Test Signal Data array
static uint16_t testSigInPnt = 0;			// Test Signal Input Pointer - size of the actual Signal
static uint16_t testSigOutPnt = 0;			// Test Signal Output Pointer
static uint16_t testSigReset = OFF;			// Test Signal Reset / Cleared flag
static uint16_t testStart = OFF;			// Test Start flag
static uint16_t testStop = OFF;				// Test Stop flag

// 4 motoros kiterjesztés
volatile int32_t nperAcc[4] = {0}, nperCnt[4] = {0};
volatile int32_t rpsmVal[4] = {0};
volatile int16_t rpsNoPulse[4] = {0};

static int16_t dcmCtrlSetp[4] = {0};  // 4 motor alapjele
volatile int16_t motorSpeed[4] = {0}; // 4 motor mért sebessége
static int16_t motorActVal[4] = {0};  // 4 motor beavatkozó jele
static int16_t motorActValBk[4] = {32767, 32767, 32767, 32767};

// PI szabályozó állapotai motoronként
static double xvalPI[4] = {0.0};
static double yvalPI[4] = {0.0};

// NSS lábak a 4 szöghelyzet-szenzorhoz
GPIO_TypeDef* AS_NSS_Ports[4] = { GPIOB, GPIOB, GPIOB, GPIOB };
uint16_t AS_NSS_Pins[4] = { RPSPI_NSS_FL_Pin, RPSPI_NSS_FR_Pin, RPSPI_NSS_RL_Pin, RPSPI_NSS_RR_Pin };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM10_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
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

  int status;
  char message[33];
  int16_t i16arg;
  int32_t i32arg;
  uint16_t u16arg, k;
  uint32_t prim;
  uint16_t regdata;
  uint16_t pulse;
  float npaval = 0.0;
  int32_t nperacc;
  uint32_t npercnt;
  uint8_t rch;
  float ctrl_par;
  float vbat_volt;
  uint8_t inSPIdata[3], outSPIdata[3];
  double xval, xtmp, yval;
  double x0, x1, x2, u, y;

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
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  for (k = 0; k < TESTSIGN_SIZE; k++) testSignal[k] = 0;

  /******	Initializing Virtual COM Port *************************************/

	StartUART2Communication();

	/******	Salutation ********************************************************/

	HAL_GPIO_WritePin(YEL3_LED_GPIO_Port,YEL3_LED_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(YEL4_LED_GPIO_Port,YEL4_LED_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(YEL5_LED_GPIO_Port,YEL5_LED_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(YEL6_LED_GPIO_Port,YEL6_LED_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(YEL7_LED_GPIO_Port,YEL7_LED_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	SendMessageLine ((char *)horLine28);
	SendMessageLine ((char *)programId);
	SendMessageLine ((char *)horLine28);
	strcpy(message,"$I\r");
	PutsUART2TxData((uint8_t *)message,strlen(message));
	HAL_GPIO_WritePin(YEL7_LED_GPIO_Port,YEL7_LED_Pin,GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(YEL6_LED_GPIO_Port,YEL6_LED_Pin,GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(YEL5_LED_GPIO_Port,YEL5_LED_Pin,GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(YEL4_LED_GPIO_Port,YEL4_LED_Pin,GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(YEL3_LED_GPIO_Port,YEL3_LED_Pin,GPIO_PIN_RESET);

	HAL_GPIO_WritePin(YEL1_LED_GPIO_Port,YEL1_LED_Pin,GPIO_PIN_SET);
	ledState |= LEDY1_STATE;

	/****** Initializing ADC **************************************************/

	InitADCmodule ();

	/****** Initializing AS ANGLE Sensor *****************************************/

	regdata = ASREG_ANGLEUNC | 0x4000;
	outSPIdata[0] = ((uint8_t *)&regdata)[1];
	outSPIdata[1] = ((uint8_t *)&regdata)[0];
	HAL_GPIO_WritePin(ANGS_NSS_GPIO_Port,ANGS_NSS_Pin,GPIO_PIN_RESET);
	status =  HAL_SPI_TransmitReceive (&hspi2, outSPIdata, inSPIdata, 2, 100);
	HAL_GPIO_WritePin(ANGS_NSS_GPIO_Port,ANGS_NSS_Pin,GPIO_PIN_SET);
	if (status == 0)
	  {
		asANGsensErr = ((inSPIdata[0] & 0xC0) >> 6) & 0x03;
		((uint8_t *)&u16arg)[1] = inSPIdata[0] & 0x3F;
		((uint8_t *)&u16arg)[0] = inSPIdata[1];
		asSensANG = (int16_t)u16arg - 8192;
		asSensANGbk = asSensANG;
		motorAngle = asSensANG;
		motorExtAngle = motorAngle;
		angTurnsNbr = 0;
	  }
	else
	  {
		SendErrorSignal(ERR_SPI1, status);
	  }

	// 1. PWM PERIÓDUSOK BEÁLLÍTÁSA (Ha a CubeMX-ben nem fixáltad)
	htim1.Init.Period = MPWMPER_MAX - 1;
	HAL_TIM_PWM_Init(&htim1);
	htim3.Init.Period = MPWMPER_MAX - 1;
	HAL_TIM_PWM_Init(&htim3);
	htim4.Init.Period = MPWMPER_MAX - 1;
	HAL_TIM_PWM_Init(&htim4);

	// 2. MÉRÉS ÉS IDŐZÍTŐ INDÍTÁSA
	HAL_TIM_Base_Start_IT(&htim10);             // Szabályozási loop időzítő
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);  // RL sebesség mérés
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);  // FL sebesség mérés
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);  // FR sebesség mérés
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);  // RR sebesség mérés

	// 3. MOTOR PWM KIMENETEK INDÍTÁSA
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);    // FL Motor
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);    // FR Motor
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	__HAL_TIM_MOE_ENABLE(&htim1);                // TIM1 speciális engedélyezés!
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);    // RL Motor
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);    // RR Motor
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	// Ha az FL/FR motorvezérlődnek szüksége van külön enable jelre:
	// Ellenőrizd a rajzodon, hogy melyik szabad lábra kötötted (pl. PC1 vagy PA4).
	// Ha nincs külön láb, és rajtuk van a jumper a vezérlőn, akkor ez a 2 sor elég.

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   mloopTick = 0;
  while (1)
  {
	while (mloopTick == 0) continue;
	if (mloopTick > 1)
		SendErrorSignal(ERR_MCOVR,0);
	mloopTick = 0;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	ctrlLoopCnt++;
	if (ctrlLoopCnt >= samplePeriod)
	  {
		for (int i = 0; i < 4; i++)
		    {
		        // RPM Sampling
		        if (nperCnt[i] > 0)
		        {
		            prim = __get_PRIMASK();
		            __disable_irq();
		            uint32_t n_cnt = nperCnt[i];
		            int32_t n_acc = nperAcc[i];
		            nperAcc[i] = 0; nperCnt[i] = 0;
		            if (!prim) __enable_irq();

		            float npaval = (n_cnt > 1) ? (float)n_acc / (float)n_cnt : (float)n_acc;
		            rpsmVal[i] = (int32_t)(RPS_FACTOR / npaval);
		            rpsNoPulse[i] = 0;
		        }
		        else
		        {
		            if (rpsmVal[i] != 0) {
		                rpsNoPulse[i]++;
		                if (rpsNoPulse[i] > 2) rpsmVal[i] /= 2;
		            }
		        }
		        motorSpeed[i] = (int16_t)rpsmVal[i];



		/******	AS5047U Angle measurement ***************************************/

		regdata = ASREG_ANGLEUNC | 0x4000;
		outSPIdata[0] = ((uint8_t *)&regdata)[1];
		outSPIdata[1] = ((uint8_t *)&regdata)[0];
		// ITT A VÁLTOZÁS: A tömb i-edik elemét használjuk a fix pin helyett!
		HAL_GPIO_WritePin(AS_NSS_Ports[i], AS_NSS_Pins[i], GPIO_PIN_RESET);
		status =  HAL_SPI_TransmitReceive (&hspi2, outSPIdata, inSPIdata, 2, 100);
		HAL_GPIO_WritePin(AS_NSS_Ports[i], AS_NSS_Pins[i], GPIO_PIN_SET);
		if (status == 0)
		  {
			asANGsensErr = ((inSPIdata[0] & 0xC0) >> 6) & 0x03;
			((uint8_t *)&u16arg)[1] = inSPIdata[0] & 0x3F;
			((uint8_t *)&u16arg)[0] = inSPIdata[1];
			asSensANG = (int16_t)u16arg - 8192;
			if (abs(asSensANG - asSensANGbk) > 4096)
			  {
				if (asSensANG >= 0)
					angTurnsNbr++;
				else
					angTurnsNbr--;
			  }
			switch (angTurnsNbr)
			  {
				case 1:
					motorExtAngle = (asSensANG - 16384);
					break;
				case 0:
					motorExtAngle = asSensANG;
					break;
				case -1:
					motorExtAngle = (asSensANG + 16384);
					break;
				default:
					motorExtAngle = asSensANG;
					angTurnsNbr = 0;
			  }
			motorAngle = asSensANG;
			asSensANGbk = asSensANG;
		  }
		else
		  {
			SendErrorSignal(ERR_SPI1, status);
		  }

		/***** ADC measurements: Potentiometer 1 and 2 *************************/

		if (CheckADCChannelForDataAvailable(ADCH_POT1) > 0 &&
			CheckADCChannelForDataAvailable(ADCH_POT2) > 0)
		  {
			GetADCChannelData(ADCH_POT1, &u16arg);
			pot1Val = 16384 - (int16_t)u16arg;
			GetADCChannelData(ADCH_POT2, &u16arg);
			pot2Val = 16384 - (int16_t)u16arg;
			// Setting Potentiometer to serve as setpoint
			if ((potSel & POTSEL_ST) == 0)
				potSVal = pot1Val;
			else
				potSVal = pot2Val;
		  }

		/****** Motor Control SetPoint Value selection ************************/

		if ((testMode & AUTOTEST_MODE) != 0)
		  { // Automatic Test Sequence mode
			if (testSigOutPnt < testSigInPnt)
			  {
				dcmCtrlSetp = testSignal[testSigOutPnt];
				remTstamp = testSigOutPnt | AUTOTEST_TSMASK;
				testSigOutPnt++;
				ctrlTstamp = testSigOutPnt;
			  }
			else
			  {
				testMode &= ~AUTOTEST_MODE;
				HAL_GPIO_WritePin(YEL6_LED_GPIO_Port,YEL6_LED_Pin,GPIO_PIN_RESET);
				ledState &= ~LEDY6_STATE;
				testStop = ON;
				ctrlTstamp = 0;
				remTstamp = ctrlTstamp;
			  }
		  }
		if ((testMode & AUTOTEST_MODE) == 0)
		  {
			if ((testMode & REMOTE_MODE) != 0)
			  { // Remote mode
				dcmCtrlSetp = remCtrlVal;
				remTstamp = ctrlTstamp;
				ctrlTstamp++;
				if (ctrlTstamp >= TSTAMP_MAX) ctrlTstamp = 0;
			  }
			else
			  { // Local mode
				if (potSVal != potSValBk)
				  {
					dcmCtrlSetp = potSVal;
					potSValBk = potSVal;
				  }
				remTstamp = ctrlTstamp;
				ctrlTstamp++;
				if (ctrlTstamp >= TSTAMP_MAX) ctrlTstamp = 0;
			  }
		  }

		/****** Motor Control Action ******************************************/

		if ((motorCtrlSt & DCMCTST_IGN) > 0)
		  { // Ignition ON
			HAL_GPIO_WritePin(DCMC_EN_B_GPIO_Port,DCMC_EN_B_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(DCMC_EN_A_GPIO_Port,DCMC_EN_A_Pin,GPIO_PIN_SET);
			// 1. MOTORVEZÉRLŐK AKTIVÁLÁSA (Enable lábak)
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // MWC_EN_RL
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // MWC_EN_RR
			    // Ha van külön FL és FR enable lábad, azokat is itt kapcsold be
			if ((motorCtrlSt & DCMCTST_BRAKE) == 0) // fék egyébként sincs
			  {
				// Motor Speed Controller
			for (int i = 0; i < 4; i++)
				{
				switch (motorCtrlType) // Szabályozó típus kiválasztása
				  {
					/*case MSERVO_LQG:
						if (dcmCtrlSetp > SANGLE_MAX)
							i16arg = SANGLE_MAX;
						else if (dcmCtrlSetp < SANGLE_MIN)
							i16arg = SANGLE_MIN;
						else
							i16arg = dcmCtrlSetp;
						u = - (double)(i16arg - motorExtAngle);
						x0 = mcglAc[0][0] * mcglPx0 + mcglAc[0][1] * mcglPx1 + mcglAc[0][2] * mcglPx2 + mcglBc[0] * u;
						x1 = mcglAc[1][0] * mcglPx0 + mcglAc[1][1] * mcglPx1 + mcglAc[1][2] * mcglPx2 + mcglBc[1] * u;
						x2 = mcglAc[2][0] * mcglPx0 + mcglAc[2][1] * mcglPx1 + mcglAc[2][2] * mcglPx2 + mcglBc[2] * u;
						y = mcglCc[0] * mcglPx0 + mcglCc[1] * mcglPx1 + mcglCc[2] * mcglPx2 + mcglDc * u;
						if (y > (double)ACTVAL_MAX)
							motorSpeed = ACTVAL_MAX;
						else if (y < (double)ACTVAL_MIN)
							motorSpeed = ACTVAL_MIN;
						else
							motorSpeed = (int16_t)y;
						mcglPx0 = x0;
						mcglPx1 = x1;
						mcglPx2 = x2;
						break;
					case MSERVO_PIPROP:
						if (dcmCtrlSetp > SANGLE_MAX)
							i16arg = SANGLE_MAX;
						else if (dcmCtrlSetp < SANGLE_MIN)
							i16arg = SANGLE_MIN;
						else
							i16arg = dcmCtrlSetp;
						i32arg = (int32_t)(mcServoPropGain * (float)(i16arg - motorExtAngle));
						xval = (double)(i32arg - motorSpeed);
						xtmp = xval + xval / mcServoPIRPMTi - xvalPI;
						yval = yvalPI + mcServoPIRPMGain * xtmp;
						if (yval > WINDUP_MAX)
							yval = WINDUP_MAX;
						else if (yval < WINDUP_MIN)
							yval = WINDUP_MIN;
						if (yval > (double)ACTVAL_MAX)
							motorActVal = ACTVAL_MAX;
						else if (yval < (double)ACTVAL_MIN)
							motorActVal = ACTVAL_MIN;
						else
							motorActVal = (int16_t)yval;
						yvalPI = yval;
						xvalPI = xval;
						break;

					case MSERVO_PROP:
						if (dcmCtrlSetp > SANGLE_MAX)
							i16arg = SANGLE_MAX;
						else if (dcmCtrlSetp < SANGLE_MIN)
							i16arg = SANGLE_MIN;
						else
							i16arg = dcmCtrlSetp;
						i32arg = (int32_t)(mcServoPropGain * (float)(i16arg - motorExtAngle));
						if (i32arg > ACTVAL_MAX)
							motorActVal = ACTVAL_MAX;
						else if (i32arg < ACTVAL_MIN)
							motorActVal = ACTVAL_MIN;
						else
							motorActVal = (int16_t)i32arg;
						break;
*/
					case MCTRL_PI:
						double xval = (double)(dcmCtrlSetp[i] - motorSpeed[i]);
						double xtmp = xval + xval / mcRPMPITi - xvalPI[i];
						double yval = yvalPI[i] + mcRPMPIGain * xtmp;

						// Anti-windup
						if (yval > WINDUP_MAX) yval = WINDUP_MAX;
						else if (yval < WINDUP_MIN) yval = WINDUP_MIN;

						// Kimenet korlátozása
						if (yval > (double)ACTVAL_MAX) motorActVal[i] = ACTVAL_MAX;
						else if (yval < (double)ACTVAL_MIN) motorActVal[i] = ACTVAL_MIN;
						else motorActVal[i] = (int16_t)yval;

						// Állapotmentés a következő mintavételhez
						yvalPI[i] = yval;
						xvalPI[i] = xval;
						break;

					/*case MCTRL_PROP:
						if (mcRPMPropGainCorr == OFF)
							i32arg = (int32_t)(mcRPMPropGain * (float)(dcmCtrlSetp - motorSpeed));
						else
							i32arg = (int32_t)(mcRPMPropGain *
											   ((mcRPMPropGain + 1.0) *
											    (float)dcmCtrlSetp / mcRPMPropGain - (float)motorSpeed));
						if (i32arg > ACTVAL_MAX)
							motorActVal = ACTVAL_MAX;
						else if (i32arg < ACTVAL_MIN)
							motorActVal = ACTVAL_MIN;
						else
							motorActVal = (int16_t)i32arg;
						break;*/

					case MCTRL_DIRECT:
					default:
						if (dcmCtrlSetp > ACTVAL_MAX)
							motorActVal = ACTVAL_MAX;
						else if (dcmCtrlSetp < ACTVAL_MIN)
							motorActVal = ACTVAL_MIN;
						else
							motorActVal = dcmCtrlSetp;
				  }
				// PWM KIÍRÁS, ha változott az érték
				    if (motorActVal[i] != motorActValBk[i])
				    {
				        motorActValPW = motorActVal[i] / MPWMRED_FACT;

				        uint16_t pulseP = (motorActValPW >= 0) ? (uint16_t)motorActValPW : 0;
				        uint16_t pulseN = (motorActValPW < 0) ? (uint16_t)(-motorActValPW) : 0;

				        switch(i)
				        {
				            case 0: // FL (TIM3 CH1/2)
				                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulseP);
				                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulseN);
				                break;
				            case 1: // FR (TIM1 CH3/4)
				                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulseP);
				                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulseN);
				                break;
				            case 2: // RL (TIM4 CH1/2)
				                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulseP);
				                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pulseN);
				                break;
				            case 3: // RR (TIM4 CH3/4)
				                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulseP);
				                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pulseN);
				                break;
				        }
				        motorActValBk[i] = motorActVal[i];
				    }

				    // Nem biztos hogy kell
					/*if (motorPWMtop != motorPWMtopBk)
					  { // Action for PS Voltage correction
						__HAL_TIM_SET_AUTORELOAD(&htim3, motorPWMtop - 1);
						motorPWMtopBk = motorPWMtop;
					  }
					if (motorActValPW >= 0)
					  {
						pulse = (uint16_t)motorActValPW;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
					  }
					else
					  {
						pulse = (uint16_t)(-motorActValPW);
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
					  }
					motorActValBk = motorActVal;
				  }*/

			  } // for ciklus vége
		  }
		else // Ignition OFF - Biztonsági leállás
		{
		    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

		    // Összes PWM leállítása
		    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
		    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
		    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
		    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
		    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

		    for(int i=0; i<4; i++) {
		        yvalPI[i] = 0; xvalPI[i] = 0; motorActValBk[i] = 0x7FFF;
		    }
		}
		// Sending Motor Control State to VCP Ez kell??
		if (lockDataTrf == OFF)
		  {
			switch (motorCtrlType)
			  {
				case MSERVO_PROP:
				case MSERVO_PIPROP:
				case MSERVO_LQG:
					sprintf(message,"$CD%04X%04X%04X%04X%04X\r",
							*(uint16_t *)&dcmCtrlSetp,*(uint16_t *)&motorSpeed,
							*(uint16_t *)&motorExtAngle,*(uint16_t *)&motorActVal,
							remTstamp);
					break;
				case MCTRL_DIRECT:
				case MCTRL_PROP:
				case MCTRL_PI:
				default:
					sprintf(message,"$CD%04X%04X%04X%04X%04X\r",
							*(uint16_t *)&dcmCtrlSetp,*(uint16_t *)&motorSpeed,
							*(uint16_t *)&motorExtAngle,*(uint16_t *)&motorActVal,
							remTstamp);
			  }
			PutsUART2TxData((uint8_t *)message,strlen(message));
		  }
		ctrlLoopCnt = 0;
	  }

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

	/******	1s Cycle ***********************************************************/

	if (secTick > 0)
	  { //-----	1s Cycle -------------------------------------------------------
		//-----	Reading Power Supply Voltage -----------------------------------
		vbatSmpCnt++;
		if (vbatSmpCnt >= VBAT_SMP_PER)
		  {
			if (CheckADCChannelForDataAvailable(ADCH_VBAT) > 0)
			  {
				GetADCChannelData(ADCH_VBAT, &vbatVal);
				if (vbatVal != vbatValBk)
				  {
					// Updating Motor PWM TOP value for correcting PS Voltage change
					motorPWMtop = (uint16_t)roundf((float)vbatVal * MPWMPER_CTRF);
					if (motorPWMtop < MPWMPER_MIN) motorPWMtop = MPWMPER_MIN;
					if (motorPWMtop > MPWMPER_MAX) motorPWMtop = MPWMPER_MAX;
					vbat_volt = (float)vbatVal / 4096.0 * 3.3 * 9.3 / 1.8;
					// Sending PS Voltage level to VCP
					u16arg = (uint16_t)(vbat_volt * 100.0);
					sprintf(message,"$BV%04X\r",u16arg);
					PutsUART2TxData((uint8_t *)message,strlen(message));
					// Indicating PS Voltage State on 3Color LED
					if (vbat_volt > BATST_OK)
					  { // Overcharged
						HAL_GPIO_WritePin(PS_LED_R_GPIO_Port,PS_LED_R_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(PS_LED_G_GPIO_Port,PS_LED_G_Pin,GPIO_PIN_SET);
					  }
					else if (vbat_volt > BATST_DISCH)
					  { // Battery OK
						HAL_GPIO_WritePin(PS_LED_R_GPIO_Port,PS_LED_R_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(PS_LED_G_GPIO_Port,PS_LED_G_Pin,GPIO_PIN_SET);
					  }
					else if (vbat_volt > BATST_DDISCH)
					  { // Discharged
						HAL_GPIO_WritePin(PS_LED_R_GPIO_Port,PS_LED_R_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(PS_LED_G_GPIO_Port,PS_LED_G_Pin,GPIO_PIN_RESET);
					  }
					else
					  { // Deep discharged
						HAL_GPIO_WritePin(PS_LED_R_GPIO_Port,PS_LED_R_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(PS_LED_G_GPIO_Port,PS_LED_G_Pin,GPIO_PIN_RESET);
					  }
					vbatValBk = vbatVal;
				  }
			  }
			vbatSmpCnt = 0;
		  }
		//------	End of 1s Cycle --------------------------------------------
		secTick = 0;
	  }

	/******	Messaging Test Reset to VCP ***************************************/

	if (testSigReset == ON)
	  {
		strcpy(message,"$TR\r");
		PutsUART2TxData((uint8_t *)message,strlen(message));
		testSigReset = OFF;
	  }

	/******	Sending Device State to VCP ***************************************/

	statTrfCnt++;
	if (statTrfCnt >= STATTRF_PER)
	  {
		if (lockDataTrf == OFF)
		  {
			// Sending Potentiometer 1 Voltage
			if (pot1Val != pot1ValBk)
			  {
				sprintf(message,"$DP1,%04X\r",*(uint16_t *)&pot1Val);
				PutsUART2TxData((uint8_t *)message,strlen(message));
				pot1ValBk = pot1Val;
			  }
			// Sending Potentiometer 2 Voltage
			if (pot2Val != pot2ValBk)
			  {
				sprintf(message,"$DP2,%04X\r",*(uint16_t *)&pot2Val);
				PutsUART2TxData((uint8_t *)message,strlen(message));
				pot2ValBk = pot2Val;
			  }
			// Sending switches' state
			if (swsState != swsStateBk)
			  {
				sprintf(message,"$DS%04X\r",swsState);
				PutsUART2TxData((uint8_t *)message,strlen(message));
				swsStateBk = swsState;
			  }
			//	Sending LEDs' state
			if (ledState != ledStateBk)
			  {
				sprintf(message,"$DL%04X\r",ledState);
				PutsUART2TxData((uint8_t *)message,strlen(message));
				ledStateBk = ledState;
			  }
			// Sending AS5047U ANG sensor error state
			if (asANGsensErr != asANGsensErrBk)
			  {
				sprintf(message,"$SA%02X\r",asANGsensErr);
				PutsUART2TxData((uint8_t *)message,strlen(message));
				asANGsensErrBk = asANGsensErr;
			  }
		  }
		statTrfCnt = 0;
	  }

	/******	Receiving Control Messages from VCP *******************************/

	while (TestUART2RxData() == SET)
	  {
		rch = GetcUART2RxData();
		if (inmsgPnt > 0)
		  {
			if (rch == CR_C)
			  {
				// A complete command is in the buffer
				inmsgBuf[inmsgPnt] = NUL_C;
				// Processing the received command
				switch (inmsgBuf[1])
				  {
					case 'D':
						switch (inmsgBuf[2])
						  {
							case 'L':	// $DL LEDs State Zequest
								sprintf(message,"$DL%04X\r",ledState);
								PutsUART2TxData((uint8_t *)message,strlen(message));
								break;
							case 'S':	// $DS Switches State Request
								sprintf(message,"$DS%04X\r",swsState);
								PutsUART2TxData((uint8_t *)message,strlen(message));
								break;
							case 'P':	// $DP Potentiometer Measurement Value Request
								switch (inmsgBuf[2])
								  {
									case '1':	// $DP1 Potentiometer 1 Value Request
										sprintf(message,"$DP1%04X\r",*(uint16_t *)&pot1Val);
										PutsUART2TxData((uint8_t *)message,strlen(message));
										break;
									case '2':	// $DP2 Potentiometer 2 Value Request
										sprintf(message,"$DP2%04X\r",*(uint16_t *)&pot2Val);
										PutsUART2TxData((uint8_t *)message,strlen(message));
										break;
									default:
										// Unknown Command
										SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
								  }
								break;
							case 'B': // 'DB' Battery Voltage Request
								u16arg = (uint16_t)((float)vbatVal / 4096.0 * 330.0 * 9.3 / 1.8);
								sprintf(message,"$BV%04X\r",u16arg);
								PutsUART2TxData((uint8_t *)message,strlen(message));
								break;
							default:
								// Unknown Command
								SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
						  }
						break;
					case 'R':	// Remote Potentiometer Value
						if (inmsgBuf[2] == 'P')
						  {
							status = sscanf ((char *)(inmsgBuf + 3),"%hx",&u16arg);
							if (status == 1)
							  {
								remCtrlVal = *(int16_t *)&u16arg;
							  }
							else
								SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
						  }
						else
							SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
						break;
					case 'T':	// $T - Test Input Handling
						switch (inmsgBuf[2])
						  {
							case 'R':	// $TR - Reset Test Signal Storage
								lockDataTrf = OFF;
								testSigInPnt = 0;
								testSigOutPnt = 0;
								HAL_GPIO_WritePin(YEL7_LED_GPIO_Port,YEL7_LED_Pin,GPIO_PIN_RESET);
								ledState &= ~LEDY7_STATE;
								HAL_GPIO_WritePin(YEL6_LED_GPIO_Port,YEL6_LED_Pin,GPIO_PIN_RESET);
								ledState &= ~LEDY6_STATE;
								break;
							case 'L':	// $TL - Receive Single Signal Datum
								status = sscanf ((char *)(inmsgBuf + 3),"%hx",&u16arg);
								if (status == 1)
								  {
									if (testSigInPnt < TESTSIGN_SIZE)
									  {
										lockDataTrf = ON;
										testSignal[testSigInPnt] = *(int16_t *)&u16arg;
										testSigInPnt++;
										if (ledY6sign == 0)
										  {
											HAL_GPIO_WritePin(YEL7_LED_GPIO_Port,YEL7_LED_Pin,GPIO_PIN_SET);
											ledState |= LEDY7_STATE;
											ledY6sign++;
										  }
										else
										  {
											if (ledY6sign > TESTDNLD_BLRATE)
											  {
												if ((ledState & LEDY7_STATE) == 0)
												  {
													HAL_GPIO_WritePin(YEL7_LED_GPIO_Port,YEL7_LED_Pin,GPIO_PIN_SET);
													ledState |= LEDY7_STATE;
												  }
												else
												  {
													HAL_GPIO_WritePin(YEL7_LED_GPIO_Port,YEL7_LED_Pin,GPIO_PIN_RESET);
													ledState &= ~LEDY7_STATE;
												  }
												ledY6sign = 0;
											  }
											ledY6sign++;
										  }
									  }
								  }
								else
									SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
								break;
							case 'E':	// $TE - End of Signal Load
								lockDataTrf = OFF;
								status = sscanf ((char *)(inmsgBuf + 3),"%hx",&u16arg);
								if (status == 1)
								  {
									if (u16arg != testSigInPnt)
									  { // Error in download data
										SendErrorSignal(ERR_DNLD,ERI_DNLD_SIZE);
									  }
								  }
								else
									SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
								if (testSigInPnt > 0)
						  	  	  {
									HAL_GPIO_WritePin(YEL7_LED_GPIO_Port,YEL7_LED_Pin,GPIO_PIN_SET);
									ledState |= LEDY7_STATE;
						  	  	  }
								else
						  	  	  {
									HAL_GPIO_WritePin(YEL7_LED_GPIO_Port,YEL7_LED_Pin,GPIO_PIN_RESET);
									ledState &= ~LEDY7_STATE;
						  	  	  }
								break;
							default:
								// Unknown Command
								SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
						  }

						break;
					case 'C':	// Control Parameters
						switch (inmsgBuf[2])
						  {
							case 'C':	// $CC Control Type and Sample Time
								switch (inmsgBuf[3])
								  {
									case 'T': // $CCT Select Control Type
										status = sscanf ((char *)(inmsgBuf + 4),"%hx",&u16arg);
										if (status == 1)
										  {
											motorCtrlType = u16arg;
										  }
										else
											SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
										break;
									case 'P': // $CCP Set Control Sample Period
										status = sscanf ((char *)(inmsgBuf + 4),"%hx",&u16arg);
										if (status == 1)
										  {
											samplePeriod = u16arg;
											ctrlTstamp = 0;
										  }
										else
											SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
										break;
									default:
										// Unknown Command
										SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
								  }
								break;
							case 'R':	// $CR Motor RPM Control
								switch (inmsgBuf[3])
								  {
									case 'P': // $CRP
										switch (inmsgBuf[4])
										  {
											case 'P': // $CRPP
												switch (inmsgBuf[5])
												  {
													case 'G': // $CRPPG  Set Proportional Gain
														status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
														if (status == 1)
														  {
															mcRPMPropGain = ctrl_par;
														  }
														else
															SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
														break;
													default:
														// Unknown Command
														SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
												  }
												break;
											case 'C': // $CRPC
												switch (inmsgBuf[5])
												  {
													case 'E': // $CRPCE  Set Proportional Gain Correction
														status = sscanf ((char *)(inmsgBuf + 6),"%hx",&u16arg);
														if (status == 1)
														  {
															mcRPMPropGainCorr = u16arg;
														  }
														else
															SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
														break;
													default:
														// Unknown Command
														SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
												  }
												break;
											case 'I':
												switch (inmsgBuf[5])
												  {
													case 'G': // $CRPIG Set PI Gain
														status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
														if (status == 1)
														  {
															mcRPMPIGain = ctrl_par;
														  }
														else
															SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
														break;
													case 'I': // $CRPII Set PI Integration Time
														status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
														if (status == 1)
														  {
															mcRPMPITi = ctrl_par;
														  }
														else
															SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
														break;
													default:
														// Unknown Command
														SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
												  }
												break;
											default:
												// Unknown Command
												SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
										  }
										break;
									default:
										// Unknown Command
										SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
								  }
								break;
							case 'S':	// Motor Servo Control
								switch (inmsgBuf[3])
								  {
									case 'P':
										switch (inmsgBuf[4])
										  {
											case 'P':
												switch (inmsgBuf[5])
												  {
													case 'G': // $CSPPG  Set Proportional Gain
														status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
														if (status == 1)
														  {
															mcServoPropGain = ctrl_par;
														  }
														else
															SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
														break;
													default:
														// Illegal Command
														SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
												  }
												break;
											case 'I':
												switch (inmsgBuf[5])
												  {
													case 'E': // $CSPIE PIRPM On/Off
														status = sscanf ((char *)(inmsgBuf + 6),"%hx",&u16arg);
														if (status == 1)
														  {
															mcServoEmbPIRPM = u16arg;
														  }
														else
															SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
														break;
													case 'G': // $CSPIG PIRPM Gain
														status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
														if (status == 1)
														  {
															mcServoPIRPMGain = ctrl_par;
														  }
														else
															SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
														break;
													case 'I': // $CSPII PIRPM Integration Time
														status = sscanf ((char *)(inmsgBuf + 6),"%hx",&u16arg);
														if (status == 1)
														  {
															mcServoPIRPMTi = ctrl_par;
														  }
														else
															SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
														break;
													default:
														// Unknown Command
														SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
												  }
												break;
											default:
												// Unknown Command
												SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
										  }
										break;
									default:
										// Unknown Command
										SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
								  }
								break;
							case 'L': // $CL General Linear Control
								switch (inmsgBuf[3])
								  {
									case 'A': // $CLA	Ac matrix
										switch (inmsgBuf[4])
										  {
											case '0': // $CLA0	Ac matrix row 0
												switch (inmsgBuf[5])
												  {
													case '0': // $CLA00	Ac(0,0)
														status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
														if (status == 1)
														  {
															mcglAc[0][0] = ctrl_par;
														  }
														else
															SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
														break;
													case '1': // $CLA01	Ac(0,1)
														status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
														if (status == 1)
														  {
															mcglAc[0][1] = ctrl_par;
														  }
														else
															SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
														break;
													case '2': // $CLA02	Ac(0,2)
														status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
														if (status == 1)
														  {
															mcglAc[0][2] = ctrl_par;
														  }
														else
															SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
														break;
													default:
														// Unknown Command
														SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
												  }
												break;
											case '1': // $CLA1	Ac matrix row 1
												switch (inmsgBuf[5])
												  {
													case '0': // $CLA10	Ac(1,0)
														status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
														if (status == 1)
														  {
															mcglAc[1][0] = ctrl_par;
														  }
														else
															SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
														break;
													case '1': // $CLA11	Ac(1,1)
														status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
														if (status == 1)
														  {
															mcglAc[1][1] = ctrl_par;
														  }
														else
															SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
														break;
													case '2': // $CLA12	Ac(1,2)
														status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
														if (status == 1)
														  {
															mcglAc[1][2] = ctrl_par;
														  }
														else
															SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
														break;
													default:
														// Unknown Command
														SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
												  }
												break;
											case '2': // $CLA2	Ac matrix row 2
												switch (inmsgBuf[5])
												  {
													case '0': // $CLA20	Ac(2,0)
														status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
														if (status == 1)
														  {
															mcglAc[2][0] = ctrl_par;
														  }
														else
															SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
														break;
													case '1': // $CLA21	Ac(2,1)
														status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
														if (status == 1)
														  {
															mcglAc[2][1] = ctrl_par;
														  }
														else
															SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
														break;
													case '2': // $CLA22	Ac(2,2)
														status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
														if (status == 1)
														  {
															mcglAc[2][2] = ctrl_par;
														  }
														else
															SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
														break;
													default:
														// Unknown Command
														SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
												  }
												break;
											default:
												// Unknown Command
												SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
										  }
										break;
									case 'B': // $CLB	Bc vector
										switch (inmsgBuf[4])
										  {
											case '0': // $CLB0
												status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
												if (status == 1)
												  {
													mcglBc[0] = ctrl_par;
												  }
												else
													SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
												break;
											case '1': // $CLB1
												status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
												if (status == 1)
												  {
													mcglBc[1] = ctrl_par;
												  }
												else
													SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
												break;
											case '2': // $CLB2
												status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
												if (status == 1)
												  {
													mcglBc[2] = ctrl_par;
												  }
												else
													SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
												break;
											default:
												// Unknown Command
												SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
										  }
										break;
									case 'C': // $CLC	Cc vector
										switch (inmsgBuf[4])
										  {
											case '0': // $CLC0
												status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
												if (status == 1)
												  {
													mcglCc[0] = ctrl_par;
												  }
												else
													SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
												break;
											case '1': // $CLC1
												status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
												if (status == 1)
												  {
													mcglCc[1] = ctrl_par;
												  }
												else
													SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
												break;
											case '2': // $CLC2
												status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
												if (status == 1)
												  {
													mcglCc[2] = ctrl_par;
												  }
												else
													SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
												break;
											default:
												// Unknown Command
												SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
										  }
										break;
									case 'D': // $CLD	Dc value
										status = sscanf ((char *)(inmsgBuf + 6),"%f",&ctrl_par);
										if (status == 1)
										  {
											mcglDc = ctrl_par;
										  }
										else
											SendErrorSignal(ERR_ICMD,ERI_ICMD_ARN);
										break;
									default:
										// Unknown Command
										SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
								  }
								break;
							default:
								// Unknown Command
								SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
						  }
						break;
					default:
						// Unknown Command
						SendErrorSignal(ERR_ICMD,ERI_ICMD_UNN);
				  }
				inmsgPnt = 0;
				break;
			  }
			else
			  {
				// Character belonging to Command
				inmsgBuf[inmsgPnt] = rch;
				inmsgPnt++;
				if (inmsgPnt >= INMSG_SIZE)
				  {
					// Invalid command: too long
					inmsgPnt = 0;
					SendErrorSignal(ERR_ICMD,ERI_ICMD_ILL);
				  }
			  }
		  }
		else
		  { // Waiting for Command Start character
			if (rch == '$')
			  {
				// Command Start character found
				inmsgBuf[0] = rch;
				inmsgPnt++;
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
void SystemClock_Config(void)
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_14;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_28CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T4_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_15;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_7;
  sConfigInjected.InjectedRank = 3;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
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
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigIC.ICFilter = 4;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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
  htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim3.Init.Period = 2048;
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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart2.Init.BaudRate = 460800;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, YEL4_LED_Pin|DCMC_EN_B_Pin|RED_LED_Pin|GREEN_LED_Pin
                          |YEL7_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RPMS_NSS_GPIO_Port, RPMS_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ANGS_NSS_Pin|YEL5_LED_Pin|YEL6_LED_Pin|DCMC_EN_A_Pin
                          |YEL3_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PS_LED_R_Pin|YEL1_LED_Pin|YEL2_LED_Pin|PS_LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : YEL4_LED_Pin DCMC_EN_B_Pin RED_LED_Pin GREEN_LED_Pin
                           YEL7_LED_Pin */
  GPIO_InitStruct.Pin = YEL4_LED_Pin|DCMC_EN_B_Pin|RED_LED_Pin|GREEN_LED_Pin
                          |YEL7_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RPMS_NSS_Pin */
  GPIO_InitStruct.Pin = RPMS_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RPMS_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMC_IN1_B_Pin DCMC_IN2_B_Pin RPSM_QB_Pin */
  GPIO_InitStruct.Pin = DCMC_IN1_B_Pin|DCMC_IN2_B_Pin|RPSM_QB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ANGS_NSS_Pin */
  GPIO_InitStruct.Pin = ANGS_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ANGS_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PS_LED_R_Pin YEL1_LED_Pin YEL2_LED_Pin PS_LED_G_Pin */
  GPIO_InitStruct.Pin = PS_LED_R_Pin|YEL1_LED_Pin|YEL2_LED_Pin|PS_LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USR_SW3_Pin USR_SW6_Pin USR_SW5_Pin USR_SW4_Pin */
  GPIO_InitStruct.Pin = USR_SW3_Pin|USR_SW6_Pin|USR_SW5_Pin|USR_SW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USR_SW1_Pin */
  GPIO_InitStruct.Pin = USR_SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USR_SW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : YEL5_LED_Pin YEL6_LED_Pin DCMC_EN_A_Pin YEL3_LED_Pin */
  GPIO_InitStruct.Pin = YEL5_LED_Pin|YEL6_LED_Pin|DCMC_EN_A_Pin|YEL3_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USR_SW2_Pin */
  GPIO_InitStruct.Pin = USR_SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USR_SW2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  SYSTICK callback.
  * @retval None
  */
void HAL_SYSTICK_Callback(void)
{
	secCnt++;
	if (secCnt == 500)
	  {
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin,GPIO_PIN_SET);
		ledState |= LEDG_STATE;
	  }
	if (secCnt >= 1000)
	  {
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin,GPIO_PIN_RESET);
		ledState &= ~LEDG_STATE;
		secTick++;
		secCnt = 0;
	  }
	// Handling LED Signals
	if (ledRsign > 0)
	  {
		if (ledRsign == 1)
		  {
			HAL_GPIO_WritePin(RED_LED_GPIO_Port,RED_LED_Pin,GPIO_PIN_RESET);
			ledState &= ~LEDR_STATE;
		  }
		ledRsign--;
	  }
	// Handling Switch (pushbutton) actions
	if (usrSw1Block > 0)
	  {
		if (usrSw1Block == 1)
		  {
			// Function: Select Potentiometer 1 or 2
			if (HAL_GPIO_ReadPin(USR_SW1_GPIO_Port,USR_SW1_Pin))
			  {
				// Pushbutton released
				swsState &= ~SW1_STATE;
			  }
			else
			  {
				// Pushbutton pressed
				swsState |= SW1_STATE;
				potSel++;
				potSel = potSel & 0x01;
				if (potSel == 1)
				  {
					HAL_GPIO_WritePin(YEL1_LED_GPIO_Port,YEL1_LED_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(YEL2_LED_GPIO_Port,YEL2_LED_Pin,GPIO_PIN_SET);
					ledState &= ~LEDY1_STATE;
					ledState |= LEDY2_STATE;
				  }
				else
				  {
					HAL_GPIO_WritePin(YEL2_LED_GPIO_Port,YEL2_LED_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(YEL1_LED_GPIO_Port,YEL1_LED_Pin,GPIO_PIN_SET);
					ledState &= ~LEDY2_STATE;
					ledState |= LEDY1_STATE;
				  }
			  }
		  }
		usrSw1Block--;
	  }
	if (usrSw2Block > 0)
	  {
		// Function: Motor Ignition On/Off
		if (usrSw2Block == 1)
		  {
			if (HAL_GPIO_ReadPin(USR_SW2_GPIO_Port,USR_SW2_Pin))
			  {
				// Pushbutton released
				swsState &= ~SW2_STATE;
			  }
			else
			  {
				// Pushbutton pressed
				swsState |= SW2_STATE;
				if ((motorCtrlSt & DCMCTST_IGN) == 0)
				  {
					motorCtrlSt |= DCMCTST_IGN;
					xvalPI = 0.0;
					yvalPI = 0.0;
					HAL_GPIO_WritePin(YEL3_LED_GPIO_Port,YEL3_LED_Pin,GPIO_PIN_SET);
					ledState |= LEDY3_STATE;
				  }
				else
				  {
					motorCtrlSt = DCMCTST_IDLE;
					HAL_GPIO_WritePin(YEL3_LED_GPIO_Port,YEL3_LED_Pin,GPIO_PIN_RESET);
					ledState &= ~LEDY3_STATE;
				  }
			  }
		  }
		usrSw2Block--;
	  }
	if (usrSw3Block > 0)
	  {
		// Function: Motor Brake On/Off
		if (usrSw3Block == 1)
		  {
			if (HAL_GPIO_ReadPin(USR_SW3_GPIO_Port,USR_SW3_Pin))
			  {
				// Pushbutton released
				swsState &= ~SW3_STATE;
				HAL_GPIO_WritePin(YEL4_LED_GPIO_Port,YEL4_LED_Pin,GPIO_PIN_RESET);
				ledState &= ~LEDY4_STATE;
				// Motor Control: Setting Brake State OFF
				motorCtrlSt = DCMCTST_IDLE;
				HAL_GPIO_WritePin(YEL3_LED_GPIO_Port,YEL3_LED_Pin,GPIO_PIN_RESET);
				ledState &= ~LEDY3_STATE;
			  }
			else
			  {
				// Pushbutton pressed
				swsState |= SW3_STATE;
				HAL_GPIO_WritePin(YEL4_LED_GPIO_Port,YEL4_LED_Pin,GPIO_PIN_SET);
				ledState |= LEDY4_STATE;
				// Motor Control: Setting Brake State ON
				motorCtrlSt |= DCMCTST_BRAKE;
			  }
		  }
		usrSw3Block--;
	  }
	if (usrSw4Block > 0)
	  {
		// Function: Remote Operation Mode On/Off
		if (usrSw4Block == 1)
		  {
			if (HAL_GPIO_ReadPin(USR_SW4_GPIO_Port,USR_SW4_Pin))
			  {
				// Pushbutton released
				swsState &= ~SW4_STATE;
			  }
			else
			  {
				// Pushbutton pressed
				swsState |= SW4_STATE;
				// Remote Mode ON / OFF
				if ((testMode & REMOTE_MODE) == 0)
				  {
					// Set Remote Mode - simultaneously clear Automatic Test
					testMode = REMOTE_MODE;
					HAL_GPIO_WritePin(YEL5_LED_GPIO_Port,YEL5_LED_Pin,GPIO_PIN_SET);
					ledState |= LEDY5_STATE;
					HAL_GPIO_WritePin(YEL6_LED_GPIO_Port,YEL6_LED_Pin,GPIO_PIN_RESET);
					ledState &= ~LEDY6_STATE;
				  }
				else
				  {
					// Clear Remote Mode - setting Local Mode
					testMode = LOCAL_MODE;
					HAL_GPIO_WritePin(YEL5_LED_GPIO_Port,YEL5_LED_Pin,GPIO_PIN_RESET);
					ledState &= ~LEDY5_STATE;
				  }
			  }
		  }
		usrSw4Block--;
	  }
	if (usrSw5Block > 0)
	  {
		// Function: Automatic Test Operation Mode On/Off
		if (usrSw5Block == 1)
		  {
			if (HAL_GPIO_ReadPin(USR_SW5_GPIO_Port,USR_SW5_Pin))
			  {
				// Pushbutton released
				swsState &= ~SW5_STATE;
			  }
			else
			  {
				// Pushbutton pressed
				swsState |= SW5_STATE;
				if ((testMode & AUTOTEST_MODE) == 0)
				  {
					// Set Automatic Test Mode if a valid Test sequence is available
					//		otherwise Error signal is sent
					if (testSigInPnt > 1)
					  {
						testMode |= AUTOTEST_MODE;
						testSigOutPnt = 0;
						HAL_GPIO_WritePin(YEL6_LED_GPIO_Port,YEL6_LED_Pin,GPIO_PIN_SET);
						ledState |= LEDY6_STATE;
						testStart = ON;
					  }
					else
					  {
						SendErrorLEDSignal();
					  }
				  }
				else
				  {
					testMode &= ~AUTOTEST_MODE;
					testStop = ON;
					HAL_GPIO_WritePin(YEL6_LED_GPIO_Port,YEL6_LED_Pin,GPIO_PIN_RESET);
					ledState &= ~LEDY6_STATE;
				  }
			  }
		  }
		usrSw5Block--;
	  }
	if (usrSw6Block > 0)
	  {
		if (usrSw6Block == 1)
		  {
			if (HAL_GPIO_ReadPin(USR_SW6_GPIO_Port,USR_SW6_Pin))
			  {
				// Pushbutton released
				swsState &= ~SW6_STATE;
			  }
			else
			  {
				// Pushbutton pressed
				swsState |= SW6_STATE;
				// Deleting Test signal
				testMode &= ~AUTOTEST_MODE;
				testSigInPnt = 0;
				testSigOutPnt = 0;
				testSigReset = ON;
				HAL_GPIO_WritePin(YEL7_LED_GPIO_Port,YEL7_LED_Pin,GPIO_PIN_RESET);
				ledState &= ~LEDY7_STATE;
				HAL_GPIO_WritePin(YEL6_LED_GPIO_Port,YEL6_LED_Pin,GPIO_PIN_RESET);
				ledState &= ~LEDY6_STATE;
			  }
		  }
		usrSw6Block--;
	  }
/*	if (blueSwBlock > 0)
	  {
		if (blueSwBlock == 1)
		  {
			if (HAL_GPIO_ReadPin(BLUE_SW_GPIO_Port,BLUE_SW_Pin))
			  {
				// Pushbutton released
				swsState &= ~SWB_STATE;
			  }
			else
			  {
				// Pushbutton pressed
				swsState |= SWB_STATE;
				SendMessageLine ((char *)programId);
			  }
		  }
		blueSwBlock--;
	  }
*/
}

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	  {
		case USR_SW1_Pin:
			usrSw1Block = SWBLOCK_PER;
			break;
		case USR_SW2_Pin:
			usrSw2Block = SWBLOCK_PER;
			break;
		case USR_SW3_Pin:
			usrSw3Block = SWBLOCK_PER;
			break;
		case USR_SW4_Pin:
			usrSw4Block = SWBLOCK_PER;
			break;
		case USR_SW5_Pin:
			usrSw5Block = SWBLOCK_PER;
			break;
		case USR_SW6_Pin:
			usrSw6Block = SWBLOCK_PER;
			break;
/*		case BLUE_SW_Pin:
			blueSwBlock = SWBLOCK_PER;
			break;
*/
		default:;
	  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM10)
	  {
		mloopTick++;
	  }
}

/**
  * @brief  Input Capture callback in non blocking mode
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  uint32_t captval;
  int32_t npincr;
  uint8_t rpsmdir;
  uint32_t prim;

	if (htim->Instance == TIM2)
	  {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		  {
			prim = __get_PRIMASK();
		    __disable_irq();
			captval = __HAL_TIM_GET_COMPARE (htim, TIM_CHANNEL_1);
			rpsmdir = (uint8_t)HAL_GPIO_ReadPin(RPSM_QB_GPIO_Port,RPSM_QB_Pin);
			if (captval < rpsmCapt)
				npincr = (int32_t)(0xFFFFFFFF - rpsmCapt + captval + 1);
			else
				npincr = (int32_t)(captval - rpsmCapt);
			if (rpsmdir == 0)
				nperAcc = nperAcc + npincr;
			else
				nperAcc = nperAcc - npincr;
			nperCnt++;
			rpsmCapt = captval;
			if (!prim) __enable_irq();
		  }
	  }
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
		UART2_RxCpltCallback();
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
 void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
		UART2_TxCpltCallback();
}

/**
  * @brief  UART error callbacks.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
		SendErrorSignal(ERR_UART2,ERI_UART_HAL);
	else
		SendErrorSignal(ERR_HAL,0);
}

/**
  * @brief  Sending Error Signal to Red LED and
  * 		to the Config Interface.
  *
  * @param  sigid	 signal (Red LED) identifier
  * @param  info	 auxiliary information
  *
  * @retval None
  */
void SendErrorSignal(uint16_t sigid, uint32_t info)
{
  char message[32];

	HAL_GPIO_WritePin(RED_LED_GPIO_Port,RED_LED_Pin,GPIO_PIN_SET);
	ledState |= LEDR_STATE;
	ledRsign = LEDSIGN_PER;
	sprintf(message,"$DE%04X,%08lX\r",sigid,info);
	PutsUART2TxData((uint8_t *)message,strlen(message));
}

/**
  * @brief  Sending Error Signal only to Red LED.
  *
  * @param  noine
  *
  * @retval None
  */
void SendErrorLEDSignal()
{
	HAL_GPIO_WritePin(RED_LED_GPIO_Port,RED_LED_Pin,GPIO_PIN_SET);
	ledState |= LEDR_STATE;
	ledRsign = LEDSIGN_PER;
}

/**
  * @brief	Sending a Message Line to VCP.
  * @param  message		a NUL terminated string
  * @retval None
  */
void SendMessageLine (char *message)
{
  char outmsg[68];

    sprintf(outmsg,"$DM%s\r",message);
    PutsUART2TxData((uint8_t *)outmsg,strlen(outmsg));
    PutcUART2TxData('\0');
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
