/************************************************************************************************************/
/*																											*/
/*  Program name:	AS5047U header file																		*/
/*																											*/
/*  ( C ) Alexandros Soumelidis, 2021																		*/
/*																											*/
/************************************************************************************************************/

#ifndef AS5047U_H
#define AS5047U_H

/***** AS5047U Register Addresses ***************************************************************************/

#define	ASREG_NOP				0x0000		/* No operation													*/
#define	ASREG_ERRFL				0x0001		/* Error register												*/
#define	ASREG_PROG				0x0001		/* Programming register											*/

#define	ASREG_DIA				0x3FF5		/* DIAGNOSTIC													*/
#define	ASREG_AGC				0x3FF9		/* AGC Value													*/
#define	ASREG_SIN_DATA			0x3FFA		/* Raw digital sine channel data								*/
#define	ASREG_COS_DATA			0x3FFB		/* Raw digital cosine channel data								*/
#define	ASREG_VEL				0x3FFC		/* Velocity														*/
#define	ASREG_MAG				0x3FFD		/* CORDIC magnitude												*/
#define	ASREG_ANGLEUNC			0x3FFE		/* Measured angle without dynamic angle error compensation		*/
#define	ASREG_ANGLECOM			0x3FFF		/* Measured angle with dynamic angle error compensation			*/
#define	ASREG_ECC_CHKSUM		0x00D1		/* ECC checksum calculated based on actual register setting		*/

#define	ASNVREG_DISABLE			0x0015		/* Outputs and filter disable register							*/
#define	ASNVREG_ZPOSM			0x0016		/* Zero position MSB											*/
#define	ASNVREG_ZPOSL			0x0017		/* Zero position LSB/ MAG diagnostic							*/
#define	ASNVREG_SETTINGS1		0x0018		/* Custom setting register 1									*/
#define	ASNVREG_SETTINGS2		0x0019		/* Custom setting register 2									*/
#define	ASNVREG_SETTINGS3		0x001A		/* Custom setting register 3									*/
#define	ASNVREG_ECC				0x001B		/* ECC Settings													*/

/***** AS5047U Register Definitions *************************************************************************/

/*---- ERRFL (0x0001) --------------------------------------------------------------------------------------*/

#define	CORDIC_OVR_BP			10			/* Bit pos.: Reading the Overflow Bit of the CORDIC				*/
#define	CORDIC_OVR_BM			0x0400		/* Bit mask: Reading the Overflow Bit of the CORDIC				*/
#define	OFFC_NOTFIN_BP			9			/* Bit pos.: Internal offset compensation is not finished (1)	*/
#define	OFFC_NOTFIN_BM			0x0200		/* Bit mask: Internal offset compensation is not finished (1)	*/
#define WDTST_BP				7			/* Bit pos.: Internal oscillator or watchdog error				*/
#define	WDTST_BM				0x0080		/* Bit mask: Internal oscillator or watchdog error				*/
#define	CRCERR_BP				6			/* Bit pos.: CRC error during SPI communication					*/
#define	CRCERR_BM				0x0040		/* Bit mask: CRC error during SPI communication					*/
#define	CMDERR_BP				5			/* Bit pos.: SPI invalid command received						*/
#define CMDERR_BM				0x0020		/* Bit mask: SPI invalid command received						*/
#define	FRMERR_BP				4			/* Bit pos.: Framing if SPI communication wrong					*/
#define	FRMERR_BM				0x0010		/* Bit mask: Framing if SPI communication wrong					*/
#define	P2RAMERR_BP				3			/* Bit pos.: ECC has detected 2 uncorrectable errors in P2RAM	*/
#define	P2RAMERR_BM				0x0008		/* Bit mask: ECC has detected 2 uncorrectable errors in P2RAM	*/
#define	P2RAMWRN_BP				2			/* Bit pos.: ECC is correcting 1 bit of P2RAM in customer area	*/
#define	P2RAMWRN_BM				0x0003		/* Bit mask: ECC is correcting 1 bit of P2RAM in customer area	*/
#define	MAGHALF_BP				1			/* Bit pos.: AGC Value reaches 255 LSB and magnitude is half	*/
#define	MAGHALF_BM				0x0002		/* Bit mask: AGC Value reaches 255 LSB and magnitude is half	*/
#define	AGCWARN_BP				0			/* Bit pos.: Agc-warning										*/
#define AGCWARN_BM				0x0001		/* Bit mask: Agc-warning										*/

/*---- PROG (0x0003) ---------------------------------------------------------------------------------------*/

#define	PROGVER_BP				6			/* Bit pos.: Program verify										*/
#define	PROGVER_BM				0x0040		/* Bit mask: Program verify										*/
#define	PROGOTP_BP				3			/* Bit pos.: Start OTP programming cycle						*/
#define	PROGOTP_BM				0x0008		/* Bit mask: Start OTP programming cycle						*/
#define	OTPREF_BP				2			/* Bit pos.: Refreshes non-volatile memory content from OTP		*/
#define	OTPREF_BM				0x0004		/* Bit mask: Refreshes non-volatile memory content from OTP		*/
#define	PROGEN_BP				0			/* Bit pos.: Program OTP enable									*/
#define	PROGEN_BM				0x0001		/* Bit mask: Program OTP enable									*/

/*---- DIA (0x3FF5) ----------------------------------------------------------------------------------------*/

#define	SPI_CNT_BP				11			/* Bit pos.: SPI frame counter									*/
#define	SPI_CNT_BM				0x0600		/* Bit mask: SPI frame counter									*/
#define	AGC_FIN_BP				9			/* Bit pos.: Initial AGC settling finished						*/
#define	AGC_FIN_BM				0x0200		/* Bit mask: Initial AGC settling finished						*/
#define	OFF_COMP_BP				8			/* Bit pos.: Error flag offset compensation finished			*/
#define	OFF_COMP_BM				0x0100		/* Bit mask: Error flag offset compensation finished			*/
#define	SINOFF_FIN_BP			7			/* Bit pos.: Sine offset compensation finished					*/
#define	SINOFF_FIN_BM			0x0080		/* Bit mask: Sine offset compensation finished					*/
#define	COSOFF_FIN_BP			6			/* Bit pos.: Cosine offset compensation finished				*/
#define	COSOFF_FIN_BM			0x0040		/* Bit mask: Cosine offset compensation finished				*/
#define	MAGHALF_FL_BP			5			/* Bit pos.: Error flag magnitude is below half of target value	*/
#define	MAGHALF_FL_BM			0x0020		/* Bit mask: Error flag magnitude is below half of target value	*/
#define COMP_H_BP				4			/* Bit pos.: Warning flag AGC high								*/
#define	COMP_H_BM				0x0010		/* Bit mask: Warning flag AGC high								*/
#define	COMP_I_BP				3			/* Bit pos.: Warning flag AGC low								*/
#define	COMP_I_BM				0x0008		/* Bit mask: Warning flag AGC low								*/
#define	CORDIC_OVF_BP			2			/* Bit pos.: Error flag CORDIC overflow							*/
#define	CORDIC_OVF_BM			0x0004		/* Bit mask: Error flag CORDIC overflow							*/
#define	LOOPS_FIN_BP			1			/* Bit pos.: All Magneto Core loops finished					*/
#define	LOOPS_FIN_BM			0x0002		/* Bit mask: All Magneto Core loops finished					*/
#define	VDD_MODE_BP				0			/* Bit pos.: VDD supply mode (0: 3.3 V, 1: 5.0 V)				*/
#define	VDD_MODE_BM				0x0001		/* Bit pos.: VDD supply mode (0: 3.3 V, 1: 5.0 V)				*/

/*---- DISABLE (0x0015) ------------------------------------------------------------------------------------*/

#define	UVW_OFF_BP				0			/* Bit pos.: Switch UVW output off (1)							*/
#define	UVW_OFF_BM				0x0001		/* Bit mask: Switch UVW output off (1)							*/
#define	ABI_OFF_BP				1			/* Bit pos.: Switch ABI output off (1)							*/
#define	ABI_OFF_BM				0x0002		/* Bit mask: Switch ABI output off (1)							*/
#define	FILTER_DS_BP			6			/* Bit pos.: Filter disabled (1)								*/
#define	FILTER_DS_BM			0x0020		/* Bit mask: Filter disabled (1)								*/

/*---- ZPOSM (0x0016) --------------------------------------------------------------------------------------*/

#define	ZPOSM_BP				0			/* Bit pos.: 8 most significant bits of the zero position		*/
#define	ZPOSM_BM				0x00FF		/* Bit mask: 8 most significant bits of the zero position		*/

/*---- ZPOSL (0x0017) --------------------------------------------------------------------------------------*/

#define	ZPOSL_BP				0			/* Bit pos.: 6 least significant bits of the zero position		*/
#define	ZPOSL_BM				0x003F		/* Bit mask: 6 least significant bits of the zero position		*/

/*---- SETTINGS1 (0x0018) ----------------------------------------------------------------------------------*/

#define	K_MAX_BP				0			/* Bit pos.: K max for adaptive filter setting					*/
#define	K_MAX_BM				0x0007		/* Bit mask: K max for adaptive filter setting					*/
#define	K_MIN_BP				3			/* Bit pos.: K min for adaptive filter setting					*/
#define	K_MIN_BM				0x0038		/* Bit mask: K min for adaptive filter setting					*/

/*---- SETTINGS2 (0x0019) ----------------------------------------------------------------------------------*/

#define	IWIDTH_BP				0			/* Bit pos.: Index width (0: 3 pulses, 1: 1 pulse)				*/
#define	IWIDTH_BM				0x0001		/* Bit mask: Index width (0: 3 pulses, 1: 1 pulse)				*/
#define	NOISESET_BP				1			/* Bit pos.: Noise setting for 3.3V operation at 150°C			*/
#define	NOISESET_BM				0x0002		/* Bit mask: Noise setting for 3.3V operation at 150°C			*/
#define	DIR_BP					2			/* Bit pos.: Rotation direction									*/
#define	DIR_BM					0x0004		/* Bit mask: Rotation direction									*/
#define	UVW_ABI_BP				3			/* Bit pos.: Defines the PWM output	(0:ABI W=PWM, 1:UVW I=PWM)	*/
#define	UVW_ABI_BM				0x0008		/* Bit mask: Defines the PWM output	(0:ABI W=PWM, 1:UVW I=PWM)	*/
#define	DAECDIS_BP				4			/* Bit pos.: Disable DAE compensation (0:ON, 1:OFF)				*/
#define	DAECDIS_BM				0x0010		/* Bit mask: Disable DAE compensation (0:ON, 1:OFF)				*/
#define	ABI_DEC_BP				5			/* Bit pos.: ABI setting to decimal count						*/
#define	ABI_DEC_BM				0x0020		/* Bit mask: ABI setting to decimal count						*/
#define	DATA_SEL_BP				7			/* Bit pos.: Data in address 0x3FFF (0: ANGLECOM 1: ANGLEUNC)	*/
#define	DATA_SEL_BM				0x0040		/* Bit mask: Data in address 0x3FFF (0: ANGLECOM 1: ANGLEUNC)	*/
#define	PWMON_BP				8			/* Bit pos.: Enables PWM										*/
#define	PWMON_BM				0x0080		/* Bit mask: Enables PWM										*/

/*---- SETTINGS3 (0x001A) ----------------------------------------------------------------------------------*/

#define	UVWPP_BP				0			/* Bit pos.: UVW number of pole pairs							*/
#define	UVWPP_BM				0x0007		/* Bit mask: UVW number of pole pairs							*/
#define	HYS_BP					3			/* Bit pos.: Hysteresis											*/
#define	HYS_BM					0x0038		/* Bit mask: Hysteresis											*/
#define	ABIRES_BP				5			/* Bit pos.: Resolution of ABI									*/
#define	ABIRES_BM				0x00E0		/* Bit mask: Resolution of ABI									*/

/*---- ECC (0x001B) ----------------------------------------------------------------------------------------*/

#define	ECC_CHSUM_BP			0			/* Bit pos.: ECC checksum										*/
#define	ECC_CHSUM_BM			0x007F		/* Bit mask: ECC checksum										*/
#define	ECC_EN_BP				7			/* Bit pos.: Enables ECC										*/
#define	ECC_EN_BM				0x0080		/* Bit mask: Enables ECC										*/

#define ABIRES_4096				0x04		/* ABI Resolution identifier									*/
#define ABIRES_2048				0x03		/* ABI Resolution identifier									*/
#define ABIRES_1024				0x00		/* ABI Resolution identifier									*/
#define ABIRES_512				0x01		/* ABI Resolution identifier									*/
#define ABIRES_256				0x02		/* ABI Resolution identifier									*/
#define ABIRES_D_1000			0x00		/* ABI Resolution identifier (decimal)							*/
#define ABIRES_D_500			0x01		/* ABI Resolution identifier (decimal)							*/
#define ABIRES_D_400			0x02		/* ABI Resolution identifier (decimal)							*/
#define ABIRES_D_300			0x03		/* ABI Resolution identifier (decimal)							*/
#define ABIRES_D_200			0x04		/* ABI Resolution identifier (decimal)							*/
#define ABIRES_D_100			0x05		/* ABI Resolution identifier (decimal)							*/
#define ABIRES_D_50				0x06		/* ABI Resolution identifier (decimal)							*/
#define ABIRES_D_25				0x07		/* ABI Resolution identifier (decimal)							*/


#define	CRC_SETTINGS2			0xC9		/* CRC for Writing Custom setting register 2					*/
#define	CRC_SETTINGS3			0xEE		/* CRC for Writing Custom setting register 3					*/

#define CRC_ABI_DEC				0x76		/* CRC for writing ABI setting to decimal count					*/

#define CRC_ABIRES_4096			0xD7		/* CRC for writing ABI Resolution identifier					*/
#define CRC_ABIRES_2048			0x65		/* CRC for writing ABI Resolution identifier					*/
#define CRC_ABIRES_1024			0xF1		/* CRC for writing ABI Resolution identifier					*/
#define CRC_ABIRES_512			0x76		/* CRC for writing ABI Resolution identifier					*/
#define CRC_ABIRES_256			0xE2		/* CRC for writing ABI Resolution identifier					*/
#define CRC_ABIRES_D_1000		0xF1		/* CRC for writing ABI Resolution identifier (decimal)			*/
#define CRC_ABIRES_D_500		0x76		/* CRC for writing ABI Resolution identifier (decimal)			*/
#define CRC_ABIRES_D_400		0xE2		/* CRC for writing ABI Resolution identifier (decimal)			*/
#define CRC_ABIRES_D_300		0x65		/* CRC for writing ABI Resolution identifier (decimal)			*/
#define CRC_ABIRES_D_200		0xD7		/* CRC for writing ABI Resolution identifier (decimal)			*/
#define CRC_ABIRES_D_100		0x50		/* CRC for writing ABI Resolution identifier (decimal)			*/
#define CRC_ABIRES_D_50			0xC4		/* CRC for writing ABI Resolution identifier (decimal)			*/
#define CRC_ABIRES_D_25			0x43		/* CRC for writing ABI Resolution identifier (decimal)			*/


/***** Function Prototypes **********************************************************************************/


/************************************************************************************************************/

#endif /* AS5047U_H */
