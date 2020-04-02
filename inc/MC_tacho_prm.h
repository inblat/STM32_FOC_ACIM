/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : MC_tacho_prm.h
* Author             : IMS Systems Lab 
* Date First Issued  : 11/28/2007
* Description        : Contains the list of project specific parameters related
*                      to the tachogenerator speed feedback.
********************************************************************************
* History:
* 11/28/07 v1.0
* 05/29/08 v2.0
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TACHO_PRM_H
#define __TACHO_PRM_H

/*----------------------------------------------------------------------------*/
/* APPLICATION SPECIFIC DEFINE -----------------------------------------------*/
/*----------------------------------------------------------------------------*/

/* PERIPHERAL SET-UP ---------------------------------------------------------*/

// Define here the 16-bit timer chosen to handle tacho feedback 
#define TIMER2_HANDLES_TACHO
//#define TIMER3_HANDLES_TACHO
//#define TIMER4_HANDLES_TACHO

// Define here the input capture chosen to handle tacho feedback 
#define TACHO_INPUT_TI1     // Input Capture 1 (TAC_TachoTimerInit assignment)
//#define TACHO_INPUT_TI2   // Input Capture 2 (TAC_TachoTimerInit assignment)

/////////////////////////////////////////////////////////////////////////
// Not to be uncommented with the following TACHO_TIMER configuration  //
/////////////////////////////////////////////////////////////////////////
//#define TACHO_INPUT_TI3   // Input Capture 3 (TAC_TachoTimerInit assignment)
//#define TACHO_INPUT_TI4   // Input Capture 4 (TAC_TachoTimerInit assignment)

/* TACHOGENERATOR PARAMETERS -------------------------------------------------*/

/* Number of pulses per revolution given by tachogenerator */
#define	TACHO_PULSE_PER_REV	((u8)8)

/* This is to validate speed feedback with a pulse generator */
//#define TACHO_PULSE_PER_REV	((u8)1)

/* APPLICATION SPEED DOMAIN AND ERROR/RANGE CHECKING -------------------------*/

/* Define here the frequency above which speed feedback is not realistic
in the application: this allows to discriminate glitches for instance */

#define MAX_SPEED_FDBK          ((u16)6400) // Unit is 0.1Hz

// With rad/PWM period unit (here 2*PI rad = 0xFFFF):
//#define MAX_PSEUDO_SPEED_FDBK   ((u16)(0x10000uL * MAX_SPEED_FDBK) / (SAMPLING_FREQ * 10))

/* Define here the returned value if measured speed is > MAX_SPEED_FDBK).
It could be 0 or FFFF depending on upper layer software management */

#define MAX_SPEED               ((u16)6400) // Unit is 0.1Hz

// With rad/PWM period unit (here 2*PI rad = 0xFFFF):

#define MAX_PSEUDO_SPEED        ((u16)9999)

/* Define here the frequency below which speed feedback is not realistic
in the application: this allows to discriminate too low freq for instance */

#define MIN_SPEED_FDBK          ((u16)10) // Unit is 0.1Hz

/* Max TIM prescaler ratio defining the lowest expected speed feedback */

#define MAX_RATIO		((u16)400u)

/* Number of consecutive timer overflows without capture: this can indicate
that informations are lost or that speed is decreasing very sharply */
/* This is needed to implement tacho time-out. This duration depends on tacho
timer pre-scaler, which is variable; the time-out will be higher at low speed*/

#define MAX_OVERFLOWS       ((u16)10)

/* Lowest speed with decreasing speed, is:
(MTC_CLOCK / (MAX_RATIO+1))/0xFFFF (here 2.5Hz) */
/* NB: if speed is maintain between this threshold and MIN_SPEED_FDBK, then the
speed will finally be zero when prescaler will reach MAX_RATIO */

/* Lowest speed with increasing speed (at start), is:
(MTC_CLOCK / (MAX_RATIO+1))/LOW_RES_THRESHOLD (here 7.4Hz) */

/* ROLLING AVERAGE DEPTH -----------------------------------------------------*/

#define SPEED_FIFO_SIZE 	((u8)4)


/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __TACHO_PRM_H */
/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
