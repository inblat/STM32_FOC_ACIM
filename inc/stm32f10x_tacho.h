/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : stm32f10x_tacho.h
* Author             : IMS Systems Lab 
* Date First Issued  : 11/28/2007
* Description        : Contains the prototypes of tacho related functions.
*
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
#ifndef __TACHO_H
#define __TACHO_H

/////////////////////// PWM Peripheral Input clock ////////////////////////////

#define CKTIM	((u32)72000000uL) 	/* Silicon running at 60MHz Resolution: 1Hz */

/* Includes ------------------------------------------------------------------*/
#include "MC_tacho_prm.h" // Tacho timer selected there: needed for prototypes

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void TAC_TachoTimerInit(void);
u16  TAC_GetRotorFreq (void);
u16  TAC_GetRotorFreqInHz (void);
void TAC_InitTachoMeasure(void);
void TAC_StartTachoFiltering(void);
bool TAC_ValidSpeedInfo(u16 hMinRotorFreq);
void TAC_ClrTimeOut(void);
bool TAC_IsTimedOut(void);
u16  TAC_GetCaptCounter(void);
void TAC_ClrCaptCounter(void);

#if defined(TIMER2_HANDLES_TACHO)
  void TIM2_IRQHandler(void);
#elif defined(TIMER3_HANDLES_TACHO)
  void TIM3_IRQHandler(void);
#else // TIMER4_HANDLES_TACHO
  void TIM4_IRQHandler(void);
#endif

#endif /* __TACHO_H */
/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
