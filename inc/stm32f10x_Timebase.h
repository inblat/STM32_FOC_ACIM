/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : stm32f10x_Timebase.h
* Author             : IMS Systems Lab 
* Date First Issued  : 11/28/2007
* Description        : Contains the prototypes of the time base module related
*                      functions.
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


#ifndef __STM32F10x_TIMEBASE_H
#define __STM32F10x_TIMEBASE_H

void TB_Init(void);
void TB_Wait(u16);
void TB_Set_Delay_500us(u16);
bool TB_Delay_IsElapsed(void);
void TB_Set_DisplayDelay_500us(u16);
bool TB_DisplayDelay_IsElapsed(void);
void TB_Set_DebounceDelay_500us(u8);
bool TB_DebounceDelay_IsElapsed(void);
bool TB_StartUp_Timeout_IsElapsed(void);
void TB_Set_StartUp_Timeout(u16);

#endif //__STM32F10x_TIMEBASE_H

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
