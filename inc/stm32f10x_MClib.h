/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : stm32f10x_MClib.h
* Author             : IMS Systems Lab 
* Date First Issued  : 11/28/2007
* Description        : This file gathers the motor control header files which 
*                      are needed depending on configuration.
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
#ifndef __STM32F10xMCLIB_H
#define __STM32F10xMCLIB_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f10x_MCconf.h"
#include "MC_type.h"
#include "stm32f10x_lib.h"

#ifdef TACHO
#include "stm32f10x_tacho.h"
#endif

#ifdef ENCODER
#include "stm32f10x_encoder.h"
#endif

#ifdef ICS_SENSORS
#include "stm32f10x_svpwm_ics.h"
#endif

#ifdef THREE_SHUNT
#include "stm32f10x_svpwm_3shunt.h"
#endif

#ifdef SINGLE_SHUNT
#include "stm32f10x_svpwm_1shunt.h"
#endif

#ifdef DAC_FUNCTIONALITY
#include "stm32f10x_MCdac.h"
#endif

#include "MC_Clarke_Park.h"
#include "MC_IFOC_Drive.h"
#include "MC_PID_regulators.h"
#include "MC_Control_Param.h"
#include "stm32f10x_Timebase.h"
#include "MC_Display.h"
#include "MC_Keys.h"
#include "stm32f10x_lcd.h"
#include "MC_MotorControl_Layer.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __STM32F10xMCLIB_H */
/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
