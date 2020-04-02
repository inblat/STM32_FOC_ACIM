/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : MC_MotorControl_Layer.h
* Author             : IMS Systems Lab
* Date First Issued  : 11/28/2007
* Description        : Export of public functions of Motor control layer 
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
#ifndef __MC_MOTORCONTROLLAYER_H
#define __MC_MOTORCONTROLLAYER_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

extern u16 h_ADCBusvolt;
extern u16 h_ADCTemp;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void MCL_Init(void);
void MCL_ChkPowerStage(void);
bool MCL_ClearFault(void);
void MCL_SetFault(u16);
//void MISC_BusVoltage_sense_ADC_Init(void);
void MCL_BusVoltage_sense_ADC_Init(void);
//bool MISC_Chk_OverTemp(void);
bool MCL_Chk_OverTemp(void);
//BusV_t MISC_Chk_BusVolt(void);
BusV_t MCL_Chk_BusVolt(void);
//u16 MISC_Compute_BusVolt(void);
u16 MCL_Compute_BusVolt(void);
//u8 MISC_Compute_Temp(void);
u8 MCL_Compute_Temp(void);
void MCL_Init_Arrays(void);

#endif //__MC_MOTORCONTROLLAYER_H
/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
