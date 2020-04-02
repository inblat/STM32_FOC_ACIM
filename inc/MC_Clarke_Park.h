/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : MC_Clarke_Park.h
* Author             : IMS Systems Lab 
* Date First Issued  : 11/28/2007
* Description        : This module implements the reference frame transformations
*                      needed for vector control: Clarke, Park and Reverse Park.
*                      It also performs the voltage circle limitation.
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
#ifndef __MC_CLARKE_PARK_H
#define __MC_CLARKE_PARK_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "MC_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

Curr_Components Clarke(Curr_Components);
Curr_Components Park(Curr_Components,s16);
void RevPark_Circle_Limitation(void);
Volt_Components Rev_Park(Volt_Components Volt_Input);

#endif //__MC_CLARKE_PARK_H
/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
