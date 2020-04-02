/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : stm32f10x_dbg.h
* Author             : MCD Application Team
* Date First Issued  : 
* Description        : This file contains all the DBG register's definitions
*                      and memory mapping.
********************************************************************************
* History:
* 
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
#ifndef __STM32F10x_DBG_H
#define __STM32F10x_DBG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  vu32 IDCODE;
  vu32 CR;	
}DBG_TypeDef;

/* Exported constants --------------------------------------------------------*/
/* DBG registers base address */
#define DBG_BASE               ((u32)0xE0042000)

#define DBG_SLEEP              ((u32)0x00000001)
#define DBG_STOP               ((u32)0x00000002)
#define DBG_STANDBY            ((u32)0x00000004)
#define DBG_IWDG_STOP          ((u32)0x00000100)
#define DBG_WWDG_STOP          ((u32)0x00000200)
#define DBG_TIM1_STOP          ((u32)0x00000400)
#define DBG_TIM2_STOP          ((u32)0x00000800)
#define DBG_TIM3_STOP          ((u32)0x00001000)
#define DBG_TIM4_STOP          ((u32)0x00002000)
#define DBG_CAN_STOP           ((u32)0x00004000)

#define DBG                    ((DBG_TypeDef *) DBG_BASE)

/*#ifndef DEBUG
  #define DBG                  ((DBG_TypeDef *) DBG_BASE)
#else   
  EXT DBG_TypeDef             *DBG;
  DBG = (DBG_TypeDef *) DBG_BASE;
#endif*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __STM32F10x_DBG_H */

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
