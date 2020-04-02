/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : stm32f10x_MCdac.h
* Author             : IMS Systems Lab 
* Date First Issued  : 07/19/07
* Description        : It contains prototypes and definitions necessary for 
*                      handling DAC functionality from LCD display and joystick
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
#ifndef __STM32F10x_MCDAC_H
#define __STM32F10x_MCDAC_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define I_A           (u8)(1)               
#define I_B           (u8)(2)
#define I_ALPHA       (u8)(3)
#define I_BETA        (u8)(4)
#define I_Q           (u8)(5)
#define I_D           (u8)(6)
#define I_Q_REF       (u8)(7)
#define I_D_REF       (u8)(8)
#define V_Q           (u8)(9)
#define V_D           (u8)(10)
#define V_ALPHA       (u8)(11)
#define V_BETA        (u8)(12)
#define SENS_ANGLE    (u8)(13)
#define SENS_SPEED    (u8)(14)
#define USER_1        (u8)(15)
#define USER_2        (u8)(16)
#define DAC_CH1       (u8)(1)
#define DAC_CH2       (u8)(2)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void MCDAC_Configuration(void);
void MCDAC_Update_Output(void);
void MCDAC_Update_Value(u8,s16);
void MCDAC_Output_Choice(s8,u8);
u8 *MCDAC_Output_Var_Name(u8);

#endif 
/* __STM32F10x_MCDAC_H */
/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
