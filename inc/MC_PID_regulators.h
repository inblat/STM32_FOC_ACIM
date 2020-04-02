/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : MC_PID_regulators.h
* Author             : IMS Systems Lab
* Date First Issued  : 11/28/2007
* Description        : Contains the prototypes of PI(D) related functions.
 
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
 
#ifndef __PI_REGULATORS__H
#define __PI_REGULATORS__H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define   ANGULAR_VELOCITY_PWM_SAMPLING_PERIOD 50   // number of PWM update events
                                                    //  in between 2 speed measurements

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void PID_Init (PID_Struct_t *,PID_Struct_t *,PID_Struct_t *);
void PID_Speed_Coefficients_update(s16, PID_Struct_t *);
s16 PID_Regulator(s16, s16, PID_Struct_t *);

/* Exported variables ------------------------------------------------------- */

#endif 

/******************* (C) COPYRIGHT 2006 STMicroelectronics *****END OF FILE****/
