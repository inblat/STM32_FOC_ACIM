/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : MC_IFOC_Drive.h
* Author             : IMS Systems Lab
* Date First Issued  : 11/28/2007
* Description        : Contains the prototypes for the AC IFOC-drive module 
*                      related functions.
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
#ifndef __MC_IFOC_DRIVE_H
#define __MC_IFOC_DRIVE_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

void IFOC_Model(void);
void IFOC_CalcFluxTorqueRef(void);
void IFOC_Init(void);

void FOC_Model_OL(void);

#endif /* __MC_IFOC_DRIVE_H */
/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
