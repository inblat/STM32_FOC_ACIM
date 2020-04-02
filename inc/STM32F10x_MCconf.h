/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : STM32F10x_MCconf.h
* Author             : IMS Systems Lab 
* Date First Issued  : 11/28/2007
* Description        : Motor Control Library configuration file.
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
#ifndef __STM32F10x_MCCONF_H
#define __STM32F10x_MCCONF_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/************** Current sensing by ICS (Isolated current sensors) ************/
//#define ICS_SENSORS

/************** Current sensing by Three Shunt resistors *********************/ 
#define THREE_SHUNT

/************** Current sensing by Single Shunt resistor *********************/
//#define SINGLE_SHUNT

/************** Position sensing by Incremental encoder **********************/
//#define ENCODER

/*********************** Speed sensing by Tachometer  ************************/
#define TACHO

/******************** PIs  Differential term enabling ***********************/
//#define DIFFERENTIAL_TERM_ENABLED   

/********************** PIDs Parameter regulation software ********************/
//#define PIDs_TUNING

/******************* Enable DAC function for debugging purposes ***************/
//#define DAC_FUNCTIONALITY

/* Check-up of the configuration validity*/
#if ( (defined(ICS_SENSORS)) && (defined(THREE_SHUNT)) )
#error "Invalid setup: Two sampling techniques selected"
#endif

#if ( (defined(ICS_SENSORS)) && (defined(SINGLE_SHUNT)) )
#error "Invalid configuration: Two current sampling techniques selected"
#endif

#if ( (defined(SINGLE_SHUNT)) && (defined(THREE_SHUNT)) )
#error "Invalid configuration: Two current sampling techniques selected"
#endif

#if ( (!defined(ICS_SENSORS)) && (!defined(THREE_SHUNT)) && (!defined(SINGLE_SHUNT)) )
#error "Invalid setup: No sampling technique selected"
#endif

#if ( (defined(ENCODER)) && (defined (TACHO)) )
#error "Invalid setup: Two position sensing techniques selected"
#endif

#if ( (!defined(ENCODER)) && (!defined (TACHO)) )
#error "Invalid setup: No position sensing technique selected"
#endif

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __STM32F10x_MCCONF_H */
/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
