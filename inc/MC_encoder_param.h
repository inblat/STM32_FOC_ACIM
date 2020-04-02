/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : MC_encoder_param.h
* Author             : IMS Systems Lab 
* Date First Issued  : 11/28/2007
* Description        : Contains the list of project specific parameters related
*                      to the encoder speed and position feedback.
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
#ifndef __MC_ENCODER_PARAM_H
#define __MC_ENCODER_PARAM_H

#include "STM32F10x_MCconf.h"

/* PERIPHERAL SET-UP ---------------------------------------------------------*/
#ifdef ENCODER

/* Define here the 16-bit timer chosen to handle encoder feedback */
#define TIMER2_HANDLES_ENCODER
//#define TIMER3_HANDLES_ENCODER
//#define TIMER4_HANDLES_ENCODER

#endif  // ENCODER


#if defined(TIMER2_HANDLES_ENCODER)
#define ENCODER_TIMER         TIM2          // Encoder unit connected to TIM2
#elif defined(TIMER3_HANDLES_ENCODER)
#define ENCODER_TIMER         TIM3          // Encoder unit connected to TIM3
#else // TIMER4_HANDLES_ENCODER
#define ENCODER_TIMER         TIM4          // Encoder unit connected to TIM4
#endif

/*****************************  Encoder settings ******************************/
#define ENCODER_PPR           (u16)(1024)   // number of pulses per revolution

/* Define here the absolute value of the application minimum and maximum speed 
                                                                in 0.1Hz unit*/
#define MINIMUM_MECHANICAL_SPEED  (u16)1    //0.1Hz
#define MAXIMUM_MECHANICAL_SPEED  (u16)6000 //600Hz

/* Define here the number of consecutive error measurement to be detected 
   before going into FAULT state */

#define MAXIMUM_ERROR_NUMBER (u8)25

/* Computation Parameter*/
//Number of averaged speed measurement
#define SPEED_BUFFER_SIZE     8   // power of 2 required to ease computations
#define SPEED_SAMPLING_TIME   PID_SPEED_SAMPLING_TIME

#endif  /*__MC_ENCODER_PARAM_H*/

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
