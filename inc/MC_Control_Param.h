/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : MC_Control_Param.h
* Author             : IMS Systems Lab 
* Date First Issued  : 11/28/2007
* Description        : This file gathers parameters related to:
*                      power devices, speed regulation frequency, PID controllers
*                      setpoints and constants, start-up ramp, lowest values for
*                      speed reading validation.
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
#ifndef __MC_CONTROL_PARAM_H
#define __MC_CONTROL_PARAM_H

/* Includes ------------------------------------------------------------------*/
#include "MC_acmotor_prm.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/*********************** POWER DEVICES PARAMETERS ******************************/

/****	Power devices switching frequency  ****/
#define PWM_FREQ ((u16) 14400)          // in Hz  (N.b.: pattern type is center aligned)

/****    Deadtime Value   ****/
#define DEADTIME_NS	((u16) 800)         //in nsec  

/****      Uncomment the Max modulation index     ****/ 
/**** corresponding to the selected PWM frequency ****/
//#define MAX_MODULATION_100_PER_CENT     // up to 11.4 kHz PWM frequency 
//#define MAX_MODULATION_99_PER_CENT      // up to 11.8 kHz
//#define MAX_MODULATION_98_PER_CENT      // up to 12.2 kHz  
//#define MAX_MODULATION_97_PER_CENT      // up to 12.9 kHz  
#define MAX_MODULATION_96_PER_CENT      // up to 14.4 kHz  
//#define MAX_MODULATION_95_PER_CENT      // up to 14.8 kHz
//#define MAX_MODULATION_94_PER_CENT      // up to 15.2 kHz  
//#define MAX_MODULATION_93_PER_CENT      // up to 16.7 kHz
//#define MAX_MODULATION_92_PER_CENT      // up to 17.1 kHz
//#define MAX_MODULATION_91_PER_CENT      // up to 17.5 kHz

/*********************** CURRENT REGULATION PARAMETERS *************************/

/****	ADC IRQ-HANDLER frequency, related to PWM  ****/
#define REP_RATE (1)  // (N.b): Internal current loop is performed every 
                      //             (REP_RATE + 1)/(2*PWM_FREQ) seconds.
                      // In case of three-shunt current reading:
		              // - REP_RATE must be an odd number 
		              // - REP_RATE must be higher than 1 for PWM_FREQ > 12500     
                      // These limitations don't apply to ICS

//Not to be modified
#define SAMPLING_FREQ   ((u16)PWM_FREQ/((REP_RATE+1)/2))   // Resolution: 1Hz

/********************** POWER BOARD PROTECTIONS THRESHOLDS ********************/

#define NTC_THRESHOLD      (u16)12800  // about 60°C on heatsink of MB459 board
#define NTC_HYSTERESIS     (u16)3200   // temperature hysteresis (4°C)

#define OVERVOLTAGE_THRESHOLD   (u16)28500  // max DC Bus voltage is 350V 
#define UNDERVOLTAGE_THRESHOLD  (u16)1130   // min DC Bus voltage is 19V

/*********************** SPEED LOOP SAMPLING TIME *****************************/
//Not to be modified
#define PID_SPEED_SAMPLING_500us      0     // min 500us
#define PID_SPEED_SAMPLING_1ms        1
#define PID_SPEED_SAMPLING_2ms        3     // (4-1)*500uS = 2ms
#define PID_SPEED_SAMPLING_5ms        9
#define PID_SPEED_SAMPLING_10ms       19
#define PID_SPEED_SAMPLING_127ms      255   // max (255-1)*500us = 127 ms

//User should make his choice here below
#define PID_SPEED_SAMPLING_TIME   (u8)(PID_SPEED_SAMPLING_2ms)

/******************** SPEED PID-CONTROLLER INIT VALUES************************/

/* default values for Speed control loop */

#define PID_SPEED_REFERENCE_RPM   	3000
#define PID_SPEED_KP_DEFAULT  		6000
#define PID_SPEED_KI_DEFAULT  		2000	
#define PID_SPEED_KD_DEFAULT  		10


/************** QUADRATURE CURRENTS PID-CONTROLLERS INIT VALUES **************/

#define PID_TORQUE_REFERENCE   8000   //(N.b.: that's the reference init value in 
                                      // open speed-loop)

#define PID_FLUX_REFERENCE     NOMINAL_FLUX

/* default values for Torque control loop */

#define PID_TORQUE_KP_DEFAULT  12000       
#define PID_TORQUE_KI_DEFAULT  150               
#define PID_TORQUE_KD_DEFAULT  0

/* default values for Flux control loop */

#define PID_FLUX_KP_DEFAULT  12000 
#define PID_FLUX_KI_DEFAULT  150 
#define PID_FLUX_KD_DEFAULT  0


/********************** START-UP TORQUE RAMP PARAMETERS **********************/

#define STARTUP_TIMEOUT       ((u16)3000)            //in msec
#define STARTUP_RAMP_DURATION ((u16)100)             //in msec
#define STARTUP_FINAL_TORQUE  ((s16)NOMINAL_TORQUE)  //in q15 format. It must be not 
                                                     //higher than htorque_reference[0]
                                                     //specified in MC_acmotor_prm.h

//Not to be modified
#define CL_STARTUP_TORQUE_INCREMENT ((u16)((s32)(STARTUP_FINAL_TORQUE)*16/(2*STARTUP_RAMP_DURATION)))

/*************** LOWEST SPEED FOR TACHOMETER RELIABILITY  *********************/
/*************** AND SPEED CLOSED LOOP VALIDATION *****************************/

#ifdef  TACHO
#define TACHO_SPEED_VAL ((u16) 8)  
#endif

/*************** ENCODER SPEED CLOSED LOOP VALIDATION  ************************/
//Not to be modified
#ifdef  ENCODER
#define ENCODER_CL_ENABLE ((u16) 1)
#endif

/*******           Ki, Kp, Kd COEFFICIENT CALCULATION       ********************/
/*******           	Closed loop operation		     *******************
		

              /|\               /
               |               /
  	       |	      /
               |             /
               |   _________/  
               |  /
               | /
	       |/_________________________\ 
	   Fmin   F_1      F_2  Fmax      /
				
		                                                                

We assume a linear variation of Ki, Kp, Kd coefficients following
the motor speed. 2 intermediate frequencies ar set (see definition here after)
and 3 terms (Ki,Kp,Kd) associated with Fmin, F_1, F_2, Fmax 
(total: 4+4+4 terms); following linear coefficients are used to compute each term.

Example: 

Fmin = 500  <->	50 Hz 	(reminder -> mechanical frequency with 0.1 Hz resolution!)
Ki_min = 20	Kp_min = 40       Kd_min = 500 

F_1 = 2000 <->	200 Hz 	
Ki_1 = 80	Kp_1 = 1000        Kd_1 = 260 

then:
alpha_Ki_1 = (Ki_1-Ki_Fmin)/(F_1-Fmin) = 60/1500 = 0.04
alpha_Kp_1 = (Kp_1-Kp_Fmin)/(F_1-Fmin) = 960/1500 = 0.64
alpha_Kd_1 = (Kd_1-Kd_Fmin)/(F_1-Fmin) = -240/1500 = -0.16

** Result **
From Freq_Min to F_1, Ki, Kp, Kd will then obey to:
Ki = Ki_Fmin + alpha_Ki_1*(Freq_motor-Freq_Min)
Kp = Kp_Fmin + alpha_Kp_1*(Freq_motor-Freq_Min)
Kd = Kd_Fmin + alpha_Kd_1*(Freq_motor-Freq_Min)

		                                                                
*********************************************************************************/
//Settings for min frequency
#define Freq_Min         10     // 1 Hz mechanical
#define Ki_Fmin          3000	// Frequency min coefficient settings
#define Kp_Fmin          2000
#define Kd_Fmin          3000

//Settings for intermediate frequency 1
#define F_1		 50    // 5 Hz mechanical 
#define Ki_F_1           2000      // Intermediate frequency 1 coefficient settings
#define Kp_F_1           1000
#define Kd_F_1           2500

//Settings for intermediate frequency 2
#define F_2		 200    // 20 Hz mechanical
#define Ki_F_2           1000     // Intermediate frequency 2 coefficient settings
#define Kp_F_2           750
#define Kd_F_2           1200
  
//Settings for max frequency
#define Freq_Max         833     // 83.3 Hz mechanical
#define Ki_Fmax          500      // Frequency max coefficient settings
#define Kp_Fmax          500
#define Kd_Fmax          500
      
                                                                             
/********************************************************************************/      
/* linear coefficients */                                                                             
#define alpha_Ki_1		(s32)( ((s32)((s16)Ki_F_1-(s16)Ki_Fmin)*1024) / (s32)(F_1-Freq_Min) )
#define alpha_Kp_1		(s32)( ((s32)((s16)Kp_F_1-(s16)Kp_Fmin)*1024) / (s32)(F_1-Freq_Min) )
#define alpha_Kd_1		(s32)( ((s32)((s16)Kd_F_1-(s16)Kd_Fmin)*1024) / (s32)(F_1-Freq_Min) )

#define alpha_Ki_2		(s32)( ((s32)((s16)Ki_F_2-(s16)Ki_F_1)*1024) / (s32)(F_2-F_1) )
#define alpha_Kp_2		(s32)( ((s32)((s16)Kp_F_2-(s16)Kp_F_1)*1024) / (s32)(F_2-F_1) )
#define alpha_Kd_2		(s32)( ((s32)((s16)Kd_F_2-(s16)Kd_F_1)*1024) / (s32)(F_2-F_1) )

#define alpha_Ki_3		(s32)( ((s32)((s16)Ki_Fmax-(s16)Ki_F_2)*1024) / (s32)(Freq_Max-F_2) )
#define alpha_Kp_3		(s32)( ((s32)((s16)Kp_Fmax-(s16)Kp_F_2)*1024) / (s32)(Freq_Max-F_2) )
#define alpha_Kd_3		(s32)( ((s32)((s16)Kd_Fmax-(s16)Kd_F_2)*1024) / (s32)(Freq_Max-F_2) )


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MC_CONTROL_PARAM_H */

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/


