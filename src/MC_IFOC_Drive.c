/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : MC_IFOC_Drive.c
* Author             : IMS Systems Lab & MCD Application Team
* Date First Issued  : 11/28/2007
* Description        : This file provides all the AC-IM IFOC drive functions.
* 
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_MClib.h"
#include "MC_globals.h"
#include "MC_const.h"
#include "MC_IFOC_Drive.h"
#include "MC_acmotor_prm.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define EUL_SENS      2
#define qK_EULER ((s16)(32767*EUL_SENS /(SAMPLING_FREQ*ROTOR_TIME_CONSTANT* 0.000001)))

#define K_RAD_PULSE (s32)(((65536/(2 * PI))/(ROTOR_TIME_CONSTANT * 0.000001 *SAMPLING_FREQ))*65536)
#define K_RAD_PULSE_HI (s16)(K_RAD_PULSE / 65536)
#define K_RAD_PULSE_LOW (u16)(K_RAD_PULSE & 0x0000FFFF)

#define K_RAD_HZ (s32)((65536*10)/(2 * PI * ROTOR_TIME_CONSTANT * 0.000001))  //Hz*10
#define K_RAD_HZ_HI (s16)(K_RAD_HZ / 65536)
#define K_RAD_HZ_LOW (u16)(K_RAD_HZ & 0x0000FFFF)

/* Private macro -------------------------------------------------------------*/

#ifdef ICS_SENSORS
#define GET_PHASE_CURRENTS SVPWM_IcsGetPhaseCurrentValues
#define CALC_SVPWM SVPWM_IcsCalcDutyCycles
#endif

#ifdef THREE_SHUNT
#define GET_PHASE_CURRENTS SVPWM_3ShuntGetPhaseCurrentValues
#define CALC_SVPWM SVPWM_3ShuntCalcDutyCycles
#endif

#ifdef SINGLE_SHUNT
#define GET_PHASE_CURRENTS SVPWM_1ShuntGetPhaseCurrentValues
#define CALC_SVPWM SVPWM_1ShuntCalcDutyCycles
#endif

const FLUX_REFERENCE_TABLE;
const TORQUE_REFERENCE_TABLE;


/* Private variables ---------------------------------------------------------*/
static s32 wIm = ((K_RAD_PULSE_HI + 1)*65536) ;
static s16 hRotFlx_Theta = 0;        // Rotor flux angle
#if defined (ENCODER)
static s32 wRotFlx_SlipTheta = 0;    // Rotor flux slip angle
#endif

/* Private function prototypes -----------------------------------------------*/
s16 CalcIm(s16);
s32 CalcRotFlxSlipFreq(s16,s16);
void div_q31_q15_q15(s16 LeftOpMsb, u16 LeftOpLsb, s16 RightOp, s16 *Output);
void mul_q15_q15_q31(s16 Op1, s16 Op2, s32 *Out);
/*******************************************************************************
* Function Name : IFOC_Model
* Description   : The purpose of this function is to perform AC-IM torque and 
*                 flux regulation, implementing the IFOC vector algorithm.
* Input         : None.
* Output        : None.
* Return        : None.
*******************************************************************************/

void IFOC_Model(void)
{
   
  s32 wRotFlx_SlipFreq;                // Rotor flux slip frequency

  s16 hIm;                             // Im = (Rotor Flux)/Lm  
      
  //GPIOC->ODR ^= 0x40;  // Toggle PC6
  
  /*loads stator currents Ias and Ibs, read by ICS or shunt resistors*/
  Stat_Curr_a_b = GET_PHASE_CURRENTS(); 
  
  
   /*Performs the Clarke transformation,
  i.e. transforms stator currents Ias and Ibs into currents Ialpha and Ibeta*/
  Stat_Curr_alfa_beta= Clarke(Stat_Curr_a_b);
  
  /*Performs the Park transformation,
  i.e transforms stator currents Ialpha and Ibeta into Iqs and Ids on a 
  reference frame synchronous with the rotor flux*/
  Stat_Curr_q_d= Park(Stat_Curr_alfa_beta,(s16)(hRotFlx_Theta));  
  
  /*** use read currents to calculate the Rotor flux slip frequency **/
 
  /*given Ids, computes Im [1.15], i.e. (Rotor flux)/Lm */
  hIm = CalcIm(Stat_Curr_q_d.qI_Component2);   
  
  /*given Iqs and Im, computes hRotFlx_SlipFreq [1.15],
  i.e. the Rotor flux slip frequency*/
  wRotFlx_SlipFreq = CalcRotFlxSlipFreq(Stat_Curr_q_d.qI_Component1,hIm); 
  
#if defined (ENCODER)
  
  wRotFlx_SlipTheta += wRotFlx_SlipFreq ;
  hRotFlx_Theta = (s16)(wRotFlx_SlipTheta/65536) + ENC_Get_Electrical_Angle() ;
  
#elif defined (TACHO)
  
  if (State == START)
  {
   hRotFlx_Theta += (s16)(wRotFlx_SlipFreq/65536);
  }
  else //State =RUN
  {
    
    hRotFlx_Theta += (s16)(wRotFlx_SlipFreq/65536) + TAC_GetRotorFreq();
  }
#endif    

  /*loads the Torque Regulator output reference voltage Vqs*/
  Stat_Volt_q_d.qV_Component1 = PID_Regulator(hTorque_Reference, 
  						Stat_Curr_q_d.qI_Component1, &PID_Torque_InitStructure);

  
  /*loads the Flux Regulator output reference voltage Vds*/
  Stat_Volt_q_d.qV_Component2 = PID_Regulator(hFlux_Reference, 
                          Stat_Curr_q_d.qI_Component2, &PID_Flux_InitStructure);  
  
  RevPark_Circle_Limitation();
  
  /*Performs the Reverse Park transformation,
  i.e transforms stator voltages Vqs and Vds into Valpha and Vbeta on a 
  stationary reference frame*/
  
  Stat_Volt_alfa_beta = Rev_Park(Stat_Volt_q_d);
  
  /*Valpha and Vbeta finally drive the power stage*/ 
  CALC_SVPWM(Stat_Volt_alfa_beta);
}

/*******************************************************************************
* Function Name   : IFOC_CalcFluxTorqueRef
* Description     : This function provides current components Iqs* and Ids* to be
*                   used as reference values (by the IFOC_Model function) in 
*                   closed-loop speed mode
* Input           : None.
* Output          : None.
* Return          : None.
*******************************************************************************/

void IFOC_CalcFluxTorqueRef(void)
{
  s32 wRotFlxSlipFreq;
  s16 hTempA;
  s16 hTempB;
  s16 hwe;  //stator generated frequency (pulses/pwm period)
  s16 hTorqueRefMax;
  
  hTempA = hSpeed_Reference;
  
  hTempB = PID_Regulator(hTempA, hRot_Freq_Hz, &PID_Speed_InitStructure);
      
  div_q31_q15_q15(K_RAD_HZ_HI,K_RAD_HZ_LOW,hFlux_Reference,&hTempA);
    
  mul_q15_q15_q31(hTempB,hTempA,&wRotFlxSlipFreq);
    
  hwe = (s16)(wRotFlxSlipFreq/65536 + hRot_Freq_Hz); //u16 to have absolute value of we
    
  hwe = (hwe < 0 ? -hwe : hwe);
    
  if (hwe > RATED_FREQ)
  {
    u8 bspeed_index;
      
    s32 wtempindex;
      
    if (hwe > MAX_FREQ)
    {
      hwe = MAX_FREQ;
    }
      
    wtempindex = (hwe - RATED_FREQ) * 255; 
      
    bspeed_index = (u8)(wtempindex / MAX_FREQ_INCR); 
      
    hFlux_Reference = hflux_reference[bspeed_index];
      
    hTorqueRefMax = htorque_reference[bspeed_index];
              
  }
  else
  {
    hFlux_Reference = hflux_reference[0];
      
    hTorqueRefMax = htorque_reference[0];
  }
    
  if (hTempB > hTorqueRefMax)
  {
    hTorque_Reference = hTorqueRefMax;
  }
  else  if ( hTempB < -hTorqueRefMax)
        {
          hTorque_Reference = -hTorqueRefMax;
        }
        else
        {
          hTorque_Reference = hTempB;
        }
  
  
}

/*******************************************************************************
* ROUTINE Name   : CalcIm
* Description    : The purpose of this routine is to supply the estimated value
*                  of the rotor flux, as a response to variations of the input
*                  current value Ids
* Input          : Stator current Ids (on the Rotor Flux rotating frame).
* Output         : None.
* Return         : Rotor Flux divided by Lm (Magnetizing inductance). 
*******************************************************************************/
s16 CalcIm(s16 hId_input)
{
  s32 wTemp1;
  s32 wTemp2;
  s16 hIm;
  
  hIm = ((s16)(wIm / 65536));
  
  mul_q15_q15_q31(qK_EULER,hId_input,&wTemp1);
  mul_q15_q15_q31(qK_EULER,hIm,&wTemp2);
  wTemp1 -= wTemp2;
  
  wTemp1 /= EUL_SENS;
  wIm += wTemp1;
 
  hIm = ((s16)(wIm / 65536));
  return (hIm);
    
}

/*******************************************************************************
* ROUTINE Name   : CalcRotFlxSlipFreq
* Description    : This function estimates the rotor flux slip frequency.
* Input          : Stator current Iqs, Rotor Flux divided by Lm (both on the  
*                  Rotor Flux rotating frame).
* Output         : None.
* Return         : Rotor Flux slip frequency,
*                  i.e Rotor Flux frequency - Rotor electrical frequency.
*******************************************************************************/
s32 CalcRotFlxSlipFreq(s16 hIq_input,s16 hIm_input)
{
  s32 wTempA;
  s16 hTemp1;
    
  div_q31_q15_q15(K_RAD_PULSE_HI,K_RAD_PULSE_LOW,hIm_input,&hTemp1);
  
  mul_q15_q15_q31(hIq_input,hTemp1,&wTempA);

  return (wTempA);
}

/*******************************************************************************
* ROUTINE Name   : IFOC_Init
* Description    : Performs the initialization of the variables interested in  
*                  IFOC algorithm
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void IFOC_Init(void)
{
  wIm = ((K_RAD_PULSE_HI + 1)*65536);
  hRotFlx_Theta = 0;        // Rotor flux angle
#if defined (ENCODER)
  wRotFlx_SlipTheta = 0;  // Rotor flux slip angle
#endif
  
  //It applies 50% duty cycle on the output
  Stat_Volt_alfa_beta.qV_Component1=0;
  Stat_Volt_alfa_beta.qV_Component2=0;
  CALC_SVPWM(Stat_Volt_alfa_beta);
}

/*******************************************************************************
* Function Name  : div_q31_q15_q15
* Description    : This function divides a q1.31 by a q1.15 variable. Its result 
*                  is a q1.15 value.
* Input          : Operand 1 (dividend) MSB, Operand 1 (dividend) LSB,
*                  Operand 2 (divisor)
* Output         : Op1 / Op2, q1.15 format
* Return         : none.
*******************************************************************************/
void div_q31_q15_q15(s16 LeftOpMsb, u16 LeftOpLsb, s16 RightOp, s16 *Output)
{
 long aux1, aux2;
 long temp1, temp2;
 short nb_bits, i, CS, test;

//aux1 = (s32)LeftOpMsb<<16;  /* creating a long to provide*/
 
  aux1 = (s32)(LeftOpMsb*65536);  /* creating a long to provide*/
 
 
 
 aux1 += (unsigned)LeftOpLsb; /* a 32 bits dividende */

 if( RightOp==0 )
 {
   if( aux1<0 )
   {
     *Output = 0x8000;
   }
   if( aux1>=0 )
   {
     *Output = 0x7fff;
   }
 }
 else
 {
  
   
   aux2 = (s32)RightOp*65536;    /* divisor  */
   
   
   temp1 = aux1&0x80000000;
   temp2 = aux2&0x80000000;

   if (temp1==temp2)
   {
     CS=0;		
     test=0;
   }
   else
   {
     CS=1;	/* CS=1 for different signs */
     test=1;
   }

   aux1 = aux1*2;
   
   aux1 += test;

   nb_bits=15;

   for (i=0; i<nb_bits; i++)
   {
     if (CS==1)
     {      
       aux1 += aux2;
     }
     else
     {      
       aux1 -= aux2;
     }

     temp1 = aux1&0x80000000;
     /*     temp2 = aux2&0x80000000;  */    /* temp2 has not been modified  */
     if ( temp1==temp2 )
     {
       CS=0;
       test=1;	// shift !CS into partial remainder
     }
     else
     {
       CS=1;
       test=0;
     }
     
     aux1 = (aux1*2);
     aux1 += test;
   }

   *Output = (s16)aux1;
 }	
}

/*******************************************************************************
* Function Name  : mul_q15_q15_q31
* Description    : This function multiplies two q1.15 variables. Its result is
*                  a q1.31 value.
* Input          : Operand 1 (factor), Operand 2 (factor)
* Output         : Op1 * Op2, q1.31 format
* Return         : none.
*******************************************************************************/                 
void mul_q15_q15_q31(s16 Op1, s16 Op2, s32 *Out)
{
  s32 temp;
 
  temp = Op1 * Op2;
  
  if (temp==0x40000000)   /* Overflow (-1*-1) */
  {
    temp=0x7fffffff;
  }
  else
  {       
     temp = temp * 2;       /* One bit left shift to remove redondant sign bit */       
  }  
  *Out=temp;
}
/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
