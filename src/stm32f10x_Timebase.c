/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : stm32f10x_Timebase.c
* Author             : IMS Systems Lab 
* Date First Issued  : 11/28/2007
* Description        : This module handles time base. It used in display and 
*                      fault management, speed regulation, motor ramp-up  
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

/* Include of other module interface headers ---------------------------------*/
/* Local includes ------------------------------------------------------------*/

#include "stm32f10x_MClib.h"
#include "stm32f10x_it.h"
#include "MC_Globals.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TB_Prescaler_5ms    31    // ((31+1)*(9374+1)/60000000) sec -> 5 ms 
#define TB_AutoReload_5ms   9374

#define TB_Prescaler_500us  29    // ((29+1)*(999+1)/60000000) sec -> 500 us 
#define TB_AutoReload_500us 999

#define SYSTICK_PRE_EMPTION_PRIORITY 3
#define SYSTICK_SUB_PRIORITY 0

#define TORQUE_DELAY 500

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static u16 hStart_Up_TimeLeft_500us =0;
static volatile u16 hTimebase_500us = 0;
static volatile u16 hTimebase_display_500us = 0;
static volatile u16 hKey_debounce_500us = 0;
volatile u8 bPID_Speed_Sampling_Time_500us = PID_SPEED_SAMPLING_TIME;
#ifdef ENCODER
static u16 hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;
#endif
#ifdef PIDs_TUNING  
static u16 hTorqueSwapping = TORQUE_DELAY; 
#endif

static volatile s32 wOL_Start_up_Torque = 0;

/*******************************************************************************
* Function Name  : TB_Init
* Description    : TimeBase peripheral initialization. The base time is set to 
*                  500usec and the related interrupt is enabled  
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TB_Init(void)
{   
  /* Select AHB clock(HCLK) as SysTick clock source */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  /* SysTick interrupt each 500usec with Core clock equal to 72MHz */
  SysTick_SetReload(36000);
  /* Enable SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Enable);

  NVIC_SystemHandlerPriorityConfig(SystemHandler_SysTick, 
                            SYSTICK_PRE_EMPTION_PRIORITY, SYSTICK_SUB_PRIORITY); 
  /* Enable SysTick interrupt */
  SysTick_ITConfig(ENABLE);
}

/*******************************************************************************
* Function Name  : TB_Wait
* Description    : The function wait for a delay to be over.   
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TB_Wait(u16 time)
{
hTimebase_500us = time;    // delay = 'time' value * 5ms
while (hTimebase_500us != 0) // wait and do nothing!
{}  

}

/*******************************************************************************
* Function Name  : TB_Set_Delay_500us
* Description    : Set delay utilized by main.c state machine.   
* Input          : Time out value
* Output         : None
* Return         : None
*******************************************************************************/
void TB_Set_Delay_500us(u16 hDelay)
{
  hTimebase_500us = hDelay;
}  

/*******************************************************************************
* Function Name  : TB_Delay_IsElapsed
* Description    : Check if the delay set by TB_Set_Delay_500us is elapsed.   
* Input          : None
* Output         : True if delay is elapsed, false otherwise 
* Return         : None
*******************************************************************************/
bool TB_Delay_IsElapsed(void)
{
 if (hTimebase_500us == 0)
 {
   return (TRUE);
 }
 else 
 {
   return (FALSE);
 }
}  

/*******************************************************************************
* Function Name  : TB_Set_DisplayDelay_500us
* Description    : Set Delay utilized by MC_Display.c module.   
* Input          : Time out value
* Output         : None
* Return         : None
*******************************************************************************/
void TB_Set_DisplayDelay_500us(u16 hDelay)
{
  hTimebase_display_500us = hDelay;
}  

/*******************************************************************************
* Function Name  : TB_DisplayDelay_IsElapsed
* Description    : Check if the delay set by TB_Set_DisplayDelay_500us is elapsed.   
* Input          : None
* Output         : True if delay is elapsed, false otherwise 
* Return         : None
*******************************************************************************/
bool TB_DisplayDelay_IsElapsed(void)
{
 if (hTimebase_display_500us == 0)
 {
   return (TRUE);
 }
 else 
 {
   return (FALSE);
 }
} 

/*******************************************************************************
* Function Name  : TB_Set_DebounceDelay_500us
* Description    : Set Delay utilized by MC_Display.c module.   
* Input          : Time out value
* Output         : None
* Return         : None
*******************************************************************************/
void TB_Set_DebounceDelay_500us(u8 hDelay)
{
  hKey_debounce_500us = hDelay;
}  

/*******************************************************************************
* Function Name  : TB_DebounceDelay_IsElapsed
* Description    : Check if the delay set by TB_Set_DebounceDelay_500us is elapsed.   
* Input          : None
* Output         : True if delay is elapsed, false otherwise 
* Return         : None
*******************************************************************************/
bool TB_DebounceDelay_IsElapsed(void)
{
 if (hKey_debounce_500us == 0)
 {
   return (TRUE);
 }
 else 
 {
   return (FALSE);
 }
} 

/*******************************************************************************
* Function Name  : TB_Set_StartUp_Timeout(STARTUP_TIMEOUT)
* Description    : Set Start up time out and initialize Start_up torque in open 
*                  loop.   
* Input          : Time out value
* Output         : None
* Return         : None
*******************************************************************************/
void TB_Set_StartUp_Timeout(u16 hTimeout)
{
  hStart_Up_TimeLeft_500us = 2*hTimeout; 
  wOL_Start_up_Torque = 0;
}  

/*******************************************************************************
* Function Name  : TB_StartUp_Timeout_IsElapsed
* Description    : Set Start up time out.   
* Input          : None
* Output         : True if start up time out is elapsed, false otherwise 
* Return         : None
*******************************************************************************/
bool TB_StartUp_Timeout_IsElapsed(void)
{
 if (hStart_Up_TimeLeft_500us == 0)
 {
   return (TRUE);
 }
 else 
 {
   return (FALSE);
 }
} 


/*******************************************************************************
* Function Name  : SysTickHandler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTickHandler(void)
{
  
  if (hTimebase_500us != 0)  
  {
    hTimebase_500us --;
  }
  
  if (hTimebase_display_500us != 0)  
  {
    hTimebase_display_500us --;
  }
  
  if (hKey_debounce_500us != 0)  
  {
    hKey_debounce_500us --;
  }

  if (hStart_Up_TimeLeft_500us !=0)
  { 
    u16 hStart_up_Timebase_500us;
    
    //Start up ramp torque implementation
    hStart_Up_TimeLeft_500us --;
    hStart_up_Timebase_500us = (u16)(STARTUP_TIMEOUT*2 - hStart_Up_TimeLeft_500us);
    
    if (hStart_up_Timebase_500us <= 2*STARTUP_RAMP_DURATION)
      { // if in speed closed loop
        if ((wGlobal_Flags & CLOSED_LOOP) == CLOSED_LOOP)
        {
          if (State == START)
          {
            wOL_Start_up_Torque += CL_STARTUP_TORQUE_INCREMENT;           
            if(hSpeed_Reference>=0)
            {
              hTorque_Reference = (s16) (wOL_Start_up_Torque/16);
            }
            else
            {
              hTorque_Reference = (s16)(-wOL_Start_up_Torque/16);
            }
          }  
        }
        else //speed open loop
        {
          wOL_Start_up_Torque += hOL_Start_up_Torque_Increment;
          hTorque_Reference = (s16)(wOL_Start_up_Torque/16);
        }
        
      }
    else // 2*STARTUP_RAMP_DURATION < hStart_up_Timebase_500us < STARTUP_TIMEOUT
    {
      if ((wGlobal_Flags & CLOSED_LOOP) == CLOSED_LOOP)
      {
        if (State == START)
        {
          hTorque_Reference = STARTUP_FINAL_TORQUE;
        }  
      }
      else //speed open loop
      { // Torque command must be equal to the value set by the user at the end 
        // of the ramp, then it is adjustable by joystick
       if (hStart_up_Timebase_500us == (2*STARTUP_RAMP_DURATION +1))
       {
         hTorque_Reference = (s16)(wOL_Start_up_Torque_Ref/16);
       }
      }
    }
  }
  
#ifdef ENCODER
  if (hSpeedMeas_Timebase_500us !=0)
  {
    hSpeedMeas_Timebase_500us--;
  }
  else
  {
    hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;
    //ENC_Calc_Average_Speed must be called ONLY every SPEED_MEAS_TIMEBASE ms
    ENC_Calc_Average_Speed();   
  }
#endif


  if (bPID_Speed_Sampling_Time_500us != 0 )  
  {
    bPID_Speed_Sampling_Time_500us --;
  }
  else
  {    
#if defined (ENCODER)
    hRot_Freq_Hz = ENC_Get_Mechanical_Speed();
#elif defined (TACHO)
    hRot_Freq_Hz = TAC_GetRotorFreqInHz();   //computes hRot_Freq [1.15], i.e the rotor mechanical frequency
#endif
    
    //bPID_Speed_Sampling_Time_500us = PID_SPEED_SAMPLING_TIME;  
    if(State == RUN)
    {
        if ((wGlobal_Flags & CLOSED_LOOP) == CLOSED_LOOP)
        {
          bPID_Speed_Sampling_Time_500us = PID_SPEED_SAMPLING_TIME;  
          IFOC_CalcFluxTorqueRef();
        }
     }
   }
}


/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
