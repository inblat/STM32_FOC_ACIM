/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : MC_MotorControl_Layer.c
* Author             : IMS Systems Lab  
* Date First Issued  : 11/28/2007
* Description        : This file contains the function implementing the motor 
*                      control layer 
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
#include "stm32f10x_type.h"
#include "MC_Globals.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BRK_GPIO GPIOE
#define BRK_PIN GPIO_Pin_15

#define FAULT_STATE_MIN_PERMANENCY 600 //0.5msec unit

#define AV_ARRAY_SIZE  (u8)16  //number of averaged acquisitions
#define AV_BIT_NUM     (u8)4   //and number of required bits to store this number

#define BUSV_CONVERSION (u16) 409 
#define TEMP_CONVERSION (u8)  204

#define VOLT_ARRAY_INIT (u16) (UNDERVOLTAGE_THRESHOLD+ OVERVOLTAGE_THRESHOLD)/2
#define TEMP_ARRAY_INIT (u16) 0

#define BREAK_GPIO_PORT       GPIOD
#define BREAK_GPIO_PIN        GPIO_Pin_13

/* Private macro -------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void MCL_Reset_PID_IntegralTerms(void);
/* Private variables ---------------------------------------------------------*/

static u32 w_BusV_Average;
static u32 w_Temp_Average;
static u16 hBusV_av_array[AV_ARRAY_SIZE];
static u16 hTemp_av_array[AV_ARRAY_SIZE];

u16 h_ADCBusvolt;
u16 h_ADCTemp;

/*******************************************************************************
* Function Name  : MCL_Init
* Description    : This function implements the motor control initialization to 
*                  be performed at each motor start-up 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MCL_Init(void)
{

// reset PID's integral values
    MCL_Reset_PID_IntegralTerms();
    
#ifdef ENCODER
    ENC_Clear_Speed_Buffer();
#elif defined TACHO
    TAC_InitTachoMeasure();  
#endif   
    
    IFOC_Init();
 
//It generates for 2 msec a 50% duty cycle on the three phases to load Boot 
//capacitance of high side drivers
    TB_Set_StartUp_Timeout(4);    
    
    /* Main PWM Output Enable */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    while(!TB_StartUp_Timeout_IsElapsed())
    {
    }
        
    if ((wGlobal_Flags & CLOSED_LOOP) != CLOSED_LOOP) 
      {
        wOL_Start_up_Torque_Ref = hTorque_Reference*16;
        hOL_Start_up_Torque_Increment = (s16)(wOL_Start_up_Torque_Ref/(2*STARTUP_RAMP_DURATION));
      }
                      
    hTorque_Reference=0;    
      
#ifdef THREE_SHUNT                    
    SVPWM_3ShuntCurrentReadingCalibration();
#elif defined ICS_SENSORS
    SVPWM_IcsCurrentReadingCalibration();
#elif defined SINGLE_SHUNT
    SVPWM_1ShuntCurrentReadingCalibration();
#endif   
    
#ifdef THREE_SHUNT    
    // Enable the Adv Current Reading during Run state
    SVPWM_3ShuntAdvCurrentReading(ENABLE);
#endif 
#ifdef SINGLE_SHUNT    
    // Enable the Adv Current Reading during Run state
    SVPWM_1ShuntAdvCurrentReading(ENABLE);
#endif  
}

/*******************************************************************************
* Function Name  : MCL_Init_Arrays
* Description    : This function initializes array to avoid erroneous Fault 
*                  detection after a reswt
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MCL_Init_Arrays(void)
{ 
  u32 i;

  for(i= AV_ARRAY_SIZE; i > 0; i--)
  {
    hTemp_av_array[i-1] = TEMP_ARRAY_INIT;
    hBusV_av_array[i-1] = VOLT_ARRAY_INIT;   
  }
}

/*******************************************************************************
* Function Name  : MCL_ChkPowerStage
* Description    : This function check for power stage working conditions
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MCL_ChkPowerStage(void) 
{
  //  check over temperature of power stage
    if (MCL_Chk_OverTemp() == TRUE) 
    {
      MCL_SetFault(OVERHEAT);
    }
 
    //  check bus over voltage 
    if (MCL_Chk_BusVolt() == OVER_VOLT) 
    {
      MCL_SetFault(OVER_VOLTAGE);
    }
    
    //  check bus under voltage 
    if (MCL_Chk_BusVolt() == UNDER_VOLT) 
    {
      MCL_SetFault(UNDER_VOLTAGE);
    }
}

/*******************************************************************************
* Function Name  : MCL_SetFault() 
* Description    : This function manage faults occurences
* Input          : Fault type
* Output         : None
* Return         : None
*******************************************************************************/
void MCL_SetFault(u16 hFault_type)
{
  TB_Set_Delay_500us(FAULT_STATE_MIN_PERMANENCY); 
  /* Main PWM Output Enable */
  TIM_CtrlPWMOutputs(TIM1, DISABLE);
  wGlobal_Flags |= hFault_type;
  State = FAULT;
  bMenu_index = FAULT_MENU;
  // It is required to disable AdvCurrentReading in IDLE to sample DC 
  // Bus Value
#ifdef THREE_SHUNT
  SVPWM_3ShuntAdvCurrentReading(DISABLE);
#endif
#ifdef SINGLE_SHUNT
  SVPWM_1ShuntAdvCurrentReading(DISABLE);
#endif
}

/*******************************************************************************
* Function Name  : MCL_ClearFault() 
* Description    : This function check if the fault source is over. In case it 
*                  is, it clears the related flag and return true. Otherwise it 
*                  returns FALSE
* Input          : Fault type
* Output         : None
* Return         : None
*******************************************************************************/
bool MCL_ClearFault(void)
{     
  if (TB_Delay_IsElapsed())
  {   
    if ((wGlobal_Flags & OVERHEAT) == OVERHEAT)   
    {               
      if(MCL_Chk_OverTemp()== FALSE)
      {
        wGlobal_Flags &= ~OVERHEAT;
      }     
    }
    
    if ((wGlobal_Flags & OVER_VOLTAGE) == OVER_VOLTAGE)   
    {            
        if(MCL_Chk_BusVolt()== NO_FAULT)
        {
          wGlobal_Flags &= ~OVER_VOLTAGE;
        } 
    }
    
     if ((wGlobal_Flags & UNDER_VOLTAGE) == UNDER_VOLTAGE)   
    {            
        if(MCL_Chk_BusVolt()== NO_FAULT)
        {
          wGlobal_Flags &= ~UNDER_VOLTAGE;
        } 
    }
    
    if ((wGlobal_Flags & OVER_CURRENT) == OVER_CURRENT)
    {
      // high level detected on emergency pin?              
      //It checks for a low level on Break Input before re-enable PWM 
      //peripheral
      if (GPIO_ReadInputDataBit(BRK_GPIO, BRK_PIN))
      {            
        wGlobal_Flags &= ~OVER_CURRENT;
      }
    }
  
    if ((wGlobal_Flags & SPEED_FEEDBACK) == SPEED_FEEDBACK )
    {
        wGlobal_Flags &= ~SPEED_FEEDBACK;
    } 
  
      
    if ( (wGlobal_Flags & SPEED_FDBK_TIMED_OUT) == SPEED_FDBK_TIMED_OUT)
    {
#ifdef TACHO       
        TAC_ClrTimeOut(); 
#endif         
        wGlobal_Flags &= ~SPEED_FDBK_TIMED_OUT;
    }
  
  }
  
  if (KEYS_ExportbKey() == SEL)
  {
    
  
    if ( (wGlobal_Flags & (OVER_CURRENT | OVERHEAT | UNDER_VOLTAGE |
          SPEED_FEEDBACK | OVER_VOLTAGE | SPEED_FDBK_TIMED_OUT)) == 0 )       

    
    { 
      return(TRUE);
    } 
    else
    {
      return(FALSE);
    }
  }
  else 
  {
    return(FALSE);
  }
}

/*******************************************************************************
* Function Name  : MCL_Chk_OverTemp
* Description    : Return TRUE if the voltage on the thermal resistor connected 
*                  to channel AIN3 has reached the threshold level or if the           
*                  voltage has not yet reached back the threshold level minus  
*                  the hysteresis value after an overheat detection.
* Input          : None
* Output         : Boolean
* Return         : None
*******************************************************************************/
bool MCL_Chk_OverTemp(void)
{
  u32 i;
  static u8 bIndex=0;
  u32 wAux = 0;
  
  hTemp_av_array[bIndex] = h_ADCTemp;
  bIndex++;
  
  if (bIndex == AV_ARRAY_SIZE)
  {
    bIndex = 0;
  }
        
  for(i= AV_ARRAY_SIZE; i>0; i--)
  {
    wAux += hTemp_av_array[i-1];
  }
  wAux >>= AV_BIT_NUM;    
  
  w_Temp_Average = wAux;
  
  if (wAux >= NTC_THRESHOLD)    
  {
    return(TRUE);
  }
  else if (wAux >= (NTC_THRESHOLD - NTC_HYSTERESIS) ) 
    {
    if ((wGlobal_Flags & OVERHEAT) == OVERHEAT)
      {
        return(TRUE);        
      }
    else
      {
        return(FALSE);
      }
    }
  else 
    {
      return(FALSE);
    }
}

/*******************************************************************************
* Function Name  : MCL_Chk_BusVolt 
* Description    : Check for Bus Under / Over Voltage
* Input          : None
* Output         : Boolean
* Return         : None
*******************************************************************************/
BusV_t MCL_Chk_BusVolt(void)
{
  u32 i;
  static u8 bIndex=0;
  u32 wAux = 0;
  
  hBusV_av_array[bIndex] = h_ADCBusvolt;
  bIndex++;
  
  if (bIndex == AV_ARRAY_SIZE)
  {
    bIndex = 0;
  }
    
  for(i= AV_ARRAY_SIZE; i > 0; i--)
  {
    wAux += hBusV_av_array[i-1];
  }
  wAux >>= AV_BIT_NUM;
  
  w_BusV_Average = wAux; 
    
  if (wAux > OVERVOLTAGE_THRESHOLD)    
  {
    return((BusV_t)(OVER_VOLT));
  }
  else 
      if (wAux < UNDERVOLTAGE_THRESHOLD)    
      {
        return((BusV_t)(UNDER_VOLT));
      }
      else 
      {
        return(NO_FAULT);
      }
}

/*******************************************************************************
* Function Name  : MCL_Compute_BusVolt
* Description    : Compute bus voltage in volt
* Input          : None
* Output         : Bus voltage in Volt unit
* Return         : None
*******************************************************************************/
u16 MCL_Compute_BusVolt(void)
{
  return ((u16)((w_BusV_Average * BUSV_CONVERSION)/32768));
}

/*******************************************************************************
* Function Name  : MCL_Compute_Temp
* Description    : Compute temperature in Celsius degrees
* Input          : None
* Output         : temperature in Celsius degrees
* Return         : None
*******************************************************************************/
u8 MCL_Compute_Temp(void)
{
  return ((u8)((w_Temp_Average * TEMP_CONVERSION)/32768+7));
}      

/*******************************************************************************
* Function Name  : MCL_Reset_PID_IntegralTerms
* Description    : Resets flux, torque and speed PID Integral Terms
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MCL_Reset_PID_IntegralTerms(void)
{
  PID_Speed_InitStructure.wIntegral=0;
  PID_Torque_InitStructure.wIntegral=0;
  PID_Flux_InitStructure.wIntegral = 0;
}
/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
