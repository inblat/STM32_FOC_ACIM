/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : main.c
* Author             : IMS Systems Lab
* Date First Issued  : 11/28/07
* Description        : Main program body.
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
#include "MC_Globals.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void RCC_Configuration(void);

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{

#ifdef DEBUG
  debug();
#endif
  
  NVIC_Configuration();   
  RCC_Configuration();
  GPIO_Configuration();
  
#ifdef THREE_SHUNT  
  SVPWM_3ShuntInit();
#elif defined ICS_SENSORS
  SVPWM_IcsInit();
#elif defined SINGLE_SHUNT
  SVPWM_1ShuntInit();
#endif
  
#if defined(TACHO)
  TAC_TachoTimerInit();
  TAC_InitTachoMeasure();  
#elif defined(ENCODER)
  ENC_Init();
#endif
  
#ifdef DAC_FUNCTIONALITY   
  MCDAC_Configuration();
#endif
  
  TB_Init();
  
  
  PID_Init(&PID_Torque_InitStructure, &PID_Flux_InitStructure, &PID_Speed_InitStructure);
  
  /* TIM1 Counter Clock stopped when the core is halted */
  DBGMCU_Config(DBGMCU_TIM1_STOP, ENABLE);
  
  KEYS_Init();
  
  // Init Bus voltage and Temperature arrays  
 
  MCL_Init_Arrays();
    
  STM3210B_LCD_Init();
  LCD_Clear(White);
  LCD_SetTextColor(Blue);
  LCD_SetBackColor(White);
  
  Display_Welcome_Message();
  
  State = IDLE;
  
  while(1)
  {

    Display_LCD();
    
    MCL_ChkPowerStage();    
    
    //User interface management    
    KEYS_process();  
    
      
    switch (State)
    {
      case IDLE:     
        break;
        
      case INIT:
        
        MCL_Init();
        TB_Set_StartUp_Timeout(STARTUP_TIMEOUT);
        State = START;
        break;
         
      case START: 

#ifdef TACHO          
          if(TAC_ValidSpeedInfo(TACHO_SPEED_VAL))
          {           
            
            if ((wGlobal_Flags & CLOSED_LOOP) == CLOSED_LOOP)
            {              
              //computes hRot_Freq [1.15], i.e the rotor mechanical frequency           
              hRot_Freq_Hz = TAC_GetRotorFreqInHz();                   
              
              //It initializes Integral term of speed PID to avoid discontinuity
              //in torque reference
              PID_Speed_InitStructure.wIntegral=((s32)(hTorque_Reference*256));
            }
            State = RUN; 
          }
          else 
            if (TB_StartUp_Timeout_IsElapsed())
            {
                //shutdown power
                
                MCL_SetFault(SPEED_FEEDBACK);

            }

            
#elif defined ENCODER          
          {
            s16 hMech_Speed_Hz;
            s16 hLocalSpeed_Reference = hSpeed_Reference;
            
            hMech_Speed_Hz = ENC_Get_Mechanical_Speed();
            if((wGlobal_Flags & CLOSED_LOOP) == CLOSED_LOOP)
            {  
              u16 Abs_Freq;
              Abs_Freq = (hMech_Speed_Hz < 0 ? -hMech_Speed_Hz : hMech_Speed_Hz);
              
              if (Abs_Freq > ENCODER_CL_ENABLE)
              {              
                //It initializes Integral term of speed PID to avoid discontinuity
                //in torque reference
                PID_Speed_InitStructure.wIntegral =
                  (((s32)(hTorque_Reference)*128)-
                   ((s32)(PID_Speed_InitStructure.hKp_Gain*8)*
                    (hLocalSpeed_Reference-hMech_Speed_Hz)));
                State = RUN; 
              }
              else if (TB_StartUp_Timeout_IsElapsed())
              {
                //shutdown power
                
                MCL_SetFault(SPEED_FEEDBACK);
                
              }
            }
            else //Torque mode loop
            { 
              if (hMech_Speed_Hz != 0)
              {
                State = RUN; 
              }
              else if (TB_StartUp_Timeout_IsElapsed())
              {
                //shutdown power
                
                MCL_SetFault(SPEED_FEEDBACK);
                
              }
            }
          }

#endif
        break;
      
      case RUN:
        
#ifdef TACHO 
            if (TAC_IsTimedOut())
            { 

#elif defined ENCODER             

            if(ENC_ErrorOnFeedback() == TRUE)
            {  
#endif            
              
              MCL_SetFault(SPEED_FDBK_TIMED_OUT);
              
            }

            break;         
            
      case BRAKE:    
          State = STOP;
        break;  
        
      case STOP: 
          
          // shutdown power 
                    
          /* Main PWM Output Disable */
          TIM_CtrlPWMOutputs(TIM1, DISABLE);
          
#ifdef THREE_SHUNT          
          SVPWM_3ShuntAdvCurrentReading(DISABLE);
#endif
#ifdef SINGLE_SHUNT          
          SVPWM_1ShuntAdvCurrentReading(DISABLE);
#endif 
          
          Stat_Volt_alfa_beta.qV_Component1 = Stat_Volt_alfa_beta.qV_Component2 = 0;
          
#ifdef ICS_SENSORS
          SVPWM_IcsCalcDutyCycles(Stat_Volt_alfa_beta);
#elif defined THREE_SHUNT
          SVPWM_3ShuntCalcDutyCycles(Stat_Volt_alfa_beta);
#endif        
          State = WAIT;
          
        
        break;
          
        case WAIT:    // wait state

#ifdef TACHO
           if (TAC_GetRotorFreqInHz() == 0) 
#elif defined ENCODER
           if (ENC_Get_Mechanical_Speed() == 0)  
#endif          
            {
              State = IDLE;
            }
          break; 
          
         case FAULT:   // Fault detected  
              
             if (MCL_ClearFault() == TRUE)
                  {
                    if(wGlobal_Flags & CLOSED_LOOP == CLOSED_LOOP)
                    {
                      bMenu_index = CONTROL_MODE_MENU_1;
                    }
                    else
                    {
                      bMenu_index = CONTROL_MODE_MENU_6;
                    }   
                    State = IDLE;
                  }
              break;
           
           
     
        default:
          break;     
    }
  }
}      
        
     
/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the TIM1 Pins.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIOA, GPIOB, GPIOC, GPIOE clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE, ENABLE);
  
  GPIO_DeInit(GPIOA);
  GPIO_DeInit(GPIOB);
  GPIO_DeInit(GPIOC);
  GPIO_DeInit(GPIOD);
 

  /* Configure PC.06, PC.07, PC.08 and PC.09 as Output push-pull ---------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /* Configure PB.01 as alternate function output */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Configure PD.08,12,14 as floating input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_12 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  /* Configure PE.00,01 as floating input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  /* Configure PB9 as floating input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  
}

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{ 
  ErrorStatus HSEStartUpStatus;

  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  
  if(HSEStartUpStatus == SUCCESS)
  {
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
}
  
/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures the Vector Table base address.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
#ifdef  VECT_TAB_RAM  
  /* Set the Vector Table base location at 0x20000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif
}

#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  //printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  /* Infinite loop */
  while (1)
  {
    
  }
}
#endif

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
