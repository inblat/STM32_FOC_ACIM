/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : stm32f10x_tacho.c
* Author             : IMS Systems Lab & MCD Application Team 
* Date First Issued  : 11/28/2007
* Description        : Module handling speed feedback provided by a
                       tachogenerator.
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
#include "stm32f10x_tacho.h"
#include "MC_tacho_prm.h"
#include "MC_globals.h"
#include "stm32f10x_MClib.h"
#include "MC_acmotor_prm.h"

/* Private define ------------------------------------------------------------*/
#define TIMx_PRE_EMPTION_PRIORITY 2
#define TIMx_SUB_PRIORITY 0

#define LOW_RES_THRESHOLD   ((u16)0x5500u)// If capture below, ck prsc decreased
#define	ROTOR_SPEED_FACTOR  ((u32)((POLE_PAIR_NUM * (CKTIM*10)) / TACHO_PULSE_PER_REV))
#define PSEUDO_FREQ_CONV    ((u32)(ROTOR_SPEED_FACTOR / (SAMPLING_FREQ * 10)) * 0x10000uL)
#define SPEED_OVERFLOW      ((u32)(ROTOR_SPEED_FACTOR / MAX_SPEED_FDBK))
#define MAX_PERIOD          ((u32)((CKTIM * 10) / MIN_SPEED_FDBK))
#define TACHO_COUNTER_RESET ((u8)0)
#define ICx_FILTER          (u8) 6

// Here is practically assigned the tacho timer
#if defined(TIMER2_HANDLES_TACHO)
    #define TACHO_TIMER TIM2
#elif defined(TIMER3_HANDLES_TACHO)
    #define TACHO_TIMER TIM3
#else // TIMER4_HANDLES_TACHO
    #define TACHO_TIMER TIM4
#endif

// Here is practically assigned the tacho timer input channel
#if defined(TACHO_INPUT_TI1)
    #define TIM_IT_CC TIM_IT_CC1
#elif defined(TACHO_INPUT_TI2)
    #define TIM_IT_CC TIM_IT_CC2
#endif

// To avoid obvious initialization errors...
#if  ( (defined(TIMER2_HANDLES_TACHO) && defined(TIMER3_HANDLES_TACHO)) \
    || (defined(TIMER2_HANDLES_TACHO) && defined(TIMER4_HANDLES_TACHO)) \
    || (defined(TIMER3_HANDLES_TACHO) && defined(TIMER4_HANDLES_TACHO)))
  #error "Invalid tacho setup: 2 timers selected"
#endif

// To avoid obvious initialization errors...
#if  ( (defined(TACHO_INPUT_TI1) && defined(TACHO_INPUT_TI2)) ) 
    #error "Invalid tacho setup: 2 inputs selected"
#endif

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct {
	u16 hCapture;
	u16 hPrscReg;
	} SpeedMeas_s;

/* Private variables ---------------------------------------------------------*/

volatile SpeedMeas_s SensorPeriod[SPEED_FIFO_SIZE]; // Holding the last captures
vu8 bSpeedFIFO_Index;   // Index of above array
vu8 bGP1_OVF_Counter;   // Count overflows if prescaler is too low
vu16 hCaptCounter;      // Holds the number of tacho interrupts

volatile bool RatioDec;
volatile bool RatioInc;
volatile bool DoRollingAverage;
volatile bool InitRollingAverage;
volatile bool TachoTimeOut;

/* Private function prototypes -----------------------------------------------*/
u32 GetLastTachoPeriod(void);
u32 GetAvrgTachoPeriod(void);


/*******************************************************************************
* Function Name  : TAC_TachoTimerInit
* Description    : Initializes the timer handling tacho feedback
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TAC_TachoTimerInit(void)
{

  TIM_TimeBaseInitTypeDef TIM_TACTimeBaseInitStructure;
  TIM_ICInitTypeDef       TIM_TACICInitStructure;
  NVIC_InitTypeDef        NVIC_InitTACStructure;
  GPIO_InitTypeDef        GPIO_InitStructure;
  
  #if defined(TIMER2_HANDLES_TACHO)
    /* TIM2 clock source enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  #elif defined(TIMER3_HANDLES_TACHO)
    /* TIM3 clock source enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  #else // TIMER4_HANDLES_TACHO
    /* TIM4 clock source enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  #endif
    
  // The following lines work properly if tacho signal output is connected to PA0
  GPIO_StructInit(&GPIO_InitStructure);
  /* Configure PA.00 as tacho input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  
  // Timer configuration in Clear on capture mode
  TIM_DeInit(TACHO_TIMER);
  
  TIM_TimeBaseStructInit(&TIM_TACTimeBaseInitStructure);
  TIM_ICStructInit(&TIM_TACICInitStructure);
  
  #if defined(TACHO_INPUT_TI1)
    TIM_TACICInitStructure.TIM_Channel = TIM_Channel_1;
  #elif defined(TACHO_INPUT_TI2)
    TIM_TACICInitStructure.TIM_Channel = TIM_Channel_2;
  #endif
  
  TIM_TACTimeBaseInitStructure.TIM_Period = U16_MAX;  // Set full 16-bit working range
  
  TIM_TACICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_TACICInitStructure.TIM_ICFilter = ICx_FILTER;

   TIM_TimeBaseInit(TACHO_TIMER,&TIM_TACTimeBaseInitStructure);
   TIM_ICInit(TACHO_TIMER,&TIM_TACICInitStructure);
   
   // Force the TACHO_TIMER prescaler with immediate access (no need of an update event) 
   TIM_PrescalerConfig(TACHO_TIMER, (u16) MAX_RATIO, TIM_PSCReloadMode_Immediate);
   
   TIM_InternalClockConfig(TACHO_TIMER);
   
   #if defined(TACHO_INPUT_TI1)
    TIM_SelectInputTrigger(TACHO_TIMER, TIM_TS_TI1FP1);
   #else //TACHO_INPUT_TI2
    TIM_SelectInputTrigger(TACHO_TIMER, TIM_TS_TI2FP2);
   #endif 

   TIM_SelectSlaveMode(TACHO_TIMER,TIM_SlaveMode_Reset);
   
   // Source of Update event is only counter overflow/underflow
   TIM_UpdateRequestConfig(TACHO_TIMER, TIM_UpdateSource_Regular);
    
  /* Enable the TACHO_TIMER IRQChannel*/
  #if defined(TIMER2_HANDLES_TACHO)
    NVIC_InitTACStructure.NVIC_IRQChannel = TIM2_IRQChannel;
  #elif defined(TIMER3_HANDLES_TACHO)
    NVIC_InitTACStructure.NVIC_IRQChannel = TIM3_IRQChannel;
  #else // TIMER4_HANDLES_TACHO
    NVIC_InitTACStructure.NVIC_IRQChannel = TIM4_IRQChannel;
  #endif

  NVIC_InitTACStructure.NVIC_IRQChannelPreemptionPriority =  TIMx_PRE_EMPTION_PRIORITY;
  NVIC_InitTACStructure.NVIC_IRQChannelSubPriority = TIMx_SUB_PRIORITY;
  NVIC_InitTACStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitTACStructure);
  
  // Clear the TIMx's pending flags
  TIM_ClearFlag(TACHO_TIMER,  TIM_FLAG_Update);
  TIM_ClearFlag(TACHO_TIMER,  TIM_FLAG_CC1);
  TIM_ClearFlag(TACHO_TIMER,  TIM_FLAG_CC2); 
  TIM_ClearFlag(TACHO_TIMER,  TIM_FLAG_CC3); 
  TIM_ClearFlag(TACHO_TIMER,  TIM_FLAG_CC4);
TIM_ClearFlag(TACHO_TIMER,  TIM_FLAG_Trigger);
TIM_ClearFlag(TACHO_TIMER,  TIM_FLAG_CC1OF);
TIM_ClearFlag(TACHO_TIMER,  TIM_FLAG_CC2OF);
TIM_ClearFlag(TACHO_TIMER,  TIM_FLAG_CC3OF);
TIM_ClearFlag(TACHO_TIMER,  TIM_FLAG_CC4OF);

//+ TIM_FLAG_Trigger + TIM_FLAG_CC1OF + \
                //TIM_FLAG_CC2OF + TIM_FLAG_CC3OF + TIM_FLAG_CC4OF);
  
  // Selected input capture and Update (overflow) events generate interrupt
  TIM_ITConfig(TACHO_TIMER, TIM_IT_CC, ENABLE);
  TIM_ITConfig(TACHO_TIMER, TIM_IT_Update, ENABLE);

  TACHO_TIMER->CNT = TACHO_COUNTER_RESET;
  
}


/*******************************************************************************
* ROUTINE Name : TAC_InitTachoMeasure
*
* Description : Clear software FIFO where are "pushed" latest speed informations
*           This function must be called before starting the motor to initialize
*	    the speed measurement process.
*
* Input       : None
* Output      : None
* Return      : None
* Note        : First measurements following this function call will be done
*               without filtering (no rolling average).
*******************************************************************************/
void TAC_InitTachoMeasure( void )
{
   // Mask interrupts to insure a clean intialization
   
   TIM_ITConfig(TACHO_TIMER, TIM_IT_CC, DISABLE);
   
  
   RatioDec = FALSE;
   RatioInc = FALSE;
   DoRollingAverage = FALSE;
   InitRollingAverage = FALSE;
   TachoTimeOut = FALSE;

   hCaptCounter = 0;
   bGP1_OVF_Counter = 0;

   for (bSpeedFIFO_Index=0; bSpeedFIFO_Index < SPEED_FIFO_SIZE; bSpeedFIFO_Index++)
   {
      SensorPeriod[bSpeedFIFO_Index].hCapture = U16_MAX;
      SensorPeriod[bSpeedFIFO_Index].hPrscReg = MAX_RATIO;
   }

   // First measurement will be stored in the 1st array location
   bSpeedFIFO_Index = SPEED_FIFO_SIZE-1;

   // Re-initialize partly the timer
   TACHO_TIMER->PSC = MAX_RATIO;
   
   // TIM_ResetCounter(TACHO_TIMER);
   TACHO_TIMER->CNT = TACHO_COUNTER_RESET;
   
   TIM_Cmd(TACHO_TIMER, ENABLE);
   
   TIM_ITConfig(TACHO_TIMER, TIM_IT_CC, ENABLE);

}


/*******************************************************************************
* ROUTINE Name : TAC_GetRotorFreqInHz
*
* Description : This routine returns Rotor frequency with [0.1Hz] definition.
*		Result is given by the following formula:
*		Frotor = K x (Fosc / (Capture x number of overflow)))
*		where K depends on the number of motor and tacho poles pairs
*
* Input    : None
* Output   : None
* Returns  : Rotor mechanical frequency, with 0.1Hz resolution.
* Comments : Result is zero if speed is too low (glitches at start for instance)
*           Excessive speed (or high freq glitches will result in a pre-defined
*           value returned.
* Warning : Maximum expectable accuracy depends on CKTIM: 72MHz will give the
* 	    best results.
*******************************************************************************/
u16 TAC_GetRotorFreqInHz ( void )
{
   u32 wFreqBuffer;
   u16 hRotorFreq;

   if ( DoRollingAverage)
   {
      wFreqBuffer = GetAvrgTachoPeriod();
   }
   else
   {  // Raw tacho period
      wFreqBuffer = GetLastTachoPeriod();
   }

   if (TachoTimeOut == TRUE)
   {
      hRotorFreq = 0;
   }
   else
   {
      if ( TACHO_TIMER->PSC >= MAX_RATIO ) /* At start-up or very low freq */
      {                           /* Based on current prescaler value only */
         hRotorFreq = 0;
      }
      else
      {
         if( wFreqBuffer > MAX_PERIOD) /* Speed is too low */
         {
            hRotorFreq = 0;
         }
         else
         {
            if ( wFreqBuffer <= (u32)SPEED_OVERFLOW ) /*Avoid u32 DIV Overflow*/
            {
               hRotorFreq = MAX_SPEED;
            }
            else
            {
               hRotorFreq = (u16) ( ROTOR_SPEED_FACTOR / wFreqBuffer );
            }
         }
      }
   }

   return (hRotorFreq);
}


/*******************************************************************************
* ROUTINE Name : TAC_GetRotorFreq
*
* Description : This routine returns Rotor frequency with an unit that can be
*               directly integrated t oget the speed in the main control loop.
*
* Input    : None
* Output   : None
* Returns  : Rotor mechanical frequency with rad/PWM period unit
*             (here 2*PI rad = 0xFFFF).
* Comments : Result is zero if speed is too low (glitches at start for instance)
*           Excessive speed (or high freq glitches will result in a pre-defined
*           value returned.
* Warning : Maximum expectable accuracy depends on CKTIM: 60MHz will give the
* 	    best results.
*******************************************************************************/
u16 TAC_GetRotorFreq ( void )
{
   u32 wFreqBuffer;
   u16 hRotorFreq;

   if ( DoRollingAverage)
   {
      wFreqBuffer = GetAvrgTachoPeriod();
   }
   else
   {  // Raw tacho period
      wFreqBuffer = GetLastTachoPeriod();
   }

   if (TachoTimeOut == TRUE)
   {
      hRotorFreq = 0;
   }
   else
   {
      if ( TACHO_TIMER->PSC >= MAX_RATIO ) /* At start-up or very low freq */
      {                           /* Based on current prescaler value only */
         hRotorFreq = 0;
      }
      else
      {
         if( wFreqBuffer > MAX_PERIOD) /* Speed is too low */
         {
            hRotorFreq = 0;
         }
         else
         {
            if ( wFreqBuffer <= (u32)SPEED_OVERFLOW )/*Avoid u32 DIV Overflow*/
            {
               hRotorFreq = MAX_PSEUDO_SPEED;
            }
            else
            {
               hRotorFreq = ((u16) (PSEUDO_FREQ_CONV / wFreqBuffer) );
            }
         }
      }
   }

   return (hRotorFreq);
}


/*******************************************************************************
* ROUTINE Name : TAC_ClrTimeOut
*
* Description     : Clears the flag indicating that that informations are lost,
*                   or speed is decreasing sharply.
* Input           : None
* Output          : Clear TachoTimeOut
* Return          : None
*******************************************************************************/
void TAC_ClrTimeOut(void)
{
   TachoTimeOut = FALSE;
}


/*******************************************************************************
* ROUTINE Name : TAC_IsTimedOut
*
* Description     : Indicates to the upper layer SW that tacho informations
*                   disappeared (for a while at least).
* Input           : None
* Output          : None
* Return          : boolean, TRUE in case of Time Out
* Note            : The time-out duration depends on tacho timer pre-scaler,
*                   which is variable; the time-out will be higher at low speed.
*******************************************************************************/
bool TAC_IsTimedOut(void)
{
   return(TachoTimeOut);
}


/*******************************************************************************
* ROUTINE Name : TAC_GetCaptCounter
*
* Description     : Gives the number of tacho capture interrupts since last call
*                   of the TAC_ClrCaptCounter function.
* Input           : None
* Output          : None
* Return          : u16 integer (Roll-over is prevented in the tacho capture
*                   routine itself).
*******************************************************************************/
u16 TAC_GetCaptCounter(void)
{
   return(hCaptCounter);
}


/*******************************************************************************
* ROUTINE Name : TAC_ClrCaptCounter
*
* Description     : Clears the variable holding the number of capture events.
* Input           : None
* Output          : hCaptCounter is cleared.
* Return          : None
*******************************************************************************/
void TAC_ClrCaptCounter(void)
{
   hCaptCounter = 0;
}


/*******************************************************************************
* ROUTINE Name : GetLastTachoPeriod
*
* Description     : returns the rotor pseudo-period based on last tacho capture
* Input           : None
* Output          : None
* Return          : rotor pseudo-period, as a number of CKTIM periods
*******************************************************************************/
u32 GetLastTachoPeriod(void)
{
      u32 wFreqBuffer;
      u8 bLastSpeedFIFO_Index;

   // Store current index to prevent errors if Capture occurs during processing
   bLastSpeedFIFO_Index = bSpeedFIFO_Index;

   // This is done assuming interval between captures is higher than time
   // to read the two values
   wFreqBuffer = SensorPeriod[bLastSpeedFIFO_Index].hCapture;
   wFreqBuffer *= (SensorPeriod[bLastSpeedFIFO_Index].hPrscReg + 1);

   return (wFreqBuffer);
}


/*******************************************************************************
* ROUTINE Name : GetAvrgTachoPeriod
*
* Description    : returns the rotor pseudo-period based on 4 last tacho capture
* Input          : None
* Output         : None
* Return         : averaged rotor pseudo-period, as a number of CKTIM periods
* Side effect: the very last tacho period acquired may not be considered for the
* calculation if a capture occurs during averaging.
*******************************************************************************/
u32 GetAvrgTachoPeriod(void)
{
    u32 wFreqBuffer, wAvrgBuffer, wIndex;

  wAvrgBuffer = 0;

  for ( wIndex = 0; wIndex < SPEED_FIFO_SIZE; wIndex++ )
  {
     // Disable capture interrupts to have presc and capture of the same period
     
     TACHO_TIMER->DIER &= ~TIM_IT_CC;     // NB:Std libray not used for perf issues
     
     wFreqBuffer = SensorPeriod[wIndex].hCapture;
     wFreqBuffer *= (SensorPeriod[wIndex].hPrscReg + 1);
     
     TACHO_TIMER->DIER |= TIM_IT_CC;     // NB:Std libray not used for perf issues
     
     wAvrgBuffer += wFreqBuffer;	// Sum the whole tacho period FIFO
  }
  wAvrgBuffer += (SPEED_FIFO_SIZE/2)-1;  // Round to upper value
  wAvrgBuffer /= SPEED_FIFO_SIZE;        // Average value	
  return (wAvrgBuffer);
}


/*******************************************************************************
* ROUTINE Name : TAC_StartTachoFiltering
*
* Description : Set the flags to initiate tacho values smoothing mechanism.
* Input       : None
* Output      : The result of the next capture will be copied in the whole array
*               to have 1st average = last value.
* Return      : None
* Note: The initialization of the FIFO used to do the averaging will be done
*       when the next tacho capture interrupt will occur.
*******************************************************************************/
void TAC_StartTachoFiltering( void )
{
   InitRollingAverage = TRUE;
}


/*******************************************************************************
* ROUTINE Name : TAC_ValidSpeedInfo
*
* Description : Used in Start function to know if rotor shaft turns at the right
*               speed.
*
* Input       : Rotor frequency (0.1Hz resolution) above which speed information
*               is not considered reliable (tacho signal too weak).
* Output      : None
* Return      : Boolean, TRUE if tacho provides clean signals.
*
* Warning : Since there's no way to differentiate rotation direction with a
*           tachogenerator, user must be aware that this routine may return TRUE
*           in certain conditions (re-start with no/very short stop time before
*           and high inertia load), even if motor is not started in the right
*           condition. User should therefore manage a minimal amout of time
*           before re-starting.
*           NB: this function may be unefficient if start-up duration is far
*           shorter than time needed to have at least two consecutive speed info
*******************************************************************************/
bool TAC_ValidSpeedInfo( u16 hMinRotorFreq )
{
      bool IsValid;

   TachoTimeOut = FALSE;  // Time-out flag is not significant during start-up

   if ( hMinRotorFreq == 0 ) // Error
   {
      IsValid = FALSE;
   }
   else			// Right start conditions in closed loop:
   {
      if (TAC_GetRotorFreqInHz() > hMinRotorFreq)
      {
         // Decreasing tacho period means accelerating motor
         if (GetAvrgTachoPeriod() >= GetLastTachoPeriod())
         {
            IsValid = TRUE;
         }
         else
         {
            IsValid = FALSE;
         }
      }
      else
      {
         IsValid = FALSE;
      }
   }
   return (IsValid);
}

#ifdef TACHO

/*******************************************************************************
* Function Name  : TIMx_IRQHandler
* Description    : This function handles both the capture event and Update event 
*                  interrupt handling the tacho signal period measurement.
*                  
*                  - On 'CAPTURE' event case:
*                    If the average is initialized, the last captured measure is
*                    copied into the whole array.
*                    Period captures are managed as following:
*                    If too low, the clock prescaler is decreased for next measure
*                    If too high (ie there was overflows), the result is
*                    re-computed as if there was no overflow and the prescaler is
*                    increased to avoid overflows during the next capture
*                   
*                  - On 'UPDATE' event case:
*                    This function handles the overflow of the timer handling
*                    the tacho signal period measurement.
* Input          : 
*                  - On 'CAPTURE' event case:
*                    None
*                   
*                  - On 'UPDATE' event case: 
*                    None
*
* Output         : 
*                  - On 'CAPTURE' event case:
*                   Updates the array holding the 4 latest period measures, reset
*                   the overflow counter and update the clock prescaler to
*                   optimize the accuracy of the measurement.
*                   
*                  - On 'UPDATE' event case:
*                    Updates a Counter of overflows, handled and reset when next
*                    capture occurs. 
*
* Return         : None (Interrupt Service routine)
*******************************************************************************/
#if defined(TIMER2_HANDLES_TACHO)
void TIM2_IRQHandler(void)
#elif defined(TIMER3_HANDLES_TACHO)
void TIM3_IRQHandler(void)
#else // TIMER4_HANDLES_TACHO
void TIM4_IRQHandler(void)
#endif

{
// Check for the source of TIMx int - Capture or Update Event - 
if ( TIM_GetFlagStatus(TACHO_TIMER, TIM_FLAG_Update) == RESET )
{

   // A capture event occured for this interrupt request generation
  #if defined(TACHO_INPUT_TI1)
    TIM_ClearFlag(TACHO_TIMER, TIM_FLAG_CC1);
  #elif defined(TACHO_INPUT_TI2)
    TIM_ClearFlag(TACHO_TIMER, TIM_FLAG_CC2);
  #endif   

   if (hCaptCounter < U16_MAX)
   {
      hCaptCounter++;
   }

   // Compute new array index
   if (bSpeedFIFO_Index < SPEED_FIFO_SIZE-1)
   {
      bSpeedFIFO_Index++;
   }
   else
   {
      bSpeedFIFO_Index = 0;
   }

   // Store the latest speed acquisition
   if (bGP1_OVF_Counter != 0)	// There was counter overflow before capture
   {
        u32 wCaptBuf;
        u16 hPrscBuf;

      #if defined(TACHO_INPUT_TI1)
        wCaptBuf = (u32)TIM_GetCapture1(TACHO_TIMER);
      #elif defined(TACHO_INPUT_TI2)
        wCaptBuf = (u32)TIM_GetGetCapture2(TACHO_TIMER);
      #endif
      
      hPrscBuf = TACHO_TIMER->PSC;

      while (bGP1_OVF_Counter != 0)
      {
         wCaptBuf += 0x10000uL;// Compute the real captured value (> 16-bit)
         bGP1_OVF_Counter--;
         // OVF Counter is 8-bit and Capt is 16-bit, thus max CaptBuf is 24-bits
      }
      while(wCaptBuf > U16_MAX)
      {
         wCaptBuf /= 2;		// Make it fit 16-bit using virtual prescaler
         // Reduced resolution not a problem since result just slightly < 16-bit
         hPrscBuf = (hPrscBuf * 2) + 1;
         if (hPrscBuf > U16_MAX/2) // Avoid Prsc overflow
         {
            hPrscBuf = U16_MAX;
            wCaptBuf = U16_MAX;
         }
      }
      SensorPeriod[bSpeedFIFO_Index].hCapture = wCaptBuf;
      SensorPeriod[bSpeedFIFO_Index].hPrscReg = hPrscBuf;
      if (RatioInc)
      {
         RatioInc = FALSE;	// Previous capture caused overflow
         // Don't change prescaler (delay due to preload/update mechanism)
      }
      else
      {
         if ((TACHO_TIMER->PSC) < MAX_RATIO) // Avoid OVF w/ very low freq
         {
            (TACHO_TIMER->PSC)++; // To avoid OVF during speed decrease
            RatioInc = TRUE;	  // new prsc value updated at next capture only
         }
      }
   }
   else		// No counter overflow
   {
        u16 hHighSpeedCapture, hClockPrescaler;

      // Read period since last tacho edge
      
      #if defined(TACHO_INPUT_TI1)
        hHighSpeedCapture = (u32)TIM_GetCapture1(TACHO_TIMER);
        
      #elif defined(TACHO_INPUT_TI2)
        hHighSpeedCapture = (u32)TIM_GetGetCapture2(TACHO_TIMER);
      #endif
        
      SensorPeriod[bSpeedFIFO_Index].hCapture = hHighSpeedCapture;
      // Store prescaler directly or incremented if value changed on last capt
      hClockPrescaler = TACHO_TIMER->PSC;

      // If prsc preload reduced in last capture, store current register + 1
      if (RatioDec)  // and don't decrease it again
      {
         SensorPeriod[bSpeedFIFO_Index].hPrscReg = (hClockPrescaler)+1;
         RatioDec = FALSE;
      }
      else  // If prescaler was not modified on previous capture
      {
         if (hHighSpeedCapture >= LOW_RES_THRESHOLD)// If capture range correct
         {
            SensorPeriod[bSpeedFIFO_Index].hPrscReg = hClockPrescaler;
         }
         else
         {
            if(TACHO_TIMER->PSC == 0) // or prescaler cannot be further reduced
            {
               SensorPeriod[bSpeedFIFO_Index].hPrscReg = hClockPrescaler;
            }
            else  // The prescaler needs to be modified to optimize the accuracy
            {
               SensorPeriod[bSpeedFIFO_Index].hPrscReg = hClockPrescaler;
               (TACHO_TIMER->PSC)--;	// Increase accuracy by decreasing prsc
               // Avoid decrementing again in next capt.(register preload delay)
               RatioDec = TRUE;
            }
         }
      }
   }

   if (InitRollingAverage)
   {
        u16 hCaptBuf, hPrscBuf;
        u32 wIndex;
      // Read last captured value and copy it into the whole array
      hCaptBuf = SensorPeriod[bSpeedFIFO_Index].hCapture;
      hPrscBuf = SensorPeriod[bSpeedFIFO_Index].hPrscReg;
      for (wIndex = 0; wIndex<SPEED_FIFO_SIZE-1; wIndex++)
      {
         SensorPeriod[wIndex].hCapture = hCaptBuf;
         SensorPeriod[wIndex].hPrscReg = hPrscBuf;
      }
      InitRollingAverage = FALSE;
      // Starting from now, the values returned by MTC_GetRotorFreq are averaged
      DoRollingAverage = TRUE;
  }
}
else // an update event occured for this interrupt request generation
{
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);

  if (bGP1_OVF_Counter < U8_MAX)
  {
     bGP1_OVF_Counter++;
  }

  if (bGP1_OVF_Counter >= MAX_OVERFLOWS)
  {
     TachoTimeOut = TRUE;
  }
}
}


#endif // TACHO defined

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
