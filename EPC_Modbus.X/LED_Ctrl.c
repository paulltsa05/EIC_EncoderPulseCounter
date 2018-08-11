
#include <xc.h>     
#include "mcc_generated_files/mcc.h"
#include "LED_Ctrl.h"

//Variables and function for LedON
static unsigned char Led_ON=0,LED_Color=LED_GREEN;
static unsigned int LED_msec_Cnt=0,Blick_tickPointMsec=0,Blick_tickPoint,LedON_Duration_msec=0;

//char LedONStartConfig(unsigned char LEDcolor,unsigned char LedONDurationSec, unsigned char LedONNumBlink);
//void LedON_Control_msec(void);
char LedONStatusBusy(void)
{
    return Led_ON;
}

//NOTE: LedONDurationSec < 60 
char LedONStartConfig(unsigned char LEDcolor, unsigned char LedONDurationSec, unsigned char LedONNumBlink)
{
    if(Led_ON!=1 && (LedONDurationSec!=0)&& (LedONDurationSec <= 60))
    {   
        Led_ON=1;
        LED_Color=LEDcolor;
        
        switch(LED_Color)//Set LED to ON
        {
            case LED_RED    :   LED_RED_RC0_SetHigh();
                                break;
            case LED_GREEN  :   LED_GREEN_RC1_SetHigh();
                                break;
            case LED_BLUE   :   LED_BLUE_RC2_SetHigh();
                                break;
            default         :   break;
        }
        
        LED_msec_Cnt=0;
        LedON_Duration_msec=((unsigned int)LedONDurationSec)*1000;
        if(LedONNumBlink > 1)
            Blick_tickPointMsec=(LedON_Duration_msec/(((LedONNumBlink-1)*2)+1));
        else
            Blick_tickPointMsec=LedON_Duration_msec;
        
        Blick_tickPoint=Blick_tickPointMsec;
        return 1;
    }
    else
    {
        return 0;//error
    }
        
}

// call void LedON_Control_msec(void) function in 10msec
void LedON_Control_10msec(void)
{
    if(1==Led_ON)//check if Enable
    {   
      
        if(LED_msec_Cnt >= Blick_tickPoint)//toggle timer ON/OFF at beep points msec
        {
            switch(LED_Color)//Toggle LED
            {
                case LED_RED    :   LED_RED_RC0_Toggle();
                                    break;
                case LED_GREEN  :   LED_GREEN_RC1_Toggle();
                                    break;
                case LED_BLUE   :   LED_BLUE_RC2_Toggle();
                                    break;
                default         :   break;
            }

            
            Blick_tickPoint=Blick_tickPoint+Blick_tickPointMsec;//increment for next beep
            
        }
        
        if(LED_msec_Cnt > LedON_Duration_msec)//Stop at duration expire
        {
            switch(LED_Color)//Set LED to OFF
            {
                case LED_RED    :   LED_RED_RC0_SetLow();
                                    break;
                case LED_GREEN  :   LED_GREEN_RC1_SetLow();
                                    break;
                case LED_BLUE   :   LED_BLUE_RC2_SetLow();
                                    break;
                default         :   break;
            }
            
            Led_ON=0;
            LED_msec_Cnt=0;
        }
        
        LED_msec_Cnt+=10;//since tick happen at 10msec, due to bug in MCC replace 10 with 5 msec.
    }
      
    
}

