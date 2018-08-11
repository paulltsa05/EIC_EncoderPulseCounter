/**
  CCP1 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    ccp1.c

  @Summary
    This is the generated driver implementation file for the CCP1 driver using MPLAB(c) Code Configurator

  @Description
    This source file provides APIs for CCP1.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 4.15.3
        Device            :  PIC16F18344
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/

#include <xc.h>
#include "ccp1.h"
#include "../main.h"

/**
  Section: Capture Module APIs:
*/

void CCP1_Initialize(void)
{
    // Set the CCP1 to the options selected in the User Interface

    // CCP1MODE Every edge; CCP1EN enabled; CCP1FMT right_aligned; 
    CCP1CON = 0x83;    

    // CCP1CTS0 CCP1 pin; 
    CCP1CAP = 0x00;    

    // CCPR1H 0; 
    CCPR1H = 0x00;    

    // CCPR1L 0; 
    CCPR1L = 0x00;    

    // Selecting Timer 3
    CCPTMRSbits.C1TSEL = 0x2;

    // Clear the CCP1 interrupt flag
    PIR4bits.CCP1IF = 0;

    // Enable the CCP1 interrupt
    PIE4bits.CCP1IE = 1;
}

void CCP1_CaptureISR(void)
{
    CCP_PERIOD_REG_T module;

    // Clear the CCP1 interrupt flag
    PIR4bits.CCP1IF = 0;
    
    // Copy captured value.
    module.ccpr1l = CCPR1L;
    module.ccpr1h = CCPR1H;
    
    // Return 16bit captured value
    CCP1_CallBack(module.ccpr1_16Bit);
}

void CCP1_CallBack(uint16_t capturedValue)
{
    // Add your code here
    EncA_PulseEdgeEvent(capturedValue);
}
/**
 End of File
*/