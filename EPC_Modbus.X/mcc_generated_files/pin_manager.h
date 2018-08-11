/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB(c) Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 4.15.3
        Device            :  PIC16F18344
        Version           :  1.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40

    Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

    Microchip licenses to you the right to use, modify, copy and distribute
    Software only when embedded on a Microchip microcontroller or digital signal
    controller that is integrated into your product or third party product
    (pursuant to the sublicense terms in the accompanying license agreement).

    You should refer to the license agreement accompanying this Software for
    additional information regarding your rights and obligations.

    SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
    EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
    MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
    IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
    CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
    OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
    CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
    SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

*/


#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set RB4 procedures
#define RB4_SetHigh()    do { LATBbits.LATB4 = 1; } while(0)
#define RB4_SetLow()   do { LATBbits.LATB4 = 0; } while(0)
#define RB4_Toggle()   do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define RB4_GetValue()         PORTBbits.RB4
#define RB4_SetDigitalInput()   do { TRISBbits.TRISB4 = 1; } while(0)
#define RB4_SetDigitalOutput()  do { TRISBbits.TRISB4 = 0; } while(0)
#define RB4_SetPullup()     do { WPUBbits.WPUB4 = 1; } while(0)
#define RB4_ResetPullup()   do { WPUBbits.WPUB4 = 0; } while(0)
#define RB4_SetAnalogMode() do { ANSELBbits.ANSB4 = 1; } while(0)
#define RB4_SetDigitalMode()do { ANSELBbits.ANSB4 = 0; } while(0)

// get/set RB5 procedures
#define RB5_SetHigh()    do { LATBbits.LATB5 = 1; } while(0)
#define RB5_SetLow()   do { LATBbits.LATB5 = 0; } while(0)
#define RB5_Toggle()   do { LATBbits.LATB5 = ~LATBbits.LATB5; } while(0)
#define RB5_GetValue()         PORTBbits.RB5
#define RB5_SetDigitalInput()   do { TRISBbits.TRISB5 = 1; } while(0)
#define RB5_SetDigitalOutput()  do { TRISBbits.TRISB5 = 0; } while(0)
#define RB5_SetPullup()     do { WPUBbits.WPUB5 = 1; } while(0)
#define RB5_ResetPullup()   do { WPUBbits.WPUB5 = 0; } while(0)
#define RB5_SetAnalogMode() do { ANSELBbits.ANSB5 = 1; } while(0)
#define RB5_SetDigitalMode()do { ANSELBbits.ANSB5 = 0; } while(0)

// get/set RS485_TXEN_RB6 aliases
#define RS485_TXEN_RB6_TRIS               TRISBbits.TRISB6
#define RS485_TXEN_RB6_LAT                LATBbits.LATB6
#define RS485_TXEN_RB6_PORT               PORTBbits.RB6
#define RS485_TXEN_RB6_WPU                WPUBbits.WPUB6
#define RS485_TXEN_RB6_OD                ODCONBbits.ODCB6
#define RS485_TXEN_RB6_ANS                ANSELBbits.ANSB6
#define RS485_TXEN_RB6_SetHigh()            do { LATBbits.LATB6 = 1; } while(0)
#define RS485_TXEN_RB6_SetLow()             do { LATBbits.LATB6 = 0; } while(0)
#define RS485_TXEN_RB6_Toggle()             do { LATBbits.LATB6 = ~LATBbits.LATB6; } while(0)
#define RS485_TXEN_RB6_GetValue()           PORTBbits.RB6
#define RS485_TXEN_RB6_SetDigitalInput()    do { TRISBbits.TRISB6 = 1; } while(0)
#define RS485_TXEN_RB6_SetDigitalOutput()   do { TRISBbits.TRISB6 = 0; } while(0)
#define RS485_TXEN_RB6_SetPullup()      do { WPUBbits.WPUB6 = 1; } while(0)
#define RS485_TXEN_RB6_ResetPullup()    do { WPUBbits.WPUB6 = 0; } while(0)
#define RS485_TXEN_RB6_SetPushPull()    do { ODCONBbits.ODCB6 = 1; } while(0)
#define RS485_TXEN_RB6_SetOpenDrain()   do { ODCONBbits.ODCB6 = 0; } while(0)
#define RS485_TXEN_RB6_SetAnalogMode()  do { ANSELBbits.ANSB6 = 1; } while(0)
#define RS485_TXEN_RB6_SetDigitalMode() do { ANSELBbits.ANSB6 = 0; } while(0)

// get/set LED_RED_RC0 aliases
#define LED_RED_RC0_TRIS               TRISCbits.TRISC0
#define LED_RED_RC0_LAT                LATCbits.LATC0
#define LED_RED_RC0_PORT               PORTCbits.RC0
#define LED_RED_RC0_WPU                WPUCbits.WPUC0
#define LED_RED_RC0_OD                ODCONCbits.ODCC0
#define LED_RED_RC0_ANS                ANSELCbits.ANSC0
#define LED_RED_RC0_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define LED_RED_RC0_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define LED_RED_RC0_Toggle()             do { LATCbits.LATC0 = ~LATCbits.LATC0; } while(0)
#define LED_RED_RC0_GetValue()           PORTCbits.RC0
#define LED_RED_RC0_SetDigitalInput()    do { TRISCbits.TRISC0 = 1; } while(0)
#define LED_RED_RC0_SetDigitalOutput()   do { TRISCbits.TRISC0 = 0; } while(0)
#define LED_RED_RC0_SetPullup()      do { WPUCbits.WPUC0 = 1; } while(0)
#define LED_RED_RC0_ResetPullup()    do { WPUCbits.WPUC0 = 0; } while(0)
#define LED_RED_RC0_SetPushPull()    do { ODCONCbits.ODCC0 = 1; } while(0)
#define LED_RED_RC0_SetOpenDrain()   do { ODCONCbits.ODCC0 = 0; } while(0)
#define LED_RED_RC0_SetAnalogMode()  do { ANSELCbits.ANSC0 = 1; } while(0)
#define LED_RED_RC0_SetDigitalMode() do { ANSELCbits.ANSC0 = 0; } while(0)

// get/set LED_GREEN_RC1 aliases
#define LED_GREEN_RC1_TRIS               TRISCbits.TRISC1
#define LED_GREEN_RC1_LAT                LATCbits.LATC1
#define LED_GREEN_RC1_PORT               PORTCbits.RC1
#define LED_GREEN_RC1_WPU                WPUCbits.WPUC1
#define LED_GREEN_RC1_OD                ODCONCbits.ODCC1
#define LED_GREEN_RC1_ANS                ANSELCbits.ANSC1
#define LED_GREEN_RC1_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define LED_GREEN_RC1_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define LED_GREEN_RC1_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define LED_GREEN_RC1_GetValue()           PORTCbits.RC1
#define LED_GREEN_RC1_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define LED_GREEN_RC1_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define LED_GREEN_RC1_SetPullup()      do { WPUCbits.WPUC1 = 1; } while(0)
#define LED_GREEN_RC1_ResetPullup()    do { WPUCbits.WPUC1 = 0; } while(0)
#define LED_GREEN_RC1_SetPushPull()    do { ODCONCbits.ODCC1 = 1; } while(0)
#define LED_GREEN_RC1_SetOpenDrain()   do { ODCONCbits.ODCC1 = 0; } while(0)
#define LED_GREEN_RC1_SetAnalogMode()  do { ANSELCbits.ANSC1 = 1; } while(0)
#define LED_GREEN_RC1_SetDigitalMode() do { ANSELCbits.ANSC1 = 0; } while(0)

// get/set LED_BLUE_RC2 aliases
#define LED_BLUE_RC2_TRIS               TRISCbits.TRISC2
#define LED_BLUE_RC2_LAT                LATCbits.LATC2
#define LED_BLUE_RC2_PORT               PORTCbits.RC2
#define LED_BLUE_RC2_WPU                WPUCbits.WPUC2
#define LED_BLUE_RC2_OD                ODCONCbits.ODCC2
#define LED_BLUE_RC2_ANS                ANSELCbits.ANSC2
#define LED_BLUE_RC2_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define LED_BLUE_RC2_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define LED_BLUE_RC2_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define LED_BLUE_RC2_GetValue()           PORTCbits.RC2
#define LED_BLUE_RC2_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define LED_BLUE_RC2_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)
#define LED_BLUE_RC2_SetPullup()      do { WPUCbits.WPUC2 = 1; } while(0)
#define LED_BLUE_RC2_ResetPullup()    do { WPUCbits.WPUC2 = 0; } while(0)
#define LED_BLUE_RC2_SetPushPull()    do { ODCONCbits.ODCC2 = 1; } while(0)
#define LED_BLUE_RC2_SetOpenDrain()   do { ODCONCbits.ODCC2 = 0; } while(0)
#define LED_BLUE_RC2_SetAnalogMode()  do { ANSELCbits.ANSC2 = 1; } while(0)
#define LED_BLUE_RC2_SetDigitalMode() do { ANSELCbits.ANSC2 = 0; } while(0)

// get/set BUZZER_RC3 aliases
#define BUZZER_RC3_TRIS               TRISCbits.TRISC3
#define BUZZER_RC3_LAT                LATCbits.LATC3
#define BUZZER_RC3_PORT               PORTCbits.RC3
#define BUZZER_RC3_WPU                WPUCbits.WPUC3
#define BUZZER_RC3_OD                ODCONCbits.ODCC3
#define BUZZER_RC3_ANS                ANSELCbits.ANSC3
#define BUZZER_RC3_SetHigh()            do { LATCbits.LATC3 = 1; } while(0)
#define BUZZER_RC3_SetLow()             do { LATCbits.LATC3 = 0; } while(0)
#define BUZZER_RC3_Toggle()             do { LATCbits.LATC3 = ~LATCbits.LATC3; } while(0)
#define BUZZER_RC3_GetValue()           PORTCbits.RC3
#define BUZZER_RC3_SetDigitalInput()    do { TRISCbits.TRISC3 = 1; } while(0)
#define BUZZER_RC3_SetDigitalOutput()   do { TRISCbits.TRISC3 = 0; } while(0)
#define BUZZER_RC3_SetPullup()      do { WPUCbits.WPUC3 = 1; } while(0)
#define BUZZER_RC3_ResetPullup()    do { WPUCbits.WPUC3 = 0; } while(0)
#define BUZZER_RC3_SetPushPull()    do { ODCONCbits.ODCC3 = 1; } while(0)
#define BUZZER_RC3_SetOpenDrain()   do { ODCONCbits.ODCC3 = 0; } while(0)
#define BUZZER_RC3_SetAnalogMode()  do { ANSELCbits.ANSC3 = 1; } while(0)
#define BUZZER_RC3_SetDigitalMode() do { ANSELCbits.ANSC3 = 0; } while(0)

// get/set ENC_B_RC4 aliases
#define ENC_B_RC4_TRIS               TRISCbits.TRISC4
#define ENC_B_RC4_LAT                LATCbits.LATC4
#define ENC_B_RC4_PORT               PORTCbits.RC4
#define ENC_B_RC4_WPU                WPUCbits.WPUC4
#define ENC_B_RC4_OD                ODCONCbits.ODCC4
#define ENC_B_RC4_ANS                ANSELCbits.ANSC4
#define ENC_B_RC4_SetHigh()            do { LATCbits.LATC4 = 1; } while(0)
#define ENC_B_RC4_SetLow()             do { LATCbits.LATC4 = 0; } while(0)
#define ENC_B_RC4_Toggle()             do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define ENC_B_RC4_GetValue()           PORTCbits.RC4
#define ENC_B_RC4_SetDigitalInput()    do { TRISCbits.TRISC4 = 1; } while(0)
#define ENC_B_RC4_SetDigitalOutput()   do { TRISCbits.TRISC4 = 0; } while(0)
#define ENC_B_RC4_SetPullup()      do { WPUCbits.WPUC4 = 1; } while(0)
#define ENC_B_RC4_ResetPullup()    do { WPUCbits.WPUC4 = 0; } while(0)
#define ENC_B_RC4_SetPushPull()    do { ODCONCbits.ODCC4 = 1; } while(0)
#define ENC_B_RC4_SetOpenDrain()   do { ODCONCbits.ODCC4 = 0; } while(0)
#define ENC_B_RC4_SetAnalogMode()  do { ANSELCbits.ANSC4 = 1; } while(0)
#define ENC_B_RC4_SetDigitalMode() do { ANSELCbits.ANSC4 = 0; } while(0)

// get/set ENC_A_RC5 aliases
#define ENC_A_RC5_TRIS               TRISCbits.TRISC5
#define ENC_A_RC5_LAT                LATCbits.LATC5
#define ENC_A_RC5_PORT               PORTCbits.RC5
#define ENC_A_RC5_WPU                WPUCbits.WPUC5
#define ENC_A_RC5_OD                ODCONCbits.ODCC5
#define ENC_A_RC5_ANS                ANSELCbits.ANSC5
#define ENC_A_RC5_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define ENC_A_RC5_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define ENC_A_RC5_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define ENC_A_RC5_GetValue()           PORTCbits.RC5
#define ENC_A_RC5_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define ENC_A_RC5_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
#define ENC_A_RC5_SetPullup()      do { WPUCbits.WPUC5 = 1; } while(0)
#define ENC_A_RC5_ResetPullup()    do { WPUCbits.WPUC5 = 0; } while(0)
#define ENC_A_RC5_SetPushPull()    do { ODCONCbits.ODCC5 = 1; } while(0)
#define ENC_A_RC5_SetOpenDrain()   do { ODCONCbits.ODCC5 = 0; } while(0)
#define ENC_A_RC5_SetAnalogMode()  do { ANSELCbits.ANSC5 = 1; } while(0)
#define ENC_A_RC5_SetDigitalMode() do { ANSELCbits.ANSC5 = 0; } while(0)

// get/set FAN_RC6 aliases
#define FAN_RC6_TRIS               TRISCbits.TRISC6
#define FAN_RC6_LAT                LATCbits.LATC6
#define FAN_RC6_PORT               PORTCbits.RC6
#define FAN_RC6_WPU                WPUCbits.WPUC6
#define FAN_RC6_OD                ODCONCbits.ODCC6
#define FAN_RC6_ANS                ANSELCbits.ANSC6
#define FAN_RC6_SetHigh()            do { LATCbits.LATC6 = 1; } while(0)
#define FAN_RC6_SetLow()             do { LATCbits.LATC6 = 0; } while(0)
#define FAN_RC6_Toggle()             do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define FAN_RC6_GetValue()           PORTCbits.RC6
#define FAN_RC6_SetDigitalInput()    do { TRISCbits.TRISC6 = 1; } while(0)
#define FAN_RC6_SetDigitalOutput()   do { TRISCbits.TRISC6 = 0; } while(0)
#define FAN_RC6_SetPullup()      do { WPUCbits.WPUC6 = 1; } while(0)
#define FAN_RC6_ResetPullup()    do { WPUCbits.WPUC6 = 0; } while(0)
#define FAN_RC6_SetPushPull()    do { ODCONCbits.ODCC6 = 1; } while(0)
#define FAN_RC6_SetOpenDrain()   do { ODCONCbits.ODCC6 = 0; } while(0)
#define FAN_RC6_SetAnalogMode()  do { ANSELCbits.ANSC6 = 1; } while(0)
#define FAN_RC6_SetDigitalMode() do { ANSELCbits.ANSC6 = 0; } while(0)

// get/set EXTRA_OUT_RC7 aliases
#define EXTRA_OUT_RC7_TRIS               TRISCbits.TRISC7
#define EXTRA_OUT_RC7_LAT                LATCbits.LATC7
#define EXTRA_OUT_RC7_PORT               PORTCbits.RC7
#define EXTRA_OUT_RC7_WPU                WPUCbits.WPUC7
#define EXTRA_OUT_RC7_OD                ODCONCbits.ODCC7
#define EXTRA_OUT_RC7_ANS                ANSELCbits.ANSC7
#define EXTRA_OUT_RC7_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define EXTRA_OUT_RC7_SetLow()             do { LATCbits.LATC7 = 0; } while(0)
#define EXTRA_OUT_RC7_Toggle()             do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define EXTRA_OUT_RC7_GetValue()           PORTCbits.RC7
#define EXTRA_OUT_RC7_SetDigitalInput()    do { TRISCbits.TRISC7 = 1; } while(0)
#define EXTRA_OUT_RC7_SetDigitalOutput()   do { TRISCbits.TRISC7 = 0; } while(0)
#define EXTRA_OUT_RC7_SetPullup()      do { WPUCbits.WPUC7 = 1; } while(0)
#define EXTRA_OUT_RC7_ResetPullup()    do { WPUCbits.WPUC7 = 0; } while(0)
#define EXTRA_OUT_RC7_SetPushPull()    do { ODCONCbits.ODCC7 = 1; } while(0)
#define EXTRA_OUT_RC7_SetOpenDrain()   do { ODCONCbits.ODCC7 = 0; } while(0)
#define EXTRA_OUT_RC7_SetAnalogMode()  do { ANSELCbits.ANSC7 = 1; } while(0)
#define EXTRA_OUT_RC7_SetDigitalMode() do { ANSELCbits.ANSC7 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);


/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handler for the IOCCF4 pin functionality
 * @Example
    IOCCF4_ISR();
 */
void IOCCF4_ISR(void);

/**
  @Summary
    Interrupt Handler Setter for IOCCF4 pin interrupt-on-change functionality

  @Description
    Allows selecting an interrupt handler for IOCCF4 at application runtime
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    InterruptHandler function pointer.

  @Example
    PIN_MANAGER_Initialize();
    IOCCF4_SetInterruptHandler(MyInterruptHandler);

*/
void IOCCF4_SetInterruptHandler(void* InterruptHandler);

/**
  @Summary
    Dynamic Interrupt Handler for IOCCF4 pin

  @Description
    This is a dynamic interrupt handler to be used together with the IOCCF4_SetInterruptHandler() method.
    This handler is called every time the IOCCF4 ISR is executed and allows any function to be registered at runtime.
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCCF4_SetInterruptHandler(IOCCF4_InterruptHandler);

*/
extern void (*IOCCF4_InterruptHandler)(void);

/**
  @Summary
    Default Interrupt Handler for IOCCF4 pin

  @Description
    This is a predefined interrupt handler to be used together with the IOCCF4_SetInterruptHandler() method.
    This handler is called every time the IOCCF4 ISR is executed. 
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCCF4_SetInterruptHandler(IOCCF4_DefaultInterruptHandler);

*/
void IOCCF4_DefaultInterruptHandler(void);


/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handler for the IOCCF5 pin functionality
 * @Example
    IOCCF5_ISR();
 */
void IOCCF5_ISR(void);

/**
  @Summary
    Interrupt Handler Setter for IOCCF5 pin interrupt-on-change functionality

  @Description
    Allows selecting an interrupt handler for IOCCF5 at application runtime
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    InterruptHandler function pointer.

  @Example
    PIN_MANAGER_Initialize();
    IOCCF5_SetInterruptHandler(MyInterruptHandler);

*/
void IOCCF5_SetInterruptHandler(void* InterruptHandler);

/**
  @Summary
    Dynamic Interrupt Handler for IOCCF5 pin

  @Description
    This is a dynamic interrupt handler to be used together with the IOCCF5_SetInterruptHandler() method.
    This handler is called every time the IOCCF5 ISR is executed and allows any function to be registered at runtime.
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCCF5_SetInterruptHandler(IOCCF5_InterruptHandler);

*/
extern void (*IOCCF5_InterruptHandler)(void);

/**
  @Summary
    Default Interrupt Handler for IOCCF5 pin

  @Description
    This is a predefined interrupt handler to be used together with the IOCCF5_SetInterruptHandler() method.
    This handler is called every time the IOCCF5 ISR is executed. 
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCCF5_SetInterruptHandler(IOCCF5_DefaultInterruptHandler);

*/
void IOCCF5_DefaultInterruptHandler(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/