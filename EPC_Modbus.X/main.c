/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB(c) Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 4.15
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

#include "mcc_generated_files/mcc.h"
//#include "ModbusManager.h"
#include "modbusMS.h"
#include "LED_Ctrl.h"
#include "main.h"
#include "RingBuffer.h"

#define MODBUSSLAVE    1//0    //if 0 - master , if >0, MODBUSSLAVE value itself is slave address and it is in Slave Mode
/*
 * PLC Function/Algorithm (Modbus Master)
 * 1. Get Set Batch Setting from HMI (NoOfPulse in revolution, Nut Length in mm, Pitch Length in mm, Nut run time, Direction of rotation,rearm delay time)
 * 2. Calculate Set Pulse Count.
 * 3. Communicate PulseInRevolution,SetTotalPulseRun,SetTotalRunTime,SetRunDirection to MCU through Modbus.
 * 4. Switch ON the solenoid(Air Valve).
 * 5. (If Trigger sensor (pressure sensor) installed), if trigger is detected,
 *    OR (If Trigger sensor NOT installed), detect trigger through modbus (ModbusData.mcuTOplcData.NutRunState == RUNNING) 
 *           Start shutoff timer.
 * 6. When Shutoff Timer equal SetTotalRunTime, Switch OFF the solenoid(Air Valve).
 * 7. Read nut run result () from Modbus.
 * 8. After rearm delay time, repeat from 4.
 */
/*
 * MCU side algorithm (Modbus Slave)
 * Prior to nut run - PLC inform Nut run pulse count, run time, direction, no of pulse in revolution
 *  1. If shaft move, Encoder detect by Encoder Pulse. Note we use only Encoder Pulse A.
 *  2. If shaft stop, Timer 3 (tick at 1 usec) detect by timeout of no pulse edge in 65.536 msec.
 *     32 pulse per rev, i.e In 65.536 msec = x rpm = 60/(32*0.065536)  ~ 30 rpm
 *  3. On every edge Pulse is counted, check Encoder A and B state to detect direction, and Total Run Time is accumulated, while shaft moves.
 *  4. If Shaft stop, Pulse count and Total Run Time are copied for PLC and reset to zero.
 *  5. If shaft stop and Pulse count is greater than half of set pulse (min+max/2)
 *     Inform PLC - OK or NOK
 *          OK  :  Actual Pulse Count within min and max pulse count given by PLC.
 *                 Actual Total Runtime within limit.
 *                 Run direction is correct. 
 *         NOK  :  Above OK condition not satisfy
 */


enum {
		IDLE_STOP = 0, 
        RUNNING,
        STOP_OK,
        STOP_NOK, 
};
enum{
    CLOCKWISE=0,
    ANTICLOCKWISE,
};

typedef struct
{
    uint16_t MeasurePulseCount;  //40000
    uint16_t MeasureRunTime;//40001
    uint16_t NutRunState; ////40002, 0 - IDLE/Stop, 1 - Running, 2 - STOP_OK, 3- STOP_NOK, 
    uint16_t NutRunDirection; //40003, 0 - CLOCKWISE, 1- ANTICLOCKWISE
}MCUtoPLC_Type;

typedef struct
{
    uint16_t PulseInRevolution;//40004// Can be change by PLC, if encoder pulse per rev change
    uint16_t SetTotalPulseRun;//40005
    uint16_t SetTotalRunTime;//40006
    uint16_t SetRunDirection;//40008
}PLCtoMCU_Type;



typedef struct
{
MCUtoPLC_Type mcuTOplcData;    
PLCtoMCU_Type plcTOmcuData;
}ModbusDataType;

ModbusDataType ModbusData;

/*
                         Main application
 */

#define GETPULSESTATE() (((ENC_A_RC5_GetValue()) + (ENC_B_RC4_GetValue()<<1))& 0x03)

/*  GETPULSESTATE()     ENCA    ENCB
 *      0                0       0   //FORWARD CLOCK WISE
 *      1                0       1
 *      3                1       1
 *      2                1       0
 *   
 *      0                0       0   //REVERSE ANTI CLOCKWISE
 *      2                1       0
 *      3                1       1
 *      1                0       1    
 */
                
unsigned int capturePulseusec;
unsigned char PulseStatus=0;

unsigned int EPCPulseCountClock=0;
unsigned int EPCPulseCountAntiClock=0;
unsigned long EPCRunTime=0;

unsigned long TimertickMsec=0;

unsigned int delayCount=0;
bool delayFlag=0;

unsigned char u8state_t;
unsigned char u8query; //!< pointer to message query
modbus_t telegram[2];

unsigned long u32wait;

void OnEncPulseEdgeEvent(uint16_t capturevalue);

void delayMsec(unsigned long);


unsigned char valuetocharHighNibble(unsigned char hexvalue);
unsigned char valuetocharLowNibble(unsigned char hexvalue);
unsigned char valtoasciichar(unsigned char hexvalue1);

void main(void)
{
    // initialize the device
    SYSTEM_Initialize();


    TMR3_WriteTimer(0);//reset TMR3
    TMR3_StopTimer();//Stop TMR3

 
    INLVLC = 0xFF;
    INLVLA = 0xFF;
  
    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

//    InitRingBuffer();

    
    LedONStartConfig(LED_RED, 1, 1);
    while(LedONStatusBusy());//wait for LED blink     
    LedONStartConfig(LED_GREEN, 1, 1);
    while(LedONStatusBusy());//wait for LED blink 
    LedONStartConfig(LED_BLUE, 1, 1);
    while(LedONStatusBusy());//wait for LED blink    
    
    RS485_TXEN_RB6_SetLow();
    
    telegram[0].u8id = 1; // slave address
    telegram[0].u8fct = MB_FC_READ_REGISTERS;//3; // function code (this one is registers read)
    telegram[0].u16RegAdd = 0;//40000; // start address in slave
    telegram[0].u16CoilsNo = sizeof(MCUtoPLC_Type)/2;//3; // number of elements (coils or registers) to read
    telegram[0].au16reg = (uint16_t *)&(ModbusData.plcTOmcuData ); // pointer to a memory array 
    // telegram 1: write a single register
    telegram[1].u8id = 1; // slave address
    telegram[1].u8fct = MB_FC_WRITE_MULTIPLE_REGISTERS;//MB_FC_WRITE_REGISTER;//6; // function code (this one is write a single register)
    telegram[1].u16RegAdd = sizeof(MCUtoPLC_Type)/2;//40006; // start address in slave
    telegram[1].u16CoilsNo = sizeof(PLCtoMCU_Type)/2; // number of elements (coils or registers) to read
    telegram[1].au16reg = (uint16_t *)&(ModbusData.mcuTOplcData); // pointer to a memory array  

    
#if MODBUSSLAVE == 0   
    Modbusinit(0);//Master mode init if 0
    u32wait = millis() + 250;
    u8state_t = 0;
    u8query= 0;
    ModbussetTimeOut( 1000 );

    RS485_TXEN_RB6_SetHigh();//Enable transmit
    
#else    
    Modbusinit(MODBUSSLAVE);//Slave Mode
    RS485_TXEN_RB6_SetLow();//Disable transmit
    
#endif    
    
    FAN_RC6_SetLow();//ON the solenoid valve, 

    ModbusData.mcuTOplcData.NutRunState = IDLE_STOP; 
    ModbusData.mcuTOplcData.MeasurePulseCount = 0;
    ModbusData.mcuTOplcData.MeasureRunTime = 0;
    ModbusData.mcuTOplcData.NutRunDirection = CLOCKWISE;
    
    //Default PLC configuration
    ModbusData.plcTOmcuData.PulseInRevolution = 32;
    ModbusData.plcTOmcuData.SetRunDirection = CLOCKWISE;
    ModbusData.plcTOmcuData.SetTotalPulseRun = 32*3;
    ModbusData.plcTOmcuData.SetTotalRunTime = 3000; 
    
    while (1)
    {
    
        #if MODBUSSLAVE == 0 
        if((millis() > u32wait))// && (ModbusData.mcuTOplcData.NutRunState != RUNNING))
        {
            if (u8query > 1) 
                u8query = 0;
            
            if(u8query==0)
            {
                Modbusquery( telegram[u8query] ); // send query
                ModbuspollMaster(); // check incoming messages
            }
            else if(u8query==1)
            {
                Modbusquery( telegram[u8query] ); // send query  
                ModbuspollMaster(); // check incoming messages
            }    

            u8query++;
            u32wait = millis() + 250;
        }  
        #else
            ModbuspollSlave((unsigned int*)&ModbusData,(unsigned char)(sizeof(ModbusDataType)/2));
        #endif
    }
}

//Interrupt trigger function call
void EncA_PulseEdgeEvent(uint16_t capturedValue)// ISR callback function
{
    //Note: encoder A and B edge capture share TIMER3
    BUZZER_RC3_SetHigh();//for debug
    // Disable the CCP1 interrupt
    PIE4bits.CCP1IE = 0;
 
    OnEncPulseEdgeEvent(capturedValue);
//    // Disable the CCP1 interrupt
    PIE4bits.CCP1IE = 1;

    BUZZER_RC3_SetLow();//for debug
    
}

void Timer3_Overflow(void)
{
    //Timer 3: tick @1usec, Reset on ENcoder A rising and falling edge
    TMR3_StopTimer();
    LED_BLUE_RC2_SetLow();

    if(ModbusData.mcuTOplcData.NutRunState == RUNNING)
    {   
        if(EPCPulseCountClock >= EPCPulseCountAntiClock)
            ModbusData.mcuTOplcData.MeasurePulseCount = EPCPulseCountClock - EPCPulseCountAntiClock;
        else
            ModbusData.mcuTOplcData.MeasurePulseCount = EPCPulseCountAntiClock - EPCPulseCountClock;
        
        ModbusData.mcuTOplcData.MeasureRunTime = (uint16_t)(EPCRunTime/1000);//in msec from usec count
        
        if(EPCPulseCountClock > 0)
            ModbusData.mcuTOplcData.NutRunDirection = CLOCKWISE;
        
        EPCRunTime = 0;
        EPCPulseCountClock = 0;
        EPCPulseCountAntiClock = 0;

        //INDICATION
        LED_GREEN_RC1_SetLow();
        LED_BLUE_RC2_SetLow();
        LED_RED_RC0_SetLow();
        
        if((ModbusData.mcuTOplcData.MeasurePulseCount > (ModbusData.plcTOmcuData.SetTotalPulseRun - ModbusData.plcTOmcuData.PulseInRevolution))
                && (ModbusData.mcuTOplcData.MeasurePulseCount < (ModbusData.plcTOmcuData.SetTotalPulseRun + ModbusData.plcTOmcuData.PulseInRevolution)))
        {
            ModbusData.mcuTOplcData.NutRunState = STOP_OK;
            //INDICATION
            LedONStartConfig(LED_GREEN, 1, 5);
        }
        else
        {
            ModbusData.mcuTOplcData.NutRunState = STOP_NOK;
            //INDICATION
            LedONStartConfig(LED_RED, 1, 5);
        } 
    }    
}

void OnEncPulseEdgeEvent(uint16_t capturevalue)//executed on ISR on each edges of encoder A and B
{
    //NOTE: Only capture even of Encoder A pulse falling and rising, Encoder B is use to check Pulse state (for Direction detection)
    LED_BLUE_RC2_SetHigh();

    TMR3_StopTimer();//PulsestatusRead
    capturePulseusec = TMR3_ReadTimer();
    TMR3_WriteTimer(0);
    TMR3_StartTimer();//Start if not started TMR3 
    PulseStatus = GETPULSESTATE();
  
    if(capturePulseusec>5)//avoid reading of timer tick due interrupt latency
    {
        EPCRunTime += capturePulseusec;
        if(PulseStatus==0 || PulseStatus==3)
            EPCPulseCountClock++;
        else if(PulseStatus==1 || PulseStatus==2)
            EPCPulseCountAntiClock++;
        
        if(ModbusData.mcuTOplcData.NutRunState != RUNNING )
        {   // Start Movement
            //Reset Modbus Parameter
            ModbusData.mcuTOplcData.MeasurePulseCount = 0;
            ModbusData.mcuTOplcData.MeasureRunTime = 0;
            if(EPCPulseCountClock >= EPCPulseCountAntiClock)
                ModbusData.mcuTOplcData.NutRunDirection = CLOCKWISE;
            else
                ModbusData.mcuTOplcData.NutRunDirection = ANTICLOCKWISE;
            EPCRunTime = 0;
            EPCPulseCountClock = 0;
            EPCPulseCountAntiClock = 0;
        }    

        ModbusData.mcuTOplcData.NutRunState = RUNNING;
        ModbusData.mcuTOplcData.MeasureRunTime = EPCRunTime;
    }
}

void Timer0_tick10msecFunc(void)//call every 10msec but due to MCC bug it works at 5 msec
{
    LedON_Control_10msec();
    //LED_RED_RC0_Toggle();
    TimertickMsec= TimertickMsec+10;
    //EUSART_Write('U');
    if(delayFlag==1)
        delayCount+=10;
}

unsigned long millis(void) //arduino Like implementation for miilis())
{
    return TimertickMsec;
}

void millisReset(void)//arduino Like implementation for miilis())
{
    TimertickMsec=0;
}

void delayMsec(unsigned long timems)
{
    delayCount=0;
    delayFlag=1;
    while(delayCount < timems);
    delayFlag=0;
}

unsigned char valuetocharHighNibble(unsigned char hexvalue)
{
    unsigned char hexval;
    hexval=(unsigned char)((unsigned char)(hexvalue>>4) & 0x0F);
    hexval= valtoasciichar(hexval);
    return hexval;
    
}

unsigned char valuetocharLowNibble(unsigned char hexvalue)
{
    unsigned char hexval;
    hexval=(unsigned char)((unsigned char)(hexvalue) & 0x0F);
    hexval= valtoasciichar(hexval); 
    return hexval;
}


unsigned char valtoasciichar(unsigned char hexvalue1)
{   
    unsigned char hexval='0';
    switch((unsigned char)hexvalue1)
    {
        case 0x00: hexval= '0'; break;
        case 0x01: hexval= '1'; break;
        case 0x02: hexval= '2'; break;
        case 0x03: hexval= '3'; break;
        case 0x04: hexval= '4'; break;
        case 0x05: hexval= '5'; break;
        case 0x06: hexval= '6'; break;
        case 0x07: hexval= '7'; break;
        case 0x08: hexval= '8'; break;
        case 0x09: hexval= '9'; break;
        case 0x0A: hexval= 'A'; break;
        case 0x0B: hexval= 'B'; break;
        case 0x0C: hexval= 'C'; break;
        case 0x0D: hexval= 'D'; break;
        case 0x0E: hexval= 'E'; break;
        case 0x0F: hexval= 'F'; break;
        default : break;             
    }
    return hexval;
}

void Serialbegin(unsigned long baud)
{
  EUSART_Initialize();
}
uint8_t Serialavailable(void)
{
  return eusartRxCount;
   //return (bool)PIR1bits.RCIF;
}
void ClearSerialRxBuffer(void)
{
  unsigned char bdummy;
  while(eusartRxCount)
  {
      bdummy=EUSART_Read();
  }  
  eusartRxCount=0;  
   //return (bool)PIR1bits.RCIF;
}


uint8_t Serialread(void)
{
  return EUSART_Read();
}
void Serialwrite(uint8_t temp)
{

    PIR1bits.TXIF=0; 
    TX1REG = temp; 
    while(0 == PIR1bits.TXIF)
    {
    }
}

/**
 End of File
*/