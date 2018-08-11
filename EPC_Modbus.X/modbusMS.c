
#ifdef __cplusplus
extern "C"{
#endif 

#include "modbusMS.h"
    
#include "mcc_generated_files/mcc.h"    
    
#include "main.h"    
    
  

#include <stdlib.h>    



unsigned char u8ID = 0; //!< 0=master, 1..247=slave number
//unsigned char u8serno; //!< serial port: 0-Serial, 1..3-Serial1..Serial3; 4: use software serial
//unsigned char u8txenpin; //!< flow control pin: 0=USB or RS-232 mode, >0=RS-485 mode
unsigned char u8state;
unsigned char u8lastError;
unsigned char au8Buffer[MAX_BUFFER];
unsigned char u8BufferSize;
unsigned char u8lastRec;
unsigned int *au16regs;
unsigned int u16InCnt, u16OutCnt, u16errCnt;
unsigned int u16timeOut;
unsigned long u32time, u32timeOut;
unsigned char u8regsize;


 
  void sendTxBuffer();
  signed char getRxBuffer();
  unsigned int calcCRC(unsigned char u8length);
  unsigned char validateAnswer();
  unsigned char validateRequest();
  void get_FC1();
  void get_FC3();
  signed char process_FC1( unsigned int *regs, unsigned char u8size );
  signed char process_FC3( unsigned int *regs, unsigned char u8size );
  signed char process_FC5( unsigned int *regs, unsigned char u8size );
  signed char process_FC6( unsigned int *regs, unsigned char u8size );
  signed char process_FC15( unsigned int *regs, unsigned char u8size );
  signed char process_FC16( unsigned int *regs, unsigned char u8size );
  void buildException( unsigned char u8exception ); // build exception message

  unsigned int wordConvert(unsigned char hByte,unsigned char lByte);



void Modbusinit(unsigned char u8id)
{
    u8ID = u8id;
//    u8serno = 4;
//    u8txenpin = 0;
    u16timeOut = 1000;
    
    Serialbegin(115200);
    //_SerialBeginFC(9600);

    u8lastRec = u8BufferSize = 0;
    u16InCnt = u16OutCnt = u16errCnt = 0;
    //SerialDebugSend("Modbus initialize...");
    
}

/**
 * @brief
 * *** Only Modbus Master ***
 * Generate a query to an slave with a modbus_t telegram structure
 * The Master must be in COM_IDLE mode. After it, its state would be COM_WAITING.
 * This method has to be called only in loop() section.
 *
 * @see modbus_t
 * @param modbus_t  modbus telegram structure (id, fct, ...)
 * @ingroup loop
 * @todo finish function 15
 */
signed char Modbusquery( modbus_t telegram )
{
    unsigned char u8regsno, u8bytesno;
    //Serialwrite('x');
    if (u8ID!=0) 
    {
    //    Serialwrite('1');
        return -2;
    }
    if (u8state != COM_IDLE) 
    {
    //    Serialwrite('2');
        return -1;
    }

    if ((telegram.u8id==0) || (telegram.u8id>247)) return -3;

    au16regs = telegram.au16reg;
    
    
    // telegram header
    au8Buffer[ ID ]         = telegram.u8id;
    au8Buffer[ FUNC ]       = telegram.u8fct;
    au8Buffer[ ADD_HI ]     = highByte(telegram.u16RegAdd );
    au8Buffer[ ADD_LO ]     = lowByte( telegram.u16RegAdd );

    switch( telegram.u8fct )
    {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUT:
    case MB_FC_READ_REGISTERS:
    case MB_FC_READ_INPUT_REGISTER:
        au8Buffer[ NB_HI ]      = highByte(telegram.u16CoilsNo );
        au8Buffer[ NB_LO ]      = lowByte( telegram.u16CoilsNo );
        u8BufferSize = 6;
        break;
    case MB_FC_WRITE_COIL:
        au8Buffer[ NB_HI ]      = ((au16regs[0] > 0) ? 0xff : 0);
        au8Buffer[ NB_LO ]      = 0;
        u8BufferSize = 6;
        break;
    case MB_FC_WRITE_REGISTER:
        au8Buffer[ NB_HI ]      = highByte(au16regs[0]);
        au8Buffer[ NB_LO ]      = lowByte(au16regs[0]);
        u8BufferSize = 6;
        break;
    case MB_FC_WRITE_MULTIPLE_COILS: // TODO: implement "sending coils"
        u8regsno = telegram.u16CoilsNo / 16;
        u8bytesno = u8regsno * 2;
        if ((telegram.u16CoilsNo % 16) != 0)
        {
            u8bytesno++;
            u8regsno++;
        }

        au8Buffer[ NB_HI ]      = highByte(telegram.u16CoilsNo );
        au8Buffer[ NB_LO ]      = lowByte( telegram.u16CoilsNo );
        au8Buffer[ NB_LO+1 ]    = u8bytesno;
        u8BufferSize = 7;

        u8regsno = u8bytesno = 0; // now auxiliary registers
        for (unsigned int i = 0; i < telegram.u16CoilsNo; i++)
        {


        }
        break;

    case MB_FC_WRITE_MULTIPLE_REGISTERS:
        au8Buffer[ NB_HI ]      = highByte(telegram.u16CoilsNo );
        au8Buffer[ NB_LO ]      = lowByte( telegram.u16CoilsNo );
        au8Buffer[ NB_LO+1 ]    = (unsigned char) ( telegram.u16CoilsNo * 2 );
        u8BufferSize = 7;

        for (unsigned int i=0; i< telegram.u16CoilsNo; i++)
        {
            au8Buffer[ u8BufferSize ] = highByte( au16regs[ i ] );
            u8BufferSize++;
            au8Buffer[ u8BufferSize ] = lowByte( au16regs[ i ] );
            u8BufferSize++;
        }
        break;
    }
    PIE1bits.RCIE = 0;

    sendTxBuffer();
    
    u8state = COM_WAITING;
    
    while(0 == PIR1bits.TXIF)
    {
    }
    PIR1bits.TXIF=0;
//    while(PIE1bits.TXIE == 1)
//    {
//        
//    }    
     
    delayMsec(1);
  //   PIR1bits.RCIF=0;//flush receive buffer
    PIE1bits.RCIE = 1;  
    
    RS485_TXEN_RB6_SetLow();//
    
    ClearSerialRxBuffer();
    //PIE1bits.RCIE = 1;
    
    return 0;
}


/**
 * @brief *** Only for Modbus Master ***
 * This method checks if there is any incoming answer if pending.
 * If there is no answer, it would change Master state to COM_IDLE.
 * This method must be called only at loop section.
 * Avoid any delay() function.
 *
 * Any incoming data would be redirected to au16regs pointer,
 * as defined in its modbus_t query telegram.
 *
 * @params  nothing
 * @return errors counter
 * @ingroup loop
 */
signed char ModbuspollMaster()//( unsigned int *regs, unsigned char u8size )
{

//    au16regs = regs;
//    u8regsize = u8size;
    // check if there is any incoming frame
    unsigned char u8current;
    
    //u8current = _SerialAvailableFC();//Serial.available();
    u8current = Serialavailable();
    ////SerialDebugSend("entry pollmaster...");
    if (millis() > u32timeOut)
    {
        u8state = COM_IDLE;
        u8lastError = NO_REPLY;
        u16errCnt++;
      //  //SerialDebugSend("Error: Timeout u32timeOut..");
        return 0;
    }

    if (u8current == 0) return 0;

    // check T35 after frame end or still no frame end
    if (u8current != u8lastRec)
    {
        u8lastRec = u8current;
        u32time = millis() + T35;
        //SerialDebugSend("Error: u8lastRec not match..");
        return 0;
    }
    if (millis() < u32time) 
    {
      //SerialDebugSend("Error: millis < u32time..");
      return 0;
    }

    // transfer Serial buffer frame to auBuffer
    u8lastRec = 0;
    signed char i8state = getRxBuffer();
    if (i8state < 7)
    {
        u8state = COM_IDLE;
        u16errCnt++;
        //SerialDebugSend("Error: getRxBuffer < 7");
        return i8state;
    }

    // validate message: id, CRC, FCT, exception
    unsigned char u8exception = validateAnswer();
    if (u8exception != 0)
    {
        u8state = COM_IDLE;
        //SerialDebugSend("Error: CRC check fail");
        return u8exception;
    }

    // process answer
    //SerialDebugSend("Processing response************");
    switch( au8Buffer[ FUNC ] )
    {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUT:
        // call get_FC1 to transfer the incoming message to au16regs buffer
        get_FC1( );
        break;
    case MB_FC_READ_INPUT_REGISTER:
    case MB_FC_READ_REGISTERS :
        // call get_FC3 to transfer the incoming message to au16regs buffer
        get_FC3( );
        //SerialDebugSend("read register performed");
        break;
    case MB_FC_WRITE_COIL:
    case MB_FC_WRITE_REGISTER :
    case MB_FC_WRITE_MULTIPLE_COILS:
    case MB_FC_WRITE_MULTIPLE_REGISTERS :
        //SerialDebugSend("write register do nothing");
        // nothing to do
        break;
//    case MB_FC_WRITE_COIL:
//        process_FC5( regs, u8size );
//        break;
//    case MB_FC_WRITE_REGISTER :
//        process_FC6( regs, u8size );
//        break;
//    case MB_FC_WRITE_MULTIPLE_COILS:
//        process_FC15( regs, u8size );
//        break;
//    case MB_FC_WRITE_MULTIPLE_REGISTERS :
//        process_FC16( regs, u8size );
//        break;
    default:
        break;
    }
    u8state = COM_IDLE;
    return u8BufferSize;
}

/**
 * @brief
 * *** Only for Modbus Slave ***
 * This method checks if there is any incoming query
 * Afterwards, it would shoot a validation routine plus a register query
 * Avoid any delay() function !!!!
 * After a successful frame between the Master and the Slave, the time-out timer is reset.
 *
 * @param *regs  register table for communication exchange
 * @param u8size  size of the register table
 * @return 0 if no query, 1..4 if communication error, >4 if correct query processed
 * @ingroup loop
 */
signed char ModbuspollSlave( unsigned int *regs, unsigned char u8size )
{

    au16regs = regs;
    u8regsize = u8size;
  unsigned char u8current;


    // check if there is any incoming frame
    //u8current = _SerialAvailableFC();//Serial.available();
    u8current = Serialavailable();

    if (u8current == 0) return 0;

    // check T35 after frame end or still no frame end
    if (u8current != u8lastRec)
    {
        u8lastRec = u8current;
        u32time = millis() + T35;
        return 0;
    }
    if (millis() < u32time) return 0;

    u8lastRec = 0;
    signed char i8state = getRxBuffer();
    u8lastError = i8state;
    if (i8state < 7) return i8state;

    // check slave id
    if (au8Buffer[ ID ] != u8ID) return 0;

    // validate message: CRC, FCT, address and size
    unsigned char u8exception = validateRequest();
    if (u8exception > 0)
    {
        if (u8exception != NO_REPLY)
        {
            buildException( u8exception );
            sendTxBuffer();
        }
        u8lastError = u8exception;
        return u8exception;
    }

    u32timeOut = millis() + (long)(u16timeOut);
    u8lastError = 0;

    // process message
    switch( au8Buffer[ FUNC ] )
    {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUT:
        return process_FC1( regs, u8size );
        break;
    case MB_FC_READ_INPUT_REGISTER:
    case MB_FC_READ_REGISTERS :
        return process_FC3( regs, u8size );
        break;
    case MB_FC_WRITE_COIL:
        return process_FC5( regs, u8size );
        break;
    case MB_FC_WRITE_REGISTER :
        return process_FC6( regs, u8size );
        break;
    case MB_FC_WRITE_MULTIPLE_COILS:
        return process_FC15( regs, u8size );
        break;
    case MB_FC_WRITE_MULTIPLE_REGISTERS :
        return process_FC16( regs, u8size );
        break;
    default:
        break;
    }
    return i8state;
}

//##################################private functions of Modbus

/**
 * @brief
 * This method moves Serial buffer data to the Modbus au8Buffer.
 *
 * @return buffer size if OK, ERR_BUFF_OVERFLOW if u8BufferSize >= MAX_BUFFER
 * @ingroup buffer
 */
signed char getRxBuffer()
{
    bool bBuffOverflow = false;

   // if (u8txenpin > 1) digitalWrite( u8txenpin, LOW );

    u8BufferSize = 0;
        //while ( _SerialAvailableFC())//Serial.available() )
        while ( Serialavailable())
        {
            au8Buffer[ u8BufferSize ] = Serialread();//_SerialReadFC(); //port->read();
            //Serialwrite(au8Buffer[ u8BufferSize ]);
            //SerialDebugSendDEC(au8Buffer[ u8BufferSize ]);
            //SerialDebugSend(",");
            u8BufferSize ++;

            if (u8BufferSize >= MAX_BUFFER) bBuffOverflow = true;
        }
        //SerialDebugSend(" ");
    u16InCnt++;

    if (bBuffOverflow)
    {
        u16errCnt++;
        return ERR_BUFF_OVERFLOW;
    }
    return u8BufferSize;
}

/**
 * @brief
 * This method transmits au8Buffer to Serial line.
 * Only if u8txenpin != 0, there is a flow handling in order to keep
 * the RS485 transceiver in output state as long as the message is being sent.
 * This is done with UCSRxA register.
 * The CRC is appended to the buffer before starting to send it.
 *
 * @param nothing
 * @return nothing
 * @ingroup buffer
 */
void sendTxBuffer()
{
    unsigned char i = 0;
    //Serialwrite('s'); 

    PIR1bits.RCIF=0;//flush receive buffer
    PIE1bits.RCIE = 0;
    RS485_TXEN_RB6_SetHigh();
    // append CRC to message
    unsigned int u16crc = calcCRC( u8BufferSize );
    au8Buffer[ u8BufferSize ] = u16crc >> 8;
    u8BufferSize++;
    au8Buffer[ u8BufferSize ] = u16crc & 0x00ff;
    u8BufferSize++;
    if(u8ID != 0 )// issue on master mode 
	{
        au8Buffer[ u8BufferSize ] = '\r';
        u8BufferSize++;
    }    
    // set RS485 transceiver to transmit mode
    //    digitalWrite( u8txenpin, HIGH );
 

    // transfer buffer to serial line
    //Serial.write( au8Buffer, u8BufferSize );

    
    for( i=0;i<u8BufferSize;i++)
       Serialwrite(au8Buffer[i]);// _SerialWriteFC(*(au8Buffer+i));
    //Serialwrite('s');    
    // keep RS485 transceiver in transmit mode as long as sending

        // return RS485 transceiver to receive mode
//        digitalWrite( u8txenpin, LOW );
    
       // while(Serial.read() >= 0);_SerialAvailableFC
    //while(Serialavailable() >= 0);//while(_SerialAvailableFC() >= 0);
    
    u8BufferSize = 0;

    // set time-out for master
    u32timeOut = millis() + (unsigned long) u16timeOut;

    // increase message counter
    u16OutCnt++;
    
	if(u8ID != 0 )// issue on master mode 
	{
 	   PIR1bits.RCIF=0;//flush receive buffer
 	   PIE1bits.RCIE = 1;
	   RS485_TXEN_RB6_SetLow();
 	   ClearSerialRxBuffer();
	}    
}

/**
 * @brief
 * This method calculates CRC
 *
 * @return unsigned int calculated CRC value for the message
 * @ingroup buffer
 */
unsigned int calcCRC(unsigned char u8length)
{
    unsigned int temp, temp2, flag;
    temp = 0xFFFF;
    for (unsigned char i = 0; i < u8length; i++)
    {
        temp = temp ^ au8Buffer[i];
        for (unsigned char j = 1; j <= 8; j++)
        {
            flag = temp & 0x0001;
            temp >>=1;
            if (flag)
                temp ^= 0xA001;
        }
    }
    // Reverse byte order.
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;
    // the returned value is already swapped
    // crcLo byte is first & crcHi byte is last
    return temp;
}

/**
 * @brief
 * This method validates slave incoming messages
 *
 * @return 0 if OK, EXCEPTION if anything fails
 * @ingroup buffer
 */
unsigned char validateRequest()
{
    // check message crc vs calculated crc
    unsigned int u16MsgCRC =
        ((au8Buffer[u8BufferSize - 2] << 8)
         | au8Buffer[u8BufferSize - 1]); // combine the crc Low & High bytes
    if ( calcCRC( u8BufferSize-2 ) != u16MsgCRC )
    {
        u16errCnt ++;
        return NO_REPLY;
    }

    // check fct code
    bool isSupported = false;
    for (unsigned char i = 0; i< sizeof( fctsupported ); i++)
    {
        if (fctsupported[i] == au8Buffer[FUNC])
        {
            isSupported = 1;
            break;
        }
    }
    if (!isSupported)
    {
        u16errCnt ++;
        return EXC_FUNC_CODE;
    }

    // check start address & nb range
    unsigned int u16regs = 0;
    unsigned char u8regs;
    switch ( au8Buffer[ FUNC ] )
    {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUT:
    case MB_FC_WRITE_MULTIPLE_COILS:
        u16regs = wordConvert( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ]) / 16;
        u16regs += wordConvert( au8Buffer[ NB_HI ], au8Buffer[ NB_LO ]) /16;
        u8regs = (unsigned char) u16regs;
        if (u8regs > u8regsize) return EXC_ADDR_RANGE;
        break;
    case MB_FC_WRITE_COIL:
        u16regs = wordConvert( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ]) / 16;
        u8regs = (unsigned char) u16regs;
        if (u8regs > u8regsize) return EXC_ADDR_RANGE;
        break;
    case MB_FC_WRITE_REGISTER :
        u16regs = wordConvert( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ]);
        u8regs = (unsigned char) u16regs;
        if (u8regs > u8regsize) return EXC_ADDR_RANGE;
        break;
    case MB_FC_READ_REGISTERS :
    case MB_FC_READ_INPUT_REGISTER :
    case MB_FC_WRITE_MULTIPLE_REGISTERS :
        u16regs = wordConvert( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ]);
        u16regs += wordConvert( au8Buffer[ NB_HI ], au8Buffer[ NB_LO ]);
        u8regs = (unsigned char) u16regs;
        if (u8regs > u8regsize) return EXC_ADDR_RANGE;
        break;
    }
    return 0; // OK, no exception code thrown
}

/**
 * @brief
 * This method validates master incoming messages
 *
 * @return 0 if OK, EXCEPTION if anything fails
 * @ingroup buffer
 */
unsigned char validateAnswer()
{
    // check message crc vs calculated crc
    unsigned int u16MsgCRC=0,u16MsgCRCcalc=0;
    u16MsgCRC =((au8Buffer[u8BufferSize - 2] << 8) | au8Buffer[u8BufferSize - 1]); // combine the crc Low & High bytes
    u16MsgCRCcalc=calcCRC( u8BufferSize-2 );
    if ( u16MsgCRCcalc != u16MsgCRC )
    {
        u16errCnt ++;
        return NO_REPLY;
    }

    // check exception
    if ((au8Buffer[ FUNC ] & 0x80) != 0)
    {
        u16errCnt ++;
        return ERR_EXCEPTION;
    }

    // check fct code
    bool isSupported = false;
    for (unsigned char i = 0; i< sizeof( fctsupported ); i++)
    {
        if (fctsupported[i] == au8Buffer[FUNC])
        {
            isSupported = 1;
            break;
        }
    }
    if (!isSupported)
    {
        u16errCnt ++;
        return EXC_FUNC_CODE;
    }

    return 0; // OK, no exception code thrown
}

/**
 * @brief
 * This method builds an exception message
 *
 * @ingroup buffer
 */
void buildException( unsigned char u8exception )
{
    unsigned char u8func = au8Buffer[ FUNC ];  // get the original FUNC code

    au8Buffer[ ID ]      = u8ID;
    au8Buffer[ FUNC ]    = u8func + 0x80;
    au8Buffer[ 2 ]       = u8exception;
    u8BufferSize         = EXCEPTION_SIZE;
}

/**
 * This method processes functions 1 & 2 (for master)
 * This method puts the slave answer into master data buffer
 *
 * @ingroup register
 * TODO: finish its implementation
 */
void get_FC1()
{
    unsigned char u8byte, i;
    u8byte = 0;

    for (i=0; i< (au8Buffer[2]/2); i++) 
    {
      au16regs[ i ] = wordConvert(au8Buffer[u8byte],au8Buffer[u8byte+1]);
      u8byte += 2;
    }
}

/**
 * This method processes functions 3 & 4 (for master)
 * This method puts the slave answer into master data buffer
 *
 * @ingroup register
 */
void get_FC3()
{
    unsigned char u8byte, i;
    u8byte = 3;

    for (i=0; i< (au8Buffer[2]/2); i++)
    {
        au16regs[ i ] = wordConvert(au8Buffer[u8byte],au8Buffer[u8byte+1]);
        //SerialDebugSendDEC(au16regs[ i ]);
        //SerialDebugSend(":");                    
        u8byte += 2;
    }
}

/**
 * @brief
 * This method processes functions 1 & 2
 * This method reads a bit array and transfers it to the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup discrete
 */
signed char process_FC1( unsigned int *regs, unsigned char u8size )
{
    unsigned char u8currentRegister, u8currentBit, u8bytesno, u8bitsno;
    unsigned char u8CopyBufferSize;
    unsigned int u16currentCoil, u16coil;

    // get the first and last coil from the message
    unsigned int u16StartCoil=0;
    unsigned int u16Coilno=0;
    u16StartCoil = wordConvert( au8Buffer[ADD_HI], au8Buffer[ADD_LO] );
    u16Coilno = wordConvert( au8Buffer[NB_HI], au8Buffer[NB_LO] );
  
    // put the number of bytes in the outcoming message
    u8bytesno = (unsigned char) (u16Coilno / 8);
    if (u16Coilno % 8 != 0) u8bytesno ++;
    au8Buffer[ ADD_HI ]  = u8bytesno;
    u8BufferSize         = ADD_LO;

    // read each coil from the register map and put its value inside the outcoming message
    u8bitsno = 0;

    for (u16currentCoil = 0; u16currentCoil < u16Coilno; u16currentCoil++)
    {
        u16coil = u16StartCoil + u16currentCoil;
        u8currentRegister = (unsigned char) (u16coil / 16);
        u8currentBit = (unsigned char) (u16coil % 16);

        bitWrite(
            au8Buffer[ u8BufferSize ],
            u8bitsno,
            bitRead( regs[ u8currentRegister ], u8currentBit ) );
        u8bitsno ++;

        if (u8bitsno > 7)
        {
            u8bitsno = 0;
            u8BufferSize++;
        }
    }

    // send outcoming message
    if (u16Coilno % 8 != 0) u8BufferSize ++;
    u8CopyBufferSize = u8BufferSize +2;
    sendTxBuffer();
    return u8CopyBufferSize;
}

/**
 * @brief
 * This method processes functions 3 & 4
 * This method reads a word array and transfers it to the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup register
 */
signed char process_FC3( unsigned int *regs, unsigned char u8size )
{

    unsigned char u8StartAdd = wordConvert( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ] );
    unsigned char u8regsno = wordConvert( au8Buffer[ NB_HI ], au8Buffer[ NB_LO ] );
    unsigned char u8CopyBufferSize;
    unsigned char i;

    au8Buffer[ 2 ]       = u8regsno * 2;
    u8BufferSize         = 3;

    for (i = u8StartAdd; i < u8StartAdd + u8regsno; i++)
    {
        au8Buffer[ u8BufferSize ] = highByte(regs[i]);
        u8BufferSize++;
        au8Buffer[ u8BufferSize ] = lowByte(regs[i]);
        u8BufferSize++;
    }
    u8CopyBufferSize = u8BufferSize +2;
    sendTxBuffer();

    return u8CopyBufferSize;
}

/**
 * @brief
 * This method processes function 5
 * This method writes a value assigned by the master to a single bit
 *
 * @return u8BufferSize Response to master length
 * @ingroup discrete
 */
signed char process_FC5( unsigned int *regs, unsigned char u8size )
{
    unsigned char u8currentRegister, u8currentBit;
    unsigned char u8CopyBufferSize;
    unsigned int u16coil = wordConvert( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ] );

    // point to the register and its bit
    u8currentRegister = (unsigned char) (u16coil / 16);
    u8currentBit = (unsigned char) (u16coil % 16);

    // write to coil
    bitWrite(
        regs[ u8currentRegister ],
        u8currentBit,
        au8Buffer[ NB_HI ] == 0xff );


    // send answer to master
    u8BufferSize = 6;
    u8CopyBufferSize = u8BufferSize +2;
    sendTxBuffer();

    return u8CopyBufferSize;
}

/**
 * @brief
 * This method processes function 6
 * This method writes a value assigned by the master to a single word
 *
 * @return u8BufferSize Response to master length
 * @ingroup register
 */
signed char process_FC6( unsigned int *regs, unsigned char u8size )
{

    unsigned char u8add = wordConvert( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ] );
    unsigned char u8CopyBufferSize;
    unsigned int u16val = wordConvert( au8Buffer[ NB_HI ], au8Buffer[ NB_LO ] );

    *(regs +u8add) = u16val;

    // keep the same header
    u8BufferSize  = RESPONSE_SIZE;

    u8CopyBufferSize = u8BufferSize +2;
    sendTxBuffer();

    return u8CopyBufferSize;
}

/**
 * @brief
 * This method processes function 15
 * This method writes a bit array assigned by the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup discrete
 */
signed char process_FC15( unsigned int *regs, unsigned char u8size )
{
    unsigned char u8currentRegister, u8currentBit, u8frameByte, u8bitsno;
    unsigned char u8CopyBufferSize;
    unsigned int u16currentCoil, u16coil;
    bool bTemp;

    // get the first and last coil from the message
    unsigned int u16StartCoil = wordConvert( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ] );
    unsigned int u16Coilno = wordConvert( au8Buffer[ NB_HI ], au8Buffer[ NB_LO ] );


    // read each coil from the register map and put its value inside the outcoming message
    u8bitsno = 0;
    u8frameByte = 7;
    for (u16currentCoil = 0; u16currentCoil < u16Coilno; u16currentCoil++)
    {

        u16coil = u16StartCoil + u16currentCoil;
        u8currentRegister = (unsigned char) (u16coil / 16);
        u8currentBit = (unsigned char) (u16coil % 16);

        bTemp = bitRead(*(au8Buffer +u8frameByte),u8bitsno );

        bitWrite(*(regs+u8currentRegister),u8currentBit,bTemp );

        u8bitsno ++;

        if (u8bitsno > 7)
        {
            u8bitsno = 0;
            u8frameByte++;
        }
    }

    // send outcoming message
    // it's just a copy of the incomping frame until 6th byte
    u8BufferSize         = 6;
    u8CopyBufferSize = u8BufferSize +2;
    sendTxBuffer();
    return u8CopyBufferSize;
}

/**
 * @brief
 * This method processes function 16
 * This method writes a word array assigned by the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup register
 */
signed char process_FC16( unsigned int *regs, unsigned char u8size )
{
    unsigned char u8func = au8Buffer[ FUNC ];  // get the original FUNC code
    unsigned char u8StartAdd = au8Buffer[ ADD_HI ] << 8 | au8Buffer[ ADD_LO ];
    unsigned char u8regsno = au8Buffer[ NB_HI ] << 8 | au8Buffer[ NB_LO ];
    unsigned char u8CopyBufferSize;
    unsigned char i;
    unsigned int temp;

    // build header
    au8Buffer[ NB_HI ]   = 0;
    au8Buffer[ NB_LO ]   = u8regsno;
    u8BufferSize         = RESPONSE_SIZE;

    // write registers
    for (i = 0; i < u8regsno; i++)
    {
        temp = wordConvert(*(au8Buffer + (BYTE_CNT + 1) + (i * 2)), *(au8Buffer +(BYTE_CNT + 2) + (i * 2)));

        *(regs+ u8StartAdd + i)= temp;
    }
    u8CopyBufferSize = u8BufferSize +2;
    sendTxBuffer();

    return u8CopyBufferSize;
}

//#########################################################
/**
 * @brief
 * Method to write a new slave ID address
 *
 * @param   u8id  new slave address between 1 and 247
 * @ingroup setup
 */
void ModbussetID( unsigned char u8id)
{
    if (( u8id != 0) && (u8id <= 247))
    {
        u8id = u8id;
    }
}

/**
 * @brief
 * Method to read current slave ID address
 *
 * @return u8id current slave address between 1 and 247
 * @ingroup setup
 */
unsigned char ModbusgetID()
{
    return u8ID;
}

/**
 * @brief
 * Initialize time-out parameter
 *
 * Call once class has been instantiated, typically within setup().
 * The time-out timer is reset each time that there is a successful communication
 * between Master and Slave. It works for both.
 *
 * @param time-out value (ms)
 * @ingroup setup
 */
void ModbussetTimeOut( unsigned int u16timeOut)
{
    u16timeOut = u16timeOut;
}

/**
 * @brief
 * Return communication Watchdog state.
 * It could be usefull to reset outputs if the watchdog is fired.
 *
 * @return TRUE if millis() > u32timeOut
 * @ingroup loop
 */
bool ModbusgetTimeOutState()
{
    return (millis() > u32timeOut);
}

/**
 * @brief
 * Get input messages counter value
 * This can be useful to diagnose communication
 *
 * @return input messages counter
 * @ingroup buffer
 */
unsigned int ModbusgetInCnt()
{
    return u16InCnt;
}

/**
 * @brief
 * Get transmitted messages counter value
 * This can be useful to diagnose communication
 *
 * @return transmitted messages counter
 * @ingroup buffer
 */
unsigned int ModbusgetOutCnt()
{
    return u16OutCnt;
}

/**
 * @brief
 * Get errors counter value
 * This can be useful to diagnose communication
 *
 * @return errors counter
 * @ingroup buffer
 */
unsigned int ModbusgetErrCnt()
{
    return u16errCnt;
}

/**
 * Get modbus master state
 *
 * @return = 0 IDLE, = 1 WAITING FOR ANSWER
 * @ingroup buffer
 */
unsigned char ModbusgetState()
{
    return u8state;
}

/**
 * Get the last error in the protocol processor
 *
 * @returnreturn   NO_REPLY = 255      Time-out
 * @return   EXC_FUNC_CODE = 1   Function code not available
 * @return   EXC_ADDR_RANGE = 2  Address beyond available space for Modbus registers
 * @return   EXC_REGS_QUANT = 3  Coils or registers number beyond the available space
 * @ingroup buffer
 */
unsigned char ModbusgetLastError()
{
    return u8lastError;
}

unsigned int wordConvert(unsigned char hByte,unsigned char lByte)
{
  return (((unsigned int)hByte) << 8 )+ (unsigned int)lByte;
}



#ifdef __cplusplus
}
#endif  

