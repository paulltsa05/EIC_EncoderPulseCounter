#include <stdlib.h>
#include "stdbool.h"


#ifdef __cplusplus
extern "C"{
#endif 



extern void SerialDebugSend(const char *dat);
extern void SerialDebugSendDEC(int dat);
#define word(x,y) (((unsigned int)x)<<8 + y)


/**
 * @struct modbus_t
 * @brief
 * Master query structure:
 * This includes all the necessary fields to make the Master generate a Modbus query.
 * A Master may keep several of these structures and send them cyclically or
 * use them according to program needs.
 */
typedef struct
{
    unsigned char u8id;          /*!< Slave address between 1 and 247. 0 means broadcast */
    unsigned char u8fct;         /*!< Function code: 1, 2, 3, 4, 5, 6, 15 or 16 */
    unsigned int u16RegAdd;    /*!< Address of the first register to access at slave/s */
    unsigned int u16CoilsNo;   /*!< Number of coils or registers to access */
    unsigned int *au16reg;     /*!< Pointer to memory image in master */
}
modbus_t;

enum
{
    RESPONSE_SIZE = 6,
    EXCEPTION_SIZE = 3,
    CHECKSUM_SIZE = 2
};

/**
 * @enum MESSAGE
 * @brief
 * Indexes to telegram frame positions
 */
enum MESSAGE
{
    ID                             = 0, //!< ID field
    FUNC, //!< Function code position
    ADD_HI, //!< Address high byte
    ADD_LO, //!< Address low byte
    NB_HI, //!< Number of coils or registers high byte
    NB_LO, //!< Number of coils or registers low byte
    BYTE_CNT  //!< byte counter
};

/**
 * @enum MB_FC
 * @brief
 * Modbus function codes summary.
 * These are the implement function codes either for Master or for Slave.
 *
 * @see also fctsupported
 * @see also modbus_t
 */
enum MB_FC
{
    MB_FC_NONE                     = 0,   /*!< null operator */
    MB_FC_READ_COILS               = 1,  /*!< FCT=1 -> read coils or digital outputs */
    MB_FC_READ_DISCRETE_INPUT      = 2, /*!< FCT=2 -> read digital inputs */
    MB_FC_READ_REGISTERS           = 3, /*!< FCT=3 -> read registers or analog outputs */
    MB_FC_READ_INPUT_REGISTER      = 4, /*!< FCT=4 -> read analog inputs */
    MB_FC_WRITE_COIL               = 5, /*!< FCT=5 -> write single coil or output */
    MB_FC_WRITE_REGISTER           = 6, /*!< FCT=6 -> write single register */
    MB_FC_WRITE_MULTIPLE_COILS     = 15,  /*!< FCT=15 -> write multiple coils or outputs */
    MB_FC_WRITE_MULTIPLE_REGISTERS = 16 /*!< FCT=16 -> write multiple registers */
};

enum COM_STATES
{
    COM_IDLE                     = 0,
    COM_WAITING                  = 1

};

enum ERR_LIST
{
    ERR_NOT_MASTER                = -1,
    ERR_POLLING                   = -2,
    ERR_BUFF_OVERFLOW             = -3,
    ERR_BAD_CRC                   = -4,
    ERR_EXCEPTION                 = -5
};

enum
{
    NO_REPLY = 255,
    EXC_FUNC_CODE = 1,
    EXC_ADDR_RANGE = 2,
    EXC_REGS_QUANT = 3,
    EXC_EXECUTE = 4
};

const unsigned char fctsupported[] =
{
    MB_FC_READ_COILS,
    MB_FC_READ_DISCRETE_INPUT,
    MB_FC_READ_REGISTERS,
    MB_FC_READ_INPUT_REGISTER,
    MB_FC_WRITE_COIL,
    MB_FC_WRITE_REGISTER,
    MB_FC_WRITE_MULTIPLE_COILS,
    MB_FC_WRITE_MULTIPLE_REGISTERS
};

#define T35  50
#define  MAX_BUFFER  32 //!< maximum size for the communication buffer in bytes

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define lowByte(w) ((unsigned char) ((w) & 0xff))
#define highByte(w) ((unsigned char) ((w) >> 8))

//void Modbusinit(unsigned char u8id,  void (*)(unsigned long), unsigned char (*)(),unsigned char (*)(),void (*)(unsigned char));
void Modbusinit(unsigned char u8id);
 
void ModbussetTimeOut(unsigned int u16timeout); //!<write communication watch-dog timer
unsigned int ModbusgetTimeOut(); //!<get communication watch-dog timer value
bool ModbusgetTimeOutState(); //!<get communication watch-dog timer state
signed char Modbusquery( modbus_t telegram ); //!<only for master
signed char ModbuspollMaster();//( unsigned int *regs, unsigned char u8size ); //!<cyclic poll for master
signed char ModbuspollSlave( unsigned int *regs, unsigned char u8size ); //!<cyclic poll for slave
unsigned int ModbusgetInCnt(); //!<number of incoming messages
unsigned int ModbusgetOutCnt(); //!<number of outcoming messages
unsigned int ModbusgetErrCnt(); //!<error counter
unsigned char ModbusgetID(); //!<get slave ID between 1 and 247
unsigned char ModbusgetState();
unsigned char ModbusgetLastError(); //!<get last error message
void ModbussetID( unsigned char u8id ); //!<write new ID for the slave
//void end(); //!<finish any communication and release serial communication port


//  void (*_SerialBeginFC)(unsigned long);
//  unsigned char (*_SerialAvailableFC)(void);
//  unsigned char (*_SerialReadFC)(void);
//  void (*_SerialWriteFC)(unsigned char);

extern void Serialbegin(unsigned long baud);
extern unsigned char Serialavailable(void);
extern void ClearSerialRxBuffer(void);
extern unsigned char Serialread(void);
extern void Serialwrite(unsigned char temp);



#ifdef __cplusplus
}
#endif  
