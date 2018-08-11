/* 
 * File:   RingBuffer.h
 * Author: paull
 *
 * Created on September 12, 2017, 12:32 AM
 */

#ifndef RINGBUFFER_H
#define	RINGBUFFER_H

#include "mcc_generated_files/mcc.h"

#ifdef	__cplusplus
extern "C" {
#endif
    
#define RINGBUFFERSIZE  32
    
#define DISABLE_ISR     INTERRUPT_GlobalInterruptDisable()   
#define ENABLE_ISR      INTERRUPT_GlobalInterruptEnable()    
    
    
typedef struct
{
    unsigned int PulseTime64[RINGBUFFERSIZE];
    unsigned char PulseState[RINGBUFFERSIZE];
    unsigned char count;
    unsigned char overflow;
    unsigned char readIndexptr;
    unsigned char writeIndexptr;
    unsigned char maxoverflowLatch;
    
}RingBufferTypes;

//extern void InitRingBuffer(void);
//extern void WriteRingBuffer(unsigned int pulsetime);//call on ISR, producer
//extern unsigned char ReadRingBuffer(unsigned int *pulsetime);//call on while, consumer

void InitRingBuffer(void);
void ResetOverflowLatch(void);
unsigned char ReadOverflowLatch(void);
void WriteRingBuffer(unsigned int pulsetime,unsigned char pulsestate);//call on ISR, producer
unsigned char ReadRingBuffer(unsigned int *pulsetime,unsigned char *pulsestate);//call on while, consumer

#ifdef	__cplusplus
}
#endif

#endif	/* RINGBUFFER_H */

