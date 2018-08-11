#include "RingBuffer.h"



static RingBufferTypes Rbuff;




void InitRingBuffer(void)
{
    for(int i=0;i<RINGBUFFERSIZE;i++)
    {
        Rbuff.PulseTime64[i]=0;
        Rbuff.PulseState[i]=0;
    }   
    Rbuff.readIndexptr=0;
    Rbuff.writeIndexptr=0;
    Rbuff.count=0;
    Rbuff.overflow=0;
    Rbuff.maxoverflowLatch=0;
}

void ResetOverflowLatch(void)
{
    Rbuff.maxoverflowLatch=0;
}

unsigned char ReadOverflowLatch(void)
{
    return Rbuff.maxoverflowLatch;
}

void WriteRingBuffer(unsigned int pulsetime,unsigned char pulsestate)
{
    if(Rbuff.writeIndexptr==RINGBUFFERSIZE)
    {
        Rbuff.writeIndexptr=0;
    }    
    if(Rbuff.count>=RINGBUFFERSIZE)
    {
        Rbuff.overflow++;
        if(Rbuff.maxoverflowLatch <Rbuff.overflow)
            Rbuff.maxoverflowLatch=Rbuff.overflow;
        Rbuff.count--;
    }    
    else
    {
        Rbuff.overflow=0;
    }    
    Rbuff.PulseTime64[Rbuff.writeIndexptr]=pulsetime;
    Rbuff.PulseState[Rbuff.writeIndexptr]=pulsestate;
    Rbuff.writeIndexptr++;
    Rbuff.count++;
}

unsigned char ReadRingBuffer(unsigned int *pulsetime,unsigned char *pulsestate)
{
    unsigned int overflowcnt;
    if(Rbuff.count!=0)
    { 
        overflowcnt=Rbuff.overflow;
        if(overflowcnt!=0)
        { 
            while(overflowcnt--)//move to last un overwritten location
            {
                if(Rbuff.readIndexptr==0)
                    Rbuff.readIndexptr=RINGBUFFERSIZE;
                Rbuff.readIndexptr--;
            }    
        }
        *pulsetime= Rbuff.PulseTime64[Rbuff.readIndexptr]; 
        *pulsestate= Rbuff.PulseState[Rbuff.readIndexptr]; 
        Rbuff.readIndexptr++;
        if(Rbuff.readIndexptr==RINGBUFFERSIZE)
            Rbuff.readIndexptr=0;
        DISABLE_ISR;//disable interrupt ****CRITICAL SECTION****
        Rbuff.count--; 
        ENABLE_ISR;//enable interrupt
        return 0;
    }    
    else
    {
        return 1;
    }
}
