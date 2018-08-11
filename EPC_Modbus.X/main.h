/* 
 * File:   main.h
 * Author: kpit
 *
 * Created on February 18, 2017, 4:46 PM
 */

#ifndef MAIN_H
#define	MAIN_H

#ifdef	__cplusplus
extern "C" {
#endif

//Interrupt Function callback
void Timer0_tick10msecFunc(void);    
void Timer3_Overflow(void);
void EncA_PulseEdgeEvent(unsigned int capturedValue);

unsigned long millis(void); //arduino Like implementation for miilis())
void delayMsec(unsigned long);
void millisReset(void);//arduino Like implementation for miilis())

#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */

