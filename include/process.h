/*
 * procecc.h
 *
 *  Created on: 7 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#ifndef PROCESS_H_
#define PROCESS_H_

#include "stm32l0xx.h"
#include "my_time.h"


extern volatile tUxTime sendTryStopTime;

void mesureStart( void );
void wutIrqHandler( void );
int8_t dataSendTry( void );
void csmaRun( void );
void csmaPause( void );

inline void deepSleepOn( void ){
  // STOP mode (SLEEPDEEP) enable
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
}
inline void deepSleepOff( void ){
  // STOP mode (SLEEPDEEP) disable
  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
}


#endif /* PROCESS_H_ */
