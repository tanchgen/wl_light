﻿/*
 * time.c
 *
 *  Created on: 31 окт. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#include "stm32l0xx.h"

#include "my_time.h"

#define BIN2BCD(__VALUE__) (uint8_t)((((__VALUE__) / 10U) << 4U) | ((__VALUE__) % 10U))
#define BCD2BIN(__VALUE__) (uint8_t)(((uint8_t)((__VALUE__) & (uint8_t)0xF0U) >> (uint8_t)0x4U) * 10U + ((__VALUE__) & (uint8_t)0x0FU))

volatile tRtc rtc;
volatile tXtime uxTime;
uint8_t secondFlag = RESET;

static inline void RTC_SetTime( tRtc * prtc );
static inline void RTC_SetDate( tRtc * prtc );
static inline void RTC_GetTime( tRtc * prtc );
static inline void RTC_GetDate( tRtc * prtc );

// *********** Инициализация структуры ВРЕМЯ (сейчас - системное ) ************
inline void rtcConfig(void){
  // Enable the LSE
  RCC->CSR |= RCC_CSR_LSEON;
  // Wait while it is not ready
  while( (RCC->CSR & RCC_CSR_LSERDY) != RCC_CSR_LSERDY)
  {}

  PWR->CR |= PWR_CR_DBP;
  // LSE enable for RTC clock
  RCC->CSR = (RCC->CSR & ~RCC_CSR_RTCSEL) | RCC_CSR_RTCSEL_0 | RCC_CSR_RTCEN;

  // --- Configure Alarm A -----
  // Write access for RTC registers
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  // Disable alarm A to modify it
  RTC->CR &=~ RTC_CR_ALRAE;
  while((RTC->ISR & RTC_ISR_ALRAWF) != RTC_ISR_ALRAWF)
  {}

  RTC->ALRMAR = 0L;
  // Alarm A each 1day
  RTC->ALRMAR = RTC_ALRMAR_MSK4;

  RTC->CR = RTC_CR_ALRAIE | RTC_CR_ALRAE;
  // Disable write access
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;

  // Configure exti and nvic for RTC ALARM IT
  EXTI->IMR |= EXTI_IMR_IM17;
  // Rising edge for line 17
  EXTI->RTSR |= EXTI_RTSR_TR17;
  NVIC_SetPriority(RTC_IRQn, 0);
  NVIC_EnableIRQ(RTC_IRQn);


}

inline void rtcInit(uint32_t Time){

  // Write access for RTC registers
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  // Enable init phase
  RTC->ISR = RTC_ISR_INIT;
  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
  {}

  // RTCCLOCK deviser
  RTC->PRER = 0x007F00FF;
  RTC->TR = RTC_TR_PM | Time;
  RTC->ISR =~ RTC_ISR_INIT;
  // Disable write access for RTC registers
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}


void timeInit( void ) {

  /*##-1- Configure the Date #################################################*/
  /* Set Date: Wednesday June 1st 2016 */
  rtc.year = 17;
  rtc.month = 12;
  rtc.date = 15;
  rtc.wday = 5;
  rtc.hour = 11;
  rtc.min = 40;
  rtc.sec = 0;
  rtc.ss = 0;


  RTC_SetDate( &rtc );
  RTC_SetTime( &rtc );

}

// Получение системного мремени
uint32_t getTick( void ) {
  // Возвращает количество тиков
  return myTick;
}

#define _TBIAS_DAYS   ((70 * (uint32_t)365) + 17)
#define _TBIAS_SECS   (_TBIAS_DAYS * (uint32_t)86400)
#define _TBIAS_YEAR   0
#define MONTAB(year)    ((((year) & 03) || ((year) == 0)) ? mos : lmos)

const int16_t lmos[] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
const int16_t mos[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

#define Daysto32(year, mon) (((year - 1) / 4) + MONTAB(year)[mon])

/////////////////////////////////////////////////////////////////////

tXtime xTm2Utime( tRtc * prtc ){
  /* convert time structure to scalar time */
int32_t   days;
int32_t   secs;
int32_t   mon, year;

  /* Calculate number of days. */
  mon = prtc->month - 1;
  // Годы считаем от 1900г.
  year = (prtc->year + 100) - _TBIAS_YEAR;
  days  = Daysto32(year, mon) - 1;
  days += 365 * year;
  days += prtc->date;
  days -= _TBIAS_DAYS;

  /* Calculate number of seconds. */
  secs  = 3600 * prtc->hour;
  secs += 60 * prtc->min;
  secs += prtc->sec;

  secs += (days * (tXtime)86400);

  return (secs);
}

/////////////////////////////////////////////////////////////////////

void xUtime2Tm( tRtc * prtc, tXtime secsarg){
  uint32_t    secs;
  int32_t   days;
  int32_t   mon;
  int32_t   year;
  int32_t   i;
  const int16_t * pm;

  #ifdef  _XT_SIGNED
  if (secsarg >= 0) {
      secs = (uint32_t)secsarg;
      days = _TBIAS_DAYS;
    } else {
      secs = (uint32_t)secsarg + _TBIAS_SECS;
      days = 0;
    }
  #else
    secs = secsarg;
    days = _TBIAS_DAYS;
  #endif

    /* days, hour, min, sec */
  days += secs / 86400;
  secs = secs % 86400;
  prtc->hour = secs / 3600;
  secs %= 3600;
  prtc->min = secs / 60;
  prtc->sec = secs % 60;

  prtc->wday = (days + 1) % 7;

  /* determine year */
  for (year = days / 365; days < (i = Daysto32(year, 0) + 365*year); ) { --year; }
  days -= i;
  // Годы выставляем от эпохи 2000г., а не 1900г., как в UNIX Time
  prtc->year = (year - 100) + _TBIAS_YEAR;

    /* determine month */
  pm = MONTAB(year);
  for (mon = 12; days < pm[--mon]; );
  prtc->month = mon + 1;
  prtc->date = days - pm[mon] + 1;
}

void setRtcTime( tXtime xtime ){

  xUtime2Tm( &rtc, xtime);
  RTC_SetTime( &rtc );
  RTC_SetDate( &rtc );
}

tXtime getRtcTime( void ){

  RTC_GetTime( &rtc );
  RTC_GetDate( &rtc );
  return xTm2Utime( &rtc );
}

/* Установка будильника
 *  xtime - UNIX-времени
 *  alrm - номере будильника
 */
void setAlrm( tXtime xtime, uint8_t alrm ){
  tRtc tmpRtc;

  xUtime2Tm( &tmpRtc, xtime);
  RTC_SetAlrm( &tmpRtc, alrm );
}

/* Получениевремени будильника
 *  alrm - номере будильника
 *  Возвращает - UNIX-время
 */
tXtime getRtcTime( uint8_t alrm ){
  tRtc tmpRtc;

  RTC_GetAlrm( &tmpRtc, alrm );
  return xTm2Utime( &tmpRtc );
}


void timersHandler( void ) {

#if 0
  // Таймаут timerCount1
  if ( timerCount1 > 1) {
    timerCount1--;
  }
  // Таймаут timerCount2
  if ( timerCount2 > 1) {
    timerCount2--;
  }
  // Таймаут timerCount3
  if ( timerCount3 > 1) {
    timerCount3--;
  }
#endif
  if ( !(myTick % 1000) ){
    secondFlag = SET;
  }
}

void timersProcess( void ) {

#if 0
  // Таймаут timerCount1
  if ( timerCount1 == 0) {
    timerCount1 = TOUTCOUNT1;
  }
  // Таймаут timerCount2
  if ( timerCount2 == 0) {
    timerCount2 = TOUTCOUNT2;
  }
  // Таймаут timerCount3
  if ( timerCount3 == 3) {
    timerCount3 = TOUTCOUNT3;
  }
#endif
  if (secondFlag) {
    secondFlag = RESET;
  }
}

// Задержка в мс
void myDelay( uint32_t del ){
  uint32_t finish = myTick + del;
  while ( myTick < finish)
  {}
}

static void RTC_SetTime( tRtc * prtc ){
  register uint32_t temp = 0U;

  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->ISR = RTC_ISR_INIT;
  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
  {}

  temp = (BIN2BCD( prtc->hour ) << RTC_POSITION_TR_HU |
          BIN2BCD( prtc->min ) << RTC_POSITION_TR_MU |
          BIN2BCD( prtc->sec ) );
  RTC->TR = ( RTC->TR & (RTC_TR_PM | RTC_TR_HT | RTC_TR_HU | RTC_TR_MNT | RTC_TR_MNU | RTC_TR_ST | RTC_TR_SU)) | temp;

  RTC->ISR =~ RTC_ISR_INIT;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}

static void RTC_SetDate( tRtc * prtc ){
  register uint32_t temp = 0U;

  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->ISR = RTC_ISR_INIT;
  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
  {}

  temp = (BIN2BCD( prtc->year ) << RTC_POSITION_DR_YU |
          BIN2BCD( prtc->month ) << RTC_POSITION_DR_MU |
          BIN2BCD( prtc->date ) << RTC_POSITION_DR_DU  |
          BIN2BCD( prtc->wday ) << RTC_POSITION_DR_WDU );
  RTC->DR = ( RTC->DR & (RTC_DR_YT | RTC_DR_YU | RTC_DR_MT | RTC_DR_MU | RTC_DR_DT | RTC_DR_DU | RTC_DR_WDU)) | temp;

  RTC->ISR =~ RTC_ISR_INIT;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}

static void RTC_GetTime( tRtc * prtc ){
  if((RTC->ISR & RTC_ISR_RSF) == 0){
    return;
  }
    prtc->hour = BCD2BIN( RTC->TR >> RTC_POSITION_TR_HU );
    prtc->min = BCD2BIN( RTC->TR >> RTC_POSITION_TR_MU );
    prtc->sec = BCD2BIN( RTC->TR );
    prtc->ss = RTC->SSR;
}

static void RTC_GetDate( tRtc * prtc ){
  if((RTC->ISR & RTC_ISR_RSF) == 0){
    return;
  }
  prtc->year = BCD2BIN( RTC->DR >> RTC_POSITION_DR_YU );
  prtc->month = BCD2BIN( RTC->DR >> RTC_POSITION_DR_MU );
  prtc->date = BCD2BIN( RTC->DR );
  prtc->wday = ( RTC->DR >> RTC_POSITION_DR_WDU ) >> 0x7;
}

void RTC_SetAlrm( tRtc * prtc, uint8_t alrm ){
  register uint32_t temp = 0U;
  // Если alrm = 0 (ALRM_A) : ALRMAR, есди alrm = 1 (ALRM_B) : ALRMBR
  register uint32_t * palrm = (uint32_t *)&(RTC->ALRMAR) + alrm;

  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->ISR = RTC_ISR_INIT;
  while((RTC->ISR & RTC_ISR_INITF)!=RTC_ISR_INITF)
  {}

  temp = (BIN2BCD( prtc->date ) << RTC_POSITION_ALMA_DU |
          BIN2BCD( prtc->hour ) << RTC_POSITION_ALMA_HU |
          BIN2BCD( prtc->min ) << RTC_POSITION_TR_MU |
          BIN2BCD( prtc->sec ) );
  *palrm = ( *palrm & (RTC_ALRMAR_PM | RTC_ALRMAR_DT | RTC_ALRMAR_DU | RTC_ALRMAR_HT | RTC_ALRMAR_HU | RTC_ALRMAR_MNT | RTC_ALRMAR_MNU | RTC_ALRMAR_ST | RTC_ALRMAR_SU)) | temp;

  RTC->ISR =~ RTC_ISR_INIT;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}

void RTC_GetAlrm( tRtc * prtc, uint8_t alrm ){
  // Если alrm = 0 (ALRM_A) : ALRMAR, есди alrm = 1 (ALRM_B) : ALRMBR
  register uint32_t * palrm = (uint32_t *)&(RTC->ALRMAR) + alrm;

  prtc->date = BCD2BIN( *palrm >> RTC_POSITION_ALMA_DU );
  prtc->hour = BCD2BIN( *palrm >> RTC_POSITION_ALMA_HU );
  prtc->min = BCD2BIN( *palrm >> RTC_POSITION_ALMA_MU );
  prtc->sec = BCD2BIN( *palrm );
}



