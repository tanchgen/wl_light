/*
 * adc.c
 *
 *  Created on: 2 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#include "stm32l0xx.h"
#include "stm32l0xx_ll_adc.h"

#include "main.h"
#include "bat.h"

/* ADC init function */
void adcInit(void){

  // Вкл тактирование АЦП
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_SYSCFGEN;

  // Конфигурация АЦП
  ADC1->CFGR2 |= LL_ADC_CLOCK_SYNC_PCLK_DIV1;
  // Включаем программный запуск и AUTOFF
  ADC1->CFGR1 = (ADC1->CFGR1 & ~ADC_CFGR1_EXTEN) | ADC_CFGR1_AUTOFF;
  // Только VREFINT канал
  ADC1->CHSELR = ADC_CHSELR_CHSEL17;
  // Длительность сэмпла = 12.5 ADCCLK (+12.5) = 25 ADCCLK ( 5.96мкс )
  ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1;

  // Прерывание по окончанию преобразования
  ADC1->IER = ADC_IER_EOCIE;
  ADC->CCR |= ADC_CCR_VREFEN;

  NVIC_EnableIRQ(ADC1_COMP_IRQn);
  NVIC_SetPriority(ADC1_COMP_IRQn,0);

  // Калибровка АЦП
  if( eeBackup.adcCal == 0x7F ){
    if ((ADC1->CR & ADC_CR_ADEN) != 0){
      ADC1->CR &= (uint32_t)(~ADC_CR_ADEN);
    }
    ADC1->CR |= ADC_CR_ADCAL;
    while ((ADC1->ISR & ADC_ISR_EOCAL) == 0)
    {}
    ADC1->ISR |= ADC_ISR_EOCAL;
    eeBackup.adcCal = ADC1->CALFACT;
  }
  else {
    ADC1->CALFACT = eeBackup.adcCal;
  }

  // Опять включаем АЦП после калибровки
  ADC1->CR |= ADC_CR_ADEN;
  if ((ADC1->CFGR1 &  ADC_CFGR1_AUTOFF) == 0) {
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0)
    {}
  }

}

void batStart( void ){
  // Ждем, когда запустится VREFINT
  while( (PWR->CSR & PWR_CSR_VREFINTRDYF) == 0 )
  {}
  ADC1->CR |= ADC_CR_ADSTART;
}

