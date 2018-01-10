/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx.h"

#include "thermo.h"
#include "main.h"
#include "process.h"
#include "rfm69.h"
#include "stm32l0xx_it.h"

/* External variables --------------------------------------------------------*/

extern uint8_t regBuf[];

/******************************************************************************/
/*            Cortex-M0+ Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

void NMI_Handler(void){
}

void HardFault_Handler(void){
  while (1)
  {}
}

void SVC_Handler(void){
}

void PendSV_Handler(void){
}

void SysTick_Handler(void) {
//  mTick++;
}

void ADC1_COMP_IRQHandler(void){
  if( (ADC1->ISR & ADC_ISR_EOS) == 0 ){
    // Неизвестное прерывание - перезапускаем АЦП
    if ((ADC1->CR & ADC_CR_ADSTART) != 0){
      ADC1->CR |= ADC_CR_ADSTP;
    }
    while ((ADC1->CR & ADC_CR_ADSTP) != 0)
    {}
    ADC1->CR |= ADC_CR_ADDIS;
    while ((ADC1->CR & ADC_CR_ADEN) != 0)
    {}
    ADC1->CR |= ADC_CR_ADEN;
  }
  else {
    uint32_t vrefCal = *((uint16_t *)0x1FF80078);
    uint32_t vref = ADC1->DR;
  	// Выключаем внутренний регулятор напряжения
    ADC1->CR |= ADC_CR_ADDIS;
    ADC1->CR &= ~ADC_CR_ADVREGEN;

    // Пересчет: X (мВ) / 10 - 150 = Y * 0.01В. Например: 3600мВ = 210ед, 2000мВ = 50ед
    sensData.bat = (uint8_t)(((3000L * vrefCal)/vref)/10 - 150);
//    deepSleepOn();
    flags.batCplt = TRUE;
    // Не пара ли передавать данные серверу?
//    dataSendTry();
  }
  // Стираем
  ADC1->ISR |= 0xFF; //ADC_ISR_EOS | ADC_ISR_EOC | ADC_ISR_EOSMP;
}


/**
* RTC global interrupt through EXTI lines 17, 19 and 20.
*/
void RTC_IRQHandler(void){
// Восстанавливаем настройки портов

  restoreContext();
  // Отмечаем запуск MCU
#if DEBUG_TIME
	dbgTime.mcuStart = mTick;
#endif // DEBUG_TIME

  if( RTC->ISR & RTC_ISR_WUTF ){
    // Wake-Up timer interrupt
    //Clear WUTF
    // Write access for RTC registers
  	RTC->WPR = 0xCA;
  	RTC->WPR = 0x53;
	  // Останавливаем WakeUp Таймер
	  RTC->CR &= ~RTC_CR_WUTE;
	  while((RTC->ISR & RTC_ISR_WUTWF) != RTC_ISR_WUTWF)
	  {}
    RTC->ISR &= ~RTC_ISR_WUTF;
//	RTC->ISR = (~((RTC_ISR_WUTF | RTC_ISR_INIT) & 0x0000FFFFU) | (RTC->ISR & RTC_ISR_INIT));
//    RTC->CR |= RTC_CR_WUTE;
    // Disable write access
    RTC->WPR = 0xFE;
    RTC->WPR = 0x64;
    wutIrqHandler();
    // Стираем флаг прерывания EXTI
    EXTI->PR |= EXTI_PR_PR20;
  }
  if( RTC->ISR & RTC_ISR_ALRAF ){
    // Alarm A interrupt
    //Clear ALRAF
    RTC->ISR &= ~RTC_ISR_ALRAF;
    uxTime = getRtcTime();
    if(state == STAT_READY){
      mesureStart();
    }
    // Стираем флаг прерывания EXTI
    EXTI->PR |= EXTI_PR_PR17;
  }

  // Отмечаем Останов MCU
#if DEBUG_TIME
	dbgTime.mcuEnd = mTick;
#endif // DEBUG_TIME

	// Проверяем на наличие прерывания EXTI
	if(EXTI->PR != 0){
		uint8_t tmp = EXTI->PR;
		EXTI->PR = tmp;
	}
  // Стираем PWR_CR_WUF
  PWR->CR |= PWR_CR_CWUF;
  while( (PWR->CSR & PWR_CSR_WUF) != 0)
  {}
	// Сохраняем настройки портов
	saveContext();
}

/**
* Главное прерывание от RFM - DIO0
*/
void EXTI0_1_IRQHandler(void)
{
	// Восстанавливаем настройки портов
  restoreContext();

	EXTI->PR |= DIO0_PIN;
  if( rfm.mode == MODE_RX ){
    // Если что-то и приняли, то случайно
    // Опустошаем FIFO
    while( dioRead(DIO_RX_FIFONE) == SET ){
      rfmRegRead( REG_FIFO );
    }
  }
  else if( rfm.mode == MODE_TX ) {
    // Отправили пакет с температурой
  	wutStop();
  	state = STAT_READY;
  }
  // Выключаем RFM69
  rfmSetMode_s( REG_OPMODE_SLEEP );
  // Отмечаем останов RFM_TX
#if DEBUG_TIME
	dbgTime.rfmTxEnd = mTick;
#endif // DEBUG_TIME

	// Сохраняем настройки портов
	saveContext();
}

// Прерывание по PA3 - DIO3 RSSI
// Канал кем-то занят
void EXTI2_3_IRQHandler( void ){
  tUxTime timeNow;

  // Восстанавливаем настройки портов
  restoreContext();

  // Выключаем прерывание от DIO3 (RSSI)
  EXTI->PR |= DIO3_PIN;
  EXTI->IMR &= ~(DIO3_PIN);

  regBuf[REG_RSSI_VAL] = rfmRegRead( REG_RSSI_VAL );
  rfmSetMode_s( REG_OPMODE_SLEEP );

  // Отмечаем останов RFM_RX
#if DEBUG_TIME
	dbgTime.rfmRxEnd = mTick;
#endif // DEBUG_TIME

  // Канал занят - Выжидаем паузу 30мс + x * 20мс
  timeNow = getRtcTime();
  if( timeNow > sendTryStopTime ){
    // Время на попытки отправить данные вышло - все бросаем до следующего раза
    wutStop();
    state = STAT_READY;
  }
  else {
  	// Можно еще попытатся - выждем паузу
  	csmaPause();
  }

	// Сохраняем настройки портов
	saveContext();
  return;
}

#if 0
/**
* @brief This function handles I2C1 event global interrupt / I2C1 wake-up interrupt through EXTI line 23.
*/
void I2C1_IRQHandler(void) {
  // Обработка прерывания I2C: работа с термодатчиком
  thermoIrqHandler();
}
#endif
