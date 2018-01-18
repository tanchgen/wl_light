/*
 * process.c
 *
 *  Created on: 7 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */


#include "stm32l0xx.h"

#include "main.h"
#include "my_time.h"
#include "light.h"
#include "bat.h"
#include "rfm69.h"
#include "process.h"

extern uint8_t regBuf[];

volatile uint8_t csmaCount = 0;
tUxTime sendTryStopTime;
static uint8_t msgNum;      // Порядковый номер отправляемого пакета

static void sensDataSend( void );

void mesureStart( void ){
  // Запускаем измерение напряжения батареи
  batStart();
  // Запускаем измерение температуры
  lightStart();
  batEnd();
}

void wutIrqHandler( void ){

  // По какому поводу был включен WUT? - состояние машины
  switch( state ){
    case STAT_L_MESUR:
      // Пора читать измеренную температуру из датчика
      lightEnd();
      // Не пара ли передавать данные серверу?
      dataSendTry();
      break;
    case STAT_RF_CSMA_START:
      // Канал свободен - отправляем сообщение
      EXTI->PR |= DIO3_PIN;
      EXTI->IMR &= ~(DIO3_PIN);

      // Отмечаем останов RFM_RX
    #if DEBUG_TIME
    	dbgTime.rfmRxEnd = mTick;
    #endif // DEBUG_TIME

    	rfmSetMode_s( REG_OPMODE_SLEEP );
      // Отправить сообщение
      correctAlrm( ALRM_A );
      state = STAT_TX_START;
      sensDataSend();
      break;
    case STAT_TX_START:
    	// Время на пердачу вышло - останавливаем
    	for( uint8_t i = 1; i < 50; i++ ){
    		regBuf[i] = rfmRegRead( i );
    	}
    	rfmSetMode_s( REG_OPMODE_SLEEP );
      txEnd();
      break;

    case STAT_RF_CSMA_PAUSE:
      // Пробуем еще раз проверить частоту канала
      csmaRun();
      break;
    default:
      break;
  }
}

int8_t dataSendTry( void ){
  int8_t rc = 0;
  int32_t tmp;
  uint8_t tmrf;
  uint8_t flag = RESET; // Отправлять или нет?

  // ------ Надо ли отправлять ? ------------
  if( flags.sensCplt ){
    if( (tmrf = rtc.min % SEND_TOUT) == 0 ){
      // Пришло ВРЕМЯ -> отправлять
      flag = SET;
    }
    else {
      tmp = sensData.volume - sensData.volumePrev;
      tmp = tmp*20/sensData.volumePrev;
      if( tmp != 0 ){
        // После последнего измерения значение изменилось более, чем на 5%
        flag = SET;
      }
      else {
        tmp = sensData.volume - sensData.volumePrev6;
        tmp = tmp*10/sensData.volumePrev6;
        if( tmp != 0 ){
          // После последней передачи значение изменилось более, чем на 10%
          flag = SET;
        }
      }
    }

    if( flag ){
      // Передача разрешена
      if(tmrf == 0){
        sensData.volumePrev6 = sensData.volume;
      }
      // Можно отправлять по радиоканалу
      // Запоминаем время остановки попыток отправки - пробуем не более 1-2 секунды
      sendTryStopTime = getRtcTime() + 1;
      csmaRun();
    }
    else {
    	txEnd();
    }
    // Сохраняем нынешнюю температуру, как предыдущую
    sensData.volumePrev = sensData.volume;
  }

  return rc;
}


// Начинаем слушат эфир на предмет свободности канала
void csmaRun( void ){
  state = STAT_RF_CSMA_START;
  // Включаем прерывание от DIO3 (RSSI)
  EXTI->IMR |= (DIO3_PIN);
  EXTI->PR |= DIO3_PIN;

  // Отмечаем запуск RFM_RX
#if DEBUG_TIME
	dbgTime.rfmRxStart = mTick;
#endif // DEBUG_TIME

	csmaCount++;
  rfmSetMode_s( REG_OPMODE_RX );
  // Будем слушать эфир в течение времени передачи одного пакета * 2
  wutSet( TX_DURAT );
}

// Устанавливааем паузу случайной длительности (30-150 мс) в прослушивании канала на предмет тишины
void csmaPause( void ){
  uint32_t pause;
#if 1
  SYSCFG->CFGR3 |= SYSCFG_CFGR3_ENREF_RC48MHz;
  RCC->CRRCR |= RCC_CRRCR_HSI48ON;
  RCC->CCIPR |= RCC_CCIPR_HSI48MSEL;
  while( (RCC->CRRCR & RCC_CRRCR_HSI48RDY) == RESET )
  {}
  // Включаем генератор случайных чисел
  RCC->AHBENR |= RCC_AHBENR_RNGEN;
  RNG->CR |= RNG_CR_RNGEN;
  // Ждем готовности числа (~ 46 тактов)
  while( ((RNG->SR & RNG_SR_DRDY) == 0 ) &&
          ((RNG->SR & RNG_SR_CEIS) == 0 ) &&
          ((RNG->SR & RNG_SR_SEIS) == 0) ){
  }
  // Число RND готово или ошибка (тогда RND = 0)
  pause = RNG->DR;
  // Выключаем
  RNG->CR &= ~RNG_CR_RNGEN;
  RCC->AHBENR &= ~RCC_AHBENR_RNGEN;
  RCC->CCIPR &= ~RCC_CCIPR_HSI48MSEL;
  RCC->CRRCR &= ~RCC_CRRCR_HSI48ON;
  SYSCFG->CFGR3 &= ~SYSCFG_CFGR3_ENREF_RC48MHz;
#else
  pause = 0x7FFFFFFF;
#endif
  // Длительность паузы
  pause = ((pause / (0xFFFFFFFFL/9)  ) + 1) * TX_DURAT ;
  state = STAT_RF_CSMA_PAUSE;
  wutSet( pause );
}

static void sensDataSend( void ){
  // ---- Формируем пакет данных -----
	pkt.payCmd = CMD_SENS_SEND;
	pkt.paySensType = SENS_TYPE_LS;
  pkt.paySrcNode = rfm.nodeAddr;
  pkt.payMsgNum = msgNum++;
  pkt.payBat = sensData.bat;
  pkt.payVolume = sensData.volume;

  // Передаем заполненую при измерении запись
  pkt.nodeAddr = BCRT_ADDR;
  // Длина payload = 1(nodeAddr) + 1(msgNum) + 1(bat) + 2(temp)
  pkt.payLen = sizeof(tSensMsg);

  rfmTransmit( &pkt );
  // Таймаут до окончания передачи
  wutSet( TX_DURAT*10 );

}

void txEnd( void ){
  sensData.bat = 0;
  flags.sensCplt = FALSE;
  flags.batCplt = FALSE;
  state = STAT_READY;
}
