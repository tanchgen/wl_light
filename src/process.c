/*
 * process.c
 *
 *  Created on: 7 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */


#include "stm32l0xx.h"

#include "main.h"
#include "my_time.h"
#include "thermo.h"
#include "bat.h"
#include "rfm69.h"
#include "process.h"

volatile tUxTime sendTryStopTime;
static uint8_t msgNum;      // Порядковый номер отправляемого пакета

static void sensDataSend( void );

void mesureStart( void ){
  // Запускаем измерение напряжения батареи
  batStart();
  // Запускаем измерение температуры
  thermoStart();
  // Устанавливаем время, через которое надо проснутся и запускаем
  wutSet( 23 * (THERMO_ACCUR - 8) );
}

void wutIrqHandler( void ){
  // По какому поводу был включен WUT? - состояние машины
  switch( state ){
    case STAT_T_MESUR:
      // Пора читать измеренную температуру из датчика
      thermoRead();
      flags.thermCplt = TRUE;
      state = STAT_T_READ;
    case STAT_T_READ:
      // Не пара ли передавать данные серверу?
      dataSendTry();
      break;
    case STAT_RF_CSMA_START:
      // Канал свободен - отправляем сообщение
      // Отправить сообщение
      correctAlrm( ALRM_A );
      sensDataSend();

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
  int16_t tmp;
  uint8_t tmrf;

  // ------ Надо ли отправлять ? ------------
  if( flags.batCplt && flags.thermCplt ){
    if( ((tmrf = rtc.min % 6) == 0 ) || // Время передачи наступило
         (((tmp = sensData.temp - sensData.tempPrev) > 0.5) || (tmp < -0.5)) || // За 1 мин температура изменилась более, чем 0.5 гр.С
         (((tmp = sensData.temp - sensData.tempPrev6) > 1.5) || (tmp < -1.5)) ){ // С предыдущей ОЧЕРЕДНОЙ отправки температура изменилась более, чем 1 гр.С
      if(tmrf == 0){
        sensData.tempPrev6 = sensData.temp;
      }
      // Можно отправлять по радиоканалу
      // Запоминаем время остановки попыток отправки - пробуем не более 1-2 секунды
      sendTryStopTime = getRtcTime() + 1;
      csmaRun();
    }
    // Сохраняем нынешнюю температуру, как предыдущую
    sensData.tempPrev = sensData.temp;
  }

  return rc;
}


// Начинаем слушат эфир на предмет свободности канала
void csmaRun( void ){
  state = STAT_RF_CSMA_START;
  rfmSetMode_s( REG_OPMODE_RX );
  // Будем слушать эфир в течение 20 мс
  wutSet( TX_DURAT );
}

// Устанавливааем паузу случайной длительности (30-150 мс) в прослушивании канала на предмет тишины
void csmaPause( void ){
  uint32_t pause;
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
  RCC->AHBENR &= ~RCC_AHBENR_RNGEN;
  // Длительность паузы
  pause = ((pause * 6) / (~(0L)) + 1) * TX_DURAT ;
  wutSet( pause );
  state = STAT_RF_CSMA_PAUSE;
}

static void sensDataSend( void ){
  // ---- Формируем пакет данных -----
  pkt.paySrcNode = rfm.nodeAddr;
  pkt.payMsgNum = msgNum++;
  pkt.payBat = sensData.bat;
  pkt.payTerm = sensData.temp;

  // Передаем заполненую при измерении запись
  pkt.nodeAddr = BCRT_ADDR;
  // Длина payload = 1(nodeAddr) + 1(msgNum) + 1(bat) + 2(temp)
  pkt.payLen = 5;

  rfmTransmit( &pkt );

}
