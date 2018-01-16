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


#define SEND_TOUT			6		// Период передачи показаний в минутах
#define CSMA_COUNT_MAX	3		// Максимальное количество попыток ожидания свободного радиоканала

// Список команд
typedef enum {
  CMD_QUERY_RTC = 1,      // Запрос ЧРВ
  CMD_NETID,              // Адрес сети 0x02
  CMD_ADDRNODE,           // Адрес термоподвески 0x03
  CMD_CHANNEL,           	// Номер радиоканала 0x04
  CMD_MESUR_TOUT,        	// Интервал измерения, секунд 0x05
  CMD_ACCUR,             	// Точность представления данных 0x05
  CMD_SENS_SEND,           	// Значение показаний датчика
  CMD_SENS_RESP,           	// ПОдтверждение получения показаний датчика
  CMD_SENS_CFG,            	// Отправка показаний датчика с запросом конфигурации
  CMD_SENS_CFG_RESP,       	// Подтверждение получения показаний датчика и отправка конфигурации
} eCmd;

// Список ошибок выполнения команд
enum {
  CMD_ERR_OK,         // Нет ошибок
  CMD_ERR_INVALID,    // Неправильная команда
  CMD_ERR_PARAM,      // Ошибка параметра
  CMD_ERR_EXEC        // Ошибка выполнения команды
};

// Типы датчиков
typedef enum {
	SENS_TYPE_TO,				// Температура
	SENS_TYPE_AL,				// Освещенности
	SENS_TYPE_SW,				// Контктный
	SENS_TYPE_PIR				// ИК Движения
} eSensType;

extern tUxTime sendTryStopTime;

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
