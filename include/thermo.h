/*
 * i2c.h
 *
 *  Created on: 3 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#ifndef THERMO_H_
#define THERMO_H_

#include "stm32l0xx.h"

#define THERMO_ADDR     0x48      // I2C-Адрес термодатчика с учетом сдвига под R/NW-бит
#define R_NW            0x01      // R/NW-бит

#define THERMO_REG_CR   0x01      //Адрес регистра конфигурации
#define THERMO_REG_T    0x00      //Адрес регистра температуры

#define THERMO_SD       (uint8_t)0x01      // Маска бита ShutDown Mode (SD)
#define THERMO_TM       (uint8_t)0x02      // Маска бита Thermostat Mode (TM)
#define THERMO_R09      (uint8_t)0x00      // Маска точности 0.5 гр.Ц (9 бит)
#define THERMO_R10      (uint8_t)0x20      // Маска точности 0.25 гр.Ц (10 бит)
#define THERMO_R11      (uint8_t)0x40      // Маска точности 0.125 гр.Ц (11 бит)
#define THERMO_R12      (uint8_t)0x60      // Маска точности 0.0625 гр.Ц (12 бит)
#define THERMO_OS       (uint8_t)0x80      // Маска бита START One-Shot

#define THERMO_ACCUR      9
#define THERMO_REG_ACCUR  (((THERMO_ACCUR) - 9) << 5)

//#define THERMO_START      0x02      // Значение CR - проснутся
//#define THERMO_STOP      0x03      // Значение CR - уснуть

#define I2C_TOUT        20

void thermoInit( void );

// Запуск конвертирования температуры
void thermoStart( void );
// Считывание измеренной температуры
uint8_t thermoRead( void );

void thermoIrqHandler( void );

#endif /* THERMO_H_ */
