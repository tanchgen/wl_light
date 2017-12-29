/*
 * i2c.c
 *
 *  Created on: 3 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#include "stm32l0xx_ll_i2c.h"

#include "main.h"
#include "thermo.h"


static uint8_t tmp75CfgWrite( uint8_t cfg );
static inline uint8_t tmp75Write( uint8_t *buf, uint8_t num, uint8_t autoend );
static uint32_t tmp75Read( uint8_t *rxBuffer, uint8_t num );

static inline void i2cGpioInit(void) {
  // SCL - PB6, SDA - PB7
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

  // PullUp for I2C signals
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7)) \
                 | (GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0);
  // Open drain for I2C signals
  GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7;
  // AF1 for I2C signals
  GPIOB->AFR[0] = (GPIOB->AFR[0] & ~((0x0FL << ( 6 * 4 )) | (0x0FL << ( 7 * 4 )))) \
                  | (1 << ( 6 * 4 )) | (1 << (7 * 4));
  //Select AF mode (10) on PB6 and PB7
  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7)) \
                 | (GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);
}


static inline void i2cInit( void ){
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
  // Use APBCLK for I2C CLK
  RCC->CCIPR &= ~RCC_CCIPR_I2C1SEL;

  // Сначала отключаем I2C
  I2C1->CR1 &= ~I2C_CR1_PE;
  // Расчитанно из STM32CubeMX
  I2C1->TIMINGR = 0x00000004L;

  // Адрес ведомого
  I2C1->CR2 = (I2C1->CR2 & ~I2C_CR2_SADD) | (TMP75_ADDR << 1);
#define I2C1_OWN_ADDRESS (0x00)
  I2C2->OAR1 = (I2C1_OWN_ADDRESS<<1);


  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);

  // Выключаем I2C
//  I2C1->CR1 |= I2C_CR1_PE;

//  NVIC_SetPriority(I2C1_IRQn, 0);
//  NVIC_EnableIRQ(I2C1_IRQn);
}

void tmp75Init( void ){
  i2cGpioInit();
  i2cInit();
  tmp75Stop();
}

void tmp75Stop( void ){
  const uint8_t cfg = TMP75_REG_ACCUR | TMP75_SD;

  // Отмечаем запуск измерения
#if DEBUG_TIME
	dbgTime.thermoStart = mTick;
#endif // DEBUG_TIME

  // Отправляем команду
  // Настройки термодатчика
  tmp75CfgWrite( cfg );
  // Выключаем I2C
  I2C1->CR1 &= ~I2C_CR1_PE;
}

uint8_t tmp75RegRead( uint8_t regAddr ){
	uint8_t rc;

  // Отправляем 1 байт без autoend
  if( tmp75Write( &regAddr, 1, 0 ) == 1){
    if( tmp75Read( &rc, 1 ) != 1) {
      rc = 0xFF;
    }
  }

  return rc;
}

void tmp75Start( void ){
  const uint8_t cfg = TMP75_OS | TMP75_REG_ACCUR | TMP75_SD;

  // Отмечаем запуск измерения
#if DEBUG_TIME
	dbgTime.thermoStart = mTick;
#endif // DEBUG_TIME

  // Отправляем команду начать измерение
  // Настройки термодатчика
  tmp75CfgWrite( cfg );
  // Выключаем I2C
  I2C1->CR1 &= ~I2C_CR1_PE;
// Уснуть на время преобразования мс: 27.5 - 9бит, 55 - 10бит, 110 - 11бит, 220 - 12бит
  state = STAT_T_MESUR;
}

uint16_t tmp75ToRead( void ){
  union{
    uint8_t u8[4];
    uint32_t u32;
  } tmpBuf;
  uint32_t to = 0xFF00;

  uint8_t regAddr = TMP75_REG_TO;

  // Отправляем 1 байт без autoend
  if( tmp75Write( &regAddr, 1, 0 ) == 1){
    if( tmp75Read( tmpBuf.u8, 2 ) != 2) {
      flags.thermoErr = SET;
      tmpBuf.u32 = 0xFF;
    }
    else {
      flags.thermoErr = RESET;
    }
  }
  else {
    flags.thermoErr = SET;
    tmpBuf.u32 = 0xFF;
  }

  // Отмечаем останов измерения
#if DEBUG_TIME
	dbgTime.thermoEnd = mTick;
#endif // DEBUG_TIME


  I2C1->CR1 &= ~I2C_CR1_PE;

  // Переворачиваем тетрады
  to = __REV16( tmpBuf.u32 );

  return ((uint16_t)to >> 4);
}


// Прием двух байт от TMP75
static uint32_t tmp75Read( uint8_t *rxBuffer, uint8_t num ) {
  uint8_t numRxBytes = 0;

  I2C1->CR2 = 0;

  // I2C_CR2_NBYTES = 2
  I2C1->CR2 = (num << 16) | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | (0x48 << 1);

  I2C1->CR1 |= I2C_CR1_PE;
  I2C1->CR2 |= I2C_CR2_START;

  while(I2C1->CR2 & I2C_CR2_START);

  for( uint32_t i = 0; i < 0x0000001F; i++) {

    if (I2C1->ISR & I2C_ISR_RXNE) {
      // Device acknowledged and we must send the next byte
      if (numRxBytes < num){
        rxBuffer[numRxBytes++] = I2C1->RXDR;
      }
      i = 0;
    }

    if( (I2C1->ISR & I2C_ISR_BUSY) == 0) {
      break;
    }

  }
  return numRxBytes;
}


uint8_t thermoRead( void ){
  if( flags.thermoErr == 0){
    sensData.temp = tmp75ToRead();
  }
  flags.thermCplt = SET;
  state = STAT_T_CPLT;

  return flags.thermoErr;
}


static inline uint8_t tmp75Write( uint8_t *buf, uint8_t num, uint8_t autoend ){
  uint8_t numTxBytes = 0;
  uint32_t tmp;

  I2C1->CR2 = 0;

  // I2C_CR2_NBYTES = 2, Slave address = 0x48
  tmp = (num << 16) | (0x48 << 1);

  if( autoend ){
    tmp |= I2C_CR2_AUTOEND;
  }
  else {
    tmp &= ~I2C_CR2_AUTOEND;
  }
  I2C1->CR2 = tmp;

  I2C1->CR1 |= I2C_CR1_PE;
  I2C1->CR2 |= I2C_CR2_START;
  while(I2C1->CR2 & I2C_CR2_START);

  // Цикл передачи num байт в I2C
  // i - индекс таймаута...
  for( uint32_t i = 0; i < 0x0000001F; i++) {
    if (I2C1->ISR & I2C_ISR_NACKF) {
      // Was not acknowledged, disable device and exit
      break;
    }

    if (I2C1->ISR & I2C_ISR_TC) {
      // Передачу закончили - выходим
      break;
    }

    if (I2C1->ISR & I2C_ISR_TXIS) {
      // Device acknowledged and we must send the next byte
      if (numTxBytes < num){
        I2C1->TXDR = buf[numTxBytes++];
      }
      // Сбрасываем таймаут
      i = 0;

    }

  }
  return numTxBytes;
}

// Запись конфигурации в TMP75
static uint8_t tmp75CfgWrite( uint8_t cfg ){

  uint8_t numTxBytes = 0;

  I2C1->CR2 = 0;

  // I2C_CR2_NBYTES = 2, Slave address = 0x48, AUTOEND
  I2C1->CR2 = (2 << 16) | (0x48 << 1) | I2C_CR2_AUTOEND;;

  I2C1->CR1 |= I2C_CR1_PE;
  I2C1->CR2 |= I2C_CR2_START;
  while( (I2C1->CR2 & I2C_CR2_START) == I2C_CR2_START );

  // Цикл передачи num байт в I2C
  // i - индекс таймаута...
  for( uint32_t i = 0; i < 0x0000001F; i++) {
    if (I2C1->ISR & I2C_ISR_NACKF) {
      // Was not acknowledged, disable device and exit
      break;
    }

    if (I2C1->ISR & I2C_ISR_TC) {
      // Передачу закончили - выходим
      break;
    }
    if( (I2C1->ISR & I2C_ISR_BUSY) == 0 ) {
      // Передачу закончили - выходим
      break;
    }

    if (I2C1->ISR & I2C_ISR_TXIS) {
      // Device acknowledged and we must send the next byte
      if (numTxBytes == 0){
        // Отправляем номер регистра - CFG;
        I2C1->TXDR = TMP75_REG_CR;
        numTxBytes++;
      }
      else if (numTxBytes == 1){
        // Отправляем байт конфигурации;
        I2C1->TXDR = cfg;
        numTxBytes++;
      }
      // Сбрасываем таймаут
      i = 0;
    }

  }
  return numTxBytes;
}

void thermoIrqHandler( void ){
  __NOP();
}

#if 0
static void thermoErrHandler( void ){
  flags.thermoErr = SET;
  sensData.temp = 0xFF00;
  // Сброс I2C
  I2C1->CR1 &= ~I2C_CR1_PE;
  // APB2 clock == system clock
  __NOP(); __NOP(); __NOP();
  I2C1->CR1 |= I2C_CR1_PE;
}
#endif
