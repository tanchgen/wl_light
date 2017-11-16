/*
 * i2c.c
 *
 *  Created on: 3 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#include "stm32l0xx_ll_i2c.h"

#include "main.h"
#include "thermo.h"


static inline int16_t thermoRcv( void );
static inline void thermoErrHandler( void );
static void thermoSend( uint8_t *buf );

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
  I2C1->CR1 &= I2C_CR1_PE;
  // Расчитанно из STM32CubeMX
  I2C1->TIMINGR = 0x00000004L;
  // Включаем I2C
  I2C1->CR1 &= I2C_CR1_PE;

  // Адрес ведомого
  I2C1->CR2 = (I2C1->CR2 & ~I2C_CR2_SADD) | (THERMO_ADDR << 1);
#define I2C1_OWN_ADDRESS (0x00)
  I2C2->CR2 = (I2C1_OWN_ADDRESS<<1);


  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);

//  NVIC_SetPriority(I2C1_IRQn, 0);
//  NVIC_EnableIRQ(I2C1_IRQn);
}

void thermoInit( void ){
  uint8_t buf[2];

  i2cGpioInit();
  i2cInit();
  // Настройки термодатчика
  buf[0] = THERMO_REG_CR;
  buf[1] = ((uint8_t)~(THERMO_OS)) | THERMO_REG_ACCUR | THERMO_SD;
  thermoSend( buf );
}

void thermoStart( void ){
  uint8_t buf[2];

  // Отправляем команду начать измерение
  buf[0] = THERMO_REG_CR;
  buf[1] = THERMO_OS | THERMO_R09 | THERMO_SD;
  thermoSend( buf );
// Уснуть на время преобразования мс: 27.5 - 9бит, 55 - 10бит, 110 - 11бит, 220 - 12бит
  state = STAT_T_MESUR;
}

uint8_t thermoRead( void ){
  if( flags.thermoErr == 0){
    sensData.temp = thermoRcv();
  }
  flags.thermCplt = SET;
  state = STAT_T_CPLT;

  return flags.thermoErr;
}

static inline int16_t thermoRcv( void ){
  union{
    int16_t i16t;
    uint8_t u8t[2];
  } rc;
  uint32_t tout = mTick + I2C_TOUT;

  if( (I2C1->ISR & I2C_ISR_BUSY) == 0 ){
    // Ошибка на шине I2C - шина занята
    thermoErrHandler();
    // -128 гр.С
    return 0xFF00;
  }

  // Передаем 1 байт без автостопа
  I2C1->CR2 = (I2C1->CR2 & ~(I2C_CR2_NBYTES | I2C_CR2_RD_WRN  | I2C_CR2_AUTOEND) ) | (1 << 16);
  // 1-st Byte - Temperature register address to send
  I2C2->TXDR = 0x00;
  I2C2->CR2 |= I2C_CR2_START; // Go
  // Подтверждение приемки адреса ведомым
  while( (I2C1->ISR & I2C_ISR_NACKF) == 0 ){
    if( tout <= mTick ){
      // Ошибка на шине I2C
      thermoErrHandler();
      // -128 гр.С
      return 0xFF00;
    }
  }
  I2C1->ISR |= I2C_ISR_NACKF;

  while( (I2C1->ISR & I2C_ISR_TC) == 0 ){
    if( tout <= mTick ){
      // Ошибка на шине I2C
      thermoErrHandler();
      // -128 гр.С
      return 0xFF00;
    }
  }
  // Принимаем 2 байта с автостопом
  I2C1->CR2 = (I2C1->CR2 & ~I2C_CR2_NBYTES) | (2 << 16) | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND;

  for( uint8_t i=0; i<2; i++){
    tout = mTick + I2C_TOUT;
    while( (I2C1->ISR & I2C_ISR_RXNE) == 0 ){
      if( tout <= mTick ){
        // Ошибка на шине I2C
        thermoErrHandler();
        // -128 гр.С
        return 0xFF00;
      }
    }
    rc.u8t[i] = I2C2->RXDR; // Data Byte (MSB and LSB temperature) reseived
  }
  // Ждем окончания приема
  tout = mTick + I2C_TOUT;
  while( (I2C1->ISR & I2C_ISR_BUSY) != 0 ){
    if( tout <= mTick ){
      // Ошибка на шине I2C
      thermoErrHandler();
    }
  }

  return (rc.i16t >> 7);
}

static void thermoSend( uint8_t *buf ){
  uint32_t tout = mTick + I2C_TOUT;

  if( (I2C1->ISR & I2C_ISR_BUSY) == 0 ){
    // Ошибка на шине I2C
    thermoErrHandler();
    return;
  }
  // Передаем 2 байта с автостопом
  I2C1->CR2 = (I2C1->CR2 & ~(I2C_CR2_NBYTES | I2C_CR2_RD_WRN)) | (2 << 16) | I2C_CR2_AUTOEND;

  I2C2->CR2 |= I2C_CR2_START; // Go
  while( (I2C1->ISR & I2C_ISR_NACKF) == 0 ){
    if( tout <= mTick ){
      // Ошибка на шине I2C
      thermoErrHandler();
    }
  }
  I2C1->ISR |= I2C_ISR_NACKF;
  while( (I2C1->ISR & I2C_ISR_TXIS) == 0 )
  {}
  I2C2->TXDR = *buf++; // 1-st Byte to send
  while( (I2C1->ISR & I2C_ISR_TXIS) == 0 )
  {}
  I2C2->TXDR = *buf; // 2-nd Byte to send

  // Ждем окончания передачи
  tout = mTick + I2C_TOUT;
  while( (I2C1->ISR & I2C_ISR_BUSY) != 0 ){
    if( tout <= mTick ){
      // Ошибка на шине I2C
      thermoErrHandler();
    }
  }

}

void thermoIrqHandler( void ){
  __NOP();
}

static inline void thermoErrHandler( void ){
  flags.thermoErr = SET;
  sensData.temp = 0xFF00;
  // Сброс I2C
  I2C1->CR1 &= ~I2C_CR1_PE;
  // APB2 clock == system clock
  __NOP(); __NOP(); __NOP();
  I2C1->CR1 |= I2C_CR1_PE;
}
