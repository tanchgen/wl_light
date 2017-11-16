/*
 * spi.c
 *
 *  Created on: 7 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#include "stm32l0xx.h"

#include "main.h"
#include "spi.h"

/* SPI1 init function */
void spiInit(void) {

  // ----------- SPI GPIO configration ----------------------
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

  /**SPI1 GPIO Configuration
  PA4   ------> SPI1_NSS
  PA5   ------> SPI1_SCK
  PA7   ------> SPI1_MOSI
  */
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE7))\
                  | (GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1 | GPIO_MODER_MODE7_1);
  GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((0xFL<<(4 * 4)) | (0xFL<<(5 * 4)) | (0xFL<<(7 * 4))));
  GPIOA->OSPEEDR |= (0x03L << (4 * 2)) | (0x03L << (5 * 2)) | (0x03L<<(7 * 2));
  //   PB4   ------> SPI1_MISO
  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE4)) | GPIO_MODER_MODE4_1;
  GPIOB->AFR[0] = (GPIOB->AFR[0] & ~((0xFL<<(3 * 4))));
  GPIOB->OSPEEDR |= (0x03L << (4 * 2));

  /* Enable the peripheral clock SPI1 */
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  /* Configure SPI1 in master */
  // Master selection, BR: Fpclk/2, CPOL and CPHA (rising first edge), 8-bit data frame
  SPI1->CR1 = (SPI1->CR1 & ~(SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_SSM)) | SPI_CR1_MSTR;
  // Slave select output enabled
  SPI1->CR2 = SPI_CR2_SSOE;
  SPI1->CR1 |= SPI_CR1_SPE;
}


// Передача по SPI в блокирующем режима
int8_t spiTrans_s( uint8_t *buf, uint8_t len ){
  uint32_t tout;

  tout = mTick + 10;
  // Отправка из буфера tx в буфер SPI
  while( len ){
    if( ((SPI1->SR & SPI_SR_TXE) != 0 )){
      *(uint8_t *)&(SPI2->DR) = *buf++;
      len--;
    }
    if( tout < mTick){
      return -1;
    }
  }
  // Ждем окончания передачи
  tout = mTick + 10;
  while( (SPI1->SR & SPI_SR_BSY) != 0 ){
    if( tout < mTick){
      return -1;
    }
  }

  return 0;
}

// Прием по SPI в блокирующем режима
int8_t spiRecv_s( uint8_t *buf, uint8_t len ){
  uint8_t txCount = len;
  uint32_t tout;

  // На всю операцию отводим не более 10мс
  tout = mTick + 10;
  // Отправка из буфера tx в буфер SPI
  while( len ){
    if( txCount && ((SPI1->SR & SPI_SR_TXE) != 0) ){
      *(uint8_t *)&(SPI2->DR) = 0xFF;
      txCount--;
    }
    if( (SPI1->SR & SPI_SR_RXNE) != 0 ){
      *buf++ = *(uint8_t *)&(SPI2->DR);
      len--;
    }
    if( tout < mTick){
      return -1;
    }
  }
  // Ждем окончания приема
  tout = mTick + 10;
  while( (SPI1->SR & SPI_SR_BSY) != 0 ){
    if( tout < mTick){
      return -1;
    }
  }

  return 0;
}

// Передача с одновременным приемом по SPI в блокирующем режима
int8_t spiTransRecv_s( uint8_t *txBuf, uint8_t *rxBuf, uint8_t len ){
  uint8_t txCount = len;
  uint32_t tout;

  // На всю операцию отводим не более 10мс
  tout = mTick + 10;
  // Отправка из буфера tx в буфер SPI
  while( len ){
    if( txCount && ((SPI1->SR & SPI_SR_TXE) != 0) ){
      *(uint8_t *)&(SPI2->DR) = *txBuf++;
      txCount--;
    }
    if( (SPI1->SR & SPI_SR_RXNE) != 0 ){
      *rxBuf++ = *(uint8_t *)&(SPI2->DR);
      len--;
    }
    if( tout < mTick){
      return -1;
    }
  }
  // Ждем окончания приема
  tout = mTick + 10;
  while( (SPI1->SR & SPI_SR_BSY) != 0 ){
    if( tout < mTick){
      return -1;
    }
  }

  return 0;
}

