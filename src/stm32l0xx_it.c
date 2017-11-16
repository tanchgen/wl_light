/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx.h"

#include "thermo.h"
#include "main.h"
#include "process.h"
#include "rfm69.h"
#include "stm32l0xx_it.h"

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0+ Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable Interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void) {
  mTick++;
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

void ADC1_COMP_IRQHandler(void){
  if( (ADC1->ISR & ADC_ISR_EOC) == 0 ){
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
    // Пересчет: X (мВ) / 10 - 150 = Y * 0.01В. Например: 3600мВ = 210ед, 2000мВ = 50ед
    sensData.bat = (uint8_t)(((3000L * vrefCal)/vref)/10 - 150);
    deepSleepOn();
    flags.batCplt = TRUE;
    // Не пара ли передавать данные серверу?
    dataSendTry();
  }
}


/**
* @brief This function handles RTC global interrupt through EXTI lines 17, 19 and 20 and LSE CSS interrupt through EXTI line 19.
*/
void RTC_IRQHandler(void){
  if( RTC->ISR & RTC_ISR_WUTF ){
    // Wake-Up timer interrupt
    //Clear WUTF
    RTC->ISR &= ~RTC_ISR_WUTF;
    wutIrqHandler();
  }
  if( RTC->ISR & RTC_ISR_ALRAF ){
    // Alarm A interrupt
    //Clear ALRAF
    RTC->ISR &= ~RTC_ISR_ALRAF;
    uxTime = getRtcTime();
    mesureStart();
  }
}

/**
* @brief This function handles EXTI line 0 and line 1 interrupts.
*/
void EXTI0_1_IRQHandler(void)
{
  if( rfm.mode == MODE_RX ){
    // Если что-то и приняли, то случайно
    // Опустошаем FIFO
    while( dioRead(DIO_RX_FIFONE) == SET ){
      rfmRegRead( REG_FIFO );
    }
  }
  else if( rfm.mode == MODE_TX ) {
    // Отправили пакет с температурой
  }
  // Выключаем RFM69
  rfmSetMode_s( REG_OPMODE_SLEEP );

}

// Прерывание по PA3 - DIO3 RSSI
// Канал кем-то занят
void EXTI2_3_IRQHandler( void ){
  tUxTime timeNow;

  EXTI->PR |= DIO3_PIN;
  rfmSetMode_s( REG_OPMODE_SLEEP );

  // Канал занят - Выжидаем паузу 30мс + x * 20мс
  timeNow = getRtcTime();
  if( timeNow > sendTryStopTime ){
    // Время на попытки отправить данные вышло - все бросаем до следующего раза
    wutStop();
    return;
  }
  // Можно еще попытатся - выждем паузу
  csmaPause();
}


/**
* @brief This function handles I2C1 event global interrupt / I2C1 wake-up interrupt through EXTI line 23.
*/
void I2C1_IRQHandler(void) {
  // Обработка прерывания I2C: работа с термодатчиком
  thermoIrqHandler();
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
