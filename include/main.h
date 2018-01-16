#ifndef __MAIN_H
#define __MAIN_H
// Includes ------------------------------------------------------------------
#include "stm32l0xx.h"

#define STOP_EN		0

#ifndef NVIC_PRIORITYGROUP_0

#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007)
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006)
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005)
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004)
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003)

#endif

#define FLASH_PEKEY1 0x89ABCDEF
#define FLASH_PEKEY2 0x02030405


#define EEMEM __attribute__((section(".eeprom")))

enum {
  FALSE,
  TRUE
};

#if 0
enum{
  RESET,
  SET
};
#endif

typedef enum {
  STAT_READY,
  STAT_L_MESUR,
  STAT_L_READ,
  STAT_L_CPLT,
  STAT_RF_CSMA_START,
  STAT_RF_CSMA_PAUSE,
  STAT_TX_START
} eState;

// Структура сохраняемых в EEPROM параметров
typedef struct {
  uint8_t adcCal;       // Цалибровочный фактор для АЦП
  uint8_t lightResol;		// Разренение измерения освещенности (VH1750_OSH_HI = 0x20, VH1750_OSH_HI2 = 0x21, VH1750_OSH_LO = 0x23)
  uint8_t rfmNetId;     // ID сети
  uint8_t rfmChannel;   // Номер канала
  uint8_t rfmNodeAddr;  // Адрес Нода
  uint8_t rfmTxPwr;     // Мощность передатчика
} tEeBackup;

// Структура измеряемых датчиком параметров
typedef struct  __packed {
  int16_t light;           // Измеряемая температура
  int16_t lightPrev;     // Температура предыдущего (1мин) измерения
  int16_t lightPrev6;    // Температура предыдущего переданного (6мин) измерения
  uint8_t bat;          // Напряжение питания
  uint8_t rssi;         // Мощность принимаемого радиосигнала
  uint8_t resolution;		// Точность измерения
  uint32_t convTout;			// Время измерения
} tSensData;

typedef struct{
  unsigned int lightErr : 1;
  unsigned int lightCplt : 1;
  unsigned int batCplt : 1;
} tFlags;

//extern volatile uint32_t mTick;
extern tEeBackup eeBackup;
extern volatile tSensData sensData;           // Структура измеряемых датчиком параметров
extern volatile tFlags flags;                 // Флаги состояний системы
extern volatile eState state;                          // Состояние машины

#define DEBUG_TIME			0

#if DEBUG_TIME
typedef struct {
	uint32_t mcuStart;
	uint32_t mcuEnd;
	uint32_t rfmRxStart;
	uint32_t rfmRxEnd;
	uint32_t rfmTxStart;
	uint32_t rfmTxEnd;
	uint32_t thermoStart;
	uint32_t thermoEnd;

} tDbgTime;

extern tDbgTime dbgTime;

#endif // DEBUG_TIME

void restoreContext(void);
void saveContext(void);

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1 */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
