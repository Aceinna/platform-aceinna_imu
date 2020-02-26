#ifndef _SPI_BITBANG_H
#define _SPI_BITBANG_H

#include "stdint.h"
#include "stm32f4xx_hal.h"

#define START_DATA_LEVEL    0
#define START_CLK_LEVEL     1
#define SPI_MIN_SPEED       20
#define SPI_MAX_SPEED       2


void SpiBitBangReadTransaction(uint8_t reg, uint8_t *dst1, uint8_t *dst2, uint8_t *dst3, int len);
void SpiBitBangWriteTransaction(uint8_t reg, uint8_t *src, int len);
void SpiBitBangInit();
void SpiBitBangSetBaud(uint16_t prescaler);
void SpiBitBangSelectActiveSensors(uint16_t sensorsMap);
void SpiBitBangSelectSaveActiveSensors(uint16_t sensorsMap);
void SpiBitBangRestoreActiveSensors();


#endif
