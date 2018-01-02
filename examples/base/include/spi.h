/** ***************************************************************************
 * @file spi.h serial peripheral interface synchronous serial bus functions
 * @Author jsun
 * @date   2011-02-10 11:42:26 -0800 (Thu, 10 Feb 2011)
 * @rev 15866
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef __SPI_H
#define __SPI_H
#include <stdint.h>

#include "stm32f2xx.h"
#include "stm32f2xx_spi.h"

#define SPI_CPOL_AND_CPHA_HIGH  1
#define SPI_CPOL_AND_CPHA_LOW   0 // not used

#define EMPTY     RESET // 0
#define NOT_EMPTY SET   // 1

enum spi_msg_size {
     _1_BYTE  =  1,
     _2_BYTES =  2,
     _3_BYTES =  3,
    _16_BYTES = 16,
    _17_BYTES = 17,
};


uint8_t spi_configure(SPI_TypeDef* SPIx, uint8_t cpolAndCphaHigh, void (*callback)(void));
uint8_t spi1_transfer(volatile uint8_t *in , volatile uint8_t *out, uint16_t length);
uint8_t spi3_transfer(volatile uint8_t *in , volatile uint8_t *out, uint16_t length);
uint8_t get_spi1_complete( void );
void    spi_set_baud(SPI_TypeDef* SPIx, uint16_t baudRatePrescaler);

#define SPI_ERROR_GENERIC 1
#define SPI_NO_ERROR      0

#endif
