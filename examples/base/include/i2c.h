/** ***************************************************************************
 * @file i2c.h  two wire bus driver functions
 * @Author jsun
 * @date   2011-02-18 16:56:57 -0800 (Fri, 18 Feb 2011)
 * @rev 15960
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/ 
#ifndef __I2C_H
#define __I2C_H
#include <stdint.h>

#include "stm32f2xx.h"
#include "stm32f2xx_i2c.h"

/// mutex to avoid two sensors trying to use the same 
/// peripheral at the same time
uint8_t i2c_open (I2C_TypeDef* I2Cx, void (*callback)()); // returns TRUE if good
void    i2c_close(I2C_TypeDef *I2Cx);

void i2c_configure(I2C_TypeDef *I2Cx);


uint8_t i2c_data_request(I2C_TypeDef *I2Cx,
                         uint16_t    slave_addr, 
                         uint8_t     read_addr,
                         uint8_t     *rec_buffer, 
                         uint8_t     num_bytes);
uint8_t i2c_data_send(I2C_TypeDef *I2Cx,
                      uint16_t    slave_addr, 
                      uint8_t     *send_buffer, 
                      uint8_t     num_bytes);

uint8_t i2c_is_done(I2C_TypeDef *I2Cx);
uint32_t i2c_has_error(I2C_TypeDef *I2Cx);

#endif
