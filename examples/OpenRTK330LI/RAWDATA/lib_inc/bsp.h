/** ***************************************************************************
 * @file bsp.h Board Support package, configure the Cortex M3
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/

#ifndef _BSP_H
#define _BSP_H
#include "stm32f4xx_hal.h"

#define TASK_TIMER_IRQ                       TIM2_IRQHandler
#define ON 		GPIO_PIN_SET
#define OFF 	GPIO_PIN_RESET
#define SPI_CHIPS_DEFAULT  0x00000007
#define SPI_NSS_PIN_OFFSET 0x05
#define SPI_NSS_MASK       (SPI_CHIPS_DEFAULT <<  SPI_NSS_PIN_OFFSET)            //0x000000E0


// need to modify
void MX_DMA_Init(void);
#if 0
void MX_USART1_UART_Init(void);
void MX_UART5_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART2_UART_Init(void);
#endif
//================== Normal mode
/// DEBUG pins
void    BSP_DEBUG_GPIOS_Gpio_Init(void);

/// leds
void   BSP_LEDS_Gpio_Init(void);

/// STA9100
void    BSP_STA9100_Interface_Gpio_Init(void);

/// ESP32
void    BSP_ESP32_Interface_Gpio_Init(void);

/// SENSORS
void    BSP_SENSOR_Interface_Gpio_Init(void);

/// board init steps
void    SetIntVectorOffset(uint32_t offset);
void    SystemClock_Config(void);
void    BSP_GPIO_Init(void);

/// api
void    BoardInit(void);

//================== IAP mode
/// bootmode detect —— switch
void    BSP_BootMode_Detect_Gpio_Init(void);
int     BootMode_Detect(void);

/// STA9100 iap
void    BSP_STA9100_Gpio_Init(void);
void    BSP_STA9100_Reset(void);
void    BSP_STA9100_Enter_BOOT_MODE(void);
void    BSP_STA9100_Enter_PASSTHROUIGH_MODE(void);

/// ESP32 iap
void    set_esp32_to_boot_mode(void);

/// api
void    ResetForEnterBootMode(void);


//================== common functions
void    delay_ms(uint32_t time);
void    DelayMs(uint32_t msec);

void BSP_Spi_Pins_For_Test(void);
void DRDY_Toggle(void);
void NSS_Toggle(void);


#define SPI_MOSI_ON         (HAL_GPIO_WritePin(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_PIN_SET))
#define SPI_MOSI_OFF        (HAL_GPIO_WritePin(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_PIN_RESET))

#define SPI_SCK_ON          (HAL_GPIO_WritePin(SPI_SCK_PORT,  SPI_SCK_PIN,  GPIO_PIN_SET))
#define SPI_SCK_OFF         (HAL_GPIO_WritePin(SPI_SCK_PORT,  SPI_SCK_PIN,  GPIO_PIN_RESET))

#define SPI_NSS_ALL_ON      (HAL_GPIO_WritePin(SPI_NSS_PORT, ((SPI_NSS_PINS)&SPI_NSS_MASK), GPIO_PIN_SET))
#define SPI_NSS_ALL_OFF     (HAL_GPIO_WritePin(SPI_NSS_PORT, ((SPI_NSS_PINS)&SPI_NSS_MASK), GPIO_PIN_RESET))

#define SPI(x,y) ((y) ? (x ##_ON): (x ##_OFF))

void HW_JumpToApp(void);
void esp32_reset();


#endif /* CONF_GPIO_H */

