/** ***************************************************************************
 * @file boardDefinition.h RTK330 ARM Cortex M4 I/O pins and interrupts
 * @brief Settings for the RTK330 board (STM32F469NI).
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************///
/*******************************************************************************
Copyright 2019 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
* HISTORY***********************************************************************
* 15/10/2019  |                                             | Daich
* Description:  modify pin mapping bug
*******************************************************************************/
#ifndef __BOARD_DEFINITION_H
#define __BOARD_DEFINITION_H

#include "stm32f4xx_hal_conf.h"
#define	APP_NVIC_OFFSET			0X10000
#define BOOTLOADER_NVIC_OFFSET  0x00000

//================== DEBUG-GPIOs
#define DEBUG_GPIO1_PIN                     GPIO_PIN_11
#define DEBUG_GPIO1_PORT                    GPIOD
#define DEBUG_GPIO1_CLK_ENABLE()            __HAL_RCC_GPIOD_CLK_ENABLE()

#define DEBUG_GPIO2_PIN                     GPIO_PIN_12
#define DEBUG_GPIO2_PORT                    GPIOD
#define DEBUG_GPIO2_CLK_ENABLE()            __HAL_RCC_GPIOD_CLK_ENABLE()


//================== Leds
#define LED1_PIN                            GPIO_PIN_8
#define LED1_PORT                           GPIOB
#define LED2_PIN                            GPIO_PIN_9
#define LED2_PORT                           GPIOB
#define LED3_PIN                            GPIO_PIN_6
#define LED3_PORT                           GPIOA
#define LED1_CLK_ENABLE()                   __HAL_RCC_GPIOB_CLK_ENABLE()  
#define LED2_CLK_ENABLE()                   __HAL_RCC_GPIOB_CLK_ENABLE()  
#define LED3_CLK_ENABLE()                   __HAL_RCC_GPIOA_CLK_ENABLE()  


//================== STA9100
#define ST_MODE_PIN                         GPIO_PIN_5
#define ST_MODE_PORT                        GPIOA
#define ST_MODE_CLK_ENABLE()                __HAL_RCC_GPIOA_CLK_ENABLE()  

#ifndef VERSION1
#define ST_RESET_PIN                        GPIO_PIN_1
#define ST_RESET_PORT                       GPIOD
#define ST_RESET_CLK_ENABLE()               __HAL_RCC_GPIOD_CLK_ENABLE()  
#else
#define ST_RESET_PIN                        GPIO_PIN_10
#define ST_RESET_PORT                       GPIOC
#define ST_RESET_CLK_ENABLE()               __HAL_RCC_GPIOC_CLK_ENABLE()  
#endif

#define ST_STDBY_PIN                        GPIO_PIN_6
#define ST_STDBY_PORT                       GPIOC
#define ST_STDBY_CLK_ENABLE()               __HAL_RCC_GPIOC_CLK_ENABLE()  

#define ST_WKUP_PIN                         GPIO_PIN_2
#define ST_WKUP_PORT                        GPIOC
#define ST_WKUP_CLK_ENABLE()                __HAL_RCC_GPIOC_CLK_ENABLE()  

#define ST_BOOT_PIN                         GPIO_PIN_0
#define ST_BOOT_PORT                        GPIOD
#define ST_BOOT_CLK_ENABLE()                __HAL_RCC_GPIOD_CLK_ENABLE()  

#define ST_PROG_BUF_CTL_PIN                 GPIO_PIN_0
#define ST_PROG_BUF_CTL_PORT                GPIOC
#define ST_PROG_BUF_CTL_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()  

#define ST_UART2_TX_PIN                     GPIO_PIN_10
#define ST_UART2_RX_PIN                     GPIO_PIN_11
#define ST_UART2_PORT                       GPIOB
#define ST_UART2_CLK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()  

#ifndef VERSION1
#define ST_PPS_PIN                          GPIO_PIN_0
#define ST_PPS_PORT                         GPIOA
#define ST_PPS_CLK_ENABLE()                 __HAL_RCC_GPIOA_CLK_ENABLE()
#define ST_PPS_IRQn                         EXTI0_IRQn
#define ST_PPS_IRQ                          EXTI0_IRQHandler
#else
#define ST_PPS_PIN                          GPIO_PIN_3
#define ST_PPS_PORT                         GPIOC
#define ST_PPS_CLK_ENABLE()                 __HAL_RCC_GPIOC_CLK_ENABLE()
#define ST_PPS_IRQn                         EXTI3_IRQn
#define ST_PPS_IRQ                          EXTI3_IRQHandler
#endif
//================== GPS-serial
#define GPS_USART                           USART3
#define GPS_USART_CLK_ENABLE()              __HAL_RCC_USART3_CLK_ENABLE()
#define GPS_USART_TX_PIN                    GPIO_PIN_10
#define GPS_USART_TX_GPIO_PORT              GPIOB
#define GPS_USART_TX_AF                     GPIO_AF7_USART3
#define GPS_USART_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define GPS_USART_RX_PIN                    GPIO_PIN_11
#define GPS_USART_RX_GPIO_PORT              GPIOB
#define GPS_USART_TX_RX_GPIO_PORT           GPIOB

#define GPS_USART_RX_AF                     GPIO_AF7_USART3
#define GPS_USART_TX_RX_AF                  GPIO_AF7_USART3
#define GPS_USART_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define GPS_USART_IRQn                      USART3_IRQn
#define GPS_USART_IRQ                       USART3_IRQHandler
#define GPS_UART_RCC_CLK_ENABLE()           __HAL_RCC_USART3_CLK_ENABLE()
#define GPS_UART_PORT_RCC_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define GPS_USART_DMA                       DMA1
#define GPS_USART_DMA_CLK_ENABLE()          __HAL_RCC_DMA1_CLK_ENABLE()
#define GPS_USART_DMA_CHANNEL               DMA_CHANNEL_4
#define GPS_USART_DMA_TX_STREAM             DMA1_Stream3
#define GPS_USART_DMA_TX_STREAM_IRQ         DMA1_Stream3_IRQn
#define GPS_USART_DMA_TX_IRQHandler         DMA1_Stream3_IRQHandler
#define GPS_USART_DMA_RX_STREAM             DMA1_Stream1
#define GPS_USART_DMA_RX_STREAM_IRQ         DMA1_Stream1_IRQn
#define GPS_USART_DMA_RX_IRQHandler         DMA1_Stream1_IRQHandler

//================== User-serial
#if 0
#define USER_USART                          USART1
#define USER_USART_CLK_ENABLE()             __HAL_RCC_USART1_CLK_ENABLE()
#define USER_USART_TX_PIN                   GPIO_PIN_9
#define USER_USART_TX_GPIO_PORT             GPIOA
#define USER_USART_TX_AF                    GPIO_AF7_USART1
#define USER_USART_TX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define USER_USART_RX_PIN                   GPIO_PIN_10
#define USER_USART_RX_GPIO_PORT             GPIOA
#define USER_USART_TX_RX_GPIO_PORT          GPIOA
#define USER_USART_RX_AF                    GPIO_AF7_USART1
#define USER_USART_TX_RX_AF                 GPIO_AF7_USART1
#define USER_USART_RX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define USER_USART_IRQn                     USART1_IRQn
#define USER_USART_IRQ                      USART1_IRQHandler
#define USER_UART_RCC_CLK_ENABLE()          __HAL_RCC_USART1_CLK_ENABLE()
#define USER_UART_PORT_RCC_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()


#define USER_USART_DMA                      DMA2
#define USER_USART_DMA_CLK_ENABLE()         __HAL_RCC_DMA2_CLK_ENABLE()
#define USER_USART_DMA_CHANNEL              DMA_CHANNEL_4
#define USER_USART_DMA_TX_STREAM            DMA2_Stream7
#define USER_USART_DMA_TX_STREAM_IRQ        DMA2_Stream7_IRQn
#define USER_USART_DMA_TX_IRQHandler        DMA2_Stream7_IRQHandler
#define USER_USART_DMA_RX_STREAM            DMA2_Stream2
#define USER_USART_DMA_RX_STREAM_IRQ        DMA2_Stream2_IRQn
#define USER_USART_DMA_RX_IRQHandler        DMA2_Stream2_IRQHandler

//================== Debug-serial

#define DEBUG_UART                          UART5
#define DEBUG_UART_TX_PIN                   GPIO_PIN_12
#define DEBUG_UART_TX_GPIO_PORT             GPIOC
#define DEBUG_UART_TX_AF                    GPIO_AF8_UART5
#define DEBUG_UART_TX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()
#define DEBUG_UART_RX_PIN                   GPIO_PIN_2
#define DEBUG_UART_RX_GPIO_PORT             GPIOD
#define DEBUG_UART_RX_AF                    GPIO_AF8_UART5
#define DEBUG_UART_RX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOD_CLK_ENABLE()
#define DEBUG_UART_IRQn                     UART5_IRQn
#define DEBUG_UART_IRQ                      UART5_IRQHandler
#define DEBUG_UART_RCC_CLK_ENABLE()         __HAL_RCC_UART5_CLK_ENABLE()
#define DEBUG_UART_PORT_RCC_CLK_ENABLE()    {__HAL_RCC_GPIOD_CLK_ENABLE();  \
                                            __HAL_RCC_GPIOD_CLK_ENABLE();}

#define DEBUG_UART_DMA                      DMA1
#define DEBUG_UART_DMA_CLK_ENABLE()         __HAL_RCC_DMA1_CLK_ENABLE()
#define DEBUG_UART_DMA_CHANNEL              DMA_CHANNEL_4
#define DEBUG_UART_DMA_TX_STREAM            DMA1_Stream7
#define DEBUG_UART_DMA_TX_STREAM_IRQ        DMA1_Stream7_IRQn
#define DEBUG_UART_DMA_TX_IRQHandler        DMA1_Stream7_IRQHandler
#define DEBUG_UART_DMA_RX_STREAM            DMA1_Stream0
#define DEBUG_UART_DMA_RX_STREAM_IRQ        DMA1_Stream0_IRQn
#define DEBUG_UART_DMA_RX_IRQHandler        DMA1_Stream0_IRQHandler

#endif


#define DEBUG_UART                          USART1 
#define DEBUG_UART_CLK_ENABLE()             __HAL_RCC_USART1_CLK_ENABLE()
#define DEBUG_UART_TX_PIN                   GPIO_PIN_9
#define DEBUG_UART_TX_GPIO_PORT             GPIOA
#define DEBUG_UART_TX_AF                    GPIO_AF7_USART1
#define DEBUG_UART_TX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define DEBUG_UART_RX_PIN                   GPIO_PIN_10
#define DEBUG_UART_RX_GPIO_PORT             GPIOA
#define DEBUG_UART_TX_RX_GPIO_PORT          GPIOA
#define DEBUG_UART_RX_AF                    GPIO_AF7_USART1
#define DEBUG_UART_TX_RX_AF                 GPIO_AF7_USART1
#define DEBUG_UART_RX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define DEBUG_UART_IRQn                     USART1_IRQn
#define DEBUG_UART_IRQ                      USART1_IRQHandler
#define DEBUG_UART_RCC_CLK_ENABLE()          __HAL_RCC_USART1_CLK_ENABLE()
#define DEBUG_UART_PORT_RCC_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()


#define DEBUG_UART_DMA                      DMA2
#define DEBUG_UART_DMA_CLK_ENABLE()         __HAL_RCC_DMA2_CLK_ENABLE()
#define DEBUG_UART_DMA_CHANNEL              DMA_CHANNEL_4
#define DEBUG_UART_DMA_TX_STREAM            DMA2_Stream7
#define DEBUG_UART_DMA_TX_STREAM_IRQ        DMA2_Stream7_IRQn
#define DEBUG_UART_DMA_TX_IRQHandler        DMA2_Stream7_IRQHandler
#define DEBUG_UART_DMA_RX_STREAM            DMA2_Stream2
#define DEBUG_UART_DMA_RX_STREAM_IRQ        DMA2_Stream2_IRQn
#define DEBUG_UART_DMA_RX_IRQHandler        DMA2_Stream2_IRQHandler

//================== Debug-serial

#define USER_USART                          UART5
#define USER_USART_TX_PIN                   GPIO_PIN_12
#define USER_USART_TX_GPIO_PORT             GPIOC
#define USER_USART_TX_AF                    GPIO_AF8_UART5
#define USER_USART_TX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()
#define USER_USART_RX_PIN                   GPIO_PIN_2
#define USER_USART_RX_GPIO_PORT             GPIOD
#define USER_USART_RX_AF                    GPIO_AF8_UART5
#define USER_USART_RX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOD_CLK_ENABLE()
#define USER_USART_IRQn                     UART5_IRQn
#define USER_USART_IRQ                      UART5_IRQHandler
#define USER_USART_RCC_CLK_ENABLE()         __HAL_RCC_UART5_CLK_ENABLE()
#define USER_USART_PORT_RCC_CLK_ENABLE()    {__HAL_RCC_GPIOD_CLK_ENABLE();  \
                                            __HAL_RCC_GPIOD_CLK_ENABLE();}

#define USER_USART_DMA                      DMA1
#define USER_USART_DMA_CLK_ENABLE()         __HAL_RCC_DMA1_CLK_ENABLE()
#define USER_USART_DMA_CHANNEL              DMA_CHANNEL_4
#define USER_USART_DMA_TX_STREAM            DMA1_Stream7
#define USER_USART_DMA_TX_STREAM_IRQ        DMA1_Stream7_IRQn
#define USER_USART_DMA_TX_IRQHandler        DMA1_Stream7_IRQHandler
#define USER_USART_DMA_RX_STREAM            DMA1_Stream0
#define USER_USART_DMA_RX_STREAM_IRQ        DMA1_Stream0_IRQn
#define USER_USART_DMA_RX_IRQHandler        DMA1_Stream0_IRQHandler

//================== Bluetooth-serial
#define BT_USART                            USART2
#define BT_USART_CLK_ENABLE()               __HAL_RCC_USART2_CLK_ENABLE()
#define BT_USART_TX_PIN                     GPIO_PIN_5
#define BT_USART_TX_GPIO_PORT               GPIOD
#define BT_USART_TX_AF                      GPIO_AF7_USART2
#define BT_USART_TX_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()
#define BT_USART_RX_PIN                     GPIO_PIN_6
#define BT_USART_RX_GPIO_PORT               GPIOD
#define BT_USART_TX_RX_GPIO_PORT            GPIOD
#define BT_USART_RX_AF                      GPIO_AF7_USART2
#define BT_USART_TX_RX_AF                   GPIO_AF7_USART2
#define BT_USART_RX_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()
#define BT_USART_IRQn                       USART2_IRQn
#define BT_USART_IRQ                        USART2_IRQHandler
#define BT_UART_RCC_CLK_ENABLE()            __HAL_RCC_USART2_CLK_ENABLE()
#define BT_UART_PORT_RCC_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()



#define BT_USART_DMA                        DMA1
#define BT_USART_DMA_CLK_ENABLE()           __HAL_RCC_DMA1_CLK_ENABLE()
#define BT_USART_DMA_CHANNEL                DMA_CHANNEL_4
#define BT_USART_DMA_TX_STREAM              DMA1_Stream6
#define BT_USART_DMA_TX_STREAM_IRQ          DMA1_Stream6_IRQn
#define BT_USART_DMA_TX_IRQHandler          DMA1_Stream6_IRQHandler
#define BT_USART_DMA_RX_STREAM              DMA1_Stream5
#define BT_USART_DMA_RX_STREAM_IRQ          DMA1_Stream5_IRQn
#define BT_USART_DMA_RX_IRQHandler          DMA1_Stream5_IRQHandler

// PF11 ESP32_BOOT_CTL
#define BT_BOOT_CTL_PIN                     GPIO_PIN_11
#define BT_BOOT_CTL_GPIO_PORT               GPIOF
#define BT_BOOT_CTL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOF_CLK_ENABLE()

// PF14 ESP32_RESET
#define BT_RESET_PIN                        GPIO_PIN_14
#define BT_RESET_GPIO_PORT                  GPIOF
#define BT_RESET_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOF_CLK_ENABLE()


//================== ETH
#define ETH_PORT_AF                         GPIO_AF11_ETH
#define ETH_PORT_CLK_ENABLE()               __HAL_RCC_ETH_CLK_ENABLE()
#define ETH_MDIO_PIN                        GPIO_PIN_2
#define ETH_MDIO_PORT                       GPIOA
#define ETH_MDIO_CLK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()
#define ETH_MDC_PIN                         GPIO_PIN_1
#define ETH_MDC_PORT                        GPIOC
#define ETH_MDC_CLK_ENABLE()                __HAL_RCC_GPIOC_CLK_ENABLE()
#define ETH_RMII_REF_CLK_PIN                GPIO_PIN_1
#define ETH_RMII_REF_CLK_PORT               GPIOA
#define ETH_RMII_REF_CLK_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define ETH_RMII_CRS_DV_PIN                 GPIO_PIN_7
#define ETH_RMII_CRS_DV_PORT                GPIOA
#define ETH_RMII_CRS_DV_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define ETH_RMII_RXD0_PIN                   GPIO_PIN_4
#define ETH_RMII_RXD0_PORT                  GPIOC
#define ETH_RMII_RXD0_CLK_ENABLE()          __HAL_RCC_GPIOC_CLK_ENABLE()
#define ETH_RMII_RXD1_PIN                   GPIO_PIN_5
#define ETH_RMII_RXD1_PORT                  GPIOC
#define ETH_RMII_RXD1_CLK_ENABLE()          __HAL_RCC_GPIOC_CLK_ENABLE()
#define ETH_RMII_TX_EN_PIN                  GPIO_PIN_11
#define ETH_RMII_TX_EN_PORT                 GPIOG
#define ETH_RMII_TX_EN_CLK_ENABLE()         __HAL_RCC_GPIOG_CLK_ENABLE()
#define ETH_RMII_TXD0_PIN                   GPIO_PIN_13
#define ETH_RMII_TXD0_PORT                  GPIOG
#define ETH_RMII_TXD0_CLK_ENABLE()          __HAL_RCC_GPIOG_CLK_ENABLE()
#define ETH_RMII_TXD1_PIN                   GPIO_PIN_14
#define ETH_RMII_TXD1_PORT                  GPIOG
#define ETH_RMII_TXD1_CLK_ENABLE()          __HAL_RCC_GPIOG_CLK_ENABLE()
#ifndef VERSION1
#define ETH_RESET_PIN                       GPIO_PIN_7
#define ETH_RESET_PORT                      GPIOI
#define ETH_RESET_CLK_ENABLE()              __HAL_RCC_GPIOI_CLK_ENABLE()
#else
#define ETH_RESET_PIN                       GPIO_PIN_0
#define ETH_RESET_PORT                      GPIOE
#define ETH_RESET_CLK_ENABLE()              __HAL_RCC_GPIOE_CLK_ENABLE()
#endif
//================== SPI
/*  PF6     MCU_USER_NSS    SPI5_NSS
 *  PF7     MCU_USER_SCK    SPI5_SCK
 *  PF8     MCU_USER_MISO   SPI5_MISO
 *  PF9     MCU_USER_MOSI   SPI5_MOSI
 *  PB12    MCU_USER_DRDY  
 */
#define USER_SPI                            SPI5
#define USER_SPI_CLK_ENABLE()               __HAL_RCC_SPI5_CLK_ENABLE()

/// MOSI
#define USER_SPI_MOSI_PIN                   GPIO_PIN_9
#define USER_SPI_MOSI_PORT                  GPIOF
#define USER_SPI_MOSI_AF                    GPIO_AF5_SPI5
#define USER_SPI_MOSI_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOF_CLK_ENABLE()

/// MISO
#define USER_SPI_MISO_PIN                   GPIO_PIN_8
#define USER_SPI_MISO_PORT                  GPIOF
#define USER_SPI_MISO_AF                    GPIO_AF5_SPI5
#define USER_SPI_MISO_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOF_CLK_ENABLE()

/// SCK
#define USER_SPI_SCK_PIN                    GPIO_PIN_7
#define USER_SPI_SCK_PORT                   GPIOF
#define USER_SPI_SCK_AF                     GPIO_AF5_SPI5
#define USER_SPI_SCK_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOF_CLK_ENABLE()

/// NSS
#define USER_SPI_NSS_PIN                    GPIO_PIN_6
#define USER_SPI_NSS_PORT                   GPIOF
#define USER_SPI_NSS_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOF_CLK_ENABLE()
#define USER_SPI_NSS_RX_IRQn                EXTI9_5_IRQn
#define USER_SPI_NSS_RX_IRQ                 EXTI9_5_IRQHandler

/// DRDY
#define USER_SPI_DRDY_PIN                   GPIO_PIN_12
#define USER_SPI_DRDY_PORT                  GPIOB
#define USER_SPI_DRDY_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()

/// USER_SPI DMA
#define USER_SPI_DMA                        DMA2
#define USER_SPI_DMA_CLK_ENABLE()           __HAL_RCC_DMA2_CLK_ENABLE()
#define USER_SPI_DMA_CHANNEL                DMA_CHANNEL_2
#define USER_SPI_DMA_TX_STREAM              DMA2_Stream4
#define USER_SPI_DMA_TX_STREAM_IRQ          DMA2_Stream4_IRQn
#define USER_SPI_DMA_TX_STREAM_IRQHANDLER   DMA2_Stream4_IRQHandler
#define USER_SPI_DMA_RX_STREAM              DMA2_Stream3
#define USER_SPI_DMA_RX_STREAM_IRQ          DMA2_Stream3_IRQn
#define USER_SPI_DMA_RX_STREAM_IRQHANDLER   DMA2_Stream3_IRQHandler


//================== CAN
/*  PA12     CAN_TX         CAN1_TX 
 *  PA11     CAN_RX         CAN1_RX
 *  PC9      CAN_AB     
 *  PC8      CAN_120R_CTL
 */
#define CAR_CAN                             CAN1
#define CAR_CAN_CLK_ENABLE()                __HAL_RCC_CAN1_CLK_ENABLE()

/// CAN_TX
#define CAR_CAN_TX_PIN                      GPIO_PIN_12
#define CAR_CAN_TX_PORT                     GPIOA
#define CAR_CAN_TX_AF                       GPIO_AF9_CAN1
#define CAR_CAN_TX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()

/// CAN_RX
#define CAR_CAN_RX_PIN                      GPIO_PIN_11
#define CAR_CAN_RX_PORT                     GPIOA
#define CAR_CAN_RX_AF                       GPIO_AF9_CAN1
#define CAR_CAN_RX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()

/// CAN_AB
#define CAR_CAN_AB_PIN                      GPIO_PIN_9
#define CAR_CAN_AB_PORT                     GPIOC
#define CAR_CAN_AB_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()

/// CAN_120R_CTL
#define CAR_CAN_120R_CTL_PIN                GPIO_PIN_8
#define CAR_CAN_120R_CTL_PORT               GPIOC
#define CAR_CAN_120R_CTL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()

#define CAR_CAN_RX_IRQn                     CAN1_RX0_IRQn
#define CAR_CAN_RX_IRQ                      CAN1_RX0_IRQHandler


//================== Sensors
#define SPI_MOSI_PIN                       GPIO_PIN_0
#define SPI_MOSI_PORT                      GPIOB
#define SPI_MOSI_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()

#define SPI_SCK_PIN                        GPIO_PIN_1
#define SPI_SCK_PORT                       GPIOB
#define SPI_SCK_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()

#define SPI_MISO1_PIN                      GPIO_PIN_2
#define SPI_MISO2_PIN                      GPIO_PIN_3
#define SPI_MISO3_PIN                      GPIO_PIN_4
#define SPI_MISO_PORT                      GPIOB
#define SPI_MISO_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()

#define SPI_NSS1_PIN                       GPIO_PIN_5
#define SPI_NSS2_PIN                       GPIO_PIN_6
#define SPI_NSS3_PIN                       GPIO_PIN_7
#define SPI_NSS_PORT                       GPIOB 
#define SPI_NSS_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()

#define SPI_MISO_PINS                      (SPI_MISO1_PIN | SPI_MISO2_PIN | SPI_MISO3_PIN)
#define SPI_NSS_PINS                       (SPI_NSS1_PIN | SPI_NSS2_PIN | SPI_NSS3_PIN)


#endif
