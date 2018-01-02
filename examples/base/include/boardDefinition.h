/** ***************************************************************************
 * @file boarDefinition.h DMU380 ARM Cortex M0 I/O pins and interrupts
 * @brief Settings for the DMU380 board (STM32F205RE).
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#define EEPROM_FLASH_SECTOR                 FLASH_Sector_7
#define EEPROM_FLASH_VOLTAGE                VoltageRange_3 // FIXME: this depends on the processor voltage

/// == set up the device's DEBUG-serial as USART ==
/// USART (serial interface) defines: TX - A9
///                                   Rx - A10
#define DEBUG_USART                        USART1
#define DEBUG_USART_CLK                    RCC_APB2Periph_USART1
#define DEBUG_USART_TX_PIN                 GPIO_Pin_9
#define DEBUG_USART_TX_GPIO_PORT           GPIOA
#define DEBUG_USART_TX_GPIO_CLK            RCC_AHB1Periph_GPIOA
#define DEBUG_USART_TX_SOURCE              GPIO_PinSource9
#define DEBUG_USART_TX_AF                  GPIO_AF_USART1
#define DEBUG_USART_RX_PIN                 GPIO_Pin_10
#define DEBUG_USART_RX_GPIO_PORT           GPIOA
#define DEBUG_USART_RX_GPIO_CLK            RCC_AHB1Periph_GPIOA
#define DEBUG_USART_RX_SOURCE              GPIO_PinSource10
#define DEBUG_USART_RX_AF                  GPIO_AF_USART1
#define DEBUG_USART_IRQn                   USART1_IRQn
#define DEBUG_USART_IRQ                    USART1_IRQHandler


/// == user-communications protocol as UART ==
/// The User pins can be SPI (MOSI, MISO, CLK, Select or two UARTS
///   (4 and 5) or a USART and a UART(3 and 5)
#define kUserA_UART                        0 // where it is in the uart.c gUartConfig structure
#define USER_A_UART                        UART4
#define USER_A_UART_CLK                    RCC_APB1Periph_UART4
#define USER_A_UART_TX_PIN                 GPIO_Pin_10
#define USER_A_UART_TX_GPIO_PORT           GPIOC
#define USER_A_UART_TX_GPIO_CLK            RCC_AHB1Periph_GPIOC
#define USER_A_UART_TX_SOURCE              GPIO_PinSource10
#define USER_A_UART_TX_AF                  GPIO_AF_UART4
#define USER_A_UART_RX_PIN                 GPIO_Pin_11
#define USER_A_UART_RX_GPIO_PORT           GPIOC
#define USER_A_UART_RX_GPIO_CLK            RCC_AHB1Periph_GPIOC
#define USER_A_UART_RX_SOURCE              GPIO_PinSource11
#define USER_A_UART_RX_AF                  GPIO_AF_UART4
#define USER_A_UART_IRQn                   UART4_IRQn
#define USER_A_UART_IRQ                    UART4_IRQHandler


#define kUserB_UART                        1 // where it is in the uart.c gUartConfig structure
#define USER_B_UART                        UART5
#define USER_B_UART_CLK                    RCC_APB1Periph_UART5
#define USER_B_UART_TX_PIN                 GPIO_Pin_12
#define USER_B_UART_TX_GPIO_PORT           GPIOC
#define USER_B_UART_TX_GPIO_CLK            RCC_AHB1Periph_GPIOC
#define USER_B_UART_TX_SOURCE              GPIO_PinSource12
#define USER_B_UART_TX_AF                  GPIO_AF_UART5
#define USER_B_UART_RX_PIN                 GPIO_Pin_2
#define USER_B_UART_RX_GPIO_PORT           GPIOD
#define USER_B_UART_RX_GPIO_CLK            RCC_AHB1Periph_GPIOD
#define USER_B_UART_RX_SOURCE              GPIO_PinSource2
#define USER_B_UART_RX_AF                  GPIO_AF_UART5
#define USER_B_UART_IRQn                   UART5_IRQn
#define USER_B_UART_IRQ                    UART5_IRQHandler

// ========= set up the nSS interrupt =========
/// The UART 5 (RX) D2 pin is tied to the SPI3 nSS pin and is configured as an
/// external interrupt pin
#define NSS_INTERRUPT_PIN                 GPIO_Pin_2
#define NSS_INTERRUPT_PORT                GPIOD
#define NSS_INTERRUPT_CLK                 RCC_AHB1Periph_GPIOD
#define NSS_INTERRUPT_SOURCE              GPIO_PinSource2

#define NSS_INTERRUPT_EXTI_LINE           EXTI_Line2
#define NSS_INTERRUPT_EXTI_PORT_SOURCE    EXTI_PortSourceGPIOD
#define NSS_INTERRUPT_EXTI_PIN_SOURCE     EXTI_PinSource2


// ========= set up the data-ready pin B3 =========
#define DATA_READY_PIN                GPIO_Pin_3    // defined as (uint16_t)
#define DATA_READY_PORT               GPIOB
#define DATA_READY_CLK                RCC_AHB1Periph_GPIOB
#define DATA_READY_SOURCE             GPIO_PinSource3

// ========= set up the 1-PPS pin A0 =========
#define ONE_PPS_PIN                GPIO_Pin_0
#define ONE_PPS_PORT               GPIOA
#define ONE_PPS_CLK                RCC_AHB1Periph_GPIOA
#define ONE_PPS_SOURCE             GPIO_PinSource0

// Placeholders for sync use
#define ONE_PPS_EXTI_LINE          EXTI_Line0
#define ONE_EXTI_PORT_SOURCE       EXTI_PortSourceGPIOA
#define ONE_PPS_EXTI_PIN_SOURCE    EXTI_PinSource0

#define ONE_PPS_EXTI_IRQn          EXTI0_IRQn
#define ONE_PPS_EXTI_IRQHandler    EXTI0_IRQHandler

// ========= set up the Unit Configuration Pins =========
#define CONFIG_0_PIN                GPIO_Pin_1
#define CONFIG_0_PORT               GPIOA
#define CONFIG_0_CLK                RCC_AHB1Periph_GPIOA
#define CONFIG_0_SOURCE             GPIO_PinSource1

#define CONFIG_1_PIN                GPIO_Pin_2
#define CONFIG_1_PORT               GPIOA
#define CONFIG_1_CLK                RCC_AHB1Periph_GPIOA
#define CONFIG_1_SOURCE             GPIO_PinSource2

#define CONFIG_2_PIN                GPIO_Pin_3
#define CONFIG_2_PORT               GPIOA
#define CONFIG_2_CLK                RCC_AHB1Periph_GPIOA
#define CONFIG_2_SOURCE             GPIO_PinSource3


// == set up the device's user-communications protocol as SPI 3 ==
//
/// Configure UART4 and UART5 ports as a combined SPI port
/// SPI3 port uses:
///   SPI1_MOSI: C12 (UART5_TX)
///   SPI3_MISO: C11 (UART4_RX)
///   SPI1_SCLK: C10 (UART4_TX)
///   SPI1_NSS:  A15 (connected to D2 - UART5_RX)
#define kUserCommunicationSPI        SPI3

/// MOSI
#define SPI3_MOSI_PIN                GPIO_Pin_12
#define SPI3_MOSI_PORT               GPIOC
#define SPI3_GPIO_MOSI_CLK           RCC_AHB1Periph_GPIOC
#define SPI3_MOSI_SOURCE             GPIO_PinSource12

/// MISO
#define SPI3_MISO_PIN                GPIO_Pin_11
#define SPI3_MISO_PORT               GPIOC
#define SPI3_GPIO_MISO_CLK           RCC_AHB1Periph_GPIOC
#define SPI3_MISO_SOURCE             GPIO_PinSource11

/// SCLK
#define SPI3_SCK_PIN                 GPIO_Pin_10
#define SPI3_SCK_PORT                GPIOC
#define SPI3_GPIO_SCK_CLK            RCC_AHB1Periph_GPIOC
#define SPI3_SCK_SOURCE              GPIO_PinSource10

/// nSS
#define SPI3_SLAVE_SELECT_PIN        GPIO_Pin_15
#define SPI3_SLAVE_SELECT_PORT       GPIOA
#define SPI3_SLAVE_SELECT_CLK        RCC_AHB1Periph_GPIOA
#define SPI3_SLAVE_SELECT_SOURCE     GPIO_PinSource15

/// SPI3 DMA
#define SPI3_DMA                     DMA1
#define SPI3_DMA_CHANNEL             DMA_Channel_0
#define SPI3_DMA_CLK                 RCC_AHB1Periph_DMA1

#define SPI3_DMA_TX_STREAM              DMA1_Stream7
#define SPI3_DMA_TX_STREAM_IRQ          DMA1_Stream7_IRQn
#define SPI3_DMA_TX_STREAM_IRQHANDLER   DMA1_Stream7_IRQHandler
#define SPI3_DMA_RX_STREAM              DMA1_Stream0
#define SPI3_DMA_RX_STREAM_IRQ          DMA1_Stream0_IRQn
#define SPI3_DMA_RX_STREAM_IRQHANDLER   DMA1_Stream0_IRQHandler

// Flags can be 0 to 7 to select
// DMA_FLAG_FEIFx  - FIFO error flag
// DMA_FLAG_DMEIFx - direct mode error flag
// DMA_FLAG_TEIFx  - transfer error flag
// DMA_FLAG_HTIFx  - half transfer complete flag
// DMA_FLAG_TCIFx  - transfer complete flag
#define SPI3_DMA_TX_FLAGS     ( DMA_FLAG_FEIF7  | \
                                DMA_FLAG_DMEIF7 | \
                                DMA_FLAG_TEIF7  | \
                                DMA_FLAG_HTIF7  | \
                                DMA_FLAG_TCIF7 )
#define SPI3_DMA_TX_INT_DONE  DMA_IT_TCIF7
#define SPI3_DMA_RX_FLAGS     ( DMA_FLAG_FEIF0  | \
                                DMA_FLAG_DMEIF0 | \
                                DMA_FLAG_TEIF0  | \
                                DMA_FLAG_HTIF0  | \
                                DMA_FLAG_TCIF0 )
#define SPI3_DMA_RX_INT_DONE  DMA_IT_TCIF0


// ==== magnetometer and temp. sensor communications I2C ====
/// I2C1 port is B6 (SCL)
///              B7 (SDA), magnetometer
#define kMagnetometerI2C        I2C1
#define kTemperatureSensorI2C   I2C1

#define I2C1_SCL_PIN                    GPIO_Pin_6
#define I2C1_SCL_GPIO_PORT              GPIOB
#define I2C1_GPIO_CLK                   RCC_AHB1Periph_GPIOB
#define I2C1_SCL_SOURCE                 GPIO_PinSource6
#define I2C1_SDA_PIN                    GPIO_Pin_7
#define I2C1_SDA_GPIO_PORT              GPIOB
#define I2C1_SDA_SOURCE                 GPIO_PinSource7
#define I2C1_SPEED                      400000
#define I2C1_INTR_SUBPRIO               0x1      // fixme
#define I2C1_INTR_PRIO                  0x6      // fixme

#define I2C1_DMA                        DMA1
#define I2C1_DMA_CHANNEL                DMA_Channel_1
#define I2C1_DMA_CLK                    RCC_AHB1Periph_DMA1
#define I2C1_DMA_RX_STREAM              DMA1_Stream5
#define I2C1_DMA_RX_STREAM_IRQ          DMA1_Stream5_IRQn
#define I2C1_DMA_RX_STREAM_IRQHANDLER   DMA1_Stream5_IRQHandler

#define I2C1_DMA_RX_FLAGS               (DMA_FLAG_FEIF5  | \
                                         DMA_FLAG_DMEIF5 | \
                                         DMA_FLAG_TEIF5  | \
                                         DMA_FLAG_HTIF5  | \
                                         DMA_FLAG_TCIF5)
#define I2C1_DMA_RX_INT_DONE            DMA_IT_TCIF5

/// Magnetometer data ready interrupt B5
#define MAG_DATA_READY_GPIO_PIN              GPIO_Pin_5
#define MAG_DATA_READY_GPIO_PORT             GPIOB
#define MAG_DATA_READY_GPIO_CLK              RCC_AHB1Periph_GPIOB
#define MAG_DATA_READY_EXTI_LINE             EXTI_Line5
#define MAG_DATA_READY_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOB
#define MAG_DATA_READY_EXTI_PIN_SOURCE       EXTI_PinSource5
#define MAG_DATA_READY_EXTI_IRQn             EXTI9_5_IRQn

#define I2C3_SCL_PIN                    GPIO_Pin_8
#define I2C3_SCL_GPIO_PORT              GPIOA
#define I2C3_GPIO_SCL_CLK               RCC_AHB1Periph_GPIOA
#define I2C3_SCL_SOURCE                 GPIO_PinSource8
#define I2C3_SDA_PIN                    GPIO_Pin_9
#define I2C3_SDA_GPIO_PORT              GPIOC
#define I2C3_SDA_SOURCE                 GPIO_PinSource9
#define I2C3_GPIO_SDA_CLK               RCC_AHB1Periph_GPIOC

#define I2C3_DMA                        DMA1
#define I2C3_DMA_CHANNEL                DMA_Channel_3
#define I2C3_DMA_CLK                    RCC_AHB1Periph_DMA1
#define I2C3_DMA_RX_STREAM              DMA1_Stream2
#define I2C3_DMA_RX_STREAM_IRQ          DMA1_Stream2_IRQn
#define I2C3_DMA_RX_STREAM_IRQHANDLER   DMA1_Stream2_IRQHandler


#define I2C3_DMA_RX_FLAGS               (DMA_FLAG_FEIF2  | \
                                         DMA_FLAG_DMEIF2 | \
                                         DMA_FLAG_TEIF2  | \
                                         DMA_FLAG_HTIF2  | \
                                         DMA_FLAG_TCIF2)
#define I2C3_DMA_RX_INT_DONE            DMA_IT_TCIF2

/// accelerometer can go to 2.25MHz but the STM32F2xx cannot,
/// 600000 is possible but the asymetric-ness of the clock gets odd
#define I2C3_SPEED                      400000
#define I2C3_INTR_SUBPRIO               0x1      // fixme
#define I2C3_INTR_PRIO                  0x5      // fixme

#define I2C2_SCL_PIN                        GPIO_Pin_10
#define I2C2_SCL_GPIO_PORT                  GPIOB
#define I2C2_SCL_SOURCE                     GPIO_PinSource10
#define I2C2_SDA_PIN                        GPIO_Pin_11
#define I2C2_SDA_GPIO_PORT                  GPIOB
#define I2C2_SDA_SOURCE                     GPIO_PinSource11
#define I2C2_GPIO_CLK                       RCC_AHB1Periph_GPIOB

#define I2C2_DMA                            DMA1
#define I2C2_DMA_CHANNEL                    DMA_Channel_7
#define I2C2_DMA_CLK                        RCC_AHB1Periph_DMA1
#define I2C2_DMA_RX_STREAM                  DMA1_Stream3
#define I2C2_DMA_RX_STREAM_IRQ              DMA1_Stream3_IRQn
#define I2C2_DMA_RX_STREAM_IRQHANDLER       DMA1_Stream3_IRQHandler


#define I2C2_DMA_RX_FLAGS                   (DMA_FLAG_FEIF3  | \
                                             DMA_FLAG_DMEIF3 | \
                                             DMA_FLAG_TEIF3  | \
                                             DMA_FLAG_HTIF3  | \
                                             DMA_FLAG_TCIF3)
#define I2C2_DMA_RX_INT_DONE                DMA_IT_TCIF3

/// accelerometer can go to 2.25MHz but the STM32F2xx cannot,
/// 600000 is possible but the asymetric-ness of the clock gets odd
#define I2C2_SPEED                          400000
#define I2C2_INTR_SUBPRIO                   0x1      // fixme
#define I2C2_INTR_PRIO                      0x5      // fixme

/// I2C3 port is A8 (SCL)
///              C9 (SDA), accelerometer
#define kAccelerometerI2C               I2C3

 /// Accelerometer data ready interrupt C8
#define ACCEL_DATA_READY_GPIO_PIN              GPIO_Pin_8
#define ACCEL_DATA_READY_GPIO_PORT             GPIOC
#define ACCEL_DATA_READY_GPIO_CLK              RCC_AHB1Periph_GPIOC
#define ACCEL_DATA_READY_EXTI_LINE             EXTI_Line8
#define ACCEL_DATA_READY_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOC
#define ACCEL_DATA_READY_EXTI_PIN_SOURCE       EXTI_PinSource8
#define ACCEL_DATA_READY_EXTI_IRQn             EXTI9_5_IRQn

/// SPI1 port is: A7 (SPI1_MOSI)
///               A6 (SPI1_MISO)
///               A5 (SPI1_SCLK)
///               A4 (SPI1_NSS)
#define kGyroSPI                     SPI1

#define SPI1_MOSI_PIN                GPIO_Pin_7
#define SPI1_MOSI_PORT               GPIOA
#define SPI1_GPIO_MOSI_CLK           RCC_AHB1Periph_GPIOA
#define SPI1_MOSI_SOURCE             GPIO_PinSource7

#define SPI1_MISO_PIN                GPIO_Pin_6
#define SPI1_MISO_PORT               GPIOA
#define SPI1_GPIO_MISO_CLK           RCC_AHB1Periph_GPIOA
#define SPI1_MISO_SOURCE             GPIO_PinSource6

#define SPI1_SCK_PIN                 GPIO_Pin_5
#define SPI1_SCK_PORT                GPIOA
#define SPI1_GPIO_SCK_CLK            RCC_AHB1Periph_GPIOA
#define SPI1_SCK_SOURCE              GPIO_PinSource5


#define SPI1_DMA                        DMA2
#define SPI1_DMA_CLK                    RCC_AHB1Periph_DMA2
#define SPI1_DMA_CHANNEL                DMA_Channel_3
#define SPI1_DMA_RX_STREAM              DMA2_Stream0
#define SPI1_DMA_RX_STREAM_IRQ          DMA2_Stream0_IRQn
#define SPI1_DMA_RX_STREAM_IRQHANDLER   DMA2_Stream0_IRQHandler
#define SPI1_DMA_TX_STREAM              DMA2_Stream3
#define SPI1_DMA_TX_STREAM_IRQHANDLER   DMA2_Stream3_IRQHandler

#define SPI1_DMA_TX_FLAGS               (DMA_FLAG_FEIF3  | \
                                         DMA_FLAG_DMEIF3 | \
                                         DMA_FLAG_TEIF3  | \
                                         DMA_FLAG_HTIF3  | \
                                         DMA_FLAG_TCIF3 )
#define SPI1_DMA_RX_FLAGS               (DMA_FLAG_FEIF0  | \
                                         DMA_FLAG_DMEIF0 | \
                                         DMA_FLAG_TEIF0  | \
                                         DMA_FLAG_HTIF0  | \
                                         DMA_FLAG_TCIF0)
#define SPI1_DMA_RX_INT_DONE            DMA_IT_TCIF0


#define GYRO_SELECT_PIN                  GPIO_Pin_4
#define GYRO_SELECT_PORT                 GPIOA
#define GYRO_SELECT_CLK                  RCC_AHB1Periph_GPIOA

#define MAXGYRO_SELECT_PIN               GYRO_SELECT_PIN
#define MAXGYRO_SELECT_PORT              GYRO_SELECT_PORT
#define MAXGYRO_SELECT_CLK               GYRO_SELECT_CLK



//CAN bus 
#define CAN1_RX_PIN                     GPIO_Pin_8
#define CAN1_RX_GPIO_PORT               GPIOB
#define CAN1_RX_GPIO_CLK                RCC_AHB1Periph_GPIOB
#define CAN1_RX_SOURCE                  GPIO_PinSource8
#define CAN1_TX_PIN                     GPIO_Pin_9
#define CAN1_TX_GPIO_PORT               GPIOB
#define CAN1_TX_GPIO_CLK                RCC_AHB1Periph_GPIOB
#define CAN1_TX_SOURCE                  GPIO_PinSource9

#define CAN2_RX_PIN                     GPIO_Pin_5
#define CAN2_RX_GPIO_PORT               GPIOB
#define CAN2_RX_GPIO_CLK                RCC_AHB1Periph_GPIOB
#define CAN2_RX_SOURCE                  GPIO_PinSource5
#define CAN2_TX_PIN                     GPIO_Pin_13
#define CAN2_TX_GPIO_PORT               GPIOB
#define CAN2_TX_GPIO_CLK                RCC_AHB1Periph_GPIOB
#define CAN2_TX_SOURCE                  GPIO_PinSource13
#define CAN_SPEED                       1000000

#define CAN_RESISTOR_CTRL_PIN           NSS_INTERRUPT_PIN
#define CAN_RESISTOR_CTRL_PORT          NSS_INTERRUPT_PORT
#define CAN_RESISTOR_CTRL_CLK           NSS_INTERRUPT_CLK
#define CAN_RESISTOR_CTRL_SOURCE        NSS_INTERRUPT_SOURCE

/// SPI2 port is: C3  (SPI2_MOSI)
///               C2  (SPI2_MISO)
///               B13 (SPI2_SCLK)
///               B12 (SPI2_NSS)
#define kAccelSPI                    SPI2

#define SPI2_MOSI_PIN                GPIO_Pin_3
#define SPI2_MOSI_PORT               GPIOC
#define SPI2_GPIO_MOSI_CLK           RCC_AHB1Periph_GPIOC
#define SPI2_MOSI_SOURCE             GPIO_PinSource3

#define SPI2_MISO_PIN                GPIO_Pin_2
#define SPI2_MISO_PORT               GPIOC
#define SPI2_GPIO_MISO_CLK           RCC_AHB1Periph_GPIOC
#define SPI2_MISO_SOURCE             GPIO_PinSource2

#define SPI2_SCK_PIN                 GPIO_Pin_13
#define SPI2_SCK_PORT                GPIOB
#define SPI2_GPIO_SCK_CLK            RCC_AHB1Periph_GPIOB
#define SPI2_SCK_SOURCE              GPIO_PinSource13

#define SPI2_SELECT_PIN              GPIO_Pin_12
#define SPI2_SELECT_PORT             GPIOB
#define SPI2_SELECT_CLK              RCC_AHB1Periph_GPIOB
#define SPI2_SELECT_SOURCE           GPIO_PinSource12

#define SPI2_DMA                        DMA1
#define SPI2_DMA_CLK                    RCC_AHB1Periph_DMA1
#define SPI2_DMA_CHANNEL                DMA_Channel_0
#define SPI2_DMA_RX_STREAM              DMA1_Stream3
#define SPI2_DMA_RX_STREAM_IRQ          DMA1_Stream3_IRQn
#define SPI2_DMA_RX_STREAM_IRQHANDLER   DMA1_Stream3_IRQHandler
#define SPI2_DMA_TX_STREAM              DMA1_Stream4
#define SPI2_DMA_TX_STREAM_IRQHANDLER   DMA1_Stream4_IRQHandler

#define SPI2_DMA_TX_FLAGS               (DMA_FLAG_FEIF4  | \
                                         DMA_FLAG_DMEIF4 | \
                                         DMA_FLAG_TEIF4  | \
                                         DMA_FLAG_HTIF4  | \
                                         DMA_FLAG_TCIF4 )
#define SPI2_DMA_TX_INT_DONE            DMA_IT_TCIF4

#define SPI2_DMA_RX_FLAGS               (DMA_FLAG_FEIF3  | \
                                         DMA_FLAG_DMEIF3 | \
                                         DMA_FLAG_TEIF3  | \
                                         DMA_FLAG_HTIF3  | \
                                         DMA_FLAG_TCIF3)
#define SPI2_DMA_RX_INT_DONE            DMA_IT_TCIF3
