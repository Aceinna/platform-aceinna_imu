/** ***************************************************************************
 * @file   configureGPIO.c
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * Initialize the GPIO pins to output for the 380 board
 ******************************************************************************/
#include "stm32f2xx_conf.h"
#include "configureGPIO.h"
#include "timer.h"
#include "xbowsp_init.h"   // for IsInternalGPS()

/** ****************************************************************************
 * @name _InitCommonPins_GPIO LOCAL
 *
 * @brief Initialize the GPIO pins common to the UART and SPI3 communication
 *        ports as input pins (one will be configured an output based on the
 *        communications protocol selected)
 * Trace:
 *
 * @param [In] N/A
 * @param [Out] N/A
 * @retval N/A
 ******************************************************************************/
void _InitCommonPins_GPIO( void )
{
    /// ---- Initialize UARTB RX and SPI3 nSS as input pins so, regardless of how
    ///      SPI is configured (in 'spi_configure'), neither interferes with the
    ///      other as they are tied together ----
    InitPin_GPIO( SPI3_SLAVE_SELECT_CLK,
                  SPI3_SLAVE_SELECT_PORT,
                  SPI3_SLAVE_SELECT_PIN,
                  GPIO_INPUT );

    InitPin_GPIO( USER_B_UART_RX_GPIO_CLK,
                  USER_B_UART_RX_GPIO_PORT,
                  USER_B_UART_RX_PIN,
                  GPIO_INPUT );
}


/** ****************************************************************************
 * @name InitUnitConfigurationPins LOCAL
 *
 * @brief Initialize the configuration pins
 * Trace:
 *
 * @param [In] N/A
 * @param [Out] N/A
 * @retval N/A
 ******************************************************************************/
void _InitUnitConfigurationPins( void )
{
    InitPin_GPIO( CONFIG_0_CLK, CONFIG_0_PORT, CONFIG_0_PIN, GPIO_INPUT );
    InitPin_GPIO( CONFIG_1_CLK, CONFIG_1_PORT, CONFIG_1_PIN, GPIO_INPUT );
    InitPin_GPIO( CONFIG_2_CLK, CONFIG_2_PORT, CONFIG_2_PIN, GPIO_INPUT );
}


/** ****************************************************************************
 * @name InitBoardConfiguration_GPIO
 *
 * @brief Initialize the pins for the 380 board called in main()
 * Trace:
 *
 * @param [In] N/A
 * @param [Out] N/A
 * @retval N/A
 ******************************************************************************/
void InitBoardConfiguration_GPIO()
{
    /// SPI DR pin as input. If pulled low at power up, user communication will
    ///   be handled via UART and the DR pin will not be used. If pulled high
    ///   then communications is SPI.  pin will be configured as an
    ///   output pin and used as SPI DR (set high to indicate that the data is
    ///   ready and reset to low upon a data read).
    InitPin_GPIO( DATA_READY_CLK,
                  DATA_READY_PORT,
                  DATA_READY_PIN,
                  GPIO_INPUT );

    _InitUnitConfigurationPins(); // as inputs (logic levels on pins read later)

    /// If pulled low at power up, the pin will be used as an input whose signal
    ///   syncs the clock to the GPS 1PPS signal. If not pulled low, the pin
    ///   will provide a 1PPS (or other frequency) signal.
    InitPin_GPIO( ONE_PPS_CLK,
                  ONE_PPS_PORT,
                  ONE_PPS_PIN,
                  GPIO_INPUT );

    /// Initialize the nSS and UARTB RX pins as input-pins (they share a common
    ///    line).  Change the status of the pins in taskUserCommunication()
    ///    depending on which communication mode is selected.
    _InitCommonPins_GPIO();

    // Used for internal/external oscillator selection information: IO3 - pin B11
    InitPin_GPIO( RCC_AHB1Periph_GPIOB,
                  GPIOB,
                  GPIO_Pin_11,   // IO3 B11
                  GPIO_OUTPUT ); // send out a signal

    // IO2 - pin B12 - used for GPS wakeup or debugging
    if( IsInternalGPS() ) {
        InitPin_GPIO( RCC_AHB1Periph_GPIOB,
                      GPIOB,
                      GPIO_Pin_12,   // FL: IO2 B12 is wakeup signal FROM gps chip
                      GPIO_INPUT ); // send out a signal
    } else {
        InitPin_GPIO( RCC_AHB1Periph_GPIOB,
                      GPIOB,
                      GPIO_Pin_12,
                      GPIO_OUTPUT );
    }
}


/** ****************************************************************************
 * @name ReadUnitConfiguration_GPIO
 *
 * @brief Initialize the pins for the 380 board called in main()
 * Trace:
 *
 * @param [In] N/A
 * @param [Out] N/A
 * @retval N/A
 ******************************************************************************/
uint8_t ReadUnitConfiguration_GPIO( void )
{
    uint8_t tmp = 0x00;
    uint8_t bit0, bit1, bit2;

    bit0 = GPIO_ReadInputDataBit( CONFIG_0_PORT, CONFIG_0_PIN );
    bit1 = GPIO_ReadInputDataBit( CONFIG_1_PORT, CONFIG_1_PIN );
    bit2 = GPIO_ReadInputDataBit( CONFIG_2_PORT, CONFIG_2_PIN );

    tmp = bit0 << 0 |
          bit1 << 1 |
          bit2 << 2;

    return tmp;
}


/** ****************************************************************************
 * @name InitPin_GPIO
 *
 * @brief Initialize a GPIO pin (using peripheral clock AHB1) as an input/output
 *        pin
 * Trace:
 *
 * @param [In] PeriphClock clock ID
 * @param [In] GPIO_Port GPIO port
 * @param [In] GPIO_Pin  GPIO Pin
 * @param [In] inputOutputSelector - input or output
 * @param [Out] N/A
 * @retval N/A
 ******************************************************************************/
void InitPin_GPIO( uint32_t     PeriphClock,
                   GPIO_TypeDef *GPIO_Port,
                   uint32_t     GPIO_Pin,
                   uint8_t      inputOutputSelector )
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Set up the peripheral clocks
    RCC_AHB1PeriphClockCmd( PeriphClock, ENABLE );

    if( inputOutputSelector == GPIO_INPUT ) {
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;  /// INPUT/output/alt func/analog
    } else {
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT; /// input/OUTPUT/alt func/analog
    }

    /// Configure the pins as Push-Pull with a Pull-Up resistor
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    /// PUSH-PULL or open-drain
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;     /// UP/Down/NoPull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; /// low/med/fast/high speed

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
    GPIO_Init( GPIO_Port, &GPIO_InitStructure);
}

