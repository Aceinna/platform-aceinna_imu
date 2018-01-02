/** ***************************************************************************
 * @file watchdog.c cpu IWDG hardware watchdog timer interface functions
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
#include <stm32f2xx.h>
#include "watchdog.h"

/** ****************************************************************************
 * @name InitTimer_Watchdog
 * @brief Configure Independant watchdog
 * Trace:
 * @param [in] NewState - if enable set up the system and start it running
 * @retval N/A
 ******************************************************************************/
void InitTimer_Watchdog( FunctionalState NewState )
{
    if( NewState == ENABLE )
    {
         /// wait until PVU bit (0x01) is reset before changing the reload value
        if( ENABLE_WATCHDOG ) {
            while( IWDG->SR & IWDG_FLAG_PVU ) { ///< 0x01 PVU (prescaler value update)
                asm("nop");
            }
        }
        /// Enable write access to IWDG_PR and IWDG_RLR registers 0x5555
        IWDG->KR = IWDG_WriteAccess_Enable;

        /// Set the prescaler to IWDG_Prescaler_256: 0x06
        IWDG->PR = NOMINAL_WATCHDOG_PRESCALER;  ///< With a prescaler, the minimum timeout is 20 millisec

        /// Set the IWDG Reload value 0x05
        IWDG->RLR = NOMINAL_WATCHDOG_DELAY;

        IWDG->KR = IWDG_WriteAccess_Disable;  // disable write
        /// Start IWDG (write access to IWDG_PR and IWDG_RLR registers disabled).
        IWDG->KR = KR_KEY_ENABLE; ///< 0xCCCC;
    }
}

/** ****************************************************************************
 * @name SetMaxDelay_Watchdog
 * @brief Set the watchdog value to its maximum. This is to avoid a time-out when
 *  writing/reading to/from the EEPROM. For PSC = 64, the timeout is 8.192 seconds
 * Trace:
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void SetMaxDelay_Watchdog( void )
{
    /// wait until RVU bit (0x02) is reset before changing the reload value
    if( ENABLE_WATCHDOG ) {
        while( IWDG->SR & IWDG_FLAG_RVU ) { // RVU (reload value update)
            asm("nop");
        }
    }

    IWDG->KR = IWDG_WriteAccess_Enable;  ///< enable write 0x5555
    IWDG->RLR = 0xFFF;  ///< reload value to its maximum value
    IWDG->KR = IWDG_WriteAccess_Disable;  ///< 0x0000 disable write
    IWDG->KR = KR_KEY_RELOAD; ///< 0xAAAA reset timer (reload RLR value in decrementor)
}

/** ****************************************************************************
 * @name RestoreDelay_Watchdog
 * @brief Restore the watchdog reload value to its original value.
 * Trace:
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void RestoreDelay_Watchdog( void )
{
    /// wait until RVU bit is reset before changing the reload value
    if( ENABLE_WATCHDOG ) {
        while( IWDG->SR & IWDG_FLAG_RVU ) {
            asm("nop");
        }
    }

    IWDG->KR = IWDG_WriteAccess_Enable;  /// enable write
    IWDG->RLR = NOMINAL_WATCHDOG_DELAY;  /// reload value to its original value
    IWDG->KR = IWDG_WriteAccess_Disable;  /// disable write
    IWDG->KR = KR_KEY_RELOAD; /// 0xAAAA reset timer
}

