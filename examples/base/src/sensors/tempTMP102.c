/** ***************************************************************************
 * @file tempTMP102.cTemperature sensor interface for the TI TMP102 temperature
 * sensor
 * @author
 * @date   September, 2008
 * @copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 * Some features of interest for implementation:
 * I2C1 should run at 400 kHz
 * http://www.ti.com/lit/gpn/tmp102
 * The conversion rate bits, TMP102 for conversion rates of 8Hz, 4Hz, 1Hz, or
 * 0.25Hz. The default rate is 4Hz. The TMP102 has a typical conversion time
 * of 26ms.
 *****************************************************************************/
#include <stdint.h>

#include "stm32f2xx_conf.h"
#include "salvodefs.h"
#include "temperatureSensor.h"
#include "tempTMP102.h"

#include "i2c.h"
// #define LOGGING_LEVEL LEVEL_INFO
#include "debug.h"
#include "dmu.h"
#include "boardDefinition.h"
#include "timer.h"

#define I2C_TIMEOUT 10000

#define TEMP_SENSOR_CONV_TIME                  500 /// ms (2Hz) --  26 msec is data sheet conv time
#define TEMP_SENSOR_READ_RETRY_TIME             25 /// ms
#define MS_PER_SECOND                         1000
#define TEMPERATURE_TIMER_PRESCALER           2046
#define TEMPERATURE_PRESCALER_PERIOD_SHIFT      12 /// 11 for prescaler + 1 for all clocks div by 1

static uint8_t  gTemperatureData[2];
static uint32_t gTemperatureConversionPeriod;
static uint32_t gTemperatureRetryConversionPeriod;

/** ****************************************************************************
 * @name _StartTemperatureTimer LOCAL sets up timer3
 * @param [in] period - the frame time to set
 * @retval N/A
 ******************************************************************************/
static void _StartTemperatureTimer(uint32_t period)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    /// Time base configuration
    TIM_ITConfig(TIM3, TIM_IT_Update , DISABLE);
    TIM_Cmd(TIM3, DISABLE);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler   = TEMPERATURE_TIMER_PRESCALER;
    TIM_TimeBaseStructure.TIM_Period      = period;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_ARRPreloadConfig(TIM5, ENABLE);
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    TIM_Cmd(TIM3, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update , ENABLE);
}

/** ****************************************************************************
 * @name _InitTemperatureConversionTimer LOCAL sets up timer3 interupt
 * @param N/A
 * @retval N/A
 ******************************************************************************/
static void _InitTemperatureConversionTimer(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_Cmd(TIM3, DISABLE); // configured as needed

    gTemperatureConversionPeriod = SystemCoreClock >> TEMPERATURE_PRESCALER_PERIOD_SHIFT;
    gTemperatureConversionPeriod /= MS_PER_SECOND;

    gTemperatureRetryConversionPeriod = gTemperatureConversionPeriod;

    gTemperatureConversionPeriod      *= (TEMP_SENSOR_CONV_TIME);
    gTemperatureRetryConversionPeriod *= (TEMP_SENSOR_READ_RETRY_TIME);

    /// Enable the TIM3 gloabal Interrupt
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/** ****************************************************************************
 * @nameInitTemperatureSensor set up the sensor over the I2C
 * @param N/A
 * @retval status
 ******************************************************************************/
uint8_t InitTemperatureSensor()
{
    uint8_t config[2];
    uint8_t buffer[3];
    uint32_t error;
    uint32_t timeout = I2C_TIMEOUT;
DEBUG_STRING("inside InitTemperature.\r\n");

    i2c_configure(kTemperatureSensorI2C);
DEBUG_STRING("past i2c_configure.\r\n");

    /// this is a dummy read
    i2c_open(kTemperatureSensorI2C, NULL);
DEBUG_STRING("past i2c_open.\r\n");
    i2c_data_request( kTemperatureSensorI2C,
                     TMP102_I2C_ADDR,
                     TMP102_CONFIG_REG,
                     config,
                     sizeof(config));
DEBUG_STRING("past i2c_data_request.\r\n");
    while (!i2c_is_done(kTemperatureSensorI2C) && timeout)
    {timeout--;} // spin
DEBUG_STRING("First dummy read.\r\n");

    /// set up the system to be in 'shutdown' mode and have nominal resolution values
    config[0] = TMP102_CONFIG_SHUTDOWN;
    config[0] |= TMP102_CONFIG_RESOLUTION << TMP102_CONFIG_RES_SHIFT;
    config[1] = TMP102_CONFIG1_DEFAULT;

    /// populate the buffer that is sent to the TMP102
    buffer[0] = TMP102_CONFIG_REG;
    buffer[1] = config[0];
    buffer[2] = config[1];

    /// Send the buffer and wait; check for an error
    i2c_data_send(kTemperatureSensorI2C,
                  TMP102_I2C_ADDR,
                  buffer,
                  sizeof(buffer));
    timeout = I2C_TIMEOUT;
    while (!i2c_is_done(kTemperatureSensorI2C) && timeout)
    {timeout--;} // spin
    error = i2c_has_error(kTemperatureSensorI2C);
    if (timeout > 0 && !error) {
        INFO_STRING("Wrote config to temp sensor.\r\n");
    } else {
        ERROR_INT("Timeout writing to temperature sensor config ", error);
        ERROR_ENDLINE();
    }

    /// read the configuration register
    i2c_data_request( kTemperatureSensorI2C,
                      TMP102_I2C_ADDR,
                      TMP102_CONFIG_REG,
                      config,
                      sizeof(config));
    timeout = I2C_TIMEOUT;
    while (!i2c_is_done(kTemperatureSensorI2C) && timeout)
    {timeout--;} // spin
    error = i2c_has_error(kTemperatureSensorI2C);
    if (timeout > 0) {
        INFO_HEX("Temp config reg re-read ", config[0]);
        INFO_HEX(" ", config[1]);
        INFO_ENDLINE();
    } else {
        ERROR_INT("Timeout reading to temp sensor config (2) ", error);
        ERROR_ENDLINE();
    }
    i2c_close(kTemperatureSensorI2C);

    /// set up and enable the TIM3 interrupt
    _InitTemperatureConversionTimer();
    if (!timeout) {
      return FALSE;
    }
    return TRUE;
}

/** ****************************************************************************
 * @name _TemperatureRegWriteDoneCallback LOCAL
 * @brief Called from the I2C interrupt
 * @param N/A
 * @retval N/A
 ******************************************************************************/
static void _TemperatureRegWriteDoneCallback(void)
{
    i2c_close(kTemperatureSensorI2C);
    /// temp sensor has started conversion, need to kick off timer to interrupt
    /// and tell us to start reading
    _StartTemperatureTimer(gTemperatureConversionPeriod);
    /// wait for timer to read the results
}

/** ****************************************************************************
 * @name TemperatureStartReading
 * @brief This kicks off reading by telling the sensor to start a
 *   conversion. When the command has been sent, I2C will call the
 *   _TemperatureRegWriteDoneCallback function which only closes I2C.
 *   That function also sets up a timer for a bit longer than the conversion
 *   time. That will call _TemperatureTimer when it is comlete which does an
 *   I2C read of the data (and when that is complete, it will call
 *   _TemperatureReadingComplete which will signal the OS that temp
 *   sensing is complete.
 * @param none
 * @retval FALSE if error
 ******************************************************************************/
uint8_t TemperatureStartReading()
{
    static uint8_t buffer[3];
    int            opened;

    /// When the device is in Shutdown Mode,
    /// writing a 1 to the OS/ALERT bit will start
    /// a single temperature conversion. The device will return to
    /// the shutdown state at the completion of the single conversion
    opened = i2c_open(kTemperatureSensorI2C, _TemperatureRegWriteDoneCallback);
    if (opened) {
        buffer[0] = TMP102_CONFIG_REG;
        buffer[1] = TMP102_CONFIG_START_CONV;
        buffer[2] = TMP102_CONFIG1_DEFAULT;
        i2c_data_send(kTemperatureSensorI2C,
                      TMP102_I2C_ADDR,
                      buffer,
                      sizeof(buffer));
    } else {
        /// fake that the data is ready again, the same data will
        /// be used but this reading will start again as soon as possible
        OSSetEFlag(EFLAGS_DATA_READY, EF_DATA_TEMP_READY);
    }
    return TRUE;
}

/** ****************************************************************************
 * @name IsTemperatureDoneReading
 * @brief read the DR and return the value
 * @param N/A
 * @retval DR flag value
 ******************************************************************************/
uint8_t IsTemperatureDoneReading(void)
{
    OStypeEFlag eFlag = OSReadEFlag(EFLAGS_DATA_READY);
    return eFlag & EF_DATA_TEMP_READY; // 0x08
}

/** ****************************************************************************
 * @name TemperatureGetLastReading
 * @brief read the DR and return the value
 * @param [out] t - pointer to outpt buffer
 * @retval status
 ******************************************************************************/
uint8_t TemperatureGetLastReading(int16_t *t)
{
    uint16_t temperature;
    uint32_t error;

    error = i2c_has_error(kTemperatureSensorI2C);
    if (!error) {
        OSDisableHook();
        /// @brief  The temperature sensor (TMP102) has 12 data-bits contained
        /// in two bytes.  The last four bits of the two (4 LSBs) contain no
        /// data - only 0.  (Can also be configured in 13-bit  mode.)
        temperature = gTemperatureData[0] << 8 | gTemperatureData[1];
        OSEnableHook();
        if (t) {
          *t = temperature;
        }
        return TRUE;
    } else { /// error
      if (t) {
        *t = 0;
      }
        return FALSE;
    }
}

// return LSB/Gauss
/** ****************************************************************************
 * @name TemperatureGetGain
 * @brief
 * @param N/A
 * @retval return fixed gain
 ******************************************************************************/
uint16_t TemperatureGetGain()
{
    /*******************************
    ** TEMPERATURE      DIGITAL OUTPUT
    **    (°C)        (BINARY)          HEX
    **   128        0111 1111 1111      7FF
    **   127.9375   0111 1111 1111      7FF
    **   100        0110 0100 0000      640
    **   80         0101 0000 0000      500
    **   75         0100 1011 0000      4B0
    **   50         0011 0010 0000      320
    **   25         0001 1001 0000      190
    **   0.25       0000 0000 0100      004
    **   0.0        0000 0000 0000      000
    **   -0.25      1111 1111 1100      FFC
    **   -25        1110 0111 0000      E70
    **   -55        1100 1001 0000      C90
    **   -128       1000 0000 0000      800
    *******************************/
   return 16 << 4; // shift is because ADC is returning 12 bits
                    // but it is in top bits of a uint16_t
}

/** ****************************************************************************
 * @name TemperatureSelfTest
 * @brief stubbed - no self test
 * @param N/A
 * @retval pass
 ******************************************************************************/
uint8_t  TemperatureSelfTest()
{
    return TRUE;
}
