/** ***************************************************************************
 * @file magHMC5883L.c Magnetometer interface for the Honeywell HMC5883L
 * magnetometer
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
 * Fast 160 Hz Maximum Output Rate
 * Built-In Self Test
 * I2C1 should run at 400 kHz
 *
 * The normal method for reading is:
 *  Call MagnetometerStartReading, probably from a timer interrupt
 *  Receive data ready interrupt, this triggers an I2C read
 *  Get I2C read complete flag set in OS (or poll IsMagnetometerDoneReading()
 *  Call MagnetometerGetLastReading to get the data out
 *  Repeat
 *****************************************************************************/
#include <stdint.h>
#include <stdlib.h> // abs()

#include "stm32f2xx_conf.h"
#include "salvodefs.h"
#include "magnetometer.h"
#include "magHMC5883L.h"
#include "i2c.h"
#include "debug.h"
#include "dmu.h"
#include "boardDefinition.h"
#include "xbowsp_algorithm.h"
#include "timer.h"
#include "taskDataAcquisition.h" // LimitInt16Value()

#define I2C_TIMEOUT 100000
static uint8_t gMagnetometerDataBuffer[NUM_AXIS *2]; // readings are 2 bytes per axis

/** ****************************************************************************
 * @name: _MagWriteReg LOCAL blocking write via I2C to magnetometer
 * @param [in] addr - i2c address
 * @param [in] value - data to write
 * @retval 0 if success, error otherwise
 ******************************************************************************/
static uint8_t _MagWriteReg(uint8_t addr,
                            uint8_t value)
{
    uint8_t  data[2];
    uint32_t error = 0;
    uint32_t timeout = I2C_TIMEOUT;

    data[0] = addr;
    data[1] = value;
    i2c_open(kMagnetometerI2C, NULL);
    i2c_data_send(kMagnetometerI2C,
                  HMC5883L_I2C_ADDR,
                  data,
                  sizeof(data));
    while (!i2c_is_done(kMagnetometerI2C) && timeout)
    {timeout--;} // spin
    error = i2c_has_error(kMagnetometerI2C);
    if (!timeout) {
        ERROR_HEX("Mag write Timeout : 0x", error);
        ERROR_ENDLINE();
        error = TRUE;
    }
    i2c_close(kMagnetometerI2C);

    return error;
}

/** ****************************************************************************
 * @name: _MagWriteReg LOCAL blocking read via I2C to magnetometer
 * @param [in] addr - i2c address
 * @param [out] value - data to read
 * @retval 0 if success, error otherwise
 ******************************************************************************/
static uint8_t _MagReadReg(uint8_t addr,
                           uint8_t *value)
{
    uint32_t error = 0;
    uint32_t timeout = I2C_TIMEOUT;

    i2c_open(kMagnetometerI2C, NULL);
    i2c_data_request( kMagnetometerI2C,
                     HMC5883L_I2C_ADDR,
                     addr,
                     value,
                     sizeof(*value));
    while (!i2c_is_done(kMagnetometerI2C) && timeout)
    {timeout--;} // spin 100000
    error = i2c_has_error(kMagnetometerI2C);
    if (error) {
        ERROR_HEX("MAG I2C error ",
                  i2c_has_error(kMagnetometerI2C));
        ERROR_ENDLINE();
    }
    i2c_close(kMagnetometerI2C);
    return error;
}

/** ****************************************************************************
 * @name: _MagnetometerDataReceivedCallback LOCAL
 * @brief Called from the I2C interrupt when the data has been obtained, this
 *        is the last step in getting a sample
 * @param N/A
 * @retval N/A
 ******************************************************************************/
static void _MagnetometerDataReceivedCallback(void)
{
    OSSetEFlag(EFLAGS_DATA_READY, EF_DATA_MAG_READY);
    i2c_close(kMagnetometerI2C);
}

/** ****************************************************************************
 * @name _MagnetometerRequestCompleteCallback LOCAL
 * @brief Called from the I2C interrupt when the data has been obtained, this is
 * the last step in getting a sample
 * @param N/A
 * @retval N/A
 ******************************************************************************/
static void _MagnetometerRequestCompleteCallback(void)
{
    i2c_close(kMagnetometerI2C);
}

/** ****************************************************************************
 * @name MagnetomterDataReadyIRQ pin 5 interrupt asserted
 * @brief This interrupt indicates the data is ready on the magenetomter. Kicks
 *        off reading data.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void MagnetomterDataReadyIRQ(void)
{
    int opened;

    opened = i2c_open(kMagnetometerI2C,
                      _MagnetometerDataReceivedCallback);
    if (opened) {
        i2c_data_request( kMagnetometerI2C,
                          HMC5883L_I2C_ADDR,
                          HMC5883L_X_MSB,
                          gMagnetometerDataBuffer,
                          sizeof(gMagnetometerDataBuffer));
    } else {
        /// fake that the data is ready again, the same data will
        /// be used but this reading will start again as soon as possible
        OSSetEFlag(EFLAGS_DATA_READY, EF_DATA_MAG_READY);
    }
}

/** ****************************************************************************
 * @name MagnetometerStartReading
 * @brief This kicks off conversion of magnetometers
 * @param N/A
 * @retval always 0
 ******************************************************************************/
uint8_t MagnetometerStartReading()
{
    static uint8_t data[2];
    int            opened;

    /// Perform a single measurement by selecting single-measurement mode in
    /// "Mode Register".  After the read is complete, DRDY is set high and the
    /// device returns to idle-mode.
    data[0] = HMC5883L_MODE;
    data[1] = HMC5883L_MODE_READ_SINGLE;

    opened = i2c_open(kMagnetometerI2C,
                      _MagnetometerRequestCompleteCallback);
    if (opened) {
        i2c_data_send(kMagnetometerI2C,
                      HMC5883L_I2C_ADDR,
                      data,
                      sizeof(data));
    } else {
        /// fake that the data is ready again, the same data will
        /// be used but this reading will start again as soon as possible
        OSSetEFlag(EFLAGS_DATA_READY, EF_DATA_MAG_READY);
    }
    OSClrEFlag(EFLAGS_DATA_READY, EF_DATA_MAG_READY);

    return 0;
}

/** ****************************************************************************
 * @name IsMagnetometerDoneReading
 * @brief Checks to see if magnetometer transaction is complete
 * @param N/A
 * @retval 0 if success, error otherwise
 ******************************************************************************/
uint8_t IsMagnetometerDoneReading()
{
    OStypeEFlag eFlag = OSReadEFlag(EFLAGS_DATA_READY);
    return eFlag & EF_DATA_MAG_READY;
}

/** ****************************************************************************
 * @name MagnetometerGetLastReading
 * @brief Gets the data that was last read by I2C process saturating and
 * transforming to the DMU380 coordinate system
 *
 * @param array of reading (NUM_AXIS)
 * @retval 0 if success, error otherwise
 * @brief The tempX/Y/Z have to be there for the data to come out right in
 * readings[]
 ******************************************************************************/
uint8_t MagnetometerGetLastReading(int16_t *readings)
{
    uint8_t  *data = gMagnetometerDataBuffer;
    uint32_t error;
    int16_t tempX, tempY, tempZ;

    error = i2c_has_error(kMagnetometerI2C);
    if (!error && readings) {
        /// Limit the rate-sensor reading to prevent the transformation from
        /// causing the signal to saturate with an opposite sign
        /// Transform the data into the DMU380 frame
        tempX = LimitInt16Value( (data[0] << 8) | data[1], INT16_LIMIT );
        tempY = LimitInt16Value( (data[2] << 8) | data[3], INT16_LIMIT );
        tempZ = LimitInt16Value( (data[4] << 8) | data[5], INT16_LIMIT );

        // Transform the data into the DMU380 frame
        readings[0] =  tempX;
        readings[1] = -tempZ;
        readings[2] = -tempY;
    } else {
        DEBUG_INT("Mag I2C error ", error);
        DEBUG_ENDLINE();
    }
    return error;
}

/** ****************************************************************************
 * @name _MagWhoAmINoCheck
 * @brief reset the I2C magnetometer state machine by calling whoami with no
 *        status check
 * @param [out] whoami - device ID = ('H' '4' '3') 0 on fail
 * @retval 1 true if success, error 0 - false
 ******************************************************************************/
uint8_t _MagWhoAmINocheck( void )
{
    uint8_t  data[3];
    uint32_t timeout = I2C_TIMEOUT;

    i2c_open(kMagnetometerI2C, NULL);
    i2c_data_request( kMagnetometerI2C,
                      HMC5883L_I2C_ADDR, // 0x3c
                      HMC5883L_ID_A, // 10
                      data,
                      sizeof(data));
    while (!i2c_is_done(kMagnetometerI2C) && timeout)
    {timeout--;} // spin
    if (timeout == 0) {
//        ERROR_STRING("_MagWhoAmINocheck() timeout fail\r\n");
    }
    i2c_close(kMagnetometerI2C);
    return TRUE;
}

/** ****************************************************************************
 * @name MagnetometerWhoAmI
 * @brief Verify magnetometer attached is one we know how to talk to
 *
 * @param [out] whoami - device ID = ('H' '4' '3') 0 on fail
 * @retval 1 true if success, error 0 - false
 ******************************************************************************/
uint8_t MagnetometerWhoAmI(uint32_t *whoami)
{
    uint8_t  data[3];
    uint32_t error = 0;
    uint32_t timeout = I2C_TIMEOUT;

    i2c_open(kMagnetometerI2C, NULL);
    i2c_data_request( kMagnetometerI2C,
                      HMC5883L_I2C_ADDR, // 0x3c
                      HMC5883L_ID_A, // 10
                      data,
                      sizeof(data));
    while (!i2c_is_done(kMagnetometerI2C) && timeout)
    {timeout--;} // spin
    error = i2c_has_error(kMagnetometerI2C);
    if (error == true) {
        ERROR_HEX("MagnetometerWhoAmI() I2C error: ", error);
        ERROR_ENDLINE();
        gAlgorithm.bitStatus.comBIT.bit.i2CError = 1;
    }
    if (timeout == 0) {
        error |= TRUE;
        ERROR_STRING("MagnetometerWhoAmI() timeout fail\r\n");
    }
    i2c_close(kMagnetometerI2C);

    if (error == false) { // return ID - should be 'H' '4' '3'
        *whoami  = data[0] << 16;
        *whoami |= data[1] << 8;
        *whoami |= data[2];
    }
    if (error || // not as expected
        data[0] != HMC5883L_ID_A_EXPECTED || // !'H' 0x48
        data[1] != HMC5883L_ID_B_EXPECTED || // !'4' 0x34
        data[2] != HMC5883L_ID_C_EXPECTED) { // !'3' 0x33
        *whoami = 0;
        return FALSE;
    }
    return TRUE;
}

/** ****************************************************************************
 * @name MagnetometerGetGain
 * @brief Get gain of magnetometer in LSB/gauss
 *
 * @param N/A
 * @retval 0 if success, error otherwise
 ******************************************************************************/
uint16_t MagnetometerGetGain()
{   // for HMC5883L_CONFIGURATION_B which sets the gain
    const uint16_t HMC5883L_GAIN_SETTINGS[] = {
        1370, // 000 +/- 0.88 Ga
        1090, // 001 +/- 1.3  Ga, default
        820,  // 010 +/- 1.9  Ga
        660,  // 011 +/- 2.5  Ga
        440,  // 100 +/- 4.0  Ga
        390,  // 101 +/- 4.7  Ga
        330,  // 110 +/- 5.6  Ga
        230,  // 111 +/- 8.1  Ga
    };
    uint8_t data = 0;
    uint8_t gain = 0;
    _MagReadReg(HMC5883L_CONFIGURATION_B, &data);
    gain = data >> 5;

    return (HMC5883L_GAIN_SETTINGS[gain]);
}

/** ****************************************************************************
 * @name _MagnetometerClearBuffer LOCAL
 * @brief Clear the magnetometer's buffer, only used in swtest
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
static void _MagnetometerClearBuffer()
{
    int16_t numReads = 6;
    int16_t readings[NUM_AXIS];

    INFO_ENDLINE();
    while (numReads) {
        MagnetometerStartReading();
        while (!IsMagnetometerDoneReading())
        { /* spin */ ; }
        MagnetometerGetLastReading(readings);
        numReads--;
    }
}

/*******************************************************************************
 * @name MagnetometerSelfTest
 * @brief Run the magnetometer self-test. This only prints results, it does not
 *        assess if the test was successful.
 *
 * @param N/A
 * @retval TRUE if magnetomter's existence if verified, FALSE if there was
 *         an error talking to the mag (or the mag's whoami is wrong)
 ******************************************************************************/
uint8_t  MagnetometerSelfTest()
{
    int16_t       initialReading[NUM_AXIS];
    int16_t       biasedReading[NUM_AXIS];
    uint8_t       originalGainSetting;
    uint8_t       config;
    uint16_t      gain;
    const uint8_t numSigDigits = 2;
    int           i;
    float         gauss;

    /// reset gain to something that won't be overwhelmed with 1.1 induced gauss
    /// but we'll need to store the original value for later
    _MagReadReg(HMC5883L_CONFIGURATION_B,
                &originalGainSetting); // 7 so 230 is original gain

    config = HMC5883L_B_GAIN_4_0_GA << HMC5883L_GAIN_SHIFT;
    _MagWriteReg(HMC5883L_CONFIGURATION_B,
                 config); // lower gain to 4.0 gauss
    gain = MagnetometerGetGain();

    /// go into test mode
    _MagnetometerClearBuffer(); // clear the I2C buffer
    _MagWriteReg(HMC5883L_CONFIGURATION_A,
                 HMC5883L_A_TEST_POSITIVE);
    /// in test mode, need two data acquisitions in single sample mode
    MagnetometerStartReading();
    while (!IsMagnetometerDoneReading())
    { ; }
    MagnetometerGetLastReading(initialReading);

    DEBUG_ENDLINE();
    DEBUG_STRING("\tPOS:");
    for ( i = 0; i < NUM_AXIS; i++ ) {
        gauss = (float) initialReading[i] / gain;
        DEBUG_FLOAT(" ", gauss, numSigDigits);
            //ex  1.25,  -1.15,  -1.10
        //   if ( (gauss - 1.1f) >  ) {
                //gAlgorithm.bitStatus.hwBIT.bit.sensorError   = 1;
                //gAlgorithm.bitStatus.hwBIT.bit.MagnetometerError = 1;
        //}
    }
    DEBUG_STRING(" (ex  1.16,  1.16,  1.08)\r\n");

    _MagWriteReg(HMC5883L_CONFIGURATION_A,
                 HMC5883L_A_TEST_NEGATIVE);
    MagnetometerStartReading();
    while (!IsMagnetometerDoneReading())
    { ; } // 6.26 ms to complete
    MagnetometerGetLastReading(biasedReading);

    DEBUG_STRING("\tNEG:");
    for ( i = 0; i < NUM_AXIS; i++ ) {
        DEBUG_FLOAT(" ", (float) biasedReading[i] / gain, numSigDigits);
        //ex  -1.21,  1.16,  1.07
        //   if ( (gauss - 1.1f) >  ) {
        //gAlgorithm.bitStatus.hwBIT.bit.sensorError   = 1;
        //gAlgorithm.bitStatus.hwBIT.bit.MagnetometerError = 1;
        // }
    }
    DEBUG_STRING(" (ex -1.16, -1.16, -1.08)\r\n\t");

    /// return things to normal
    i2c_open(kMagnetometerI2C, NULL);
    _MagWriteReg(HMC5883L_CONFIGURATION_A,
                 HMC5883L_A_TEST_NORMAL);
    _MagWriteReg(HMC5883L_CONFIGURATION_B,
                 originalGainSetting); // put gain back
    _MagnetometerClearBuffer();

    return TRUE;
}

/** ****************************************************************************
 * @name MagnetometerConfig Set up gain and range of the magnetometer
 *
 * @param [in] range in milligauss,
 * @retval always return true - success forces system to wait until fcn returns
 ******************************************************************************/
uint8_t MagnetometerConfig(uint32_t *rangeInMilliGauss)
{
    uint8_t config;

    /// Range is +/- the value selected
    switch (*rangeInMilliGauss) {
        case 8100:
            config = HMC5883L_B_GAIN_8_1_GA;
            break;
        case 5600:
            config = HMC5883L_B_GAIN_5_6_GA;
            break;
        case 4700:
            config = HMC5883L_B_GAIN_4_7_GA;
            break;
        case 4000:
            // (-200/-400 setting: this is +/- 4.0 G)
            config = HMC5883L_B_GAIN_4_0_GA;
            break;
        case 2500:
            config = HMC5883L_B_GAIN_2_5_GA;
            break;
        case 1900:
            config = HMC5883L_B_GAIN_1_9_GA;
            break;
        case 1300:
            config = HMC5883L_B_GAIN_1_3_GA;
            break;
        case  880:
        default:
            config = HMC5883L_B_GAIN_0_88_GA;
            *rangeInMilliGauss = 880;
    }
    /// Set unit Configuration B to 0xE0 (1110 0000):
    ///   - Device gain: +/- 8.1G
    config <<= HMC5883L_GAIN_SHIFT;
    _MagWriteReg(HMC5883L_CONFIGURATION_B, config);

    return TRUE;
}

/*******************************************************************************
 * @name _InitMagDataReadyInt
 * @brief Set up data ready GPIO B5 and make it an interrupt
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void _InitMagDataReadyInt(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(MAG_DATA_READY_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); // for interrupts

    /// Configure data ready B5 pin as input
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin  = MAG_DATA_READY_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(MAG_DATA_READY_GPIO_PORT, &GPIO_InitStructure);

    /// Configure EXTI line
    EXTI_StructInit(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line    = MAG_DATA_READY_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /// Connect EXTI Line to GPIO Pin
    SYSCFG_EXTILineConfig(MAG_DATA_READY_EXTI_PORT_SOURCE, MAG_DATA_READY_EXTI_PIN_SOURCE);

    /// Enable and set EXTI Interrupt to the lowest priority
    NVIC_InitStructure.NVIC_IRQChannel                   = MAG_DATA_READY_EXTI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x8;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x2;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
 * @name InitMagnetometer
 * @brief  Initialize magnetometer, no assumptions made about rest of
 *  system's initialization. This will call i2c_configure and set up the
 *  data ready line up as an interrupt. Finally, it will put the magnetometer
 *  in the normal running mode. uses I2C1
 *
 * HMC5883L_CONFIGURATION_A default is
 * MS0:1 0 bias
 * D00:2 15Hz output in continuous mode
 * MA0:0 1 sample ave
 * leave that for now, especially since we won't use continuous mode
 *
 * HMC5883L_CONFIGURATION_B default is
 *      GN0:2 gain is set 001, +/1 1.3Ga, 1090 lsb/gauss
 * see MagnetometerGetGain for more info

 * HMC5883L_MODE defaults to single measurement mode *
 * @param N/A
 * @retval FALSE if error
 * @version DKH 10.03.14 moved _InitAccelDataReadyInt() to after I2C init,
 * set the flag to enable and added a dealu after !2C init
 ******************************************************************************/
uint8_t InitMagnetometer()
{
    uint8_t  status;

    i2c_configure(kMagnetometerI2C);
    _InitMagDataReadyInt();
    DelayMs(10);
    _MagWhoAmINocheck(); // reset the I2C state machine
    DelayMs(1);
    _MagReadReg(HMC5883L_STATUS, &status); // 9 error handled in _MagReadReg() call

    INFO_HEX("Mag status: ", status);
    INFO_ENDLINE();

    _MagWriteReg(HMC5883L_CONFIGURATION_A, // 0
                 HMC5883L_A_TEST_NORMAL); // 0x10;
    return MagnetometerWhoAmI(NULL);
}
