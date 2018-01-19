/** ***************************************************************************
 * @file accelMMA8451Q.c Accelerometer interface for the Freescale MMA8451Q
 *       accelerometer
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * @brief Some features of interest for implementation:
 *  * Fast 160 Hz Maximum Output Rate
 *  * Built-In Self Test
 *  * I2C3 should run at 400 kHz
 *
 * The normal method for reading is:
 *  * Call AccelerometerStartReading, probably from a timer interrupt
 *  * Receive data ready interrupt, this triggers an I2C read
 *  * Get I2C read complete flag set in OS (or poll IsAccelerometerDoneReading()
 *  * Call AccelerometerGetLastReading to get the data out
 * Repeat
 *****************************************************************************/

#include <stdint.h>

#include "stm32f2xx_conf.h"
#include "salvodefs.h"
#include "dmu.h"
#include "accelerometer.h"
#include "accelMMA8451Q.h"
#include "boardDefinition.h"
#include "i2c.h"
#include "xbowsp_algorithm.h"

#include "gyroscope.h"    // for BeginRateSensorRead

//#define LOGGING_LEVEL LEVEL_INFO
#include "debug.h"
#include "configuration.h"
#include "ucb_packet.h"
#include "taskDataAcquisition.h"
#include "UserCommunication_SPI.h"
#include "filter.h" // debounce()
#include "timer.h"
#include "Indices.h"
#include "qmath.h"

#include "lowpass_filter.h"

#define I2C_TIMEOUT 10000
#define ACCEL_MAX_CACHED_NUM               16

typedef enum {
    eAccelOff                = 0,
    eAccelI2CError           = 1,
    eAccelStartGatheringData = 2,
    eAccelAccumulatingData   = 3,
} eAccelState;

typedef enum {
  ACCEL_BUFFER_IDLE = 0,
  ACCEL_BUFFER_AVAILABlE = 1,
  ACCEL_BUFFER_FULL = 2,
  ACCEL_BUFFER_OVERFLOW = 3
} ACCEL_BUFFER_STATUS;

static struct {
    uint8_t              dataBuffer[ACCEL_MAX_CACHED_NUM][NUM_AXIS*2 + 1 ]; // readings are 2 bytes per axis + 1 for status
    volatile int16_t     accumulationBuffer[ACCEL_MAX_CACHED_NUM][NUM_AXIS];
    volatile int32_t     accumulationCount;
    volatile eAccelState state;
    volatile uint8_t     i2cBusy;
    ACCEL_BUFFER_STATUS  status;
} gAccel; // FIXME: change name this is local

debounce       i2cDebounce;
debounceBuffer i2cDebounceBuffer;
uint8_t        i2cFails[100]; // debounce buffer for reads

uint32_t accel_success = 1;
uint32_t accel_fail = 0;
uint8_t accel_range = 0;

void _InitAccelDataReadyInt(FunctionalState enable);

/** ****************************************************************************
 * @name: _AccWhoamiNoCheck LOCAL non blocking read via I2C to accelerometer
 * called first time to reset the I2c state machine with no check of status
 * @param N/A
 * @retval always true
 ******************************************************************************/
uint8_t _AccWhoamiNoCheck( void )
{
    uint8_t  rx      = 0xFF;
    uint32_t timeout = I2C_TIMEOUT; // 10000

    i2c_data_request(kAccelerometerI2C,
                     MMA8451_I2C_ADDR, // 0x38
                     MMA8451_WHO_AM_I,
                     &rx, sizeof(rx));
    while (!i2c_is_done(kAccelerometerI2C) && timeout)
    {timeout--;}
    if (timeout==0){
        ERROR_STRING("_AccWhoamiNoCheck() i2c_data_request() timeout.\r\n");
    }
    i2c_close( kAccelerometerI2C );
    return true;
}

/** ****************************************************************************
 * @name: _AccReadReg LOCAL blocking read via I2C to accelerometer
 * @param [in] address
 * @retval acceleration read value
 ******************************************************************************/
static uint8_t _AccReadReg(uint8_t address)
{
    uint8_t  rx      = 0xFF;
    uint32_t timeout = I2C_TIMEOUT; // 10000

    i2c_open(kAccelerometerI2C, NULL);
    i2c_data_request(kAccelerometerI2C,
                     MMA8451_I2C_ADDR, // 0x38
                     address,
                     &rx, sizeof(rx));
    while (!i2c_is_done(kAccelerometerI2C) && timeout)
    {timeout--;}
    if (timeout==0) {
        ERROR_STRING("Accel read reg timeout.\r\n");
    }

    i2c_close( kAccelerometerI2C );
    return rx;
}

/** ****************************************************************************
 * @name: _AccWriteReg LOCAL blocking write via I2C to accelerometer
 * @param [in] address - address to send data to
 * @param [in] value - data to send out
 * @param [in] waitForDone - value for done flag
 * @retval status
 ******************************************************************************/
static int _AccWriteReg(uint8_t address,
                        uint8_t data,
                        int     waitForDone)
{
    static uint8_t buffer[2];
    uint32_t       error;
    uint32_t       timeout = I2C_TIMEOUT; // 10,000

    buffer[0] = address;
    buffer[1] = data;

    error = i2c_open(kAccelerometerI2C, NULL);
    if (!error) {
        ERROR_HEX("I2C OPEN error on accel write reg: 0x", address);
        ERROR_HEX(" Data = 0x", data);
        ERROR_ENDLINE();
    }
    i2c_data_send(kAccelerometerI2C,
                  MMA8451_I2C_ADDR,
                  buffer,
                  sizeof(buffer));
    while (waitForDone && !i2c_is_done(kAccelerometerI2C) && timeout)
    {timeout--;} // spin
    error = i2c_has_error(kAccelerometerI2C);
    if (timeout == 0){
        ERROR_INT("I2C Timeout on accel write reg: ", address);
        ERROR_ENDLINE();
    } if (error) {
        ERROR_HEX("I2C error on accel write reg: 0x", address);
        ERROR_ENDLINE();
    } if (!timeout)
        error = TRUE;
    i2c_close(kAccelerometerI2C);
    return !error;
}

/** ****************************************************************************
 * @name: _SetStandby LOCAL Sets the MMA845x to standby mode.
 * @brief It must be in standby to change most register settings
 * @param N/A
 * @retval N/A
 ******************************************************************************/
static void _SetStandby()
{
    uint8_t reg;

    reg = _AccReadReg(MMA8451_CTRL_REG1);
    reg &= ~(MMA8451_CTRL_REG1_ACTIVE);
    _AccWriteReg(MMA8451_CTRL_REG1, reg, TRUE);
}

/** ****************************************************************************
 * @name: _SetActive LOCAL Sets the MMA845x to active mode.
 * @brief It must be in active to read sensors
 * @param N/A
 * @retval N/A
 ******************************************************************************/
static void _SetActive()
{
    uint8_t reg;

    reg = _AccReadReg(MMA8451_CTRL_REG1);
    reg |= (MMA8451_CTRL_REG1_ACTIVE);
    _AccWriteReg(MMA8451_CTRL_REG1, reg, TRUE);
}


/** ****************************************************************************
 * @name: _AccelerometerDataReceivedCallback LOCAL
 * @brief Called from the I2C interrupt when the data has been
 *   obtained.  Data is placed into a buffer and filtered when
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void _AccelerometerDataReceivedCallback(void)
{
    uint8_t  index;
    uint8_t  *data;
    uint8_t  status;
    int16_t  val; /// need int16_t to make positives and negatives work out

    uint32_t error = i2c_has_error(kAccelerometerI2C);

    if (error) {
        gAccel.state = eAccelI2CError;
        OSSetEFlag(EFLAGS_DATA_READY, EF_DATA_ACCEL_READY);
        ERROR_INT("Accel I2C callback  error: ", error);
        ERROR_ENDLINE();
    }

    if (gAccel.accumulationCount > 8) {  // FIXME: should this be 16?
        gAccel.accumulationCount = 0;
    }

    if (gAccel.state != eAccelOff && gAccel.state != eAccelI2CError) {
        data = &gAccel.dataBuffer[gAccel.accumulationCount][1];
        status = gAccel.dataBuffer[gAccel.accumulationCount][0];

        /// Place the accelerometer readings into a buffer for use by the
        ///   digital filter (prior to use in taskDataAcquisition)
        if ((status & MMA8451_STATUS_READY) && ! (status & MMA8451_STATUS_ERROR)) {
            /// X-Axis
            index = 2 * X_AXIS;
            val   = ( data[ index ] << 8 )  | data[ index + 1 ];
            gAccel.accumulationBuffer[gAccel.accumulationCount][X_AXIS] = val;

            /// Y-Axis
            index = 2 * Y_AXIS;
            val   = ( data[ index ] << 8 )  | data[ index + 1 ];
            gAccel.accumulationBuffer[gAccel.accumulationCount][Y_AXIS] = val;

            /// Z-Axis
            index = 2 * Z_AXIS;
            val   = ( data[ index ] << 8 )  | data[ index + 1 ];
            gAccel.accumulationBuffer[gAccel.accumulationCount][Z_AXIS] = val;
           

            gAccel.accumulationCount++;
        } else {
            gAccel.accumulationBuffer[0][0] = status;
        }
        OSSetEFlag(EFLAGS_DATA_READY, EF_DATA_ACCEL_READY);
        gAccel.state = eAccelAccumulatingData;
    }// end accel off
    else 
      accel_fail++;

    i2c_close( kAccelerometerI2C );
    setAccelI2CBusy( NOT_BUSY );
}

/** ****************************************************************************
 * @name AccelerometerDataReadyIRQ pin 9 intrrupt aserted
 * @brief This interrupt indicates the data is ready on the accelerometer. Kicks
 *        off reading data.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void AccelerometerDataReadyIRQ(void)
{
    // Set IO3 high when the 800 Hz interrupt occurs (when the units begins 
    //   obtaining sensor dat)
    GPIOB->BSRRL = GPIO_Pin_11;

    uint32_t error = i2c_has_error(kAccelerometerI2C);
    
    if (gAccel.status >= ACCEL_BUFFER_FULL) {
        gAccel.status = ACCEL_BUFFER_OVERFLOW;
        return;
    }

    // No error, let's do this!
    if (error == false) {
        if (i2c_open(kAccelerometerI2C, _AccelerometerDataReceivedCallback)) {
            i2c_data_request(kAccelerometerI2C,
                             MMA8451_I2C_ADDR, // 0x1c << 1 = 0x38
                             MMA8451_STATUS,   // 0x00
                             &(gAccel.dataBuffer[gAccel.accumulationCount][0]),
                             NUM_AXIS*2 + 1 ); // 0x7

            if (gAccel.accumulationCount < (ACCEL_MAX_CACHED_NUM - 1)) {
                gAccel.status = ACCEL_BUFFER_AVAILABlE;
            } else {
                gAccel.status = ACCEL_BUFFER_FULL;
            }

            // Initiate the read of the rate-sensor data buffer (here it allows
            //   the SPI commmunications to work).  Reading the rate-sensor after
            //   the accelerometer read was complete caused the SPI communications
            //   to fail.
            BeginRateSensorRead();
        } else {
            ERROR_STRING("Accel DR IRQ open error.\r\n");
            error = TRUE;
        }
    }

    if (error == true) {
        ERROR_INT("Accel (DR IRQ) I2C error.", error);
        ERROR_ENDLINE();
        gAccel.state = eAccelI2CError;
        OSSetEFlag(EFLAGS_DATA_READY, EF_DATA_ACCEL_READY);
        i2c_close(kAccelerometerI2C);

        debounce_calc(&i2cDebounce, 1); // add to the debouce
        if (i2cDebounce.sum >= i2cDebounce.size) {
            // status = fail for long time report
            gAlgorithm.bitStatus.comBIT.bit.i2CError = 1;
        }
    } else {
        debounce_calc(&i2cDebounce, 0);
    }
}


/** ****************************************************************************
 * @name AccelerometerStartReading
 * Description:  This doesn't kick off a conversion cycle as the sampling
 *   asynchronous to the system. It just makes it so the data ready line
 *   goes (but it goes whenever it was ready).
 * @param N/A
 * @retval 0 if success, error otherwise
 ******************************************************************************/
uint8_t AccelerometerStartReading()
{
    gAccel.state             = eAccelStartGatheringData;
    return 0;
}


/** ****************************************************************************
 * @name IsAccelerometerDoneReading
 * @brief Checks to see if  transaction is complete
 * @param N/A
 * @retval 0 if success, error otherwise
 ******************************************************************************/
uint8_t IsAccelerometerDoneReading()
{
    OStypeEFlag eFlag = OSReadEFlag(EFLAGS_DATA_READY);
    return eFlag & EF_DATA_ACCEL_READY;
}


/** ****************************************************************************
 * @name: AccelerometerGetLastReading
 * @brief Gets the data that has been accumulated, filters it, and returns it to
 *        the calling routine
 * @param [out] readings - array of read data (NUM_AXIS)
 * @retval 0 if success, error otherwise
 ******************************************************************************/
uint8_t AccelerometerGetLastReading(int16_t *readings)
{
    static uint8_t initFlag = 1;

    int32_t count;
    int i;
    int32_t  accelFilteredVal[NUM_AXIS];
    uint8_t cutoff_freq = ACCEL_COF_LPF_50HZ;      // FIXME if it will be used
    static uint8_t input_data_rate;
//    uint32_t range;
//    uint32_t outputDataRate;

    if( initFlag ) {
        initFlag = 0;
        if( UcbGetSysType() <= UNAIDED_AHRS_SYS ) {
            input_data_rate = BWF_LOWPASS_DATA_RATE_800;
        } else {
            input_data_rate = BWF_LOWPASS_DATA_RATE_400;
        }
    }

    // Run the filter on all the captured values then reset the counter and
    //   repeat the process
    count = gAccel.accumulationCount;

    if (count && !i2c_has_error(kAccelerometerI2C) && gAccel.state != eAccelI2CError) {
        // The accelerometer data is being read; will be used as output data.
        //   accelGetLastReading won't be called for another 2.5 milliseconds
        //   so resetting accleration buffer here will prevent data from being
        //   lost in the last 0.5 milliseconds of the current TIM5 interrupt cycle.
        gAccel.state             = eAccelStartGatheringData;
    } else { // count == 0
        gAccel.state = eAccelOff;

        accel_success++;
        if( !InitAccelerometer(TRUE) ) {
            ERROR_STRING("Failed to init accel.\r\n");
        } else {
              
              gAccel.state             = eAccelStartGatheringData;

            _InitAccelDataReadyInt(ENABLE);
        }
        
        return TRUE; // error occured
    }

     
     OSDisableHook();
    // if there is data in the buffer, read it all in and filter it at the prescribed
    //   filter setting
    if (readings) {
        for (i = 0; i < count; i++) {
          // Filter the data (or not)
            switch( gConfiguration.analogFilterClocks[0] & 0xF ) {
                case 0x0:
                case 0x8:
                    // Do not prefilter the accelerometer data
                    accelFilteredVal[X_AXIS] = gAccel.accumulationBuffer[i][X_AXIS];
                    accelFilteredVal[Y_AXIS] = gAccel.accumulationBuffer[i][Y_AXIS];
                    accelFilteredVal[Z_AXIS] = gAccel.accumulationBuffer[i][Z_AXIS];
                    break;

                case 0x1:
                    // 3rd-order, 50 Hz BWF (cascaded 1st-order filters)
                    _accelFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(X_AXIS, gAccel.accumulationBuffer[i][X_AXIS],
                                                                    &accelFilteredVal[X_AXIS],
                                                                    cutoff_freq, input_data_rate);
                    _accelFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(Y_AXIS, gAccel.accumulationBuffer[i][Y_AXIS],
                                                                    &accelFilteredVal[Y_AXIS],
                                                                    cutoff_freq, input_data_rate);
                    _accelFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(Z_AXIS, gAccel.accumulationBuffer[i][Z_AXIS],
                                                                    &accelFilteredVal[Z_AXIS],
                                                                    cutoff_freq, input_data_rate);
                    break;

                case 0x9:
                    // 4th-order, 50 Hz BWF (cascaded 2nd-order filters)
                    _accel_4thOrderBWF_LowPass_Axis_cascaded2nd(X_AXIS,
                                                                gAccel.accumulationBuffer[i][X_AXIS],
                                                                &accelFilteredVal[X_AXIS],
                                                                cutoff_freq, input_data_rate);
                    _accel_4thOrderBWF_LowPass_Axis_cascaded2nd(Y_AXIS, gAccel.accumulationBuffer[i][Y_AXIS],
                                                                &accelFilteredVal[Y_AXIS],
                                                                cutoff_freq, input_data_rate);
                    _accel_4thOrderBWF_LowPass_Axis_cascaded2nd(Z_AXIS, gAccel.accumulationBuffer[i][Z_AXIS],
                                                                &accelFilteredVal[Z_AXIS],
                                                                cutoff_freq, input_data_rate);
                    break;

                default:
                    // 3rd-order, 50 Hz BWF (cascaded 1st-order filters)
                    _accelFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(X_AXIS, gAccel.accumulationBuffer[i][X_AXIS],
                                                                    &accelFilteredVal[X_AXIS],
                                                                    cutoff_freq, input_data_rate);
                    _accelFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(Y_AXIS, gAccel.accumulationBuffer[i][Y_AXIS],
                                                                    &accelFilteredVal[Y_AXIS],
                                                                    cutoff_freq, input_data_rate);
                    _accelFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(Z_AXIS, gAccel.accumulationBuffer[i][Z_AXIS],
                                                                    &accelFilteredVal[Z_AXIS],
                                                                    cutoff_freq, input_data_rate);
                    break;
            }
        }

        /// @brief Limit the rate-sensor reading to prevent the transformation
        /// from causing the signal to saturate with an opposite sign (32767 -->
        /// -32768 after transformation)
        /// Transform the data into the DMU380 frame
        readings[X_AXIS] =  LimitInt16Value( (int16_t)accelFilteredVal[X_AXIS], INT16_LIMIT ); // x
        readings[Y_AXIS] = -LimitInt16Value( (int16_t)accelFilteredVal[Y_AXIS], INT16_LIMIT ); // y
        readings[Z_AXIS] = -LimitInt16Value( (int16_t)accelFilteredVal[Z_AXIS], INT16_LIMIT ); // z
    }
    OSEnableHook();

    gAccel.accumulationCount = 0;
    gAccel.status = ACCEL_BUFFER_AVAILABlE;

    return FALSE;
}

/** ****************************************************************************
 * @name AccelerometerWhoAmI
 * @brief Verify accelerometer attached is one we know how to talk to
 * @param [out] pointer output data will be returned if this is not NULL
 * @retval TRUE if successful, FALSE if error
 ******************************************************************************/
uint8_t  AccelerometerWhoAmI(uint32_t *whoami)
{
    uint8_t read;

    read = _AccReadReg(MMA8451_WHO_AM_I);
    if (whoami) {
      *whoami = read;
    }
    if (read == MMA8451_EXPECTED_WHO_AM_I) { // 0x1A
      return TRUE;
    }
    return FALSE;
}

/** ****************************************************************************
 * @name AccelerometerGetGain
 * @brief Get gain of accelerometer in LSB/G
 * @param N/A
 * @retval 0 if error, gain if succesful
 ******************************************************************************/
uint16_t AccelerometerGetGain()
{
    uint16_t gain;

    _SetStandby();
    gain = _AccReadReg(MMA8451_XYZ_DATA_CFG);
    gain =  (gain & MMA8451_CFG_xG_MASK) >> MMA8451_CFG_xG_SHIFT;
    _SetActive();

    switch (gain)
    {
        case MMA8451_CFG_2G:
            return 4096 << 2; /// the shift up is because
        case MMA8451_CFG_4G:
            return 2048 << 2; /// the data is 14 bit but
        case MMA8451_CFG_8G:
            return 1024 << 2; /// we treat it as 16
        default:
            ERROR_INT("Accelerometer gain is not in range! ", gain);
            ERROR_ENDLINE();
            return 0;
    }
}

/** ****************************************************************************
 * @name AccelerometerConfig
 * @brief 2G's range default
 * @param [in] rangeInGs - data range G's
 * @param [in] outputDataRate - Hz
 * @retval not fail 1 fail 0
 ******************************************************************************/
uint8_t AccelerometerConfig(uint32_t *rangeInGs,
                            uint32_t *outputDataRate)
{
    uint8_t config, reg;

    _SetStandby();
    /// Sensor range set in register 0x0E
    switch (*rangeInGs) {
        case ACCEL_RANGE_8G:
            config = MMA8451_CFG_8G;
            break;
        case ACCEL_RANGE_4G:
            config = MMA8451_CFG_4G;
            break;
        case ACCEL_RANGE_2G: /// two G is the default
        default:
            config = MMA8451_CFG_2G;
            *rangeInGs = ACCEL_RANGE_2G;
    }
    config <<=  MMA8451_CFG_xG_SHIFT;

    reg = _AccReadReg(MMA8451_XYZ_DATA_CFG);
    reg &= ~MMA8451_CFG_xG_MASK;
    reg |= config;
    _AccWriteReg(MMA8451_XYZ_DATA_CFG, reg, TRUE);
    accel_range = _AccReadReg(MMA8451_XYZ_DATA_CFG);

    /// Output data rate and low-noise mode set in Ctrl_Reg1
    ///   Note: there isn't a way to sync the accelerometer so output at the
    ///         fastest rate and use an interrupt to determine when complete
    switch(*outputDataRate) {
        case 800: // Hz
            config = MMA8451_DATARATE_800HZ;
            break;
        case 400:
            config = MMA8451_DATARATE_400HZ;
            break;
        case 200:
            config = MMA8451_DATARATE_200HZ;
            break;
        case 100:
            config = MMA8451_DATARATE_100HZ;
    }
    config <<= MMA8451_DATARATE_SHIFT; // << 3

    /// Place the sensor in low-noise mode if operating in a 4g or lower range
    if (*rangeInGs <= ACCEL_RANGE_4G) {
        config |= MMA8451_CTRL_REG1_LOW_NOISE;
    }
    _AccWriteReg(MMA8451_CTRL_REG1, config, TRUE);

    // Place the accelerometer into high-resolution mode
    //if (*rangeInGs > ACCEL_RANGE_4G) {
    //    config = 0x2;
    //    _AccWriteReg(MMA8451_CTRL_REG2, config, TRUE);
    //}

    _SetActive();

    return TRUE;
}

/** ****************************************************************************
 * @name _InitAccelDataReadyInt
 * @brief Set up data ready GPIO and make it an interrupt
 * @param N/A
 * @retval FALSE if error
 ******************************************************************************/
void _InitAccelDataReadyInt(FunctionalState enable)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    RCC_AHB1PeriphClockCmd(ACCEL_DATA_READY_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); // for interrupts

    /// Configure data ready pin as input
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin  = ACCEL_DATA_READY_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(ACCEL_DATA_READY_GPIO_PORT, &GPIO_InitStructure);

    /// Configure EXTI line
    EXTI_StructInit(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line    = ACCEL_DATA_READY_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = enable;
    EXTI_Init(&EXTI_InitStructure);
    /// Connect EXTI Line to GPIO Pin
    SYSCFG_EXTILineConfig( ACCEL_DATA_READY_EXTI_PORT_SOURCE,
                           ACCEL_DATA_READY_EXTI_PIN_SOURCE ); // C8

    /// Enable and set EXTI Interrupt to the lowest priority
    AccelDataReadyInt( ENABLE );
    EXTI->PR = ACCEL_DATA_READY_EXTI_LINE; // Clear interrupt
}

/** ****************************************************************************
 * @name AccelDataReadyInt API
 * @brief Set up the ACCEL DR interrupt
 * @param enable - the pin ENABLE = 1, DISABLE = 0 stm32f2xx.h
 * @retval N/A
 ******************************************************************************/
void AccelDataReadyInt( FunctionalState enable )
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /// Enable and set EXTI Interrupt to the lowest priority
    NVIC_InitStructure.NVIC_IRQChannel                   = ACCEL_DATA_READY_EXTI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x7; // was 0xc
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = enable;
    NVIC_Init( &NVIC_InitStructure );
}

/** ****************************************************************************
 * @name InitAccelerometer call local function to set up accel DR interrupt
 * @brief I2C3
 * @param N/A
 * @retval value of whoami if succssful
 * @version DKH 10.03.14 moved _InitAccelDataReadyInt() to after I2C init,
 * set the flag to enable and added a dealu after !2C init
 ******************************************************************************/
uint8_t InitAccelerometer(uint8_t reinit)
{
    i2cDebounceBuffer.size   = sizeof(i2cFails);
    i2cDebounceBuffer.values = i2cFails;
    debounce_init(&i2cDebounce, &i2cDebounceBuffer);

    i2c_configure(kAccelerometerI2C); // I2C3
    if (reinit == FALSE)
        DelayMs(10);
   
    _InitAccelDataReadyInt(ENABLE);
    _AccWhoamiNoCheck(); // reset I2c State machine

    _SetStandby();  // set accel to standby prior to changing settings

    _AccWriteReg(MMA8451_CTRL_REG1, // 0x2A
                 0,
                 TRUE); // Reset system-control register 1
    // Select the data-ready interrupt and set the interrupt pin
    _AccWriteReg(MMA8451_CTRL_REG4, // 0x2d
                 MMA8451_CFG4_INT_EN_DRDY, // 0x01
                 TRUE);
    _AccWriteReg(MMA8451_CTRL_REG5, // 0x2e
                 MMA8451_CFG5_INT_PIN_SEL, // 0x00
                 TRUE);
    gAccel.state   = eAccelOff;
    gAccel.i2cBusy = 0;

    _SetActive();

    return AccelerometerWhoAmI(NULL);
}

/** ****************************************************************************
 * @name AccelSelfTest_Bias sends the command to the accelerometer to
 *       apply an electro-static force to the sensors, resulting in a sensor bias.
 * @brief
 * @param [in] apply flag - APPLY 1, REMOVE 0
 * @retval N/A
 ******************************************************************************/
void AccelSelfTest_Bias( uint8_t apply )
{
    uint8_t config = 0xFF;

    _SetStandby();
    config = _AccReadReg( MMA8451_CTRL_REG2 );

    switch (apply) {
        case APPLY: /// Modify the register to set the self-test bit
            config |=  0x80;   // 1000 0000
            break;
        case REMOVE:
        default:
            config &= 0x7F;   // 0111 1111 - reset the self-test bit
    }

    /// Enable self-test; do not wait for transfer to be complete
    _AccWriteReg( MMA8451_CTRL_REG2, config, TRUE );
    _SetActive();
}

/** ****************************************************************************
 * @name getAccelI2CBusy Getter for gAccel.i2cBusy
 * @brief
 * @param N/A
 * @retval BUSY 1, NOT_BUSY  0
 ******************************************************************************/
uint8_t getAccelI2CBusy( void ) { return gAccel.i2cBusy; }

/** ****************************************************************************
 * @name setAccelI2CBusy  Setter for gAccel.i2cBusy
 * @brief
 * @param [in] state - state to set  BUSY 1, NOT_BUSY  0
 * @retval N/A
 ******************************************************************************/
void setAccelI2CBusy( uint8_t state ) { gAccel.i2cBusy = state; }

