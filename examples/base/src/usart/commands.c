/** ***************************************************************************
 * @file   commands.c callback functions from the commandTable
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *  Commands available to the commandLine.c shell
 ******************************************************************************/
#include <math.h> // floating point sqrt

#include "stm32f2xx_conf.h"
#include "commands.h"
#include "commandLine.h"
#include "salvodefs.h"

#define LOGGING_LEVEL LEVEL_INFO
#include "debug.h"
#include "dmu.h"
#include "Indices.h"
#include "temperatureSensor.h"
#include "magnetometer.h"
#include "accelerometer.h"
#include "gyroscope.h"
#include "gyroMAX21000.h"
#include "gps.h"
#include "Indices.h"
#include "timer.h"
#include "xbowsp_init.h"
#include "timer.h"
#include "xbowsp_generaldrivers.h"
#include "bsp.h" // for definition of LED4

// for spi3
#include "spi.h"
#include "boardDefinition.h"
#include "UserCommunication_SPI.h"

// for user-defined baud rate (terminal)
#include "debug_usart.h"

// Display firmware version
void CmdVersion(uint32_t data)
{
    char versionString[N_VERSION_STR/SIZEOF_WORD];
    int i;
    // version is put in using 16 bit chars from TI
    for (i = 0; i < sizeof(versionString); i++) {
        versionString[i] = gCalibration.versionString[i*2];
    }
    DEBUG_STRING(versionString);
    DEBUG_INT(" ", VERSION_MAJOR);
    DEBUG_INT(".", VERSION_MINOR);
    DEBUG_INT(".", VERSION_PATCH);
    DEBUG_INT(".", VERSION_STAGE);
    DEBUG_INT(".", VERSION_BUILD);
    DEBUG_ENDLINE();
} // End of display firmware version

void CmdReadSensors(uint32_t inputParameters)
{
    int16_t  reading[NUM_AXIS*NUM_SENSORS];
    uint32_t numReads = 10;
    uint32_t msApart  = 2;
    uint16_t gain[NUM_SENSORS];
    uint16_t magStart = TRUE;
    tTime    startTime;
    int      axis;
    int      sensor;
    int      readInUnits = inputParameters & 0xF;

    CmdLineGetArgUInt(&numReads);
    CmdLineGetArgUInt(&msApart);

    INFO_INT("Reading sensors ", numReads);
    INFO_INT(" times, ", msApart);
    INFO_STRING(" ms apart\r\n");

    if (readInUnits) { // always do this
        // FIXME: Remove - gain is set via look-up table (but the debug port may
        //        not use the look up table)
        gain[ACCEL_GAIN] = AccelerometerGetGain();
        gain[MAG_GAIN]   = MagnetometerGetGain();
        gain[GYRO_GAIN]  = GyroGetGain();
    }

    DEBUG_STRING("\tt(ms)");
    DEBUG_STRING("\tAX\tAY\tAZ");
    DEBUG_STRING("\tGX\tGY\tGZ");
    DEBUG_STRING("\tMX\tMY\tMZ");
    DEBUG_ENDLINE();
    startTime = TimeNow();
    while (numReads) {
        OSClrEFlag(EFLAGS_DATA_READY, EF_DATA_GYRO_ACCEL_READY);

        // start everybody off
        AccelerometerStartReading();
        //GyroStartReading();

        if (magStart) {
            OSClrEFlag(EFLAGS_DATA_READY, EF_DATA_MAG_READY);
            MagnetometerStartReading();
            magStart = FALSE;
        }

        DelayMs(msApart);
        DEBUG_INT("\t", TimeNow() - startTime);

        // accels and gyros should all be outputting at the same rate: wait for each
        while ( !IsGyroDoneReading() )
        { /*spin*/; }
        GyroGetLastReading( &reading[RATE_START] );

        while (!IsAccelerometerDoneReading())
        { /*spin*/;}
        AccelerometerGetLastReading(&reading[ACCEL_START]);

        if (IsMagnetometerDoneReading()) {
            MagnetometerGetLastReading(&reading[MAG_START]);
            magStart = TRUE;
        }

        if (readInUnits) {
            for (sensor = 0; sensor < NUM_SENSORS; sensor++) {
                for (axis = 0; axis < NUM_AXIS; axis++) {
                    float tmp;
                    tmp = (float) reading[axis + (sensor*NUM_AXIS)] / (float) gain[sensor];
                    DEBUG_FLOAT("\t", tmp, 2);
                }
            }
        } else {
            for (sensor = 0; sensor < NUM_SENSORS; sensor++) {
                for (axis = 0; axis < NUM_AXIS; axis++) {
                    DEBUG_INT("\t", reading[axis + (sensor*NUM_AXIS)]);
                }
            }
        }
        DEBUG_ENDLINE();
        numReads--;
    }
}

/** ***************************************************************************
 * @name CmdChangeOutputDataRate() Change the output data rate
 * @brief this modifies the timer as well as the gyro.
 * ">>sample outputDataRate"
 *
 * @param [in] outputDataRate - rate to be set
 * @retval N/A
 ******************************************************************************/
void CmdChangeOutputDataRate(uint32_t data)
{
    uint32_t outputDataRate = ODR_100_HZ; // 200
    uint32_t range          = (uint32_t) gCalibration.GyroSensorRange;

    DataAquisitionStop();

    CmdLineGetArgUInt(&outputDataRate);
    if (outputDataRate > ODR_100_HZ) {
      outputDataRate = ODR_100_HZ;
    }
    GyroConfig(&range, &outputDataRate);
    InitDataAcquisitionTimer(outputDataRate);

    DEBUG_INT("Averaging now to output ", outputDataRate);
    DEBUG_STRING(" Hz\r\n");
}

/** ***************************************************************************
 * @name CmdReadAccelerometer() Read accelerometer
 * @brief  <num reads, default 10> <ms between reads, default asap>
 * ">>sample numReads msApart"
 *
 * @param [in] numReads - number of times to read the sensor
 * @param [in] msApart - delay between reads
 * @retval N/A
 ******************************************************************************/
void CmdReadAccelerometer(uint32_t readInGs)
{
    int16_t  readings[NUM_AXIS];
    uint32_t numReads = 10;
    uint32_t msApart = 2;
    uint8_t  error;
    uint16_t gain = 0;
    int      i;

    CmdLineGetArgUInt(&numReads);
    CmdLineGetArgUInt(&msApart);

    INFO_INT("Reading accel ", numReads);
    INFO_INT(" times, ", msApart);
    INFO_STRING(" ms apart\r\n");

    if (readInGs) { // default 0
        gain = AccelerometerGetGain();
    }

    DEBUG_STRING("\tX\tY\tZ\tsuccess\r\n");
    AccelerometerStartReading();
    while (numReads) {
        OSClrEFlag(EFLAGS_DATA_READY, EF_DATA_ACCEL_READY);
        DelayMs(msApart);
        while (!IsAccelerometerDoneReading())
        { /*spin*/;}
        error = AccelerometerGetLastReading(readings);
        if (error) {
            uint32_t rangeInGs = ACCEL_RANGE_4G;
            uint32_t outputDataRate = ACCEL_ODR;
            error = InitAccelerometer(FALSE);
            AccelerometerConfig(&rangeInGs, &outputDataRate);
            DEBUG_INT(" ERROR: ", error);
            DelayMs(msApart);
        } else {
            if (readInGs) {
                for (i = 0; i < NUM_AXIS; i++) {
                    float tmp;
                    tmp = (float) readings[i] / (float) gain;
                    DEBUG_FLOAT("\t", tmp, 2);
                }
            } else {
                for (i = 0; i < NUM_AXIS; i++) {
                    DEBUG_INT("\t", readings[i]);
                }
            }
            DEBUG_INT("\t", error);
        }
        DEBUG_ENDLINE();
        numReads--;
    }
}

/** ***************************************************************************
 * @name CmdReadTemperature() Read temperature
 * @brief  <num reads, default 10> <ms between reads, default asap>
 * ">>t numReads msApart" - readInC = 0
 * ">>tt" - readInC = 1
 *
 * @param [in] numReads - number of times to read the sensor
 * @param [in] msApart - delay between reads
 * @retval N/A
 ******************************************************************************/
void CmdReadTemperature(uint32_t readInC)
{
    int16_t  reading;
    float    temperature;
    uint32_t numReads = 10;
    uint32_t msApart  =  0;
    uint16_t gain;
    uint32_t timeout;

    CmdLineGetArgUInt(&numReads);
    CmdLineGetArgUInt(&msApart);

    INFO_INT("Reading temp ", numReads);
    INFO_INT(" times, ", msApart);
    INFO_STRING(" ms apart (each conversion is about 0.75s anyway) \r\n");

    if (!InitTemperatureSensor()) {
        ERROR_STRING("Temp sensor init failed\r\n");
        return;
    }

    if (readInC) {
        gain = TemperatureGetGain();
    }

    DEBUG_STRING("\ttemp\r\n");

    while (numReads) {
       OSClrEFlag(EFLAGS_DATA_READY, EF_DATA_TEMP_READY);

        if (!TemperatureStartReading()) {
            ERROR_STRING("Temp sensor read failed\r\n");
            continue;
        }
        timeout = 300;
        while (timeout && ! IsTemperatureDoneReading()) {
            timeout--;
            DelayMs(5);
        }
        if (TemperatureGetLastReading(&reading)) {
            if (readInC) {
                temperature = (float) reading / (float) gain;
                DEBUG_FLOAT("\t", temperature, 2);
                DEBUG_STRING(" C\r\n");
            } else {
                DEBUG_INT("\t", reading);
                DEBUG_ENDLINE();
            }
        } else {
            ERROR_STRING("Temp sensor read failed\r\n");
        }
        DelayMs(msApart);
        numReads--;
    }
}

/** ***************************************************************************
 * @name CmdReadMagnetometer() Read magnetometer
 * @brief  <num reads, default 10> <ms between reads, default asap>
 * ">>m numReads msApart" - readInGauss = 0
 * ">>mf" - readInGauss = 1
 *
 * @param [in] numReads - number of times to read the sensor
 * @param [in] msApart - delay between reads
 * @retval N/A
 ******************************************************************************/
void CmdReadMagnetometer(uint32_t readInGauss)
{
    int16_t  readings[NUM_AXIS];
    uint32_t numReads = 10;
    uint32_t msApart  =  0;
    uint16_t gain     =  0;
    int      i;

    CmdLineGetArgUInt(&numReads);
    CmdLineGetArgUInt(&msApart);

    INFO_INT("Reading magnetometer ", numReads);
    INFO_INT(" times, ", msApart);
    INFO_STRING(" ms apart\r\n");

    if (readInGauss) {
        gain = MagnetometerGetGain();
    }

    while (numReads) {
        MagnetometerStartReading();
        OSClrEFlag(EFLAGS_DATA_READY, EF_DATA_MAG_READY);
        while (!IsMagnetometerDoneReading())
        { /* spin */ ; }
        MagnetometerGetLastReading(readings);
        if (readInGauss) {
            double mag = 0.0;
            float tmp;
            for ( i = 0; i < NUM_AXIS; i++) {
                tmp = (float) readings[i] / (float) gain;
                mag += tmp * tmp;
                DEBUG_FLOAT("\t", tmp, 2);
            }
            tmp = sqrt(mag);
            DEBUG_FLOAT("\t ( ", tmp, 2);
            DEBUG_STRING(" )");
        } else {
            for ( i = 0; i < NUM_AXIS; i++) {
                DEBUG_INT("\t", readings[i]);
            }
        }
        DEBUG_ENDLINE();
        DelayMs(msApart);
        numReads--;
    }
}

/** ***************************************************************************
 * @name CmdReadGyro() Read gyro
 * @brief  <num reads, default 10> <ms between reads, default asap>
 * ">>g numReads msApart" - readInDegPerSec = 0
 * ">>gf" - readInDegPerSec = 1
 *
 * @param [in] numReads - number of times to read the sensor
 * @param [in] msApart - delay between reads
 * @retval N/A
 ******************************************************************************/
void CmdReadGyro(uint32_t readInDegPerSec)
{
    int16_t  readings[NUM_AXIS];
    uint32_t numReads = 10;
    uint32_t msApart  =  0;
    uint16_t gain;
    int      i;

    CmdLineGetArgUInt(&numReads);
    CmdLineGetArgUInt(&msApart);

    INFO_INT("Reading gyro ", numReads);
    INFO_INT(" times, ", msApart);
    INFO_STRING(" ms apart\r\n");

    if (readInDegPerSec) {
        gain = GyroGetGain();
    }

    while (numReads) {
        OSClrEFlag(EFLAGS_DATA_READY, EF_DATA_GYRO_READY);
        //GyroStartReading();
        while (!IsGyroDoneReading())
        { /* spin */ ; }
        GyroGetLastReading(readings);
        if (readInDegPerSec) {
            for ( i = 0; i < NUM_AXIS; i++ ) {
                float tmp;
                tmp = (float) (readings[i])/ (float) gain;
                DEBUG_FLOAT("\t", tmp, 2);
            }
        } else {
            for ( i = 0; i < NUM_AXIS; i++ ) {
                DEBUG_INT("\t", readings[i]);
            }
        }
        DEBUG_ENDLINE();
        DelayMs(msApart);
        numReads--;
    }
}

/** ***************************************************************************
 * @name CmdReadGyro() Read gyro temperature
 * @brief  <num reads, default 10> <ms between reads, default asap>
 * ">>gtemp numReads msApart" - readInC = 0
 * ">>gtempf" - readInC = 1
 *
 * @param [in] numReads - number of times to read the sensor
 * @param [in] msApart - delay between reads
 * @retval N/A
 ******************************************************************************/
void CmdReadGyroTemp(uint32_t readInC)
{
    int16_t  reading;
    float    temperature;
    uint32_t numReads = 10;
    uint32_t msApart  =  0;

    CmdLineGetArgUInt(&numReads);
    CmdLineGetArgUInt(&msApart);

    INFO_INT("Reading gryo temp ", numReads);
    INFO_INT(" times, ", msApart);
    INFO_STRING(" ms apart\r\n");

    DEBUG_STRING("\ttemp\r\n");

    while (numReads) {
        OSClrEFlag(EFLAGS_DATA_READY, EF_DATA_GYRO_READY);
        //GyroStartReading();
        while (!IsGyroDoneReading())
        { /* spin */ ; }
        GyroGetLastReading(NULL);
        if (! GyroTempGetLastReading(&reading)) {
            if (readInC) {
                GyroTempGetTemperature(reading, &temperature);
                DEBUG_FLOAT("\t", temperature, 2);
                DEBUG_STRING(" C\r\n");
            } else {
                DEBUG_INT("\t", reading);
                DEBUG_ENDLINE();
            }
        } else {
            ERROR_STRING("Temp sensor read failed\r\n");
        }

        DelayMs(msApart);
        numReads--;
    }
}

/** ***************************************************************************
 * @name CmdInertialCalib() Read the accelerometer, rate, mags, and temperature
 *       sensors
 * @brief display the information to the serial terminal. Arguments to the
 *        function dictate whether the function displays floating-point or
 *        integer values, the number of data points, and the time between samples.
 * ">>iCal numReads msApart"  - inputParameters = 0x00
 * ">>iCalf" - inputParameters = 0x01
 * ">>tCal"  - inputParameters = 0x10
 * ">>tCalf" - inputParameters = 0x11
 *
 * @param [in] numReads - number of times to read the sensor
 * @param [in] msApart - delay between reads
 * @retval N/A
 ******************************************************************************/
void CmdInertialCalib(uint32_t inputParameters)
{
    /// Read in data from accelerometer (3 axes), rate sensor (3 axes),
    ///    gyro temperature, and temperature sensor
    ///   (1 measurement) - 8 total readings.
    int16_t  reading[NUM_SENSOR_READINGS] = {0};
    float    gyroTemp;
    float    temperature;
    uint16_t magStart = TRUE;
    uint32_t numReads;
    uint32_t msApart;
    uint16_t dontOutputMags;
    /// Only two sensors (accelerometer and rate) have gains associated with them.
    uint16_t gain[NUM_GAIN_ENTRIES]; // [4]
    tTime    startTime;
    // Counting variables
    int      axis;
    int      sensor;
    /// "readInUnits" controls the format of the terminal output. When set to one,
    ///   floating point values are displayed on the serial terminal, if zero
    ///   then raw counts are displayed.
    int      readInUnits = inputParameters & 0xF;

    if (inputParameters & 0xF0)  /// tcal is a long, slow process
    {   /// Default values of reads
        numReads       = 28800; ///< (1 Hz for 8 hours = 28,800 samples)
        msApart        =  1000;
        dontOutputMags =     0;

    } else { /// ical is shorter, faster
        /// Default values of reads
        numReads       = 4000; ///< (400 Hz for 10 seconds = 4000 samples)
        msApart        = 0;
        dontOutputMags = 1; // too fast to output everything
    }

    CmdLineGetArgUInt(&numReads);
    CmdLineGetArgUInt(&msApart);

    INFO_INT("Reading accelerometer and rate sensors (with temperature) ", numReads);
    INFO_INT(" times, ", msApart);
    INFO_STRING(" ms apart\r\n");

    /// display floating-point representations of the sensor data. This logic
    ///    sets the scale factors of the sensors.
    if (readInUnits)
    {
        gain[ACCEL_GAIN] = AccelerometerGetGain();
        gain[MAG_GAIN]   = MagnetometerGetGain();
        gain[GYRO_GAIN]  = GyroGetGain();
        gain[TEMP_GAIN]  = TemperatureGetGain();
    }

    /// Generate a header (accels, rates, mag, & temp)
    DEBUG_STRING("\tt(ms)");
    DEBUG_STRING("\tAX\tAY\tAZ");
    DEBUG_STRING("\tGX\tGY\tGZ");
    if (dontOutputMags == 0) {
        DEBUG_STRING("\tMX\tMY\tMZ");
    }
    DEBUG_STRING("\tTG\tT");
    DEBUG_ENDLINE();


    startTime = TimeNow();
    TemperatureStartReading();  /// Begin sensor reads

    while (numReads)
    {
        OSClrEFlag(EFLAGS_DATA_READY, EF_DATA_GYRO_ACCEL_READY);
        AccelerometerStartReading();

        if (magStart) {
            OSClrEFlag(EFLAGS_DATA_READY, EF_DATA_MAG_READY);
            MagnetometerStartReading();
            magStart = FALSE;
        }
        //GyroStartReading();

        // Wait then display the time it took to read and convert the sensor data
        DelayMs(msApart);
        DEBUG_INT("\t", TimeNow() - startTime);

        // Accels and gyros should all be outputting at the same rate
        while ( !IsGyroDoneReading() )
        { /*spin*/; }
        GyroGetLastReading( &reading[RATE_START] );

        if( GyroTempGetLastReading( &reading[GYRO_TEMP] ) ) {
            ERROR_STRING( "Temp sensor read failed\r\n" ); }

        while (!IsAccelerometerDoneReading())
        { /*spin*/; }
        AccelerometerGetLastReading(&reading[ACCEL_START]);

        if (IsMagnetometerDoneReading()) {
            MagnetometerGetLastReading(&reading[MAG_START]);
            magStart = TRUE;
        }

        if (IsTemperatureDoneReading()) {
            TemperatureGetLastReading(&reading[TEMP_SENSOR]);
            OSClrEFlag(EFLAGS_DATA_READY, EF_DATA_TEMP_READY);
            TemperatureStartReading();
        } // else, don't worry about it

        // format of the output (float or integer)
        if (readInUnits)
        {
            for (sensor = 0; sensor < NUM_SENSORS - dontOutputMags; sensor++) {
                for (axis = 0; axis < NUM_AXIS; axis++)
                {
                    float tmp;
                    /// Convert to a float and display results
                    tmp = (float)reading[axis + (sensor * NUM_AXIS)] / (float)gain[sensor];
                    DEBUG_FLOAT("\t", tmp, 2);
                }
            }

            GyroTempGetTemperature( reading[GYRO_TEMP], &gyroTemp );
            DEBUG_FLOAT("\t", gyroTemp, 2);

            temperature = (float)reading[TEMP_SENSOR];
            temperature = temperature / (float)gain[TEMP_GAIN];
            DEBUG_FLOAT("\t", temperature, 2);
        }
        else
        {
            for (sensor = 0; sensor < NUM_SENSORS-dontOutputMags; sensor++)
            {
                for (axis = 0; axis < NUM_AXIS; axis++)
                {   /// raw counts
                    DEBUG_INT("\t", reading[axis + (sensor * NUM_AXIS)]);
                }
            }

            /// raw counts
            DEBUG_INT("\t", reading[GYRO_TEMP]);
            DEBUG_INT("\t", reading[TEMP_SENSOR]);
        }
        DEBUG_ENDLINE();
        numReads--;
    }
}

/** ***************************************************************************
 * @name CmdSelfTest() Run self test
 * @brief verify existence of each sensor.
 * ">>swtest"  - data = 0x00
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CmdSelfTest(uint32_t data)
{
    uint32_t whoami;

#ifdef USE_MAIN_TEMPERTATURE
    uint32_t timeout = 300;
    int16_t  reading;
    float    temperature;
    uint16_t gain;

    DEBUG_STRING("Checking temperature sensor:\t");
    if (InitTemperatureSensor()) {
        gain = TemperatureGetGain();
        OSClrEFlag(EFLAGS_DATA_READY, EF_DATA_TEMP_READY);
        TemperatureStartReading(); //main board temp tempTMP102.c
        while (timeout && ! IsTemperatureDoneReading()) {
          timeout--;
          DelayMs(5);
        }
        TemperatureGetLastReading(&reading);
        temperature = (float) reading / (float) gain;
        DEBUG_FLOAT("SUCCESS (", temperature, 2);
        DEBUG_STRING(" C)\r\n");
    } else {
        DEBUG_STRING("FAIL - check I2C open \r\n");
    }
#endif

    DEBUG_STRING("Checking magnetometer:\t");
    whoami = 0;
    if (MagnetometerWhoAmI(&whoami)) {
        if (MagnetometerSelfTest()) {
            DEBUG_HEX("SUCCESS (", whoami);
            DEBUG_STRING(")\r\n");
        } else {
            DEBUG_HEX("FAIL (", whoami);
            DEBUG_STRING(") - mag self test failed\r\n");
        }
    } else {
        DEBUG_HEX("FAIL (", whoami);
        DEBUG_STRING(") - whoami failed, check mag inited and I2C open \r\n");
    }

    DEBUG_STRING("Checking accelerometer:\t");
    if (AccelerometerWhoAmI(&whoami)) {
        DEBUG_HEX("SUCCESS (", whoami);
        DEBUG_STRING(")\r\n");
    } else {
        DEBUG_HEX("FAIL (", whoami);
        DEBUG_STRING(") - check accel inited and I2C open \r\n");
    }
#ifdef FL
    DEBUG_STRING("Checking gryoscope:\t");
    if (GyroWhoAmI(&whoami)) {
        if (GyroSelfTest()) {
            DEBUG_HEX("SUCCESS (", whoami);
            DEBUG_STRING(")\r\n");
        } else {
            DEBUG_HEX("FAIL (", whoami);
            DEBUG_STRING(") - gyro self test failed \r\n");
        }
    } else {
        DEBUG_HEX("FAIL (", whoami);
        DEBUG_STRING(") - check gyro inited and SPI open \r\n");
    }
#endif
}

/** ***************************************************************************
 * @name CmdAccelInit() initialize the accelerometer sensors
 * @brief
 * ">>ai rangeInGs outputDataRate"  - data = 0x00
 *
 * @param [in] rangeInGs - data range G's
 * @retval N/A
 ******************************************************************************/
void CmdAccelInit(uint32_t data)
{
    uint32_t rangeInGs      = ACCEL_RANGE_4G;
    uint32_t outputDataRate = ACCEL_ODR; // fixed to allow for accumulation

    CmdLineGetArgUInt(&rangeInGs);

    if (InitAccelerometer(FALSE)) {
        AccelerometerConfig(&rangeInGs, &outputDataRate);
//DEBUG_INT("Accel init'd ok, ", rangeInGs);
//DEBUG_STRING(" G range and (fixed) 800 Hz output\r\n");
    } else {
        DEBUG_STRING("Accel error on init\r\n");
    }
}

/** ***************************************************************************
 * @name CmdGyroInit() initialize the gyro sensors
 * @brief
 * ">>gi rangeInDps outputDataRate"  - data = 0x00
 *
 * @param [in] rangeInDps - degrees per second
 * @param [in] outputDataRate - hz
 * @retval N/A
 ******************************************************************************/
void CmdGyroInit(uint32_t data)
{
    uint32_t rangeInDps     = DEFAULT_GYRO_RANGE; // 250
    uint32_t outputDataRate = ODR_200_HZ;   // 200 - asap

    CmdLineGetArgUInt(&rangeInDps);
    CmdLineGetArgUInt(&outputDataRate);

    if (InitGyro()) {
        GyroConfig(&rangeInDps, &outputDataRate);
        DEBUG_INT("Gyro init'd ok, ", rangeInDps);
        DEBUG_INT(" dps range and  ", outputDataRate);
        DEBUG_STRING(" Hz output\r\n");
    } else {
        DEBUG_STRING("Gyro error on init\r\n");
    }
}


/** ***************************************************************************
 * @name CmdApplyRateSensorBias()  User (debug) command
 * @brief to apply and remove the sensor bias from rate-sensor data (applied via
 * an electrostatic force internal to the sensor).  Used during a sensor
 * self-test.
 * ">>gb applyBiasFlag"  - data = 0x00
 *
 * @param [in] applyBiasFlag -
 * @retval N/A
 ******************************************************************************/
void CmdApplyRateSensorBias(uint32_t data)
{
    uint32_t applyBiasFlag = 0;

    CmdLineGetArgUInt( &applyBiasFlag );

    if( applyBiasFlag ) {
        DEBUG_STRING("Gyro self test applying bias\r\n");
        GyroSelfTest_Bias( APPLY );
    } else {
        DEBUG_STRING("Gyro self test removing bias\r\n");
        GyroSelfTest_Bias( REMOVE );
    }
}

/** ***************************************************************************
 * @name CmdApplyAccelSensorBias()  User (debug) command
 * @brief to apply and remove the sensor bias from accel-sensor data (applied via
 * an electrostatic force internal to the sensor).  Used during a sensor
 * self-test.
 * ">>gb applyBiasFlag"  - data = 0x00
 *
 * @param [in] applyBiasFlag -
 * @retval N/a
 ******************************************************************************/
void CmdApplyAccelSensorBias(uint32_t data)
{
    uint32_t applyBiasFlag = 0;

    CmdLineGetArgUInt( &applyBiasFlag );

    if( applyBiasFlag ) {
        AccelSelfTest_Bias( APPLY );
    } else {
        AccelSelfTest_Bias( REMOVE );
    }
}


/** ***************************************************************************
 * @name CmdPerformAccelSelfTest()  User (debug) command
 * @brief to apply and remove the sensor bias from accel-sensor data (applied via
 * an electrostatic force internal to the sensor).  Used during a sensor
 * self-test. repeats 1000 times
 * ">>gb applyBiasFlag"  - data = 0x00
 *
 * @param [in] applyBiasFlag -
 * @retval N/a
 ******************************************************************************/
void CmdPerformAccelSelfTest(uint32_t data)
{
    uint32_t applyBiasFlag = 0;
    uint16_t iteration = 0;
    uint16_t numberOfIterations = 1000;
    uint32_t arguments[ 3 ]     = { 0, 5, 2 };

    CmdLineGetArgUInt( &applyBiasFlag );

    CmdReadAccelerometer( arguments[0] );
    DelayMs( 1 );
    for( iteration = 0; iteration < numberOfIterations; iteration++ ) {
        AccelSelfTest_Bias( APPLY );
        DelayMs( 1 );
        CmdReadAccelerometer( arguments[0] );
        DelayMs( 1 );
        AccelSelfTest_Bias( REMOVE );

        DelayMs( 1 );
        CmdReadAccelerometer( arguments[0] );
    }
}

/** ***************************************************************************
 * @name CmdMagnetometerInit()  initialize magentometer
 * @brief to apply the sensor range in milli Gauss
 *        Need to ensure that the proper range is available for the magnetometer
 *        and the functions work (and the data is correct)
 * ">>mi rangeInMilliGauss"  - data = 0x00
 *
 * @param [in] rangeInMilliGauss - set sensor output rangge
 * @retval N/a
 ******************************************************************************/
void CmdMagnetometerInit(uint32_t data)
{
    uint32_t rangeInMilliGauss = 8100; // Default value is 8.1 Gauss range

    CmdLineGetArgUInt(&rangeInMilliGauss);

    if (InitMagnetometer()) {
        MagnetometerConfig(&rangeInMilliGauss);
        DEBUG_INT("Mag init'd ok, ", rangeInMilliGauss);
        DEBUG_STRING(" mGa\r\n");
    } else {
        DEBUG_STRING("Mag error on init\r\n");
    }
}

/** ***************************************************************************
 * @name CmdAutoDataAquisitionMode() start or stop automatic data acquisition
 * @brief
 * ">>auto on" - data = 0x00
 *
 * @param [in] on - 1 = start, 0 = stop
 * @retval N/a
 ******************************************************************************/
void CmdAutoDataAquisitionMode(uint32_t data)
{
    uint32_t on = TRUE;

    CmdLineGetArgUInt(&on);

    if (on) {
        DataAquisitionStart();
    } else {
        DataAquisitionStop();
    }
}

#include "port_def.h"
#include "comm_buffers.h"
#include "uart.h"
extern port_struct gPort0, gPort1; /// reference to physical port structures
/** ***************************************************************************
 * @name CmdUserUsart() test of UART ports
 * @brief
 * ">>output uart buffer" - data = 0x00
 *
 * @param [in] uart - port 0 or 1
 * @param [in] buffer - pointer to string to send out
 * @retval N/a
 ******************************************************************************/
void CmdUserUsart(uint32_t data)
{
    uint32_t      uart = 0;
    unsigned char *buffer;
    port_struct   *port;
    int           ok;

    CmdLineGetArgUInt(&uart);
    CmdLineGetArgString(&buffer);

    if (uart == 0) {
        port = &gPort0;
    } else if (uart == 1) {
        port = &gPort1;
    } else {
        ERROR_STRING("Unknown port\r\n");
        return;
    }

    ok = COM_buf_add(&(port->xmit_buf),
                      buffer,
                      strlen((const char*)buffer));
    uart_write(uart, port);
    DEBUG_STRING("Wrote (");
    DEBUG_STRING((const char*)buffer);
    DEBUG_INT(") to ", uart);
    DEBUG_INT(", ok= ", ok);
    DEBUG_ENDLINE();
}

/** ***************************************************************************
 * @name CmdGpioPin() select and set up and set the state of a output pin
 * @brief
 * ">>pin port pin state" - data = 0x00
 *
 * @param [in] port - letter
 * @param [in] pin - number
 * @param [in] state - set [1] or reset [0]
 * @retval N/A
 ******************************************************************************/
void CmdGpioPin(uint32_t data)
{
    uint32_t         pin   = 1;
    uint32_t         state = 1;
    uint8_t          *port;
    GPIO_TypeDef     *GPIO;
    GPIO_InitTypeDef GPIO_InitStructure;

    CmdLineGetArgString(&port);
    CmdLineGetArgUInt(&pin);
    CmdLineGetArgUInt(&state);

    switch(port[0]) {
    case 'A':
      GPIO = GPIOA;
      break;
    case 'B':
      GPIO = GPIOB;
      break;
    case 'C':
      GPIO = GPIOC;
      break;
    case 'D':
      GPIO = GPIOD;
      break;
    case 'F':
      GPIO = GPIOF;
      break;
    default: ERROR_STRING("Unknown port\r\n");
      return;
    }

    pin = 1 << pin;
    GPIO_InitStructure.GPIO_Pin   = pin;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIO, &GPIO_InitStructure);

    // Set [1] or reset [0] pin
    if (state) {
        GPIO_SetBits(GPIO, pin);
    } else {
        GPIO_ResetBits(GPIO, pin);
    }
}

#include "s_eeprom.h"
/** ***************************************************************************
 * @name CmdEeRead() read EEprom memory
 * @brief
 * ">>eeRead addr numWords" - data = 0x00
 *
 * @param [in] addr - address to read
 * @param [in] numWords - number of words to read from memory
 * @retval N/A
 ******************************************************************************/
void CmdEeRead(uint32_t junk)
{
    uint32_t       addr = 0;
    uint16_t       data[60];
    const uint32_t sizeofData = sizeof(data) / sizeof(data[0]);
    uint32_t       numWords = sizeofData;
    int            i;

    CmdLineGetArgUInt(&addr);
    CmdLineGetArgUInt(&numWords);

    if (numWords > sizeofData) {
         numWords = sizeofData;
    }
    readEEPROMWords(addr, numWords, data);
    for ( i = 0; i < numWords; i++) {
        DEBUG_HEX("x ", i);
        DEBUG_HEX(" ", data[i]);
        DEBUG_ENDLINE();
        DelayMs(100);
    }
}

/** ***************************************************************************
 * @name CmdEeWrite() write data to EEprom memory
 * @brief
 * ">>eeWrite addr numWords data" - data = 0x00
 *
 * @param [in] addr - address to write to
 * @param [in] numWords - number of words to write
 * @param [in] data - word to write to memory
 * @retval N/A
 ******************************************************************************/
void CmdEeWrite(uint32_t junk)
{
    uint32_t       addr = 0;
    uint32_t       data = 0;
    const uint32_t sizeofData = sizeof(data) / sizeof(uint16_t);
    uint32_t       numWords   = sizeofData;
    int            i;
    int            success;

    CmdLineGetArgUInt(&addr);
    CmdLineGetArgUInt(&numWords);
    CmdLineGetArgUInt(&data);

    if (numWords > sizeofData) {
        numWords = sizeofData;
    }

    success = writeEEPROMWords(addr,
                               numWords,
                               &data);
    if (success) {
        DEBUG_STRING("Write succeeded\r\n")
        for ( i = 0; i < numWords; i++) {
            DEBUG_HEX("x ", i);
            DEBUG_HEX(" ", data);
            DEBUG_ENDLINE();
        }
    } else {
        DEBUG_STRING("Write failed.\r\n");
    }
}


// Command to display or supress sensor data
void CmdDisplayDebugMessages_SensorData( uint32_t data )
{
    static uint32_t DisplayData = 1;

    CmdLineGetArgUInt( &DisplayData );

    if( DisplayData == 1 ) {
        INFO_STRING( "Displaying sensor data\r\n" );
    } else {
        INFO_STRING( "Suppressing the display of sensor data\r\n" );
    }
}


/** ***************************************************************************
 * @name CmdBlinkLED() Blink LED4
 * @brief
 * ">>blink numBlinks msApart" - data = 0x00
 *
 * @param [in] numBlinks - number of times to blink the LED
 * @param [in] msApart - rate of blinking in ms
 * @retval N/A
 ******************************************************************************/
void CmdBlinkLED(uint32_t data)
{
#ifdef FL // disable blinking function
    static uint32_t blinks;

    uint32_t numBlinks = 3;
    uint32_t msApart = 500;

    CmdLineGetArgUInt( &numBlinks );
    CmdLineGetArgUInt( &msApart );

    blinks = numBlinks;

    INFO_INT( "Blinking LED4 ", blinks );
    INFO_INT( " times, ", msApart );
    INFO_STRING( " ms apart\r\n" );
    DEBUG_ENDLINE();

    while( blinks )
    {   /// Toggle the LED
        led_on(LED4);

        DelayMs(msApart / 2);
        led_off(LED4);
        DelayMs(msApart / 2);
        blinks--;
    }
#endif //FL
}

/** ***************************************************************************
 * @name CmdUsartBaudRate() set/change the serial (USART) baudrate
 * @brief
 * ">>termBaud baudRateSelector" - data = 0x00
 *
 * @param [in] baudRateSelector - data rate in baud
 * @retval N/A
 ******************************************************************************/
void CmdUsartBaudRate( uint32_t data )
{
    static uint32_t baudRate;
    uint32_t        baudRateSelector = 3;

    CmdLineGetArgUInt( &baudRateSelector );

    /// compute the baud rate based on the argument
    if( baudRateSelector <= 6 ) {
        baudRate = 921600 >> baudRateSelector;
    } else {
        baudRate = 115200;
    }

    INFO_INT( "Setting USART baud rate to ", baudRate );
    INFO_STRING( ".\r\n" );
    DEBUG_ENDLINE();

    DelayMs( 2000 );
    InitDebugSerialCommunication( baudRate ); ///< the USART (serial) port
}

// Should have another command to initialize the UART port
/** ***************************************************************************
 * @name CmdInitSPIPeripheral() set/reset the SPI bus
 * @brief
 * ">>initSpi" - data = 0x00
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CmdInitSPIPeripheral( uint32_t data )
{
    InitCommunication_UserSPI();
}

#include "driverGPS.h"
//extern void processMsg(char *msg, unsigned int *msgLength, GpsData_t *GPSData);

/** ***************************************************************************
 * @name PrintGPSPositionHeader() send header lables out to USART debug console
 * @brief
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
static void PrintGPSPositionHeader()
{
    DEBUG_STRING("\t Lat \t\t Lon \t\t Alt (m)\r\n");
}

/** ***************************************************************************
 * @name PrintGPSPosition() send lat, lon, alt out to USART debug console
 * @brief
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
static void PrintGPSPosition()
{
//    DEBUG_INT("\t", gGpsDataPtr->lat_deg * gGpsDataPtr->lat_sign);
//    DEBUG_INT(" ", gGpsDataPtr->lat_min);
//    DEBUG_FLOAT(" ", gGpsDataPtr->lat_min_fraction, 3);
//
//    DEBUG_INT("\t", gGpsDataPtr->lon_deg * gGpsDataPtr->lon_sign);
//    DEBUG_INT(" ",  gGpsDataPtr->lon_min);
//    DEBUG_FLOAT(" ", gGpsDataPtr->lon_min_fraction, 3);
//
//    DEBUG_FLOAT("\t", gGpsDataPtr->alt, 3);
//    DEBUG_ENDLINE();
}

/** ***************************************************************************
 * @name PrintGPSPosition() send header and lat, lon, alt out to USART debug
 * @brief console
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CmdParseGPS(uint32_t data)
{
    char *s;
    //unsigned int len;
    CmdLineGetArgString((uint8_t**)&s);
    //len = strlen(s) + 2; // process needs /r and /n to make gps processing happy
    //processMsg(s, &len, gGpsDataPtr);
    PrintGPSPositionHeader();
    PrintGPSPosition();
}

/** ***************************************************************************
 * @name CmdGpsInit() send config command for internal GPS data format
 * @brief baud rate does not make it through: default is 4800
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CmdGpsInit(uint32_t data)
{
    int32_t baudRate = 4800;
    CmdLineGetArgInt( &baudRate );

    int32_t protocol = NMEA_TEXT;
    CmdLineGetArgInt( &protocol );

    SetConfigurationProtocolGPS(protocol);
    SetConfigurationBaudRateGps(baudRate);
    //initGPSHandler();
}

/** ***************************************************************************
 * @name CmdGpsHandler() call the GPS parser and send results out to the debug
 * @brief console
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CmdGpsHandler(uint32_t data)
{
 //   int  initialBytes      = COM_buf_bytes_available(&(gPort1.rec_buf));
    int i;

    int32_t time = 0;
    tTime   startTime;

    CmdLineGetArgInt( &time );
    startTime = TimeNow();
    PrintGPSPositionHeader();

    do {
        //GPSHandler();
    if  ( ((gGpsDataPtr->GPSFix == 0) && (gGpsDataPtr->GPSProtocol == SIRF_BINARY)) || // SiRF 0 = fix non-zero less than ideal fix
          ((gGpsDataPtr->GPSFix != 0) && (gGpsDataPtr->GPSProtocol == NMEA_TEXT))) {  // NMEA > 0 = fix, zero no fix
            PrintGPSPosition();

            DEBUG_STRING("Vel: ");
            for ( i = 0; i < 3; i++) {
                DEBUG_FLOAT(" ", gGpsDataPtr->vNed[i], 4);
            }
            DEBUG_ENDLINE();
        }
    } while (TimePassed(startTime) < time * 1000) ;
}

/** ***************************************************************************
 * @name CmdGpsInternal() set the flag and change running config for internal or
 * @brief bypass the gps
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CmdGpsInternal(uint32_t data)
{
    int32_t config = 1;

    CmdLineGetArgInt( &config );

    gCalibration.productConfiguration.bit.hasGps = config;
    //initGPSHandler(); //reinit for internal
}

/** ***************************************************************************
 * @name CmdGpsRead() print the header and current position
 * @brief bypass the gps
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CmdGpsRead(uint32_t data)
{
    PrintGPSPositionHeader();
    PrintGPSPosition();
}
