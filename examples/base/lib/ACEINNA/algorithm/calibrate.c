/** ***************************************************************************
 * @file calibrate.c
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *  Sensor Calibration Algorithms.
 ******************************************************************************/
#include <stdint.h>
#include <string.h> // memcpy

#include "dmu.h"
#include "calibrate.h"
#include "configuration.h"
#include "scaling.h"
#include "stm32f2xx.h"
#include "taskUserCommunication.h"
#include "UserCommunication_SPI.h"
#include "filter.h"
#include "ucb_packet.h" // UcbPacketStruct
#include "qmath.h"
#include "debug.h"
#include "lowpass_filter.h"

//// For filter testing
#include "math.h"

//#include "CompilerFlags.h"   // for GYRO_MAXIM21000 and GYRO_BMI160

static double _ApplySensorCalibration( int sensor, int tempSensor );
static void   _AdjustForSensorMisalignment( int sensor, double *scaledSensors );
static void   _AdjustForSensorMisalignment_q27( int sensor, int32_t *scaledSensors_q27 );
void          _orientSensors( double *scaledSensors );
void          _LimitSensorValues( double *scaledSensors );

extern UcbPacketStruct gSpiUcbPacket; // taskDataAcquisition.c

/// Compensation lookup index... first six values are for temperature bias
///   (accelerometer and rate-sensor), next nine are for scale factor
///   (accelerometer, rate-sensor, and magnetometer)
int16_t gCurrentTableLookupIndex[15] = {0};

/** ***************************************************************************
 * @name _searchTempTable() LOCAL linear interpolation
 * @brief linear interpolation based on a look-up table to estimate the
 *        temperature bias-compensation for the rate sensors and accelerometers.
 *
 * given inertial sensor index 0:5 (accels,gyros), this function looks at temp
 * sensor and searches the inertial sensor's temp comp table to find the correct
 * table entry. The search begins from where the last search ended.
 * Note: The "temperatures" are in counts
 *
 * @param [in] sensor - inertial sensor index 0:5 (accels, gyros)
 * @param [in] tempReading - index
 * @retval The interpolated temp bias is calculated and returned.
 ******************************************************************************/
static uint32_t _searchTempTable( int      sensor,
                                  uint32_t tempReading )
{
    int16_t  B_Index, E_Index;
    uint32_t BCount,  ECount;
    int32_t  BValue,  EValue;
    double   valOverCount;
// FIXME: this only needs to be done at one hz and only needs to be re-calulated
// if the slope changes
    /// linear interpolation with the independent value (x) being temperature
    ///   dependent value (y) the bias.  The equation used is y = y1 + m( x - x1 ),
    ///   where x1 is equivalent to BCount.  ECount ~= to the x2 variable used to
    ///   compute the slope, m = ( y2 - y1 )/( x2 - x1 ).
    ///
    ///    |        /
    ///    |       o ( x2, y2 ) = ( ECount, EValue )
    ///  y -------+
    ///    |     /.
    ///    |    / .
    ///  --|---/--|----------
    ///    |  /   x
    ///    | /
    ///    |o ( x1, y1 ) = ( BCount, BValue )
    ///    /
    ///   /|

    /// Begin search at the last point. The temperature does not change rapidly;
    ///   reduces steps in the search. B_Index - location of count and value in
    ///   the calibration table.
    B_Index = gCurrentTableLookupIndex[sensor];
    E_Index = B_Index + 1;

    /// "counts" are the independent variables (x) in the calibration tables
    BCount = gCalibration.calibrationTablesA[B_Index][0];
    ECount = gCalibration.calibrationTablesA[E_Index][0];

    if (tempReading < BCount)
    {   /// drop to a lower index and recheck
        do {
            /// If the program cannot go to a lower index, drop out of the do-loop
            if (B_Index <= gCalibration.calibrationTableIndexA[sensor]) {
                B_Index = gCalibration.calibrationTableIndexA[sensor];
                break;
            }
            B_Index--;
            BCount = gCalibration.calibrationTablesA[B_Index][0];
            if (tempReading >= BCount) {
                break;
            }
        } while (1);

        /// Save off the beginning index for the next time through the table
        gCurrentTableLookupIndex[sensor] = B_Index;

        E_Index = B_Index + 1; /// end-index
        /// end count (equivalent to x2 in the interpolation scheme)
        ECount = gCalibration.calibrationTablesA[E_Index][0];
    }
    else if (tempReading > ECount)
    {   // increase the index and recheck
        do {
            // If the program cannot go to a higher index, drop out of the do-loop
            if( E_Index >= ( gCalibration.calibrationTableIndexA[ sensor + 1 ] - 1 ) ) {
                E_Index = gCalibration.calibrationTableIndexA[ sensor + 1 ] - 1;
                break;
            }
            E_Index++;
            ECount = gCalibration.calibrationTablesA[E_Index][0];
            if (tempReading <= ECount) {
                break; }
        } while (1);

        /// Save off the index for the next time through the table, and determine
        ///   BCount (equivalent to x1 in the interpolation scheme)
        B_Index = E_Index - 1;
        gCurrentTableLookupIndex[sensor] = B_Index;
        BCount  = gCalibration.calibrationTablesA[B_Index][0];
    }

    // ------ linear interpolation ------
    // 1) y1 and y2
    BValue = gCalibration.calibrationTablesA[B_Index][1];
    EValue = gCalibration.calibrationTablesA[E_Index][1];

    // 2) m = ( y2 - y1 )/( x2 - x1 )
    valOverCount = ((double)(EValue - BValue) / (double)(ECount - BCount));

    // 3)  y = y1 + m*( x - x1 ) FIXME: this should be rounded it is truncated now
    uint32_t temp = (uint32_t)(BValue + ((int32_t)(tempReading - BCount)) * valOverCount);
    return temp;
}

/** ***************************************************************************
 * @name _searchScaleTable() LOCAL table search
 * @brief given sensor index 0:8 (accels,gyros,mags), and the temp compensated
 *        sensor counts, this function searches the sensor's scale table to find
 *        the correct table entry.  The search begins from where the last search
 *        ended.
 *
 * @param [in] sensor sensor index 0:8 (accels,gyros,mags)
 * @param [in] sensorCounts - temp compensated sensor counts
 * @retval The interpolated engineering unit value is calculated and returned.
 ******************************************************************************/
double _searchScaleTable(int     sensor,
                         int32_t sensorCounts)
{
    int16_t B_Index;
    int16_t E_Index;
    int32_t BCount;
    int32_t ECount;
    float   BValue;
    float   EValue;
    double  valOverCount;
    double  num;
    double  den;
    double  temp;

    B_Index = gCurrentTableLookupIndex[6 + sensor];
    E_Index = B_Index + 1;
    BCount = gCalibration.calibrationTablesA[B_Index][0];
    ECount = gCalibration.calibrationTablesA[E_Index][0];

    if (sensorCounts < BCount)  {
        do {
            if( B_Index <= gCalibration.calibrationTableIndexA[ sensor + 6 ] ) {
                B_Index =  gCalibration.calibrationTableIndexA[ sensor + 6 ];
                break;
            }
            B_Index--;
            BCount = gCalibration.calibrationTablesA[B_Index][0];
        } while (sensorCounts < BCount);

        gCurrentTableLookupIndex[6 + sensor] = B_Index;
        E_Index = B_Index + 1;
        ECount = gCalibration.calibrationTablesA[E_Index][0];

    } else if (sensorCounts > ECount)  {
        do {
            if( E_Index >= ( gCalibration.calibrationTableIndexA[ sensor + 6 + 1 ] - 1 ) ) {
                E_Index =    gCalibration.calibrationTableIndexA[ sensor + 6 + 1 ] - 1;
                break; }
            E_Index++;
            ECount = gCalibration.calibrationTablesA[E_Index][0];
        } while (sensorCounts > ECount);

        B_Index                            = E_Index - 1;
        gCurrentTableLookupIndex[6+sensor] = B_Index;
        BCount                             = gCalibration.calibrationTablesA[B_Index][0];
    }

    /// force the bits into a double!
    *((uint32_t*)&BValue) = gCalibration.calibrationTablesA[B_Index][1];
    *((uint32_t*)&EValue) = gCalibration.calibrationTablesA[E_Index][1];
    num          = (double)(EValue - BValue);
    den          = (double)(ECount - BCount);
    valOverCount = num / den;
    temp         = (double)BValue + ((double)(sensorCounts - BCount)) * valOverCount;
    return temp;
}

/** ***************************************************************************
 * @name CalibrateInit() set up defaults in calibration
 * @brief Initialize the calibration global variables in case the eeprom version
 *  isn't any good
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CalibrateInit( void )
{
    int       i;
    int       j;
    int       s;
    int       calStart;
    int       calIndex;
    const int tempStep    = MAXUINT32 >> 5; // 0 to 65k in 20 something steps
    const int tempStart   = 0;
    const int sensorStep  = MAXUINT16 >> 5;
    const int sensorStart = -MAXINT16;
    float     scaleValueFloat;
    int       scaleValueInt;

    int sizeofArray = sizeof(gCurrentTableLookupIndex) / sizeof(gCurrentTableLookupIndex[0]);

    if (gCalibration.serialNumber == 0) { /// unit not calibrated, use default settings
        for (i = 0; i < NUM_AXIS; i++) {
            /// build temp tables that don't do anything
            calStart = ACCEL_CAL_TABLE_OFFSET + (i * ACCEL_CAL_TABLE_SIZE);
            calIndex = ACCEL_START + i + TEMP_BIAS_OFFSET;
            gCalibration.calibrationTableIndexA[calIndex] = calStart;

            for (j = 0; j < ACCEL_CAL_TABLE_SIZE / 2; j++) {
                calIndex = j + calStart;
                gCalibration.calibrationTablesA[calIndex][TEMP_COUNT] = (tempStep * j) - tempStart;
                gCalibration.calibrationTablesA[calIndex][BIAS_VALUE] = 0;
            }
            calIndex = ACCEL_START + i + TEMP_SCALE_OFFSET;
            gCalibration.calibrationTableIndexA[calIndex] = calStart +  ACCEL_CAL_TABLE_SIZE / 2;
            for (j = j, s = 0; j < ACCEL_CAL_TABLE_SIZE; j++, s++) {
                calIndex = j + calStart;
                // if count == step, then bias ends up at 1.0
                scaleValueFloat =  (sensorStep * s) + sensorStart;
                scaleValueInt   = *(uint32_t *)&scaleValueFloat;
                gCalibration.calibrationTablesA[calIndex][SENSOR_COUNT] = (uint32_t) scaleValueFloat;
                gCalibration.calibrationTablesA[calIndex][SCALE_VALUE]  = scaleValueInt;
            }

            calStart = RATE_CAL_TABLE_OFFSET + (i * RATE_CAL_TABLE_SIZE);
            calIndex = RATE_START + i + TEMP_BIAS_OFFSET;
            gCalibration.calibrationTableIndexA[calIndex] = calStart;
            for (j = 0; j < RATE_CAL_TABLE_SIZE / 2; j++) {
                calIndex = j + calStart;
                gCalibration.calibrationTablesA[calIndex][TEMP_COUNT] = (tempStep * j) - tempStart;
                gCalibration.calibrationTablesA[calIndex][BIAS_VALUE] = 0;
            }
            calIndex = RATE_START + i + TEMP_SCALE_OFFSET;
            gCalibration.calibrationTableIndexA[calIndex] = calStart +  RATE_CAL_TABLE_SIZE / 2;
            for (j = j, s = 0; j < RATE_CAL_TABLE_SIZE; j++, s++) {
                calIndex        = j + calStart;
                scaleValueFloat =  (sensorStep * s) + sensorStart;
                scaleValueInt   = *(uint32_t *)&scaleValueFloat;
                gCalibration.calibrationTablesA[calIndex][SENSOR_COUNT] = (uint32_t) scaleValueFloat;
                gCalibration.calibrationTablesA[calIndex][SCALE_VALUE]  = scaleValueInt;
            }

            calStart = MAG_CAL_TABLE_OFFSET + (i * MAG_CAL_TABLE_SIZE);
            calIndex = MAG_START + i + TEMP_SCALE_OFFSET; // no temp bias for mag
            gCalibration.calibrationTableIndexA[calIndex] = calStart;
            for (j = 0; j < MAG_CAL_TABLE_SIZE ; j++) {
                calIndex        = j + calStart;
                scaleValueFloat =  (sensorStep * j) + sensorStart;
                scaleValueInt   = *(uint32_t *)&scaleValueFloat;
                gCalibration.calibrationTablesA[calIndex][SENSOR_COUNT] = (uint32_t) scaleValueFloat;
                gCalibration.calibrationTablesA[calIndex][SCALE_VALUE]  = scaleValueInt;
            }

        }
        /// last entry for out-of-range look up
        calStart = ACCEL_CAL_TABLE_OFFSET + (i * ACCEL_CAL_TABLE_SIZE);
        calIndex = ACCEL_START + i + TEMP_BIAS_OFFSET;
        gCalibration.calibrationTableIndexA[calIndex] = calStart;

        /// misalignment table
        for ( i = 0; i < NUM_AXIS * 2; i++ ) {
           gCalibration.misalign[i] = 0; /// no off axis influences
        }

    } // end if no serial number so fake a calibration table

    for (i = 0; i < sizeofArray; i++) {
        gCurrentTableLookupIndex[i] = gCalibration.calibrationTableIndexA[i];
    }

    // FIXME: This is placed here to take the place of the original definition of
    //        gCalibration.misalign, which was originally defined as a float.  Now it is redefined
    //        as an int32_t so we could implement this as fixed-point math.  REMOVE AFTER IMPLEMENTATION!
    // 7.450580596923828e-09 = 1/2^27
    for( i = 0; i < 18; i++ ) {
        gAlgorithm.tempMisalign[i] = (float)gCalibration.misalign[i] * 7.450580596923828e-09;
    }
//    // Compute the misalignment from the values loaded into EEPROM
//    int32_t misalign_hex[18];
//    for( i = 0; i < 18; i++ )
//    {
//        // Convert the data stored in memory into floats
//        memcpy( &misalign_hex[i], &gCalibration.misalign[i], sizeof( uint32_t ) );
//        gCalibration.misalign[i] = INT32_TO_MISALIGN_SCALING * (float)misalign_hex[i];
//    }
}

// borrowing the AnalogFilterClocks from the configuration to allow
// the filtering to be changed on the fly.
uint32_t _GetFilter(uint32_t counts)
{
    if (counts > 18749 ) {
        return IIR_02HZ_LPF;
    } else if ( (counts <= 18749) && (counts > 8034) ) {
        return IIR_05HZ_LPF;
    } else if ( (counts <= 8034) && (counts > 4017) ) {
        return IIR_10HZ_LPF;
    } else if ( (counts <= 4017) && (counts > 2410) ) {
        return IIR_20HZ_LPF;
    } else if ( (counts <= 2410) && (counts > 1740) ) {
        return IIR_25HZ_LPF;
    } else if ( (counts <= 1740) && (counts > 1204) ) {
        return IIR_40HZ_LPF;
    } else if ( (counts <= 1204) && (counts > 0) ) {
        return IIR_50HZ_LPF;
    } else if (counts == 0) {
        return UNFILTERED;
    } else {
        return 0;  // never hit this, just here to remove compiler warning
    }
}


/** ***************************************************************************
 * @name CalibrateFilter() apply selected DSP filter to raw sensor counts.
 * Moved to just after data collection to avoid applying calibration to
            unfiltered data.
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CalibrateFilter(void)
{
    uint8_t sensor = 0;

    // gAlgorithm.rawSensors is uint32_t (the data stored in x)
    static uint32_t  fir_5Hz_x [NUM_SENSOR_READINGS][24]; // [11][24] raw + delay data
    static uint32_t  fir_10Hz_x[NUM_SENSOR_READINGS][12]; // [11][12]
    static uint32_t  fir_20Hz_x[NUM_SENSOR_READINGS] [6]; // [11][6]
    static uint32_t  fir_40Hz_x[NUM_SENSOR_READINGS] [3]; // [11][4]

    if( getUserCommunicationType() == SPI_COMM ) {
        /// Filter the data based on the system configuration
        switch( gUserSpi.DataRegister[SPI_REG_FILTER_TAPS_CONFIG_READ] )
        {   /// Filter selection based on SPI data-register
            case UNFILTERED:
              // Do nothing
                break;
            case FIR_40HZ_LPF: // Bartlett
                for( sensor = XACCEL; sensor <= ZMAG; sensor++ ) {
                    Apply_Bartlett_Q27_Filter( &firTaps_40_Hz,
                                               sensor,
                                               fir_40Hz_x[sensor] );
                }
                break;
            case FIR_20HZ_LPF: // Bartlett
                for( sensor = XACCEL; sensor <= ZMAG; sensor++ ) {
                    Apply_Bartlett_Q27_Filter( &firTaps_20_Hz,
                                               sensor,
                                               fir_20Hz_x[sensor] );
                }
                break;
            case FIR_10HZ_LPF: // Bartlett
                for( sensor = XACCEL; sensor <= ZMAG; sensor++ ) {
                    Apply_Bartlett_Q27_Filter( &firTaps_10_Hz,
                                               sensor,
                                               fir_10Hz_x[sensor] );
                }
                break;
            case FIR_05HZ_LPF: // Bartlett
                for( sensor = XACCEL; sensor <= ZMAG; sensor++ ) {
                    Apply_Bartlett_Q27_Filter( &firTaps_5_Hz,
                                               sensor,
                                               fir_5Hz_x[sensor] );
                }
                break;
            case IIR_50HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZMAG; sensor++ ) {
                    Apply_Butterworth_Q27_Filter(&iirTaps_50_Hz,
                                                 sensor);
                }
                break;
            case IIR_20HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZMAG; sensor++ ) {
                    Apply_Butterworth_Q27_Filter(&iirTaps_20_Hz,
                             sensor);
                }
                break;
            case IIR_10HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZMAG; sensor++ ) {
                    Apply_Butterworth_Q27_Filter(&iirTaps_10_Hz,
                             sensor);
                }
                break;
            case IIR_05HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZMAG; sensor++ ) {
                    Apply_Butterworth_Q27_Filter(&iirTaps_5_Hz,
                                                 sensor);
                }
                break;
            default: /// 50 Hz LPF (Butterworth)
                for( sensor = XACCEL; sensor <= ZMAG; sensor++ ) {
                    Apply_Butterworth_Q27_Filter(&iirTaps_50_Hz,
                                                 sensor);
                }
                break;
        }
    } else { /// UART - 50 Hz LPF (Butterworth)
        // Filter accelerometer signal
        switch (_GetFilter(gConfiguration.analogFilterClocks[1])) {
            case IIR_50HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZACCEL; sensor++ )
                    Apply_Butterworth_Q27_Filter(&iirTaps_50_Hz, sensor);
                break;
            case IIR_40HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZACCEL; sensor++ )
                    Apply_Butterworth_Q27_Filter(&iirTaps_40_Hz, sensor);
                break;
            case IIR_25HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZACCEL; sensor++ )
                    Apply_Butterworth_Q27_Filter(&iirTaps_25_Hz, sensor);
                break;
            case IIR_20HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZACCEL; sensor++ )
                    Apply_Butterworth_Q27_Filter(&iirTaps_20_Hz, sensor);
                break;
            case IIR_10HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZACCEL; sensor++ )
                    Apply_Butterworth_Q27_Filter(&iirTaps_10_Hz, sensor);
                break;
            case IIR_05HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZACCEL; sensor++ )
                    Apply_Butterworth_Q27_Filter(&iirTaps_5_Hz, sensor);
                break;
            case IIR_02HZ_LPF:
                for( sensor = XACCEL; sensor <= ZACCEL; sensor++ )
                    Apply_Butterworth_Q27_Filter(&iirTaps_2_Hz, sensor);
                break;
            case UNFILTERED:
                break;
      }

      // Filter rate-sensor signal
      switch (_GetFilter(gConfiguration.analogFilterClocks[2])) {
          case IIR_50HZ_LPF: // Butterworth
              for( sensor = XRATE; sensor <= ZRATE; sensor++ )
                  Apply_Butterworth_Q27_Filter(&iirTaps_50_Hz, sensor);
              break;
          case IIR_40HZ_LPF: // Butterworth
              for( sensor = XRATE; sensor <= ZRATE; sensor++ )
                  Apply_Butterworth_Q27_Filter(&iirTaps_40_Hz, sensor);
              break;
          case IIR_25HZ_LPF: // Butterworth
              for( sensor = XRATE; sensor <= ZRATE; sensor++ )
                  Apply_Butterworth_Q27_Filter(&iirTaps_25_Hz, sensor);
              break;
          case IIR_20HZ_LPF: // Butterworth
              for( sensor = XRATE; sensor <= ZRATE; sensor++ )
                  Apply_Butterworth_Q27_Filter(&iirTaps_20_Hz, sensor);
              break;
          case IIR_10HZ_LPF: // Butterworth
              for( sensor = XRATE; sensor <= ZRATE; sensor++ )
                  Apply_Butterworth_Q27_Filter(&iirTaps_10_Hz, sensor);
              break;
          case IIR_05HZ_LPF: // Butterworth
              for( sensor = XRATE; sensor <= ZRATE; sensor++ )
                  Apply_Butterworth_Q27_Filter(&iirTaps_5_Hz, sensor);
              break;
            case IIR_02HZ_LPF:
                for( sensor = XRATE; sensor <= ZRATE; sensor++ )
                    Apply_Butterworth_Q27_Filter(&iirTaps_2_Hz, sensor);
                break;
          case UNFILTERED:
              break;
        }

        // Filter magnetometer signal
        for( sensor = XMAG; sensor <= ZMAG; sensor++ ) {
            Apply_Butterworth_Q27_Filter(&iirTaps_5_Hz,
                                         sensor);
        }
    }
}

/// @brief
/// ============ DMU380: Coordinate-frame definition ============
///
/// Memsic refers to the heritage (Nav-View) definition
/// JD refers to the OEM coordinate frame (preferred by John Deere)
///   (Verified: February 26, 2013)
///                                          _
///            -z-axis (Memsic)  ^           /| -y-axis (Memsic)
///                +z-axis (JD)  |          /   +x-axis (JD)
///                              |         /
///                              |        /
///                              |       /
///                        ______|______/_________
///                      /       |               /|
///                     /        |              / |
///                    /         |             /  |
///                   /          |            /   |
///  -x-axis (Memsic)/           |           /    /
///  +y-axis (JD)   /                       /    /
///   <------------/                       /    /
///               /     _____________     /    /
///              /    /|            /    /    /
///             /____/ |___________/____/    /
///             |    | /           |    |   /
///             |    |/____________|    |  /
///             |                       | /
///             |_______________________|/
///

/** ***************************************************************************
 * @name CalibrateApply() calculates calibrated sensor variables
 * @brief The input is the raw readings (accels, gyros, mags, temp as described
 *        in eSensorOrder. The gain for each sensor is also passed in.
 *
 * The output is in scaledSensors, the results are in g's, rad/s, G, deg C,
 *  (rotated to user body frame)
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CalibrateApply()
{
    double  scaledSensors[NUM_SENSOR_READINGS]     = { 0.0 }; // [11]
    int32_t scaledSensors_q27[NUM_SENSOR_READINGS] = { 0 }; // fixed point

    // scaled board and accelerometer temperature
    // Tmax = ( X [cnts] - 6400 ) * ( 1/256 [degC/cnt] ) + 25 [degC]
    gAlgorithm.scaledSensors[BTEMP]  = BOARD_TEMP_SCALE_FACTOR * (float)( CONVERT_XBOW_TO_380( gAlgorithm.rawSensors[BTEMP] << 16 ) );
    gAlgorithm.scaledSensors[XATEMP] = gAlgorithm.scaledSensors[BTEMP];
    gAlgorithm.scaledSensors[YATEMP] = gAlgorithm.scaledSensors[BTEMP];
    gAlgorithm.scaledSensors[ZATEMP] = gAlgorithm.scaledSensors[BTEMP];

    // scaled temperature (by 2^27) that has units of [ 10 degC ]
    gAlgorithm.scaledSensors_q27[BTEMP]  = CONVERT_XBOW_TO_380( gAlgorithm.rawSensors[BTEMP] << 16 );
    gAlgorithm.scaledSensors_q27[BTEMP]  = _qmul( BOARD_TEMP_SCALE_FACTOR_q30, gAlgorithm.scaledSensors_q27[BTEMP], 30, 0, 27 );
    gAlgorithm.scaledSensors_q27[XATEMP] = gAlgorithm.scaledSensors_q27[BTEMP];
    gAlgorithm.scaledSensors_q27[YATEMP] = gAlgorithm.scaledSensors_q27[BTEMP];
    gAlgorithm.scaledSensors_q27[ZATEMP] = gAlgorithm.scaledSensors_q27[BTEMP];
    // rate-sensor temperature
    gAlgorithm.scaledSensors[XRTEMP] = MAXIM21000_TEMP_SCALE_FACTOR * (float)( CONVERT_XBOW_TO_380( gAlgorithm.rawSensors[ XRTEMP ] << 2 ) );
    gAlgorithm.scaledSensors[YRTEMP] = gAlgorithm.scaledSensors[XRTEMP];
    gAlgorithm.scaledSensors[ZRTEMP] = gAlgorithm.scaledSensors[XRTEMP];

    // scaled temperature (by 2^27) [ 10 degC ]
    // q27 temp can go to +/-16 [ 10 degC ].  Account for this on the output end...
    // CONVERT_XBOW_TO_380 converts a uint32_t to a int32_t (but the data is only int16_t)
    gAlgorithm.scaledSensors_q27[XRTEMP] = CONVERT_XBOW_TO_380( gAlgorithm.rawSensors[XRTEMP] << 2 );
    gAlgorithm.scaledSensors_q27[XRTEMP] = _qmul( BOARD_TEMP_SCALE_FACTOR_q30, gAlgorithm.scaledSensors_q27[XRTEMP], 30, 0, 27 );
    gAlgorithm.scaledSensors_q27[YRTEMP] = gAlgorithm.scaledSensors_q27[XRTEMP];
    gAlgorithm.scaledSensors_q27[ZRTEMP] = gAlgorithm.scaledSensors_q27[XRTEMP];

    /// temperature-dependent bias correction to rate-sensor and accelerometer
    /// readings. scale factor correction to rate-sensors, acclerometers, and
    /// magnetometers.

    /// -- Accelerometer ( negative sign to get the correct value) --
    scaledSensors[XACCEL] = -( _ApplySensorCalibration( XACCEL, XATEMP ) );
    scaledSensors[YACCEL] = -( _ApplySensorCalibration( YACCEL, YATEMP ) );
    scaledSensors[ZACCEL] = -( _ApplySensorCalibration( ZACCEL, ZATEMP ) );

    // to Fixed point
    scaledSensors_q27[XACCEL] = doubleToQ27( scaledSensors[XACCEL] );
    scaledSensors_q27[YACCEL] = doubleToQ27( scaledSensors[YACCEL] );
    scaledSensors_q27[ZACCEL] = doubleToQ27( scaledSensors[ZACCEL] );

    /// ----- Rate-sensor  -----
    scaledSensors[XRATE] = _ApplySensorCalibration( XRATE, XRTEMP );
    scaledSensors[YRATE] = _ApplySensorCalibration( YRATE, YRTEMP );
    scaledSensors[ZRATE] = _ApplySensorCalibration( ZRATE, ZRTEMP );

    // to Fixed point (output is [ rad/(2^27*sec) ])
    scaledSensors_q27[XRATE] = doubleToQ27( scaledSensors[XRATE] );
    scaledSensors_q27[YRATE] = doubleToQ27( scaledSensors[YRATE] );
    scaledSensors_q27[ZRATE] = doubleToQ27( scaledSensors[ZRATE] );

    /// ----- Magnetometer (scale only no temp comp) -----
    scaledSensors[XMAG] = _ApplySensorCalibration( XMAG, XATEMP );
    scaledSensors[YMAG] = _ApplySensorCalibration( YMAG, YATEMP );
    scaledSensors[ZMAG] = _ApplySensorCalibration( ZMAG, ZATEMP );

    // to Fixed point
    scaledSensors_q27[XMAG] = doubleToQ27( scaledSensors[XMAG] );
    scaledSensors_q27[YMAG] = doubleToQ27( scaledSensors[YMAG] );
    scaledSensors_q27[ZMAG] = doubleToQ27( scaledSensors[ZMAG] );

    // Adjust for misalignment
    //   offsets ACCELEROMETER: = 0, GYRO: = 3, MAGNETOMETER: = 6
    _AdjustForSensorMisalignment( XACCEL, scaledSensors );
    _AdjustForSensorMisalignment( XRATE,  scaledSensors );
    _AdjustForSensorMisalignment( XMAG,   scaledSensors );

    _AdjustForSensorMisalignment_q27( XACCEL, scaledSensors_q27 );
    _AdjustForSensorMisalignment_q27( XRATE,  scaledSensors_q27 );
    _AdjustForSensorMisalignment_q27( XMAG,   scaledSensors_q27 );

    _orientSensors( scaledSensors ); ///< based on faces
    orientSensors_q27( scaledSensors_q27 );
    _LimitSensorValues( scaledSensors ); ///< based on the range of the sensors

    // Populate the structure used in the program (by EKF and output functions)
    //   (Error in previous implementation.  Overwrote data with zeros.  The
    //   following fixes that.)
    memcpy(&gAlgorithm.scaledSensors[0], scaledSensors, 9*sizeof(double) );
    memcpy(&gAlgorithm.scaledSensors_q27[0], scaledSensors_q27, 9*sizeof(int32_t) );
}

/** ***************************************************************************
 * @name _ApplySensorCalibration() LOCAL
 * @brief apply temperature dependant bias and scale factor corrections
 *
 * @param [in] sensor - index
 * @param [in] tempSensor - sensor index
 * @retval N/A
 ******************************************************************************/
static double _ApplySensorCalibration( int sensor,
                                       int tempSensor )
{
    uint32_t tempCompBias;
    int32_t rawReadingMinusBias;

    if( sensor < XMAG ) { // accel and rate
        /// Temperature-dependent bias correction (in raw sensor counts)
        tempCompBias        = _searchTempTable( sensor, gAlgorithm.rawSensors[tempSensor] );
        rawReadingMinusBias = gAlgorithm.rawSensors[sensor] - tempCompBias;

        /// Scale-factor correction
        return IQ27( _searchScaleTable( sensor, rawReadingMinusBias ) );
    } else { /// Mag Scale-factor correction
        return IQ27( _searchScaleTable( sensor, gAlgorithm.rawSensors[sensor] ) );
    }
}

/** ***************************************************************************
 * @name _AdjustForSensorMisalignment() LOCAL
 * @brief apply coodinate misalignment to ortho sensor coordinates
 *
 * @param [in] sensor - index
 * @param [in] scaledSensors - sensor data
 * @retval N/A
 ******************************************************************************/
static void _AdjustForSensorMisalignment( int    sensor,
                                          double *scaledSensors )
{
    double tempX;
    double tempY;
    double tempZ;
    float  misalign[6] = { 0.0 };

    misalign[0] = gAlgorithm.tempMisalign[2 * sensor + 0];
    misalign[1] = gAlgorithm.tempMisalign[2 * sensor + 1];
    misalign[2] = gAlgorithm.tempMisalign[2 * sensor + 2];
    misalign[3] = gAlgorithm.tempMisalign[2 * sensor + 3];
    misalign[4] = gAlgorithm.tempMisalign[2 * sensor + 4];
    misalign[5] = gAlgorithm.tempMisalign[2 * sensor + 5];
    tempX = scaledSensors[sensor    ];
    tempY = scaledSensors[sensor + 1];
    tempZ = scaledSensors[sensor + 2];

    // Matrix time vector (use floating point conversion)
    *( scaledSensors + (sensor    ) ) =                tempX - misalign[0] * tempY - misalign[1] * tempZ;
    *( scaledSensors + (sensor + 1) ) = -misalign[2] * tempX +               tempY - misalign[3] * tempZ;
    *( scaledSensors + (sensor + 2) ) = -misalign[4] * tempX - misalign[5] * tempY +               tempZ;
}

/** ***************************************************************************
 * @name _AdjustForSensorMisalignment_q27() LOCAL Fixed point version
 * @brief apply coodinate misalignment to ortho sensor coordinates
 *
 * @param [in] sensor - index
 * @param [in] scaledSensors - sensor data
 * @retval N/A
 ******************************************************************************/
static void _AdjustForSensorMisalignment_q27(int sensor,
                                             int32_t *scaledSensors_q27 )
{
    int32_t tempX;
    int32_t tempY;
    int32_t tempZ;

    // misalign is a pointer to type iq27 (float)
    int32_t *misalign = &( gCalibration.misalign[2 * sensor] );

    tempX = scaledSensors_q27[sensor  ];
    tempY = scaledSensors_q27[sensor + 1];
    tempZ = scaledSensors_q27[sensor + 2];

    // Matrix time vector
    *( scaledSensors_q27 + (sensor+0) ) =                                    tempX - _qmul( misalign[0], tempY, 27, 27, 27 ) - _qmul( misalign[1], tempZ, 27, 27, 27 );
    *( scaledSensors_q27 + (sensor+1) ) = -_qmul( misalign[2], tempX, 27, 27, 27 )                                   + tempY - _qmul( misalign[3], tempZ, 27, 27, 27 );
    *( scaledSensors_q27 + (sensor+2) ) = -_qmul( misalign[4], tempX, 27, 27, 27 ) - _qmul( misalign[5], tempY, 27, 27, 27 ) + tempZ;
}

// FIXME: remove mod operator since it has been shown to work
#define NO_MOD_OPER 1

/** ***************************************************************************
 * @name _orientSensors() LOCAL rotates the accels, rates, mags, and temps to the
 *       user defined coordinate frame
 * @brief (does not preserve right-handed frame; responsibility of the user?
 *         or is this done in Nav-View or an packet-handler?)
 * @author Darren Liccardo, Aug. 2005
 * @param [in] scaledSensors - sensor data
 * @retval N/A
 ******************************************************************************/
void _orientSensors( double *scaledSensors )
{
   int i;
   int j;
   iq27 temp[NUM_AXIS];

   for( i = 0; i < NUM_SENSOR_IN_AXIS; i += NUM_SENSORS )
   {
      // Store sensor data for next interation of swapping
      for( j = 0; j < NUM_AXIS; j++ ) {
         temp[j] = scaledSensors[i + j];
      }

      // Adjust the sign of the sensor data if the configuration bit is set
      if( gConfiguration.orientation.bit.forwardAxisSign ) {
         scaledSensors[i] = -temp[ gConfiguration.orientation.bit.forwardAxis ];
      } else {
         scaledSensors[i] =  temp[ gConfiguration.orientation.bit.forwardAxis ];
      }

      // Adjust the sign of the sensor data if the configuration bit is set
      uint8_t index;
      // Add switch to get rid of mod-operation (this is run 3x per each pass
      //   through the calibrate routine)
      switch( gConfiguration.orientation.bit.rightAxis ) {
           case 0:
               index = 1;
               break;
           case 1:
               index = 2;
               break;
           case 2:
               index = 0;
               break;
           default:
               // error if configuration is other than 0,1, or 2
               ;
      }

      if(gConfiguration.orientation.bit.rightAxisSign) {
         scaledSensors[i + 1] = -temp[index];
      } else {
         scaledSensors[i + 1] =  temp[index];
      }

      // Add switch to get rid of mod-operation
      switch( gConfiguration.orientation.bit.downAxis ) {
           case 0:
               index = 2;
               break;
           case 1:
               index = 0;
               break;
           case 2:
               index = 1;
               break;
           default:
               // error if configuration is other than 0,1, or 2
               ;
      }

      // Adjust the sign of the sensor data if the configuration bit is set
      if(gConfiguration.orientation.bit.downAxisSign) {
         scaledSensors[i + 2] = -temp[index];
      } else {
         scaledSensors[i + 2] =  temp[index];
      }
   }
}


/** ***************************************************************************
 * @name _orientSensors_q27() LOCAL rotates the accels, rates and mags to the
 *       user defined coordinate frame
 * @brief (does not preserve right-handed frame; responsibility of the user?
 *         or is this done in Nav-View or an packet-handler?)
 * @author Joe Motyka, Aug. 2014
 * @param [in] scaledSensors - sensor data
 * @retval N/A
 ******************************************************************************/
void orientSensors_q27( int32_t *scaledSensors_q27 )
{
   int i,j;
   int32_t temp[NUM_AXIS];

   // Loop through for each inertial sensor
   for( i = 0; i < NUM_SENSOR_IN_AXIS; i += NUM_SENSORS )
   {
      // Store sensor data for next interation of swapping
      for( j = 0; j < NUM_AXIS; j++ ) {
         temp[j] = scaledSensors_q27[i + j];
      }

      // Adjust the sign of the sensor data if the configuration bit is set
      if( gConfiguration.orientation.bit.forwardAxisSign ) {
         scaledSensors_q27[i] = -temp[ gConfiguration.orientation.bit.forwardAxis ];
      } else {
         scaledSensors_q27[i] =  temp[ gConfiguration.orientation.bit.forwardAxis ];
      }

      uint8_t index;
      // Add switch to get rid of mod-operation (this is run 3x per each pass
      //   through the calibrate routine)
      switch( gConfiguration.orientation.bit.rightAxis ) {
           case 0:
               index = 1;
               break;
           case 1:
               index = 2;
               break;
           case 2:
               index = 0;
               break;
           default:
               // error if configuration is other than 0,1, or 2
               ;
      }

      if(gConfiguration.orientation.bit.rightAxisSign) {
         scaledSensors_q27[i+1] = -temp[index];
      } else {
         scaledSensors_q27[i+1] =  temp[index];
      }

      // Add switch to get rid of mod-operation
      switch( gConfiguration.orientation.bit.downAxis ) {
           case 0:
               index = 2;
               break;
           case 1:
               index = 0;
               break;
           case 2:
               index = 1;
               break;
           default:
               // error if configuration is other than 0,1, or 2
               ;
      }

      // Adjust the sign of the sensor data if the configuration bit is set
      if(gConfiguration.orientation.bit.downAxisSign) {
         scaledSensors_q27[i+2] = -temp[index];
      } else {
         scaledSensors_q27[i+2] =  temp[index];
      }
   }
}


/** ***************************************************************************
 * @name _LimitSensorValues()
 * @brief set a flag for SPI data and limit the sensor values based
*         upon the database value - rate limited to 400dps for ITAR
 *
 * @param [in] scaledSensors - sensor data
 * @retval N/A
 ******************************************************************************/
void _LimitSensorValues( double *scaledSensors )
{
    uint8_t temp = 0;

    /// limit rates for ITAR
    /// ---------- Rate Sensor Values ----------
    /// X-Axis
    if ( scaledSensors[ XRATE ] > gAlgorithm.Limit.rateAlarm ) {
        if ( scaledSensors[ XRATE ] > ITAR_RATE_LIMIT ) {
            scaledSensors[ XRATE ] = ITAR_RATE_LIMIT;
        }
        temp = temp || 1;
    } else if ( scaledSensors[ XRATE ] < -gAlgorithm.Limit.rateAlarm ) {
        if ( scaledSensors[ XRATE ] < -ITAR_RATE_LIMIT ) {
            scaledSensors[ XRATE ] = -ITAR_RATE_LIMIT;
        }
        temp = temp || 1;
    }

    /// Y-Axis
    if ( scaledSensors[ YRATE ] > gAlgorithm.Limit.rateAlarm ) {
        if ( scaledSensors[ YRATE ] > ITAR_RATE_LIMIT ) {
            scaledSensors[ YRATE ] =  ITAR_RATE_LIMIT;
        }
        temp = temp || 1;
    } else if ( scaledSensors[ YRATE ] < -gAlgorithm.Limit.rateAlarm )  {
        if ( scaledSensors[ YRATE ] < -ITAR_RATE_LIMIT ) {
            scaledSensors[ YRATE ] = -ITAR_RATE_LIMIT;
        }
        temp = temp || 1;
    }

    /// Z-Axis
    if ( scaledSensors[ ZRATE ] > gAlgorithm.Limit.rateAlarm ) {
        if ( scaledSensors[ ZRATE ] > ITAR_RATE_LIMIT ) {
            scaledSensors[ ZRATE ] =   ITAR_RATE_LIMIT;
        }
        temp = temp || 1;
    } else if ( scaledSensors[ ZRATE ] < -gAlgorithm.Limit.rateAlarm ) {
        if ( scaledSensors[ ZRATE ] < -ITAR_RATE_LIMIT ) {
            scaledSensors[ ZRATE ] =  -ITAR_RATE_LIMIT;
        }
        temp = temp || 1;
    }

    if( getUserCommunicationType() == SPI_COMM )
    {    /// limits apply for the SPI implementation of the software
        ///   - FLAG the data
        /// ---------- Accelerometer Values ----------
        /// X-Axis
        if( ( scaledSensors[ XACCEL ] > gAlgorithm.Limit.accelAlarm ) ||
            ( scaledSensors[ XACCEL ] < -gAlgorithm.Limit.accelAlarm ) ) {
            temp = temp || 1;
        }
        /// Y-Axis
        if( ( scaledSensors[ YACCEL ] > gAlgorithm.Limit.accelAlarm ) ||
            ( scaledSensors[ YACCEL ] < -gAlgorithm.Limit.accelAlarm ) ) {
            temp = temp || 1;
        }
        /// Z-Axis
        if( ( scaledSensors[ ZACCEL ] > gAlgorithm.Limit.accelAlarm ) ||
            ( scaledSensors[ ZACCEL ] < -gAlgorithm.Limit.accelAlarm ) ) {
            temp = temp || 1;
        }

        /// If a rate-sensor or accelerometer exceed the defined limit then set
        ///   the over-range bit in the diagnostic register.
        if( temp != 0 ) {
            gUserSpi.DataRegister[ 0x3D ] = gUserSpi.DataRegister[ 0x3D ] | 0x10;
            gAlgorithm.bitStatus.swAlgBIT.bit.overRange = 1;
        } else {
            // Clear the bit
            gUserSpi.DataRegister[ 0x3D ] = gUserSpi.DataRegister[ 0x3D ] & 0xEF;
            gAlgorithm.bitStatus.swAlgBIT.bit.overRange = 0;
        }
        /// No saturation for Magnetometers
    }
    else
    {
        // UART implementation (Nav-View) of the software
    }
}

