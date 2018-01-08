/** ***************************************************************************
 * @file   UserCommunication_SPI.c
 * @Author
 * @date   September, 2013
 * @copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 * Set up client SPI3 DMA and register handler
 ******************************************************************************/
#include <salvo.h>
#include <math.h> // pow()

#include "stm32f2xx.h"
#include "stm32f2xx_spi.h"
#include "boardDefinition.h"
#include "spi.h"
#include "timer.h"
#include "UserCommunication_SPI.h"
#include "debug.h"
#include "s_eeprom.h"
#include "bitSelfTest.h" // For the self-test functions
#include "Indices.h"
#include "gyroscope.h"
#include "accelerometer.h"
#include "xbowsp_init.h"
#include "scaling.h" //  RADS_TO_DEGREES

#include <string.h>
#include "configureGPIO.h"

#include "filter.h"

volatile UserSpiStruct gUserSpi;

// local functions
void _InitStructureVariables_UserSpi( void );
void _FillSpiDataRegister_Unassigned01( void );
void _FillSpiDataRegister_ProductIdentification( void );
void _FillSpiDataRegister_Configuration( void );
void _FillSpiDataRegister_Filter( void );
void _FillSpiDataRegister_Sampling( void );
void _FillSpiDataRegister_DataReady( void );
void _FillSpiDataRegister_RateSensorOutput( void );
void _FillSpiDataRegister_HWandSWVersion( void );
void _FillSpiDataRegister_Orientation( void );

// Compute the CRC for the burst register
uint16_t    _calcCRC( volatile uint8_t *buf, uint16_t num );
// so the pointer is not NULL to pass to spi_configure
static void _EmptyCallback( void ){ }

/** ***************************************************************************
 * @name InitCommunication_UserSPI()
 * @brief Initialize the customer (user - external) communication SPI3 line
 *    run bus at 200 Hz whether an external sync signal is provided or not (if
 *    an external (1kHz) is provided run every 5th tick TICKS_FOR_200_HZ
 * @param N/A
 * @retval 1 success
 ******************************************************************************/
uint8_t InitCommunication_UserSPI( void )
{
    uint8_t error;

    /// ------ Set up the SPI3 IO pins, protocol, and interrupt ------
    error = spi_configure( SPI3, // kUserCommunicationSPI
                           SPI_CPOL_AND_CPHA_HIGH,
                           &_EmptyCallback );
    if( error ) {
        return error;
    }

    _InitStructureVariables_UserSpi();

    return 1 ;
}

// FIXME: USED????
/** ***************************************************************************
 * @name HandleRegisterRead_UserSPI()
 * @brief Used to reset data registers upon read (if necessary - for example,
 *        SPI-error register)
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void HandleRegisterRead_UserSPI( void )
{
    if ( gUserSpi.registerAddr == SPI_REG_SPI_ERROR) // 0x02
    {   /// reset after read
        gUserSpi.DataRegister[ SPI_REG_SPI_ERROR ] = 0x00;
    }
}

/** ***************************************************************************
 * @name HandleRegisterWrite_UserSPI() Calls the MOSI "write" callback handler
 *       using the register address as the array index for the callback fcn.
 * @brief Used to configure the burst read size, crc and sensor data inclusion
 * in the burst message. Send a dynamically set config here on startup
 * @param [in] registerAddress - index in the UserSpiStruct.DataRegister[]
 * @param [in] writeValue - value to proces for the register
 * @retval 1 success
 ******************************************************************************/
void HandleRegisterWrite_UserSPI( uint8_t registerAddress,
                                  uint8_t writeValue )
{
    (*(gSpiWrite[registerAddress].callback))(writeValue);
}

/** ***************************************************************************
 * @name _InitStructureVariables_UserSpi()
 * @brief Initialize the gUserSpi structure variables here (this function called
 *        in InitCommunication_UserSPI()) Nominal data register is 128 bytes
 *        long (address 0x00 to 0x7F)
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void _InitStructureVariables_UserSpi()
{
    // clear then only add non-zero data
    memset((void*)&gUserSpi, 0, sizeof(gUserSpi));

    /// @brief filled with the complete set of data regardless of the desired
    /// output (set vias the rate, accelerometer, ... configuration bytes).
    _FillSpiDataRegister_Unassigned01();           ///< Registers 0x00 to 0x01

    // FIXME: normally zero after init but debugging changes this value
    // gUserSpi.DataRegister[ SPI_REG_SPI_ERROR ] = 0x00;
    FillSpiDataRegister_OutputBytesExpected(16);    ///< Register 0x03
    FillSpiDataRegister_Sensors();                  ///< Registers 0x04 to 0x19
    _FillSpiDataRegister_DataReady();
    _FillSpiDataRegister_Sampling();                ///< Registers 0x36 to 0x37
//    _FillSpiDataRegister_RateSensorOutput();        ///< Registers 0x38 to 0x39  (not useful as overwritten by config)
    FillSpiDataRegister_MagAlign();
    _FillSpiDataRegister_ProductIdentification();   ///< Registers 0x52 to 0x5B
    _FillSpiDataRegister_Configuration();           ///< Registers 0x5C to 0x7F (sets output configuration)
    _FillSpiDataRegister_Filter();                  ///< Registers 0x5C to 0x7F (sets output configuration)
    _FillSpiDataRegister_HWandSWVersion();          ///< Registers 0x5C to 0x7F
    _FillSpiDataRegister_Orientation();             ///< Registers 0x38 to 0x39

    UserSPI_BytesInBurstRegister();
// DEBUG - set a value to be read in ISS read message
gUserSpi.DataRegister[0x6f] = 0xa5;
}

/** ***************************************************************************
 * @name _FillSpiDataRegister_Unassigned01()
 * @brief initialize the Status bytes the first two registers (bytes)
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void _FillSpiDataRegister_Unassigned01( void )
{   // ----- Status bytes -----
    gUserSpi.DataRegister[ 0x00 ] = 0x00;
    gUserSpi.DataRegister[ 0x01 ] = 0x0C;
}

/** ***************************************************************************
 * @name FillSpiDataRegister_OutputBytesExpected()
 * @brief Called after the burst-register configuration changes are made
 * @param [in] NumberOfBytesExpected - size of a burst (not used)
 * @retval N/A
 ******************************************************************************/
void FillSpiDataRegister_OutputBytesExpected( uint8_t NumberOfBytesExpected )
{
    gUserSpi.DataRegister[ SPI_REG_NUM_BYTES_BURST ] = 0x04;
}

/** ***************************************************************************
 * @name _loadRegisters()
 * @brief convert float values from the gAgorithm data structure scaled and cast
 *        to to an short then split to bytes to load a pair of SPI "registers"
 *        after data collection.
 *
 * @param [in] dataIn floating point data to be processed
 * @param [in] floatLimit input data limit
 * @param [in] scale factor to scale the input value to a signed short
 * @param [in] buffer - the SPI register to be filled
 * @param [in] index - where in the spi register to put the data
 * @retval N/A
 ******************************************************************************/
void _loadRegisters( float   dataIn,
                     double  floatLimit,
                     double  scaleFactor,
                     uint8_t volatile *buffer,
                     uint8_t index )
{
    static uint16_t UpperLimit = 32760;
    double          tempFloat  = 0.0;
    int             sign;
    static int16_t  tempInt16  = 0x0000;

    tempFloat = dataIn;
    if( tempFloat > floatLimit ) {
        tempFloat = floatLimit;
    } else if( tempFloat < -floatLimit ) {
        tempFloat = -floatLimit;
    }

    if( tempFloat >= 0 ) {
      sign = 1;
    } else {
      sign = -1;
    }

    // now tempInt16 can be positive:
    tempInt16 = ( int )( sign * tempFloat * scaleFactor );
    if( tempInt16 > UpperLimit ) {
        tempInt16 = UpperLimit;
    }
    tempInt16 = sign * tempInt16;

    buffer[ index ]     = ( tempInt16 & 0xFF00 ) >> 8;
    buffer[ index + 1 ] = ( tempInt16 & 0x00FF ) >> 0;
}

/** ***************************************************************************
 * @name FillSpiDataRegister_Sensors()
 * @brief Called in TaskDataAcquisition(), after data collection and
 *        processing is complete, load the sensor data into the data register
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void FillSpiDataRegister_Sensors( void )
{
    /// ----- Rate sensor output (bytes 0x04 to 0x09) -----
    /// Sensor range = 250 deg/sec; Output range = +/- 250 deg/sec
    /// NOTE: gAlgorithm.scaledSensors[ RATE ] is in [rad/sec]
    /// === X-Axis ===
    _loadRegisters( gAlgorithm.scaledSensors[ XRATE ], // dataIn [rad/sec]
                    gUserSpi.floatLimit_Rate,          // floatLimit [rad/sec]
                    gAlgorithm.RateOutSF,              // scaleFactor
                    gUserSpi.DataRegister,             // *buffer
                    SPI_REG_RATE_X_MSB );              // index

    /// === Y-Axis ===
    _loadRegisters( gAlgorithm.scaledSensors[ YRATE ],
                    gUserSpi.floatLimit_Rate,
                    gAlgorithm.RateOutSF,
                    gUserSpi.DataRegister,
                    SPI_REG_RATE_Y_MSB );

    /// === Z-Axis ===
    _loadRegisters( gAlgorithm.scaledSensors[ ZRATE ],
                    gUserSpi.floatLimit_Rate,
                    gAlgorithm.RateOutSF,
                    gUserSpi.DataRegister,
                    SPI_REG_RATE_Z_MSB);
    // ----- End of rate sensor output (bytes 0x04 to 0x09) -----

    /// ----- Accelerometer output (bytes 0x0A to 0x0F) -----
    /// Sensor range = 8g; Output range = +/- 5g
    /// === X-Axis ===
    _loadRegisters( gAlgorithm.scaledSensors[ XACCEL ],
                    gUserSpi.floatLimit_Accel,
                    gAlgorithm.AccelOutSF,
                    gUserSpi.DataRegister,
                    SPI_REG_ACCEL_X_MSB );

    /// === Y-Axis ===
    _loadRegisters( gAlgorithm.scaledSensors[ YACCEL ],
                    gUserSpi.floatLimit_Accel,
                    gAlgorithm.AccelOutSF,
                    gUserSpi.DataRegister,
                    SPI_REG_ACCEL_Y_MSB );

    // === Z-Axis ===
    _loadRegisters( gAlgorithm.scaledSensors[ ZACCEL ],
                    gUserSpi.floatLimit_Accel,
                    gAlgorithm.AccelOutSF,
                    gUserSpi.DataRegister,
                    SPI_REG_ACCEL_Z_MSB );
    // ----- End of accelerometer output (bytes 0x0A to 0x0F) -----

    /// ----- Magnetometer output (bytes 0x10 to 0x15) -----
    /// Sensor range = 8G; Output range = +/- 1G
    /// === X-Axis ===
    _loadRegisters( gAlgorithm.scaledSensors[ XMAG ],
                    gUserSpi.floatLimit_Mag,
                    gAlgorithm.MagOutSF,
                    gUserSpi.DataRegister,
                    SPI_REG_MAG_X_MSB );

    _loadRegisters( gAlgorithm.scaledSensors[ YMAG ],
                    gUserSpi.floatLimit_Mag,
                    gAlgorithm.MagOutSF,
                    gUserSpi.DataRegister,
                    SPI_REG_MAG_Y_MSB );

    /// === Z-Axis ===
    _loadRegisters( gAlgorithm.scaledSensors[ ZMAG ],
                    gUserSpi.floatLimit_Mag,
                    gAlgorithm.MagOutSF,
                    gUserSpi.DataRegister,
                    SPI_REG_MAG_Z_MSB );
    // ----- End of magnetometer output (bytes 0x10 to 0x15) -----

    /// ----- Temperature output (bytes 0x16 to 0x19) -----
    /// Rate sensor temperature
    _loadRegisters( gAlgorithm.scaledSensors[ XRTEMP ],
                    100.0,
                    gAlgorithm.TempOutSF,
                    gUserSpi.DataRegister,
                    SPI_REG_RATE_TEMP_MSB );

    /// Board temperature (included in burst-register)
    _loadRegisters( gAlgorithm.scaledSensors[ BTEMP ]  - 31.0,
                    100.0, // The needs to be out of the range of the sensor output
                    gAlgorithm.TempOutSF,
                    gUserSpi.DataRegister,
                    SPI_REG_BOARD_TEMP_MSB );
    // ----- End of temperature output (bytes 0x16 to 0x19) -----
}


/** ***************************************************************************
 * @name FillSpiDataRegister_BIT()
 * @brief Called in TaskDataAcquisition(), after data collection and
 *        processing is complete, load the BIT data into the data registes
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void FillSpiDataRegister_BIT( void )
{
    /// Master BIT Top level flags
    gUserSpi.DataRegister[SPI_BIT_MASTER_STATUS_READ] = gAlgorithm.bitStatus.BITStatus.all;

    /// HW status Top level flags
    gUserSpi.DataRegister[SPI_BIT_HW_STATUS_READ] = gAlgorithm.bitStatus.hwBIT.all;

    /// SW status Top level flags
    gUserSpi.DataRegister[SPI_BIT_SW_MASTER_READ] = gAlgorithm.bitStatus.swBIT.all;

    /// SW status flags
    gUserSpi.DataRegister[SPI_BIT_SW_STATUS_READ] = gAlgorithm.bitStatus.swStatus.all;

    /// SW Alg flags
    gUserSpi.DataRegister[SPI_BIT_SW_ALG_READ] = gAlgorithm.bitStatus.swAlgBIT.all;

    /// SW data flags
    gUserSpi.DataRegister[SPI_BIT_SW_DATA_READ] = gAlgorithm.bitStatus.swDataBIT.all;

    /// COMM master data flags
    gUserSpi.DataRegister[SPI_BIT_COMM_MASTER_READ] = gAlgorithm.bitStatus.comBIT.all;

    /// COMM status data flags
    gUserSpi.DataRegister[SPI_BIT_COMM_DATA_STATUS_READ] = gAlgorithm.bitStatus.comStatus.all;

    /// COMM serial A bus flags
    gUserSpi.DataRegister[SPI_BIT_COMM_BUS_A_READ] = gAlgorithm.bitStatus.comSABIT.all;

    /// COMM serial B bus flags
    gUserSpi.DataRegister[SPI_BIT_COMM_BUS_B_READ] = gAlgorithm.bitStatus.comSBBIT.all;

    /// Sensor flags
    gUserSpi.DataRegister[SPI_BIT_SENSOR_STATUS_READ] = gAlgorithm.bitStatus.sensorStatus.all;
}
/** ***************************************************************************
 * @name FillSpiDataRegister_MagAlign()
 * @brief Called in MagAlign(), after completion of alignment procedure and
 *        processing is complete, load the compesation data into the data registes
 * @param [in] N/A
 * @retval N/A
 * @details loaded at init and after a align command
 ******************************************************************************/
void FillSpiDataRegister_MagAlign( void )
{
    gUserSpi.DataRegister[SPI_HARD_IRON_BIAS_0_READ]     = (gConfiguration.hardIronBias[0] & 0xff00) >> 8;
    gUserSpi.DataRegister[SPI_HARD_IRON_BIAS_0_READ + 1] = gConfiguration.hardIronBias[0] & 0xff;

    gUserSpi.DataRegister[SPI_HARD_IRON_BIAS_1_READ]     = (gConfiguration.hardIronBias[1] & 0xff00) >> 8;
    gUserSpi.DataRegister[SPI_HARD_IRON_BIAS_1_READ + 1] = gConfiguration.hardIronBias[1] & 0xff;

    gUserSpi.DataRegister[SPI_SOFT_IRON_SCALE_READ]      = (gConfiguration.softIronScaleRatio & 0xff00) >> 8;
    gUserSpi.DataRegister[SPI_SOFT_IRON_SCALE_READ + 1]  = gConfiguration.softIronScaleRatio &0xff;

    gUserSpi.DataRegister[SPI_SOFT_IRON_ANGLE_READ]      = (gConfiguration.softIronAngle & 0xff00) >> 8;
    gUserSpi.DataRegister[SPI_SOFT_IRON_ANGLE_READ + 1]  = gConfiguration.softIronAngle &0xff;
}

/** ***************************************************************************
 * @name FillSpiBurstRegister_Sensors() load the burst registers prior to
 *       transmission
 * @brief The burst-register is sized to hold system status (2 bytes), x/y/z rates
 *       (6), x/y/z accel (6), x/y/z mags (6), gyro/board temperature (4), and
 *       the CRC of the preceeding items (2). This output array is configurable
 *       via the configuration registers for a maximum size of 24 bytes. The
 *       default will be the above items excluding the mags and rate-sensor
 *      temperature for a default size of 18 bytes.
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void FillSpiBurstRegister_Sensors()
{
    uint8_t index = 0;
    // load the system status
    gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_SYSTEM_STATUS_MSB ];
    gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_SYSTEM_STATUS_LSB ];

    /// Sensors - rate
    if( gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] & INERTIAL_SENSOR_ENABLE ) {
        if( gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] & INERTIAL_SENSOR_ENABLE_X_AXIS ) {
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_RATE_X_MSB ];
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_RATE_X_LSB ];
        }

        if( gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] & INERTIAL_SENSOR_ENABLE_Y_AXIS ) {
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_RATE_Y_MSB ];
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_RATE_Y_LSB ];
        }

        if( gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] & INERTIAL_SENSOR_ENABLE_Z_AXIS ) {
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_RATE_Z_MSB ];
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_RATE_Z_LSB ];
        }
    }

    /// Sensors - accel
    if( gUserSpi.DataRegister[ SPI_REG_ACCEL_OUTPUT_CONFIG_READ ] & INERTIAL_SENSOR_ENABLE ) {
        if( gUserSpi.DataRegister[ SPI_REG_ACCEL_OUTPUT_CONFIG_READ ] & INERTIAL_SENSOR_ENABLE_X_AXIS ) {
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_ACCEL_X_MSB ];
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_ACCEL_X_LSB ];
        }

        if( gUserSpi.DataRegister[ SPI_REG_ACCEL_OUTPUT_CONFIG_READ ] & INERTIAL_SENSOR_ENABLE_Y_AXIS ) {
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_ACCEL_Y_MSB ];
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_ACCEL_Y_LSB ];
        }

        if( gUserSpi.DataRegister[ SPI_REG_ACCEL_OUTPUT_CONFIG_READ ] & INERTIAL_SENSOR_ENABLE_Z_AXIS ) {
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_ACCEL_Z_MSB ];
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_ACCEL_Z_LSB ];
        }
    }

    /// Sensors - mag
    if( gUserSpi.DataRegister[ SPI_REG_MAG_OUTPUT_CONFIG_READ ] & INERTIAL_SENSOR_ENABLE ) {
        if( gUserSpi.DataRegister[ SPI_REG_MAG_OUTPUT_CONFIG_READ ] & INERTIAL_SENSOR_ENABLE_X_AXIS ) {
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_MAG_X_MSB ];
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_MAG_X_LSB ];
        }

        if( gUserSpi.DataRegister[ SPI_REG_MAG_OUTPUT_CONFIG_READ ] & INERTIAL_SENSOR_ENABLE_Y_AXIS ) {
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_MAG_Y_MSB ];
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_MAG_Y_LSB ];
        }

        if( gUserSpi.DataRegister[ SPI_REG_MAG_OUTPUT_CONFIG_READ ] & INERTIAL_SENSOR_ENABLE_Z_AXIS ) {
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_MAG_Z_MSB ];
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_MAG_Z_LSB ];
        }
    }

    /// Rate-sensor temperature (rate-sensor must be enabled to get this data)
    if( gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] & INERTIAL_SENSOR_ENABLE ) {
        if( gUserSpi.DataRegister[ SPI_REG_TEMP_OUTPUT_CONFIG_READ ] & RATE_TEMP_SENSOR_ENABLE ) {
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_RATE_TEMP_MSB ];
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_RATE_TEMP_LSB ];
        }
    }

    /// Board temperature
    // FIXME: Board temp sensor must be enabled to get this data!
    if( gUserSpi.DataRegister[ SPI_REG_TEMP_OUTPUT_CONFIG_READ ] & TEMP_SENSOR_ENABLE ) {
        if( gUserSpi.DataRegister[ SPI_REG_TEMP_OUTPUT_CONFIG_READ ] & BOARD_TEMP_SENSOR_ENABLE ) {
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_BOARD_TEMP_MSB ];
            gUserSpi.BurstRegister[index++] = gUserSpi.DataRegister[ SPI_REG_BOARD_TEMP_LSB ];
        }
    }

    uint16_t temp = _calcCRC( gUserSpi.BurstRegister, index );

    /// The last two elements of the array is a CRC based on the above
    gUserSpi.BurstRegister[index++] = ( temp & 0xFF00 ) >> 8;
    gUserSpi.BurstRegister[index++] = ( temp & 0x00FF ) >> 0;
}

/** ***************************************************************************
 * @name _calcCRC()
 * @brief Calculate the 16-bit CRC for the burst register
 *
 * @param [in] buf - input buffer to calculate the CRC
 * @param [in] num - size of the buffer
 * @retval CRC
 ******************************************************************************/
uint16_t _calcCRC( volatile uint8_t *buf,
                   uint16_t         num )
{
    int      i;
    int      j;
    /// non-augmented inital value = augmented initial value 0xFFFF
    uint16_t crc = 0x1D0F;

    for( i = 0; i < num; i++ ) {
        crc ^= buf[i] << 8;

        for( j = 0; j < 8; j++ ) {
            if(crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}


/** ***************************************************************************
 * @name _FillSpiDataRegister_DataReady()
 * @brief Set the defaults: data-ready enabled; high when data is ready; output
 *        on pin X this is written like this to avoid a compiler warning
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void _FillSpiDataRegister_DataReady( void )
{
    uint8_t tempUInt8 = 0;

    gUserSpi.EnableDataReady   = 1;    ///< 0x04
    gUserSpi.DataReadyPolarity = 0;    ///< 0x02
    gUserSpi.DataReadyPin      = 0;    ///< 0x01

    tempUInt8  =  0x04 * gUserSpi.EnableDataReady;
    tempUInt8 +=  0x02 * gUserSpi.DataReadyPolarity;
    tempUInt8 +=  0x01 * gUserSpi.DataReadyPin;
    gUserSpi.DataRegister[ SPI_REG_DATA_READY_READ ] = tempUInt8;
}

/** ***************************************************************************
 * @name _FillSpiDataRegister_Sampling()
 * @brief initialize SPI data registers 0x36 and 0x37 timing set up with default
 *        values
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void _FillSpiDataRegister_Sampling( void )
{
    gUserSpi.DataRegister[ SPI_REG_DECIMATION_READ ]   = 0x01; ///< Decimation = 1: 200 Hz
    gUserSpi.DataRegister[ SPI_REG_CLOCK_CONFIG_READ ] = 0x01; ///< Internal clock

    /// Set via the decimation register
    gUserSpi.outputDataRate  = 1;
    gUserSpi.dataRateCounter = 0;
}

#include "ucb_packet.h"

/** ***************************************************************************
 * @name _FillSpiDataRegister_RateSensorOutput()
 * @brief initialize rate sensor configuration registers 0x38 and 0x39 with
 *        default values
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void _FillSpiDataRegister_RateSensorOutput( void )
{
    //uint8_t sysRange = ; // from system config
    switch (UcbGetSysRange()) {
        case _200_DPS_RANGE: // same as default
            // output range (+/-250 deg/sec)
            gUserSpi.DataRegister[SPI_REG_DYNAMIC_RANGE_CONFIG_READ] = 0x04;

            // Set limit and scale factor based on register 0x38
            gAlgorithm.Limit.rateAlarm = 200.00 * DEGREES_TO_RADS;  // 200.0 [deg/sec] = 3.4907 [rad/sec]
            gUserSpi.floatLimit_Rate   = 220.00 * DEGREES_TO_RADS;  // 200.0 [deg/sec] = 3.4907 [rad/sec]

            gAlgorithm.RateOutSF       = 100.00 * RADS_TO_DEGREES;

            /// changes to rate-sensor configuration register (0x70 = 112)
            gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] = 0x2F;
            break;

        case _400_DPS_RANGE:
            // output range (+/-500 deg/sec)
            gUserSpi.DataRegister[SPI_REG_DYNAMIC_RANGE_CONFIG_READ] = 0x08;

            // Set limit and scale factor based on register 0x38
            gAlgorithm.Limit.rateAlarm = 400.00 * DEGREES_TO_RADS;  // 400.0 [deg/sec] = 6.9813 [rad/sec]
            gUserSpi.floatLimit_Rate   = 440.00 * DEGREES_TO_RADS;  // 200.0 [deg/sec] = 3.4907 [rad/sec]

            gAlgorithm.RateOutSF       =  50.00 * RADS_TO_DEGREES;

            /// changes to rate-sensor configuration register (0x70 = 112)
            gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] = 0x3F;
            break;

        case _1000_DPS_RANGE:
            // output range (+/-1000 deg/sec)
            gUserSpi.DataRegister[SPI_REG_DYNAMIC_RANGE_CONFIG_READ] = 0x10;

            // Set limit and scale factor based on register 0x38
            gAlgorithm.Limit.rateAlarm = 600.00 * DEGREES_TO_RADS;  // 600 [deg/sec] = 10.4720 [rad/sec]
            gUserSpi.floatLimit_Rate   = 660.00 * DEGREES_TO_RADS;  // // 1000 dps

            gAlgorithm.RateOutSF       =  25.00 * RADS_TO_DEGREES;

            /// changes to rate-sensor configuration register (0x70 = 112)
            gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] = 0x4F;
            break;

        default:
            gCalibration.AccelSensorRange = ACCEL_RANGE_4G;
            gCalibration.GyroSensorRange  = GYRO_RANGE_250DPS;
    }
}


/** ***************************************************************************
 * @name _FillSpiDataRegister_ProductIdentification()
 * @brief Initialize the product ID registers 0x52 - 0x59 with default values
 * The serial number is generated at product configuration and loaded once.
 * register loaded from EEPROM at startup does not change during operation.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void _FillSpiDataRegister_ProductIdentification( void )
{
    static uint8_t w1;
    static uint8_t w2;
    static uint8_t w3;
    static uint8_t w4;
    static uint8_t x1;
    static uint8_t x2;
    static uint8_t x3;
    static uint8_t y1;
    static uint8_t y2;
    static uint8_t y3;
    uint32_t       SN;

    /// get the serial number
    readEEPROMSerialNumber((void *)&gCalibration.serialNumber);
    SN = gCalibration.serialNumber;

    // FIXME: this doesn't look like it works
    /// Pull out the serial number digits
    w4 = ( SN/1000000000 );
    w3 = ( SN/100000000  ) - ( SN/1000000000 )*10;
    w2 = ( SN/10000000   ) - ( SN/100000000  )*10;
    w1 = ( SN/1000000    ) - ( SN/10000000   )*10;
    x3 = ( SN/100000     ) - ( SN/1000000    )*10;
    x2 = ( SN/10000      ) - ( SN/100000     )*10;
    x1 = ( SN/1000       ) - ( SN/10000      )*10;
    y3 = ( SN/100        ) - ( SN/1000       )*10;
    y2 = ( SN/10         ) - ( SN/100        )*10;
    y1 = ( SN/1          ) - ( SN/10         )*10;

    /// Load lot ID 1
    gUserSpi.DataRegister[ 0x52 ] = w4 * 0x10 + w3 * 0x01; // 0x52
    gUserSpi.DataRegister[ 0x53 ] = w2 * 0x10 + w1 * 0x01;
    /// Load lot ID 2
    gUserSpi.DataRegister[ 0x54 ] =             x3 * 0x01;
    gUserSpi.DataRegister[ 0x55 ] = x2 * 0x10 + x1 * 0x01;
    /// ----- Who am I (Memsic specific Product ID: 0x3810 = 14352) -----
    gUserSpi.DataRegister[ 0x56 ] = 0x38;
    gUserSpi.DataRegister[ 0x57 ] = 0x10;
    /// Serial number (last four digits of the serial number that Nav-View uses)
    gUserSpi.DataRegister[ 0x58 ] =             y3 * 0x01;
    gUserSpi.DataRegister[ 0x59 ] = y2 * 0x10 + y1 * 0x01;
}

#include "SPIWriteCallbackTable.h"

/** ***************************************************************************
 * @name _FillSpiDataRegister_Configuration()
 * @brief Initialize the sensor configuration registers 0x5C - 0x7F with default
 *        values
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void _FillSpiDataRegister_Configuration( void )
{
    /// -------- Configure sensors --------
    /// ----- Rate sensor configuration -----
    switch (UcbGetSysRange()) {
        case _200_DPS_RANGE: // same as default
            Rate_Output_Config_Write(0x2F);
            break;

        case _400_DPS_RANGE:
            Rate_Output_Config_Write(0x3F);
            break;

        case _1000_DPS_RANGE:
            Rate_Output_Config_Write(0x4F);
            break;

        default:
//            gCalibration.AccelSensorRange = ACCEL_RANGE_4G;
//            gCalibration.GyroSensorRange  = GYRO_RANGE_250DPS;
            break;
    }

    /// ----- Accelerometer configuration -----
    switch (UcbGetSysRange()) {
        case _200_DPS_RANGE:
            Accel_Output_Config_Write(0x2F);  // 3.92 [g]
            break;

        case _400_DPS_RANGE:
        case _1000_DPS_RANGE:
            Accel_Output_Config_Write(0x4F);  // +/- 8 [g]
            break;

        default:
//            gCalibration.AccelSensorRange = ACCEL_RANGE_4G;
//            gCalibration.GyroSensorRange  = GYRO_RANGE_250DPS;
            break;
    }

    // ----- Set the default for the DMU380 -----
    //   Rate-Sensor: +/-125 [dps]
    //   Accel: +/-5 [g]
    //   Magnetometer: +/-2 [G] (sensors excluded from burst)
    Rate_Output_Config_Write(0x1F);
    Accel_Output_Config_Write(0x3F);
    Mag_Output_Config_Write(0x10);
    // ----- Set the default for the DMU380 -----

    /// ----- Temperature sensor configuration -----
    gAlgorithm.TempOutSF = 13.6778;
    /// Default: 0x03 (Board temperature sensors enabled, rate-sensor temp sensor enabled)
    gUserSpi.DataRegister[ SPI_REG_TEMP_OUTPUT_CONFIG_READ ] = BOARD_TEMP_SENSOR_ENABLE |
                                                                     TEMP_SENSOR_ENABLE;
}

void _FillSpiDataRegister_Filter( void )
{
    /// -------- Configure filter --------
    // Set the default filter 
    gUserSpi.DataRegister[SPI_REG_FILTER_TAPS_CONFIG_READ] = FIR_05HZ_LPF;
}

/** ***************************************************************************
 * @name _FillSpiDataRegister_ProductIdentification()
 * @brief Initialize the product ID registers 0x74 - 0x75 with default values
 * The serial number is generated at product configuration and loaded once.
 * register loaded from EEPROM at startup does not change during operation.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void _FillSpiDataRegister_Orientation( void )
{
    // Pull orientation from the memory
    uint16_t tmpOrient = gConfiguration.orientation.all;

    /// Populate the SPI registers with the orientation settings
    gUserSpi.DataRegister[ 0x74 ] = ( ( tmpOrient & 0xFF00 ) >> 8 );
    gUserSpi.DataRegister[ 0x75 ] = ( tmpOrient & 0x00FF );
}


/** ***************************************************************************
 * @name _FillSpiDataRegister_ProductIdentification()
 * @brief Initialize the product ID registers 0x52 - 0x59 with default values
 * The serial number is generated at product configuration and loaded once.
 * register loaded from EEPROM at startup does not change during operation.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void _FillSpiDataRegister_HWandSWVersion( void )
{
    /// ----- Firmware version -----
    /// get the Part number set in xbowsp_version.h
    const uint8_t PART_NUMBER_STRING [] = SOFTWARE_PART;
    uint8_t PartNumberLength = SOFTWARE_PART_LEN;

    uint8_t count = 0;

    char swVersionStr[6] = {0}, *pEnd;
    uint8_t j = 0, swVersion = 0;
    uint8_t minusOne = 0;

    // Search the part number string for the two hyphens.  Once the second is
    //   found, extract the SW version (major and minor) from the following four
    //   characters (ignoring the decimal point that separates the major and
    //   minor values).  If the major-version gets larger than 25 in the future
    //   then we need to rework the way this is handled.  Also, this methodology
    //   limits the minor-version to values less than 10.
    for( uint8_t i = 0; i < PartNumberLength; i ++ ) {
        if( count == 2 ) {
            if( PART_NUMBER_STRING[i] == '.' ) {
                minusOne = 1;
            } else {
                swVersionStr[j-minusOne] = PART_NUMBER_STRING[i];
            }

            j++;
            if( j == 4 ) {
                break;
            }
        }

        if( PART_NUMBER_STRING[i] == '-' ) {
            count++;
        }
    }

    // Convert from a string to an integer
    swVersion = strtol( swVersionStr, &pEnd, 10 );

    // Read the unit HW configuration based on the three resistors populated
    //   during manufacturing.  Note: HW configuration 2 (010) is set to
    //   configuration 4 as boards were built with resistors set incorrectly
    //   to 010 (should have been set to 100).
    uint8_t tmp = ReadUnitConfiguration_GPIO();
    if( tmp == 2 ) {
        tmp = 4;
    }

    /// Populate the SPI registers with the HW and SW configurations
    gUserSpi.DataRegister[ SPI_REG_HW_VERSION_READ ] = tmp;
    gUserSpi.DataRegister[ SPI_REG_SW_VERSION_READ ] = swVersion;
}

// FIXME: USED?
/** ****************************************************************************
 * @name: CheckPacketRateDivider
 * @brief checks for valid data.
*
* Trace:
* [SDD_CHECK_PACKET_RATE_DIVIDER <-- SRC_CHECK_PACKET_RATE_DIVIDER]
*
 * @param [In] dataRate: - the divider for the requested packet rate.
 * @retval 1 if available, zero otherwise.
 * @author: Darren Liccardo, Jan. 2004
*   	   Dong An, 2007, 2008
 ******************************************************************************/
BOOL CheckPacketRateDivider_UserSpi( uint16_t dataRate )
{
    switch( dataRate )
    {
        case OUTPUT_DATA_RATE_ZERO  :
        case OUTPUT_DATA_RATE_200_HZ:
        case OUTPUT_DATA_RATE_100_HZ:
        case OUTPUT_DATA_RATE_50_HZ :
        case OUTPUT_DATA_RATE_25_HZ :
        case OUTPUT_DATA_RATE_20_HZ :
        case OUTPUT_DATA_RATE_10_HZ :
        case OUTPUT_DATA_RATE_5_HZ  :
        case OUTPUT_DATA_RATE_4_HZ  :
        case OUTPUT_DATA_RATE_2_HZ  :
        case OUTPUT_DATA_RATE_1_HZ  :
            return TRUE;
            break;
        default:
            return FALSE;
            break;
    }
} /* end CheckPacketRateDivider */
