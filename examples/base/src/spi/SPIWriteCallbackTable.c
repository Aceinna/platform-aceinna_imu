/** ***************************************************************************
 * @file   SPIWriteCallbackTable.c
 * @Author DKH
 * @date   August, 2014
 * @copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 * Data Dictionary with pointers to callback functions used in SPI Master (MOSI)
 * 'write' to here calls. This function looks up a callback in O(0).
 ******************************************************************************/
#include <salvo.h> // memset
#include <stdint.h>

#include "UserCommunication_SPI.h"
#include "BIT.h"
#include "scaling.h" //  RADS_TO_DEGREES
#include "debug.h"
#include "bitSelfTest.h"
#include "sensor.h" // Mag align states
#include "xbowsp_algorithm.h"
#include "xbowsp_generaldrivers.h" // config struct
#include "xbowsp_fields.h" // ORIENTATION_FIELD_ID
#include "MagAlign.h"

#include "SPIWriteCallbackTable.h"

/// Local callback functions for SPI Write
void _Test(uint8_t data)
{ // DEBUG unit test fcn DEBUG
    gUserSpi.DataRegister[0x7e] = data;
}

void _Reserved(uint8_t data);
void _GPIO_Direction_Write(uint8_t data);       // 0x32
void _GPIO_Level_Write(uint8_t data);           // 0x33
void _Data_Ready_Write(uint8_t data);           // 0x34
void _Self_Test_Write(uint8_t data);            // 0x35
void _Clock_Config_Write(uint8_t data);         // 0x36
void _Decimation_Write(uint8_t data);           // 0x37
void _Filter_Taps_Config_Write(uint8_t data);   // 0x38
void _Dynamic_Range_Config_Write(uint8_t data); // 0x39

void _SpiWriteCal(uint8_t data);                // 0x50 // Mag Align

// Orientation setting functions
void _Axis_Orientation_Msb_Write(uint8_t data);   // 0x74
void _Axis_Orientation_Lsb_Write(uint8_t data);   // 0x75

// EEPROM write
void _Write_EEPROM_Cmd_Write(uint8_t data);       // 0x76

void _CalculateItemsInBurstRegister( void );

/** ***************************************************************************
 * @name tSpiCallback handle SPI MOSI "write" calls to registers
 * @brief These are used to configure the burst read size, crc, and sensor data
 * inclusion in the burst message. Will be used for sending a dynamically set
 * config to a unit on startup
 * big table lots of entries they are all pointers so small footprint in memory
 ******************************************************************************/
const tWriteTable gSpiWrite[] =
{
    {&_Reserved}, // 0x00
    {&_Reserved}, // 0x01
    {&_Reserved}, // 0x02 SPI Error
    {&_Reserved}, // 0x03 Num bytes Burst
    {&_Reserved}, // 0x04 Rate X msb
    {&_Reserved}, // 0x05        lsb
    {&_Reserved}, // 0x06 Rate Y msb
    {&_Reserved}, // 0x07        lsb
    {&_Reserved}, // 0x08 Rate Z msb
    {&_Reserved}, // 0x09        lsb
    {&_Reserved}, // 0x0a Accel X msb
    {&_Reserved}, // 0x0b         lsb
    {&_Reserved}, // 0x0c Accel Y msb
    {&_Reserved}, // 0x0d         lsb
    {&_Reserved}, // 0x0e Accel Z msb
    {&_Reserved}, // 0x0f         lsb
    {&_Reserved}, // 0x10 Mag X msb
    {&_Reserved}, // 0x11       lsb
    {&_Reserved}, // 0x12 Mag Y msb
    {&_Reserved}, // 0x13       lsb
    {&_Reserved}, // 0x14 Mag Z msb
    {&_Reserved}, // 0x15       lsb
    {&_Reserved}, // 0x16 Rate Temp msb
    {&_Reserved}, // 0x17           lsb
    {&_Reserved}, // 0x18 Board Temp msb
    {&_Reserved}, // 0x19            lsb
    {&_Reserved}, // 0x1a
    {&_Reserved}, // 0x1b
    {&_Reserved}, // 0x1c
    {&_Reserved}, // 0x1d
    {&_Reserved}, // 0x1e
    {&_Reserved}, // 0x1f
    {&_Reserved}, // 0x20
    {&_Reserved}, // 0x21
    {&_Reserved}, // 0x22
    {&_Reserved}, // 0x23
    {&_Reserved}, // 0x24
    {&_Reserved}, // 0x25
    {&_Reserved}, // 0x26
    {&_Reserved}, // 0x27
    {&_Reserved}, // 0x28
    {&_Reserved}, // 0x29
    {&_Reserved}, // 0x2a
    {&_Reserved}, // 0x2b
    {&_Reserved}, // 0x2c
    {&_Reserved}, // 0x2d
    {&_Reserved}, // 0x2e
    {&_Reserved}, // 0x2f
    {&_Reserved}, // 0x30
    {&_Reserved}, // 0x31
    {&_GPIO_Direction_Write},       // 0x32 GPIO Read, GPIO direction Read, GPIO direction Write
    {&_GPIO_Level_Write},           // 0x33
    {&_Data_Ready_Write},           // 0x34
    {&_Self_Test_Write},            // 0x35 Self test read
    {&_Clock_Config_Write},         // 0x36
    {&_Decimation_Write},           // 0x37
    {&_Filter_Taps_Config_Write},   // 0x38
    {&_Dynamic_Range_Config_Write}, // 0x39
    {&_Reserved}, // 0x3a
    {&_Reserved}, // 0x3b
    {&_Reserved}, // 0x3c 60d system status msb
    {&_Reserved}, // 0x3d               lsb
    {&_Reserved}, // 0x3E  62d John Deere MISO (read) data burst
    {&_Reserved}, // 0x3F:  UCB F1 Raw 1
    {&_Reserved}, // 0x40:  UCB F2 Raw 2
    {&_Reserved}, // 0x41:  UCB S0 Scaled 0
    {&_Reserved}, // 0x42:  UCB S1 Scaled 1
    {&_Reserved}, // 0x43:  UCB A1 Angle 1
    {&_Reserved}, // 0x44:  UCB A4 Angle 4
    {&_Reserved}, // 0x45:  UCB N0 Nav 0
    {&_Reserved}, // 0x46:
    {&_Reserved}, // 0x47:
    {&_Reserved}, // 0x48: Hard Iron Bias 0
    {&_Reserved}, // 0x49:
    {&_Reserved}, // 0x4a: Hard Iron Bias 1
    {&_Reserved}, // 0x4b:
    {&_Reserved}, // 0x4c: Soft Iron Scale
    {&_Reserved}, // 0x4d:
    {&_Reserved}, // 0x4e: Soft Iron angle
    {&_Reserved}, // 0x4f:
    {&_SpiWriteCal}, // 0x50: 80d UCB Write Cal - Mag align cmd
    {&_Reserved}, // 0x51: UCB Read  Cal - Mag Align status
    {&_Reserved}, // 0x52: Lot ID 1
    {&_Reserved}, // 0x53:
    {&_Reserved}, // 0x54: Lot ID 2
    {&_Reserved}, // 0x55:
    {&_Reserved}, // 0x56: whomai 0x56
    {&_Reserved}, // 0x57: 0x10
    {&_Reserved}, // 0x58: Serial number
    {&_Reserved}, // 0x59:
    {&_Reserved}, // 0x5a: Built In Test registers
    {&_Reserved}, // 0x5b:
    {&_Reserved}, // 0x5C: SPI_BIT_HW_STATUS_READ
    {&_Reserved}, // 0x5D:
    {&_Reserved}, // 0x5E: high level flags
    {&_Reserved}, // 0x5F:
    {&_Reserved}, // 0x60: algorithm state
    {&_Reserved}, // 0x61:
    {&_Reserved}, // 0x62: algorithm status
    {&_Reserved}, // 0x63:
    {&_Reserved}, // 0x64: CRC, Mag align out of bounds
    {&_Reserved}, // 0x65:
    {&_Reserved}, // 0x66: A and B serial
    {&_Reserved}, // 0x67:
    {&_Reserved}, // 0x68: SPI_BIT_COMM_DATA_STATUS_READ
    {&_Reserved}, // 0x69:
    {&_Reserved}, // 0x6A: detail bus error flags
    {&_Reserved}, // 0x6B:
    {&_Reserved}, // 0x6C: detail bus error flags
    {&_Reserved}, // 0x6D:
    {&_Reserved}, // 0x6e: over range etc.
    {&_Reserved}, // 0x6f:
    {&Accel_Output_Config_Write},   // 0x70
    {&Rate_Output_Config_Write},    // 0x71
    {&Temp_Output_Config_Write},    // 0x72
    {&Mag_Output_Config_Write},     // 0x73
    {&_Axis_Orientation_Msb_Write}, // 0x74:
    {&_Axis_Orientation_Lsb_Write}, // 0x75:
    {&_Write_EEPROM_Cmd_Write},     // 0x76:
    {&_Reserved}, // 0x77:
    {&_Reserved}, // 0x78:
    {&_Reserved}, // 0x79:
    {&_Reserved}, // 0x7A:
    {&_Reserved}, // 0x7B:
    {&_Reserved}, // 0x7C:
    {&_Reserved}, // 0x7D:
    {&_Reserved}, // 0x7E: HW version
    {&_Reserved}, // 0x7F: SW version
};

/** ***************************************************************************
 @brief
 For each item written to the device (when applicable):
   1) If register is not writeable set invalid write in the SPI register.
   2) If it is valid ...
      a) if the new value matches the one already in the register continue
         without doing anything.
      b) Check if the value is not valid create a invalid command error.
      c) Write the new value to the register.
      d) Change the variable in memory and act on the change.
  the sensor ranges and x/y/z/ configuration bits
 *****************************************************************************/

/** ***************************************************************************
 * @name _Reserved() no callback defined for this entry in the data dictionary
 * @brief place holder for future callbacks with code to avoid compiler errors
 * and exceptions for null table entries.
 * @param [in] data - call with the register number in error trap
 * @retval N/A
 ******************************************************************************/
void _Reserved(uint8_t data)
{
    DEBUG_HEX("No Callback defined for SPI REGISTER 0x", data);
    gUserSpi.DataRegister[ SPI_REG_SPI_ERROR ] = WRITE_TO_INVALID_REGISTER; // 0x03
}

/** ***************************************************************************
 * @name _GPIO_Direction_Write() set a flag that can be used to configure
 * a pin as input or output.
 * @brief SPI3 register 0x32
 * @param [in] data - flag to (re)set pin use
 * @retval N/A
 ******************************************************************************/
void _GPIO_Direction_Write(uint8_t data)       // 0x32
{
    gUserSpi.DataRegister[ SPI_REG_GPIO_DIRECTION_READ ] = data;
}

 // (FIXME: not active)
 /** ***************************************************************************
 * @name _GPIO_Level_Write() set a flag that can be used to configure
 * a pin IF it is configured as an output.
 * @brief SPI3 register 0x32
 * @param [in] data - flag to (re)set pin use
 * @retval N/A
 ******************************************************************************/
void _GPIO_Level_Write(uint8_t data)           // 0x33
{
    gUserSpi.DataRegister[ SPI_REG_GPIO_LEVEL_READ ] = data;
}

 /** ***************************************************************************
 * @name _Data_Ready_Write() enables data-ready and sets the pin and polarity.
 * a pin IF it is configured as an output.
 * @brief SPI3 register 0x34  (write: 0x34, read: 0x35)
 * @param [in] data - flag to (re)set pin use
 * @retval N/A
 ******************************************************************************/
void _Data_Ready_Write(uint8_t data)           // 0x34
{
    // Decode the data-ready bits written to the 380
    gUserSpi.EnableDataReady   = data & 0x4;  ///< logic flag in TDaq
    gUserSpi.DataReadyPolarity = data & 0x2;
    gUserSpi.DataReadyPin      = data & 0x1;  // FIXME: not active

    gUserSpi.DataRegister[ SPI_REG_DATA_READY_READ ] = // [0x35]
        data & SPI_REG_DATA_READY_MASK;

}

/** ***************************************************************************
 * @name _Self_Test_Write() Self-test settings - byte initiates a self-test
 * (sensor check).
 * a pin IF it is configured as an output.
 * @brief SPI3 register 0x35  (write: 0x35, read: 0x34)
 * @param [in] data - flag to (re)set pin use
 * @retval N/A
 ******************************************************************************/
void _Self_Test_Write(uint8_t data)            // [0x35]
{
    if( data & 0x8 ) { /* Do Nothing */ }

    /// start sensor self-test routine
    if( data & 0x4 ) {
        /// Results are placed in register 0x3C (bits 15:10) and bit5
        BITSetRunFlags();

        // self-test bit
        gUserSpi.DataRegister[ SPI_REG_SELF_TEST_READ ] = 0x04; // [24]
    }
}

/** ***************************************************************************
 * @name _Clock_Config_Write() indicate internal/external clock
 * @brief SPI3 register 0x36  (write: 0x35, read: 0x37)
 * @param [in] data - flag to (re)set pin use
 * @retval N/A
 ******************************************************************************/
void _Clock_Config_Write(uint8_t data)         // 0x36
{
    // bit 0: 1 internal clock, 0 external 1PPS
    gUserSpi.InternalTimer = data & 0x01;

    if( data & 0xFE ) { // Invalid command - mask off the lsb
        gUserSpi.DataRegister[ SPI_REG_SPI_ERROR ] = INVALID_WRITE_COMMAND;
    } else {
        gUserSpi.DataRegister[ SPI_REG_CLOCK_CONFIG_READ ] = //0x37
            data & SPI_REG_CLOCK_CONFIG_MASK;
    }
}

/** ***************************************************************************
 * @name _Decimation_Write() Set the number of 200 hz frames to get the listed
 *       rate (frame rate divisor)
 * @brief SPI3 register 0x37
 * @param [in] data - Enum: Requested frame frequency
 * @retval N/A
 ******************************************************************************/
void _Decimation_Write(uint8_t data)           // 0x37
{
    switch( data )
    {
        case REQUEST_200_HZ:
            gUserSpi.outputDataRate = OUTPUT_DATA_RATE_200_HZ;

            // If possible to use with a GPS then fall through this statement
            //   and set the ODR to the maximum possible: 100 Hz.  If unaided
            //   then set to 200 Hz output.
            if ( UcbGetSysType() <= UNAIDED_AHRS_SYS ) {
                break;
            }
        case REQUEST_100_HZ:
            gUserSpi.outputDataRate = OUTPUT_DATA_RATE_100_HZ;
            break;
        case REQUEST_50_HZ:
            gUserSpi.outputDataRate = OUTPUT_DATA_RATE_50_HZ;
            break;
        case REQUEST_25_HZ:
            gUserSpi.outputDataRate = OUTPUT_DATA_RATE_25_HZ;
            break;
        case REQUEST_20_HZ:
            gUserSpi.outputDataRate = OUTPUT_DATA_RATE_20_HZ;
            break;
        case REQUEST_10_HZ:
            gUserSpi.outputDataRate = OUTPUT_DATA_RATE_10_HZ;
            break;
        case REQUEST_5_HZ:
            gUserSpi.outputDataRate = OUTPUT_DATA_RATE_5_HZ;
            break;
        case REQUEST_4_HZ:
            gUserSpi.outputDataRate = OUTPUT_DATA_RATE_4_HZ;
            break;
        case REQUEST_2_HZ:
            gUserSpi.outputDataRate = OUTPUT_DATA_RATE_2_HZ;
            break;
        case REQUEST_1_HZ:
            gUserSpi.outputDataRate = OUTPUT_DATA_RATE_1_HZ;
            break;
        case REQUEST_ZERO_HZ:
            gUserSpi.outputDataRate = OUTPUT_DATA_RATE_ZERO;
            break;
        default:
            gUserSpi.outputDataRate = OUTPUT_DATA_RATE_100_HZ;
            break;
    }
        gUserSpi.DataRegister[ SPI_REG_DECIMATION_READ ] = data;
}

/** ***************************************************************************
 * @name _Filter_Taps_Config_Write() Set the number of 'taps' in the filter
 * @brief SPI3 register 0x38
 * @param [in] data - Number of taps
 * @retval N/A
 ******************************************************************************/
void _Filter_Taps_Config_Write(uint8_t data)   // 0x38
{
    // FIXME: unused - a read will be accepted
    gUserSpi.DataRegister[ SPI_REG_FILTER_TAPS_CONFIG_READ ] = data;
}

/** ***************************************************************************
 * @name _Dynamic_Range_Config_Write() Set the dynamic range of the rate-sensor
 * to scaled for xmit out SPI MISO
 * @brief SPI3 register 0x39
 * makes changes to rate-sensor configuration register 0x70
 * @param [in] data - Enum: Rate limit in 1, 2, 4, or 8
 * @retval N/A
 ******************************************************************************/
 void _Dynamic_Range_Config_Write(uint8_t data) // 0x39
{
    // check to see if the value changed (compare to the value in 0x38)
    if( data != gUserSpi.DataRegister[ SPI_REG_DYNAMIC_RANGE_CONFIG_READ ] )
    {
        if( data & 0xE0 ) {     /// upper nibble valid?
            gUserSpi.DataRegister[ SPI_REG_SPI_ERROR ] = INVALID_WRITE_COMMAND;
        } else {
            // Only look at the data in the lower 5 bits
            switch( data & 0x1F ) ///  lower nibble valid?
            {
                // The Maxim MAX21000 has a full-scale of �31.25/�62.5/�125/�250/�500/�1000/�2000 dps
                //  -200:  250 dps
                //  -400:  500 dps
                //  -600: 1000 dps
                case 0x01:
                    // RateLimit: set a bit when the output rate is greater than the limit
                    // floatLimit_Rate: limit above which the rate sensor output cannot be greater
                    // RateOutSF:
                    gAlgorithm.RateOutSF       = 400.00 * RADS_TO_DEGREES;  //

                    gAlgorithm.Limit.rateAlarm =  62.50 * DEGREES_TO_RADS;  // 62.5 [deg/sec] = 1.090830782496456 [rad/sec]
                    gUserSpi.floatLimit_Rate   =   0.98 * (real)MAXINT16 * ( 1/gAlgorithm.RateOutSF );   // 80.28 [deg/sec]

                    gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] = 0x00 |
                        ( gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] & 0x0F );
                    /// update the current register (SPI_REG_DYNAMIC_RANGE_CONFIG)
                    gUserSpi.DataRegister[ SPI_REG_DYNAMIC_RANGE_CONFIG_READ ] =
                        data & 0x0F;
                    break;
                case 0x02:
                    gAlgorithm.RateOutSF       = 200.00 * RADS_TO_DEGREES;

                    gAlgorithm.Limit.rateAlarm = 125.00 * DEGREES_TO_RADS;  // 125.0 [deg/sec] = 2.181661564992912 [rad/sec]
                    gUserSpi.floatLimit_Rate   =   0.98 * (real)MAXINT16 * ( 1/gAlgorithm.RateOutSF );  // 160.56 [deg/sec]

                    /// changes to rate-sensor configuration register (0x70 = 112)
                    gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] = 0x10 |
                        ( gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] & 0x0F );

                    gUserSpi.DataRegister[ SPI_REG_DYNAMIC_RANGE_CONFIG_READ ] =
                        data & 0x0F;
                    break;
                case 0x04:
                    gAlgorithm.RateOutSF       = 100.00 * RADS_TO_DEGREES;

                    gAlgorithm.Limit.rateAlarm = 200.00 * DEGREES_TO_RADS;  // 200.0 [deg/sec] = 3.4907 [rad/sec]
                    gUserSpi.floatLimit_Rate   = 220.00 * DEGREES_TO_RADS;  // 200.0 [deg/sec] = 3.4907 [rad/sec]

                    /// changes to rate-sensor configuration register (0x70 = 112)
                    gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] = 0x20 |
                        ( gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] & 0x0F );

                    gUserSpi.DataRegister[ SPI_REG_DYNAMIC_RANGE_CONFIG_READ ] =
                        data & 0x0F;
                    break;
                case 0x08:
                    gAlgorithm.RateOutSF       =  50.00 * RADS_TO_DEGREES;

                    gAlgorithm.Limit.rateAlarm = 400.00 * DEGREES_TO_RADS;  // 400.0 [deg/sec] = 6.9813 [rad/sec]
                    gUserSpi.floatLimit_Rate   = 440.00 * DEGREES_TO_RADS;  // 200.0 [deg/sec] = 3.4907 [rad/sec]

                    /// changes to rate-sensor configuration register (0x70 = 112)
                    gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] = 0x30 |
                        ( gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] & 0x0F );

                    gUserSpi.DataRegister[ SPI_REG_DYNAMIC_RANGE_CONFIG_READ ] =
                        data & 0x0F;
                    break;
                case 0x10:
                    gAlgorithm.RateOutSF       =  25.00 * RADS_TO_DEGREES;

                    gAlgorithm.Limit.rateAlarm = 600.00 * DEGREES_TO_RADS;  // 600 [deg/sec] = 10.4720 [rad/sec]
                    gUserSpi.floatLimit_Rate   = 660.00 * DEGREES_TO_RADS;  // // 1000 dps

                    /// changes to rate-sensor configuration register (0x70 = 112)
                    gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] = 0x40 |
                        ( gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] & 0x0F );

                    gUserSpi.DataRegister[ SPI_REG_DYNAMIC_RANGE_CONFIG_READ ] =
                        data & 0x0F;
                    break;
                default: /// Input is invalid
                    gUserSpi.DataRegister[ SPI_REG_SPI_ERROR ] = INVALID_WRITE_COMMAND;
                    break;
            }
        }
    }
}

/** ***************************************************************************
 * @name _SpiWriteCal() command start of mag align over SPI bus
 * @brief SPI3 register 0x50 calls UCB handler
 * makes changes to mag align register 0x50
 * @param [in] data - MAG_ALIGN_STATUS_LEVEL_START
 * MAG_ALIGN_STATUS_LEVEL_END
 * MAG_ALIGN_STATUS_SAVE2EEPROM
 * MAG_ALIGN_STATUS_START_CAL_WITHOUT_AUTOEND
 * MAG_ALIGN_STATUS_START_CAL_WITH_AUTOEND
 * MAG_ALIGN_STATUS_TERMINATION
 * MAG_ALIGN_STATUS_IDLE
 * @retval N/A
 ******************************************************************************/
void _SpiWriteCal(uint8_t data)
{
    /// start phase 1 - axis leveling
    gAlgorithm.calState = MAG_ALIGN_STATUS_START_CAL_WITH_AUTOEND; // cmd start
    // set status flag changes to 0x0b when complete
    gUserSpi.DataRegister[SPI_REG_MAG_ALIGN_READ] = MAG_ALIGN_STATUS_START_CAL_WITH_AUTOEND;

}

/** ***************************************************************************
 * @name _Accel_Output_Config_Write() Set the output range of the Accel-sensor
 * to scaled for xmit out SPI MISO
 * @brief SPI3 register 0x70
 * makes changes to Accel-sensor configuration register 0x71
 * @param [in] data - Enum: Accel limit 0, 10, 20, 40
 * @retval N/A
 ******************************************************************************/
void Accel_Output_Config_Write(uint8_t data) // 0x70
{
    uint8_t tempUInt8;

    /// New config?
    if( data != gUserSpi.DataRegister[SPI_REG_ACCEL_OUTPUT_CONFIG_READ] )
    {
        if( data & 0x80 )
        {   // data is only in the lower nibble
            gUserSpi.DataRegister[ SPI_REG_SPI_ERROR ] = INVALID_WRITE_COMMAND;
        } else {
           /// Adjust the scale factor based on bits 4, 5, and 6 of the register
            switch( data & 0x70 )
            {
                case 0x00:
                    gAlgorithm.AccelOutSF       = 32768.0;  // 2^15

                    gAlgorithm.Limit.accelAlarm = 0.90 * 1.0;  // will never trigger this but keep anyway
                    gUserSpi.floatLimit_Accel   = 1.0;  //0.98 * (double)MAXINT16 * ( 1.0 / gAlgorithm.AccelOutSF );   // 0.98 [g]
                    break;
                case 0x10:
                    gAlgorithm.AccelOutSF       = 16384.0;  // 2^14

                    gAlgorithm.Limit.accelAlarm = 0.90 * 2.0;   // 1.8 [g]
                    gUserSpi.floatLimit_Accel   = 0.98 * (double)MAXINT16 * ( 1.0 / gAlgorithm.AccelOutSF );   // 1.96 [g]
                    break;
                case 0x20:
                    gAlgorithm.AccelOutSF       =  8192.0;  // 2^13

                    gAlgorithm.Limit.accelAlarm = 0.90 * 4.0;   // 3.6 [g]
                    gUserSpi.floatLimit_Accel   = 0.98 * (double)MAXINT16 * ( 1.0 / gAlgorithm.AccelOutSF );   // 3.92 [g]
                    break;
                case 0x30:
                    gAlgorithm.AccelOutSF       =  4000.0;  // 4096.0 = 2^12

                    gAlgorithm.Limit.accelAlarm = 0.90 * 5.0;
                    gUserSpi.floatLimit_Accel   = 0.98 * (double)MAXINT16 * ( 1.0 / gAlgorithm.AccelOutSF );   // 0.98 [g]
                    break;
                case 0x40:
                    gAlgorithm.AccelOutSF       =  4096.0;  // 4096.0 = 2^12

                    gAlgorithm.Limit.accelAlarm = 0.90 * 8.0;   // 7.2 [g]
                    gUserSpi.floatLimit_Accel   = 0.98 * (double)MAXINT16 * ( 1.0 / gAlgorithm.AccelOutSF );   // 7.84 [g]
                    break;
                default:
                    /// @brief Do not change previously-valid limit or
                    /// scale-factor. Issue SPI invalid-write error (make change
                    /// to writeBuffer - copy to data-register is below).
                    tempUInt8 = gUserSpi.DataRegister[ SPI_REG_ACCEL_OUTPUT_CONFIG_READ ];
                    gUserSpi.writeBuffer[ 0 ] = ( gUserSpi.writeBuffer[ 0 ] & 0x0F ) |
                                                ( tempUInt8 & 0x70 );

                    /// Set error bits (clear after a read of the register)
                    gUserSpi.DataRegister[ SPI_REG_SPI_ERROR ] = INVALID_WRITE_VALUE;
                    break;
            }
            
            // The 0x0F was added to ensure that the accelerometer data is added
            //   to the Burst-Register.  Otherwise it will be excluded and the
            //   master code will not work.  Can augment this later so the burst
            //   register is configurable.
            // SPI_REG_RATE_CONFIG_MASK = 0x3F = 0111 1111
            gUserSpi.DataRegister[ SPI_REG_ACCEL_OUTPUT_CONFIG_READ ] = data | 0x0F;

            /// Make the change to the data-register.
            UserSPI_BytesInBurstRegister(); ///< number of active elements
            memset( (void*)gUserSpi.BurstRegister, 0 , BURST_REGISTER_SIZE);
        }
    }
}

/** ***************************************************************************
 * @name _Rate_Output_Config_Write() Set the output range of the Accel-sensor
 * to scaled for xmit out SPI MISO
 * @brief SPI3 register 0x71
 * makes changes to rate-sensor configuration register 0x70
 * @param [in] data - Enum: Rate limit in 1, 2, 4, or 8
 * @retval N/A
 ******************************************************************************/
void Rate_Output_Config_Write(uint8_t data)  // 0x71
{
    /// Has data changed?
    if( data != gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] )
    {
        if( data & 0x00 ) { /// Check for a valid write
            gUserSpi.DataRegister[ SPI_REG_SPI_ERROR ] = INVALID_WRITE_COMMAND;
        } else {
            /// @brief Adjust the scale factor based on BITS 4 and 5
            /// of the register; NOTE: This is only done for the
            /// rate-sensor configuration register.
            switch( data & 0x70 )  ///< mask to return bits 4 and 5
            {
                case 0x00:
                    // 62.5 dps
                    _Dynamic_Range_Config_Write( 0x01 );
                    break;
                case 0x10:
                    // 125 dps
                    _Dynamic_Range_Config_Write( 0x02 );
                    break;
                case 0x20:
                    // 250 dps
                    _Dynamic_Range_Config_Write( 0x04 );
                    break;
                case 0x30:
                    // 500 dps
                    _Dynamic_Range_Config_Write( 0x08 );
                    break;
                case 0x40:
                    // 1000 dps
                    _Dynamic_Range_Config_Write( 0x10 );
                    break;
            }

            // The 0x0F was added to ensure that the rate-sensor data is added
            //   to the Burst-Register.  Otherwise it will be excluded and the
            //   master code will not work.  Can augment this later so the burst
            //   register is configurable.
            // SPI_REG_RATE_CONFIG_MASK = 0x3F = 0111 1111
            gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] = // [0x70]
                ( data & SPI_REG_RATE_CONFIG_MASK ) | 0x0F;

            UserSPI_BytesInBurstRegister(); ///< # of active elements
            memset( (void*)gUserSpi.BurstRegister, 0 , BURST_REGISTER_SIZE);
        }
    }
}

/** ***************************************************************************
 * @name _Temp_Output_Config_Write() Set the output range of the Temp-sensor
 * @brief SPI3 register 0x72
 * For BURST the temperature is read straight into the burst register
 * makes changes to accel-sensor read register 0x71
 * @param [in] data
 * @retval N/A
 ******************************************************************************/
void Temp_Output_Config_Write(uint8_t data)  // 0x72
{
    if( data != gUserSpi.DataRegister[ SPI_REG_TEMP_OUTPUT_CONFIG_READ ] )
    {
        if( data & 0xF8 ) {
            gUserSpi.DataRegister[ SPI_REG_SPI_ERROR ] = INVALID_WRITE_COMMAND;
        } else {   /// Make the change to the data-register.
//            gUserSpi.DataRegister[ SPI_REG_MAG_OUTPUT_CONFIG_READ ] = // [0x72]
//                ( data & SPI_REG_MAG_CONFIG_MASK ) | 0x0F;
            gUserSpi.DataRegister[ SPI_REG_TEMP_OUTPUT_CONFIG_READ ] = ( data & 0x07 ); // [0x71]
            
            UserSPI_BytesInBurstRegister(); ///< Compute the number of active elements
            memset( (void*)gUserSpi.BurstRegister, 0 , BURST_REGISTER_SIZE);
        }
    }
}

/** ***************************************************************************
 * @name _Mag_Output_Config_Write() Set the output range and axis config of the
 *       Magnetic-sensor
 * @brief SPI3 register 0x73
 * makes changes to rmag-sensor read register 0x72
 * @param [in] data
 * @retval N/A
 ******************************************************************************/
void Mag_Output_Config_Write(uint8_t data)   // 0x73
{
    if( data != gUserSpi.DataRegister[ SPI_REG_MAG_OUTPUT_CONFIG_READ ] )
    {
        if( data & 0xC0 ) {
            gUserSpi.DataRegister[ SPI_REG_SPI_ERROR ] = INVALID_WRITE_COMMAND;
        } else {
            /// @brief  Adjust the scale factor based on BITS 4 and 5 of the
            /// register. Set the bits in register 0x38 Only done for the
            /// rate-sensor configuration register.
            switch( data & 0x30 )
            {
                case 0x00:   /// +/-1 G
                    gAlgorithm.MagOutSF       = 32768.0;  // 2^15
                    
                    gAlgorithm.Limit.magAlarm = 0.9 * 1.0;
                    gUserSpi.floatLimit_Mag   = 0.98 * 1.0; //(double)MAXINT16 * ( 1.0 / gAlgorithm.MagOutSF );   // 3.92 [g]
                    break;
                case 0x10:   /// +/-2 G
                    gAlgorithm.MagOutSF       = 16384.0;  // 2^14

                    gAlgorithm.Limit.magAlarm = 0.9 * 2.0;
                    gUserSpi.floatLimit_Mag   = 0.98 * 2.0; //(double)MAXINT16 * ( 1.0 / gAlgorithm.MagOutSF );   // 1.96 [G]
                    break;
                case 0x20:   /// +/-4 G
                    gAlgorithm.MagOutSF       =  8192.0;  // 2^13

                    gAlgorithm.Limit.magAlarm = 0.9 * 4.0;
                    gUserSpi.floatLimit_Mag   = 0.98 * 4.0; //(double)MAXINT16 * ( 1.0 / gAlgorithm.MagOutSF );   // 3.92 [G]
                    break;
                case 0x30:   /// +/-8 G
                    gAlgorithm.MagOutSF       =  4096.0;  // 2^12

                    gAlgorithm.Limit.magAlarm = 0.9 * 8.0;
                    gUserSpi.floatLimit_Mag   = 0.98 * 8.0; //(double)MAXINT16 * ( 1.0 / gAlgorithm.MagOutSF );   // 3.92 [g]
                    break;
            }

            // The 0x0F was added to ensure that the rate-sensor data is added
            //   to the Burst-Register.  Otherwise it will be excluded and the
            //   master code will not work.  Can augment this later so the burst
            //   register is configurable.
            // SPI_REG_RATE_CONFIG_MASK = 0x3F = 0111 1111
            gUserSpi.DataRegister[ SPI_REG_MAG_OUTPUT_CONFIG_READ ] = // [0x72]
                ( data & SPI_REG_MAG_CONFIG_MASK ) & 0xF0;

            UserSPI_BytesInBurstRegister(); ///< number of active elements
            memset( (void*)gUserSpi.BurstRegister, 0 , BURST_REGISTER_SIZE);
        }
    }
}


/** ***************************************************************************
 * @name UserSPI_BytesInBurstRegister() API
 * @brief Compute the number of sensors in the burst-register (does not include
 * CRC)
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void UserSPI_BytesInBurstRegister( void )
{
    uint8_t BurstRegisterIndex = 2;  ///< Status bytes

    /// Check the rate sensor configuration
    if( (gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] & 0x01) != 0 )
    {
        if( (gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] & 0x02) != 0 ) {
            BurstRegisterIndex++; }

        if( (gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] & 0x04) != 0 ) {
            BurstRegisterIndex++; }

        if( (gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] & 0x08) != 0 ) {
            BurstRegisterIndex++; }
    }

    /// Check the accelerometer configuration
    if( (gUserSpi.DataRegister[ SPI_REG_ACCEL_OUTPUT_CONFIG_READ ] & 0x01) != 0 )
    {
        if( (gUserSpi.DataRegister[ SPI_REG_ACCEL_OUTPUT_CONFIG_READ ] & 0x02) != 0 ) {
            BurstRegisterIndex++; }

        if( (gUserSpi.DataRegister[ SPI_REG_ACCEL_OUTPUT_CONFIG_READ ] & 0x04) != 0 ) {
            BurstRegisterIndex++; }

        if( (gUserSpi.DataRegister[ SPI_REG_ACCEL_OUTPUT_CONFIG_READ ] & 0x08) != 0 ) {
            BurstRegisterIndex++; }
    }

    /// Check the accelerometer configuration
    if( (gUserSpi.DataRegister[ SPI_REG_MAG_OUTPUT_CONFIG_READ ] & 0x01) != 0 )
    {
        if( (gUserSpi.DataRegister[ SPI_REG_MAG_OUTPUT_CONFIG_READ ] & 0x02) != 0 ) {
            BurstRegisterIndex++; }

        if( (gUserSpi.DataRegister[ SPI_REG_MAG_OUTPUT_CONFIG_READ ] & 0x04) != 0 ) {
            BurstRegisterIndex++; }

        if( (gUserSpi.DataRegister[ SPI_REG_MAG_OUTPUT_CONFIG_READ ] & 0x08) != 0 ) {
            BurstRegisterIndex++; }
    }

    /// Check the rate-sensor and temp sensor configurations
    if( (gUserSpi.DataRegister[ SPI_REG_RATE_OUTPUT_CONFIG_READ ] & 0x01) != 0 ) {
        if( (gUserSpi.DataRegister[ SPI_REG_TEMP_OUTPUT_CONFIG_READ ] & 0x04) != 0 ) {
            BurstRegisterIndex++; }
    }

    /// Check the rate-sensor and temp sensor configurations
    if( (gUserSpi.DataRegister[ SPI_REG_TEMP_OUTPUT_CONFIG_READ ] & 0x02) != 0 ) {
        BurstRegisterIndex++;
    }
    FillSpiDataRegister_OutputBytesExpected( BurstRegisterIndex );
}


static uint16_t fieldData = 0;

/** ***************************************************************************
 * @name _Axis_Orientation_Msb_Write() msb callback save first byte
 * @brief must get Msb even if the values are zero
 * @param [in] data
 * @retval N/A
 ******************************************************************************/
void _Axis_Orientation_Msb_Write(uint8_t data)   // 0x73
{
    // If byteCntr is nonzero then declare invalid and start again
    if( gUserSpi.byteCntr == 0 ) {
        gUserSpi.byteCntr = 1;
        gUserSpi.firstByte = data << 8;
    } else {
        gUserSpi.byteCntr = 0;
        gUserSpi.firstByte = 0;
    }
}

/** ***************************************************************************
 * @name _Axis_Orientation_Lsb_Write() msb callback save second byte
 * @brief must get Msb first then will pit the values in the config struct
 * @param [in] data
 * @retval N/A
 ******************************************************************************/
void _Axis_Orientation_Lsb_Write(uint8_t data)   // 0x75
{
    if( gUserSpi.byteCntr == 1 ) {
        // Place the separate bytes into the temporary variable
        uint16_t tmpOrient = gUserSpi.firstByte | data;

        /// Check user orientation field for validity and leave the field unchanged
        ///   if not valid
        if (CheckOrientation(tmpOrient) == TRUE) {
            gUserSpi.DataRegister[ 0x74 ] = gUserSpi.firstByte >> 8;
            gUserSpi.DataRegister[ 0x75 ] = data;

            gConfiguration.orientation.all = tmpOrient;
        }
    }

    // Reset variables
    gUserSpi.firstByte = 0;
    gUserSpi.byteCntr  = 0;
}


/** ***************************************************************************
 * @name _Write_EEPROM_Cmd_Write() write EEPROM callback
 * @brief will write all values in cal/config struct to flash memory. For now
 * the only values sent are the user (non-standard) orientation. Note JD
 * orientation are set in calibrate but user can select any orientation
 * @param [in] data
 * @retval N/A
 ******************************************************************************/
void _Write_EEPROM_Cmd_Write(uint8_t data)   // 0x76
{
    uint8_t  validFieldCount;
    uint16_t fieldId;
    uint8_t  numFields = 1;
    uint8_t tmp;

    if( data == 0x00 || data == 0x74 ) {
        fieldId = ORIENTATION_FIELD_ID;
        /// check if data to set is valid xbowsp_fields.c
        tmp = gUserSpi.DataRegister[ 0x75 ];
        fieldData = (gUserSpi.DataRegister[ 0x74 ] << 8) | tmp;
        validFieldCount = CheckEepromFieldData(numFields,
                                               &fieldId,
                                               &fieldData,
                                               &fieldId);
        if (validFieldCount > 0) { ///< all or some requested field changes valid?
            /// apply any changes
            if (WriteFieldData() == FALSE) { // xbowsp_fields.c
                ERROR_STRING("faild to write spi orientation to EEPROM\r\n");
            }
        }
    }
}
