/*****************************************************************************
 * @file xbowsp_fields.c Checking field data per Crossbow Serial Protocol
 *   - Providing validity checks on configuration field data.
 * @Author denglish
 * @rev 17482
 * @date   2011-02-09 22:58:58 -0800 (Wed, 09 Feb 2011)
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#include <math.h>
#include <stdint.h>
\
#include "dmu.h"
#include "xbowsp_algorithm.h"
#include "xbowsp_version.h"
#include "xbowsp_generaldrivers.h"
#include "calc_airData.h"
#include "ucb_packet.h"
#include "xbowsp_fields.h"
#include "s_eeprom.h"
#include "timer.h"
#include "xbowsp_init.h"
#include "extern_port.h"
#include "extern_port_config.h"
#include "scaling.h"
#include "sensor.h"
#include "EKF_Algorithm.h"
#include "qmath.h"
#include "driverGPS.h" // gps data struct
#include "Indices.h"


//#include "CompilerFlags.h"   // for GYRO_MAXIM21000 and GYRO_BMI160

/// proposed configurations
static ConfigurationStruct proposedRamConfiguration;
static ConfigurationStruct proposedEepromConfiguration;

static BOOL portConfigurationChanged = FALSE; // port settings are to be changed

/// for scaled sensor packet
#define MAX_TEMP_4_SENSOR_PACKET   99.9
#define MAX_OUTPUT_TEMP_q27  1335466394   // iq27( 1335466394 ) =  9.5 [ 10 degC ] =  99.5 [ degC ]
#define MIN_OUTPUT_TEMP_q27  -671088640   // iq27( -671088640 ) = -5.0 [ 10 degC ] = -50.0 [ degC ]

/** ****************************************************************************
 * @name DefaultPortConfiguration
 * @brief Set initial ports:
 * port 1 primary UCB, 57600 baud, AU packet, /1 packet divider
 * port 2 CRM input, 38400 baud
 * port 3 9600 baud, unassigned
 * port 4 9600 baud, unassigned
 * defined in extern_port_config.h
 * Trace:
 * [SDD_CFG_PORT_DEF_01 <-- SRC_CFG_PORT_DEF]
 * [SDD_CFG_PORT_DEF_02 <-- SRC_CFG_PORT_DEF]
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void DefaultPortConfiguration (void)
{
  gConfiguration.packetRateDivider = PACKET_RATE_DIVIDER;
  gConfiguration.packetType        = PACKET_TYPE;
  gConfiguration.baudRateUser      = BAUD_RATE_USER;
} /* end DefaultPortConfiguration */

/** ****************************************************************************
 * @name CheckPacketRateDivider
 * @brief checks for valid data.
 * @author Darren Liccardo, Jan. 2004
 * @author Dong An, 2007, 2008
 * Trace:
 * [SDD_CHECK_PACKET_RATE_DIVIDER <-- SRC_CHECK_PACKET_RATE_DIVIDER]
 * @param [in] packetRateDivider: the divider for the requested packet rate.
 * @retval 1 if available, zero otherwise.
 ******************************************************************************/
BOOL CheckPacketRateDivider (uint16_t packetRateDivider)
{
    switch (packetRateDivider) {
    case 0:
    case 1:
    case 2:
    case 4:
    case 5:
    case 10:
    case 20:
    case 25:
    case 50:
        return TRUE;
    default:
        return FALSE;
    }
} /* end CheckPacketRateDivider */

/** ****************************************************************************
 * @name CheckContPacketRate
 * @brief verify the packet can be 'comfortably' output at the baud rate and
 *        divider rate
 * Trace:
 * [SDD_PKT_CONT_RATE_CHK <-- SRC_PKT_CONT_RATE_CHK]
 * @param [in] outputPacket packet type,
 * @param [in] baud rate
 * @param [in] packetRateDivider - divider
 * @retval 	boolean, TRUE if the packet output is possible, FALSE if not
 ******************************************************************************/
BOOL CheckContPacketRate (UcbPacketType outputPacket,
                          uint16_t      baudRate,
                          uint16_t      packetRateDivider)
{
    BOOL     valid = TRUE;
    uint16_t bytesPerPacket;
    uint16_t bytesPerSecond;

    if (packetRateDivider == 0) {
        valid = TRUE;
    } else {
        bytesPerPacket = UCB_SYNC_LENGTH +
                         UCB_PACKET_TYPE_LENGTH +
                         UCB_PAYLOAD_LENGTH_LENGTH +
                         UCB_CRC_LENGTH;

        switch (outputPacket) {
            case UCB_IDENTIFICATION:
                bytesPerPacket += UCB_IDENTIFICATION_LENGTH;
                break;
            case UCB_TEST_0:
                bytesPerPacket += UCB_TEST_0_LENGTH;
                break;
            case UCB_TEST_1:
                bytesPerPacket += UCB_TEST_1_LENGTH;
                break;
            case UCB_FACTORY_1:
                bytesPerPacket += UCB_FACTORY_1_LENGTH;
                break;
            case UCB_FACTORY_2:
                bytesPerPacket += UCB_FACTORY_2_LENGTH;
                break;
            case UCB_ANGLE_1:
                bytesPerPacket += UCB_ANGLE_1_LENGTH;
                break;
            case UCB_ANGLE_2:
                bytesPerPacket += UCB_ANGLE_2_LENGTH;
                break;
            case UCB_ANGLE_5:
                bytesPerPacket += UCB_ANGLE_5_LENGTH;
                break;
            case UCB_ANGLE_U:
                bytesPerPacket += UCB_ANGLE_U_LENGTH;
                break;
            case UCB_VERSION_DATA:
                bytesPerPacket += UCB_VERSION_DATA_LENGTH;
                break;
            case UCB_VERSION_ALL_DATA:
                bytesPerPacket += UCB_VERSION_ALL_DATA_LENGTH;
                break;
            case UCB_SCALED_0:
                bytesPerPacket += UCB_SCALED_0_LENGTH;
                break;
            case UCB_SCALED_1:
                bytesPerPacket += UCB_SCALED_1_LENGTH;
                break;
            case UCB_NAV_0:
                bytesPerPacket += UCB_NAV_0_LENGTH;
                break;
            case UCB_NAV_1:
                bytesPerPacket += UCB_NAV_1_LENGTH;
                break;
            case UCB_NAV_2:
                bytesPerPacket += UCB_NAV_2_LENGTH;
                break;
            default:
                valid = FALSE;
        }

        bytesPerSecond = bytesPerPacket * (SERIAL_TX_ROUTINE_FREQUENCY / packetRateDivider);

        // For a message with 10 bits/byte (data, start, and stop-bits) and a
        //   safety-factor of 80%, determine if the baud-rate can support the
        //   message and output data rate (ODR)
        real dutyCycle = 0.81;
        switch (baudRate) {
            case BAUD_9600:
                valid = (BOOL)(bytesPerSecond < dutyCycle * 960.0 );
                break;
            case BAUD_19200:
                valid = (BOOL)(bytesPerSecond < dutyCycle * 1920.0 );
                break;
            case BAUD_38400:
                valid = (BOOL)(bytesPerSecond < dutyCycle * 3840.0 );
                break;
            case BAUD_57600:
                valid = (BOOL)(bytesPerSecond < dutyCycle * 5760.0 );
                break;
            case BAUD_115200:
                valid = (BOOL)(bytesPerSecond < dutyCycle * 11520.0 );
                break;
            case BAUD_230400:
                valid = (BOOL)(bytesPerSecond < dutyCycle * 23040.0 );
                break;
//            case BAUD_460800:
//                valid = (BOOL)(bytesPerSecond < 22960);   /// with 80 byte/sec margin
//                valid = (BOOL)(bytesPerSecond < 20736);  // ( 0.90/10 ) * 230400
//                valid = (BOOL)(bytesPerSecond < 18432);  // ( 0.80/10 ) * 230400
//                valid = (BOOL)(bytesPerSecond < dutyCycle * 960.0 );
//                break;
            default:
                valid = FALSE;
        }
    }
    return valid;
} /* end CheckContPacketRate */

/** ****************************************************************************
 * @name CheckPortBaudRate
 * @brief all serial port baud rates must be 9600, 19200, 38400 or 57600
 * Trace:
 * @param [in] portBaudRate - baud rate
 * @retval 	boolean, TRUE if the packet output is possible, FALSE if not
 ******************************************************************************/
BOOL CheckPortBaudRate (uint16_t portBaudRate)
{
    BOOL valid = TRUE;

    if (portBaudRate >= NUM_BAUD_RATES) {
        valid = FALSE;
    }
    return valid;
}

/** ****************************************************************************
 * @name ValidPortConfiguration
 * @brief Check output packet configuration members for sanity
 * Trace: [SDD_PORT_CFG_VALID_01 <-- SRC_PORT_CFG_VALID]
 *        [SDD_PORT_CFG_VALID_02 <-- SRC_PORT_CFG_VALID]
 *        [SDD_PORT_CFG_VALID_03 <-- SRC_PORT_CFG_VALID]
 *        [SDD_PORT_CFG_VALID_04 <-- SRC_PORT_CFG_VALID]
 *        [SDD_PORT_CFG_VALID_05 <-- SRC_PORT_CFG_VALID]
 * @param [in] proposedConfiguration - configuratione
 * @retval 	boolean, TRUE if the packet output is possible, FALSE if not
 ******************************************************************************/
BOOL ValidPortConfiguration (ConfigurationStruct *proposedConfiguration)
{
    /// working packet type byte buffer
    uint8_t       type [UCB_PACKET_TYPE_LENGTH];
    UcbPacketType continuousPacketType;
    BOOL          valid = TRUE;

    /// check packet rate divider
    valid &= (BOOL)(CheckPacketRateDivider(proposedConfiguration->packetRateDivider));

    /// get enum for requested continuous packet type
    type[0] = (uint8_t)((proposedConfiguration->packetType >> 8) & 0xff);
    type[1] = (uint8_t)(proposedConfiguration->packetType & 0xff);

    continuousPacketType = UcbPacketBytesToPacketType(type);

    /// check that a valid continuous output packet has been selected
    valid &= UcbPacketIsAnOutputPacket(continuousPacketType);

    /// check continuous packet rate
    valid &= CheckContPacketRate( continuousPacketType,
                                  proposedConfiguration->baudRateUser,
                                  proposedConfiguration->packetRateDivider );
    /// check port baud rates
    valid &= CheckPortBaudRate(proposedConfiguration->baudRateUser);

    return valid;
}
/* end ValidPortConfiguration */

/** ****************************************************************************
 * @name checkOrientation
 * @brief verifies the integrity of a proposed orientation field.
 * @author Darren Liccardo, Jan. 2004
 * @author Dong An, 2007, 2008
 * Trace: [SDD_CHK_FIELD_ORIENT<-- SRC_CHECK_ORIENTATION]
 * [SDD_INIT_CONFIGURATION_ORIENT_VALID <-- SRC_CHECK_ORIENTATION]
 * @param [in] orientation: the value of the orientation field to verify.
 * @retval 	one for valid orientations, zero otherwise.
 ******************************************************************************/
BOOL CheckOrientation (uint16_t orientation)
{
    switch ( orientation ) {
    case 0:
    case 9:
    case 35:
    case 42:
    case 65:
    case 72:
    case 98:
    case 107:
    case 133:
    case 140:
    case 146:
    case 155:
    case 196:
    case 205:
    case 211:
    case 218:
    case 273:
    case 280:
    case 292:
    case 301:
    case 336:
    case 345:
    case 357:
    case 364:
        return TRUE;
    default:
        return FALSE;
    }
}  /*end CheckOrientation */

/** ****************************************************************************
 * @name CheckBaroCorrection
 * @brief checks if the input baro correction is valid.
 * @name CheckBaroCorrection
 * [SDD_CHK_FIELD_BARO <-- SRC_CHECK_BARO_CORRECTION]
 * @param [in] baroCorrection the input baro correction.
 * @retval 	1: valid 0: invalid
 ******************************************************************************/
BOOL CheckBaroCorrection(int32_t baroCorrection)
{
    if (baroCorrection >=(BARO_CORRECTION_RANGE_LOW *
                          BARO_CORRECTION_SCALING_IN_EEPROM) &&
        baroCorrection <=(BARO_CORRECTION_RANGE_HIGH *
                          BARO_CORRECTION_SCALING_IN_EEPROM))
        return TRUE;
    else
        return FALSE;
}  /* end CheckBaroCorrection */


#include "taskUserCommunication.h"

/** ****************************************************************************
 * @name CheckFieldData
 * @brief checks if field data has valid values.
 * @author Dong An, 2007, 2008
 * Trace: [SDD_CHECK_FIELD_DATA <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_ADDRESS <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_ORIENT  <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_BARO   <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_PKT_01  <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_PKT_02  <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_RATE_DIVIDER_01  <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_RATE_DIVIDER_02  <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_PORT_USE_01  <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_PORT_USE_02  <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_PORT_RATE_01  <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_PORT_RATE_02  <-- SRC_CHECK_FIELD_DATA]
 *
 * @param [in] currentConfiguration - current configuration to modify
 * @param [in] numFields - number of fields to check
 * @author Dong An, 2007, 2008
 * Trace: [SDD_CHECK_FIELD_DATA <-- SRC_CHECK_FIELD_DATA]
 *							index-wise to the array of field IDs
 * @param [out] validFields[] - array of fieldId that have been validated.
 * @retval 	number of valid fields returned in validFields[]
 ******************************************************************************/
static uint8_t CheckFieldData (ConfigurationStruct *currentConfiguration,
                               uint8_t             numFields,
                               uint16_t            fieldId [],
                               uint16_t            fieldData [],
                               uint16_t            validFields [])
{
    BOOL                packetTypeChanged        = FALSE;
    BOOL                packetRateDividerChanged = FALSE;
    BOOL                userBaudChanged          = FALSE;
    /// index for stepping through proposed configuration fields
    uint8_t             fieldIndex      = 0;
    uint8_t             validFieldIndex = 0; ///< index for building valid return array
    uint8_t             type [UCB_PACKET_TYPE_LENGTH]; ///< working packet type byte buffer
    UcbPacketType       continuousPacketType;
    ConfigurationStruct proposedPortConfig;

    /// copy current configuration - for testing validity of port configuration only
    proposedPortConfig = *currentConfiguration;

    /// update new field settings in proposed configuration */
    for (fieldIndex = 0; fieldIndex < numFields; ++fieldIndex) {
        if ((fieldId[fieldIndex] >= LOWER_CONFIG_ADDR_BOUND) &&
            (fieldId[fieldIndex] <= UPPER_CONFIG_ADDR_BOUND)) {
            /// parse field ID and, if applicable, check using respective
            /// function (not all fields require this)
            switch (fieldId[fieldIndex]) {
                case PACKET_TYPE_FIELD_ID:
                    /// get enum for requested continuous packet type
                    type[0] = (uint8_t)((fieldData[fieldIndex] >> 8) & 0xff);
                    type[1] = (uint8_t)(fieldData[fieldIndex] & 0xff);

                    continuousPacketType = UcbPacketBytesToPacketType(type);

                    /// check that a valid continuous output packet has been selected
                    if (UcbPacketIsAnOutputPacket(continuousPacketType) == TRUE) {
                        packetTypeChanged             = TRUE;
                        proposedPortConfig.packetType = fieldData[fieldIndex];
                    }
                    break;
                case PACKET_RATE_DIVIDER_FIELD_ID:
                    packetRateDividerChanged             = TRUE;
                    proposedPortConfig.packetRateDivider = fieldData[fieldIndex];
                    break;
                case PORT_1_BAUD_RATE_FIELD_ID:
                    userBaudChanged = TRUE;
                    proposedPortConfig.baudRateUser = fieldData[fieldIndex];
                    break;
                case ORIENTATION_FIELD_ID:
                    if (CheckOrientation(fieldData[fieldIndex]) == TRUE) {
                        /// update proposed configuration
                        currentConfiguration->orientation.all = fieldData[fieldIndex];
                        /// add to valid list
                        validFields[validFieldIndex++]        = fieldId[fieldIndex];
                        // Set the flags to RESTART the algorithm
                        InitializeAlgorithmStruct(&gAlgorithm);
                    }
                    break;
                case BARO_CORRECTION_FIELD_ID:
                    if (CheckBaroCorrection((int32_t)fieldData[fieldIndex]) == TRUE) {
                        /// update proposed configuration
                        currentConfiguration->baroCorrection = (int16_t)(fieldData[fieldIndex]);
                        /// add to valid list
                        validFields[validFieldIndex++]       = fieldId[fieldIndex];
                    }
                    break;
                case OFFSET_ROLL_ALIGN_FIELD_ID:
                    //int16_t tmp = (int16_t)(fieldData[fieldIndex]);
                    break;
                case OFFSET_PITCH_ALIGN_FIELD_ID:
                    //tmp = (int16_t)(fieldData[fieldIndex]);
                    break;
                case OFFSET_YAW_ALIGN_FIELD_ID:
                    //tmp = (int16_t)(fieldData[fieldIndex]);
                    break;
                default:
                    ((uint16_t *)currentConfiguration)[(fieldId[fieldIndex])] = fieldData[fieldIndex];
                    validFields[validFieldIndex++] = fieldId[fieldIndex];
                    break;
            }
        }
    }

    /// check proposed port configuration field settings (order/priority matters!)
    if (userBaudChanged == TRUE) {
        if (ValidPortConfiguration(&proposedPortConfig) == TRUE) {
            portConfigurationChanged = TRUE;

            /// add configuration changes to proposed configuration and add all
            /// relevant fields to valid list
            if (packetTypeChanged == TRUE) {
                currentConfiguration->packetType = proposedPortConfig.packetType;
                validFields[validFieldIndex++]   = PACKET_TYPE_FIELD_ID;
            }

            if (packetRateDividerChanged == TRUE) {
                currentConfiguration->packetRateDivider = proposedPortConfig.packetRateDivider;
                validFields[validFieldIndex++]          = PACKET_RATE_DIVIDER_FIELD_ID;
            }

            if (userBaudChanged == TRUE) {
                currentConfiguration->baudRateUser = proposedPortConfig.baudRateUser;
                validFields[validFieldIndex++]     = BAUD_RATE_USER_ID;
            }
        }
    } else if ((packetTypeChanged == TRUE) ||
               (packetRateDividerChanged == TRUE)) {
        /// port usage or baud settings haven't changed, DON'T indicate port
        /// configuration change
        proposedPortConfig.baudRateUser = currentConfiguration->baudRateUser;

        if (ValidPortConfiguration(&proposedPortConfig) == TRUE) {
            if (packetTypeChanged == TRUE) {
                currentConfiguration->packetType = proposedPortConfig.packetType;
                validFields[validFieldIndex++]   = PACKET_TYPE_FIELD_ID;
            }

            if (packetRateDividerChanged == TRUE) {
                currentConfiguration->packetRateDivider = proposedPortConfig.packetRateDivider;
                validFields[validFieldIndex++]          = PACKET_RATE_DIVIDER_FIELD_ID;
            }
        }
    }
    return validFieldIndex;
} /* end CheckFieldData */

/** ****************************************************************************
 * @name CheckRamFieldData
 * @brief checks if field data has valid values.
 * Trace: [SDD_CHK_RAM_FIELDS_02 <-- SRC_CHECK_RAM_FIELD_DATA]
 * @param [in] numFields - number of fields to check
 * @param [in] fieldId[] - array of field IDs.
 * @param [in] fieldData[] - array of field data corresponding
 *							index-wise to the array of field IDs
 * @param [out] validFields[] - array of fieldId that have been validated.
 * @retval number of valid fields returned in validFields[]
 ******************************************************************************/
uint8_t CheckRamFieldData (uint8_t  numFields,
                           uint16_t fieldId [],
                           uint16_t fieldData [],
                           uint16_t validFields [])
{
    proposedRamConfiguration = gConfiguration; /// current RAM configuration

    return CheckFieldData(&proposedRamConfiguration,
                          numFields,
                          fieldId,
                          fieldData,
                          validFields);
}

/** ****************************************************************************
 * @name CheckEepromFieldData
 * @brief checks if field data has valid values.
 * Trace: [SDD_CHK_EEPROM_FIELDS_02 <-- SRC_CHECK_EEPROM_FIELD_DATA]
 * @param [in] numFields - number of fields to check
 * @param [in] fieldId[] - array of field IDs.
 * @param [in] fieldData[] - array of field data corresponding
 *							index-wise to the array of field IDs
 * @param [out] validFields[] - array of fieldId that have been validated.
 * @retval number of valid fields returned in validFields[]
 ******************************************************************************/
uint8_t CheckEepromFieldData (uint8_t  numFields,
                              uint16_t fieldId [],
                              uint16_t fieldData [],
                              uint16_t validFields [])
{   /// copy current EEPROM configuration
    readEEPROMConfiguration(&proposedEepromConfiguration);

    return CheckFieldData(&proposedEepromConfiguration,
                          numFields,
                          fieldId,
                          fieldData,
                          validFields);
}

/** ****************************************************************************
 * @name SetFieldData
 * @brief Perform config changes required by the "SF" command
 * Trace: [SDD_UCB_SET_FIELD_01 <-- SRC_SET_FIELD_DATA]
 *	[SDD_UCB_SET_FIELD_02 <-- SRC_SET_FIELD_DATA]
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void SetFieldData (void)
{
    /// special handling for changing port configuration
    if (portConfigurationChanged == TRUE) {
        /// wait for all data in output buffers to be completely sent
        ExternPortWaitOnTxIdle();
        /// assign proposed configuration to actual configuration
        gConfiguration = proposedRamConfiguration;
        ExternPortInit(); ///< reset communication interface

        portConfigurationChanged = FALSE; /// reset changed flag
    } else {  /// non-port related configuration change
        /// assign proposed configuration to actual configuration
        gConfiguration = proposedRamConfiguration;
    }
} /* end SetFieldData */

/** ****************************************************************************
 * @name WriteFieldData
 * @brief write the data from the WF command into eeprom
 * Trace: [SDD_UCB_WRITE_FIELD <-- SRC_WRITE_FIELD_DATA]
 * @param N/A
 * @retval status TRUE, FALSE
 ******************************************************************************/
BOOL WriteFieldData (void)
{
    BOOL     success;
    // ConfigurationStruct xbowsp_generaldrivers.h
    uint16_t *ptr = (uint16_t*) &proposedEepromConfiguration;

    ptr++; ///< get past CRC at top

    /// write entire proposed configuration back to EEPROM
    if (writeEEPROMByte(LOWER_CONFIG_ADDR_BOUND, // 0x1
                        NUM_CONFIG_FIELDS * // 0x2b - 1 - 2 = 0x28 = 40 byts
                        SIZEOF_WORD, // 2 bytes
                        (void *)ptr) == 0) {
        success = TRUE;
    } else {
        success = FALSE;
    }

    return success;
} /* end WriteFieldData */

/** ****************************************************************************
 * @name appendCorrectedRates
 * @brief calculates the algorithm corrected angular rates and formats them for
 *        output.
 * @author Darren Liccardo, Aug. 2005
 * @author Dong An, 2007,2008
 * Trace: [SDD_APPEND_CORRECTED_RATES <-- SRC_APPEND_CORRECTED_RATES]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendCorrectedRates (uint8_t  *response,
                               uint16_t index)
{
    int16_t tmp;

    /// i = 0
    tmp = (int16_t) SCALE_BY_2POW16_OVER_7PI(gKalmanFilter.correctedRate_B[X_AXIS]);
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// i = 1
    tmp = (int16_t) SCALE_BY_2POW16_OVER_7PI(gKalmanFilter.correctedRate_B[Y_AXIS]);
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// i = 2
    tmp = (int16_t) SCALE_BY_2POW16_OVER_7PI(gKalmanFilter.correctedRate_B[Z_AXIS]);
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    return index;
} /* end appendCorrectedRates */

/** ****************************************************************************
 * @name appendRates
 * @brief calculates the algorithm corrected angular
 *    rates and formats them for output.
 * @author Darren Liccardo, Aug. 2005
 * @author Dong An, 2007,2008
 * Trace: [SDD_APPEND_RATES <-- SRC_APPEND_RATES]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendRates (uint8_t  *response,
                      uint16_t index)
{
    int tmp;

    /// X-Axis
    tmp = _qmul( TWO_POW16_OVER_7PI_q19, gAlgorithm.scaledSensors_q27[XRATE], 19, 27, 16) >> 16;
    index = uint16ToBuffer(response,
                           index,
                           (uint16_t)tmp);
    /// Y-Axis
    tmp = _qmul( TWO_POW16_OVER_7PI_q19, gAlgorithm.scaledSensors_q27[YRATE], 19, 27, 16) >> 16;
    index = uint16ToBuffer(response,
                           index,
                           (uint16_t)tmp);
    /// Z-Axis
    tmp = _qmul( TWO_POW16_OVER_7PI_q19, gAlgorithm.scaledSensors_q27[ZRATE], 19, 27, 16) >> 16;
    index = uint16ToBuffer(response,
                           index,
                           (uint16_t)tmp);
    return index;
} /* end appendRates */

/** ****************************************************************************
 * @name appendMagReadings
 * @brief calculates the algorithm corrected magnetometer readings and formats
 *        them for output.
 * @author Darren Liccardo, Aug. 2005
 * @author Dong An, 2007,2008
 * @author Joe Motyka, June 2013
 * Trace: [SDD_APPEND_MAG_READINGS <-- SRC_APPEND_MAG_READINGS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendMagReadings( uint8_t  *response,
                            uint16_t index )
{
    int tmp;

    /// Cycle through each magnetometer axis and convert it to a 16-bit integer
    /// X-Axis
    tmp = _qmul( TWO_POW16_OVER_20_q19, gAlgorithm.scaledSensors_q27[XMAG], 19, 27, 16 ) >> 16;
    /// Split the 16-bit integer into two 8-bit numbers and place it into the buffer
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Y-Axis
    tmp = _qmul( TWO_POW16_OVER_20_q19, gAlgorithm.scaledSensors_q27[YMAG], 19, 27, 16 ) >> 16;
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Z-Axis
    tmp = _qmul( TWO_POW16_OVER_20_q19, gAlgorithm.scaledSensors_q27[ZMAG], 19, 27, 16 ) >> 16;
    index = uint16ToBuffer(response,
                           index,
                           tmp);

    return index;
} /* end appendMagReadings */

/** ****************************************************************************
 * @name appendTangentRates
 * @brief formats the local level frame angular rates for output.
 * Trace: [SDD_APPEND_TANGENT_RATES <-- SRC_APPEND_TANGENT_RATES]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendTangentRates (uint8_t  *response,
                             uint16_t index)
{
    int tmp;

    /// X-axis
    tmp = (int)(SCALE_BY_2POW16_OVER_7PI(gAlgorithm.tangentRates[X_AXIS]));
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Y-axis
    tmp = (int)(SCALE_BY_2POW16_OVER_7PI(gAlgorithm.tangentRates[Y_AXIS]));
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Z-axis
    tmp = (int)(SCALE_BY_2POW16_OVER_7PI(gAlgorithm.tangentRates[Z_AXIS]));
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    return index;
} /* end appendTangentRates */


/** ****************************************************************************
 * @name appendAccels
 * @brief calculates the algorithm corrected accelerations and formats them for
 *        output.
 * @author Darren Liccardo, Aug. 2005
 * @author Dong An, 2007,2008
 * Trace: [SDD_APPEND_ACCELS <-- SRC_APPEND_ACCELS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendAccels (uint8_t  *response,
                       uint16_t index)
{
    uint16_t tmp;
   

    /// X-Axis
    tmp = _qmul( TWO_POW16_OVER_20_q19, gAlgorithm.scaledSensors_q27[XACCEL], 19, 27, 16 ) >> 16;
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Y-Axis
    tmp = _qmul( TWO_POW16_OVER_20_q19, gAlgorithm.scaledSensors_q27[YACCEL], 19, 27, 16 ) >> 16;
    
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Z-Axis
    tmp = _qmul( TWO_POW16_OVER_20_q19, gAlgorithm.scaledSensors_q27[ZACCEL], 19, 27, 16 ) >> 16;
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    return index;
} /* end appendAccels */

/** ****************************************************************************
 * @name appendCorrectedAccels
 * @brief calculates the algorithm corrected accelerations and formats them for
 *        output.
 * @author Darren Liccardo, Aug. 2005
 * @author Dong An, 2007,2008
 * Trace: [SDD_APPEND_ACCELS <-- SRC_APPEND_ACCELS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendCorrectedAccels (uint8_t  *response,
                                uint16_t index)
{
    uint16_t tmp;

    /// X-Axis
    tmp = _qmul( TWO_POW16_OVER_20_q19,
                 (int32_t)(gKalmanFilter.correctedAccel_B[XACCEL]*13681725.58613659),
                 19, 27, 16 ) >> 16;
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Y-Axis
    tmp = _qmul( TWO_POW16_OVER_20_q19,
                 (int32_t)(gKalmanFilter.correctedAccel_B[YACCEL]*13681725.58613659),
                 19, 27, 16 ) >> 16;
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Z-Axis
    tmp = _qmul( TWO_POW16_OVER_20_q19,
                 (int32_t)(gKalmanFilter.correctedAccel_B[ZACCEL]*13681725.58613659),
                 19, 27, 16 ) >> 16;
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    return index;
} /* end appendAccels */

/** ****************************************************************************
 * @name appendTangentAccels
 * @brief calculates Along Heading Acceleration, Cross Heading Acceleration and
 *        Vertical Acceleration for ARINC705, and formats them for output.
 * Trace: [SDD_APPEND_ACCELS <-- SRC_APPEND_ACCELS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendTangentAccels (uint8_t  *response,
                              uint16_t index)
{
    int    tmp;
    double aHcHvAccel[3];

    aHcHvAccel[X_AXIS] =   gAlgorithm.tangentAccels[X_AXIS];
    aHcHvAccel[Y_AXIS] =   gAlgorithm.tangentAccels[Y_AXIS];
    aHcHvAccel[Z_AXIS] = -(gAlgorithm.tangentAccels[Z_AXIS] + 1.0);

    /// X-Axis
    tmp = (int)( SCALE_BY_2POW16_OVER_20( aHcHvAccel[X_AXIS] ) );
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Y-Axis
    tmp = (int)( SCALE_BY_2POW16_OVER_20( aHcHvAccel[Y_AXIS] ) );
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Z-Axis
    tmp = (int)( SCALE_BY_2POW16_OVER_20( aHcHvAccel[Z_AXIS] ) );
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    return index;
} /* end appendTangentAccels */


/** ****************************************************************************
 * @name appendRateTemp
 * @brief formats rate and board temperature data for output.
 * @author Darren Liccardo, Dec. 2005
 * @author Dong An, 2007,2008
 * Trace: [SDD_APPEND_TEMPS_01 <-- SRC_APPEND_TEMPS]
 * [SDD_APPEND_TEMPS_02 <-- SRC_APPEND_TEMPS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendRateTemp (uint8_t  *response,
                         uint16_t index)
{
    uint16_t tmp;
    double   tmpD;

    tmpD = gAlgorithm.scaledSensors[XRTEMP];

    if (tmpD >= MAX_TEMP_4_SENSOR_PACKET) {
        tmpD =  MAX_TEMP_4_SENSOR_PACKET;
    }
    if (tmpD <= -MAX_TEMP_4_SENSOR_PACKET) {
        tmpD =  -MAX_TEMP_4_SENSOR_PACKET;
    }

    tmp   = (int)(SCALE_BY_2POW16_OVER_200(tmpD));

    index = uint16ToBuffer(response,
                           index,
                           tmp);
    return index;

} /*end appendTemps */


/** ****************************************************************************
 * @name appendTemps
 * @brief formats rate and board temperature data for output.
 * @author Darren Liccardo, Dec. 2005
 * @author Dong An, 2007,2008
 * Trace: [SDD_APPEND_TEMPS_01 <-- SRC_APPEND_TEMPS]
 * [SDD_APPEND_TEMPS_02 <-- SRC_APPEND_TEMPS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendTemps (uint8_t  *response,
                      uint16_t index)
{
    uint16_t tmp;

    // Rate sensor temperature hn units of [ 10 degC ] but the output must be in
    //   degC scaled by 2^16/200 so the multiplier must be ( 2^16/20 ). The max
    //   temp should be 15.5 [ 10 degC ], which places the max value of l
    // iqmath
    if( gAlgorithm.scaledSensors_q27[XRTEMP+0] >= MAX_OUTPUT_TEMP_q27 ) {
        gAlgorithm.scaledSensors_q27[XRTEMP+0] = MAX_OUTPUT_TEMP_q27;
    } else if( gAlgorithm.scaledSensors_q27[XRTEMP+0] <= MIN_OUTPUT_TEMP_q27 ) {
        gAlgorithm.scaledSensors_q27[XRTEMP+0] = MIN_OUTPUT_TEMP_q27;
    }

    // Convert to scaled output T { degC ] * ( 2^16/200 )
    tmp = _qmul( TWO_POW16_OVER_20_q19, gAlgorithm.scaledSensors_q27[XRTEMP+0], 19, 27, 15 ) >> 15;
    index = uint16ToBuffer(response,
                           index,
                           tmp);

    // iqmath
    if( gAlgorithm.scaledSensors_q27[XRTEMP+1] >= MAX_OUTPUT_TEMP_q27 ) {
        gAlgorithm.scaledSensors_q27[XRTEMP+1] = MAX_OUTPUT_TEMP_q27;
    } else if( gAlgorithm.scaledSensors_q27[XRTEMP+1] <= MIN_OUTPUT_TEMP_q27 ) {
        gAlgorithm.scaledSensors_q27[XRTEMP+1] = MIN_OUTPUT_TEMP_q27;
    }

    tmp = _qmul( TWO_POW16_OVER_20_q19, gAlgorithm.scaledSensors_q27[XRTEMP+1], 19, 27, 15 ) >> 15;
    index = uint16ToBuffer(response,
                           index,
                           tmp);

    // iqmath
    if( gAlgorithm.scaledSensors_q27[XRTEMP+2] >= MAX_OUTPUT_TEMP_q27 ) {
        gAlgorithm.scaledSensors_q27[XRTEMP+2] = MAX_OUTPUT_TEMP_q27;
    } else if( gAlgorithm.scaledSensors_q27[XRTEMP+2] <= MIN_OUTPUT_TEMP_q27 ) {
        gAlgorithm.scaledSensors_q27[XRTEMP+2] = MIN_OUTPUT_TEMP_q27;
    }
    tmp = _qmul( TWO_POW16_OVER_20_q19, gAlgorithm.scaledSensors_q27[XRTEMP+2], 19, 27, 15 ) >> 15;
    index = uint16ToBuffer(response,
                           index,
                           tmp);

    // Board temperature is in units of [ 10 degC ] but the output must be in
    //   degC scaled 2^16/200 so the multiplier must be ( 2^16/20 ). The max
    //   temp should be 15.5 [ 10 degC ], which places the max value of l
    // iqmath
    if( gAlgorithm.scaledSensors_q27[BTEMP] >= MAX_OUTPUT_TEMP_q27 ) {
        gAlgorithm.scaledSensors_q27[BTEMP] = MAX_OUTPUT_TEMP_q27;
    } else if( gAlgorithm.scaledSensors_q27[BTEMP] <= MIN_OUTPUT_TEMP_q27 ) {
        gAlgorithm.scaledSensors_q27[BTEMP] = MIN_OUTPUT_TEMP_q27;
    }

    tmp = _qmul( TWO_POW16_OVER_20_q19, gAlgorithm.scaledSensors_q27[BTEMP], 19, 27, 15 ) >> 15;
    // Split the 16-bit integer into two 8-bit numbers and place it into the response array
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    return index;
} /*end appendTemps */

/** ****************************************************************************
 * @name appendAttitudeTrue
 * @brief calculates roll, pitch, and true heading and formats them for output.
 * @author Darren Liccardo, Dec. 2005
 * @author Dong An, 2007,2008
 * Trace: [SDD_APPEND_ATTITUDE_TRUE <-- SRC_APPEND_ATTITUDE_TRUE]
 * [SDD_APPEND_TEMPS_02 <-- SRC_APPEND_TEMPS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendAttitudeTrue (uint8_t  *response,
                             uint16_t index)
{
    int16_t tmp;

    // X-Axis (angle in radians
    tmp = (int16_t) SCALE_BY_2POW16_OVER_2PI(gKalmanFilter.eulerAngles[ROLL]);
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Y-Axis
    tmp = (int16_t) SCALE_BY_2POW16_OVER_2PI(gKalmanFilter.eulerAngles[PITCH]);
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Z-Axis
    tmp = (int16_t) SCALE_BY_2POW16_OVER_2PI(gKalmanFilter.eulerAngles[YAW]);
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    return index;
} /* end appendAttitudeTrue */

/** ****************************************************************************
 * @name appendInertialCounts
 * @briefformats accels and rates for output.
 * @author Dong An, 2007,2008
 * Trace: [SDD_APPEND_INERTIAL_COUNTS <-- SRC_APPEND_INERTIAL_COUNTS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendInertialCounts (uint8_t  *response,
                               uint16_t index)
{
    /// X-Axis Accelerometer
    index = uint32ToBuffer(response,
                           index,
                           gAlgorithm.rawSensors[XACCEL]);
    /// Y-Axis Accelerometer
    index = uint32ToBuffer(response,
                           index,
                           gAlgorithm.rawSensors[YACCEL]);

    /// Z-Axis Accelerometer
    index = uint32ToBuffer(response,
                           index,
                           gAlgorithm.rawSensors[ZACCEL]);

    /// X-Axis Rate-Sensor
    index = uint32ToBuffer(response,
                           index,
                           gAlgorithm.rawSensors[XRATE]);

    /// Y-Axis Rate-Sensor
    index = uint32ToBuffer(response,
                           index,
                           gAlgorithm.rawSensors[YRATE]);

    /// Z-Axis Rate-Sensor
    index = uint32ToBuffer(response,
                           index,
                           gAlgorithm.rawSensors[ZRATE]);

    return index;
} /* end appendInertialCounts */

/** ****************************************************************************
 * @name appendMagnetometerCounts
 * @brief formats magnetometers for output.
 * @author Joe Motyka, 2013
 * Trace: [SDD_APPEND_MAGNETOMETER_COUNTS <-- SRC_APPEND_MAGNETOMETER_COUNTS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendMagnetometerCounts (uint8_t  *response,
                                   uint16_t index)
{
    /// X-Axis
    index = uint32ToBuffer(response,
                           index,
                           gAlgorithm.rawSensors[XMAG]);
    /// Y-Axis
    index = uint32ToBuffer(response,
                           index,
                           gAlgorithm.rawSensors[YMAG]);
    /// Z-Axis
    index = uint32ToBuffer(response,
                           index,
                           gAlgorithm.rawSensors[ZMAG]);
    return index;
} /* end appendMagnetometerCounts */

/** ****************************************************************************
 * @name appendAllTempCounts
 * @brief formats the measured temperature counts of gyro, accel and PCB for
 *        output.
 * @author Darren Liccardo, Oct. 2005
 * @author Dong An, 2007,2008
 * Trace: [SDD_APPEND_ALL_TEMP_COUNTS <-- SRC_APPEND_ALL_TEMP_COUNTS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendAllTempCounts (uint8_t  *response,
                              uint16_t index)
{
    uint32_t temp;

    /// @brief
    /// The older device had a temperature sensor on each of the sensors (prior
    /// to tri-axial device). The DMU380 only has a temperature sensor on the
    /// rate sensor and the boad.  To preserve the packet definition used
    /// previously (and to make the packet work with NavView), the first element
    /// of the group of temperature sensors has data while the rest are zero.

    /// accelerometer temperature sensors
    temp = gAlgorithm.rawSensors[ XATEMP ];

   /// X-Axis accelerometer temperature
    index = uint32ToBuffer(response,
                           index,
                           temp);

    /// Y-Axis accelerometer temperature (same as x-axis)
    index = uint32ToBuffer(response,
                           index,
                           temp);

    /// Z-Axis accelerometer temperature (same as x-axis)
    index = uint32ToBuffer(response,
                           index,
                           temp);

    /// Rate Sensor temperature values
    temp = gAlgorithm.rawSensors[ XRTEMP ];

    /// X-Axis rate-sensor temperature
    index = uint32ToBuffer(response,
                           index,
                           temp);

    /// Y-Axis rate-sensor temperature (same as x-axis)
    index = uint32ToBuffer(response,
                           index,
                           temp);

    /// Z-Axis rate-sensor temperature (same as x-axis)
    index = uint32ToBuffer(response,
                           index,
                           temp);

    /// The last element of the packet contains the board temperature.
    temp = gAlgorithm.rawSensors[ BTEMP ];
    index = uint32ToBuffer(response,
                           index,
                           temp);
    return index;
} /* end appendAllTempCounts */

/** ****************************************************************************
 * @name appendGpsVel
 * @brief Add GPS North East and Down velocities to message
 * @author Doug Hiranaka, 2014
 * Trace:
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendGpsVel (uint8_t  *response,
                       uint16_t index)
{
    uint16_t temp = 0;

    /// North
    temp = (uint16_t)SCALE_BY_2POW16_OVER_512(gGpsDataPtr->vNed[GPS_NORTH]);
    index = uint16ToBuffer(response,
                           index,
                           temp);
    /// East
    temp = (uint16_t)SCALE_BY_2POW16_OVER_512(gGpsDataPtr->vNed[GPS_EAST]);
    index = uint16ToBuffer(response,
                           index,
                           temp);
    /// Down
    temp = (uint16_t)SCALE_BY_2POW16_OVER_512(gGpsDataPtr->vNed[GPS_DOWN]);
    index = uint16ToBuffer(response,
                           index,
                           temp);
    return index;
} /* end appendGpsVel */


/** ****************************************************************************
 * @name appendKalmanVel
 * @brief Add North East and Down velocities from the EKF to message
 * @author t. malerich
 * Trace:
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendKalmanVel(uint8_t  *response,
                         uint16_t index)
{
    /// North
    uint16_t temp = (uint16_t)SCALE_BY_2POW16_OVER_512(gKalmanFilter.Velocity_N[GPS_NORTH]);
    index = uint16ToBuffer(response,
                           index,
                           temp);
    /// East
    temp = (uint16_t)SCALE_BY_2POW16_OVER_512(gKalmanFilter.Velocity_N[GPS_EAST]);
    index = uint16ToBuffer(response,
                           index,
                           temp);
    /// Down
    temp = (uint16_t)SCALE_BY_2POW16_OVER_512(gKalmanFilter.Velocity_N[GPS_DOWN]);
    index = uint16ToBuffer(response,
                           index,
                           temp);
    return index;
} /* end appendKalmanVel */


/** ****************************************************************************
 * @name appendGpsPos
 * @brief Add GPS Latitude, longitude and Altitude (elevation) to message
 * @author Doug Hiranaka, 2014
 * Trace:
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendGpsPos (uint8_t  *response,
                       uint16_t index)
{
  int16_t temp16 = 0;
  int32_t temp32 = 0;

    /// Longitude
    temp32 = (int32_t) SCALE_BY_2POW32_OVER_2PI(gGpsDataPtr->lonSign * gGpsDataPtr->lon * D2R);
    index = uint32ToBuffer(response,
                           index,
                           temp32);
    /// Latitude
    temp32 = (int32_t) SCALE_BY_2POW32_OVER_2PI(gGpsDataPtr->latSign * gGpsDataPtr->lat * D2R) ;
    index = uint32ToBuffer(response,
                           index,
                           temp32);
    /// Down
    temp16 = (int16_t) SCALE_BY_2POW16_OVER_2POW14(gGpsDataPtr->alt);
    index = uint16ToBuffer(response,
                           index,
                           temp16);
    return index;
} /* end appendGpsPos */


/** ****************************************************************************
 * @name appendKalmanPos
 * @brief Add EKF Latitude, longitude and Altitude (elevation) to message
 * @author t. malerich
 * Trace:
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendKalmanPos(uint8_t  *response,
                         uint16_t index)
{
    /// Longitude
    int32_t temp32 = (int32_t) SCALE_BY_2POW32_OVER_2PI(gKalmanFilter.llaDeg[LON_IDX] * D2R);
    index = uint32ToBuffer(response,
                           index,
                           temp32);
    /// Latitude
    temp32 = (int32_t) SCALE_BY_2POW32_OVER_2PI(gKalmanFilter.llaDeg[LAT_IDX] * D2R) ;
    index = uint32ToBuffer(response,
                           index,
                           temp32);
    /// altitude
    int16_t temp16 = (int16_t) SCALE_BY_2POW16_OVER_2POW14(gKalmanFilter.llaDeg[ALT_IDX]);
    index = uint16ToBuffer(response,
                           index,
                           temp16);
    return index;
} /* end appendKalmanPos */


/** ****************************************************************************
 * @name uint32ToBuffer
 * @brief formats the input word for output to a byte buffer.
 * @author Douglas Hiranaka, Jul. 2014
 * @param [in] buffer - points to output buffer being loaded.
 * @param [in] index - response[index] is where data is added.
 * @param [in] inWord - data being added.
 * @retval return the incremented index
 ******************************************************************************/
uint32_t
uint32ToBuffer(uint8_t  *buffer,
               uint16_t index,
               uint32_t inWord)
{
    buffer[index++] = (uint8_t)((inWord >> 24) & 0xff);
    buffer[index++] = (uint8_t)((inWord >> 16) & 0xff);
    buffer[index++] = (uint8_t)((inWord >>  8) & 0xff);
    buffer[index++] = (uint8_t)(inWord & 0xff);

  return index;
}

/** ****************************************************************************
 * @name uint16ToBuffer
 * @brief formats the input short integer for output to a byte buffer.
 * @author Douglas Hiranaka, Jul. 2014
 * @param [in] buffer - points to output buffer being loaded.
 * @param [in] index - response[index] is where data is added.
 * @param [in] inWord - data being added.
 * @retval return the incremented index
 ******************************************************************************/
uint32_t
uint16ToBuffer(uint8_t  *buffer,
               uint16_t index,
               uint16_t inShort)
{
    buffer[index++] = (char)((inShort >> 8) & 0xff);
    buffer[index++] = (char)(inShort & 0xff);

  return index;
}

