/** ***************************************************************************
 * @file xbowsp_init.c Initialization for UCB's Comm. and Cal.
 * @Author dan
 * @date   2011-02-09 22:02:39 -0800 (Wed, 09 Feb 2011)
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 * @rev 17479
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @version
 * DKH 10.02.14 set sensor range based on EEPROM config
 *****************************************************************************/
#include <string.h>
#include <stdint.h>

#include "stm32f2xx_conf.h"
#include "sensor.h"
#include "s_eeprom.h"
#include "timer.h"
#include "xbowsp_algorithm.h"
#include "xbowsp_generaldrivers.h"
#include "xbowsp_init.h"
#include "calc_airData.h"
#include "ucb_packet.h"
#include "extern_port_config.h"
#include "xbowsp_fields.h"
#include "accelerometer.h"
#include "gyroscope.h"
#include "magnetometer.h"
#include "ucb_packet.h"
#include "taskUserCommunication.h" // com type

CalibrationStruct   gCalibration;
ConfigurationStruct gConfiguration;
AlgorithmStruct     gAlgorithm;

// GPS get set INLINE functions
BOOL       IsInternalGPS(void)                    { return gCalibration.productConfiguration.bit.hasGps;}
int16_t ConfigurationBaudRateGps(void)         { return gConfiguration.baudRateGPS; }
void    SetConfigurationBaudRateGps(int16_t b) { gConfiguration.baudRateGPS = b; }
int16_t ConfigurationProtocolGPS(void)         { return gConfiguration.protocolGPS; }
void    SetConfigurationProtocolGPS(int16_t p) { gConfiguration.protocolGPS = p; }
void    SetAlgorithmUseDgps(BOOL d)            { gAlgorithm.bitStatus.hwStatus.bit.noDGPS = d; }

// placholders for Nav_view compatibility
softwareVersionStruct dupFMversion;  /// 525 digital processor DUP code base
softwareVersionStruct ioupFMversion; /// 525 input output processor IOUP code base
softwareVersionStruct bootFMversion; /// bootloader code base

extern int calibrateTableValid;

/** ****************************************************************************
 * @name: CrcCcittType helper for CRCing 16 bit values
 * TRACE:
 *      [SDD_EEPROM_CRC_METHOD <-- SRC_CRC_LOAD_EE]
 * @param [in] v - input value
 * @param [in] seed - crc seed
 * @retval rx - data read at the address crc.c, handle_packet.c
 ******************************************************************************/
static CrcCcittType
initCRC_16bit(uint16_t     v,
              CrcCcittType seed)
{
    uint8_t c[2];

    /// unpack 16 bit into array of 8's
    c[0] = (uint8_t)((v >> 8) & 0xFF);
    c[1] = (uint8_t)(v & 0x00FF);
    return CrcCcitt(c, 2, seed);
}

#if 0
/** ****************************************************************************
 * @name: initAlgStruct Initialize the algorithm structure NOT CALLED
 * TRACE:
 *      [SDD_INIT_ALG_STRUCT <-- SRC_INIT_ALG_STRUCT]
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void initAlgStruct(void) {

    memset(&gAlgorithm, 0, sizeof(AlgorithmStruct));
    gAlgorithm.calState = MAG_ALIGN_STATUS_IDLE;
}
#endif

/** ****************************************************************************
 * @name: _readConfigAndCalIntoMem LOCAL Read configuration from EEPROM into RAM
 * TRACE:
 * [SDD_INIT_CONFIGURATION_ADAHRS <-- SRC_READ_CONFIGURATION_AND_CALIBRATION_INTO_MEMORY]
 * [SDD_EEPROM_INIT_READ <-- SRC_READ_CONFIGURATION_AND_CALIBRATION_INTO_MEMORY]
 * [SDD_INIT_MISALIGN_ADAHRS <-- SRC_READ_CONFIGURATION_AND_CALIBRATION_INTO_MEMORY]
 * @param N/A
 * @retval N/A
 ******************************************************************************/
static void _readConfigAndCalIntoMem (void)
{
    readEEPROMCalibration(&gCalibration); // s_eeprom.c
    readEEPROMConfiguration(&gConfiguration); // s_eeprom.c
}

/** ****************************************************************************
 * @name _CheckCalibrationCrc LOCAL Verifies the stored calibration CRC matches
 *       the computed CRC
 * TRACE: [SDD_EEPROM_CRC_METHOD <-- SRC_CHECK_CALIBRATION_CRC]
 *        [SDD_EEPROM_CRC_DATA <-- SRC_CHECK_CALIBRATION_CRC]
 * @param N/A
 * @retval boolean indicating success or failure of CRC of calibration area
 ******************************************************************************/
static BOOL _CheckCalibrationCrc (void)
{
    unsigned int i;
    uint16_t     eepromWord;
    uint16_t     offset;
    uint16_t     length;
    CrcCcittType testCRC = CRC_CCITT_INITIAL_SEED;

    // offset 0x100 = 256, length 0x61d = 1565
    readEEPROMCalOffsetAndLength(&offset, &length);
    /// CRC valid EEPROM range to check
    for (i = offset; i < length + offset; i++) { // 1309
        /// read word, swap the bytes, add it to the CRC
        readEEPROMByte(i, 2, &eepromWord); // read 2 bytes put in eepromWord
        eepromWord = (eepromWord << 8) | (eepromWord >> 8);
        testCRC    = initCRC_16bit(eepromWord, testCRC); // 0xe729 = 59177
    }

    if (testCRC == gConfiguration.calibrationCRC) { // 0x29e7 = 10727
        return TRUE;
    } else {
        return FALSE;
    }
}

/** ****************************************************************************
 * @name _ProcessCalibrationData Calibration data processing
 * TRACE: [SDD_INIT_CALTABLE_DEFAULT_SIZE_ADAHRS <-- SRC_PROCESS_CALIBRATION_DATA]
 *   [SDD_INIT_CALTABLE_INVALID <-- SRC_PROCESS_CALIBRATION_DATA]
 *	[SDD_INIT_IF_ADAHRS_01 <-- SRC_PROCESS_CALIBRATION_DATA]
 *	[SDD_INIT_IF_ADAHRS_02 <-- SRC_PROCESS_CALIBRATION_DATA]
 *	[SDD_INIT_CALTABLE_IEEE2TI_ADAHRS <-- SRC_PROCESS_CALIBRATION_DATA]
 *   [SDD_INIT_SWAP_16_BITS <-- SRC_PROCESS_CALIBRATION_DATA]
 *   [SDD_INIT_G_SENS_CHECK <-- SRC_PROCESS_CALIBRATION_DATA]
 *   [SDD_WATCHDOG <-- SRC_PROCESS_CALIBRATION_DATA]
 * @param N/A
 * @retval N/A
 ******************************************************************************/
static int _ProcessCalibrationData (void)
{
    unsigned int i;
    int          invalidCalTableA = 0;
    uint16_t     tempLastLUTRstIndexA[N_TABLES_A + 1]; // [15 + 1]

    /// set initial lookup index
    for (i = 0; i < (N_TABLES_A + 1); i++) { // N_TABLES_A = 15
        tempLastLUTRstIndexA[i] = gCalibration.calibrationTableIndexA[i];
    }
    /// simple check if the EEPROM is blank or cal table index is invalid
    for (i = 0; i < (N_TABLES_A + 1); i++) {
        if ( tempLastLUTRstIndexA[i] > N_ROWS_TABLE_A ) { // 15 tables 360 rows
            invalidCalTableA = 1;
            break;
        }
    }
#if 0
    for (i = 1; i < (N_TABLES_A + 1); i++) {
        // check for only a single element in the table - calibration error
        if ( (  gCalibration.calibrationTableIndexA[i] -  gCalibration.calibrationTableIndexA[i - 1]) < 2 ) {
            invalidCalTableA = 1;
            break;
        }
    }
#endif

    if ( invalidCalTableA ) {
        /// flag cal data error
//        gAlgorithm.bitStatus.swDataBIT.bit.calibrationCRCError = TRUE;

        for (i = 0; i < (N_TABLES_A + 1); i++) {
            tempLastLUTRstIndexA[i] = 2 * i;
            gCalibration.calibrationTableIndexA[i] = tempLastLUTRstIndexA[i];
        }
    }

    kick_dog();  /// in case it takes too long
    
    return invalidCalTableA;
}



static uint8_t sysRange;
uint8_t getSensorRange(){ return sysRange; };

/** ****************************************************************************
 * @name initConfigureUnit initializes the data structures and configurations
 *     that are read from EEPROM for DUP software to run on ADAHRS platform
 * TRACE: * [SDD_INIT_CONFIGURATION_ADAHRS <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_EEPROM_INIT_READ <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_MISALIGN_ADAHRS <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_EEPROM_CRC_DATA <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_EEPROM_CRC_ADAHRS <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_BIT_LIMITS_EEPROM <-- SRC_INIT_CONFIGURE_UNIT]
 *
 * [SDD_CAL_G_DATA_CHECK <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_CONFIGURATION_ORIENT_VALID <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_CFG_PORT_DEF_01 <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_CFG_PORT_DEF_02 <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_CONFIGURATION_DEFAULT_BARO <-- SRC_INIT_CONFIGURE_UNIT]
 *
 * [SDD_INIT_RPY_OFFSETS_EXTEND <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_RPY_OFFSETS <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_EXT_MAG_CONFIG <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_DUP_SW_VERSION_ADAHRS <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_BOOTLOADER_SW_VERSION <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_WATCHDOG <-- SRC_INIT_CONFIGURE_UNIT]
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void initConfigureUnit(void)
{
    char* pString;
    sysRange = _200_DPS_RANGE;

    /// reads EEPROM -> gConfiguration and gCalibration
    _readConfigAndCalIntoMem(); // xbowsp_init.c

    // Why is this hardcoded to 115200?  Needed for CAN?
    //gConfiguration.baudRateUser = BAUD_115200;

    // Read the versionString.  If it is an INS then do nothing, else
    //   set hasGPS to FALSE.  This is to prevent the unit from being configured
    //   as an INS after the memory has been cleared (all memory locations
    //   are set to one at this point so it will be interpreted as hasGps, ...).
    //   If INS with internal GPS, the system will try to communicate with the
    //   Origin GPS and never time out.  DEBUG: There should be a time-out in
    //   the logic!!!
    pString = strstr(gCalibration.versionString, "INS");
    if ( !pString ) {
        gCalibration.productConfiguration.bit.hasGps = FALSE;
    }

    /// CRC the calibration area and set BIT appropriately
    if (_CheckCalibrationCrc() == TRUE) {
        gAlgorithm.bitStatus.swDataBIT.bit.calibrationCRCError = FALSE;
    } else {
//        gAlgorithm.bitStatus.swDataBIT.bit.calibrationCRCError = TRUE;
    }

    calibrateTableValid = _ProcessCalibrationData(); /// LOCAL process indices and data

    /// check user orientation field for validity and set defaults based on com
    //  type if not valid xbow_fields.c
    if (CheckOrientation(gConfiguration.orientation.all) == FALSE) {
        if( getUserCommunicationType() == UART_COMM ) {
            gConfiguration.orientation.all = 0;
        } else { // SPI
            gConfiguration.orientation.all = 0x6b;
        }
    }

    /// check port configuration fields against rules
	// xbow_fields.c
    if (ValidPortConfiguration(&gConfiguration) == FALSE) {
        DefaultPortConfiguration();
    }

    /// check baro correction: set to standard if the reading from EEPROM is invalid
	// xbow_fields.c
    if (CheckBaroCorrection((int32_t)gConfiguration.baroCorrection) == FALSE) {
        gConfiguration.baroCorrection = 29.92 * BARO_CORRECTION_SCALING_IN_EEPROM; // * 1000
    }

    // used to check for values
    // DKH 10.02.14 always set limits based on loaded config
    gCalibration.MagSensorRange   = MAG_RANGE_4000_MILLI_GA;
    sysRange = UcbGetSysRange(); // from system config
    switch (sysRange) {
        case _200_DPS_RANGE: // same as default
            gCalibration.AccelSensorRange = ACCEL_RANGE_4G;
            gCalibration.GyroSensorRange  = GYRO_RANGE_250DPS;
            break;

        case _400_DPS_RANGE:
            gCalibration.AccelSensorRange = ACCEL_RANGE_8G;
            gCalibration.GyroSensorRange  = GYRO_RANGE_500DPS;
            break;

        case _1000_DPS_RANGE:
            gCalibration.AccelSensorRange = ACCEL_RANGE_8G;
            gCalibration.GyroSensorRange  = GYRO_RANGE_1000DPS;
            break;

        default:
            gCalibration.AccelSensorRange = ACCEL_RANGE_4G;
            gCalibration.GyroSensorRange  = GYRO_RANGE_250DPS;
    }

    gCalibration.RollIncidenceLimit  = ROLL_INCIDENCE_LIMIT; // 0x1000
    gCalibration.PitchIncidenceLimit = PITCH_INCIDENCE_LIMIT; // 0x1000
    gCalibration.HardIronLimit       = HARD_IRON_LIMIT; // 0x6666
    gCalibration.SoftIronLimit       = SOFT_IRON_LIMIT; // 0x4000

    dupFMversion.major = VERSION_MAJOR;
    dupFMversion.minor = VERSION_MINOR;
    dupFMversion.patch = VERSION_PATCH;
    dupFMversion.stage = VERSION_STAGE;
    dupFMversion.build = VERSION_BUILD;

    kick_dog();
}
void SetIntVectorOffset(u32 offset)
{
	 NVIC_SetVectorTable(NVIC_VectTab_FLASH, offset);   
}

/*end void initConfigureUnit(void) */
