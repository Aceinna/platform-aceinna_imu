/** ****************************************************************************
 * @file s_eeprom.c
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * @brief File description:
 *   This driver emulates a serial EEPROM by writing to internal flash.
 *  NOTE: This does a direct erase and write, if power is pulled part way through
 *  the erase cycle (~3s) or the write cycle ( <1s), the system will lose its
 *  configuration and calibration.
 *  FIXME: A future improvement would be to use two flash sectors and ping pong
 *  between them so that the important information is never lost.
 *
 * ******* IMPLEMENTATION NOTES AND SUGGESTIONS FOR MODIFICATION
 * We have a fake EERPOM that is implemented using on-chip flash. I chose the
 * uppermost flash sector. It is incredibly large but having it be the uppermost
 * means we don't run into problems as the code grows.
 * The three most important things you'll need to know:
 *  - where all the pieces are located
 *  - removing default values from being programmed
 *  - moving to a different sector.
 ******* WHERE ALL THE PIECES ARE LOCATED
 * The variable gEepromInFlash is defined in s_eeprom.c. It is a structure, meant
 * to represent the EEPROM variables in flash. Note that the EEPROM was implemented
 * from a C2xxx processor which has 16 bit bytes (see email "The tale of 16 bit
 * bytes" send to Joe and Tony on 6/24/13). Some quirkiness in the variable
 * definition is to match the oddness caused by the 16 bit bytes.
 *
 * The gEepromInFlash is defined at a fixed location by this pragma line:
 * #pragma location="FLASH_BASED_EEPROM"  // defined in the linker's ICF file
 *
 * As noted in the comment, this is defined in the linker ICF file. This is in
 * the project under board\stm32f2xx_flash.icf. The linker file is fairly odd,
 * very compiler specific. The IAR documentation gives some examples but it takes
 * awhile to build up to the complexity of the default file (which I only
 * changed a little).
 *
 * The first important thing to see is the ROM definition:
 * define symbol __ICFEDIT_region_ROM_start__ = 0x08000000;
 * define symbol __ICFEDIT_region_ROM_end__   = 0x080DFFFF; // end one sector early
 * Note the comment- this used to say "__ICFEDIT_region_ROM_end__   = 0x080FFFFF"
 * which would allocate all of the code space to the noted region.
 * This is actually done in the line
 *    place in ROM_region   { readonly };
 * which puts all of the readonly sections (i.e. compiled code) into the ROM_region.
 *
 * The main modification to the file is this section:
 * define symbol _region_FLASH_EEPROM_start__   = 0x080E0000;
 * define symbol _region_FLASH_EEPROM_end__     = 0x080FFFFF;
 * define region FLASH_EEPROM_region   = mem:[from _region_FLASH_EEPROM_start__   to _region_FLASH_EEPROM_end__];
 * define block FLASH_EEPROM_block { section FLASH_BASED_EEPROM };
 * place in FLASH_EEPROM_region { block FLASH_EEPROM_block };
 *
 * This allocated a block to belong to FLASH_EEPROM_region and puts the
 * FLASH_BASED_EEPROM section into that block/region.
 *
 ******* REMOVING DEFAULT VALUES FROM BEING PROGRAMMED
 * Once you can program the unit with proper configuration and calibration values,
 * you won't want it to revert to the defaults whenever you reprogram the code.
 *
 * To program the defaults, you have
 *
 * #pragma location="FLASH_BASED_EEPROM"  // 0x08060000 defined in the linker's ICF file
 * __root const uEeprom gEepromInFlash  = {
 *    .table = {
 *        .configuration = {
 *            .port1Usage = 1, // PRI_UCB_PORT
 *            .calibrationCRC = 53088,
 * 	},
 *	};
 *
 * To not program defaults, change the declaration of gEepromInFlash to be:
 * __root __no_init const uEeprom gEepromInFlash;
 *
 * This will make it so the memory doesn't get changed by the linker or loaded
 * to flash when programming.
 *
 ******* MOVING TO A DIFFERENT SECTOR
 * You may need to move to a different sector if the size of the flash on the
 * chip changes.
 *
 * If you need to move the EEPROM section, modify
 * _region_FLASH_EEPROM_start_ and _end_ accordingly.
 * Also modify __ICFEDIT_region_ROM_start__ and _end__.
 * (I usually find it is easiest to draw a memory map and then make the changes
 * in the linker file once I've sorted out the numbers and section sizes.)
 *
 ******************************************************************************/

#include <stdint.h>
#include <string.h> // memcpy

#include "stm32f2xx_conf.h"
#include "salvodefs.h"
#include "stm32f2xx.h"
#include "dmu.h"
#include "s_eeprom.h"
#include "port_def.h"
#include "uart.h"
#include "xbowsp_generaldrivers.h"
#include "boardDefinition.h"
#include "Indices.h"
#include "accelerometer.h"
#include "gyroscope.h"
#include "magnetometer.h"
#include "debug.h"
#include "xbowsp_fields.h"

#define CALIBRATION_OFFSET (0x200) // 0x100 = 256 in bytes

#include "watchdog.h"

#pragma pack(1)
typedef union  {
    uint8_t memory[sizeof(ConfigurationStruct) + sizeof(CalibrationStruct) +0x202];
    struct {
        uint16_t            countOfEepromErases;  // FIXME: why isn't this at element 0x0 in the memory?
        ConfigurationStruct configuration; // xbowsp_generaldrivers.h
        uint8_t             junkSpacing[CALIBRATION_OFFSET - sizeof(ConfigurationStruct)]; // to match with 440 EEPROM
        CalibrationStruct   calibration; // xbowsp_generaldrivers.h
    } table;
} uEeprom;
#pragma pack() // undo pragma pack 1

// memory map for flash
#define ERASE_EEPROM_ON_LOADING_CODE 0
//#pragma location = "FLASH_BASED_EEPROM"  ///< defined in the linker's ICF file
#if ERASE_EEPROM_ON_LOADING_CODE
const uEeprom gEepromInFlash __attribute__((section (".myEeprom"))) = {
    .table = {
        .configuration = {
            .calibrationCRC = 0,// 43007,   // 0xA7FF at

            /// ------ Configuration fields as defined in the DMU interface doc ------
            /// Set Nav-View output rate divider
            ///   0: quiet, 1: 100Hz, 2: 50Hz, 4: 25Hz, 5: 20Hz, 10: 10Hz, 20: 5Hz, ...
            .packetRateDivider = 2,

            /// Set the UCB UART baud rate
            ///   0: 9600, 1: 19200, 2: 38400, 3: 57600
            .baudRateUser = 3,

            // Set the packet type
            //.packetType = 0x4944, // id
            //.packetType = 0x5331, // scaled sensor 1
            //.packetType = 0x5330, // scaled sensor 0
            .packetType = 0x4632, // Factory raw 2
        },
        .calibration = {
            /// default
            .serialNumber = 0, //1300000010, //1300000009,  //1380999998,//0x04030201,

            /// Define the product version string
            .versionString = "IMU380ZA-200",

            /// Set the values in the calibration index
            .calibrationTableIndexA[ 0] = 0,//0x0000, // index to the first temp cal
            .calibrationTableIndexA[ 1] = 0,//0x0010,
            .calibrationTableIndexA[ 2] = 0,//0x0020,
            .calibrationTableIndexA[ 3] = 0,//0x0030,
            .calibrationTableIndexA[ 4] = 0,//0x0040,
            .calibrationTableIndexA[ 5] = 0,//0x0050,
            .calibrationTableIndexA[ 6] = 0,//0x0060,
            .calibrationTableIndexA[ 7] = 0,//0x0070,
            .calibrationTableIndexA[ 8] = 0,//0x0080,
            .calibrationTableIndexA[ 9] = 0,//0x0090,
            .calibrationTableIndexA[10] = 0,//0x0100,
            .calibrationTableIndexA[11] = 0,//0x0110,
            .calibrationTableIndexA[12] = 0,//0x0120,
            .calibrationTableIndexA[13] = 0,//0x0130,
            .calibrationTableIndexA[14] = 0,//0x0140,
            .calibrationTableIndexA[15] = 0x10,//0x0150, // index to the last (false) temp cal

            // The off-diagonal terms of the misalignment matrix
            .misalign[ 0] = 0.0,
            .misalign[ 1] = 0.0,
            .misalign[ 2] = 0.0,
            .misalign[ 3] = 0.0,
            .misalign[ 4] = 0.0,
            .misalign[ 5] = 0.0,
            .misalign[ 6] = 0.0,
            .misalign[ 7] = 0.0,
            .misalign[ 8] = 0.0,
            .misalign[ 9] = 0.0,
            .misalign[10] = 0.0,
            .misalign[11] = 0.0,
            .misalign[12] = 0.0,
            .misalign[13] = 0.0,
            .misalign[14] = 0.0,
            .misalign[15] = 0.0,
            .misalign[16] = 0.0,
            .misalign[17] = 0.0,

            /// Set the product configuration field for the DMU380
            ///   (as above top byte and bottom byte are transposed)
            .productConfiguration.bit.hasMags      = 1,   // lsb
            .productConfiguration.bit.hasGps       = 0,
            .productConfiguration.bit.algorithm    = 0,
            .productConfiguration.bit.extAiding    = 0,
            .productConfiguration.bit.architecture = 1,
            .productConfiguration.bit.rsvd         = 0,
            .productConfiguration.bit.isADAHRS     = 0,   // msb

            .AccelSensorRange = ACCEL_RANGE_4G, // [4]
            .GyroSensorRange  = DEFAULT_GYRO_RANGE, // [250]
            .MagSensorRange   = MAG_RANGE_4000_MILLI_GA, // [4000]
        },
    },
} ;
#else
const uEeprom gEepromInFlash;
//__root __no_init const uEeprom gEepromInFlash;
#endif /* ERASE_EEPROM_ON_LOADING_CODE */

static uEeprom gEepromRamShadow;

/** ***************************************************************************
 * @name _configurationOffset() LOCAL returns address offset of the
         configuration to the start of the flash region
 * @brief
 *
 * @param [in] N/A
 * @retval offset
 ******************************************************************************/
static uint16_t _configurationOffset() {
    int* p1 = (int*) &gEepromInFlash;
    int* p2 = (int*) &(gEepromInFlash.table.configuration);

    return (uint16_t) ((unsigned int) p2 - (unsigned int) p1) ;
}

/** ***************************************************************************
 * @name readEEPROMCalOffsetAndLength() LOCAL returns address offset of the
         configuration to the calibration fields
 * @brief
 *
 * @param [in] offset - address offset between the regions
 * @param [in] length - static defined length
 * @retval N/A
 ******************************************************************************/
void readEEPROMCalOffsetAndLength(uint16_t *offset,
                                  uint16_t *length)
{
    int* p1 = (int*) &(gEepromInFlash.table.configuration);
    int* p2 = (int*) &(gEepromInFlash.table.calibration);

    *offset = ((uint16_t) ((unsigned int) p2 - (unsigned int) p1)) >> 1 ;
    *length = 0x61d;
}

#define ERROR    1
#define NO_ERROR 0

/** ***************************************************************************
 * @name _s_eepromWrite() LOCAL write 8-bit byte stream into simulated EEPROM.
 *       Since this writes the whole sector, there are no offsets or sizes;
 *       changes should be made to gEepromRamShadow before calling this function.
 * @brief
 *
 * @param [in] N/A
 * @retval error (1), no error (0)
 ******************************************************************************/
BOOL _s_eepromWrite()
{
    uint16_t                  offset = 0;
    int16_t                   num    = sizeof(gEepromInFlash);
    static volatile uint32_t  status = FLASH_COMPLETE;
    uint32_t                  start  = (unsigned int) &gEepromInFlash;

    PetWatchdog(); // Reloads timer with counter value in the reload register

    status = FLASH->SR;
    if ((status & 0xFE) != 0) {
        FLASH_ClearFlag(status & 0xFE);
    }

    OSDisableHook();
    FLASH_Unlock(); // this should be sufficient disabling the interrupts
    // stops the system. This access is only us long.
    status = FLASH_EraseSector(EEPROM_FLASH_SECTOR, EEPROM_FLASH_VOLTAGE);
    gEepromRamShadow.table.countOfEepromErases++;
    while (num > 0) {
        uint32_t *data = (uint32_t *) &gEepromRamShadow.memory[offset];
        status = FLASH_ProgramWord(start+offset, *data); // stm32fxx_flash.c
        offset += 4;
        num    -= 4;
        PetWatchdog(); // Reloads timer with counter value in the reload register
    }
    FLASH_Lock();
    OSEnableHook();
    PetWatchdog(); // Reloads timer with counter value in the reload register

    return NO_ERROR;
} /* end _s_eepromWrite() */


/** ***************************************************************************
 * @name writeEEPROMByte() write 8-bit byte stream into EEPROM:
 * @brief eeprom is written with addresses that were developed with the xbow
 *       protocol.
 * @param [in] addr - startAdd: offset address to start writing;
 * @param [in] num -  number of the 8-bit bytes
 * @param [in] source *buffer: pointer to the 8-bit byte buffer to be written;
 * @retval error (1), no error (0)
 ******************************************************************************/
BOOL writeEEPROMByte(uint16_t addr,
                     uint16_t num,
                     void     *source)
{
    const uint8_t kEraseValue = 0xFF;
    uint8_t       *src        = (uint8_t*) source;
    uint8_t       changed     = FALSE;
    uint8_t       eraseOnly   = TRUE;
    uint16_t      maxSize     = sizeof(gEepromInFlash);
    uint16_t      offset      = _configurationOffset();
    int           i;

    PetWatchdog(); // Reloads timer with counter value in the reload register
    memcpy(&gEepromRamShadow, &gEepromInFlash, sizeof(gEepromRamShadow));

    addr   = addr * SIZEOF_WORD;
    offset += addr;

    if ((offset + num) > maxSize)
    {
        DEBUG_STRING(" outside\r\n");
        return NO_ERROR; // it is ok for it to try to write outside the bounds
        // it is just setting the old EEPROM to values that it shouldn't...
    }

    for (i = 0; i < num; i++) {
        if (gEepromRamShadow.memory[offset + i] != src[i]) {
            gEepromRamShadow.memory[offset + i] = src[i];
            changed = TRUE;
        }
        if (src[i] != kEraseValue) {
            eraseOnly = FALSE;
        }
    }

    if ( eraseOnly ) {
        if (changed) {
            memset(&gEepromRamShadow, 0xFF, sizeof(gEepromRamShadow));
        } else {
            return NO_ERROR;
        }
    }
    /// if something changed, erase flash and write new data
    if ( changed ) {
        return _s_eepromWrite();
    }
    PetWatchdog(); // Reloads timer with counter value in the reload register

    return NO_ERROR;
} /* end writeEEPROMByte() */

/** ***************************************************************************
 * @name writeEEPROMWords() write multiple words into the EEPROM.
 * @brief byte swaps the source before writing to match Nav-View.
 * Trace:
 * [SDD_WRITE_EEPROM_WORDS_01 <-- SRC_WRITE_EEPROM_WORDS]
 * [SDD_WRITE_EEPROM_WORDS_02 <-- SRC_WRITE_EEPROM_WORDS]
 * [SDD_WRITE_EEPROM_WORDS_03 <-- SRC_WRITE_EEPROM_WORDS]
 *
 * @param [in] addr - the xbow 16-bit word EEPROM address to write to.
 * @param [in] num -  the number of 16-bit fields to write.
 * @param [in] source points to the beginning of an array of data to be written.
 * @retval  FALSE: writing fails; TRUE: writing succeeds.
 ******************************************************************************/
BOOL writeEEPROMWords(uint16_t addr,
                      uint16_t num,
                      void     *source)
{
    BOOL     ret;
    int      i;
    uint16_t *src;
    uint16_t swap;
    uint16_t addrTemp;
    uint16_t  j;         // increment the src array (16-bit byte)
    uint8_t  count = 0; // which half of the 16-bit word (which byte) the data
                        //   is saved to.
    addrTemp = addr;

    PetWatchdog(); // Reloads timer with counter value in the reload register
    // swap bytes
    src = (uint16_t*) source;
    for (i = 0; i < num; i++)
    {
        /// Convoluted logic added to handle the conversion from a 16-bit word to
        ///   an 8-bit byte containing the version string (a char). This applies
        ///   only for the memory locations between 0x0102 and 0x0142. 0x204-0x284
        ///
        /// Note: the way the system loads the initial data onto the unit affects
        ///       if the byte/word needs to be swapped.  For instance, the initial
        ///       configuration is loaded using a 'set-field' command.  Upon a mag-
        ///       align, the unit uses this function to reload the configuration.
        ///       However, the bytes do not get swapped.  The reason that addresses
        ///       between 0x0000 and 0x0100 are not affected by the following if-
        ///       statements.  Any use of the writeEEPROM command may necessitate
        ///       testing to ensure this logic works properly.

      // FIXME: this makes no sense the word offsets need to be muliplied by 2
      // this address range is in the reserved area.
        if( ( addrTemp >= 0x0102 && addrTemp < 0x0142 ) )
        {
            // This section of code applies to the version-string part of the EEPROM
            swap = src[i];
            /// Reset the location variables upon entering the versionString
            ///    location in memory
            if( addrTemp == 0x0102 )
            {
                count = 0;
                j     = i;
            }

            /// If the first byte of a 16-bit word, then shift the byte to the right
            if( count == 0 )
            {
                src[j] = ( ( swap >> 8 ) & 0xFF ); //( swap << 8 );
                count = 1;
            }
            else if( count == 1 )
            { // FIXME: this makes no sense you just shifted the upper byter into the lower position
                src[j] = src[j] | swap;
                j++;
                count = 0;
            }
        } else if( ( addrTemp >= 0x0100 && addrTemp < 0x0102 ) || addrTemp >= 0x0142 ) {
            swap   = src[i];
            swap   = (swap << 8) | (swap >> 8); // endianess
            src[i] = swap;
        }
        addrTemp++;
        PetWatchdog(); // Reloads timer with counter value in the reload register
    }

    // Does not change (byte swap) data in EEPROM addresses 0x0000 to 0x0102
    ret = writeEEPROMByte(addr,
	                      num * SIZEOF_WORD,
						  source);

    return ret;
}  /* end writeEEPROMWords() */

/** ***************************************************************************
 * @name readEEPROMWords() wrapper - read multiple-words from the EEPROM.
 * @brief
 * Trace:
 * [SDD_READ_EEPROM_WORDS_01 <-- SRC_READ_EEPROM_WORDS]
 * [SDD_READ_EEPROM_WORDS_02 <-- SRC_READ_EEPROM_WORDS]
 *
 * @param [in] addr - the 16-bit word EEPROM address to read from.
 * @param [in] num -  the number of 16-bit words to write.
 * @param [in] *destination: points to the beginning of an array where the
 *             EEPROM data will be put.
 * @retval  *   FALSE: writing fails; TRUE: writing succeeds.
 ******************************************************************************/
void readEEPROMWords(uint16_t addr,
                     uint16_t num,
                     void     *destination)
{
    readEEPROMByte(addr,
	               num * SIZEOF_WORD,
				   destination);
} /* end readEEPROMWords() */

/** ***************************************************************************
 * @name readEEPROMByte() read bytes from the EEPROM.
 * @brief
 * Trace:
 * [SDD_READ_EEPROM_TWO_WORDS_01 <-- SRC_READ_EEPROM_TWO_WORDS]
 * [SDD_READ_EEPROM_TWO_WORDS_02 <-- SRC_READ_EEPROM_TWO_WORDS]
 * [SDD_READ_EEPROM_TWO_WORDS_03 <-- SRC_READ_EEPROM_TWO_WORDS]
 *
 * @param [in] addr - the 16-bit word EEPROM address to read from.
 * @param [in] num -  the number of bytes to read.
 * @param [in] *destination: points to the beginning of an array where the
 *             EEPROM data will be put.
 * @param [out] *destination - data in destination
 * @retval    FALSE: writing fails; TRUE: writing succeeds.
 ******************************************************************************/
void readEEPROMByte(uint16_t addr,
                    uint16_t num,
                    void     *destination)
{
    uint16_t offset  = _configurationOffset();
    uint16_t maxSize = sizeof(gEepromInFlash);
    uint8_t  *dst    = (uint8_t*)destination;
    uint8_t  *src;
    uint8_t  temp1; ///< byte-swap elements for a Nav-View/IMU Test read

    PetWatchdog(); // Reloads timer with counter value in the reload register

    addr = addr * SIZEOF_WORD; // 16 bit bytes in 440 to 8 bit bytes here
    memset(dst, 0, num);

    if (num > maxSize) {
        return;
    }

    offset += addr;

    src = (uint8_t*)&gEepromInFlash.memory[offset];
    while (num) { // Calibration index and table (including the product configuration section)
        if( ( addr >= 0x0142 * SIZEOF_WORD ) && ( addr < 0x071D * SIZEOF_WORD ) )
        {   // Byte-swap
            temp1 = *src;
            src++;

            *dst = *src;
            dst++;
            *dst = temp1;

            src++;
            dst++;
            num -= 2;
        }
        else
        {   // Original code - no swap
            *dst = *src;
            src++;
            dst++;
            num--;
            *dst = *src;
            src++;
            dst++;
            num--;
        }
        addr++;
        addr++;
        PetWatchdog(); // Reloads timer with counter value in the reload register
    }
} /*end readEEPROMByte() */


// individual getters
/** ***************************************************************************
 * @name readEEPROMSerialNumber() read serial number from the EEPROM.
 *
 * @param [out] *destination - pointer to buffer to return data to
 * @retval  N/A
 ******************************************************************************/
void readEEPROMSerialNumber(void *destination)
{
    uint32_t * sn;
    sn  = destination;
    *sn = gEepromInFlash.table.calibration.serialNumber;
}

/** ***************************************************************************
 * @name readEEPROMProdConfig() read product configuration from the EEPROM.
 *
 * @param [out] *destination - pointer to buffer to return data to
 * @retval  N/A
 ******************************************************************************/
void readEEPROMProdConfig(void *destination)
{
    union  ProductConfiguration *pc;
    pc  = destination;
    *pc = gEepromInFlash.table.calibration.productConfiguration;
}

/** ***************************************************************************
 * @name readEEPROMCalibration() read calibration from the EEPROM.
 *
 * @param [out] *destination - pointer to buffer to return data to
 * @retval  N/A
 ******************************************************************************/
void readEEPROMCalibration(void* destination)
{
    PetWatchdog(); // Reloads timer with counter value in the reload register
    memcpy( destination,
	        &gEepromInFlash.table.calibration,
            sizeof(gEepromInFlash.table.calibration) );
    PetWatchdog(); // kick the dog
}

/** ***************************************************************************
 * @name readEEPROMConfiguration() read configuration from the EEPROM.
 *
 * @param [out] *destination - pointer to buffer to retuen data to
 * @retval  N/A
 ******************************************************************************/
void readEEPROMConfiguration(void* destination)
{
    PetWatchdog(); // Reloads timer with counter value in the reload register
    memcpy( destination,
	        &gEepromInFlash.table.configuration,
            sizeof(gEepromInFlash.table.configuration) );
    PetWatchdog(); // kick the dog
}

/** ***************************************************************************
 * @name setJumpFlag() set the value 0x12345678 at address 0x8007fffc to indicate it is needed to upgrade.
 *
 * @param [dat] 
 * @retval  N/A
 ******************************************************************************/
BOOL    setJumpFlag(uint32_t dat)
{
    static volatile uint32_t  status = FLASH_COMPLETE;
	int						  retry_cnt = 0;
	uint32_t 				  read_dat = 0;

    PetWatchdog(); // Reloads timer with counter value in the reload register

    status = FLASH->SR;
    if ((status & 0xFE) != 0) {
        FLASH_ClearFlag(status & 0xFE);
    }

    OSDisableHook();
    FLASH_Unlock(); // this should be sufficient disabling the interrupts
    // stops the system. This access is only us long.
    status = FLASH_EraseSector(FLASH_Sector_1, EEPROM_FLASH_VOLTAGE);

    do{
        status = FLASH_ProgramWord(APP_UPDATE_FLAG_ADDR, dat); // stm32fxx_flash.c
        PetWatchdog(); // Reloads timer with counter value in the reload register
    }while(status !=FLASH_COMPLETE && (++retry_cnt)<3);
    FLASH_Lock();
    OSEnableHook();
    PetWatchdog(); // Reloads timer with counter value in the reload register

	if(status != FLASH_COMPLETE)
	{
		return ERROR;
	}
	read_dat = *((uint32_t *)APP_UPDATE_FLAG_ADDR);
	if(dat != read_dat)
		return ERROR;
	
    return NO_ERROR;
}




