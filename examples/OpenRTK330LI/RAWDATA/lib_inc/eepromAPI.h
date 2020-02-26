/** ***************************************************************************
 * @file s_eeprom.h legacy functions from older systems that used eeprom
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef _EEPROM_API_H
#define _EEPROM_API_H

#include "GlobalConstants.h" 

#define APP_STACK_START         0x2000FF00
#define EEPROM_APP_UPDATE_ADDR	0x0801FFF8
#define	ENTER_BOOTLOADER_FLAG	0x12345678
#define	APP_SIGNATURE	        0x55AA1234


#define ENTER_APP_ADDR          0x080A0004
#define	ENTER_APP_FLAG	        0X55AA1234
#define	UPDATE_APP_FLAG	        0X50A01030

#define EEPROM_APP_PAGE1        16  // 
#define EEPROM_USER_PAGE        56  // 2K
#define EEPROM_CONFIG_PAGE      57  // 2K
#define EEPROM_CAL_PAGE1        58  // and 59   4K
#define EEPROM_CAL_PAGE2        60  // and 61   4K
#define EEPROM_CAL_PAGE3        62  // and 63   4K
// #define EEPROM_CAL_ADDR1        0x0801d000
// #define EEPROM_CAL_ADDR2        0x0801e000
// #define EEPROM_CAL_ADDR3        0x0801f000
// #define EEPROM_CONFIG_ADDR      0x0801c800
#define EEPROM_CAL_ADDR1        0x080Ed000
#define EEPROM_CAL_ADDR2        0x080Ee000
#define EEPROM_CAL_ADDR3        0x080Ef000
#define EEPROM_CONFIG_ADDR      0x080Ec800
#define CAL_PARTITION_SIZE      4096        // 4K
#define CAL_CRC_OFFSET          4092
#define EEPROM_USER_ADDR        0x0801c000
#define EEPROM_PAGE_SIZE        0x800
#define BOOT_SIGNATURE_ADDR     0x2004ff10  
#define APP_START_ADDR             0x08010000
#define APP_LAST_ADDR              0x0801C7FF
#define APP_MAX_SIZE               0x00014800

#define APP_SIGNATURE_ADDR         0x080A0000
#define APP_SIGNATURE_ADDR1        0x08008190
#define APP_SIGNATURE_ADDR2        0x08008198
#define APP_SIGNATURE_ADDR3        0x080081A0
#define APP_SIGNATURE_ADDR4        0x080081A4
#define APP_SIGNATURE_ADDR5        0x080081A8
#define APP_SIGNATURE_ADDR6        0x080081AC

#define APP_SIGNATURE_OFFSET3      0x01A0
#define APP_SIGNATURE_OFFSET4      0x01A4
#define APP_SIGNATURE_OFFSET5      0x01A8
#define APP_SIGNATURE_OFFSET6      0x01AC


#define	APP_SIGNATURE3	           0x01234567
#define	APP_SIGNATURE4	           0x89ABCDEF
#define	APP_SIGNATURE5	           0xAA55AA55
#define	APP_SIGNATURE6	           0x55AA55AA




//#define EEPROM_APP_UPDATE_PAGE  63

extern BOOL needUpdate, forcedBootMode, spiBootMode;



extern void readEEPROMWords(uint16_t addr, uint16_t num, void *destination) ;
extern BOOL writeEEPROMWords(uint16_t addr, uint16_t num, void *source) ;
extern void readEEPROMByte(uint16_t addr, uint16_t num, void *destination) ;
extern BOOL writeEEPROMByte(uint16_t addr, uint16_t num, void *source) ;

extern void readEEPROMSerialNumber(void *destination) ;
extern void readEEPROMProdConfig(void *destination) ;
extern void readEEPROMCalAddrAndLength(int idx, uint32_t *addr, int *length);
void readEEPROMCalOffsetAndLength(int idx, uint16_t *offset, uint16_t *length);

extern void readEEPROMCalibration(int idx, void* destination);
extern void readEEPROMConfiguration(void* destination);
extern void setBORLevel(float voltage);
extern bool programDefaultConfig();
extern uint8_t *getEEPROMCalPartitionData(int idx, uint16_t *length);
extern BOOL readFromEEPROMCalPartition( uint16_t offset, uint16_t num, void  *destination);
extern BOOL writeToEEPROMCalPartition(uint16_t offset, uint16_t num, void *source);
extern BOOL writeToEEPROMAppPartition(uint32_t offset, uint16_t num, void *source);
extern uint8_t *getEEPROMCalTabPtr(int idx);
extern BOOL calSectorsLocked();
extern BOOL lockCalSectors(void);
extern BOOL unlockCalSectors(void);
extern BOOL ApplyAppSignature(BOOL bootMode);
extern BOOL appStartedFirstTime(void);
extern BOOL IsNeedToUpdateApp();
extern BOOL HW_IsBootModeEnforced();
extern BOOL HW_IsAppModeEnforced();
extern void HW_EnforceAppMode();
extern void HW_HDTestMode();
extern void HW_ClearBootSignature();
extern bool IsNeedToHardwareTest();
#endif /* S_EEPROM_H */ 


