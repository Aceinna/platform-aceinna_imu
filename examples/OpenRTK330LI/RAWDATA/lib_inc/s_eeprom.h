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
#ifndef S_EEPROM_H
#define S_EEPROM_H

#include "GlobalConstants.h" 

#define APP_USER_CONFIG_ADDR    0x080C0000
#define BOOT_FLAG_ADDR          0x080A0000
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
extern BOOL setJumpFlag(uint32_t dat);
extern void setBORLevel(float voltage);
extern BOOL programDefaultConfig();
extern uint8_t *getEEPROMCalPartitionData(int idx, uint16_t *length);
extern BOOL readFromEEPROMCalPartition( uint16_t offset, uint16_t num, void  *destination);
extern BOOL writeToEEPROMCalPartition(uint16_t offset, uint16_t num, void *source);
extern uint8_t *getEEPROMCalTabPtr(int idx);
extern BOOL writeFlash(uint32_t addr,uint8_t *buf, uint16_t len);
uint8_t getJumpFlag();


#endif /* S_EEPROM_H */ 


