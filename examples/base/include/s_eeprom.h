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

extern void readEEPROMWords(uint16_t addr, uint16_t num, void *destination) ;
extern BOOL writeEEPROMWords(uint16_t addr, uint16_t num, void *source) ;
extern void readEEPROMByte(uint16_t addr, uint16_t num, void *destination) ;
extern BOOL writeEEPROMByte(uint16_t addr, uint16_t num, void *source) ;

extern void readEEPROMSerialNumber(void *destination) ;
extern void readEEPROMProdConfig(void *destination) ;
extern void readEEPROMCalOffsetAndLength(uint16_t *offset, uint16_t *length);

extern void readEEPROMCalibration(void* destination);
extern void readEEPROMConfiguration(void* destination);
extern BOOL    setJumpFlag(uint32_t dat);
#endif /* S_EEPROM_H */ 


