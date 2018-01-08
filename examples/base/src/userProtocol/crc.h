/*********************************************************************************
* File name:  crc.h
*
* File description: 
*   - header file for CRC functions
*               
*********************************************************************************/

#ifndef CRC_H
#define CRC_H

typedef uint16_t CrcCcittType;
typedef uint32_t Crc32Type;
 
#define CRC_CCITT_LENGTH	2
#define CRC_32_LENGTH       4
 
#define CRC_CCITT_INITIAL_SEED	0x1d0f
#define CRC_32_INITIAL_SEED		0xffffffff 

extern CrcCcittType CrcCcitt			(const uint8_t data [], uint16_t length, const CrcCcittType seed);
extern Crc32Type    Crc32				(const uint8_t data [], uint16_t length, const Crc32Type seed); 
extern void         CrcCcittTypeToBytes	(CrcCcittType type, uint8_t bytes []);
extern CrcCcittType BytesToCrcCcittType (const uint8_t bytes []);
extern void         Crc32TypeToBytes    (Crc32Type type, uint8_t bytes []);
extern Crc32Type    BytesToCrc32Type    (const uint8_t bytes []);

#endif

