/** ***************************************************************************
 * @file crc.c  functions for computing CRC's
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
#include "stm32f2xx.h"
#include "crc.h"

#define BITS_PER_BYTE	8

#define ONE_BYTE	(BITS_PER_BYTE)
#define TWO_BYTES	(2 * BITS_PER_BYTE)
#define THREE_BYTES	(3 * BITS_PER_BYTE)

#define BYTE_0	0x000000ff
#define BYTE_1  0x0000ff00
#define BYTE_2	0x00ff0000
#define BYTE_3	0xff000000

#define	MSB		0x80

static const uint8_t CRC_CCITT_POLY [] = { 0x10, 0x21 };
static const uint8_t CRC_32_POLY []    = { 0xed, 0xb8, 0x83, 0x20 };

/** ****************************************************************************
 * @name Crc perform crc calculation on the input data
 * @brief
*
* Input parameters:
*
* @param [out] crc - the calulated value
* @retval N/A
********************************************************************************/
static void Crc (const uint8_t poly [],
                 uint8_t       polyLength,
                 const uint8_t seed [],
                 const uint8_t data [],
                 uint16_t      dataLength,
                 uint8_t       crc [])
{
	uint16_t dataIndex;
	uint8_t  crcIndex;
	uint8_t  bitCount;

	/// load seed
	for (crcIndex = 0; crcIndex < polyLength; ++crcIndex) {
		crc[crcIndex] = seed[crcIndex];
	}

	/// step through all bytes of data
	for (dataIndex = 0; dataIndex < dataLength; ++dataIndex) {
		/// next data byte is XORed into the most-significant byte of the CRC
		crc[0] ^= data[dataIndex];

		/// index through all bits of current data byte
		for (bitCount = 0; bitCount < BITS_PER_BYTE; ++bitCount) {
			/// top CRC bit = 1 -> apply poly to current CRC and shift CRC left
			if ((crc[0] & MSB) != 0) {
				/// left-shift bits in all bytes of CRC except for
                /// least-significant byte
				for (crcIndex = 0; crcIndex < (polyLength - 1); ++crcIndex) {
					/// shift in MSB from next least-significant byte to LSB of
                    /// current byte
					crc[crcIndex] = ((uint8_t)((crc[crcIndex] << 1) & 0xff)) |
                                    ((crc[crcIndex + 1] & MSB) >> (BITS_PER_BYTE - 1));

					/// XOR poly with current CRC value
					crc[crcIndex] ^= poly[crcIndex];
				}

				/// left-shift least-significant byte
				crc[crcIndex] = (uint8_t)((crc[crcIndex] << 1) & 0xff);
				crc[crcIndex] ^= poly[crcIndex]; /// poly XOR current CRC value
			}
			else {	/// top CRC bit = 0 -> shift CRC left only
				/// left-shift bits in all bytes of CRC except for
                /// least-significant byte
				for (crcIndex = 0; crcIndex < (polyLength - 1); ++crcIndex) {
					/// shift in MSB from next least-significant byte to LSB of
                    /// current byte
					crc[crcIndex] = ((uint8_t)((crc[crcIndex] << 1) & 0xff)) |
                                    ((crc[crcIndex + 1] & MSB) >> (BITS_PER_BYTE - 1));
				}
				/// left-shift least-significant byte
				crc[crcIndex] = (uint8_t)((crc[crcIndex] << 1) & 0xff);
			}
		}
	}
}

/** ****************************************************************************
 * @name	CrcCcittTypeToBytes
 * @brief split the input type into bytes
 * @param [in] type - input
 * @param [out] bytes - shifted and masked bytes
 * @retval N/A
 ******************************************************************************/
void CrcCcittTypeToBytes (CrcCcittType type,
                          uint8_t bytes [])
{
	bytes[0] = (uint8_t)((type >> ONE_BYTE) & BYTE_0);
	bytes[1] = (uint8_t)(type & 0xff);
}

/** ****************************************************************************
 * @name BytesToCrcCcittType
 * @brief split the into bytes
 * @param [out] bytes - shifted andmasked bytes
 * @retval N/A
 ******************************************************************************/
CrcCcittType BytesToCrcCcittType (const uint8_t bytes [])
{
	return ((((CrcCcittType)bytes[0] << ONE_BYTE) & BYTE_1) |
		     ((CrcCcittType)bytes[1] & BYTE_0));
}

/** ****************************************************************************
 * @name	Crc32TypeToBytes
 * @brief split the input type into bytes
 * @param [in] type - input
 * @param [out] bytes - shifted bytes
 * @retval N/A
 ******************************************************************************/
void Crc32TypeToBytes (Crc32Type type,
                       uint8_t   bytes [])
{
	bytes[0] = (uint8_t)((type >> THREE_BYTES) & BYTE_0);
	bytes[1] = (uint8_t)((type >> TWO_BYTES)   & BYTE_0);
	bytes[2] = (uint8_t)((type >> ONE_BYTE)    & BYTE_0);
	bytes[3] = (uint8_t)(type                  & BYTE_0);
}

/** ****************************************************************************
 * @name	BytesToCrc32Type
 * @brief convert individual bytes to 32 bit type
 * @param [out] bytes - pointer to the converted
 * @retval N/A
 ******************************************************************************/
Crc32Type BytesToCrc32Type (const uint8_t bytes [])
{
	return ((((Crc32Type)bytes[0] << THREE_BYTES) & BYTE_3) |
		    (((Crc32Type)bytes[1] << TWO_BYTES)   & BYTE_2) |
		    (((Crc32Type)bytes[2] << ONE_BYTE)    & BYTE_1) |
		     ((Crc32Type)bytes[3]                 & BYTE_0));
}

/** ****************************************************************************
 * @name	CrcCcitt
 * @brief perform the crc calculations
 * @param [out] data - pointer to the input data
 * @param [out] length - of the input data
 * @param [out] seed - starting value for the calc
 * @retval N/A
 ******************************************************************************/
CrcCcittType CrcCcitt (const uint8_t      data[],
                       uint16_t           length,
                       const CrcCcittType seed)
{
	uint8_t crc[CRC_CCITT_LENGTH];

	CrcCcittTypeToBytes(seed, crc);
	Crc(CRC_CCITT_POLY,
        CRC_CCITT_LENGTH,
        crc,
        data,
        length,
        crc);

	return (BytesToCrcCcittType(crc));
}

/** ****************************************************************************
 * @name	Crc32
 * @brief perform the crc calculations
 * @param [out] data - pointer to the input data
 * @param [out] length - of the input data
 * @param [out] seed - starting value for the calc
 * @retval N/A
 ******************************************************************************/
Crc32Type Crc32 (const uint8_t   data[],
                 uint16_t        length,
                 const Crc32Type seed)
{
	uint8_t crc[CRC_32_LENGTH];

	Crc32TypeToBytes(seed, crc);
	Crc(CRC_32_POLY,
        CRC_32_LENGTH,
        crc,
        data,
        length,
        crc);

	return (BytesToCrc32Type(crc));
}

