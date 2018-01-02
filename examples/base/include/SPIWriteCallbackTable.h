/** ***************************************************************************
 * @file   SPIWriteCallbackTable.h
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
 * DKH added mag and spi 10.17.14
 ******************************************************************************/
#ifndef _SPI_WRITE_CALLBACK_TABLE_H_
#define _SPI_WRITE_CALLBACK_TABLE_H_

// These aren't advertised to the user at this point
void Accel_Output_Config_Write(uint8_t data); // 0x70
void Rate_Output_Config_Write(uint8_t data);  // 0x71
void Temp_Output_Config_Write(uint8_t data);  // 0x72
void Mag_Output_Config_Write(uint8_t data);   // 0x73

#endif