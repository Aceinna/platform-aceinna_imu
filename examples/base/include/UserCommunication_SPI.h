/** ***************************************************************************
 * @file   UserCommunication_SPI.c
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
 #ifndef _USER_COMMUNICATION_SPI_H_
#define _USER_COMMUNICATION_SPI_H_
#include <stdint.h>
#include <stm32f2xx.h>

#include "dmu.h" // For BOOL

// Define the size of and addresses in the data register - NOT USED
#define STARTING_ADDR_OF_DATA_REGISTER          0x00
#define LENGTH_OF_READ_ONLY_SECTION_OF_REGISTER 0X50
#define LENGTH_OF_DATA_REGISTER                 0x80

// Define SPI write error codes - NOT USED
#define NO_WRITE_ERROR                          0x00
#define WRITE_TO_READ_ONLY_REGISTER_ATTEMPTED   0x01
#define WRITE_BEYOND_END_OF_REGISTER            0x02

// Callback data dictionary the indecies match the register values so running
// a callback is just the index
typedef void(*tSpiCallback)(uint8_t);
typedef struct {
    tSpiCallback callback;
} tWriteTable;
extern const tWriteTable gSpiWrite[];

  // ---- initializes the user SPI interface ----
  uint8_t InitCommunication_UserSPI( void );

  void HandleRegisterRead_UserSPI( void ); // used?
  void HandleRegisterWrite_UserSPI( uint8_t registerAddress, uint8_t writeValue );

  // ---- Populate the SPI data register ----
  void FillSpiDataRegister_Sensors( void );
  void FillSpiDataRegister_Algorithms( void );
  void FillSpiDataRegister_MagAlign( void );
  void FillSpiDataRegister_BIT( void );
  void FillSpiDataRegister_OutputBytesExpected( uint8_t NumberOfBytesExpected );

  //  ---- Populate the SPI data burst register ----
  void FillSpiBurstRegister_Sensors( void );
  void UserSPI_BytesInBurstRegister();

/// External user SPI3 DMA Data transfer
// Burst-register contents, (26 bytes = 13 shorts)
//   Bytes  1- 2: Status
//   Bytes  3- 8: Rate Sensor  (x, y, and z)
//   Bytes  9-14: Acceleration (x, y, and z)
//   Bytes 15-20: Magnetometer (x, y, and z)
//   Bytes 21-22: Rate-sensor temperature
//   Bytes 23-24: Board temperature
//   Bytes 25-26: Calculate the CRC
#define BURST_REGISTER_SIZE 26

typedef struct {
    uint8_t registerAddr;       // DataRegister[registerAddr]
    uint8_t ErrorCode;

    uint8_t DataRegister[ 128 ]; // output (master MISO read) buffer for DMA
    uint8_t BurstRegister[ BURST_REGISTER_SIZE ]; // [26] bytes
    uint8_t writeBuffer[ 256 ]; // 380 input (master MOSI write) data buffer -- Increased to 255 to fix issue
                                //                                              with ODR value being overwritten
                                //                                              when too many elements written
                                //                                              to writeBuffer

    uint16_t outputDataRate;// packetRateDivider;
    uint16_t dataRateCounter;
    uint8_t InternalTimer; // use TIM5 [1] or 1PPS [0]

    // bits for SPI data ready output line SPI_REG_DATA_READY_READ [0x35]
    uint8_t EnableDataReady;   // 0x04
    uint8_t DataReadyPolarity; // 0x02
    uint8_t DataReadyPin;      // 0x01

    double floatLimit_Rate; // SPI saturation limits
    double floatLimit_Accel;
    double floatLimit_Mag;

    // avoid interrupts from colliding by preventing function calls before the
    //   SPI DMA is fully running.
    uint8_t dmaStartedFlag;

    // Variables used in the SPI DMA interrupt
    uint8_t zeroBuffer[ 128 ];

    // orientaion loading variables (so the read will reset the load process)
    uint8_t  byteCntr;
    uint16_t firstByte;
} UserSpiStruct;

volatile extern UserSpiStruct gUserSpi;

// -----------------------------------------------------------------------------------
BOOL CheckPacketRateDivider_UserSpi( uint16_t packetRateDivider );

// clock divider [5] to get 200hz from external 1khz or [10] to get 100 Hz
#define  TICKS_FOR_200_HZ  5
#define  TICKS_FOR_100_HZ  10

// Output Data rate from SPI MASTER
enum requested_data_rate {
    REQUEST_ZERO_HZ =  0,
    REQUEST_200_HZ  =  1,
    REQUEST_100_HZ  =  2,
    REQUEST_50_HZ   =  3,
    REQUEST_25_HZ   =  4,
    REQUEST_20_HZ   =  5,
    REQUEST_10_HZ   =  6,
    REQUEST_5_HZ    =  7,
    REQUEST_4_HZ    =  8,
    REQUEST_2_HZ    =  9,
    REQUEST_1_HZ    = 10
};

// Number of 200 hz frames to get the listed rate (frame rate divisor)
enum output_data_rate {
    OUTPUT_DATA_RATE_ZERO   =   0,
    OUTPUT_DATA_RATE_200_HZ =   1,
    OUTPUT_DATA_RATE_100_HZ =   2,
    OUTPUT_DATA_RATE_50_HZ  =   4,
    OUTPUT_DATA_RATE_25_HZ  =   8,
    OUTPUT_DATA_RATE_20_HZ  =  10,
    OUTPUT_DATA_RATE_10_HZ  =  20,
    OUTPUT_DATA_RATE_5_HZ   =  40,
    OUTPUT_DATA_RATE_4_HZ   =  50,
    OUTPUT_DATA_RATE_2_HZ   = 100,
    OUTPUT_DATA_RATE_1_HZ   = 200
};


#define INERTIAL_SENSOR_ENABLE          0x01
#define INERTIAL_SENSOR_ENABLE_X_AXIS   0x02
#define INERTIAL_SENSOR_ENABLE_Y_AXIS   0x04
#define INERTIAL_SENSOR_ENABLE_Z_AXIS   0x08

#define TEMP_SENSOR_ENABLE       0x01
#define BOARD_TEMP_SENSOR_ENABLE 0x02
#define RATE_TEMP_SENSOR_ENABLE  0x04

#define RATE_OUTPUT_62_DEGPERSEC   0x00
#define RATE_OUTPUT_125_DEGPERSEC  0x10
#define RATE_OUTPUT_250_DEGPERSEC  0x20
#define RATE_OUTPUT_500_DEGPERSEC  0x30

#define ACCEL_OUTPUT_1_G   0x00
#define ACCEL_OUTPUT_2_G   0x10
#define ACCEL_OUTPUT_4_G   0x20
#define ACCEL_OUTPUT_5_G   0x30
#define ACCEL_OUTPUT_8_G   0x40

#define MAG_OUTPUT_1_GAUSS   0x00
#define MAG_OUTPUT_2_GAUSS   0x10
#define MAG_OUTPUT_4_GAUSS   0x20
#define MAG_OUTPUT_8_GAUSS   0x30

// Register map addresses
#define SPI_REG_SPI_ERROR         0x02
#define SPI_REG_NUM_BYTES_BURST   0x03

#define SPI_REG_RATE_X_MSB 0x04
#define SPI_REG_RATE_X_LSB 0x05
#define SPI_REG_RATE_Y_MSB 0x06
#define SPI_REG_RATE_Y_LSB 0x07
#define SPI_REG_RATE_Z_MSB 0x08
#define SPI_REG_RATE_Z_LSB 0x09

#define SPI_REG_ACCEL_X_MSB 0x0A
#define SPI_REG_ACCEL_X_LSB 0x0B
#define SPI_REG_ACCEL_Y_MSB 0x0C
#define SPI_REG_ACCEL_Y_LSB 0x0D
#define SPI_REG_ACCEL_Z_MSB 0x0E
#define SPI_REG_ACCEL_Z_LSB 0x0F

#define SPI_REG_MAG_X_MSB 0x10
#define SPI_REG_MAG_X_LSB 0x11
#define SPI_REG_MAG_Y_MSB 0x12
#define SPI_REG_MAG_Y_LSB 0x13
#define SPI_REG_MAG_Z_MSB 0x14
#define SPI_REG_MAG_Z_LSB 0x15

#define SPI_REG_RATE_TEMP_MSB  0x16
#define SPI_REG_RATE_TEMP_LSB  0x17
#define SPI_REG_BOARD_TEMP_MSB 0x18
#define SPI_REG_BOARD_TEMP_LSB 0x19
// 0x1a - 0x35 reserved
#define SPI_REG_GPIO_LEVEL_WRITE     0x33
#define SPI_REG_GPIO_LEVEL_READ      0x32
#define SPI_REG_GPIO_DIRECTION_WRITE 0x32
#define SPI_REG_GPIO_DIRECTION_READ  0x33

#define SPI_REG_DATA_READY_WRITE 0x34
#define SPI_REG_DATA_READY_READ  0x35

#define SPI_REG_SELF_TEST_READ   0x34
#define SPI_REG_SELF_TEST_WRITE  0x35   // 0x35 = 53 dec

#define SPI_REG_DECIMATION_READ           0x36
#define SPI_REG_CLOCK_CONFIG_READ         0x37
#define SPI_REG_DYNAMIC_RANGE_CONFIG_READ 0x38
#define SPI_REG_FILTER_TAPS_CONFIG_READ   0x39

#define SPI_REG_CLOCK_CONFIG_WRITE          0x36
#define SPI_REG_DECIMATION_WRITE            0x37
#define SPI_REG_FILTER_TAPS_CONFIG_WRITE    0x38
#define SPI_REG_DYNAMIC_RANGE_CONFIG_WRITE  0x39

#define SPI_REG_SYSTEM_STATUS_MSB 0x3C
#define SPI_REG_SYSTEM_STATUS_LSB 0x3D

#define SPI_REG_JD_BURST_READ 0x3E // John Deere burst message
#define SPI_REG_F1_BURST_READ 0x3F // UCB F1 Raw1
#define SPI_REG_F2_BURST_READ 0x40 // UCB F2 Raw2
#define SPI_REG_S0_BURST_READ 0x41 // UCB S0 Scaled0
#define SPI_REG_S1_BURST_READ 0x42 // UCB S1 Scaled1
#define SPI_REG_A1_BURST_READ 0x43 // UCB A1 Angle1 kalman
#define SPI_REG_A4_BURST_READ 0x44 // UCB A4 Angle4 kalman MODIFIED
#define SPI_REG_N0_BURST_READ 0x45 // UCB N0 Nav0
// DEBUG
#define SPI_REG_DEBUG_BURST_READ 0x44 // UCB S0 Scaled 0 +
// DEBUG
// DKH added mag and spi 10.17.14
#define SPI_REG_MAG_ALIGN_WRITE           0x50 // UCB Write Cal - Mag align cmd
#define SPI_REG_MAG_ALIGN_READ            0x51 // UCB Read  Cal - Mag Align status

#define SPI_HARD_IRON_BIAS_0_READ         0x48 // Hard Iron Bias 0
#define SPI_HARD_IRON_BIAS_1_READ         0x4a // Hard Iron Bias 1
#define SPI_SOFT_IRON_SCALE_READ          0x4c // Soft Iron Scale
#define SPI_SOFT_IRON_ANGLE_READ          0x4e // Soft Iron angle

#define SPI_BIT_MASTER_STATUS_READ        0x5a // Built In Test registers
#define SPI_BIT_HW_STATUS_READ            0x5c
#define SPI_BIT_SW_MASTER_READ            0x5e // high level flags
#define SPI_BIT_SW_STATUS_READ            0x60 // algorithm state
#define SPI_BIT_SW_ALG_READ               0x62 // algorithm status
#define SPI_BIT_SW_DATA_READ              0x64 // CRC, Mag align out of bounds
#define SPI_BIT_COMM_MASTER_READ          0x66 // A and B serial
#define SPI_BIT_COMM_DATA_STATUS_READ     0x68
#define SPI_BIT_COMM_BUS_A_READ           0x6a // detail bus error flags
#define SPI_BIT_COMM_BUS_B_READ           0x6c // detail bus error flags
#define SPI_BIT_SENSOR_STATUS_READ        0x6e // over range etc.

#define SPI_REG_RATE_OUTPUT_CONFIG_READ   0x70
#define SPI_REG_ACCEL_OUTPUT_CONFIG_READ  0x71
#define SPI_REG_MAG_OUTPUT_CONFIG_READ    0x72
#define SPI_REG_TEMP_OUTPUT_CONFIG_READ   0x73

#define SPI_REG_RATE_OUTPUT_CONFIG_WRITE  0x71
#define SPI_REG_ACCEL_OUTPUT_CONFIG_WRITE 0x70
#define SPI_REG_MAG_OUTPUT_CONFIG_WRITE   0x73
#define SPI_REG_TEMP_OUTPUT_CONFIG_WRITE  0x72

// Added for -12 code
#define SPI_REG_HW_VERSION_READ   0x7E
#define SPI_REG_SW_VERSION_READ   0x7F

// REGISTER MASKS
#define SPI_REG_RATE_CONFIG_MASK 0x3F         // b00111111
#define SPI_REG_MAG_CONFIG_MASK  0x3F

#define SPI_REG_CLOCK_CONFIG_MASK       0x01  // b00000001
#define SPI_REG_FILTER_TAPS_CONFIG_MASK 0x07  // b00000111

#define SPI_REG_SELF_TEST_MASK  0x0C   // b01000111
#define SPI_REG_DATA_READY_MASK 0x47   // b01000111

#define BYTES_IN_DATA_REGISTER  128

// SPI Error codes
#define WRITE_TO_INVALID_REGISTER 0x01
#define INVALID_WRITE_COMMAND 0x02
#define INVALID_WRITE_VALUE   0x04

#endif