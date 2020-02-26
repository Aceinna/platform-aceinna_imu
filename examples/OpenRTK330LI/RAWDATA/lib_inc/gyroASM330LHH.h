/** ***************************************************************************
 * @file gyroBMI16.h Gyroscope header file for the ASM330LHH gyro 
 * @author
 * @date   March, 2017
 * @copyright (c) 2017 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * Note, the gyro should implement the interface described in
 * gyroscope.h. This file just provides the specifics, for use by the
 * associated C file only.
 *****************************************************************************/
#ifndef ASM330LHH_H
#define ASM330LHH_H

#include <stdint.h>

enum gyro_cutoff_freq_mode {
  OSR1_mode      =     2,
  OSR2_mode      =     1,
  OSR4_mode      =     0
};


enum gyro_power_mode {
  ASM330LHH_gyro_suspend_mode   = 0x00,
  ASM330LHH_gyro_normal_mode    = 0x44,   // 1667 Hz, 500DPS
};

enum accel_power_mode {
  ASM330LHH_accel_suspend_mode  = 0x00,
  ASM330LHH_accel_normal_mode   = 0x4E,   // 833 Hz, 8G, LPF2 On
};


enum ASM330LHH_oeration_command {
  ASM330LHHsoft_reset_cmd  =  0x01
};

typedef union {
  struct {
    uint8_t     gyr_odr            :   4;
    uint8_t     gyr_bwp            :   2;
    uint8_t     reserved           :   2;
  } b;
  
  uint8_t    r;
} GYRO_CONF_REG;

typedef union {
  struct {
    uint8_t   gyr_range           :    3;
    uint8_t   reserved            :    5;
  } b;
  
  uint8_t r;
} GYRO_RANGE_REG;


typedef union {
  struct {
    uint8_t     odr                :   4;
    uint8_t     bwp                :   3;
    uint8_t     us                 :   1;
  } b;
  
  uint8_t    r;
} ACCEL_CONF_REG;

typedef union {
  struct {
    uint8_t   range               :    4;
    uint8_t   reserved            :    4;
  } b;
  
  uint8_t r;
} ACCEL_RANGE_REG;


#define ASM330LHH_PIN_CTRL            0x02  // 0x3F
#define ASM330LHH_SDO_PU_ENABLE       0x40  // 

#define ASM330LHH_FIFO_CTRL1          0x07  // 0x00 watermark threshold
#define ASM330LHH_FIFO_CTRL2          0x08  // 0x00
#define ASM330LHH_FIFO_CTRL3          0x09  // 0x00 Gyroscope and accel fifo write rate 
#define ASM330LHH_FIFO_CTRL4          0x0A  // 0x00 fifo modemode

#define ASM330LHH_COUNTER_BDR_REG1    0x0B  // 0x00 Data ready  
#define ASM330LHH_COUNTER_BDR_REG2    0x0C  // 0x00 batching threshold
#define ASM330LHH_INT1_CTRL           0x0D  // 0x00 int1 pin control 
#define ASM330LHH_INT2_CTRL           0x0E  // 0x00 int2 pin control
#define ASM330LHH_WHO_AM_I            0x0F  // 0x00 whoami - 0x6B 

#define ASM330LHH_CHIP_ID             0x6B  // 0x6B


#define ASM330LHH_CTRL1_XL            0x10  // 0x00 accel control - ODR, Full scale, enable LPF2
#define ASM330LHH_CTRL2_G             0x11  // 0x00 gyro control  - ODR, Full scale, 
#define ASM330LHH_CTRL3_C             0x12  // 0x04 interface control
#define ASM330LHH_CTRL4_C             0x13  // 0x00 interface control
#define ASM330LHH_CTRL5_C             0x14  // 0x00 self test, rounding 
#define ASM330LHH_CTRL6_G             0x15  // 0x00 DEN trigger mode , Gyro LPF Filter
#define ASM330LHH_CTRL7_G             0x16  // 0x00 Gyro HPF
#define ASM330LHH_CTRL8_XL            0x17  // 0x00 Accel filters settings
#define ASM330LHH_CTRL9_XL            0x18  // 0x00 ACCEL DEN
#define ASM330LHH_CTRL10_C            0x19  // 0x00 Timestamp enable
#define ASM330LHH_ALL_INT_SRC         0x1A  // Motion Detection interrupts
#define ASM330LHH_WAKE_UP_SRC         0x1B  // wakeup events control
#define ASM330LHH_D6D_SRC             0x1D  // orientation change detection
#define ASM330LHH_STATUS_REG          0x1E  // Status - new data available bits
#define ASM330LHH_OUT_TEMP_L          0x20  // Temperature out LOW
#define ASM330LHH_OUT_TEMP_H          0x21  // Temperature out HI
#define ASM330LHH_OUTX_L_G            0x22  // Gyro Data
#define ASM330LHH_OUTX_H_G            0x23
#define ASM330LHH_OUTY_L_G            0x24
#define ASM330LHH_OUTY_H_G            0x25
#define ASM330LHH_OUTZ_L_G            0x26
#define ASM330LHH_OUTZ_H_G            0x27
#define ASM330LHH_OUTX_L_A            0x28  // Accel Data
#define ASM330LHH_OUTX_H_A            0x29
#define ASM330LHH_OUTY_L_A            0x2A 
#define ASM330LHH_OUTY_H_A            0x2B 
#define ASM330LHH_OUTZ_L_A            0x2C 
#define ASM330LHH_OUTZ_H_A            0x2D 
#define ASM330LHH_FIFO_STATUS1        0x3A 
#define ASM330LHH_FIFO_STATUS2        0x3B 
#define ASM330LHH_TIMESTAMP0_REG      0x40  // Timestamp, 32 Bit 0.25 uS 
#define ASM330LHH_TIMESTAMP1_REG      0x41
#define ASM330LHH_TIMESTAMP2_REG      0x42
#define ASM330LHH_TIMESTAMP3_REG      0x43
#define ASM330LHH_INT_CFG0            0x56  // 0x00 Interrupt clear config
#define ASM330LHH_INT_CFG1            0x58  // 0x00 low power mode
#define ASM330LHH_THS_6D              0x59  // 0x00 portrait-landscape
#define ASM330LHH_INT_DUR2            0x5A  // 0x00
#define ASM330LHH_WAKE_UP_THS         0x5B  // 0x00 wakeup threshold  
#define ASM330LHH_WAKE_UP_DUR         0x5C  // 0x00 wakeup duration
#define ASM330LHH_FREE_FALL           0x5D  // 0x00 free fall threshold
#define ASM330LHH_MD1_CFG             0x5E  // 0x00 routing activity detection to Int1
#define ASM330LHH_MD2_CFG             0x5F  // 0x00 routing activity detection to Int2
#define ASM330LHH_INTERNAL_FREQ_FINE  0x63  // 0x00 Actual ODR estimation
#define ASM330LHH_X_OFS_USR           0x73  // 0x00 accelerometer offset correction
#define ASM330LHH_Y_OFS_USR           0x74  // 0x00 
#define ASM330LHH_Z_OFS_USR           0x75  // 0x00 
#define ASM330LHH_FIFO_DATA_OUT_TAG   0x78  // fifo data tagging control
#define ASM330LHH_FIFO_DATA_OUT_X_L   0x79  // fifo data out XYZ
#define ASM330LHH_FIFO_DATA_OUT_X_H   0x7A 
#define ASM330LHH_FIFO_DATA_OUT_Y_L   0x7B 
#define ASM330LHH_FIFO_DATA_OUT_Y_H   0x7C 
#define ASM330LHH_FIFO_DATA_OUT_Z_L   0x7D 
#define ASM330LHH_FIFO_DATA_OUT_Z_H   0x7E 

// transaction from ASM330LHH_STATUS_REG (0x1E) or ASM330LHH_OUT_TEMP_L (0x20) to ASM330LHH_OUTZ_H_A (0x2d) = 14 bytes
// need to enable address increment

#define ASM330LHH_BUFFER_SIZE (NUM_AXIS * sizeof(uint16_t) + 16 + 1)

#define ASM330LHH_FIFO_TAG_RATE_1  0x09 
#define ASM330LHH_FIFO_TAG_RATE_2  0x0a 
#define ASM330LHH_FIFO_TAG_RATE_3  0x0c 
#define ASM330LHH_FIFO_TAG_RATE_4  0x0f 

#define ASM330LHH_FIFO_TAG_ACCEL_1 0x11
#define ASM330LHH_FIFO_TAG_ACCEL_2 0x12
#define ASM330LHH_FIFO_TAG_ACCEL_3 0x14
#define ASM330LHH_FIFO_TAG_ACCEL_4 0x17

#define ASM330LHH_FIFO_TAG_TEMP_1  0x18
#define ASM330LHH_FIFO_TAG_TEMP_2  0x1B
#define ASM330LHH_FIFO_TAG_TEMP_3  0x1d
#define ASM330LHH_FIFO_TAG_TEMP_4  0x1e

// SPI interface - idle CLK high, latching on HI edge, 10MHz max 
// To disable I2C - write 1 into CTRL4_C
// Register address masked with 0x80 to read from device 
// Address change while multiple reads - CTRL3_C (12h) (IF_INC) bit set to 1
// The accelerometer is activated from power-down by writing ODR_XL[3:0] in CTRL1_XL (10h) 
// while the gyroscope is activated from power-down by writing ODR_G[3:0] in CTRL2_G (11h). 
// For combo-mode the ODRs are totally independent


//Gyroscope ODR [Hz] LPF2 cut-off [Hz]
//      12.5            4.3
//      26              8.3
//      52              16.7
//      104             33
//      208             67
//      417             133
//      833             267
//      1667            539
//      3333            1137
//      6667            3333



extern uint8_t isGyroPowerOn(int chip);
extern uint8_t isAccelPowerOn(int chip);
extern uint8_t isErrASM330LHH(void);
extern uint8_t isASM330LHH(void);
extern int     ASM330LHHConfig();
extern uint8_t powerOnASM330LHHGyro(void);
extern uint8_t powerOffASM330LHHGyro(void);
uint8_t        powerOnASM330LHHAccel(void);
uint8_t        powerOffASM330LHHAccel(void);
extern void    ASM330LHHSync(void);
extern int     ASM330LHHSoftReset(void);
extern uint8_t ASM330LHHSetAddrIncMode(void);
extern uint8_t ASM330LHHDisableFifo(void);
extern uint8_t ASM330LHHEnableFifo(void);
extern uint8_t ASM330LHHSetInertialSensorsBDR(void);




#endif  // ASM330LHH_H
