/** ***************************************************************************
 * @file gyroBMI16.h Gyroscope header file for the BMI160 gyro 
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
#ifndef BMI160_H
#define BMI160_H

#include <stdint.h>

enum gyro_cutoff_freq_mode {
  OSR1_mode      =     2,
  OSR2_mode      =     1,
  OSR4_mode      =     0
};


enum gyro_power_mode {
  gyro_suspend_mode         =     0x14,
  gyro_normal_mode          =     0x15,
  gyro_fast_startup_mode    =     0x17
};

enum accel_power_mode {
  accel_suspend_mode        =     0x10,
  accel_normal_mode         =     0x11,
  accel_lowpower_mode       =     0x12
};

enum mag_power_mode {
  mag_suspend_mode          =     0x18,
  mag_normal_mode           =     0x19,
  mag_lowpower_mode         =     0x1a
};

enum bmi160_oeration_command {
  fast_calibration_cmd     =     0x3,
  program_nvm_cmd          =     0xa0,
  flush_fifo_cmd           =     0xb0,
  reset_interrupt_cmd      =     0xb1,
  soft_reset_cmd           =     0xb6,
  reset_step_counter_cmd   =     0xb2
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

      
#define BMI160_CHIP_ID_REG_ADDR           0x0
#define BMI160_CHIP_ID                    0xd1

#define BMI160_ERR_REG_ADDR               0x2
#define BMI160_NO_ERROR                   0x0
#define BMI160_FATAL_ERROR                0x1
#define BMI160_UNKNOW_ERROR               0x2
#define BMI160_UNKNOW_ERROR1              0x4
#define BMI160_PREFILTERED_DATA_ERROR     0x6
#define BMI160_ODR_ERROR                  0xc
#define BMI160_LOWPOWER_DATA_ERROR        0xe
#define BMI160_I2C_ERROR                  0x20
#define BMI160_DROPPED_CMD_ERROR          0x40
#define BMI160_MAG_DRDY_ERROR             0x80

#define BMI160_POWER_STATUS_REG_ADDR      0x3
#define BMI160_MAG_NORMAL_POPWER          0x1
#define BMI160_MAG_LOW_POWER              0x2
#define BMI160_MAG_MAG_POWER_MASK         0x3
#define BMI160_GYRO_NORMAL_POWER          0x4
#define BMI160_GYRO_FAST_STARTUP          0xc
#define BMI160_GYRO_POWER_MASK            0xc
#define BMI160_ACCEL_NORMAL_POWER         0x10
#define BMI160_ACCEL_LOW_POWER            0x20
#define BMI160_ACCEL_POWER_MASK           0x30

#define BMI160_MAG_DATA_ADDR              0x4
#define BMI160_GYRO_DATA_ADDR             0xc
#define BMI160_ACCEL_DATA_ADDR            0x12
#define BMI160_SENSOR_TIME_ADDR           0x18

#define BMI160_STATUS_REG_ADDR            0x1b
#define BMI160_GYRO_SELFTEST              0x2
#define BMI160_MAG_INTERFACE              0x4
#define BMI160_FOC_READY                  0x8
#define BMI160_NVM_READY                  0x10
#define BMI160_MAG_DRDY                   0x20
#define BMI160_GYRO_DRDY                  0x40
#define BMI160_ACCEL_DRDY                 0x80

#define BMI160_TEMP_REG_ADDR              0x20


#define BMI160_GYRO_CONF_REG_ADDR         0x42
#define BMI160_GYRO_RANGE_REG_ADDR        0x43

#define BMI160_COMMAND_REG_ADDR           0x7e

#define BMI160_BUFFER_SIZE (NUM_AXIS * sizeof(uint16_t) + 16 + 1)

extern uint8_t isGyroPowerOn(void);
extern uint8_t isAccelPowerOff(void);
extern uint8_t isErrBMI160(void);
extern uint8_t isBMI160(void);
extern void BMI160Config(uint32_t *, uint32_t *);
extern void powerOnBMI160Gyro(void);
extern void powerOffBMI160Gyro(void);
extern void powerOnBMI160Accel(void);
extern void poweroffBMI160Accel(void);



#endif  // BMI160_H
