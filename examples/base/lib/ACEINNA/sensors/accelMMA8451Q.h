/** ***************************************************************************
 * @file accelMMA8451Q.h Accelerometer interface for the Freescale MMA8451Q
 *       accelerometer MMA8451, triple axis, I2C interface, accelerometer.
 *       14 bits of resolution
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 * Datasheet:
 * http://cache.freescale.com/files/sensors/doc/data_sheet/MMA8451Q.pdf
 *
 * Note, the accelerometer should implement the interface described in
 * accelerometer.h. This file just provides the specifics, for use by the
 * associated C file only.
 *****************************************************************************/
#ifndef MMA8451_I2C_H
#define MMA8451_I2C_H

/**
 * Define registers
 */
#define MMA8451_STATUS       0x00
#define MMA8451_OUT_X_MSB    0x01
#define MMA8451_OUT_X_LSB    0x02
#define MMA8451_OUT_Y_MSB    0x03
#define MMA8451_OUT_Y_LSB    0x04
#define MMA8451_OUT_Z_MSB    0x05
#define MMA8451_OUT_Z_LSB    0x06

#define MMA8451_F_SETUP             0x09
#define MMA8451_TRIG_CFG            0x0A
#define MMA8451_SYSMOD              0x0B
#define MMA8451_INT_SOURCE          0x0C
#define MMA8451_WHO_AM_I            0x0D
#define MMA8451_XYZ_DATA_CFG        0x0E
#define MMA8451_HP_FILTER_CUTOFF    0x0F
#define MMA8451_PL_STATUS           0x10
#define MMA8451_PL_CFG              0x11
#define MMA8451_PL_COUNT            0x12
#define MMA8451_PL_BF_ZCOMP         0x13
#define MMA8451_P_L_THS_REG         0x14
#define MMA8451_FF_MT_CFG           0x15
#define MMA8451_FF_MT_SRC           0x16
#define MMA8451_FF_MT_THS           0x17
#define MMA8451_FF_MT_COUNT         0x18

#define MMA8451_TRANSIENT_CFG       0x1D
#define MMA8451_TRANSIENT_SRC       0x1E
#define MMA8451_TRANSIENT_THS       0x1F
#define MMA8451_TRANSIENT_COUNT     0x20
#define MMA8451_PULSE_CFG           0x21
#define MMA8451_PULSE_SRC           0x22
#define MMA8451_PULSE_THSX          0x23
#define MMA8451_PULSE_THSY          0x24
#define MMA8451_PULSE_THSZ          0x25
#define MMA8451_PULSE_TMLT          0x26
#define MMA8451_PULSE_LTCY          0x27

#define MMA8451_PULSE_WIND          0x28
#define MMA8451_ASLP_COUNT          0x29
#define MMA8451_CTRL_REG1           0x2A
#define MMA8451_CTRL_REG2           0x2B
#define MMA8451_CTRL_REG3           0x2C
#define MMA8451_CTRL_REG4           0x2D
#define MMA8451_CTRL_REG5           0x2E
#define MMA8451_OFF_X               0x2F
#define MMA8451_OFF_Y               0x30
#define MMA8451_OFF_Z               0x31

#define MMA8451_X           0x00
#define MMA8451_Y           0x01
#define MMA8451_Z           0x02

#define MMA8451_DATARATE_800HZ  0
#define MMA8451_DATARATE_400HZ  1
#define MMA8451_DATARATE_200HZ  2
#define MMA8451_DATARATE_100HZ  3
#define MMA8451_DATARATE_12_5HZ 4
#define MMA8451_DATARATE_6_25HZ 5
#define MMA8451_DATARATE_1_56HZ 6
#define MMA8451_DATARATE_MASK   0x38
#define MMA8451_DATARATE_SHIFT  3

#define MMA8451_CTRL_REG1_ACTIVE    0x1
#define MMA8451_CTRL_REG1_LOW_NOISE 0x4

#define MMA8451_EXPECTED_WHO_AM_I 0x1A

#define MMA8451_CFG_2G 0
#define MMA8451_CFG_4G 1
#define MMA8451_CFG_8G 2
#define MMA8451_CFG_xG_SHIFT 0
#define MMA8451_CFG_xG_MASK  0x03

#define MMA8451_CFG4_INT_EN_DRDY 0x01 // no other interrupts enabled

#define MMA8451_CFG5_INT_PIN_SEL 0x00 // select INT2 (pin 9) for the DRDY interrupt

// Previous X, Y, or Z data was overwritten by new X, Y, or Z data before it was read
#define MMA8451_STATUS_ERROR   0x80
#define MMA8451_STATUS_READY   0x08 // A new set of data is ready

#define MMA8451_I2C_BASE       0x1C // SA0-grounded base address is 0x1C
#define MMA8451_I2C_ADDR       (MMA8451_I2C_BASE << 1) // 0x38

#endif /* MMA8451_I2C_H */

