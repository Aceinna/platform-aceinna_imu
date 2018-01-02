/** ***************************************************************************
 * @file Gyroscope.h gyroscope interface this is a generalized Gyro interface
 * possibly implemented using the Honeywell HMC5883L Gyro
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

#ifndef GYROSCOPE_H
#define GYROSCOPE_H
#include <stdint.h>

// (FIXME) JSM: Added for Feng's LPF implementation
#define GYRO_MAX_CACHED_NUM               16

enum gyro_output_data_rate {
  _25_Samples_Per_Second   =       6,
  _50_Samples_Per_Second   =       7,
  _100_Samples_Per_Second  =       8,
  _200_Samples_Per_Second  =       9,
  _400_Samples_Per_Second  =      10,
  _800_Samples_Per_Second  =      11,
  _1600_Samples_Per_Second =      12,
  _3200_Samples_Per_Second =      13,
  _20_Hz                   =      20,        
  _25_Hz                   =      25,
  _50_Hz                   =      50,
  _100_Hz                  =     100,
  _200_Hz                  =     200,
  _400_Hz                  =     400,
  _800_Hz                  =     800,
  _1600_Hz                 =    1600,
  _3200_Hz                 =    3200
};

enum measure_range {
  _125_Degrees_Per_Second  =  125,
  _250_Degrees_Per_Second  =  250,
  _500_Degrees_Per_Second  =  500,
  _1000_Degrees_Per_Second = 1000,
  _2000_Degrees_Per_Second = 2000,
};

typedef enum {
  GYRO_BUFFER_IDLE = 0,
  GYRO_BUFFER_AVAILABlE = 1,
  GYRO_BUFFER_FULL = 2,
  GYRO_BUFFER_OVERFLOW = 3
} GYRO_BUFFER_STATUS;

static struct {      
      uint8_t cached_num;
      uint8_t temp_buff_num;
      int16_t gyroData[GYRO_MAX_CACHED_NUM][NUM_AXIS];
      uint32_t errorNum;
      GYRO_BUFFER_STATUS status;
} gRate;

uint8_t InitGyro(); // This will use boardDefinition.h to set up the IO lines
                    //   returns TRUE if init was successful

#define GYRO_RANGE_1000DPS   1000
#define GYRO_RANGE_500DPS     500
#define GYRO_RANGE_250DPS     250
#define DEFAULT_GYRO_RANGE GYRO_RANGE_250DPS

extern uint8_t _ReadGyroRegister(uint8_t);
extern uint8_t _WriteGyroRegister(uint8_t, uint8_t);
extern uint8_t GyroConfig(uint32_t *, uint32_t *);

extern void BeginRateSensorRead(void);
//uint8_t GyroStartReading(); // true if success
extern uint8_t IsGyroDoneReading(); // true if read is complete
extern uint8_t GyroGetLastReading(int16_t *readings); // true if success

extern void GyroSelfTest_ApplyBias( void );

extern void GyroSelfTest_Bias( uint8_t apply );

extern uint8_t GyroTempGetLastReading(int16_t* reading);
extern uint8_t GyroTempGetTemperature(int16_t reading, float *out);

#endif /* GYROSCOPE_H */