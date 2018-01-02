/*
 * File:   Indices.h
 * Author: joemotyka
 *
 * Created on April 10, 2016, 10:12 PM
 */

#ifndef INDICES_H
#define INDICES_H

#include <stdint.h>

#define X_AXIS   0
#define Y_AXIS   1
#define Z_AXIS   2
#define NUM_AXIS 3

#define  ROLL   0
#define  PITCH  1
#define  YAW    2

/* raw sensor order  (from xbowsp_algorithm.h) */
enum rawSensor_e {
    XACCEL = 0, 
    YACCEL = 1, 
    ZACCEL = 2,
    XRATE = 3, 
    YRATE = 4, 
    ZRATE = 5,
    XMAG = 6, 
    YMAG = 7, 
    ZMAG = 8,
    XATEMP = 9, 
    YATEMP = 10, 
    ZATEMP = 11,
    XRTEMP = 12, 
    YRTEMP = 13, 
    ZRTEMP = 14,
    BTEMP = 15,
    N_RAW_SENS = 16 // size
};

/// raw sensor order (from dmu.h)
enum eSensorOrder {
    ACCEL_START = 0,
    RATE_START = 3,
    MAG_START = 6,
    NUM_SENSOR_IN_AXIS = 9,
    TEMP_START = 9,
    GYRO_TEMP = 9,
    TEMP_SENSOR = 10,
    NUM_SENSOR_READINGS = 11
};

#define NUM_SENSORS 3  // from dmu.h

#define Q0      0
#define Q1      1
#define Q2      2
#define Q3      3

#define LAT_IDX     0
#define LON_IDX     1
#define ALT_IDX     2

enum NED_Index {
  GPS_NORTH = 0,
  GPS_EAST  = 1,
  GPS_DOWN  = 2
};


#endif /* INDICES_H */
