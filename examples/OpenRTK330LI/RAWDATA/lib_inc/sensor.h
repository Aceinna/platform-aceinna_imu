#ifndef _SENSOR_H
#define _SENSOR_H

#include "GlobalConstants.h"
#include "Indices.h"


#define CONVERT_XBOW_TO_380(x) ( (int32_t)( (int32_t)( (x) >> 16 ) +  MININT16 ) ) ///< going to signed int 32
#define CONVERT_380_TO_XBOW(x) (( (uint32_t)( (x) - MININT16 ) ) << 16 ) ///< going to unsigned int 32


typedef struct{
    int accelData[NUM_AXIS];
    int gyroData[NUM_AXIS];
    int tempData;
}sensorSamplingData_t;

typedef struct{
    sensorSamplingData_t sensorData[4];
    uint16_t fragment;
    uint16_t index;
}sensorsSamplingData_t;

extern sensorsSamplingData_t _SensorsData;

#pragma pack(1)
typedef struct {
    uint16_t preamble;
    uint16_t pktCode;
    uint8_t  pldLen;
    sensorsSamplingData_t pld;
    uint16_t crc;
}sensorsDataPacket_t;
#pragma pack()

extern uint16_t globalSensorsMask;

#endif /* SENSOR_H */

