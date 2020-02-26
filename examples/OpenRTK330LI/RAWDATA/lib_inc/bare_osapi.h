#ifndef _BARE_OS__H_
#define _BARE_OS__H_
#include <stdint.h>
#include "rtcm.h"
#include "Indices.h"

#define ENTER_CRITICAL()      //
#define EXIT_CRITICAL()       //
#define OSEnterISR()          //
#define OSExitISR()           //


#define NUM_SENSOR_CHIPS 3
#define NUM_AXIS 3


#define N_MISALIGN         18
#define N_RESERVED_CAL      6

#define N_TABLES_A    	   15   ///< 6 + 6 + 3
#define N_ROWS_TABLE_A    360 	///< 162 + 99 + 99
#define N_COLS_TABLE_A      2

typedef void * TaskHandle_t;
typedef uint32_t   TickType_t;
typedef struct {            //TODO:
    // Timer output counter
    uint32_t timerCntr, dTimerCntr;

    // Algorithm states
    double accel_g[3];
    double rate_radPerSec[3];
    double rate_degPerSec[3];
//  double mag_G[3];
    double temp_C;
} IMUDataStruct;

typedef struct {
    int32_t           rawSensors[NUM_SENSOR_CHIPS][N_RAW_SENS];
    int32_t           rawSensorsCombined[N_RAW_SENS];
    int32_t           rawTempSensors[NUM_SENSOR_CHIPS];
    int32_t           rawTempSensorsCombined;
    float             scaledSensors[NUM_SENSOR_CHIPS][N_RAW_SENS]; 	/// g's, rad/s, G, deg C, (body frame)
    float             scaledSensorsCombined[N_RAW_SENS]; 			/// g's, rad/s, G, deg C, (body frame)
    float             rateAlarm;
    float             accelAlarm;
    uint32_t          samplingTstamp;
}sensors_data_t;
sensors_data_t gSensorsData;


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
sensorsSamplingData_t _SensorsData;

typedef struct {
	int	   sat;        /*prn*/
    double rs[6];
	double dts[2];
    double var;
    int svh;    
	double azel[2];    /*azimuth,elevation*/
	double e[3];       /*partial deviation*/
	double tgd;        /* tgd*/
	double r;          /* vector */
	double rate;
	double tro;        /* tropospheric */
	int vflag;
	int slip[NFREQ];
	int numofepoch[NFREQ];
}vec_t;
typedef struct
{
	obs_t obs;
	vec_t vec[MAXOBS];
}epoch_t;
epoch_t gRov;


typedef struct {
	int32_t  counts;
	float    value;
}cal_table_entry_t;

struct productConfiguration_BITS  { /// bits   description
    uint16_t hasMags     :1;        /// 0 - lsb
    uint16_t hasGps      :1;        /// 1
    uint16_t algorithm   :1;        /// 2 - 380 only uses one algorithm
    uint16_t extAiding   :1;        /// 3
    uint16_t architecture:4;        /// 4-7 - 380 only has one architecture
    uint16_t rsvd        :7;        /// 8-14
    uint16_t isADAHRS    :1;        /// 15 - msb
};

union ProductConfiguration {
    uint16_t                         all;
    struct productConfiguration_BITS bit;
};

typedef struct {
    uint32_t serialNumber;
    char     versionString[128];

    /// index of first element for each sensor (including 7th sensor)
    uint16_t calibrationTableIndexA[N_TABLES_A + 1]; // 15 + 1
    /// table order: (Temp Tables: xAccel, yAccel, zAccel, xRate, yRate, zRate),
    /// Inertial Tables: (same order), mag tables (x, y, z)
    cal_table_entry_t calibrationTablesA[N_ROWS_TABLE_A]; // [360]
    float    misalign[N_MISALIGN]; // [18]

    uint16_t reserved[N_RESERVED_CAL]; // [6]
    union    ProductConfiguration productConfiguration;

    /// added from the 440 - NAV-view may not program them
    float    AccelSensorRange;
    float    GyroSensorRange;
    float    MagSensorRange;

    uint16_t RollIncidenceLimit;
    uint16_t PitchIncidenceLimit;

    uint16_t HardIronLimit;
    uint16_t SoftIronLimit;
} CalibrationStruct;

CalibrationStruct   gCalibration[NUM_SENSOR_CHIPS];


#define __weak   __attribute__((weak))
__weak void OSDisableHook();
__weak void OSEnableHook();
__weak void OS_Delay(uint32_t msec);
__weak inline void OSDisableHookIfNotInIsr();
__weak inline void OSEnableHookIfNotInIsr();
#endif