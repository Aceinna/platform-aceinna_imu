#ifndef SENSOR_H
#define SENSOR_H

#include "dmu.h"

#define ACCEL_FILTER_TAPS 9
#define GYRO_FILTER_TAPS 9

#define NUM_AXIS 3

#define MAX_COVAR 6


#define RMAG_SERIAL_NUMBER_SIZE 4
#define RMAG_VERSION_STRING_SIZE 60
#define RMAG_MODEL_STRING_SIZE 20

typedef struct {
    double a[MAX_COVAR];     /* covariance information */
    int size;       /* amount of covar info depends on system */
    double matNorm;          /* different measure of goodness */
} covarStruct;


typedef struct {
    double magVector[3];	/* X Y Z Remote Mag sensor (Gauss) Filtered */
	double accelVector[3];	/* X Y Z Remote Mag sensor Accel Vector (G) */
    double accelinputscale[NUM_AXIS];
    double maginputscale[NUM_AXIS];
    BOOL newData;    /* new remote mag data ready, set in driver, unset in use */
    BOOL status;	 /* Remote Magnetometer comm status */
    char versionString[RMAG_VERSION_STRING_SIZE];
    char modelString[RMAG_MODEL_STRING_SIZE];
    unsigned long serialNumber;
    unsigned long serialNumberStoredWithCal;
    BOOL serialNumberMatch;
    unsigned int BITWord;
    double  magTemp;
} remoteMagStruct;

typedef struct {
    double trueAirSpeed;			/* True Air Speed KNOTS */
	BOOL airSpeedValid;				/* airspeed portion of airdata is valid */
    BOOL newData;					/* new data ready, set in driver, unset in use */
    BOOL status;					/* Airdata comm status */
} airdataStruct;

typedef struct {
    double turnRate;				/* Leveled earth frame yaw rate rad/sec*/
    double slipSkidAngle;			/* Slip or Skid Angle (rad) */
} flightParamStruct;

typedef struct {
    uint32_t state; 					/* what is filled: LSB to MSB in list */
    uint32_t algoState;
    double accel[3];				/* Accelerometer sensor (G) filtered, temp comp*/
    double gyro[3];					/* Gyro sensor (rad/sec) filtered, temp comp*/
    double accelNonFiltered[3];		/* Accelerometer sensor (G) temp comp*/
    double gyroNonFiltered[3];		/* Gyro sensor (rad/sec) temp comp*/
	BOOL gyroSaturated;
	airdataStruct* air;
	flightParamStruct* flight;
	remoteMagStruct* remotemag;
    double roll, pitch, yaw;
    double filteredroll, filteredpitch, filteredyaw;
//    matrixStruct LevelMat;  	/* 3x3 Cosine Rotation Body to Level (Not Heading) */
	double bodyRate[3];			/* Kalman Filter Corrected Body Rate rad/sec */
	double bodyRateUnfilt[3];	/* Kalman Filter Corrected Unfiltered Body Rate rad/sec */
	double bodyAccel[3];		/* Kalman Filter Corrected Body Accelerations m/sec^2 */
	double tangentRate[3];
	double tangentAccel[3];
	double accTanCalc[3];
	double accMeasGrav[3];
	double centripetalAccFilt;
    double dt;
	unsigned int countToStableSensor;
    covarStruct covar;
    BOOL solutionReset;
    double yawRateAve;
    uint32_t HeadingReset;
    double extHeadingFromGH;

} sensorStruct;

/* for sensor state variable */
#define M_ACCEL 				0x004
#define M_GYRO 					0x008
#define M_MAG 				  0x80000
#define M_AIR 				 0x100000
#define M_REMOTEMAG 		 0x200000

/* for sensor algoState variable */
#define M_INIT              	0x1
#define M_TURN              	0x2
#define M_FORCED_VG   		    0x4

/* for HeadingReset*/
#define TRUE_AND_USE_REMOTE_MAG      0x1
#define TRUE_AND_USE_EXT_HEADING     0x2

extern volatile unsigned char ioupDataStart;
extern unsigned char gIOUPSync;

#define NOSYNC	0
#define SYNCED	1
#define ZEROMSEC_CNT 0
#define ONEMSEC_CNT	 1
#define NINEMSEC_CNT 9
#define TENMSEC_CNT	10
#define TWENTYONEMSEC_CNT  21
#define SHIFT2BYTES	16
#define MAX_DELTA_PKT_CNT (100)
#define INITIOUPCOMM_TIMEOUT 1000

/* IOUP version string xmitted from IOUP
   15 char + null max: "xxxx-xxxx-xx_xx"   */
#define IOUP_VERSION_SIZE 15                /* # chars in version string */
extern char gIoupVersion[IOUP_VERSION_SIZE+1];
/*for ADAHRS, software version is added   */
#define IOUP_PN_VERSION_SIZE 20                /* # chars in PN +version string */
extern char gIoupPNversion[IOUP_PN_VERSION_SIZE];


#define AIRDATADROPOUTMAX 500		/* 5 seconds */
#define REMOTEMAGDROPOUTMAX 200		/* 2 seconds */
#define VER_HDR 0xAA
#define SENSOR_PACKET_SIZE 16
#define	PKTCHKSUM 15             	/* location of packet checksum */
#define IOUP_TIMEOUT_RESET  10      /* timer loops to wait for IOUP to reset */
#define IOUP_MAX_COMM_ERR   500

#define SIGNED_TO_UNSIGNED(x) (((x) + 0x8000) & 0xFFFF)

/* IOUP flips ADC counts before sending them out, need to flip them back */
#define REVERSE_ADC_COUNTS(counts)	(65535 - counts)

extern unsigned int packet[SENSOR_PACKET_SIZE];
extern unsigned int gIOUPpktcnt;

#define SENSOR_SCALING_FOR_RAW_ADC  7       /* set the same as that for VG440 */
#define TEMP_SCALING_FOR_RAW_ADC    14      /* set the same as that for VG440 */



#endif /* SENSOR_H */

