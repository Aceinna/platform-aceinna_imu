/** ***************************************************************************
 * @file calibrate_mag.c functions for magnetometer alignment
 * @Author
 * @Rev:
 * @date   2011-02-02 15:23:33 -0800 (Wed, 02 Feb 2011)
 * @copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @brief soft iron - non magnetic iron affecting the magnetic field
 * hard iron - magnetic objects causing a field
 *****************************************************************************/
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "scaling.h"
#include "sensor.h"
#include "dmu.h"
#include "qmath.h"
#include "EKF_Algorithm.h"
#include "timer.h"
#include "xbowsp_algorithm.h"
#include "xbowsp_version.h"
#include "xbowsp_generaldrivers.h"
#include "xbowsp_init.h"
#include "extern_port.h"
#include "ucb_packet.h"
#include "debug.h"
#include "sensor_cal.h"
#include "taskUserCommunication.h" // getUserCommunicationType
#include "xbowsp_fields.h"
#include "calibrate.h" // configuration data struct
#include "UserCommunication_SPI.h" // gUserSpi struct
#include "timer.h"
#include "s_eeprom.h" // EEPROM write
#include "Indices.h"

#include "MagAlign.h"

#include "TimingVars.h"   // for timer

// Specify the maximum number of points in the magnetic-field storage array
#define  TOTAL_POINTS  400 // 380
#define NUMPOINT_LS 360

int32_t  magReadings_q27[2][TOTAL_POINTS] = {0};

static float Amat[5][5] = {0};
static float bvec[5][1] = {0};
static float Cmat[5][6] = {0};
static uint16_t magNum = 0;

UcbPacketStruct magUcbPacket;    /// all other data
UcbPacketStruct magAlignUcbPacket;    /// Structure used to enable the mag-align
                                      ///    data to be writtent to the EEPROM at
                                      ///    the completion of a mag-align
                                      ///    maneuver during SPI operation 

// Declare local functions
static uint8_t _PerformMagAlign( int32_t state, int32_t *hardIron_q27, int32_t *softIron_q27 );

static void _ProcessMagAlignData( int32_t  *minMag, int32_t  *maxMag,
                                  uint16_t numofpts,
                                  int32_t  *hardIron, int32_t  *softIron );

static void _CalcMagAlignScaleFactor(void);

void lowPass2Pole(int32_t *out, int32_t *lastInput, int32_t input, int resolution, int shiftCoef, int shiftCoefLast);

uint8_t accumAMat(real x, real y)
{
//  A(1,1:5) = [ sum( mMeas(:,X_AXIS).^2 .* mMeas(:,Y_AXIS).^2 ), ...   % xi^2 * yi^2
//             sum( mMeas(:,X_AXIS) .* mMeas(:,Y_AXIS).^3 ), ...      % xi * yi^3
//             sum( mMeas(:,X_AXIS).^2 .* mMeas(:,Y_AXIS) ), ...      % xi^2 * yi
//             sum( mMeas(:,X_AXIS) .* mMeas(:,Y_AXIS).^2 ), ...      % xi * yi^2
//             sum( mMeas(:,X_AXIS) .* mMeas(:,Y_AXIS) ) ];           % xi * yi
  Amat[0][0] += x*x * y*y ;    // xi^2 * yi^2
  Amat[0][1] += x * y*y*y ;       // xi * yi^3
  Amat[0][2] += x*x * y ;        // xi^2 * yi
  Amat[0][3] += x * y*y ;       // xi * yi^2
  Amat[0][4] += x * y ;           // xi * yi

//
//A(2,1:5) = [ sum( mMeas(:,X_AXIS) .* mMeas(:,Y_AXIS).^3 ), ...      % xi * yi^3
//             sum( mMeas(:,Y_AXIS).^4 ), ...                         % yi^4
//             sum( mMeas(:,X_AXIS) .* mMeas(:,Y_AXIS).^2 ), ...
//             sum( mMeas(:,Y_AXIS).^3 ), ...
//             sum( mMeas(:,Y_AXIS).^2 ) ];
  Amat[1][0] += x * y*y*y ;       // xi * yi^3
  Amat[1][1] += y*y*y*y ;            // yi^4
  Amat[1][2] += x * y*y ;
  Amat[1][3] += y*y*y ;
  Amat[1][4] += y*y ;
//
//A(3,1:5) = [ sum( mMeas(:,X_AXIS).^2 .* mMeas(:,Y_AXIS) ), ...
//             sum( mMeas(:,X_AXIS) .* mMeas(:,Y_AXIS).^2 ), ...
//             sum( mMeas(:,X_AXIS).^2 ), ...
//             sum( mMeas(:,X_AXIS) .* mMeas(:,Y_AXIS) ), ...
//             sum( mMeas(:,X_AXIS) ) ];
  Amat[2][0] += x*x * y ;
  Amat[2][1] += x * y*y ;
  Amat[2][2] += x*x ;
  Amat[2][3] += x * y ;
  Amat[2][4] += x ;

//
//A(4,1:5) = [ sum( mMeas(:,X_AXIS) .* mMeas(:,Y_AXIS).^2 ), ...
//             sum( mMeas(:,Y_AXIS).^3 ), ...
//             sum( mMeas(:,X_AXIS) .* mMeas(:,Y_AXIS) ), ...
//             sum( mMeas(:,Y_AXIS).^2 ), ...
//             sum( mMeas(:,Y_AXIS) ) ];
  Amat[3][0] += x * y*y ;
  Amat[3][1] += y*y*y ;
  Amat[3][2] += x * y ;
  Amat[3][3] += y*y ;
  Amat[3][4] += y ;
//
//A(5,1:5) = [ sum( mMeas(:,X_AXIS) .* mMeas(:,Y_AXIS) ), ...
//             sum( mMeas(:,Y_AXIS).^2 ), ...
//             sum( mMeas(:,X_AXIS) ), ...
//             sum( mMeas(:,Y_AXIS) ), ...
//             length(mMeas(:,X_AXIS)) ];
  Amat[4][0] += x * y ;
  Amat[4][1] += y*y ;
  Amat[4][2] += x ;
  Amat[4][3] += y ;
  Amat[4][4] = NUMPOINT_LS;

  return 1;
}

uint8_t initializeAmat()
{
    uint8_t i, j;
    for (i=0; i<5; i++) {
        for (j=0; j<5; j++) {
            Amat[i][j] = 0;
        }
    }
    return 1;
}

uint8_t accumbvec(float x, float y)
{
//  b(1,1) = -sum( mMeas(:,X_AXIS).^3 .* mMeas(:,Y_AXIS) );
//  b(2,1) = -A(1,1);
//  b(3,1) = -sum( mMeas(:,X_AXIS).^3 );
//  b(4,1) = -A(1,3);
//  b(5,1) = -A(3,3);
  bvec[0][0] -= x*x*x*y;   //-sum( mMeas(:,X_AXIS).^3 .* mMeas(:,Y_AXIS) );
  bvec[1][0] = -Amat[0][0];
  bvec[2][0] -= x*x*x;  //-sum( mMeas(:,X_AXIS).^3 );
  bvec[3][0] = -Amat[0][2];
  bvec[4][0] = -Amat[2][2];

  return 1;
}

uint8_t initializebvec()
{
    uint8_t i;
    for (i=0; i<5; i++) {
        bvec[i][0] = 0;
    }
    return 1;
}

uint8_t formCmat()
{
    //  % Form a matrix that includes b as the final column.
    //  C(1:5,1:5) = A;
    //  C(:,6) = b;
    uint8_t i, j;
    for (i=0; i<5; i++) {
        for (j=0; j<6; j++) {
            if (j < 5) {
                Cmat[i][j] = Amat[i][j];
            } else {
                Cmat[i][j] = bvec[i][0];
            }
        }
    }
    return 1;
}

float absf(float x)
{
    if (x<0)
        return -x;
    else
        return x;
}

uint8_t findPivot(uint8_t n, uint8_t *pivot)
{
    float CrowMax = absf(Cmat[n][n]);
    uint8_t i;

    // Save off the value in row n (in case it is the maximum value)
    *pivot = n;
    for (i=n+1; i<5; i++) {
        if (absf(Cmat[i][n]) > CrowMax) {
            CrowMax = absf(Cmat[i][n]);
            *pivot = i;
        }
    }
    return 1;
}

/** ****************************************************************************
 * @name MagAlign API call to collect data for Hard/Soft Iron calibration
 * @brief called every frame in taskDataAcquisition.c
 *
 * Trace:
 *
 * @param N/A
 * @retval always returns 1
 ******************************************************************************/
uint8_t MagAlign( void )
{
    int32_t hardIron_q27[2]; // pass the calculated data out _magCal
    int32_t softIron_q27[2];

    if( gAlgorithm.calState == MAG_ALIGN_STATUS_START_CAL_WITH_AUTOEND ) {
        if( _PerformMagAlign( gAlgorithm.calState, hardIron_q27, softIron_q27 ) ) {
            // end of calibration
            gAlgorithm.calState = MAG_ALIGN_STATUS_TERMINATION;

            // Unit has completed ~380 degree rotation. Write data to memory
            // Convert values from Q27 to 16-bit integers with a range of [-10,10)
            // This is the scaled hard-iron value in 'normal' numbers (no q-scaling, just integers)
            gConfiguration.hardIronBias[0] = _qmul( TWO_POW16_OVER_20_q19, hardIron_q27[0], 19, 27, 16 ) >> 16;
            gConfiguration.hardIronBias[1] = _qmul( TWO_POW16_OVER_20_q19, hardIron_q27[1], 19, 27, 16 ) >> 16;

            // this should be unsigned int? [0, 2)
            // unsigned int! (Output to Nav-View reflect the value above)
            gConfiguration.softIronScaleRatio = _qmul( TWO_POW16_OVER_2_q15, softIron_q27[0], 15, 27, 16 ) >> 16;

            // [-pi, pi) -- Value not displayed in Nav-View
            gConfiguration.softIronAngle = _qmul( MAXINT16_OVER_2PI_q18, softIron_q27[1], 18, 27, 16 ) >> 16;

            // Values that go into the hard/soft-iron variables in gConfiguration
            //   are loaded into gMagAlign variables
            InitMagAlignParams();

            // Tell the user (Nav-View) that the process is finished
            magUcbPacket.packetType = UCB_MAG_CAL_3_COMPLETE;
            SendUcbPacket( PRIMARY_UCB_PORT, &magUcbPacket );

            if (getUserCommunicationType() == SPI_COMM) {
                /// write gConfiguration with the new mag fields
                WriteMagAlignParamsToMemory(PRIMARY_UCB_PORT, &magAlignUcbPacket);
                // set SPI status to UCB_MAG_CAL_3_COMPLETE - Tell spi master we are complete
                gUserSpi.DataRegister[SPI_REG_MAG_ALIGN_READ] = MAG_ALIGN_STATUS_TERMINATION; // 0xb
                // put copy of hard soft iron in the Mag align results registers
                FillSpiDataRegister_MagAlign();
            }

            // reset / restart the Kalman filter
            InitializeAlgorithmStruct(&gAlgorithm);

            //  Go into stabilization phase for 5 additional seconds (100 Hz ==> 500 counts).  This
            //    provides the user additional time to bring their system to a halt before the
            //    DMU380 begins initialization.
            gAlgorithm.stateTimer = gAlgorithm.Duration.Stabilize_System +
                                    (uint32_t)(5*gAlgorithm.callingFreq);
        }
    } else if( gAlgorithm.calState == MAG_ALIGN_STATUS_START_CAL_WITHOUT_AUTOEND ) {
        ;
    }

    return 1;
}

#define  DT_100HZ_Q30  10737418
#define  DT_200HZ_Q30  5368709

#define  THREE_HUNDRED_EIGHTY_DEGREES_Q28  1780331364

/** ****************************************************************************
 * @name _magCal
 * @brief gather data during the 380-degree turn for hard/soft iron alignment
 *
 * Trace:
 * [SDD_CALC_MAG_CAL_INIT_01 <-- SRC_MAG_CAL]
 * [SDD_CALC_MAG_CAL_01 <-- SRC_MAG_CAL]
 * [SDD_CALC_MAG_CAL_02 <-- SRC_MAG_CAL]
 * [SDD_CALC_MAG_CAL_03 <-- SRC_MAG_CAL]
 *
 * @param [in]  state - state indicator of mag-align steps
 * @param [out] state - state indicator of mag-align steps
 * @param [out] *hardIron - hard iron
 * @param [out] *softIron - soft iron
 * @retval  TRUE: the process is complete
 *          FALSE the process is in progress
 ******************************************************************************/
static uint8_t _PerformMagAlign( int32_t state,
                                 int32_t *hardIron_q27,
                                 int32_t *softIron_q27 )
{
    static BOOL initMagAlignFlag = TRUE;

    static int32_t psi_q28[TOTAL_POINTS] = {0};
    static int32_t psiPrev_q28;
    int32_t        psiComp_q28;

    //static int32_t zAxisRateBias_q27; // this is never set

    // variables used to compute the magnetic field in the plane normal to gravity
    int32_t sinRoll_q30;
    int32_t cosRoll_q30;
    int32_t sinPitch_q30;
    int32_t cosPitch_q30;

    int32_t MagFieldVector_q27[3];
    int32_t xyMagFieldVector_q27[3] = {0};
    float xmag, ymag;

    static uint16_t sampleNumber = 0;

    uint8_t axis;
    static int32_t maxMagField_q27[2] = {0};
    static int32_t minMagField_q27[2] = {0};

    static int64_t triggerAngle_q29;

    int32_t rollPitchAngles_q29[2] = {0};
    int32_t AccelUnitVector_q30[3];

    // NOTE: THE FOLLOWING ALGORITHM IS FUNDAMENTALLY INCORRECT AS IT PERFORMS ALL CALCULATIONS IN
    //       THE NED-FRAME.  ALL READINGS AND THE HARD/SOFT-IRON EFFECTS ARE IN THE BODY-FRAME
    //       SO ALL CALCULATIONS SHOULD BE IN THE BODY-FRAME.  HOWEVER, HOW MUCH THE END RESULT IS
    //       AFFECTED IS UNKNOWN.  THE DIFFERENCE BETWEEN THE TWO MAY BE INSIGNIFICANT.

    // Compute the roll and pitch angles from the accelerometer measurements (this is done to
    //   ensure the mag-align is valid even if the Kalman filter estimates are not)
    VectorNormalize_q30( &gAlgorithm.scaledSensors_q27[XACCEL], AccelUnitVector_q30 );

    /// At rest, the gravity vector is the negative of the system acceleration
    AccelUnitVector_q30[X_AXIS] = -AccelUnitVector_q30[X_AXIS];
    AccelUnitVector_q30[Y_AXIS] = -AccelUnitVector_q30[Y_AXIS];
    AccelUnitVector_q30[Z_AXIS] = -AccelUnitVector_q30[Z_AXIS];

    // For a 321 rotation sequence:
    //   rollAngle  = atan( gHaty/gHatz );
    //   pitchAngle = asin( gHatx );
switch(1) {
case 1:
    rollPitchAngles_q29[ROLL]  =  atan2_q29( AccelUnitVector_q30[Y_AXIS],
                                             AccelUnitVector_q30[Z_AXIS], 30 );
    rollPitchAngles_q29[PITCH] = -asin_q29( AccelUnitVector_q30[X_AXIS] );
    break;
case 2:
    rollPitchAngles_q29[ROLL]  = 0;
    rollPitchAngles_q29[PITCH] = 0;
    break;
case 3:
    rollPitchAngles_q29[ROLL]  = doubleToQ29((double)gKalmanFilter.eulerAngles[0]);
    rollPitchAngles_q29[PITCH] = doubleToQ29((double)gKalmanFilter.eulerAngles[1]);
    break;
default:
    rollPitchAngles_q29[ROLL]  = 0;
    rollPitchAngles_q29[PITCH] = 0;
}

//    if( initMagAlignFlag == TRUE ) {
//      filteredMag[0] = xyMagFieldVector_q27[X_AXIS];
//      lastMag[0] = xyMagFieldVector_q27[X_AXIS];
//      filteredMag[1] = xyMagFieldVector_q27[Y_AXIS];
//      lastMag[1] = xyMagFieldVector_q27[Y_AXIS];
//    {
//      filteredAcc[0] = rollPitchAngles_q29[ROLL];
//      lastAcc[0] = rollPitchAngles_q29[ROLL];
//      filteredAcc[1] = rollPitchAngles_q29[PITCH];
//      lastAcc[1] = rollPitchAngles_q29[PITCH];
//    }

//    lowPass2Pole(&filteredAcc[0],&lastAcc[0],rollPitchAngles_q29[ROLL],0,5,5);
//    lowPass2Pole(&filteredAcc[1],&lastAcc[1],rollPitchAngles_q29[PITCH],0,5,5);
//    rollPitchAngles_q29[ROLL] = filteredAcc[0];
//    rollPitchAngles_q29[PITCH] = filteredAcc[1];

    sinRoll_q30  = sin_q30( rollPitchAngles_q29[ROLL] );
    cosRoll_q30  = cos_q30( rollPitchAngles_q29[ROLL] );
    sinPitch_q30 = sin_q30( rollPitchAngles_q29[PITCH] );
    cosPitch_q30 = cos_q30( rollPitchAngles_q29[PITCH] );

    // Transform the magnetic-field vector from the sensor-frame to the inertial (NED)
    //   frame, in which the d-axis is aligned with the gravity vector.  The x
    //   and y-axes assume a zero yaw angle.
    //             N   N S     S
    //            m  =  R  *  m
    MagFieldVector_q27[X_AXIS] = (int32_t)( gAlgorithm.scaledSensors_q27[XMAG] );
    MagFieldVector_q27[Y_AXIS] = (int32_t)( gAlgorithm.scaledSensors_q27[YMAG] );
    MagFieldVector_q27[Z_AXIS] = (int32_t)( gAlgorithm.scaledSensors_q27[ZMAG] );

    int32_t tempVar_q27 = _qmul( sinRoll_q30, MagFieldVector_q27[Y_AXIS], 30, 27, 27 ) +
                          _qmul( cosRoll_q30, MagFieldVector_q27[Z_AXIS], 30, 27, 27 );
    xyMagFieldVector_q27[X_AXIS] = _qmul( cosPitch_q30, MagFieldVector_q27[X_AXIS], 30, 27, 27 ) +
                                   _qmul( sinPitch_q30, tempVar_q27, 30, 27, 27 );
    xyMagFieldVector_q27[Y_AXIS] = _qmul( cosRoll_q30, MagFieldVector_q27[Y_AXIS], 30, 27, 27 ) -
                                   _qmul( sinRoll_q30, MagFieldVector_q27[Z_AXIS], 30, 27, 27 );
    xyMagFieldVector_q27[Z_AXIS] = _qmul( -sinPitch_q30, MagFieldVector_q27[X_AXIS], 30, 27, 27 ) +
                                   _qmul( cosPitch_q30, tempVar_q27, 30, 27, 27 );

//    lowPass2Pole(&filteredMag[0],&lastMag[0],xyMagFieldVector_q27[X_AXIS],0,5,5);
//    lowPass2Pole(&filteredMag[1],&lastMag[1],xyMagFieldVector_q27[Y_AXIS],0,5,5);
//    xyMagFieldVector_q27[X_AXIS] = filteredMag[0];
//    xyMagFieldVector_q27[Y_AXIS] = filteredMag[1];

    xmag = q27ToFloat(xyMagFieldVector_q27[X_AXIS]);
    ymag = q27ToFloat(xyMagFieldVector_q27[Y_AXIS]);

    if( initMagAlignFlag == TRUE ) { // first time through
        initMagAlignFlag       = FALSE;

        sampleNumber           = 0;

        // Init the mag-readings array and save the magnetometer readings into an array (first x &
        //   y-data points)
        memset( &magReadings_q27, 0, sizeof(magReadings_q27));
        magReadings_q27[X_AXIS][sampleNumber] = xyMagFieldVector_q27[X_AXIS];
        magReadings_q27[Y_AXIS][sampleNumber] = xyMagFieldVector_q27[Y_AXIS];

        // Initialize the min/max variables with the current reading
        minMagField_q27[X_AXIS] = xyMagFieldVector_q27[X_AXIS];   minMagField_q27[Y_AXIS] = xyMagFieldVector_q27[Y_AXIS];
        maxMagField_q27[X_AXIS] = xyMagFieldVector_q27[X_AXIS];   maxMagField_q27[Y_AXIS] = xyMagFieldVector_q27[Y_AXIS];

        // Save the angle to an array
        psi_q28[sampleNumber] = 0;
        psiPrev_q28           = psi_q28[sampleNumber];

        // Set the initial trigger value to 1.0 degree
        triggerAngle_q29 = ONE_DEGREE_Q29;
    } else {
        // Continually track min and max bounds -- used for the "box-centroid" method
        //   (sampled at one degree increments, take the average of the complete
        //   set of points to produce a better estimate of the bias)
        // Note: runs at 200 Hz.  Will get multiple points, not just at 1 deg increments.
        for( axis = X_AXIS; axis <= Y_AXIS; axis++ ) {
            if( minMagField_q27[axis] > xyMagFieldVector_q27[axis] ) {
                minMagField_q27[axis] = xyMagFieldVector_q27[axis];
            } else if( maxMagField_q27[axis] < xyMagFieldVector_q27[axis] ) {
                maxMagField_q27[axis] = xyMagFieldVector_q27[axis];
            }
        }

        // psi (heading) = psiPrev + wz * dt
        // Run the system at 200 Hz unless it has potential to be used with GPS
        if ( timer.odr == ODR_200_HZ ) {
            psiComp_q28 = psiPrev_q28 + _qmul( gAlgorithm.scaledSensors_q27[ZRATE], DT_200HZ_Q30, 27, 30, 28 );
        } else {
            psiComp_q28 = psiPrev_q28 + _qmul( gAlgorithm.scaledSensors_q27[ZRATE], DT_100HZ_Q30, 27, 30, 28 );
        }
        psiPrev_q28 = psiComp_q28;

        // possible to dither back-and-forth, covering only a portion of the circle
        if( nabs_64( psiComp_q28 ) >= ( triggerAngle_q29 >> 1 ) ) {
            sampleNumber++;

            // Increment the trigger angle by the desired amount
            triggerAngle_q29 = triggerAngle_q29 + ONE_DEGREE_Q29;

            if (sampleNumber == 10) {
                initializeAmat();
                initializebvec();
                accumAMat(xmag, ymag);
                accumbvec(xmag, ymag);
                magNum = 1;
            } else if ( ( sampleNumber > 10 ) &&
                        ( sampleNumber < (NUMPOINT_LS + 10) ) )
            {
                accumAMat(xmag, ymag);
                accumbvec(xmag, ymag);
                magNum++;
            }

            // save the angle and magnetic-field data to the storage arrays
            psi_q28[sampleNumber] = psiComp_q28;
            magReadings_q27[X_AXIS][sampleNumber] = xyMagFieldVector_q27[X_AXIS];
            magReadings_q27[Y_AXIS][sampleNumber] = xyMagFieldVector_q27[Y_AXIS];
        }
    }

    // End data collection and process the data
    if( ( ( state == MAG_ALIGN_STATUS_START_CAL_WITH_AUTOEND ) &&
          ( nabs_64( psi_q28[sampleNumber] ) > THREE_HUNDRED_EIGHTY_DEGREES_Q28 ) ) ||
        ( state == MAG_ALIGN_STATUS_TERMINATION ) )
    {
        // Compute the hard-iron bias and the soft-iron field distortion
        _ProcessMagAlignData( minMagField_q27, maxMagField_q27, sampleNumber, hardIron_q27, softIron_q27 );
        initMagAlignFlag = TRUE;

        return TRUE;
    }
    return FALSE;
}


/** ***************************************************************************
* @name _ProcessMagAlignData
* @brief compute hard and soft iron at the completion of the 380-degree turn
*        for hard/soft iron alignment
*
* Trace:
* [SDD_CALC_MAG_CAL_02_BIAS <-- SRC_STOP_MAG_CAL]
* [SDD_CALC_MAG_CAL_02_RATIO1 <-- SRC_STOP_MAG_CAL]
* [SDD_CALC_MAG_CAL_02_RATIO2 <-- SRC_STOP_MAG_CAL]
* [SDD_CALC_MAG_CAL_02_ANGLE <-- SRC_STOP_MAG_CAL]
*
* @param [in] *minMag - minimal XY mag during the turn
* @param [in] *maxMag - maximal XY mag during the turn
* @param [in] *numofpts - number of the data sample
* @param [out] *hardIron - hard iron
* @param [out] *softIron - soft iron
* @retval N/A
******************************************************************************/
static void _ProcessMagAlignData( int32_t  *minMag_q27, int32_t  *maxMag_q27,
                                  uint16_t numberOfSamples,
                                  int32_t  *hardIron_q27, int32_t  *softIron_q27)
{
    /*uint16_t sampleNumber = 10;
    uint16_t rMaxIndex;
    uint16_t sampleNumberP90 = 0;
    uint16_t sampleNumberP180 = 0;
    uint16_t sampleNumberP270 = 0;
//    uint16_t rMax_x_q27, rMax_y_q27;
    int32_t  xMag_q27, yMag_q27;
    int64_t  rSq_q27, rSqP90_q27; //rSqRatioMin_q27, rSqRatio_q27,
    int64_t  rSqMin_q27, rSqMax_q27;*/
//    static real rSq_real[380];



    // Form a matrix that contains A in the first 5 columns and appends b as the
    // sixth column
    formCmat();

    // Solve for the parameters using pivoting (the pivot is the largest value
    //   in the column).  Find the largest value and swap that row with the
    //   current row.
    uint8_t pivot;
    findPivot(0, &pivot);

    float temp;
    uint8_t j;
    for (j=0; j<6; j++)
    {
        temp = Cmat[pivot][j];
        Cmat[pivot][j] = Cmat[0][j];
        Cmat[0][j] = temp;
    }

    // Find the multiplier for each row that enables subtraction by the first row
    //   to result in a leading zero
    float sf[5];
    sf[1] = Cmat[1][0]/Cmat[0][0];
    sf[2] = Cmat[2][0]/Cmat[0][0];
    sf[3] = Cmat[3][0]/Cmat[0][0];
    sf[4] = Cmat[4][0]/Cmat[0][0];

    for (j=0; j<6; j++)
    {
        Cmat[1][j] -= sf[1]*Cmat[0][j];
        Cmat[2][j] -= sf[2]*Cmat[0][j];
        Cmat[3][j] -= sf[3]*Cmat[0][j];
        Cmat[4][j] -= sf[4]*Cmat[0][j];
    }

    // Repeat the reduction for row #2
    findPivot(1, &pivot);

    for (j=0; j<6; j++)
    {
        temp = Cmat[pivot][j];
        Cmat[pivot][j] = Cmat[1][j];
        Cmat[1][j] = temp;
    }

    sf[2] = Cmat[2][1]/Cmat[1][1];
    sf[3] = Cmat[3][1]/Cmat[1][1];
    sf[4] = Cmat[4][1]/Cmat[1][1];

    for (j=0; j<6; j++)
    {
        Cmat[2][j] -= sf[2]*Cmat[1][j];
        Cmat[3][j] -= sf[3]*Cmat[1][j];
        Cmat[4][j] -= sf[4]*Cmat[1][j];
    }

    //    % row #3
    findPivot(2, &pivot);

    for (j=0; j<6; j++)
    {
        temp = Cmat[pivot][j];
        Cmat[pivot][j] = Cmat[2][j];
        Cmat[2][j] = temp;
    }

    //
    sf[3] = Cmat[3][2]/Cmat[2][2];
    sf[4] = Cmat[4][2]/Cmat[2][2];

    for (j=0; j<6; j++)
    {
      Cmat[3][j] -= sf[3]*Cmat[2][j];
      Cmat[4][j] -= sf[4]*Cmat[2][j];
    }

    // Row #4
    findPivot(3, &pivot);

    for (j=0; j<6; j++)
    {
      temp = Cmat[pivot][j];
      Cmat[pivot][j] = Cmat[3][j];
      Cmat[3][j] = temp;
    }

    sf[4] = Cmat[4][3]/Cmat[3][3];

    for (j=0; j<6; j++)
    {
      Cmat[4][j] -= sf[4]*Cmat[3][j];
    }

    float x[5];
    // Back-substitution to solve for the ellipse parameters
    x[4] = Cmat[4][5] / Cmat[4][4];
    x[3] = (Cmat[3][5] - Cmat[3][4]*x[4]) / Cmat[3][3];
    x[2] = (Cmat[2][5] - Cmat[2][4]*x[4] - Cmat[2][3]*x[3]) / Cmat[2][2];
    x[1] = (Cmat[1][5] - Cmat[1][4]*x[4] - Cmat[1][3]*x[3] - Cmat[1][2]*x[2]) / Cmat[1][1];
    x[0] = (Cmat[0][5] - Cmat[0][4]*x[4] - Cmat[0][3]*x[3]- Cmat[0][2]*x[2] - Cmat[0][1]*x[1]) / Cmat[0][0];

    // Place the solution into the ellipse parameters (based on analytical geometry)
    float A,B,C,D,E,F;
    A = 1;
    B = x[0];
    C = x[1];
    D = x[2];
    E = x[3];
    F = x[4];

    // Find the ellipse parameters (in canonical form)
    //  Center of ellipse
    float xc,yc;
    xc = ( 2*C*D - B*E ) / ( B*B - 4*A*C );
    yc = ( 2*A*E - B*D ) / ( B*B - 4*A*C );

    //  Angle of semi-major axis
    float theta;
    if( B == 0 ) {
        if( A < C ) {
            theta = 0;
        } else {
            theta = 90 * PI / 180;
        }
    } else {
        theta = atan2f( ( C-A-sqrtf( (A-C)*(A-C) + B*B ) ), B ) * 180/PI;
    }

    if( theta < -90 ) {
      theta = theta + 180;
    } else if( theta > 90 ) {
      theta = theta - 180;
    }

    //  Major/minor-axes and ratio of minor to major (sf)
    float den,aEst,bEst,sf1;
    den = B*B - 4*A*C;
    aEst = -sqrtf( 2*(A*E*E + C*D*D - B*D*E + den*F ) *
                 ( A + C + sqrtf( (A-C)*(A-C) + B*B ) ) ) / den;

    bEst = -sqrtf( 2*(A*E*E + C*D*D - B*D*E + den*F ) *
                    ( A + C - sqrtf( (A-C)*(A-C) + B*B ) ) ) / den;

    sf1 = bEst/aEst;

    hardIron_q27[X_AXIS] = floatToQ27(xc);
    hardIron_q27[Y_AXIS] = floatToQ27(yc);

    // softIron[0]: ratio of the semi-minor axis of the ellipse to the semi-major axis
    softIron_q27[0]   = floatToQ27(sf1);

    // softIron[1] is the angle that the semi-major axis makes with the x-axis
    softIron_q27[1]  = floatToQ27(theta/180*PI);
}

#define  TWO_OVER_2POW16     3.0517578125e-05
#define  TWENTY_OVER_2POW16  3.0517578125e-04

/** ***************************************************************************
 * @name InitMagAlignParams
 * @brief initialize the parameters for magnetic heading calculation
 * taskDataAcquisition.c and HardSoftIronCalibration()
 * Trace:
 * [SDD_INIT_EXT_MAG_CONFIG <-- SRC_INIT_MAGALIGN_RESULT]
 * @param N/A
 * @retval None
 ******************************************************************************/
void InitMagAlignParams(void)
{
    // Copy mag-alignment parameters from the EEPROM (loaded in xbowsp_init.c) to Mag-Align struct;
    //   values are used to compensate hard/soft-iron effects in the EKF algorithms.
    gMagAlign.hardIronBias[0] = (real)gConfiguration.hardIronBias[0] * (real)TWENTY_OVER_2POW16;
    gMagAlign.hardIronBias[1] = (real)gConfiguration.hardIronBias[1] * (real)TWENTY_OVER_2POW16;

    // TWO_OVER_TWO_POW_16    = 3.0517578125e-05
    // TWO_PI_OVER_TWO_POW_16 = 9.587379924285257e-05
    gMagAlign.softIronScaleRatio = (real)gConfiguration.softIronScaleRatio * (real)3.0517578125e-05;
    gMagAlign.softIronAngle      = (real)gConfiguration.softIronAngle * (real)9.587379924285257e-05;

    // Calculate the Scale-Factor matrix (to account for soft-iron effects)
    _CalcMagAlignScaleFactor();
}


/** ***************************************************************************
 * @name _CalcMagAlignScaleFactor
 * @brief calculate scale factor matrix for accommodating soft iron angle effect
 *
 * Trace:
 * [SDD_CALC_SOFT_IRON_MATRIX <-- SRC_CALC_MAG_SCALE_PARAMETER]
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
static void _CalcMagAlignScaleFactor(void)
{
    real sinAlpha;
    real cosAlpha;
    real sinAlphaSquared;
    real cosAlphaSquared;
    real sinAlphaTimesCosAlpha;

    // Soft-iron angle (in rad) expressed in Q27 format
    sinAlpha = sin( gMagAlign.softIronAngle );
    cosAlpha = cos( gMagAlign.softIronAngle );

    // Alpha is the rotation angle of the best-fit ellipse to the magnetic-field
    //   data.
    sinAlphaSquared       = sinAlpha * sinAlpha;
    cosAlphaSquared       = cosAlpha * cosAlpha;
    sinAlphaTimesCosAlpha = sinAlpha * cosAlpha;

    // The following is equivalent to R*S*R', which adjusts for the ellipse
    //   rotation and scaling of the x and y-components of the magnetic field
    //   in the presence of soft-iron effects (hard-iron generates a bias and
    //   is handled elsewhere).
    gMagAlign.SF[0] = cosAlphaSquared * gMagAlign.softIronScaleRatio + sinAlphaSquared;

    gMagAlign.SF[1] = sinAlphaTimesCosAlpha * gMagAlign.softIronScaleRatio - sinAlphaTimesCosAlpha;
    gMagAlign.SF[2] = gMagAlign.SF[1];

    gMagAlign.SF[3] = sinAlphaSquared * gMagAlign.softIronScaleRatio + cosAlphaSquared;
}

// ------------- The following functions and variables are unused by the firmware -------------

//#define  TWENTY_DEGREES_Q29  187403301  // 20 degrees -> radians in Q29 format
//#define  THREE_HUNDRED_EIGHTY_DEGREES_Q29  3560662727  // 380 degrees -> radians in Q29 format

//#define  PI_Q28      843314857
//#define  TWO_PI_Q28 1686629713

//#define  PI_Q29     1686629713
//#define  TWO_PI_Q29 3373259426   // q29 values saved in int64 - int32 overflow

//static void _calcMagHeading(double *AHRSrp,double *remoteMag, double*magHeading) ;
//void _calcLeveledMags(double *AHRSrp, double *remoteMag);

//int16_t  magRollOffset;
//int16_t  magPitchOffset;

//int16_t  hardIronBias[2];
//uint16_t softIronScaleRatio;
//int16_t  softIronAngle;
//double   sfMat[4];

//double   sMagx1[TOTAL_POINTS];
//double   sMagy1[TOTAL_POINTS];


///** ****************************************************************************
// * @name _calcLeveledMags floating point version
// * @brief project the Earth's magnetometer field from CRM body frame to level
// *        frame. floating point version of _TransformMagFieldToNEDFrame() in
// *        kalmanfilter.c
// * Trace:
// * [SDD_CALC_LEVELED_MAG1 <-- SRC_CALC_LEVELED_MAG]
// * [SDD_CALC_LEVELED_MAG2 <-- SRC_CALC_LEVELED_MAG]
// * @param [in] *AHRSrp - AHRS's roll and pitch
// * @param [in] *remoteMag -  the Earth's magnetometer field
// * @retval N/A
// ******************************************************************************/
//void _calcLeveledMags(double *AHRSrp,
//                     double *remoteMag)
//{
//	int i;
//	double cosP;
//    double sinP;
//    double cosR;
//    double sinR;
//    double mag[3];
//    double levelMagswrtAHRS[3];
//    double extRoll;
//    double extPitch;
//
//	extRoll  = magRollOffset  / ((double)MAXUINT16_OVER_2PI);
//	extPitch = magPitchOffset / ((double)MAXUINT16_OVER_2PI);
//
//	for(i = 0;i < 3;i++) {
//      mag[i]= remoteMag[i];
//    }
//
//	cosR = cos(extRoll);
//	sinR = sin(extRoll);
//	cosP = cos(extPitch);
//	sinP = sin(extPitch);
//
//	levelMagswrtAHRS[0] =  cosP * mag[0] + sinP * (cosR * mag[2] + sinR * mag[1]);
//	levelMagswrtAHRS[1] =  cosR * mag[1] - sinR * mag[2];
//	levelMagswrtAHRS[2] = -sinP * mag[0] + cosP * (cosR * mag[2] + sinR * mag[1]);
//
//	/// Level mag wrt AHRS frame to Level frame
//	cosR = cos(AHRSrp[0]);
//	sinR = sin(AHRSrp[0]);
//	cosP = cos(AHRSrp[1]);
//	sinP = sin(AHRSrp[1]);
//
//	gAlgorithm.leveledMags[0] = cosP * levelMagswrtAHRS[0] +
//                                sinP * (cosR * levelMagswrtAHRS[2] +
//							    sinR * levelMagswrtAHRS[1]);
//	gAlgorithm.leveledMags[1] = cosR * levelMagswrtAHRS[1] -
//                                sinR * levelMagswrtAHRS[2];
//}
//
//
///** ***************************************************************************
// * @name _calcMagHeading floating point version
// * @brief calculate magnetic heading. floating point version of
// *        _FieldVectorsToEulerAngles() in KalmanFilter.c
// * Trace:
// * [SDD_CALC_HEADING1 <-- SRC_CALC_MAG_HEADING]
// * [SDD_CALC_HEADING2 <-- SRC_CALC_MAG_HEADING]
// * [SDD_CALC_HEADING3 <-- SRC_CALC_MAG_HEADING]
// * [SDD_CALC_HEADING4 <-- SRC_CALC_MAG_HEADING]
// * [SDD_CALC_HEADING5 <-- SRC_CALC_MAG_HEADING]
// * @param [in] *AHRSrp:  AHRS's roll and pitch
// * @param [in] *remoteMag:  magnetic field measurement from remote mag
// * @param [out] *magHeading: magnetic heading
// * @retval N/A
// ******************************************************************************/
//static void _calcMagHeading( double *AHRSrp,
//                             double *remoteMag,
//                             double *magHeading )
//{
//	double xMag;
//    double yMag;
//    double tmpX;
//    double tmpY;
//
//	_calcLeveledMags(AHRSrp, remoteMag);
//
//	tmpX = hardIronBias[0] / ((double)IRON_SCALE);
//    tmpY = hardIronBias[1] / ((double)IRON_SCALE);
//
//	tmpX = gAlgorithm.leveledMags[0] - tmpX ;
//    tmpY = gAlgorithm.leveledMags[1] - tmpY;
//
//    xMag = sfMat[0] * tmpX + sfMat[1] * tmpY;
//	yMag = sfMat[2] * tmpX + sfMat[3] * tmpY;
//
//	*magHeading = atan2(-yMag, xMag);
//	gAlgorithm.compassHeading = *magHeading;
//}
//
//
///** ****************************************************************************
// * @name MagCalOutOfBounds
// * @brief check if the magnitude of the mag-align results are within the
// *        thresholds
// * Trace: [SDD_CHECK_MAG_OUT_OF_BOUNDS <-- SRC_MAGCAL_IN_BOUNDS]
// * @param N/A
// * @retval TRUE: the magnitude is outside the bounds
// *         FALSE: the magnitude is within the bounds
// ******************************************************************************/
//bool MagCalOutOfBounds (void)
//{
//	bool outOfBounds = FALSE;
//	uint16_t tmp;
//
//	tmp =(uint16_t)(abs((int32_t)gConfiguration.softIronScaleRatioExt - IRON_SCALE)); // IRON_SCALE = 32768
//
//	if ( ( (abs(gConfiguration.hardIronBiasExt[0])) > gCalibration.HardIronLimit ) ||
//	     ( (abs(gConfiguration.hardIronBiasExt[1])) > gCalibration.HardIronLimit ) ||
//	     (  tmp > gCalibration.SoftIronLimit ) ||
//	     ( gConfiguration.softIronScaleRatioExt == 0) )
//	{   /// work around because abs(-32768) returns zero!
//		outOfBounds = TRUE;
//	}
//
//	if (((abs(gConfiguration.OffsetAnglesExtMag[0])) > gCalibration.RollIncidenceLimit ) ||
//	    ((abs(gConfiguration.OffsetAnglesExtMag[1])) > gCalibration.PitchIncidenceLimit ))
//	{
//		outOfBounds = TRUE;
//	}
//	return outOfBounds;
//}
