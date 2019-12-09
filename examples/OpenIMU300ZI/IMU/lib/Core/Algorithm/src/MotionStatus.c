/*****************************************************************************
 * @file    MotionStatus.c
 * @brief   Calculate sensor stats, and detect motion status using IMU/ODO/GNSS
 * @author  Dong Xiaoguang
 * @version 1.0
 * @date    20190801
 *****************************************************************************/
#include <math.h>
#include "MotionStatus.h"
#include "buffer.h"
#include "VectorMath.h"


//=============================================================================
// Filter variables (Third-Order BWF w/ default 5 Hz Cutoff)
#define FILTER_ORDER 3

#define CURRENT 0
#define PASTx1  1
#define PASTx2  2
#define PASTx3  3
/* Replace this with a fuction that will compute the coefficients so the
 * input is the cutoff frequency in Hertz
 */
#define  NO_LPF              0
#define  TWO_HZ_LPF          1
#define  FIVE_HZ_LPF         2
#define  TEN_HZ_LPF          3
#define  TWENTY_HZ_LPF       4
#define  TWENTY_FIVE_HZ_LPF  5
#define  N_LPF               6

#define SAMPLES_FOR_STATS    20     /* 20 samples can give a relative good estimate of var
                                     * This value should not be below FILTER_ORDER.
                                     */

 /******************************************************************************
  * @brief Get filter coefficients of a 3rd Butterworth low-pass filter.
  * For now only a few specific cut-off frequencies are supported.
  * TRACE:
  * @param [in] lpfType   Low-pass filter cut-off frequency.
  * @param [in] callingFreq   Sampling frequency, only 100Hz and 200Hz are supported.
  * @param [out] b    coefficients of the numerator of the filter.
  * @param [out] a    coefficients of the denominator of the filter.
  * @retval None.
 ******************************************************************************/
static void _PopulateFilterCoefficients(uint8_t lpfType, uint8_t callingFreq, real *b, real *a);

/******************************************************************************
 * @brief Process input data through a low-pass Butterworth filter.
 * TRACE:
 * @param [in]  in      Input data
 * @param [in]  bfIn    Input data buffer
 * @param [in]  bfOut   Output data buffer
 * @param [in]  b       Numerator coef of the filter 
 * @param [in]  a       Denominator coef of the filter
 *
 * @param [out] filtered    Filtered IMU data.
 * @retval None.
******************************************************************************/
static void LowPassFilter(real *in, Buffer *bfIn, Buffer *bfOut, real *b, real *a, real *filtered);

/******************************************************************************
 * @brief Compute mean and var of the input data.
 * Calculate mena and var of the latest n samples.
 * TRACE:
 * @param [in] bf       The buffer to hold the latest n samples.
 * @param [in] latest   The latest sample.
 * @param [in/out] mean Mean of samples already in buffer is used as input, and
 *                      mean of latest samples (remove oldest and include latest)
 *                      is returned as output.
 * @param [in/out] var  Var of samples already in buffer is used as input, and
 *                      var of latest samples (remove oldest and include latest)
 *                      is returned as output.
 * @retval None.
******************************************************************************/
static void ComputeStats(Buffer *bf, real *latest, real *mean, real *var);

/******************************************************************************
 * @brief Detect zero velocity using IMU data.
 * TRACE:
 * @param [in]  gyroVar     variance of gyro    [rad/s]^2
 * @param [in]  gyroMean    mean of gyro        [rad/s]
 * @param [in]  accelVar    variance of accel   [m/s/s]^2
 * @param [in]  threshold   threshold to detect zero velocity
 * @retval TRUE if static, otherwise FALSE.
******************************************************************************/
BOOL DetectStaticIMU(real *gyroVar, real *gyroMean, real *accelVar, STATIC_DETECT_SETTING *threshold);


void MotionStatusImu(real *gyro, real *accel, ImuStatsStruct *imuStats, BOOL reset)
{
    static BOOL bIni = false;                       // indicate the routine is initialized or not
    // Buffer to store input IMU data. Data in buffer is ued to calcualte filtered IMU dat.
    static real dAccel[3][FILTER_ORDER];            // a section in memory as buffer to store accel data
    static real dGyro[3][FILTER_ORDER];             // a section in memory as buffer to store gyro data
    static Buffer bfAccel;                          // a ring buffer of accel
    static Buffer bfGyro;                           // a ring buffer of gyro
    // Buffer to store filtered IMU data. Data in buffer is used to calculate IMU stats.
    static real dLpfAccel[3][SAMPLES_FOR_STATS];    // a section in memory as buffer to store accel data
    static real dLpfGyro[3][SAMPLES_FOR_STATS];     // a section in memory as buffer to store gyro data
    static Buffer bfLpfAccel;                       // a ring buffer of accel
    static Buffer bfLpfGyro;                        // a ring buffer of gyro
    // filter coefficients. y/x = b/a
    static real b_AccelFilt[FILTER_ORDER + 1];
    static real a_AccelFilt[FILTER_ORDER + 1];

    // reset the calculation of motion stats
    if (reset)
    {
        bIni = false;
    }

    // initialization
    if (!bIni)
    {
        bIni = true;
        // reset stats
        imuStats->bValid = false;
        imuStats->accelMean[0] = 0.0;
        imuStats->accelMean[1] = 0.0;
        imuStats->accelMean[2] = 0.0;
        imuStats->accelVar[0] = 0.0;
        imuStats->accelVar[1] = 0.0;
        imuStats->accelVar[2] = 0.0;
        imuStats->gyroMean[0] = 0.0;
        imuStats->gyroMean[1] = 0.0;
        imuStats->gyroMean[2] = 0.0;
        imuStats->gyroVar[0] = 0.0;
        imuStats->gyroVar[1] = 0.0;
        imuStats->gyroVar[2] = 0.0;
        // create/reset buffer
        bfNew(&bfGyro, &dGyro[0][0], 3, FILTER_ORDER);
        bfNew(&bfAccel, &dAccel[0][0], 3, FILTER_ORDER);
        bfNew(&bfLpfGyro, &dLpfGyro[0][0], 3, SAMPLES_FOR_STATS);
        bfNew(&bfLpfAccel, &dLpfAccel[0][0], 3, SAMPLES_FOR_STATS);
        // Set the filter coefficients based on selected cutoff frequency and sampling rate
        _PopulateFilterCoefficients(gAlgorithm.linAccelLPFType, gAlgorithm.callingFreq, b_AccelFilt, a_AccelFilt);
    }

    /* Low-pass filter.
     * The input IMU data is put into the buffer, and then filtered.
     */
    LowPassFilter(gyro, &bfGyro, &bfLpfGyro, b_AccelFilt, a_AccelFilt, imuStats->lpfGyro);
    LowPassFilter(accel, &bfAccel, &bfLpfAccel, b_AccelFilt, a_AccelFilt, imuStats->lpfAccel);

    /* Compute accel norm using raw accel data. 
     * The norm will be used to detect static periods via magnitude.
     */
    imuStats->accelNorm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

    /* Compute mean/variance from input data.
     * The latest will be put into the buffer, and stats of data in buffer is then calculated.
     * When the input data buffer is full, the stats can be assumed valid.
     */
    ComputeStats(&bfLpfGyro, imuStats->lpfGyro, imuStats->gyroMean, imuStats->gyroVar);
    ComputeStats(&bfLpfAccel, imuStats->lpfAccel, imuStats->accelMean, imuStats->accelVar);
    imuStats->bValid = bfLpfGyro.full && bfLpfAccel.full;

    // Detect static period using var calculated above.
    imuStats->bStaticIMU = DetectStaticIMU( imuStats->gyroVar,
                                            imuStats->gyroMean,
                                            imuStats->accelVar,
                                            &gAlgorithm.staticDetectParam);
}

static void ComputeStats(Buffer *bf, real *latest, real *mean, real *var)
{
    real lastMean[3];
    real lastVar[3];
    lastMean[0] = mean[0];
    lastMean[1] = mean[1];
    lastMean[2] = mean[2];
    lastVar[0] = var[0];
    lastVar[1] = var[1];
    lastVar[2] = var[2];
    if (bf->full)
    {
        /* when buffer is full, the var and mean are computed from
         * all data in the buffer. From now on, the var and mean
         * should be computed by removing the oldest data and including
         * the latest data.
         */
         // Get the oldest data which will be removed by following bfPut
        real oldest[3];
        bfGet(bf, oldest, bf->num - 1);
        // put this accel into buffer
        bfPut(bf, latest);
        // mean(n+1) = ( mean(n) * n - x(1) + x(n+1) ) / n
        mean[0] += (latest[0] - oldest[0]) / (real)(bf->num);
        mean[1] += (latest[1] - oldest[1]) / (real)(bf->num);
        mean[2] += (latest[2] - oldest[2]) / (real)(bf->num);

        // naive var calculation is adopted because recursive method is numerically instable
        real tmpVar[3];
        tmpVar[0] = vecVar(&(bf->d[0]), mean[0], bf->num);
        tmpVar[1] = vecVar(&(bf->d[bf->n]), mean[1], bf->num);
        tmpVar[2] = vecVar(&(bf->d[2 * bf->n]), mean[2], bf->num);
        // make var estimation smooth
        real k = 0.96f;
        int i;
        for (i = 0; i < 3; i++)
        {
            if (tmpVar[i] >= var[i])
            {
                var[i] = tmpVar[i];
            }
            else
            {
                var[i] = k * var[i] + (1.0f - k)*tmpVar[i];
            }
        }
    }
    else
    {
        // put this accel into buffer
        bfPut(bf, latest);
        /* Recursivly include new accel. The data num used to compute mean and
         * var are increasing.
         */
         // mean(n+1) = mean(n) *n / (n+1) + x(n+1) / (n+1)
        mean[0] = lastMean[0] + (latest[0] - lastMean[0]) / (real)(bf->num);
        mean[1] = lastMean[1] + (latest[1] - lastMean[1]) / (real)(bf->num);
        mean[2] = lastMean[2] + (latest[2] - lastMean[2]) / (real)(bf->num);
        var[0] = lastVar[0] + lastMean[0] * lastMean[0] - mean[0] * mean[0] +
            (latest[0] * latest[0] - lastVar[0] - lastMean[0] * lastMean[0]) / (real)(bf->num);
        var[1] = lastVar[1] + lastMean[1] * lastMean[1] - mean[1] * mean[1] +
            (latest[1] * latest[1] - lastVar[1] - lastMean[1] * lastMean[1]) / (real)(bf->num);
        var[2] = lastVar[2] + lastMean[2] * lastMean[2] - mean[2] * mean[2] +
            (latest[2] * latest[2] - lastVar[2] - lastMean[2] * lastMean[2]) / (real)(bf->num);
    }
}

BOOL DetectStaticIMU(real *gyroVar, real *gyroMean, real *accelVar, STATIC_DETECT_SETTING *threshold)
{
    BOOL bStatic = TRUE;
    int i;
    static float multiplier[3] = { 0.0, 0.0, 0.0 };
    static float gyroVarThreshold = 0.0;
    static float accelVarThreshold = 0.0;
    static float gyroBiasThreshold = 0.0;
    // Update threshold
    if (multiplier[0] != threshold->staticNoiseMultiplier[0])
    {
        multiplier[0] = threshold->staticNoiseMultiplier[0];
        gyroVarThreshold = multiplier[0] * threshold->staticVarGyro;
    }
    if (multiplier[1] != threshold->staticNoiseMultiplier[1])
    {
        multiplier[1] = threshold->staticNoiseMultiplier[1];
        accelVarThreshold = multiplier[1] * threshold->staticVarAccel;
    }
    if (multiplier[2] != threshold->staticNoiseMultiplier[2])
    {
        multiplier[2] = threshold->staticNoiseMultiplier[2];
        gyroBiasThreshold = multiplier[2] * threshold->maxGyroBias;
    }

    for (i = 0; i < 3; i++)
    {
        if (gyroVar[i] > gyroVarThreshold ||
            accelVar[i] > accelVarThreshold ||
            fabs(gyroMean[i]) > gyroBiasThreshold)
        {
            bStatic = FALSE;
            break;
        }
    }

    return bStatic;
}

/* Set the accelerometer filter coefficients, which are used to filter the
 * accelerometer readings prior to determining the setting of the linear-
 * acceleration switch and computing the roll and pitch from accelerometer
 * readings.
 */
static void _PopulateFilterCoefficients(uint8_t lpfType, uint8_t callingFreq, real *b, real *a)
{
    switch (lpfType)
    {
    case NO_LPF:
        b[0] = (real)(1.0);
        b[1] = (real)(0.0);
        b[2] = (real)(0.0);
        b[3] = (real)(0.0);

        a[0] = (real)(0.0);
        a[1] = (real)(0.0);
        a[2] = (real)(0.0);
        a[3] = (real)(0.0);
        break;
    case TWO_HZ_LPF:
        if (callingFreq == FREQ_100_HZ)
        {
            b[0] = (real)(2.19606211225382e-4);
            b[1] = (real)(6.58818633676145e-4);
            b[2] = (real)(6.58818633676145e-4);
            b[3] = (real)(2.19606211225382e-4);

            a[0] = (real)(1.000000000000000);
            a[1] = (real)(-2.748835809214676);
            a[2] = (real)(2.528231219142559);
            a[3] = (real)(-0.777638560238080);
        }
        else
        {
            b[0] = (real)(2.91464944656705e-5);
            b[1] = (real)(8.74394833970116e-5);
            b[2] = (real)(8.74394833970116e-5);
            b[3] = (real)(2.91464944656705e-5);

            a[0] = (real)(1.000000000000000);
            a[1] = (real)(-2.874356892677485);
            a[2] = (real)(2.756483195225695);
            a[3] = (real)(-0.881893130592486);
        }
        break;
    case FIVE_HZ_LPF:
        if (callingFreq == FREQ_100_HZ)
        {
            b[0] = (real)(0.002898194633721);
            b[1] = (real)(0.008694583901164);
            b[2] = (real)(0.008694583901164);
            b[3] = (real)(0.002898194633721);

            a[0] = (real)(1.000000000000000);
            a[1] = (real)(-2.374094743709352);
            a[2] = (real)(1.929355669091215);
            a[3] = (real)(-0.532075368312092);
        }
        else
        {
            b[0] = (real)(0.000416546139076);
            b[1] = (real)(0.001249638417227);
            b[2] = (real)(0.001249638417227);
            b[3] = (real)(0.000416546139076);

            a[0] = (real)(1.000000000000000);
            a[1] = (real)(-2.686157396548143);
            a[2] = (real)(2.419655110966473);
            a[3] = (real)(-0.730165345305723);
        }
        break;
    case TWENTY_HZ_LPF:
        if (callingFreq == FREQ_100_HZ)
        {
            // [B,A] = butter(3,20/(100/2))
            b[0] = (real)(0.098531160923927);
            b[1] = (real)(0.295593482771781);
            b[2] = (real)(0.295593482771781);
            b[3] = (real)(0.098531160923927);

            a[0] = (real)(1.000000000000000);
            a[1] = (real)(-0.577240524806303);
            a[2] = (real)(0.421787048689562);
            a[3] = (real)(-0.056297236491843);
        }
        else
        {
            // [B,A] = butter(3,20/(200/2))
            b[0] = (real)(0.018098933007514);
            b[1] = (real)(0.054296799022543);
            b[2] = (real)(0.054296799022543);
            b[3] = (real)(0.018098933007514);

            a[0] = (real)(1.000000000000000);
            a[1] = (real)(-1.760041880343169);
            a[2] = (real)(1.182893262037831);
            a[3] = (real)(-0.278059917634546);
        }
        break;
    case TWENTY_FIVE_HZ_LPF:
        if (callingFreq == FREQ_100_HZ)
        {
            b[0] = (real)(0.166666666666667);
            b[1] = (real)(0.500000000000000);
            b[2] = (real)(0.500000000000000);
            b[3] = (real)(0.166666666666667);

            a[0] = (real)(1.000000000000000);
            a[1] = (real)(-0.000000000000000);
            a[2] = (real)(0.333333333333333);
            a[3] = (real)(-0.000000000000000);
        }
        else
        {
            b[0] = (real)(0.031689343849711);
            b[1] = (real)(0.095068031549133);
            b[2] = (real)(0.095068031549133);
            b[3] = (real)(0.031689343849711);

            a[0] = (real)(1.000000000000000);
            a[1] = (real)(-1.459029062228061);
            a[2] = (real)(0.910369000290069);
            a[3] = (real)(-0.197825187264319);
        }
        break;
    case TEN_HZ_LPF:
    default:
        if (callingFreq == FREQ_100_HZ)
        {
            b[0] = (real)(0.0180989330075144);
            b[1] = (real)(0.0542967990225433);
            b[2] = (real)(0.0542967990225433);
            b[3] = (real)(0.0180989330075144);

            a[0] = (real)(1.0000000000000000);
            a[1] = (real)(-1.7600418803431690);
            a[2] = (real)(1.1828932620378310);
            a[3] = (real)(-0.2780599176345460);
        }
        else
        {
            b[0] = (real)(0.002898194633721);
            b[1] = (real)(0.008694583901164);
            b[2] = (real)(0.008694583901164);
            b[3] = (real)(0.002898194633721);

            a[0] = (real)(1.000000000000000);
            a[1] = (real)(-2.374094743709352);
            a[2] = (real)(1.929355669091215);
            a[3] = (real)(-0.532075368312092);
        }
        break;
    }
}

static void LowPassFilter(real *in, Buffer *bfIn, Buffer *bfOut, real *b, real *a, real *filtered)
{
    // Fill the buffer with the first input.
    if (!bfIn->full)
    {
        bfPut(bfIn, in);
        filtered[0] = in[0];
        filtered[1] = in[1];
        filtered[2] = in[2];
        return;
    }

    /* Filter accelerometer readings (Note: a[0] =  1.0 and the filter coefficients are symmetric)
     * y = filtered output; x = raw input;
     * a[0]*y(k) + a[1]*y(k-1) + a[2]*y(k-2) + a[3]*y(k-3) =
     * b[0]*x(k) + b[1]*x(k-1) + b[2]*x(k-2) + b[3]*x(k-3) =
     * b[0]*( x(k) + x(k-3) ) + b[1]*( x(k-1) + x(k-2) )
     */
    int i;
    real tmpIn[3];
    real tmpOut[3];
    filtered[0] = b[CURRENT] * in[0];
    filtered[1] = b[CURRENT] * in[1];
    filtered[2] = b[CURRENT] * in[2];
    for (i = PASTx1; i <= PASTx3; i++)
    {
        bfGet(bfIn, tmpIn, i-1);
        bfGet(bfOut, tmpOut, i-1);
        filtered[0] += b[i] * tmpIn[0] - a[i] * tmpOut[0];
        filtered[1] += b[i] * tmpIn[1] - a[i] * tmpOut[1];
        filtered[2] += b[i] * tmpIn[2] - a[i] * tmpOut[2];
    }

    // New data into buffer 
    bfPut(bfIn, in);
}

void EstimateAccelError(real *accel, real *w, real dt, uint32_t staticDelay, ImuStatsStruct *imuStats)
{
    static BOOL bIni = false;               // indicate if the procedure is initialized
    static real lastAccel[3];               // accel input of last step
    static real lastGyro[3];                // gyro input of last step
    static float lastEstimatedAccel[3];     // propagated accel of last step
    static uint32_t counter = 0;            // propagation counter
    static uint32_t t[3];
    // initialize
    if (!bIni)
    {
        bIni = true;
        lastAccel[0] = accel[0];
        lastAccel[1] = accel[1];
        lastAccel[2] = accel[2];
        lastGyro[0] = w[0];
        lastGyro[1] = w[1];
        lastGyro[2] = w[2];
        t[0] = 0;
        t[1] = 0;
        t[2] = 0;
        imuStats->accelErr[0] = 0.0;
        imuStats->accelErr[1] = 0.0;
        imuStats->accelErr[2] = 0.0;
        return;
    }

    /* Using gyro to propagate accel and then to detect accel error can give valid result for a
     * short period of time because the inhere long-term drift of integrating gyro data.
     * So, after this period of time, a new accel input will be selected.
     * Beside, this method cannot detect long-time smooth linear acceleration. In this case, we
     * can only hope the linear acceleration is large enough to make an obvious diffeerence from
     * the Earth gravity 1g.
     */
    if (counter == 0)
    {
        lastEstimatedAccel[0] = lastAccel[0];
        lastEstimatedAccel[1] = lastAccel[1];
        lastEstimatedAccel[2] = lastAccel[2];
    }
    counter++;
    if (counter == staticDelay)
    {
        counter = 0;
    }

    // propagate accel using gyro
    //  a(k) = a(k-1) -w x a(k-1)*dt
    real ae[3];
    lastGyro[0] *= -dt;
    lastGyro[1] *= -dt;
    lastGyro[2] *= -dt;
    cross(lastGyro, lastEstimatedAccel, ae);
    ae[0] += lastEstimatedAccel[0];
    ae[1] += lastEstimatedAccel[1];
    ae[2] += lastEstimatedAccel[2];

    // save this estimated accel
    lastEstimatedAccel[0] = ae[0];
    lastEstimatedAccel[1] = ae[1];
    lastEstimatedAccel[2] = ae[2];

    // err = a(k) - am
    ae[0] -= accel[0];
    ae[1] -= accel[1];
    ae[2] -= accel[2];

    /* If the difference between the propagted accel and the input accel exceeds some threshold,
     * we assume there is linear acceleration and set .accelErr to be a large value (0.1g).
     * If the difference has been within the threshold for a period of time, we start to decrease
     * estimated accel error .accelErr.
     */
    int j;
    imuStats->accelErrLimit = false;
    for (j = 0; j < 3; j++)
    {
        if (fabs(ae[j]) > 0.0980665) // linear accel detected, 0.01g
        {
            t[j] = 0;
            imuStats->accelErr[j] = (real)0.980665;   // 0.1g
        }
        else    // no linear accel detected, start to decrease estimated accel error
        {
            if (t[j] > staticDelay) // decrease error  
            {
                imuStats->accelErr[j] *= 0.9f;
                imuStats->accelErr[j] += 0.1f * ae[j];
            }
            else    // keep previous error value
            {
                t[j]++;
                // imuStats->accelErr[j];
            }
        }
        // limit error, not taking effect here since the max accelErr should be 0.1g
        if (imuStats->accelErr[j] > 5.0)    // 0.5g
        {
            imuStats->accelErr[j] = 5.0;
            imuStats->accelErrLimit = true;
        }
        if (imuStats->accelErr[j] < -5.0)
        {
            imuStats->accelErr[j] = -5.0;
            imuStats->accelErrLimit = true;
        }
    }
    // record accel for next step
    lastAccel[0] = accel[0];
    lastAccel[1] = accel[1];
    lastAccel[2] = accel[2];
    lastGyro[0] = w[0];
    lastGyro[1] = w[1];
    lastGyro[2] = w[2];
}

BOOL DetectMotionFromAccel(real accelNorm, int iReset)
{
    if (iReset)
    {
        gAlgorithm.linAccelSwitch = false;
        gAlgorithm.linAccelSwitchCntr = 0;
    }
    /* Check for times when the acceleration is 'close' to 1 [g].  When this occurs,
     * increment a counter.  When it exceeds a threshold (indicating that the system
     * has been at rest for a given period) then decrease the R-values (in the
     * update stage of the EKF), effectively increasing the Kalman gain.
     */
    if (fabs(1.0 - accelNorm/GRAVITY) < gAlgorithm.Limit.accelSwitch)
    {
        gAlgorithm.linAccelSwitchCntr++;
        if (gAlgorithm.linAccelSwitchCntr >= gAlgorithm.Limit.linAccelSwitchDelay)
        {
            gAlgorithm.linAccelSwitch = TRUE;
        }
        else
        {
            gAlgorithm.linAccelSwitch = FALSE;
        }
    }
    else
    {
        gAlgorithm.linAccelSwitchCntr = 0;
        gAlgorithm.linAccelSwitch = FALSE;
    }

    return TRUE;
}

BOOL DetectStaticGnssVelocity(double *vNED, real threshold, BOOL gnssValid)
{
    static uint8_t cntr = 0;
    if (gnssValid)
    {
        if (fabs(vNED[0]) < threshold && fabs(vNED[1]) < threshold && fabs(vNED[2]) < threshold)
        {
            if (cntr < 3)
            {
                cntr++;
            }
        }
        else
        {
            cntr = 0;
        }
    }
    else
    {
        cntr = 0;
    }

    return cntr >= 3;
}

BOOL DetectStaticOdo(real odo)
{
    return FALSE;
}