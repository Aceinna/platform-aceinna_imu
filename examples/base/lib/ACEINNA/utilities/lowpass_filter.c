/** ***************************************************************************
 * @file   lowpass_filter.c
 * @Author
 * @date   March, 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * generic accelerometer interface, it should be implemented
 * by whichever accelerometer is in use
 *****************************************************************************/
#include <stdint.h>

#include "xbowsp_algorithm.h"
#include "lowpass_filter.h"
#include "ucb_packet.h" // for UcbGetSysType(); and AHRS_SYS

#define WAIT_TIL_VALID 40

// The following array contains the coefficients for a 50 Hz, 3rd-order,
//   low-pass Butterworth filter.  The first column are specific to AHRS, VG,
//   and INS units that sample the accelerometer at 400 Hz (setting of the
//   accelerometer).  The second column contains the coefficients needed to
//   filter the readings when the acceleromter provides data to 800 Hz.
static const int64_t a_q27[4][2][7] = { { { 134217728,  134217728,  134217728,  134217728,
                                            134217728,  134217728,  134217728 },
                                          { 134217728,  134217728,  134217728,  134217728,
                                            134217728,  134217728,  134217728 } },
                                        { {-394220382, -381575702, -360529943, -318645603,
                                           -297851770, -236228822, -195827566 },
                                          {-398436653, -392112425, -381575702, -360529943,
                                           -350028195, -318645603, -297851770 } },
                                        { { 386050414,  362120843,  324760612,  258953734,
                                            230199218,  158765246,  122187659 },
                                          { 394286094,  381981514,  362120843,  324760612,
                                            307223563,  258953734,  230199218 } },
                                        { {-126043726, -114702664,  -98001134,  -71413947,
                                            -60873905,  -37320570,  -26551647 },
                                          {-130066657, -124078999, -114702664,  -98001134,
                                            -90570376,  -71413947,  -60873905 } } };

static const int64_t b_q27[4][2][7] = { { { 504,  7526,  55908,  388989,  711409, 2429198,  4253272 },
                                          {  64,   977,   7526,   55908,  105340,  388989,   711409 } },
                                        { {1513, 22577, 167724, 1166967, 2134227, 7287593, 12759815 },
                                          { 192,  2932,  22577,  167724,  316020, 1166967,  2134227 } },
                                        { {1513, 22577, 167724, 1166967, 2134227, 7287593, 12759815 },
                                          { 192,  2932,  22577,  167724,  316020, 1166967,  2134227 } },
                                        { { 504,  7526,  55908,  388989,  711409, 2429198,  4253272 },
                                          {  64,   977,   7526,   55908,  105340,  388989,   711409 } } };

#define  ONE_HALF_Q27  67108864

/** ****************************************************************************
 * @name _butterWorth3rdLowPass
 * @brief Butterworth 3rd order low pass filter
 *
 * Trace:
 *
 * @param [in] in input value
 * @param [out] out output value
 * @retval  TRUE: the filter reached steady state, output valid
 *          FALSE the filter has not reached steady state, input copied to output
 ******************************************************************************/
uint8_t _accelFilt_3rdOrderBWF_LowPass_Axis(uint8_t axis, int16_t in, int32_t *out, uint8_t freq, uint8_t dataRate)
{
    static uint8_t initFilt[3] = {1,1,1};

    static uint8_t accelCalledCount = 0;
    static int64_t accel_x_q27[4][NUM_AXIS] = {0};
    static int64_t accel_y_q27[4][NUM_AXIS] = {0};

    // Initialize the vector of previous readings
    if( initFilt[axis] ) {
        initFilt[axis] = 0;

        accel_x_q27[1][axis] = (int64_t)in;
        accel_x_q27[2][axis] = (int64_t)in;
        accel_x_q27[3][axis] = (int64_t)in;

        accel_y_q27[1][axis] = (int64_t)in;
        accel_y_q27[2][axis] = (int64_t)in;
        accel_y_q27[3][axis] = (int64_t)in;
    }

    // Filter the input signal
    int64_t tmp_out = a_q27[3][dataRate][freq] * accel_y_q27[3][axis] +
                      a_q27[2][dataRate][freq] * accel_y_q27[2][axis] +
                      a_q27[1][dataRate][freq] * accel_y_q27[1][axis];
    int64_t tmp_in  = b_q27[0][dataRate][freq] * ( (int64_t)in + accel_x_q27[3][axis] +
                                                   3*( accel_x_q27[1][axis] +
                                                       accel_x_q27[2][axis] ) );
    // add 0.5 to the data to round
    accel_y_q27[0][axis]  = ( tmp_in - tmp_out + ONE_HALF_Q27 ) >> 27;
    *out = (int32_t) accel_y_q27[0][axis];

    // Save off past values
    accel_x_q27[3][axis] = accel_x_q27[2][axis];   accel_y_q27[3][axis] = accel_y_q27[2][axis];
    accel_x_q27[2][axis] = accel_x_q27[1][axis];   accel_y_q27[2][axis] = accel_y_q27[1][axis];
    accel_x_q27[1][axis] = (int64_t)in;            accel_y_q27[1][axis] = *out;

    // Allow WAIT_TIL_VALID data points to pass through unfiltered before
    //   passing out filtered data (meant to be part of the initialization
    //   routine)
    if (accelCalledCount > WAIT_TIL_VALID) {
        return 1;
    } else {
        accelCalledCount++;
        *out = in;
        return 0;
    }
}


uint8_t _rateFilt_3rdOrderBWF_LowPass_Axis(uint8_t axis, int16_t in, int32_t *out, uint8_t freq, uint8_t dataRate)
{
    static uint8_t initFilt[3] = {1,1,1};

    static uint8_t rateCalledCount = 0;
    static int64_t rate_x_q27[4][NUM_AXIS] = {0};
    static int64_t rate_y_q27[4][NUM_AXIS] = {0};

    if( initFilt[axis] ) {
        initFilt[axis] = 0;

        rate_x_q27[1][axis] = (int64_t)in;
        rate_x_q27[2][axis] = (int64_t)in;
        rate_x_q27[3][axis] = (int64_t)in;

        rate_y_q27[1][axis] = (int64_t)in;
        rate_y_q27[2][axis] = (int64_t)in;
        rate_y_q27[3][axis] = (int64_t)in;
    }

    // Filter the input signal
    int64_t tmp_out = a_q27[3][dataRate][freq] * rate_y_q27[3][axis] +
                      a_q27[2][dataRate][freq] * rate_y_q27[2][axis] +
                      a_q27[1][dataRate][freq] * rate_y_q27[1][axis];
    int64_t tmp_in  = b_q27[0][dataRate][freq] * ( (int64_t)in + rate_x_q27[3][axis] +
                                                   3*( rate_x_q27[1][axis] +
                                                       rate_x_q27[2][axis] ) );
    rate_y_q27[0][axis]  = ( tmp_in - tmp_out + ONE_HALF_Q27 ) >> 27;

    *out = (int32_t) rate_y_q27[0][axis];

    // Save off past values
    rate_x_q27[3][axis] = rate_x_q27[2][axis];   rate_y_q27[3][axis] = rate_y_q27[2][axis];
    rate_x_q27[2][axis] = rate_x_q27[1][axis];   rate_y_q27[2][axis] = rate_y_q27[1][axis];
    rate_x_q27[1][axis] = (int64_t)in;           rate_y_q27[1][axis] = *out;

    // Allow WAIT_TIL_VALID data points to pass through unfiltered before
    //   passing out filtered data (meant to be part of the initialization
    //   routine)
    if (rateCalledCount > WAIT_TIL_VALID) {
        return 1;
    } else {
        rateCalledCount++;
        *out = in;
        return 0;
    }
}


#define  CURR    0
#define  PASTx1  1
#define  PASTx2  2

static const int64_t ac_q27[3] = {134217728, -178407861,  67562416};
static const int64_t bc_q27[3] = {  5843071,   11686141,   5843071};

uint8_t _rateFilt_4thOrderBWF_LowPass_Axis_cascaded2nd(uint8_t axis, int16_t in, int32_t *out, uint8_t freq, uint8_t dataRate)
{
    static uint8_t initFilt[3] = {1,1,1};

    static uint8_t rateCalledCount = 0;
    static int64_t x_q27[3][NUM_AXIS] = {0};
    static int64_t v_q27[3][NUM_AXIS] = {0};
    static int64_t w_q27[3][NUM_AXIS] = {0};

    int64_t tmp_in, tmp_out;

    if( initFilt[axis] ) {
        initFilt[axis] = 0;

        x_q27[CURR][axis]   = (int64_t)in;
        x_q27[PASTx1][axis] = (int64_t)in;
        x_q27[PASTx2][axis] = (int64_t)in;

        v_q27[CURR][axis]   = (int64_t)in;
        v_q27[PASTx1][axis] = (int64_t)in;
        v_q27[PASTx2][axis] = (int64_t)in;

        w_q27[CURR][axis]   = (int64_t)in;
        w_q27[PASTx1][axis] = (int64_t)in;
        w_q27[PASTx2][axis] = (int64_t)in;
    }

    // Filter the input signal (first stage)
    tmp_in  = bc_q27[CURR] * ( (int64_t)in +
                               2*x_q27[PASTx1][axis] +
                                 x_q27[PASTx2][axis] );
    tmp_out = ac_q27[PASTx1] * v_q27[PASTx1][axis] +
              ac_q27[PASTx2] * v_q27[PASTx2][axis];

    v_q27[CURR][axis]  = ( tmp_in - tmp_out + ONE_HALF_Q27 ) >> 27;

    // Filter the input signal (second stage)
    tmp_in  = bc_q27[CURR] * (   v_q27[CURR][axis] +
                               2*v_q27[PASTx1][axis] +
                                 v_q27[PASTx2][axis] );
    tmp_out = ac_q27[PASTx1] * w_q27[PASTx1][axis] +
              ac_q27[PASTx2] * w_q27[PASTx2][axis];
    w_q27[0][axis]  = ( tmp_in - tmp_out + ONE_HALF_Q27 ) >> 27;

    *out = (int32_t) w_q27[0][axis];

    // Save off past values
    x_q27[PASTx2][axis] = x_q27[PASTx1][axis];
    x_q27[PASTx1][axis] = (int64_t)in;

    v_q27[PASTx2][axis] = v_q27[PASTx1][axis];
    v_q27[PASTx1][axis] = v_q27[CURR][axis];

    w_q27[PASTx2][axis] = w_q27[PASTx1][axis];
    w_q27[PASTx1][axis] = w_q27[CURR][axis];

    // Allow WAIT_TIL_VALID data points to pass through unfiltered before
    //   passing out filtered data (meant to be part of the initialization
    //   routine)
    if (rateCalledCount > WAIT_TIL_VALID) {
        return 1;
    } else {
        rateCalledCount++;
        *out = in;
        return 0;
    }
}

uint8_t _accel_4thOrderBWF_LowPass_Axis_cascaded2nd(uint8_t axis, int16_t in, int32_t *out, uint8_t freq, uint8_t dataRate)
{
    static uint8_t initFilt[3] = {1,1,1};

    static uint8_t rateCalledCount = 0;
    static int64_t x_q27[3][NUM_AXIS] = {0};
    static int64_t v_q27[3][NUM_AXIS] = {0};
    static int64_t w_q27[3][NUM_AXIS] = {0};

    int64_t tmp_in, tmp_out;

    if( initFilt[axis] ) {
        initFilt[axis] = 0;

        x_q27[CURR][axis]   = (int64_t)in;
        x_q27[PASTx1][axis] = (int64_t)in;
        x_q27[PASTx2][axis] = (int64_t)in;

        v_q27[CURR][axis]   = (int64_t)in;
        v_q27[PASTx1][axis] = (int64_t)in;
        v_q27[PASTx2][axis] = (int64_t)in;

        w_q27[CURR][axis]   = (int64_t)in;
        w_q27[PASTx1][axis] = (int64_t)in;
        w_q27[PASTx2][axis] = (int64_t)in;
    }

    // Filter the input signal (first stage)
    tmp_in  = bc_q27[CURR] * ( (int64_t)in +
                               2*x_q27[PASTx1][axis] +
                                 x_q27[PASTx2][axis] );
    tmp_out = ac_q27[PASTx1] * v_q27[PASTx1][axis] +
              ac_q27[PASTx2] * v_q27[PASTx2][axis];

    v_q27[CURR][axis]  = ( tmp_in - tmp_out + ONE_HALF_Q27 ) >> 27;

    // Filter the input signal (second stage)
    tmp_in  = bc_q27[CURR] * (   v_q27[CURR][axis] +
                               2*v_q27[PASTx1][axis] +
                                 v_q27[PASTx2][axis] );
    tmp_out = ac_q27[PASTx1] * w_q27[PASTx1][axis] +
              ac_q27[PASTx2] * w_q27[PASTx2][axis];
    w_q27[0][axis]  = ( tmp_in - tmp_out + ONE_HALF_Q27 ) >> 27;

    *out = (int32_t) w_q27[0][axis];

    // Save off past values
    x_q27[PASTx2][axis] = x_q27[PASTx1][axis];
    x_q27[PASTx1][axis] = (int64_t)in;

    v_q27[PASTx2][axis] = v_q27[PASTx1][axis];
    v_q27[PASTx1][axis] = v_q27[CURR][axis];

    w_q27[PASTx2][axis] = w_q27[PASTx1][axis];
    w_q27[PASTx1][axis] = w_q27[CURR][axis];

    // Allow WAIT_TIL_VALID data points to pass through unfiltered before
    //   passing out filtered data (meant to be part of the initialization
    //   routine)
    if (rateCalledCount > WAIT_TIL_VALID) {
        return 1;
    } else {
        rateCalledCount++;
        *out = in;
        return 0;
    }
}



static const int64_t ac1_q27[2] = {134217728, -58883939};
static const int64_t bc1_q27[2] = { 37666894,  37666894};

uint8_t _rateFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(uint8_t axis, int16_t in, int32_t *out, uint8_t freq, uint8_t dataRate)
{
    static uint8_t initFilt[3] = {1,1,1};

    static uint8_t rateCalledCount = 0;
    static int64_t x_q27[2][NUM_AXIS] = {0};
    static int64_t u_q27[2][NUM_AXIS] = {0};
    static int64_t v_q27[2][NUM_AXIS] = {0};
    static int64_t w_q27[2][NUM_AXIS] = {0};

    int64_t tmp_in, tmp_out;

    if( initFilt[axis] ) {
        initFilt[axis] = 0;

        x_q27[CURR][axis]   = (int64_t)in;
        x_q27[PASTx1][axis] = (int64_t)in;

        u_q27[CURR][axis]   = (int64_t)in;
        u_q27[PASTx1][axis] = (int64_t)in;

        v_q27[CURR][axis]   = (int64_t)in;
        v_q27[PASTx1][axis] = (int64_t)in;

        w_q27[CURR][axis]   = (int64_t)in;
        w_q27[PASTx1][axis] = (int64_t)in;
    }

    // Filter the input signal (first stage)
    tmp_in  = bc1_q27[CURR] * ( (int64_t)in +
                                x_q27[PASTx1][axis] );
    tmp_out = ac1_q27[PASTx1] * u_q27[PASTx1][axis];

    u_q27[CURR][axis]  = ( tmp_in - tmp_out + ONE_HALF_Q27 ) >> 27;

    // Second stage
    tmp_in  = bc1_q27[CURR] * ( u_q27[CURR][axis] +
                                u_q27[PASTx1][axis] );
    tmp_out = ac1_q27[PASTx1] * v_q27[PASTx1][axis];

    v_q27[CURR][axis]  = ( tmp_in - tmp_out + ONE_HALF_Q27 ) >> 27;

    // Third stage
    tmp_in  = bc1_q27[CURR] * ( v_q27[CURR][axis] +
                                v_q27[PASTx1][axis] );
    tmp_out = ac1_q27[PASTx1] * w_q27[PASTx1][axis];

    w_q27[CURR][axis]  = ( tmp_in - tmp_out + ONE_HALF_Q27 ) >> 27;

    *out = (int32_t) w_q27[CURR][axis];

    // Save off past values
    x_q27[PASTx1][axis] = (int64_t)in;
    u_q27[PASTx1][axis] = u_q27[CURR][axis];
    v_q27[PASTx1][axis] = v_q27[CURR][axis];
    w_q27[PASTx1][axis] = w_q27[CURR][axis];

    // Allow WAIT_TIL_VALID data points to pass through unfiltered before
    //   passing out filtered data (meant to be part of the initialization
    //   routine)
    if (rateCalledCount > WAIT_TIL_VALID) {
        return 1;
    } else {
        rateCalledCount++;
        *out = in;
        return 0;
    }
}


uint8_t _accelFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(uint8_t axis, int16_t in, int32_t *out, uint8_t freq, uint8_t dataRate)
{
    static uint8_t initFilt[3] = {1,1,1};

    static uint8_t rateCalledCount = 0;
    static int64_t x_q27[2][NUM_AXIS] = {0};
    static int64_t u_q27[2][NUM_AXIS] = {0};
    static int64_t v_q27[2][NUM_AXIS] = {0};
    static int64_t w_q27[2][NUM_AXIS] = {0};

    int64_t tmp_in, tmp_out;

    if( initFilt[axis] ) {
        initFilt[axis] = 0;

        x_q27[CURR][axis]   = (int64_t)in;
        x_q27[PASTx1][axis] = (int64_t)in;

        u_q27[CURR][axis]   = (int64_t)in;
        u_q27[PASTx1][axis] = (int64_t)in;

        v_q27[CURR][axis]   = (int64_t)in;
        v_q27[PASTx1][axis] = (int64_t)in;

        w_q27[CURR][axis]   = (int64_t)in;
        w_q27[PASTx1][axis] = (int64_t)in;
    }

    // Filter the input signal (first stage)
    tmp_in  = bc1_q27[CURR] * ( (int64_t)in +
                                x_q27[PASTx1][axis] );
    tmp_out = ac1_q27[PASTx1] * u_q27[PASTx1][axis];

    u_q27[CURR][axis]  = ( tmp_in - tmp_out + ONE_HALF_Q27 ) >> 27;

    // Second stage
    tmp_in  = bc1_q27[CURR] * ( u_q27[CURR][axis] +
                                u_q27[PASTx1][axis] );
    tmp_out = ac1_q27[PASTx1] * v_q27[PASTx1][axis];

    v_q27[CURR][axis]  = ( tmp_in - tmp_out + ONE_HALF_Q27 ) >> 27;

    // Third stage
    tmp_in  = bc1_q27[CURR] * ( v_q27[CURR][axis] +
                                v_q27[PASTx1][axis] );
    tmp_out = ac1_q27[PASTx1] * w_q27[PASTx1][axis];

    w_q27[CURR][axis]  = ( tmp_in - tmp_out + ONE_HALF_Q27 ) >> 27;

    *out = (int32_t) w_q27[CURR][axis];

    // Save off past values
    x_q27[PASTx1][axis] = (int64_t)in;
    u_q27[PASTx1][axis] = u_q27[CURR][axis];
    v_q27[PASTx1][axis] = v_q27[CURR][axis];
    w_q27[PASTx1][axis] = w_q27[CURR][axis];

    // Allow WAIT_TIL_VALID data points to pass through unfiltered before
    //   passing out filtered data (meant to be part of the initialization
    //   routine)
    if (rateCalledCount > WAIT_TIL_VALID) {
        return 1;
    } else {
        rateCalledCount++;
        *out = in;
        return 0;
    }
}



