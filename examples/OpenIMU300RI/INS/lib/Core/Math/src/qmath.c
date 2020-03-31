/** ***************************************************************************
 * @file qmath.c Fixed Point Math Library
 * @Author  Alex Forencich alex@alexelectronics.com
 *          Jordan Rhee rhee.jordan@gmail.com
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @brief IEEE UCSD
 * http://ieee.ucsd.edu
 *****************************************************************************/
#include "qmath.h"
#include "math.h"


#ifdef DISPLAY_DIAGNOSTIC_MSG
#include "debug.h"
#endif

#include "Indices.h"

#define Q27 134217728
#define Q54 18014398509481984

/**
 * square root
 */
fixedp qsqrt( fixedp p_Square )
{
  // fixedp = long
    fixedp   res;
    fixedp   delta;
    fixedp   maxError;

    if ( p_Square <= 0 )
        return 0;

    /* start at half */
    res = (p_Square >> 1);

    /* determine allowable error */
    maxError =  qmul( p_Square, FIXED_SQRT_ERR );

//#define qmul(x, y) _qmul(x, y, FIXED_FRACBITS, FIXED_FRACBITS, FIXED_FRACBITS)

    do {
        delta =  ( qmul( res, res ) - p_Square );
        res -=  qdiv( delta, ( res << 1 ) );
    } while ( delta > maxError || delta < -maxError );

    return res;
}


// Range of input: +/- 16
int32_t qsqrt_q27( int32_t pSquare_q27 )
{
  // fixedp = long
    int32_t res;
    int32_t delta;
    int32_t maxError;

    // Check for negative number and return 0 (error checking: return -1???)
    if( pSquare_q27 <= 0 ) {
        return 0;
    }

    /* start at half */
    res = ( pSquare_q27 >> 1 );

    /* determine allowable error */
    // this limits the error to be less than 1e-6
    maxError = 135;

    uint8_t loopCntr = 0;
    do {
        loopCntr++;
        delta =  ( _qmul( res, res, 27, 27, 27 ) - pSquare_q27 );
        res = res - _qdiv( delta, ( res << 1 ), 27, 27, 27 );
        if( loopCntr >= 250 ) {
//            DEBUG_STRING( " SQRT27 Err " );
            break;
        }
    } while ( delta > maxError || delta < -maxError );

    // Do one more iteration for the heck of it
    delta =  ( _qmul( res, res, 27, 27, 27 ) - pSquare_q27 );
    res = res - _qdiv( delta, ( res << 1 ), 27, 27, 27 );
    return res;
}


// Range of input: +/- 2
int32_t qsqrt_q30( int32_t pSquare_q30 )
{
  // fixedp = long
    int32_t res;
    int32_t delta;
    int32_t maxError;

    // Check for negative number and return 0 (error checking: return -1???)
    if( pSquare_q30 <= 0 ) {
        return 0;
    }

    /* start at one (2^30) */
    res = 536870912; //pSquare_q30 >> 1;

    /* determine allowable error */
    // this limits the error to be less than 1e-6
    //maxError = 135;
    maxError = 1080;

    uint8_t loopCntr = 0;
    do {
        loopCntr++;

        delta =  ( _qmul( res, res, 30, 30, 30 ) - pSquare_q30 );
        res = res - _qdiv( delta, ( res << 1 ), 30, 30, 30 );

        if( loopCntr == 250  ) {
 //           DEBUG_ENDLINE();
 //           DEBUG_INT( " QSQRT_Q30 stopped at iteration ", loopCntr );
 //           DEBUG_LONGINT( " (function argument: ", pSquare_q30 );
 //           DEBUG_STRING( ")" );
 //           DEBUG_ENDLINE();
            break;
        }
    } while ( ( delta > maxError ) || ( delta < -maxError ) );

//    for( int loopCntr = 0; loopCntr < 8; loopCntr++ ) {
//        delta =  ( _qmul( res, res, 30, 30, 30 ) - pSquare_q30 );
//        res = res - _qdiv( delta, ( res << 1 ), 30, 30, 30 );
//    }

    // Do one more iteration for the heck of it
    delta =  ( _qmul( res, res, 30, 30, 30 ) - pSquare_q30 );
    res = res - _qdiv( delta, ( res << 1 ), 30, 30, 30 );
    return res;
}


// Range of input: +/- 2
int32_t qsqrt_q29( int32_t pSquare_q29 )
{
  // fixedp = long
    int32_t res;
    int32_t delta;
    int32_t maxError;

    // Check for negative number and return 0 (error checking: return -1???)
    if( pSquare_q29 <= 0 ) {
        return 0;
    }

    /* start at one (2^30) */
    res = pSquare_q29 >> 1;

    /* determine allowable error */
    // this limits the error to be less than 1e-6
    //maxError = 135;
    maxError = 1080;

    uint8_t loopCntr = 0;
    do {
        loopCntr++;

        delta =  ( _qmul( res, res, 29, 29, 29 ) - pSquare_q29 );
        res = res - _qdiv( delta, ( res << 1 ), 29, 29, 29 );

        if( loopCntr == 250  ) {
//            DEBUG_ENDLINE();
//            DEBUG_INT( " QSQRT_Q29 stopped at iteration ", loopCntr );
//            DEBUG_LONGINT( " (function argument: ", pSquare_q29 );
//            DEBUG_STRING( ")" );
//            DEBUG_ENDLINE();
            break;
        }
    } while ( ( delta > maxError ) || ( delta < -maxError ) );

//    for( int loopCntr = 0; loopCntr < 8; loopCntr++ ) {
//        delta =  ( _qmul( res, res, 30, 30, 30 ) - pSquare_q30 );
//        res = res - _qdiv( delta, ( res << 1 ), 30, 30, 30 );
//    }

    // Do one more iteration for the heck of it
    delta =  ( _qmul( res, res, 29, 29, 29 ) - pSquare_q29 );
    res = res - _qdiv( delta, ( res << 1 ), 29, 29, 29 );
    return res;
}


//
int32_t qsqrt_q23( int32_t pSquare_q23 )
{
  // fixedp = long
    int32_t res;
    int32_t delta;
    int32_t maxError;

    if( pSquare_q23 <= 0 )
        return 0;

    /* start at half */
    res = ( pSquare_q23 >> 1 );

    /* determine allowable error */
    // this limits the error to be less than 1e-6
    maxError = 10;

    do {
        delta =  ( _qmul( res, res, 23, 23, 23 ) - pSquare_q23 );
        res = res - _qdiv( delta, ( res << 1 ), 23, 23, 23 );
    } while ( delta > maxError || delta < -maxError );

    // Do one more iteration for the heck of it
    delta =  ( _qmul( res, res, 23, 23, 23 ) - pSquare_q23 );
    res = res - _qdiv( delta, ( res << 1 ), 23, 23, 23 );
    return res;
}



/**
 * log (base e)
 */
fixedp qlog( fixedp p_Base )
{
    fixedp w = 0;
        fixedp y = 0;
        fixedp z = 0;
        fixedp num = int2q(1);
        fixedp dec = 0;

        if ( p_Base == int2q(1) )
                return 0;

        if ( p_Base == 0 )
                return 0xffffffff;

        for ( dec=0 ; qabs( p_Base ) >= int2q(2) ; dec += int2q(1) )
                p_Base = qdiv(p_Base, QLN_E);

        p_Base -= int2q(1);
        z = p_Base;
        y = p_Base;
        w = int2q(1);

        while ( y != y + w )
        {
                z = 0 - qmul( z , p_Base );
                num += int2q(1);
                w = qdiv( z , num );
                y += w;
        }

        return y + dec;
}

/**
 * log base 10
 */
fixedp qlog10( fixedp p_Base )
{
    // ln(p_Base)/ln(10)
    // more accurately, ln(p_Base) * (1/ln(10))
    return qmul(qlog(p_Base), Q1OLN_10);
}

/**
 * exp (e to the x)
 */
fixedp qexp(fixedp p_Base)
{
        fixedp w;
        fixedp y;
        fixedp num;

        for ( w=int2q(1), y=int2q(1), num=int2q(1) ; y != y+w ; num += int2q(1) )
        {
                w = qmul(w, qdiv(p_Base, num));
                y += w;
        }

        return y;
}

/**
 * pow
 */
fixedp qpow( fixedp p_Base,
             fixedp p_Power )
{
        if ( p_Base < 0 && qmod(p_Power, int2q(2)) != 0 )
                return - qexp( qmul(p_Power, qlog( -p_Base )) );
        else
                return qexp( qmul(p_Power, qlog(qabs( p_Base ))) );
}

/**
 * sinx, internal sine function
 */
static fixedp sinx(fixedp x)
{
        fixedp xpwr;
        long xftl;
        fixedp xresult;
        short positive;
        int i;

        xresult = 0;
        xpwr = x;
        xftl = 1;
        positive = -1;

        // Note: 12! largest for long
        for (i = 1; i < 7; i+=2)
        {
                if ( positive )
                        xresult += qdiv(xpwr, long2q(xftl));
                else
                        xresult -= qdiv(xpwr, long2q(xftl));

                xpwr = qmul(x, xpwr);
                xpwr = qmul(x, xpwr);
                xftl *= i+1;
                xftl *= i+2;
                positive = !positive;
        }

        return xresult;
}

/**
 * sine
 */
fixedp qsin(fixedp theta)
{
        fixedp xresult;
        short bBottom = 0;
        //static fixed xPI = XPI;
        //static fixed x2PI = X2PI;
        //static fixed xPIO2 = XPIO2;

        fixedp x = qmod(theta, Q2PI);
        if ( x < 0 )
                x += Q2PI;

        if ( x > QPI )
        {
                bBottom = -1;
                x -= QPI;
        }

        if ( x <= QPIO2 )
                xresult = sinx(x);
        else
                xresult = sinx(QPIO2-(x-QPIO2));

        if ( bBottom )
                return -xresult;

        return xresult;
}

/**
 * cosine
 */
fixedp qcos(fixedp theta)
{
        return qsin(theta + QPIO2);
}

/**
 * tangent
 */
fixedp qtan(fixedp theta)
{
        return qdiv(qsin(theta), qcos(theta));
}

/**
 * q2a - converts a fixed point number to a string
 */
char *q2a(char   *buf,
          fixedp n)
{
    long ipart = qipart(n);
    long fpart = qfpart(n);
    long intdigits = 0;

    int i = 0;
    int j = 0;
    int d = 0;

    int offset = 0;

    long v;
    long t;
    long st = 0;

    if (n != 0)
    {
            intdigits = qipart(qceil(qlog10(qabs(n))));
    }

    if (intdigits < 1) intdigits = 1;

    offset = intdigits - 1;

    if (n < 0)
    {
            buf[0] = '-';
            offset++;
            n = -n;
            ipart = -ipart;
            fpart = -fpart;
    }

    // integer portion
    for (i = 0; i < intdigits; i++)
    {
            j = offset - i;
            d = ipart % 10;
            ipart = ipart / 10;
            buf[j] = '0' + d;
    }

    // decimal point
    buf[offset + 1] = '.';

    // fractional portion
    v = 5;
    t = FIXED_RESOLUTION >> 1;

    for (i = 0; i < FIXED_DECIMALDIGITS - 1; i++)
    {
            v = (v << 1) + (v << 3);
    }

    for (i = 0; i < FIXED_FRACBITS; i++)
    {
            if (fpart & t)
            {
                    st += v;
            }
            v = v >> 1;
            t = t >> 1;
    }

    offset += FIXED_DECIMALDIGITS + 1;

    for (i = 0; i < FIXED_DECIMALDIGITS; i++)
    {
            j = offset - i;
            d = st % 10;
            st = st / 10;
            buf[j] = '0' + d;
    }

    buf[offset + 1] = '\0';    // ending zero

    return buf;
}


//
int32_t nabs( int32_t signedInteger )
{
    if( signedInteger >= 0 ) {
        return signedInteger;
    } else {
        return -signedInteger;
    }
}


//
int32_t nabs_32( int32_t signedInteger )
{
    if( signedInteger >= 0 ) {
        return signedInteger;
    } else {
        return -signedInteger;
    }
}

//
int64_t nabs_64( int64_t signedInteger )
{
    if( signedInteger >= 0 ) {
        return signedInteger;
    } else {
        return -signedInteger;
    }
}

//
int32_t sign( int32_t signedInteger )
{
    if( signedInteger >= 0 ) {
        return 1;
    } else {
        return -1;
    }
}


// asin function based on sqrt and atan2
//   asin( x ) returns an angle (between -pi and +pi) for an input between -1 and +1
//   however, x is represented as a q number (i.e. q27).  Tested.  Result is within ~0.2 degrees
//   (the precision of the atan2 function).  FIXME: change atan2 to increase precision.
int32_t asin_q27( int32_t x_q27 )
{
    int32_t tmp1_q27;

    tmp1_q27 = _qmul( x_q27, x_q27, 27, 27, 27 );
    tmp1_q27 = Q27 - tmp1_q27;  // 1 - x^2  (x < 1)
    tmp1_q27 = qsqrt_q27( tmp1_q27 );

    return atan2_q27( x_q27, tmp1_q27 );
}


// asin function based on sqrt and atan2
//   asin( x ) returns an angle (between -pi and +pi) for an input between -1 and +1
//   however, x is represented as a q number (i.e. q27).  Tested.  Result is within ~0.2 degrees
//   (the precision of the atan2 function).  FIXME: change atan2 to increase precision.
int32_t asin_q29( int32_t x_q30 )
{
    int32_t tmp1_q30;

    tmp1_q30 = _qmul( x_q30, x_q30, 30, 30, 30 );
    tmp1_q30 = ONE_Q30 - tmp1_q30;  // 1 - x^2  (x <= 1)
//DEBUG_STRING( " A" );
//    tmp1_q30 = qsqrt_q30( tmp1_q30 );
    tmp1_q30 = qsqrt_q27( tmp1_q30 >> 3) << 3;

    return atan2_q29( x_q30, tmp1_q30, 30 );
}

//asin(x) = atan2 (x, sqrt ((1.0 + x) * (1.0 - x)))
//acos(x) = atan2 (sqrt ((1.0 + x) * (1.0 - x)), x)

//[ 1072959488, 345219611, 157684912, 42200635 ]

typedef struct atan2_coeff {
    int32_t a0_q27;
    int32_t a1_q27;
    int32_t a2_q27;
    int32_t a3_q27;

    int32_t a0_q29;
    int32_t a1_q29;
    int32_t a2_q29;
    int32_t a3_q29;

    int32_t a0_q30;
    int32_t a1_q30;
    int32_t a2_q30;
    int32_t a3_q30;
} atan2_coeff;


//
// 32-bit fixed point (Q27) four-quadrant arctangent. Given a Cartesian vector (x, y), return the
//   angle subtended by the vector and the positive x-axis.  Both function input and output are in
//   q27 format.  Accuracy is greater than 5 mdeg (18 arcsec).
//
// The value returned falls in the follwing range: [ -pi, pi ) and is converted to a q27 data-type
//   (floating-point value is multiplied by 2^27).
//
// * Because the magnitude of the input vector does not change the angle it
// * represents, the inputs can be in any signed 16-bit fixed-point format.
// *
// * @param y y-coordinate is int32_t, q27 format
// * @param x x-coordinate is int32_t, q27 format
// * @return angle in int32_t, q27 format between -( pi*2^27 ) and +( pi*2^27 )
//
// atan( y, x ) returns an angle in the range [ -pi, +pi ).  A Q29 representation will encompass
//   the range of output values.  The input must support a wider range of values
int32_t atan2_q27( int32_t y_q27, int32_t x_q27 )
{
    static atan2_coeff coeff;
    coeff.a0_q27 = 134119936;
    coeff.a1_q27 =  43152451;
    coeff.a2_q27 =  19710614;
    coeff.a3_q27 =   5275079;

    coeff.a0_q29 = 536479744;
    coeff.a1_q29 = 172609805;
    coeff.a2_q29 =  78842456;
    coeff.a3_q29 =  21100318;

    int32_t nabs_y = nabs( y_q27 );
    int32_t nabs_x = nabs( x_q27 );

    int32_t angle;

    int32_t tmp1_q27, tmp2_q27, tmp3_q27;
    int32_t xOverY_q27, yOverX_q27;
    int32_t xOverYSquared_q27, yOverXSquared_q27;

    // If the absolute values of x and y are the same, then the angle is 45 degrees off the x-axis.
    //   Only need to determine the quadrant to find the angle.  Additionally, check if both are
    //   zero.  FIXME: Return zero but should also set a bit indicating this.
    if( nabs_y == nabs_x ) {
        if( y_q27 > 0 ) {
            if( x_q27 > 0 ) {
                // angle = round( ( 45* pi/180 ) * 2^27 );
                return 105414357;
            } else {
                // angle = round( ( 3*45* pi/180 ) * 2^27 )
                return 316243071;
            }
        } else if( y_q27 < 0 ) {
            if( x_q27 > 0 ) {
                // angle = round( ( -45* pi/180 ) * 2^27 );
                return -105414357;
            } else {
                // angle = round( ( -3*45* pi/180 ) * 2^27 );
                return -316243071;
            }
        } else {
            // Both x and y are zero.  Indeterminate angle (return 0 but set an error flag)
            return 0;
        }
    }

    // The point is on the y-axis. Angle = +/-90 deg
    if( nabs_x == 0 ) {
        if( y_q27 > 0 ) {
            //angle = round( ( 90 * pi/180 ) * 2^27 );
            return 210828714;
        } else {
            // angle = round( ( -90 * pi/180 ) * 2^27 );
            return -210828714;
        }
    }

    // The point is on the x-axis; angle = 0, 180 deg (but return -180 degrees to be consistent with
    //   range definition: [ -pi, +pi )
    if( nabs_y == 0 ) {
        if( x_q27 > 0 ) {
            // angle = round( ( 0 * pi/180 ) * 2^27 );
            return 0;
        } else {
            // angle = round( ( -180 * pi/180 ) * 2^27 );
            return -421657428;
        }
    }

    // Point is not on the x or y-axes nor is it on a +/-45 degree line.  Calculate the angle
    if( nabs_x < nabs_y ) {
        // The point is above a line that is 45 degree above the x-axis (octants II, III, Vi, and VII)
        xOverY_q27 = _qdiv( x_q27, y_q27, 27, 27, 27 ); // ( x >> 27 ) / y;
        xOverYSquared_q27 = _qmul( xOverY_q27, xOverY_q27, 27, 27, 27 );

        tmp1_q27 = coeff.a2_q27 - _qmul( coeff.a3_q27, xOverYSquared_q27, 27, 27, 27 );
        tmp2_q27 = coeff.a1_q27 - _qmul( tmp1_q27, xOverYSquared_q27, 27, 27, 27 );
        tmp3_q27 = coeff.a0_q27 - _qmul( tmp2_q27, xOverYSquared_q27, 27, 27, 27 );
        angle = _qmul( xOverY_q27, tmp3_q27, 27, 27, 27 );

        //  Adjust the angle depending upon the octant in which the point falls
        if( y_q27 > 0 ) {
            // angle = 90 - result
            angle = 210828714 - angle;
        } else {
            angle = -210828714 - angle;
        }
    } else {
        // The point is below a line that is 45 degree above the x-axis (octants I, IV, V, and VIII)
        yOverX_q27 = _qdiv( y_q27, x_q27, 27, 27, 27 );
        yOverXSquared_q27 = _qmul( yOverX_q27, yOverX_q27, 27, 27, 27 );

        tmp1_q27 = coeff.a2_q27 - _qmul( coeff.a3_q27, yOverXSquared_q27, 27, 27, 27 );
        tmp2_q27 = coeff.a1_q27 - _qmul( tmp1_q27, yOverXSquared_q27, 27, 27, 27 );
        tmp3_q27 = coeff.a0_q27 - _qmul( tmp2_q27, yOverXSquared_q27, 27, 27, 27 );
        angle = _qmul( yOverX_q27, tmp3_q27, 27, 27, 27 );

        // Near theta = 180 degrees, the program needs to check the quadrant when computing the
        //   angle as the output must fall between -pi (-180 deg) and +pi (180 deg)
        if( x_q27 < 0 ) {
            if( y_q27 > 0 ) {
                // In octant IV
                angle = 421657428 + angle;
            } else {
                // In octant V
                angle = -421657429 + angle;
            }
        }
    }

    // Must be less than 2,147,483,648 = 2^15-1
    return (int32_t)angle;
}

// atan( y, x ) returns an angle in the range [ -pi, +pi ).  A Q29 representation will encompass
//   the range of output values.  The input must support a wider range of values
int32_t atan2_q29Out_q27In( int32_t y_q27, int32_t x_q27 )
{
    static atan2_coeff coeff;

    //            fixed-point            floating-point
    //               value                    value
    coeff.a0_q29 = 536479744;   // a0 = 0.999271392822266
    coeff.a1_q29 = 172609805;   // a1 = 0.321510815992951
    coeff.a2_q29 =  78842456;   // a2 = 0.146855518221855
    coeff.a3_q29 =  21100318;   // a3 = 0.039302404969931

    coeff.a0_q30 = 1072959488;
    coeff.a1_q30 =  345219611;
    coeff.a2_q30 =  157684912;
    coeff.a3_q30 =   42200635;

    int32_t nabs_y = nabs( y_q27 );
    int32_t nabs_x = nabs( x_q27 );

    int32_t angle_q29;

    int32_t tmp1_q30, tmp2_q30, tmp3_q30;
    int32_t xOverY_q30, yOverX_q30;
    int32_t xOverYSquared_q30, yOverXSquared_q30;

    // If the absolute values of x and y are the same, then the angle is 45 degrees off the x-axis.
    //   Only need to determine the quadrant to find the angle.  Additionally, check if both are
    //   zero.  FIXME: Return zero but should also set a bit indicating this.
    if( nabs_y == nabs_x ) {
        if( y_q27 > 0 ) {
            if( x_q27 > 0 ) {
                return FORTY_FIVE_DEGREES_Q29;
            } else {
                return ONE_HUNDRED_THIRTY_FIVE_DEGREES_Q29;
            }
        } else if( y_q27 < 0 ) {
            if( x_q27 > 0 ) {
                return -FORTY_FIVE_DEGREES_Q29;
            } else {
                return -ONE_HUNDRED_THIRTY_FIVE_DEGREES_Q29;
            }
        } else {
            // Both x and y are zero.  Indeterminate angle (return 0 but set an error flag)
            return 0;
        }
    }

    // The point is on the y-axis. Angle = +/-90 deg
    if( nabs_x == 0 ) {
        if( y_q27 > 0 ) {
            return NINETY_DEGREES_Q29;
        } else {
            return -NINETY_DEGREES_Q29;
        }
    }

    // The point is on the x-axis; angle = 0, 180 deg (but return -180 degrees to be consistent with
    //   range definition: [ -pi, +pi )
    if( nabs_y == 0 ) {
        if( x_q27 > 0 ) {
            return 0;
        } else {
            return -ONE_HUNDRED_EIGHTY_DEGS_Q29;
        }
    }

    // Point is not on the x or y-axes nor is it on a +/-45 degree line.  Calculate the angle.
    if( nabs_x < nabs_y ) {
        // The point is above a line that is 45 degree above the x-axis (octants II, III, VI, and VII)
        //  The slope is always less than one as, in this case, it is computed from the y-axis
        xOverY_q30 = _qdiv( x_q27, y_q27, 27, 27, 30 );
        xOverYSquared_q30 = _qmul( xOverY_q30, xOverY_q30, 30, 30, 30 );

        // Can use Q30 since the angle will be less than 45 degrees (pi/4) but convert it at the end
        //   so it can be combined with the offset in q_29 format.
        tmp1_q30 = coeff.a2_q30 - _qmul( coeff.a3_q30, xOverYSquared_q30, 30, 30, 30 );
        tmp2_q30 = coeff.a1_q30 - _qmul( tmp1_q30, xOverYSquared_q30, 30, 30, 30 );
        tmp3_q30 = coeff.a0_q30 - _qmul( tmp2_q30, xOverYSquared_q30, 30, 30, 30 );
        angle_q29 = _qmul( xOverY_q30, tmp3_q30, 30, 30, 29 );

        //  Adjust the angle depending upon the octant in which the point falls
        if( y_q27 > 0 ) {
            // angle = 90 - result
            angle_q29 = NINETY_DEGREES_Q29 - angle_q29;
        } else {
            angle_q29 = -NINETY_DEGREES_Q29 - angle_q29;
        }
    } else {
        // The point is below a line that is 45 degree above the x-axis (octants I, IV, V, and VIII)
        yOverX_q30 = _qdiv( y_q27, x_q27, 27, 27, 30 );
        yOverXSquared_q30 = _qmul( yOverX_q30, yOverX_q30, 30, 30, 30 );

        // Can use Q30 since the angle will be less than 45 degrees (pi/4) but convert it at the end
        //   so it can be combined with the offset in q_29 format.
        tmp1_q30 = coeff.a2_q30 - _qmul( coeff.a3_q30, yOverXSquared_q30, 30, 30, 30 );
        tmp2_q30 = coeff.a1_q30 - _qmul( tmp1_q30, yOverXSquared_q30, 30, 30, 30 );
        tmp3_q30 = coeff.a0_q30 - _qmul( tmp2_q30, yOverXSquared_q30, 30, 30, 30 );
        angle_q29 = _qmul( yOverX_q30, tmp3_q30, 30, 30, 29 );

        // Near theta = 180 degrees, the program needs to check the quadrant when computing the
        //   angle as the output must fall between -pi (-180 deg) and +pi (180 deg)
        if( x_q27 < 0 ) {
            if( y_q27 > 0 ) {
                // In octant IV
                angle_q29 = ONE_HUNDRED_EIGHTY_DEGS_Q29 + angle_q29;
            } else {
                // In octant V
                angle_q29 = -ONE_HUNDRED_EIGHTY_DEGS_Q29 + angle_q29;
            }
        }
    }

    // Must be less than 2,147,483,648 = 2^15-1
    return (int32_t)angle_q29;
}

// atan( y, x ) returns an angle in the range [ -pi, +pi ).  A Q29 representation will encompass
//   the range of output values.  The input must support a wider range of values
int32_t atan2_q29Out_q30In( int32_t y_q30,
                            int32_t x_q30 )
{
    static atan2_coeff coeff;

    //            fixed-point            floating-point
    //               value                    value
    coeff.a0_q29 = 536479744;   // a0 = 0.999271392822266
    coeff.a1_q29 = 172609805;   // a1 = 0.321510815992951
    coeff.a2_q29 =  78842456;   // a2 = 0.146855518221855
    coeff.a3_q29 =  21100318;   // a3 = 0.039302404969931

    coeff.a0_q30 = 1072959488;
    coeff.a1_q30 =  345219611;
    coeff.a2_q30 =  157684912;
    coeff.a3_q30 =   42200635;

    int32_t nabs_y = nabs( y_q30 );
    int32_t nabs_x = nabs( x_q30 );

    int32_t angle_q29;

    int32_t tmp1_q30, tmp2_q30, tmp3_q30;
    int32_t xOverY_q30, yOverX_q30;
    int32_t xOverYSquared_q30, yOverXSquared_q30;

    // If the absolute values of x and y are the same, then the angle is 45 degrees off the x-axis.
    //   Only need to determine the quadrant to find the angle.  Additionally, check if both are
    //   zero.  FIXME: Return zero but should also set a bit indicating this.
    if( nabs_y == nabs_x ) {
        if( y_q30 > 0 ) {
            if( x_q30 > 0 ) {
                return FORTY_FIVE_DEGREES_Q29;
            } else {
                return ONE_HUNDRED_THIRTY_FIVE_DEGREES_Q29;
            }
        } else if( y_q30 < 0 ) {
            if( x_q30 > 0 ) {
                return -FORTY_FIVE_DEGREES_Q29;
            } else {
                return -ONE_HUNDRED_THIRTY_FIVE_DEGREES_Q29;
            }
        } else {
            // Both x and y are zero.  Indeterminate angle (return 0 but set an error flag)
            return 0;
        }
    }

    // The point is on the y-axis. Angle = +/-90 deg
    if( nabs_x == 0 ) {
        if( y_q30 > 0 ) {
            return NINETY_DEGREES_Q29;
        } else {
            return -NINETY_DEGREES_Q29;
        }
    }

    // The point is on the x-axis; angle = 0, 180 deg (but return -180 degrees to be consistent with
    //   range definition: [ -pi, +pi )
    if( nabs_y == 0 ) {
        if( x_q30 > 0 ) {
            return 0;
        } else {
            return -ONE_HUNDRED_EIGHTY_DEGS_Q29;
        }
    }

    // Point is not on the x or y-axes nor is it on a +/-45 degree line.  Calculate the angle.
    if( nabs_x < nabs_y ) {
        // The point is above a line that is 45 degree above the x-axis (octants II, III, VI, and VII)
        //  The slope is always less than one as, in this case, it is computed from the y-axis
        xOverY_q30 = _qdiv( x_q30, y_q30, 30, 30, 30 );
        xOverYSquared_q30 = _qmul( xOverY_q30, xOverY_q30, 30, 30, 30 );

        // Can use Q30 since the angle will be less than 45 degrees (pi/4) but convert it at the end
        //   so it can be combined with the offset in q_29 format.
        tmp1_q30 = coeff.a2_q30 - _qmul( coeff.a3_q30, xOverYSquared_q30, 30, 30, 30 );
        tmp2_q30 = coeff.a1_q30 - _qmul( tmp1_q30, xOverYSquared_q30, 30, 30, 30 );
        tmp3_q30 = coeff.a0_q30 - _qmul( tmp2_q30, xOverYSquared_q30, 30, 30, 30 );
        angle_q29 = _qmul( xOverY_q30, tmp3_q30, 30, 30, 29 );

        //  Adjust the angle depending upon the octant in which the point falls
        if( y_q30 > 0 ) {
            // angle = 90 - result
            angle_q29 = NINETY_DEGREES_Q29 - angle_q29;
        } else {
            angle_q29 = -NINETY_DEGREES_Q29 - angle_q29;
        }
    } else {
        // The point is below a line that is 45 degree above the x-axis (octants I, IV, V, and VIII)
        yOverX_q30 = _qdiv( y_q30, x_q30, 30, 30, 30 );
        yOverXSquared_q30 = _qmul( yOverX_q30, yOverX_q30, 30, 30, 30 );

        // Can use Q30 since the angle will be less than 45 degrees (pi/4) but convert it at the end
        //   so it can be combined with the offset in q_29 format.
        tmp1_q30 = coeff.a2_q30 - _qmul( coeff.a3_q30, yOverXSquared_q30, 30, 30, 30 );
        tmp2_q30 = coeff.a1_q30 - _qmul( tmp1_q30, yOverXSquared_q30, 30, 30, 30 );
        tmp3_q30 = coeff.a0_q30 - _qmul( tmp2_q30, yOverXSquared_q30, 30, 30, 30 );
        angle_q29 = _qmul( yOverX_q30, tmp3_q30, 30, 30, 29 );

        // Near theta = 180 degrees, the program needs to check the quadrant when computing the
        //   angle as the output must fall between -pi (-180 deg) and +pi (180 deg)
        if( x_q30 < 0 ) {
            if( y_q30 > 0 ) {
                // In octant IV
                angle_q29 = ONE_HUNDRED_EIGHTY_DEGS_Q29 + angle_q29;
            } else {
                // In octant V
                angle_q29 = -ONE_HUNDRED_EIGHTY_DEGS_Q29 + angle_q29;
            }
        }
    }
    return (int32_t)angle_q29; // Must be less than 2,147,483,648 = 2^15-1
}

// atan( y, x ) returns an angle in the range [ -pi, +pi ).  A Q29 representation will encompass
//   the range of output values.  The input must support a wider range of values
int32_t atan2_q29( int32_t y_qX,
                   int32_t x_qX,
                   uint8_t qVal )
{
    static atan2_coeff coeff;

    // Note: while the result of the calculation below is an angle represented in Q29 format, the
    //       intermediate calculation generates an angle between -45 and +45 degrees (pi/4 = 0.7854)
    //       radians.  Therefore, all calulations prior to the final one can be performed using Q30
    //       math.
    //             fixed-point            floating-point
    //                value                    value
    coeff.a0_q30 = 1072959488;
    coeff.a1_q30 =  345219611;
    coeff.a2_q30 =  157684912;
    coeff.a3_q30 =   42200635;

    int32_t nabs_y = nabs( y_qX );
    int32_t nabs_x = nabs( x_qX );

    int32_t angle_q29;

    int32_t tmp1_q30, tmp2_q30, tmp3_q30;
    int32_t xOverY_q30, yOverX_q30;
    int32_t xOverYSquared_q30, yOverXSquared_q30;

    // If the absolute values of x and y are the same, then the angle is 45 degrees off the x-axis.
    //   Only need to determine the quadrant to find the angle.  Additionally, check if both are
    //   zero.  FIXME: Return zero but should also set a bit indicating this.
    if( nabs_y == nabs_x ) {
        if( y_qX > 0 ) {
            if( x_qX > 0 ) {
                return FORTY_FIVE_DEGREES_Q29;
            } else {
                return ONE_HUNDRED_THIRTY_FIVE_DEGREES_Q29;
            }
        } else if( y_qX < 0 ) {
            if( x_qX > 0 ) {
                return -FORTY_FIVE_DEGREES_Q29;
            } else {
                return -ONE_HUNDRED_THIRTY_FIVE_DEGREES_Q29;
            }
        } else {
            // Both x and y are zero.  Indeterminate angle (return 0 but set an error flag)
            return 0;
        }
    }

    // The point is on the y-axis. Angle = +/-90 deg
    if( nabs_x == 0 ) {
        if( y_qX > 0 ) {
            return NINETY_DEGREES_Q29;
        } else {
            return -NINETY_DEGREES_Q29;
        }
    }

    // The point is on the x-axis; angle = 0, 180 deg (but return -180 degrees to be consistent with
    //   range definition: [ -pi, +pi )
    if( nabs_y == 0 ) {
        if( x_qX > 0 ) {
            return 0;
        } else {
            return -ONE_HUNDRED_EIGHTY_DEGS_Q29;
        }
    }

    // Point is not on the x or y-axes nor is it on a +/-45 degree line.  Calculate the angle.
    if( nabs_x < nabs_y ) {
        // The point is above a line that is 45 degree above the x-axis (octants II, III, VI, and VII)
        //  The slope is always less than one as, in this case, it is computed from the y-axis
        xOverY_q30 = _qdiv( x_qX, y_qX, qVal, qVal, 30 );                  // slope < 1 (Q30)
        xOverYSquared_q30 = _qmul( xOverY_q30, xOverY_q30, 30, 30, 30 );   // slope^2 < 1 (Q30)

        // Can use Q30 since the angle will be less than 45 degrees (pi/4) but convert it at the end
        //   so it can be combined with the offset in q_29 format.
        tmp1_q30 = coeff.a2_q30 - _qmul( coeff.a3_q30, xOverYSquared_q30, 30, 30, 30 );
        tmp2_q30 = coeff.a1_q30 - _qmul( tmp1_q30, xOverYSquared_q30, 30, 30, 30 );
        tmp3_q30 = coeff.a0_q30 - _qmul( tmp2_q30, xOverYSquared_q30, 30, 30, 30 );
        angle_q29 = _qmul( xOverY_q30, tmp3_q30, 30, 30, 29 );

        //  Adjust the angle depending upon the octant in which the point falls
        if( y_qX > 0 ) {
            // angle = 90 - result
            angle_q29 = NINETY_DEGREES_Q29 - angle_q29;
        } else {
            angle_q29 = -NINETY_DEGREES_Q29 - angle_q29;
        }
    } else {
        // The point is below a line that is 45 degree above the x-axis (octants I, IV, V, and VIII)
        yOverX_q30 = _qdiv( y_qX, x_qX, qVal, qVal, 30 );
        yOverXSquared_q30 = _qmul( yOverX_q30, yOverX_q30, 30, 30, 30 );

        // Can use Q30 since the angle will be less than 45 degrees (pi/4) but convert it at the end
        //   so it can be combined with the offset in q_29 format.
        tmp1_q30 = coeff.a2_q30 - _qmul( coeff.a3_q30, yOverXSquared_q30, 30, 30, 30 );
        tmp2_q30 = coeff.a1_q30 - _qmul( tmp1_q30, yOverXSquared_q30, 30, 30, 30 );
        tmp3_q30 = coeff.a0_q30 - _qmul( tmp2_q30, yOverXSquared_q30, 30, 30, 30 );
        angle_q29 = _qmul( yOverX_q30, tmp3_q30, 30, 30, 29 );

        // Near theta = 180 degrees check the quadrant
        //   the output must fall between -pi (-180 deg) and +pi (180 deg)
        if( x_qX < 0 ) {
            if( y_qX > 0 ) {
                // In octant IV
                angle_q29 = ONE_HUNDRED_EIGHTY_DEGS_Q29 + angle_q29;
            } else {
                // In octant V
                angle_q29 = -ONE_HUNDRED_EIGHTY_DEGS_Q29 + angle_q29;
            }
        }
    }

    // Must be less than 2,147,483,648 = 2^15-1
    return (int32_t)angle_q29;
}


// Create an atan function
typedef struct sin_coeff {
    int32_t a0_q29;
    int32_t a1_q29;
    int32_t a2_q29;
    int32_t a3_q29;
    int32_t a4_q29;
    int32_t a5_q29;
} sin_coeff;


#define PI_OVER_TWO_Q29 843314857
#define PI_Q29         1686629713

//#define ONE_Q30 1073741824

// Input can range from -pi to +pi.  Must not be larger than 4.0, so angles larger than 229 degrees
//   can't be used.
// Input: angle in radians
// Checked and verified on May 15, 2014
int32_t sin_q30( int32_t angleRad_q29 )
{
    static sin_coeff coeff;
    coeff.a0_q29 = 536870912;
    coeff.a1_q29 =  89478485;
    coeff.a2_q29 =   4473924;
    coeff.a3_q29 =    106522;
    coeff.a4_q29 =      1479;
    coeff.a5_q29 =        13;

    // find the absolute value of the angle and, if greater than pi/2 (843314857), reduce the value
    int32_t ang_q29 = nabs( angleRad_q29 );
    if( ang_q29 > PI_OVER_TWO_Q29 ) {
        // constrain the value to be less than 90 degrees
        ang_q29 = PI_Q29 - ang_q29;
    }

    // check for zero or pi
    if( angleRad_q29 == 0 || ang_q29 == PI_Q29 ) {
        return 0;
    }

    int32_t angleSign = sign( angleRad_q29 );

    // Check for +/- 90 degrees = pi/2 radians = 1073741824 rad_q30
    if( ang_q29 == PI_OVER_TWO_Q29 ) {
        return( angleSign*ONE_Q30 );
    }

    int32_t tmp1_q29, tmp2_q29, tmp3_q29, tmp4_q29, tmp5_q29, tmp6_q30;
    int32_t angleSquared_q29 = _qmul( ang_q29, ang_q29, 29, 29, 29 );

    // Maclaurin series for sine
    //   sin( x ) = x*( 1 - x^2*( 1/3! - x^2*( 1/5! - x^2*( 1/7! - x^2*( 1/9! - x^2*( 1/11! - � ) ) ) ) )
    tmp1_q29 = coeff.a4_q29 - _qmul( angleSquared_q29, coeff.a5_q29, 29, 29, 29 );
    tmp2_q29 = coeff.a3_q29 - _qmul( angleSquared_q29, tmp1_q29, 29, 29, 29 );
    tmp3_q29 = coeff.a2_q29 - _qmul( angleSquared_q29, tmp2_q29, 29, 29, 29 );
    tmp4_q29 = coeff.a1_q29 - _qmul( angleSquared_q29, tmp3_q29, 29, 29, 29 );
    tmp5_q29 = coeff.a0_q29 - _qmul( angleSquared_q29, tmp4_q29, 29, 29, 29 );
    tmp6_q30 = _qmul( ang_q29, tmp5_q29, 29, 29, 30 );

    return( angleSign * tmp6_q30 );
}


// Input can range from -pi to +pi.  Input value cannot be larger than 4.0, so
//   angles larger than 229 degrees won't produce correct results.
// Input: angle in radians
// Checked and verified on May 15, 2014
int32_t cos_q30( int32_t angleRad_q29 )
{
    // Use the identity to calc cosine from sin:
    //   cos( x ) = sin( x + 90 ), where x is in degrees.
    if( angleRad_q29 >= PI_OVER_TWO_Q29 ) {
        //   Note: the angle (x+90) cannot be larger than 229 degrees and remain in q29 format (the
        //         value is represented in radians).  Therefore, if the angle is greater than 90
        //         degrees, subtract 270 (+90-360) from the angle to prevent the intermediate value
        //         from going beyond 229 degrees.
        angleRad_q29 = angleRad_q29 - PI_OVER_TWO_Q29;  // -90
        angleRad_q29 = angleRad_q29 - PI_OVER_TWO_Q29;  // -180
        angleRad_q29 = angleRad_q29 - PI_OVER_TWO_Q29;  // -270
        // Equivalent to (int32_t)( (int64_t)angleRad_q29 - 2529944570 );
    } else {
        // x is less than 90 degrees: cos( x ) = sin( x + 90 )
        angleRad_q29 = angleRad_q29 + PI_OVER_TWO_Q29;
    }
    return sin_q30( angleRad_q29 );
}


void VectorNormalize_q30( int32_t *vectorIn_q27,
                          int32_t *vectorOut_q30 )
{
    int32_t vectorMag_q27;

    // sqrt( v1^2 + v2^2 + v3^2 )
    vectorMag_q27 = VectorMag_q27( vectorIn_q27 );

    vectorOut_q30[0] = _qdiv( vectorIn_q27[0], vectorMag_q27, 27, 27, 30 );
    vectorOut_q30[1] = _qdiv( vectorIn_q27[1], vectorMag_q27, 27, 27, 30 );
    vectorOut_q30[2] = _qdiv( vectorIn_q27[2], vectorMag_q27, 27, 27, 30 );
}


// Components must be less than 1239850262 (Q27) ~ 9.23 (dec) in order to use this function without
//   overflow.  Else, need to represent the output in another q-format.
int32_t VectorMag_q27( int32_t *vectorIn_q27 )
{
    int32_t temp_q27[3];

    //
    temp_q27[0] = _qmul( vectorIn_q27[0], vectorIn_q27[0], 27, 27, 27 );
    temp_q27[1] = _qmul( vectorIn_q27[1], vectorIn_q27[1], 27, 27, 27 );
    temp_q27[2] = _qmul( vectorIn_q27[2], vectorIn_q27[2], 27, 27, 27 );

    // sqrt( v1^2 + v2^2 + v3^2 )
    return( qsqrt_q27( temp_q27[0] + temp_q27[1] + temp_q27[2] ) );
}


void VectorCrossProduct_q27( int32_t *vect1_q27,
                             int32_t *vect2_q27,
                             int32_t *vectOut_q27 )
{
    // |   i       j       k   |
    // | v1[0]   v1[1]   v1[2] | = i*( v1[1]*v2[2] - v1[2]*v2[1] ) - j*( v1[0]*v2[2] - v1[2]*v2[0] ) + k*( v1[0]*v2[1] - v1[1]*v2[0] )
    // | v2[0]   v2[1]   v2[2] |

    vectOut_q27[0] = _qmul( vect1_q27[1], vect2_q27[2], 27, 27, 27 ) -
                     _qmul( vect1_q27[2], vect2_q27[1], 27, 27, 27 );   // v1[1]*v2[2] - v1[2]*v2[1]
    vectOut_q27[1] = _qmul( vect1_q27[2], vect2_q27[0], 27, 27, 27 ) -
                     _qmul( vect1_q27[0], vect2_q27[2], 27, 27, 27 );   // v1[2]*v2[0] - v1[0]*v2[2]
    vectOut_q27[2] = _qmul( vect1_q27[0], vect2_q27[1], 27, 27, 27 ) -
                     _qmul( vect1_q27[1], vect2_q27[0], 27, 27, 27 );   // v1[0]*v2[1] - v1[1]*v2[0]
}

int32_t VectorDotProduct_q27( int32_t *vect1_q27, int32_t *vect2_q27 )
{
    int32_t result_q27;

    result_q27 = _qmul( vect1_q27[0], vect2_q27[0], 27, 27, 27 ) +
                 _qmul( vect1_q27[1], vect2_q27[1], 27, 27, 27 ) +
                 _qmul( vect1_q27[2], vect2_q27[2], 27, 27, 27 );

    return result_q27;
}


//
// 32-bit fixed point (Q27) four-quadrant arctangent. Given a Cartesian vector (x, y), return the
//   angle subtended by the vector and the positive x-axis.  Both function input and output are in
//   q27 format.
//
// The value returned falls in the follwing range: [ -pi, pi ) and is converted to a q27 data-type
//   (floating-point value is multiplied by 2^27).
//
// * Because the magnitude of the input vector does not change the angle it
// * represents, the inputs can be in any signed 16-bit fixed-point format.
// *
// * @param y y-coordinate is int32_t, q27 format
// * @param x x-coordinate is int32_t, q27 format
// * @return angle in int32_t, q27 format between -( pi*2^27 ) and +( pi*2^27 )
//
int32_t atan2Old_q27( int32_t y,
                      int32_t x )
{
    int32_t nabs_y = nabs( y );
    int32_t nabs_x = nabs( x );

    int32_t angle;
    int32_t innerTerm_q27;

    // If the absolute values of x and y are the same, then the angle is 45 degrees off the x-axis.
    //   Only need to determine the quadrant to find the angle.
    if( nabs_y == nabs_x ) {
        if( y > 0 ) {
            if( x > 0 ) {
                return 105414357; // angle = round( ( 45* pi/180 ) * 2^27 );
            } else {
                return 316243071; // angle = round( ( 3*45* pi/180 ) * 2^27 )
            }
        } else if( y < 0 ) {
            if( x > 0 )
            {
                return -105414357; // angle = round( ( -45* pi/180 ) * 2^27 );
            } else {
                return -316243071; // angle = round( ( -3*45* pi/180 ) * 2^27 );
            }
        } else {
            // Both x and y are zero.  Indeterminate angle (return 0 but set error flag)
            return 0;
        }
    }

    // The point is on the y-axis. Angle = +/-90 deg
    if( nabs_x == 0 ) {
        if( y > 0 ) {
            return 210828714;  // angle = round( ( 90 * pi/180 ) * 2^27 );
        } else {
            return -210828714; // angle = round( ( -90 * pi/180 ) * 2^27 );
        }
    }

    // The point is on the x-axis; angle = 0, 180 deg
    if( nabs_y == 0 ) {
        if( x > 0 ) {
            return 0;          // angle = round( ( 0 * pi/180 ) * 2^27 );
        } else {
            return -421657428; // angle = round( ( -180 * pi/180 ) * 2^27 );
        }
    }

    // Point is not on the x or y-axis nor is it on a +/-45 degree line.  Calculate the angle
    if( nabs_x < nabs_y ) {
        // The point is above a line that is 45 degree above the x-axis (octants II and III)
        int32_t xOverY_q27 = _qdiv( x, y, 27, 27, 27 ); // ( x >> 27 ) / y;
        int32_t absXOverY_q27 = nabs( xOverY_q27 );

        // ( pi/4 + 0.273 ) = round( ( pi/4 + 0.273 ) * 2^27 ) = 142055797
        // 0.273 = round( 0.273 * 2^27 ) = 36641440
        innerTerm_q27 = 142055797 - _qmul( 36641440, absXOverY_q27, 27, 27, 27 );
        if( y > 0 ) {
            // angle = 90 - result
            angle = 210828714 - _qmul( xOverY_q27, innerTerm_q27, 27, 27, 27 );
        } else {
            angle = -210828714 - _qmul( xOverY_q27, innerTerm_q27, 27, 27, 27 );
        }
    } else {
        // The point is below a line that is 45 degree above the x-axis (octants I and IV)
        int32_t yOverX_q27 = _qdiv( y, x, 27, 27, 27 ); //fix( y * 2^27 / x );
        int32_t absYOverX_q27 = nabs( yOverX_q27 );

        // ( pi/4 + 0.273 ) = round( ( pi/4 + 0.273 ) * 2^27 ) = 142055797
        // 0.273 = round( 0.273 * 2^27 ) = 36641440
        innerTerm_q27 = 142055797 - _qmul( 36641440, absYOverX_q27, 27, 27, 27 );
        if( x > 0 ) {
            // In octant I or VIII
            angle = _qmul( yOverX_q27, innerTerm_q27, 27, 27, 27 );
        } else {
            // Near theta = 180 degrees, the program needs to check the quadrant when computing the
            //   angle as the output must fall between -pi (-180 deg) and +pi (180 deg)
            if( y > 0 ) {
                // In octant IV or V
                angle =  421657428 + _qmul( yOverX_q27, innerTerm_q27, 27, 27, 27 );
            } else {
                angle = -421657429 + _qmul( yOverX_q27, innerTerm_q27, 27, 27, 27 );
            }
        }
    }
    return (int32_t)angle; // Must be less than 2,147,483,648 = 2^15-1
}


// This is a replacement for 'lowPass2Pole' found in the 440 code.  Note, the implementation in the
//   440 SW is not a second-order filter but is an implementation of a first-order filter.
void firstOrderLowPass_q27( int32_t *output_q27,
                            int32_t *input_q27,
                            int32_t *inputPast_q27,
                            uint8_t shiftCoeff )
{
    *output_q27 = *output_q27 +
                  _qmul( ( *input_q27 - *output_q27 ), ONE_Q30 >> shiftCoeff, 27, 30, 27 ) +
                  _qmul( ( *inputPast_q27 - *output_q27 ), ONE_Q30 >> shiftCoeff, 27, 30, 27 );

    *inputPast_q27 = *input_q27;
}

