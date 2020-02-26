/* **********************************************************************
 *
 * supplement header file to build math library
 *
 * **********************************************************************/
 
#ifndef _XMATH_H
#define _XMATH_H

// These are in IQMath.h

typedef float iq27;
typedef float iq30;
typedef float iq23;
typedef float iq29;

#define IQ27(x) (x)



#define MAXUINT32 4294967295 	///< max unsigned 32 bit int=> ((2^32)-1)
#define MAXUINT16      65535    ///< max unsigned 16 bit int=> ((2^16)-1)
#define MAXINT16     ( 32767)   ///< max signed 16 bit int=> ((2^15)-1)
#define MININT16     (-32768)   ///< max negative signed 16 bit int=> (-(2^15))

#endif /* XMATH_H_ */
