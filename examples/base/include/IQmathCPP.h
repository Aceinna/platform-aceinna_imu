 //###########################################################################
//
// FILE:	IQmathCPP.h
//
// TITLE:	IQ Math C++ definitions.
//
//###########################################################################
//
// Ver  | dd-mmm-yyyy |  Who  | Description of changes
// =====|=============|=======|==============================================
//  1.3 | 19 Nov 2001 | A. T. | Original Release.
// -----|-------------|-------|----------------------------------------------
//  1.4 | 17 May 2002 | A. T. | Added new functions and support for
//      |             |       | intrinsics IQmpy, IQxmpy, IQsat.
//      |             |       | Added support for boolean and bit-wise
//      |             |       | operations.
//      |             |       | Expanded Q support range: 15 <= Q <= 30
// -----|-------------|-------|----------------------------------------------
//  1.4d| 30 Mar 2003 | DA/SD | Added macro definition to include header 
//      |             |       | file multiple times in the program.   
//------|-------------|-------|----------------------------------------------
//
//###########################################################################

#ifndef __IQMATHCPP_H_INCLUDED__

#define __IQMATHCPP_H_INCLUDED__

//###########################################################################
#if MATH_TYPE == IQ_MATH
//###########################################################################
// If IQ_MATH is used, the following IQmath library definitions are used:
//===========================================================================

struct iq {
	//constructors:
	iq () : val(0) { }
	iq (long x) : val(x) { }

	//copy constructor:
	iq (const iq & x) : val(x.val) { }

	//assignment operators:
	inline iq & operator = (const iq & x);

	//arithmetic operators:
	inline iq & operator += (const iq &x);
	inline iq & operator -= (const iq &x);
	inline iq & operator *= (const iq &x);
	inline iq & operator /= (const iq &x);

    //bitwise operators:
    inline iq & operator &= (const long &x);
    inline iq & operator |= (const long &x);
    inline iq & operator ^= (const long &x);

	long val;
};

struct iq30 {
	//constructors:
	iq30 () : val(0) { }
	iq30 (long x) : val(x) { }

	//copy constructor:
	iq30 (const iq30 & x) : val(x.val) { }

	//assignment operators:
	inline iq30 & operator = (const iq30 & x);

	//arithmetic operators:
	inline iq30 & operator += (const iq30 &x);
	inline iq30 & operator -= (const iq30 &x);
	inline iq30 & operator *= (const iq30 &x);
	inline iq30 & operator /= (const iq30 &x);

    //bitwise operators:
    inline iq30 & operator &= (const long &x);
    inline iq30 & operator |= (const long &x);
    inline iq30 & operator ^= (const long &x);

	long val;
};

struct iq29 {
	//constructors:
	iq29 () : val(0) { }
	iq29 (long x) : val(x) { }

	//copy constructor:
	iq29 (const iq29 & x) : val(x.val) { }

	//assignment operators:
	inline iq29 & operator = (const iq29 & x);

	//arithmetic operators:
	inline iq29 & operator += (const iq29 &x);
	inline iq29 & operator -= (const iq29 &x);
	inline iq29 & operator *= (const iq29 &x);
	inline iq29 & operator /= (const iq29 &x);

    //bitwise operators:
    inline iq29 & operator &= (const long &x);
    inline iq29 & operator |= (const long &x);
    inline iq29 & operator ^= (const long &x);

	long val;
};

struct iq28 {
	//constructors:
	iq28 () : val(0) { }
	iq28 (long x) : val(x) { }

	//copy constructor:
	iq28 (const iq28 & x) : val(x.val) { }

	//assignment operators:
	inline iq28 & operator = (const iq28 & x);

	//arithmetic operators:
	inline iq28 & operator += (const iq28 &x);
	inline iq28 & operator -= (const iq28 &x);
	inline iq28 & operator *= (const iq28 &x);
	inline iq28 & operator /= (const iq28 &x);

    //bitwise operators:
    inline iq28 & operator &= (const long &x);
    inline iq28 & operator |= (const long &x);
    inline iq28 & operator ^= (const long &x);

	long val;
};

struct iq27 {
	//constructors:
	iq27 () : val(0) { }
	iq27 (long x) : val(x) { }

	//copy constructor:
	iq27 (const iq27 & x) : val(x.val) { }

	//assignment operators:
	inline iq27 & operator = (const iq27 & x);

	//arithmetic operators:
	inline iq27 & operator += (const iq27 &x);
	inline iq27 & operator -= (const iq27 &x);
	inline iq27 & operator *= (const iq27 &x);
	inline iq27 & operator /= (const iq27 &x);

    //bitwise operators:
    inline iq27 & operator &= (const long &x);
    inline iq27 & operator |= (const long &x);
    inline iq27 & operator ^= (const long &x);

	long val;
};

struct iq26 {
	//constructors:
	iq26 () : val(0) { }
	iq26 (long x) : val(x) { }

	//copy constructor:
	iq26 (const iq26 & x) : val(x.val) { }

	//assignment operators:
	inline iq26 & operator = (const iq26 & x);

	//arithmetic operators:
	inline iq26 & operator += (const iq26 &x);
	inline iq26 & operator -= (const iq26 &x);
	inline iq26 & operator *= (const iq26 &x);
	inline iq26 & operator /= (const iq26 &x);

    //bitwise operators:
    inline iq26 & operator &= (const long &x);
    inline iq26 & operator |= (const long &x);
    inline iq26 & operator ^= (const long &x);

	long val;
};

struct iq25 {
	//constructors:
	iq25 () : val(0) { }
	iq25 (long x) : val(x) { }

	//copy constructor:
	iq25 (const iq25 & x) : val(x.val) { }

	//assignment operators:
	inline iq25 & operator = (const iq25 & x);

	//arithmetic operators:
	inline iq25 & operator += (const iq25 &x);
	inline iq25 & operator -= (const iq25 &x);
	inline iq25 & operator *= (const iq25 &x);
	inline iq25 & operator /= (const iq25 &x);

    //bitwise operators:
    inline iq25 & operator &= (const long &x);
    inline iq25 & operator |= (const long &x);
    inline iq25 & operator ^= (const long &x);

	long val;
};

struct iq24 {
	//constructors:
	iq24 () : val(0) { }
	iq24 (long x) : val(x) { }

	//copy constructor:
	iq24 (const iq24 & x) : val(x.val) { }

	//assignment operators:
	inline iq24 & operator = (const iq24 & x);

	//arithmetic operators:
	inline iq24 & operator += (const iq24 &x);
	inline iq24 & operator -= (const iq24 &x);
	inline iq24 & operator *= (const iq24 &x);
	inline iq24 & operator /= (const iq24 &x);

    //bitwise operators:
    inline iq24 & operator &= (const long &x);
    inline iq24 & operator |= (const long &x);
    inline iq24 & operator ^= (const long &x);

	long val;
};

struct iq23 {
	//constructors:
	iq23 () : val(0) { }
	iq23 (long x) : val(x) { }

	//copy constructor:
	iq23 (const iq23 & x) : val(x.val) { }

	//assignment operators:
	inline iq23 & operator = (const iq23 & x);

	//arithmetic operators:
	inline iq23 & operator += (const iq23 &x);
	inline iq23 & operator -= (const iq23 &x);
	inline iq23 & operator *= (const iq23 &x);
	inline iq23 & operator /= (const iq23 &x);

    //bitwise operators:
    inline iq23 & operator &= (const long &x);
    inline iq23 & operator |= (const long &x);
    inline iq23 & operator ^= (const long &x);

	long val;
};

struct iq22 {
	//constructors:
	iq22 () : val(0) { }
	iq22 (long x) : val(x) { }

	//copy constructor:
	iq22 (const iq22 & x) : val(x.val) { }

	//assignment operators:
	inline iq22 & operator = (const iq22 & x);

	//arithmetic operators:
	inline iq22 & operator += (const iq22 &x);
	inline iq22 & operator -= (const iq22 &x);
	inline iq22 & operator *= (const iq22 &x);
	inline iq22 & operator /= (const iq22 &x);

    //bitwise operators:
    inline iq22 & operator &= (const long &x);
    inline iq22 & operator |= (const long &x);
    inline iq22 & operator ^= (const long &x);

	long val;
};

struct iq21 {
	//constructors:
	iq21 () : val(0) { }
	iq21 (long x) : val(x) { }

	//copy constructor:
	iq21 (const iq21 & x) : val(x.val) { }

	//assignment operators:
	inline iq21 & operator = (const iq21 & x);

	//arithmetic operators:
	inline iq21 & operator += (const iq21 &x);
	inline iq21 & operator -= (const iq21 &x);
	inline iq21 & operator *= (const iq21 &x);
	inline iq21 & operator /= (const iq21 &x);

    //bitwise operators:
    inline iq21 & operator &= (const long &x);
    inline iq21 & operator |= (const long &x);
    inline iq21 & operator ^= (const long &x);

	long val;
};

struct iq20 {
	//constructors:
	iq20 () : val(0) { }
	iq20 (long x) : val(x) { }

	//copy constructor:
	iq20 (const iq20 & x) : val(x.val) { }

	//assignment operators:
	inline iq20 & operator = (const iq20 & x);

	//arithmetic operators:
	inline iq20 & operator += (const iq20 &x);
	inline iq20 & operator -= (const iq20 &x);
	inline iq20 & operator *= (const iq20 &x);
	inline iq20 & operator /= (const iq20 &x);

    //bitwise operators:
    inline iq20 & operator &= (const long &x);
    inline iq20 & operator |= (const long &x);
    inline iq20 & operator ^= (const long &x);

	long val;
};

struct iq19 {
	//constructors:
	iq19 () : val(0) { }
	iq19 (long x) : val(x) { }

	//copy constructor:
	iq19 (const iq19 & x) : val(x.val) { }

	//assignment operators:
	inline iq19 & operator = (const iq19 & x);

	//arithmetic operators:
	inline iq19 & operator += (const iq19 &x);
	inline iq19 & operator -= (const iq19 &x);
	inline iq19 & operator *= (const iq19 &x);
	inline iq19 & operator /= (const iq19 &x);

    //bitwise operators:
    inline iq19 & operator &= (const long &x);
    inline iq19 & operator |= (const long &x);
    inline iq19 & operator ^= (const long &x);

	long val;
};

struct iq18 {
	//constructors:
	iq18 () : val(0) { }
	iq18 (long x) : val(x) { }

	//copy constructor:
	iq18 (const iq18 & x) : val(x.val) { }

	//assignment operators:
	inline iq18 & operator = (const iq18 & x);

	//arithmetic operators:
	inline iq18 & operator += (const iq18 &x);
	inline iq18 & operator -= (const iq18 &x);
	inline iq18 & operator *= (const iq18 &x);
	inline iq18 & operator /= (const iq18 &x);

    //bitwise operators:
    inline iq18 & operator &= (const long &x);
    inline iq18 & operator |= (const long &x);
    inline iq18 & operator ^= (const long &x);

	long val;
};

struct iq17 {
	//constructors:
	iq17 () : val(0) { }
	iq17 (long x) : val(x) { }

	//copy constructor:
	iq17 (const iq17 & x) : val(x.val) { }

	//assignment operators:
	inline iq17 & operator = (const iq17 & x);

	//arithmetic operators:
	inline iq17 & operator += (const iq17 &x);
	inline iq17 & operator -= (const iq17 &x);
	inline iq17 & operator *= (const iq17 &x);
	inline iq17 & operator /= (const iq17 &x);

    //bitwise operators:
    inline iq17 & operator &= (const long &x);
    inline iq17 & operator |= (const long &x);
    inline iq17 & operator ^= (const long &x);

	long val;
};

struct iq16 {
	//constructors:
	iq16 () : val(0) { }
	iq16 (long x) : val(x) { }

	//copy constructor:
	iq16 (const iq16 & x) : val(x.val) { }

	//assignment operators:
	inline iq16 & operator = (const iq16 & x);

	//arithmetic operators:
	inline iq16 & operator += (const iq16 &x);
	inline iq16 & operator -= (const iq16 &x);
	inline iq16 & operator *= (const iq16 &x);
	inline iq16 & operator /= (const iq16 &x);

    //bitwise operators:
    inline iq16 & operator &= (const long &x);
    inline iq16 & operator |= (const long &x);
    inline iq16 & operator ^= (const long &x);

	long val;
};

struct iq15 {
	//constructors:
	iq15 () : val(0) { }
	iq15 (long x) : val(x) { }

	//copy constructor:
	iq15 (const iq15 & x) : val(x.val) { }

	//assignment operators:
	inline iq15 & operator = (const iq15 & x);

	//arithmetic operators:
	inline iq15 & operator += (const iq15 &x);
	inline iq15 & operator -= (const iq15 &x);
	inline iq15 & operator *= (const iq15 &x);
	inline iq15 & operator /= (const iq15 &x);

    //bitwise operators:
    inline iq15 & operator &= (const long &x);
    inline iq15 & operator |= (const long &x);
    inline iq15 & operator ^= (const long &x);

	long val;
};

//---------------------------------------------------------------------------
// Functions: IQ(A), IQN(A)
//---------------------------------------------------------------------------

#define IQ(A)   (iq)   _IQ(A)
#define IQ30(A) (iq30) _IQ30(A)
#define IQ29(A) (iq29) _IQ29(A)
#define IQ28(A) (iq28) _IQ28(A)
#define IQ27(A) (iq27) _IQ27(A)
#define IQ26(A) (iq26) _IQ26(A)
#define IQ25(A) (iq25) _IQ25(A)
#define IQ24(A) (iq24) _IQ24(A)
#define IQ23(A) (iq23) _IQ23(A)
#define IQ22(A) (iq22) _IQ22(A)
#define IQ21(A) (iq21) _IQ21(A)
#define IQ20(A) (iq20) _IQ20(A)
#define IQ19(A) (iq19) _IQ19(A)
#define IQ18(A) (iq18) _IQ18(A)
#define IQ17(A) (iq17) _IQ17(A)
#define IQ16(A) (iq16) _IQ16(A)
#define IQ15(A) (iq15) _IQ15(A)

//---------------------------------------------------------------------------
// Functions: IQtoF(A), IQNtoF(A)
//---------------------------------------------------------------------------

inline	float	IQtoF(iq x)     { return _IQtoF(x.val);   }
inline	float	IQ30toF(iq30 x) { return _IQ30toF(x.val); }
inline	float	IQ29toF(iq29 x) { return _IQ29toF(x.val); }
inline	float	IQ28toF(iq28 x) { return _IQ28toF(x.val); }
inline	float	IQ27toF(iq27 x) { return _IQ27toF(x.val); }
inline	float	IQ26toF(iq26 x) { return _IQ26toF(x.val); }
inline	float	IQ25toF(iq25 x) { return _IQ25toF(x.val); }
inline	float	IQ24toF(iq24 x) { return _IQ24toF(x.val); }
inline	float	IQ23toF(iq23 x) { return _IQ23toF(x.val); }
inline	float	IQ22toF(iq22 x) { return _IQ22toF(x.val); }
inline	float	IQ21toF(iq21 x) { return _IQ21toF(x.val); }
inline	float	IQ20toF(iq20 x) { return _IQ20toF(x.val); }
inline	float	IQ19toF(iq19 x) { return _IQ19toF(x.val); }
inline	float	IQ18toF(iq18 x) { return _IQ18toF(x.val); }
inline	float	IQ17toF(iq17 x) { return _IQ17toF(x.val); }
inline	float	IQ16toF(iq16 x) { return _IQ16toF(x.val); }
inline	float	IQ15toF(iq15 x) { return _IQ15toF(x.val); }

//---------------------------------------------------------------------------
// Functions: IQsat(A, Pos, Neg)
//---------------------------------------------------------------------------

inline	iq		IQsat(iq x, iq Pos, iq Neg)   
{ 
	iq temp;
	temp.val = _IQsat(x.val, Pos.val, Neg.val);
	return temp;   
}

inline	iq30	IQsat(iq30 x, iq30 Pos, iq30 Neg)   
{ 
	iq30 temp;
	temp.val = _IQsat(x.val, Pos.val, Neg.val);
	return temp;   
}

inline	iq29	IQsat(iq29 x, iq29 Pos, iq29 Neg)   
{ 
	iq29 temp;
	temp.val = _IQsat(x.val, Pos.val, Neg.val);
	return temp;   
}

inline	iq28	IQsat(iq28 x, iq28 Pos, iq28 Neg)   
{ 
	iq28 temp;
	temp.val = _IQsat(x.val, Pos.val, Neg.val);
	return temp;   
}

inline	iq27	IQsat(iq27 x, iq27 Pos, iq27 Neg)   
{ 
	iq27 temp;
	temp.val = _IQsat(x.val, Pos.val, Neg.val);
	return temp;   
}

inline	iq26	IQsat(iq26 x, iq26 Pos, iq26 Neg)   
{ 
	iq26 temp;
	temp.val = _IQsat(x.val, Pos.val, Neg.val);
	return temp;   
}

inline	iq25	IQsat(iq25 x, iq25 Pos, iq25 Neg)   
{ 
	iq25 temp;
	temp.val = _IQsat(x.val, Pos.val, Neg.val);
	return temp;   
}

inline	iq24	IQsat(iq24 x, iq24 Pos, iq24 Neg)   
{ 
	iq24 temp;
	temp.val = _IQsat(x.val, Pos.val, Neg.val);
	return temp;   
}

inline	iq23	IQsat(iq23 x, iq23 Pos, iq23 Neg)   
{ 
	iq23 temp;
	temp.val = _IQsat(x.val, Pos.val, Neg.val);
	return temp;   
}

inline	iq22	IQsat(iq22 x, iq22 Pos, iq22 Neg)   
{ 
	iq22 temp;
	temp.val = _IQsat(x.val, Pos.val, Neg.val);
	return temp;   
}

inline	iq21	IQsat(iq21 x, iq21 Pos, iq21 Neg)   
{ 
	iq21 temp;
	temp.val = _IQsat(x.val, Pos.val, Neg.val);
	return temp;   
}

inline	iq20	IQsat(iq20 x, iq20 Pos, iq20 Neg)   
{ 
	iq20 temp;
	temp.val = _IQsat(x.val, Pos.val, Neg.val);
	return temp;   
}

inline	iq19	IQsat(iq19 x, iq19 Pos, iq19 Neg)   
{ 
	iq19 temp;
	temp.val = _IQsat(x.val, Pos.val, Neg.val);
	return temp;   
}

inline	iq18	IQsat(iq18 x, iq18 Pos, iq18 Neg)   
{ 
	iq18 temp;
	temp.val = _IQsat(x.val, Pos.val, Neg.val);
	return temp;   
}

inline	iq17	IQsat(iq17 x, iq17 Pos, iq17 Neg)   
{ 
	iq17 temp;
	temp.val = _IQsat(x.val, Pos.val, Neg.val);
	return temp;   
}

inline	iq16	IQsat(iq16 x, iq16 Pos, iq16 Neg)   
{ 
	iq16 temp;
	temp.val = _IQsat(x.val, Pos.val, Neg.val);
	return temp;   
}

inline	iq15	IQsat(iq15 x, iq15 Pos, iq15 Neg)   
{ 
	iq15 temp;
	temp.val = _IQsat(x.val, Pos.val, Neg.val);
	return temp;   
}

//---------------------------------------------------------------------------
// Functions: IQtoIQN(A)
//---------------------------------------------------------------------------

inline iq30 IQtoIQ30(iq x)
{
	iq30 temp;
	temp.val = _IQtoIQ30(x.val);
	return temp;
}

inline iq29 IQtoIQ29(iq x)
{
	iq29 temp;
	temp.val = _IQtoIQ29(x.val);
	return temp;
}

inline iq28 IQtoIQ28(iq x)
{
	iq28 temp;
	temp.val = _IQtoIQ28(x.val);
	return temp;
}

inline iq27 IQtoIQ27(iq x)
{
	iq27 temp;
	temp.val = _IQtoIQ27(x.val);
	return temp;
}

inline iq26 IQtoIQ26(iq x)
{
	iq26 temp;
	temp.val = _IQtoIQ26(x.val);
	return temp;
}

inline iq25 IQtoIQ25(iq x)
{
	iq25 temp;
	temp.val = _IQtoIQ25(x.val);
	return temp;
}

inline iq24 IQtoIQ24(iq x)
{
	iq24 temp;
	temp.val = _IQtoIQ24(x.val);
	return temp;
}

inline iq23 IQtoIQ23(iq x)
{
	iq23 temp;
	temp.val = _IQtoIQ23(x.val);
	return temp;
}

inline iq22 IQtoIQ22(iq x)
{
	iq22 temp;
	temp.val = _IQtoIQ22(x.val);
	return temp;
}

inline iq21 IQtoIQ21(iq x)
{
	iq21 temp;
	temp.val = _IQtoIQ21(x.val);
	return temp;
}

inline iq20 IQtoIQ20(iq x)
{
	iq20 temp;
	temp.val = _IQtoIQ20(x.val);
	return temp;
}

inline iq19 IQtoIQ19(iq x)
{
	iq19 temp;
	temp.val = _IQtoIQ19(x.val);
	return temp;
}

inline iq18 IQtoIQ18(iq x)
{
	iq18 temp;
	temp.val = _IQtoIQ18(x.val);
	return temp;
}

inline iq17 IQtoIQ17(iq x)
{
	iq17 temp;
	temp.val = _IQtoIQ17(x.val);
	return temp;
}

inline iq16 IQtoIQ16(iq x)
{
	iq16 temp;
	temp.val = _IQtoIQ16(x.val);
	return temp;
}

inline iq15 IQtoIQ15(iq x)
{
	iq15 temp;
	temp.val = _IQtoIQ15(x.val);
	return temp;
}

//---------------------------------------------------------------------------
// Functions: IQNtoIQ(A)
//---------------------------------------------------------------------------

inline iq IQ30toIQ(iq30 x)
{
	iq temp;
	temp.val = _IQ30toIQ(x.val);
	return temp;
}

inline iq IQ29toIQ(iq29 x)
{
	iq temp;
	temp.val = _IQ29toIQ(x.val);
	return temp;
}

inline iq IQ28toIQ(iq28 x)
{
	iq temp;
	temp.val = _IQ28toIQ(x.val);
	return temp;
}

inline iq IQ27toIQ(iq27 x)
{
	iq temp;
	temp.val = _IQ27toIQ(x.val);
	return temp;
}

inline iq IQ26toIQ(iq26 x)
{
	iq temp;
	temp.val = _IQ26toIQ(x.val);
	return temp;
}

inline iq IQ25toIQ(iq25 x)
{
	iq temp;
	temp.val = _IQ25toIQ(x.val);
	return temp;
}

inline iq IQ24toIQ(iq24 x)
{
	iq temp;
	temp.val = _IQ24toIQ(x.val);
	return temp;
}

inline iq IQ23toIQ(iq23 x)
{
	iq temp;
	temp.val = _IQ23toIQ(x.val);
	return temp;
}

inline iq IQ22toIQ(iq22 x)
{
	iq temp;
	temp.val = _IQ22toIQ(x.val);
	return temp;
}

inline iq IQ21toIQ(iq21 x)
{
	iq temp;
	temp.val = _IQ21toIQ(x.val);
	return temp;
}

inline iq IQ20toIQ(iq20 x)
{
	iq temp;
	temp.val = _IQ20toIQ(x.val);
	return temp;
}

inline iq IQ19toIQ(iq19 x)
{
	iq temp;
	temp.val = _IQ19toIQ(x.val);
	return temp;
}

inline iq IQ18toIQ(iq18 x)
{
	iq temp;
	temp.val = _IQ18toIQ(x.val);
	return temp;
}

inline iq IQ17toIQ(iq17 x)
{
	iq temp;
	temp.val = _IQ17toIQ(x.val);
	return temp;
}

inline iq IQ16toIQ(iq16 x)
{
	iq temp;
	temp.val = _IQ16toIQ(x.val);
	return temp;
}

inline iq IQ15toIQ(iq15 x)
{
	iq temp;
	temp.val = _IQ15toIQ(x.val);
	return temp;
}

//---------------------------------------------------------------------------
// Functions: IQtoQN(A)
//---------------------------------------------------------------------------

inline	long	IQtoQ15(iq x)  { return _IQtoQ15(x.val);   }
inline	long	IQtoQ14(iq x)  { return _IQtoQ14(x.val);   }
inline	long	IQtoQ13(iq x)  { return _IQtoQ13(x.val);   }
inline	long	IQtoQ12(iq x)  { return _IQtoQ12(x.val);   }
inline	long	IQtoQ11(iq x)  { return _IQtoQ11(x.val);   }
inline	long	IQtoQ10(iq x)  { return _IQtoQ10(x.val);   }
inline	long	IQtoQ9(iq x)   { return _IQtoQ9(x.val);    }
inline	long	IQtoQ8(iq x)   { return _IQtoQ8(x.val);    }
inline	long	IQtoQ7(iq x)   { return _IQtoQ7(x.val);    }
inline	long	IQtoQ6(iq x)   { return _IQtoQ6(x.val);    }
inline	long	IQtoQ5(iq x)   { return _IQtoQ5(x.val);    }
inline	long	IQtoQ4(iq x)   { return _IQtoQ4(x.val);    }
inline	long	IQtoQ3(iq x)   { return _IQtoQ3(x.val);    }
inline	long	IQtoQ2(iq x)   { return _IQtoQ2(x.val);    }
inline	long	IQtoQ1(iq x)   { return _IQtoQ1(x.val);    }

//---------------------------------------------------------------------------
// Functions: QNtoIQ(A)
//---------------------------------------------------------------------------

inline	iq	Q15toIQ(long x)
{
   iq temp;
   temp.val = _Q15toIQ(x);
   return temp;
}

inline	iq	Q14toIQ(long x)
{
   iq temp;
   temp.val = _Q14toIQ(x);
   return temp;
}

inline	iq	Q13toIQ(long x)
{
   iq temp;
   temp.val = _Q13toIQ(x);
   return temp;
}

inline	iq	Q12toIQ(long x)
{
   iq temp;
   temp.val = _Q12toIQ(x);
   return temp;
}

inline	iq	Q11toIQ(long x)
{
   iq temp;
   temp.val = _Q11toIQ(x);
   return temp;
}

inline	iq	Q10toIQ(long x)
{
   iq temp;
   temp.val = _Q10toIQ(x);
   return temp;
}

inline	iq	Q9toIQ(long x)
{
   iq temp;
   temp.val = _Q9toIQ(x);
   return temp;
}

inline	iq	Q8toIQ(long x)
{
   iq temp;
   temp.val = _Q8toIQ(x);
   return temp;
}

inline	iq	Q7toIQ(long x)
{
   iq temp;
   temp.val = _Q7toIQ(x);
   return temp;
}

inline	iq	Q6toIQ(long x)
{
   iq temp;
   temp.val = _Q6toIQ(x);
   return temp;
}

inline	iq	Q5toIQ(long x)
{
   iq temp;
   temp.val = _Q5toIQ(x);
   return temp;
}

inline	iq	Q4toIQ(long x)
{
   iq temp;
   temp.val = _Q4toIQ(x);
   return temp;
}

inline	iq	Q3toIQ(long x)
{
   iq temp;
   temp.val = _Q3toIQ(x);
   return temp;
}

inline	iq	Q2toIQ(long x)
{
   iq temp;
   temp.val = _Q2toIQ(x);
   return temp;
}

inline	iq	Q1toIQ(long x)
{
   iq temp;
   temp.val = _Q1toIQ(x);
   return temp;
}

//---------------------------------------------------------------------------
// Functions: atoIQ(A), atoIQN(A)
//---------------------------------------------------------------------------

inline	iq	atoIQ(const char *A)
{
   iq temp;
   temp.val = _atoIQ(A);
   return temp;
}

inline	iq30  atoIQ30(const char *A)
{
   iq30 temp;
   temp.val = _atoIQ30(A);
   return temp;
}

inline	iq29  atoIQ29(const char *A)
{
   iq29 temp;
   temp.val = _atoIQ29(A);
   return temp;
}

inline	iq28  atoIQ28(const char *A)
{
   iq28 temp;
   temp.val = _atoIQ28(A);
   return temp;
}

inline	iq27  atoIQ27(const char *A)
{
   iq27 temp;
   temp.val = _atoIQ27(A);
   return temp;
}

inline	iq26  atoIQ26(const char *A)
{
   iq26 temp;
   temp.val = _atoIQ26(A);
   return temp;
}

inline	iq25  atoIQ25(const char *A)
{
   iq25 temp;
   temp.val = _atoIQ25(A);
   return temp;
}

inline	iq24  atoIQ24(const char *A)
{
   iq24 temp;
   temp.val = _atoIQ24(A);
   return temp;
}

inline	iq23  atoIQ23(const char *A)
{
   iq23 temp;
   temp.val = _atoIQ23(A);
   return temp;
}

inline	iq22  atoIQ22(const char *A)
{
   iq22 temp;
   temp.val = _atoIQ22(A);
   return temp;
}

inline	iq21  atoIQ21(const char *A)
{
   iq21 temp;
   temp.val = _atoIQ21(A);
   return temp;
}

inline	iq20  atoIQ20(const char *A)
{
   iq20 temp;
   temp.val = _atoIQ20(A);
   return temp;
}

inline	iq19  atoIQ19(const char *A)
{
   iq19 temp;
   temp.val = _atoIQ19(A);
   return temp;
}

inline	iq18  atoIQ18(const char *A)
{
   iq18 temp;
   temp.val = _atoIQ18(A);
   return temp;
}

inline	iq17  atoIQ17(const char *A)
{
   iq17 temp;
   temp.val = _atoIQ17(A);
   return temp;
}

inline	iq16  atoIQ16(const char *A)
{
   iq16 temp;
   temp.val = _atoIQ16(A);
   return temp;
}

inline	iq15  atoIQ15(const char *A)
{
   iq15 temp;
   temp.val = _atoIQ15(A);
   return temp;
}

//---------------------------------------------------------------------------
// Functions: IQint(A), IQNint(A)
//---------------------------------------------------------------------------

inline	long	IQint(iq x)     { return _IQint(x.val);   }
inline	long	IQ30int(iq30 x) { return _IQ30int(x.val); }
inline	long	IQ29int(iq29 x) { return _IQ29int(x.val); }
inline	long	IQ28int(iq28 x) { return _IQ28int(x.val); }
inline	long	IQ27int(iq27 x) { return _IQ27int(x.val); }
inline	long	IQ26int(iq26 x) { return _IQ26int(x.val); }
inline	long	IQ25int(iq25 x) { return _IQ25int(x.val); }
inline	long	IQ24int(iq24 x) { return _IQ24int(x.val); }
inline	long	IQ23int(iq23 x) { return _IQ23int(x.val); }
inline	long	IQ22int(iq22 x) { return _IQ22int(x.val); }
inline	long	IQ21int(iq21 x) { return _IQ21int(x.val); }
inline	long	IQ20int(iq20 x) { return _IQ20int(x.val); }
inline	long	IQ19int(iq19 x) { return _IQ19int(x.val); }
inline	long	IQ18int(iq18 x) { return _IQ18int(x.val); }
inline	long	IQ17int(iq17 x) { return _IQ17int(x.val); }
inline	long	IQ16int(iq16 x) { return _IQ16int(x.val); }
inline	long	IQ15int(iq15 x) { return _IQ15int(x.val); }

//---------------------------------------------------------------------------
// Functions: IQfrac(A), IQNfrac(A)
//---------------------------------------------------------------------------

inline	iq	IQfrac(iq x)
{
   iq temp;
   temp.val = _IQfrac(x.val);
   return temp;
}

inline	iq30  IQ30frac(iq30 x)
{
   iq30 temp;
   temp.val = _IQ30frac(x.val);
   return temp;
}

inline	iq29  IQ29frac(iq29 x)
{
   iq29 temp;
   temp.val = _IQ29frac(x.val);
   return temp;
}

inline	iq28  IQ28frac(iq28 x)
{
   iq28 temp;
   temp.val = _IQ28frac(x.val);
   return temp;
}

inline	iq27  IQ27frac(iq27 x)
{
   iq27 temp;
   temp.val = _IQ27frac(x.val);
   return temp;
}

inline	iq26  IQ26frac(iq26 x)
{
   iq26 temp;
   temp.val = _IQ26frac(x.val);
   return temp;
}

inline	iq25  IQ25frac(iq25 x)
{
   iq25 temp;
   temp.val = _IQ25frac(x.val);
   return temp;
}

inline	iq24  IQ24frac(iq24 x)
{
   iq24 temp;
   temp.val = _IQ24frac(x.val);
   return temp;
}

inline	iq23  IQ23frac(iq23 x)
{
   iq23 temp;
   temp.val = _IQ23frac(x.val);
   return temp;
}

inline	iq22  IQ22frac(iq22 x)
{
   iq22 temp;
   temp.val = _IQ22frac(x.val);
   return temp;
}

inline	iq21  IQ21frac(iq21 x)
{
   iq21 temp;
   temp.val = _IQ21frac(x.val);
   return temp;
}

inline	iq20  IQ20frac(iq20 x)
{
   iq20 temp;
   temp.val = _IQ20frac(x.val);
   return temp;
}

inline	iq19  IQ19frac(iq19 x)
{
   iq19 temp;
   temp.val = _IQ19frac(x.val);
   return temp;
}

inline	iq18  IQ18frac(iq18 x)
{
   iq18 temp;
   temp.val = _IQ18frac(x.val);
   return temp;
}

inline	iq17  IQ17frac(iq17 x)
{
   iq17 temp;
   temp.val = _IQ17frac(x.val);
   return temp;
}

inline	iq16  IQ16frac(iq16 x)
{
   iq16 temp;
   temp.val = _IQ16frac(x.val);
   return temp;
}

inline	iq15  IQ15frac(iq15 x)
{
   iq15 temp;
   temp.val = _IQ15frac(x.val);
   return temp;
}

//---------------------------------------------------------------------------
// Functions: IQrmpy(A,B), IQNrmpy(A,B)
//---------------------------------------------------------------------------

inline	iq	IQrmpy(iq x, iq y)
{
   iq temp;
   temp.val = _IQrmpy(x.val, y.val);
   return temp;
}

inline	iq30	IQ30rmpy(iq30 x, iq30 y)
{
   iq30 temp;
   temp.val = _IQ30rmpy(x.val, y.val);
   return temp;
}

inline	iq29	IQ29rmpy(iq29 x, iq29 y)
{
   iq29 temp;
   temp.val = _IQ29rmpy(x.val, y.val);
   return temp;
}

inline	iq28	IQ28rmpy(iq28 x, iq28 y)
{
   iq28 temp;
   temp.val = _IQ28rmpy(x.val, y.val);
   return temp;
}

inline	iq27	IQ27rmpy(iq27 x, iq27 y)
{
   iq27 temp;
   temp.val = _IQ27rmpy(x.val, y.val);
   return temp;
}

inline	iq26	IQ26rmpy(iq26 x, iq26 y)
{
   iq26 temp;
   temp.val = _IQ26rmpy(x.val, y.val);
   return temp;
}

inline	iq25	IQ25rmpy(iq25 x, iq25 y)
{
   iq25 temp;
   temp.val = _IQ25rmpy(x.val, y.val);
   return temp;
}

inline	iq24	IQ24rmpy(iq24 x, iq24 y)
{
   iq24 temp;
   temp.val = _IQ24rmpy(x.val, y.val);
   return temp;
}

inline	iq23	IQ23rmpy(iq23 x, iq23 y)
{
   iq23 temp;
   temp.val = _IQ23rmpy(x.val, y.val);
   return temp;
}

inline	iq22	IQ22rmpy(iq22 x, iq22 y)
{
   iq22 temp;
   temp.val = _IQ22rmpy(x.val, y.val);
   return temp;
}

inline	iq21	IQ21rmpy(iq21 x, iq21 y)
{
   iq21 temp;
   temp.val = _IQ21rmpy(x.val, y.val);
   return temp;
}

inline	iq20	IQ20rmpy(iq20 x, iq20 y)
{
   iq20 temp;
   temp.val = _IQ20rmpy(x.val, y.val);
   return temp;
}

inline	iq19	IQ19rmpy(iq19 x, iq19 y)
{
   iq19 temp;
   temp.val = _IQ19rmpy(x.val, y.val);
   return temp;
}

inline	iq18	IQ18rmpy(iq18 x, iq18 y)
{
   iq18 temp;
   temp.val = _IQ18rmpy(x.val, y.val);
   return temp;
}

inline	iq17	IQ17rmpy(iq17 x, iq17 y)
{
   iq17 temp;
   temp.val = _IQ17rmpy(x.val, y.val);
   return temp;
}

inline	iq16	IQ16rmpy(iq16 x, iq16 y)
{
   iq16 temp;
   temp.val = _IQ16rmpy(x.val, y.val);
   return temp;
}

inline	iq15	IQ15rmpy(iq15 x, iq15 y)
{
   iq15 temp;
   temp.val = _IQ15rmpy(x.val, y.val);
   return temp;
}

//---------------------------------------------------------------------------
// Functions: IQrsmpy(A,B), IQNrsmpy(A,B)
//---------------------------------------------------------------------------

inline	iq	IQrsmpy(iq x, iq y)
{
   iq temp;
   temp.val = _IQrsmpy(x.val, y.val);
   return temp;
}

inline	iq30	IQ30rsmpy(iq30 x, iq30 y)
{
   iq30 temp;
   temp.val = _IQ30rsmpy(x.val, y.val);
   return temp;
}

inline	iq29	IQ29rsmpy(iq29 x, iq29 y)
{
   iq29 temp;
   temp.val = _IQ29rsmpy(x.val, y.val);
   return temp;
}

inline	iq28	IQ28rsmpy(iq28 x, iq28 y)
{
   iq28 temp;
   temp.val = _IQ28rsmpy(x.val, y.val);
   return temp;
}

inline	iq27	IQ27rsmpy(iq27 x, iq27 y)
{
   iq27 temp;
   temp.val = _IQ27rsmpy(x.val, y.val);
   return temp;
}

inline	iq26	IQ26rsmpy(iq26 x, iq26 y)
{
   iq26 temp;
   temp.val = _IQ26rsmpy(x.val, y.val);
   return temp;
}

inline	iq25	IQ25rsmpy(iq25 x, iq25 y)
{
   iq25 temp;
   temp.val = _IQ25rsmpy(x.val, y.val);
   return temp;
}

inline	iq24	IQ24rsmpy(iq24 x, iq24 y)
{
   iq24 temp;
   temp.val = _IQ24rsmpy(x.val, y.val);
   return temp;
}

inline	iq23	IQ23rsmpy(iq23 x, iq23 y)
{
   iq23 temp;
   temp.val = _IQ23rsmpy(x.val, y.val);
   return temp;
}

inline	iq22	IQ22rsmpy(iq22 x, iq22 y)
{
   iq22 temp;
   temp.val = _IQ22rsmpy(x.val, y.val);
   return temp;
}

inline	iq21	IQ21rsmpy(iq21 x, iq21 y)
{
   iq21 temp;
   temp.val = _IQ21rsmpy(x.val, y.val);
   return temp;
}

inline	iq20	IQ20rsmpy(iq20 x, iq20 y)
{
   iq20 temp;
   temp.val = _IQ20rsmpy(x.val, y.val);
   return temp;
}

inline	iq19	IQ19rsmpy(iq19 x, iq19 y)
{
   iq19 temp;
   temp.val = _IQ19rsmpy(x.val, y.val);
   return temp;
}

inline	iq18	IQ18rsmpy(iq18 x, iq18 y)
{
   iq18 temp;
   temp.val = _IQ18rsmpy(x.val, y.val);
   return temp;
}

inline	iq17	IQ17rsmpy(iq17 x, iq17 y)
{
   iq17 temp;
   temp.val = _IQ17rsmpy(x.val, y.val);
   return temp;
}

inline	iq16	IQ16rsmpy(iq16 x, iq16 y)
{
   iq16 temp;
   temp.val = _IQ16rsmpy(x.val, y.val);
   return temp;
}

inline	iq15	IQ15rsmpy(iq15 x, iq15 y)
{
   iq15 temp;
   temp.val = _IQ15rsmpy(x.val, y.val);
   return temp;
}

//---------------------------------------------------------------------------
// Functions: IQmpyIQX(A,IQA,B,IQB), IQNmpyIQX(A,IQA,B,IQB)
//---------------------------------------------------------------------------

#define  IQmpyIQX(A, IQA, B, IQB)   ((iq)   __IQxmpy(A.val, B.val, (GLOBAL_Q + 32 - IQA - IQB)))
#define  IQ30mpyIQX(A, IQA, B, IQB) ((iq30) __IQxmpy(A.val, B.val, (30 + 32 - IQA - IQB)))
#define  IQ29mpyIQX(A, IQA, B, IQB) ((iq29) __IQxmpy(A.val, B.val, (29 + 32 - IQA - IQB)))
#define  IQ28mpyIQX(A, IQA, B, IQB) ((iq28) __IQxmpy(A.val, B.val, (28 + 32 - IQA - IQB)))
#define  IQ27mpyIQX(A, IQA, B, IQB) ((iq27) __IQxmpy(A.val, B.val, (27 + 32 - IQA - IQB)))
#define  IQ26mpyIQX(A, IQA, B, IQB) ((iq26) __IQxmpy(A.val, B.val, (26 + 32 - IQA - IQB)))
#define  IQ25mpyIQX(A, IQA, B, IQB) ((iq25) __IQxmpy(A.val, B.val, (25 + 32 - IQA - IQB)))
#define  IQ24mpyIQX(A, IQA, B, IQB) ((iq24) __IQxmpy(A.val, B.val, (24 + 32 - IQA - IQB)))
#define  IQ23mpyIQX(A, IQA, B, IQB) ((iq23) __IQxmpy(A.val, B.val, (23 + 32 - IQA - IQB)))
#define  IQ22mpyIQX(A, IQA, B, IQB) ((iq22) __IQxmpy(A.val, B.val, (22 + 32 - IQA - IQB)))
#define  IQ21mpyIQX(A, IQA, B, IQB) ((iq21) __IQxmpy(A.val, B.val, (21 + 32 - IQA - IQB)))
#define  IQ20mpyIQX(A, IQA, B, IQB) ((iq20) __IQxmpy(A.val, B.val, (20 + 32 - IQA - IQB)))
#define  IQ19mpyIQX(A, IQA, B, IQB) ((iq19) __IQxmpy(A.val, B.val, (19 + 32 - IQA - IQB)))
#define  IQ18mpyIQX(A, IQA, B, IQB) ((iq18) __IQxmpy(A.val, B.val, (18 + 32 - IQA - IQB)))
#define  IQ17mpyIQX(A, IQA, B, IQB) ((iq17) __IQxmpy(A.val, B.val, (17 + 32 - IQA - IQB)))
#define  IQ16mpyIQX(A, IQA, B, IQB) ((iq16) __IQxmpy(A.val, B.val, (16 + 32 - IQA - IQB)))
#define  IQ15mpyIQX(A, IQA, B, IQB) ((iq15) __IQxmpy(A.val, B.val, (15 + 32 - IQA - IQB)))

//---------------------------------------------------------------------------
// Functions: IQmpyI32(A,B), IQNmpyI32(A,B)
//---------------------------------------------------------------------------

inline	iq	IQmpyI32(iq y, long x)
{
   iq temp;
   temp.val = (y.val * x);
   return temp;
}

inline	iq 	IQmpyI32(long y, iq x)
{
   iq temp;
   temp.val = (y * x.val);
   return temp;
}

inline	iq30	IQ30mpyI32(iq30 y, long x)
{
   iq30 temp;
   temp.val = (y.val * x);
   return temp;
}

inline	iq30 	IQ30mpyI32(long y, iq30 x)
{
   iq30 temp;
   temp.val = (y * x.val);
   return temp;
}

inline	iq29	IQ29mpyI32(iq29 y, long x)
{
   iq29 temp;
   temp.val = (y.val * x);
   return temp;
}

inline	iq29 	IQ29mpyI32(long y, iq29 x)
{
   iq29 temp;
   temp.val = (y * x.val);
   return temp;
}

inline	iq28	IQ28mpyI32(iq28 y, long x)
{
   iq28 temp;
   temp.val = (y.val * x);
   return temp;
}

inline	iq28 	IQ28mpyI32(long y, iq28 x)
{
   iq28 temp;
   temp.val = (y * x.val);
   return temp;
}

inline	iq27	IQ27mpyI32(iq27 y, long x)
{
   iq27 temp;
   temp.val = (y.val * x);
   return temp;
}

inline	iq27 	IQ27mpyI32(long y, iq27 x)
{
   iq27 temp;
   temp.val = (y * x.val);
   return temp;
}

inline	iq26	IQ26mpyI32(iq26 y, long x)
{
   iq26 temp;
   temp.val = (y.val * x);
   return temp;
}

inline	iq26 	IQ26mpyI32(long y, iq26 x)
{
   iq26 temp;
   temp.val = (y * x.val);
   return temp;
}

inline	iq25	IQ25mpyI32(iq25 y, long x)
{
   iq25 temp;
   temp.val = (y.val * x);
   return temp;
}

inline	iq25 	IQ25mpyI32(long y, iq25 x)
{
   iq25 temp;
   temp.val = (y * x.val);
   return temp;
}

inline	iq24	IQ24mpyI32(iq24 y, long x)
{
   iq24 temp;
   temp.val = (y.val * x);
   return temp;
}

inline	iq24 	IQ24mpyI32(long y, iq24 x)
{
   iq24 temp;
   temp.val = (y * x.val);
   return temp;
}

inline	iq23	IQ23mpyI32(iq23 y, long x)
{
   iq23 temp;
   temp.val = (y.val * x);
   return temp;
}

inline	iq23 	IQ23mpyI32(long y, iq23 x)
{
   iq23 temp;
   temp.val = (y * x.val);
   return temp;
}

inline	iq22	IQ22mpyI32(iq22 y, long x)
{
   iq22 temp;
   temp.val = (y.val * x);
   return temp;
}

inline	iq22 	IQ22mpyI32(long y, iq22 x)
{
   iq22 temp;
   temp.val = (y * x.val);
   return temp;
}

inline	iq21	IQ21mpyI32(iq21 y, long x)
{
   iq21 temp;
   temp.val = (y.val * x);
   return temp;
}

inline	iq21 	IQ21mpyI32(long y, iq21 x)
{
   iq21 temp;
   temp.val = (y * x.val);
   return temp;
}

inline	iq20	IQ20mpyI32(iq20 y, long x)
{
   iq20 temp;
   temp.val = (y.val * x);
   return temp;
}

inline	iq20 	IQ20mpyI32(long y, iq20 x)
{
   iq20 temp;
   temp.val = (y * x.val);
   return temp;
}

inline	iq19	IQ19mpyI32(iq19 y, long x)
{
   iq19 temp;
   temp.val = (y.val * x);
   return temp;
}

inline	iq19 	IQ19mpyI32(long y, iq19 x)
{
   iq19 temp;
   temp.val = (y * x.val);
   return temp;
}

inline	iq18	IQ18mpyI32(iq18 y, long x)
{
   iq18 temp;
   temp.val = (y.val * x);
   return temp;
}

inline	iq18 	IQ18mpyI32(long y, iq18 x)
{
   iq18 temp;
   temp.val = (y * x.val);
   return temp;
}

inline	iq17	IQ17mpyI32(iq17 y, long x)
{
   iq17 temp;
   temp.val = (y.val * x);
   return temp;
}

inline	iq17 	IQ17mpyI32(long y, iq17 x)
{
   iq17 temp;
   temp.val = (y * x.val);
   return temp;
}

inline	iq16	IQ16mpyI32(iq16 y, long x)
{
   iq16 temp;
   temp.val = (y.val * x);
   return temp;
}

inline	iq16 	IQ16mpyI32(long y, iq16 x)
{
   iq16 temp;
   temp.val = (y * x.val);
   return temp;
}

inline	iq15	IQ15mpyI32(iq15 y, long x)
{
   iq15 temp;
   temp.val = (y.val * x);
   return temp;
}

inline	iq15 	IQ15mpyI32(long y, iq15 x)
{
   iq15 temp;
   temp.val = (y * x.val);
   return temp;
}

//---------------------------------------------------------------------------
// Functions: IQmpyI32int(A,B), IQNmpyI32int(A,B)
//---------------------------------------------------------------------------

inline	long	IQmpyI32int(iq y, long x) { return _IQmpyI32int(y.val, x); }
inline	long	IQmpyI32int(long y, iq x) { return _IQmpyI32int(y, x.val); }
inline	long	IQ30mpyI32int(iq30 y, long x) { return _IQ30mpyI32int(y.val, x); }
inline	long	IQ30mpyI32int(long y, iq30 x) { return _IQ30mpyI32int(y, x.val); }
inline	long	IQ29mpyI32int(iq29 y, long x) { return _IQ29mpyI32int(y.val, x); }
inline	long	IQ29mpyI32int(long y, iq29 x) { return _IQ29mpyI32int(y, x.val); }
inline	long	IQ28mpyI32int(iq28 y, long x) { return _IQ28mpyI32int(y.val, x); }
inline	long	IQ28mpyI32int(long y, iq28 x) { return _IQ28mpyI32int(y, x.val); }
inline	long	IQ27mpyI32int(iq27 y, long x) { return _IQ27mpyI32int(y.val, x); }
inline	long	IQ27mpyI32int(long y, iq27 x) { return _IQ27mpyI32int(y, x.val); }
inline	long	IQ26mpyI32int(iq26 y, long x) { return _IQ26mpyI32int(y.val, x); }
inline	long	IQ26mpyI32int(long y, iq26 x) { return _IQ26mpyI32int(y, x.val); }
inline	long	IQ25mpyI32int(iq25 y, long x) { return _IQ25mpyI32int(y.val, x); }
inline	long	IQ25mpyI32int(long y, iq25 x) { return _IQ25mpyI32int(y, x.val); }
inline	long	IQ24mpyI32int(iq24 y, long x) { return _IQ24mpyI32int(y.val, x); }
inline	long	IQ24mpyI32int(long y, iq24 x) { return _IQ24mpyI32int(y, x.val); }
inline	long	IQ23mpyI32int(iq23 y, long x) { return _IQ23mpyI32int(y.val, x); }
inline	long	IQ23mpyI32int(long y, iq23 x) { return _IQ23mpyI32int(y, x.val); }
inline	long	IQ22mpyI32int(iq22 y, long x) { return _IQ22mpyI32int(y.val, x); }
inline	long	IQ22mpyI32int(long y, iq22 x) { return _IQ22mpyI32int(y, x.val); }
inline	long	IQ21mpyI32int(iq21 y, long x) { return _IQ21mpyI32int(y.val, x); }
inline	long	IQ21mpyI32int(long y, iq21 x) { return _IQ21mpyI32int(y, x.val); }
inline	long	IQ20mpyI32int(iq20 y, long x) { return _IQ20mpyI32int(y.val, x); }
inline	long	IQ20mpyI32int(long y, iq20 x) { return _IQ20mpyI32int(y, x.val); }
inline	long	IQ19mpyI32int(iq19 y, long x) { return _IQ19mpyI32int(y.val, x); }
inline	long	IQ19mpyI32int(long y, iq19 x) { return _IQ19mpyI32int(y, x.val); }
inline	long	IQ18mpyI32int(iq18 y, long x) { return _IQ18mpyI32int(y.val, x); }
inline	long	IQ18mpyI32int(long y, iq18 x) { return _IQ18mpyI32int(y, x.val); }
inline	long	IQ17mpyI32int(iq17 y, long x) { return _IQ17mpyI32int(y.val, x); }
inline	long	IQ17mpyI32int(long y, iq17 x) { return _IQ17mpyI32int(y, x.val); }
inline	long	IQ16mpyI32int(iq16 y, long x) { return _IQ16mpyI32int(y.val, x); }
inline	long	IQ16mpyI32int(long y, iq16 x) { return _IQ16mpyI32int(y, x.val); }
inline	long	IQ15mpyI32int(iq15 y, long x) { return _IQ15mpyI32int(y.val, x); }
inline	long	IQ15mpyI32int(long y, iq15 x) { return _IQ15mpyI32int(y, x.val); }

//---------------------------------------------------------------------------
// Functions: IQmpyI32frac(A,B), IQNmpyI32frac(A,B)
//---------------------------------------------------------------------------

inline	iq	IQmpyI32frac(iq y, long x)
{
   iq temp;
   temp.val = _IQmpyI32frac(y.val, x);
   return temp;
}

inline	iq 	IQmpyI32frac(long y, iq x)
{
   iq temp;
   temp.val = _IQmpyI32frac(y, x.val);
   return temp;
}

inline	iq30	IQ30mpyI32frac(iq30 y, long x)
{
   iq30 temp;
   temp.val = _IQ30mpyI32frac(y.val, x);
   return temp;
}

inline	iq30 	IQ30mpyI32frac(long y, iq30 x)
{
   iq30 temp;
   temp.val = _IQ30mpyI32frac(y, x.val);
   return temp;
}

inline	iq29	IQ29mpyI32frac(iq29 y, long x)
{
   iq29 temp;
   temp.val = _IQ29mpyI32frac(y.val, x);
   return temp;
}

inline	iq29 	IQ29mpyI32frac(long y, iq29 x)
{
   iq29 temp;
   temp.val = _IQ29mpyI32frac(y, x.val);
   return temp;
}

inline	iq28	IQ28mpyI32frac(iq28 y, long x)
{
   iq28 temp;
   temp.val = _IQ28mpyI32frac(y.val, x);
   return temp;
}

inline	iq28 	IQ28mpyI32frac(long y, iq28 x)
{
   iq28 temp;
   temp.val = _IQ28mpyI32frac(y, x.val);
   return temp;
}

inline	iq27	IQ27mpyI32frac(iq27 y, long x)
{
   iq27 temp;
   temp.val = _IQ27mpyI32frac(y.val, x);
   return temp;
}

inline	iq27 	IQ27mpyI32frac(long y, iq27 x)
{
   iq27 temp;
   temp.val = _IQ27mpyI32frac(y, x.val);
   return temp;
}

inline	iq26	IQ26mpyI32frac(iq26 y, long x)
{
   iq26 temp;
   temp.val = _IQ26mpyI32frac(y.val, x);
   return temp;
}

inline	iq26 	IQ26mpyI32frac(long y, iq26 x)
{
   iq26 temp;
   temp.val = _IQ26mpyI32frac(y, x.val);
   return temp;
}

inline	iq25	IQ25mpyI32frac(iq25 y, long x)
{
   iq25 temp;
   temp.val = _IQ25mpyI32frac(y.val, x);
   return temp;
}

inline	iq25 	IQ25mpyI32frac(long y, iq25 x)
{
   iq25 temp;
   temp.val = _IQ25mpyI32frac(y, x.val);
   return temp;
}

inline	iq24	IQ24mpyI32frac(iq24 y, long x)
{
   iq24 temp;
   temp.val = _IQ24mpyI32frac(y.val, x);
   return temp;
}

inline	iq24 	IQ24mpyI32frac(long y, iq24 x)
{
   iq24 temp;
   temp.val = _IQ24mpyI32frac(y, x.val);
   return temp;
}

inline	iq23	IQ23mpyI32frac(iq23 y, long x)
{
   iq23 temp;
   temp.val = _IQ23mpyI32frac(y.val, x);
   return temp;
}

inline	iq23 	IQ23mpyI32frac(long y, iq23 x)
{
   iq23 temp;
   temp.val = _IQ23mpyI32frac(y, x.val);
   return temp;
}

inline	iq22	IQ22mpyI32frac(iq22 y, long x)
{
   iq22 temp;
   temp.val = _IQ22mpyI32frac(y.val, x);
   return temp;
}

inline	iq22 	IQ22mpyI32frac(long y, iq22 x)
{
   iq22 temp;
   temp.val = _IQ22mpyI32frac(y, x.val);
   return temp;
}

inline	iq21	IQ21mpyI32frac(iq21 y, long x)
{
   iq21 temp;
   temp.val = _IQ21mpyI32frac(y.val, x);
   return temp;
}

inline	iq21 	IQ21mpyI32frac(long y, iq21 x)
{
   iq21 temp;
   temp.val = _IQ21mpyI32frac(y, x.val);
   return temp;
}

inline	iq20	IQ20mpyI32frac(iq20 y, long x)
{
   iq20 temp;
   temp.val = _IQ20mpyI32frac(y.val, x);
   return temp;
}

inline	iq20 	IQ20mpyI32frac(long y, iq20 x)
{
   iq20 temp;
   temp.val = _IQ20mpyI32frac(y, x.val);
   return temp;
}

inline	iq19	IQ19mpyI32frac(iq19 y, long x)
{
   iq19 temp;
   temp.val = _IQ19mpyI32frac(y.val, x);
   return temp;
}

inline	iq19 	IQ19mpyI32frac(long y, iq19 x)
{
   iq19 temp;
   temp.val = _IQ19mpyI32frac(y, x.val);
   return temp;
}

inline	iq18	IQ18mpyI32frac(iq18 y, long x)
{
   iq18 temp;
   temp.val = _IQ18mpyI32frac(y.val, x);
   return temp;
}

inline	iq18 	IQ18mpyI32frac(long y, iq18 x)
{
   iq18 temp;
   temp.val = _IQ18mpyI32frac(y, x.val);
   return temp;
}

inline	iq17	IQ17mpyI32frac(iq17 y, long x)
{
   iq17 temp;
   temp.val = _IQ17mpyI32frac(y.val, x);
   return temp;
}

inline	iq17 	IQ17mpyI32frac(long y, iq17 x)
{
   iq17 temp;
   temp.val = _IQ17mpyI32frac(y, x.val);
   return temp;
}

inline	iq16	IQ16mpyI32frac(iq16 y, long x)
{
   iq16 temp;
   temp.val = _IQ16mpyI32frac(y.val, x);
   return temp;
}

inline	iq16 	IQ16mpyI32frac(long y, iq16 x)
{
   iq16 temp;
   temp.val = _IQ16mpyI32frac(y, x.val);
   return temp;
}

inline	iq15	IQ15mpyI32frac(iq15 y, long x)
{
   iq15 temp;
   temp.val = _IQ15mpyI32frac(y.val, x);
   return temp;
}

inline	iq15 	IQ15mpyI32frac(long y, iq15 x)
{
   iq15 temp;
   temp.val = _IQ15mpyI32frac(y, x.val);
   return temp;
}

//---------------------------------------------------------------------------
// Functions: IQsin(A), IQNsin(A)
//---------------------------------------------------------------------------

inline	iq	IQsin(iq x)
{
   iq temp;
   temp.val = _IQsin(x.val);
   return temp;
}

inline	iq30	IQ30sin(iq30 x)
{
   iq30 temp;
   temp.val = _IQ30sin(x.val);
   return temp;
}

inline	iq29	IQ29sin(iq29 x)
{
   iq29 temp;
   temp.val = _IQ29sin(x.val);
   return temp;
}

inline	iq28	IQ28sin(iq28 x)
{
   iq28 temp;
   temp.val = _IQ28sin(x.val);
   return temp;
}

inline	iq27	IQ27sin(iq27 x)
{
   iq27 temp;
   temp.val = _IQ27sin(x.val);
   return temp;
}

inline	iq26	IQ26sin(iq26 x)
{
   iq26 temp;
   temp.val = _IQ26sin(x.val);
   return temp;
}

inline	iq25	IQ25sin(iq25 x)
{
   iq25 temp;
   temp.val = _IQ25sin(x.val);
   return temp;
}

inline	iq24	IQ24sin(iq24 x)
{
   iq24 temp;
   temp.val = _IQ24sin(x.val);
   return temp;
}

inline	iq23	IQ23sin(iq23 x)
{
   iq23 temp;
   temp.val = _IQ23sin(x.val);
   return temp;
}

inline	iq22	IQ22sin(iq22 x)
{
   iq22 temp;
   temp.val = _IQ22sin(x.val);
   return temp;
}

inline	iq21	IQ21sin(iq21 x)
{
   iq21 temp;
   temp.val = _IQ21sin(x.val);
   return temp;
}

inline	iq20	IQ20sin(iq20 x)
{
   iq20 temp;
   temp.val = _IQ20sin(x.val);
   return temp;
}

inline	iq19	IQ19sin(iq19 x)
{
   iq19 temp;
   temp.val = _IQ19sin(x.val);
   return temp;
}

inline	iq18	IQ18sin(iq18 x)
{
   iq18 temp;
   temp.val = _IQ18sin(x.val);
   return temp;
}

inline	iq17	IQ17sin(iq17 x)
{
   iq17 temp;
   temp.val = _IQ17sin(x.val);
   return temp;
}

inline	iq16	IQ16sin(iq16 x)
{
   iq16 temp;
   temp.val = _IQ16sin(x.val);
   return temp;
}

inline	iq15	IQ15sin(iq15 x)
{
   iq15 temp;
   temp.val = _IQ15sin(x.val);
   return temp;
}

//---------------------------------------------------------------------------
// Functions: IQsinPU(A), IQNsinPU(A)
//---------------------------------------------------------------------------

inline	iq	IQsinPU(iq x)
{
   iq temp;
   temp.val = _IQsinPU(x.val);
   return temp;
}

inline	iq30	IQ30sinPU(iq30 x)
{
   iq30 temp;
   temp.val = _IQ30sinPU(x.val);
   return temp;
}

inline	iq29	IQ29sinPU(iq29 x)
{
   iq29 temp;
   temp.val = _IQ29sinPU(x.val);
   return temp;
}

inline	iq28	IQ28sinPU(iq28 x)
{
   iq28 temp;
   temp.val = _IQ28sinPU(x.val);
   return temp;
}

inline	iq27	IQ27sinPU(iq27 x)
{
   iq27 temp;
   temp.val = _IQ27sinPU(x.val);
   return temp;
}

inline	iq26	IQ26sinPU(iq26 x)
{
   iq26 temp;
   temp.val = _IQ26sinPU(x.val);
   return temp;
}

inline	iq25	IQ25sinPU(iq25 x)
{
   iq25 temp;
   temp.val = _IQ25sinPU(x.val);
   return temp;
}

inline	iq24	IQ24sinPU(iq24 x)
{
   iq24 temp;
   temp.val = _IQ24sinPU(x.val);
   return temp;
}

inline	iq23	IQ23sinPU(iq23 x)
{
   iq23 temp;
   temp.val = _IQ23sinPU(x.val);
   return temp;
}

inline	iq22	IQ22sinPU(iq22 x)
{
   iq22 temp;
   temp.val = _IQ22sinPU(x.val);
   return temp;
}

inline	iq21	IQ21sinPU(iq21 x)
{
   iq21 temp;
   temp.val = _IQ21sinPU(x.val);
   return temp;
}

inline	iq20	IQ20sinPU(iq20 x)
{
   iq20 temp;
   temp.val = _IQ20sinPU(x.val);
   return temp;
}

inline	iq19	IQ19sinPU(iq19 x)
{
   iq19 temp;
   temp.val = _IQ19sinPU(x.val);
   return temp;
}

inline	iq18	IQ18sinPU(iq18 x)
{
   iq18 temp;
   temp.val = _IQ18sinPU(x.val);
   return temp;
}

inline	iq17	IQ17sinPU(iq17 x)
{
   iq17 temp;
   temp.val = _IQ17sinPU(x.val);
   return temp;
}

inline	iq16	IQ16sinPU(iq16 x)
{
   iq16 temp;
   temp.val = _IQ16sinPU(x.val);
   return temp;
}

inline	iq15	IQ15sinPU(iq15 x)
{
   iq15 temp;
   temp.val = _IQ15sinPU(x.val);
   return temp;
}

//---------------------------------------------------------------------------
// Functions: IQcos(A), IQNcos(A)
//---------------------------------------------------------------------------

inline	iq	IQcos(iq x)
{
   iq temp;
   temp.val = _IQcos(x.val);
   return temp;
}

inline	iq30	IQ30cos(iq30 x)
{
   iq30 temp;
   temp.val = _IQ30cos(x.val);
   return temp;
}

inline	iq29	IQ29cos(iq29 x)
{
   iq29 temp;
   temp.val = _IQ29cos(x.val);
   return temp;
}

inline	iq28	IQ28cos(iq28 x)
{
   iq28 temp;
   temp.val = _IQ28cos(x.val);
   return temp;
}

inline	iq27	IQ27cos(iq27 x)
{
   iq27 temp;
   temp.val = _IQ27cos(x.val);
   return temp;
}

inline	iq26	IQ26cos(iq26 x)
{
   iq26 temp;
   temp.val = _IQ26cos(x.val);
   return temp;
}

inline	iq25	IQ25cos(iq25 x)
{
   iq25 temp;
   temp.val = _IQ25cos(x.val);
   return temp;
}

inline	iq24	IQ24cos(iq24 x)
{
   iq24 temp;
   temp.val = _IQ24cos(x.val);
   return temp;
}

inline	iq23	IQ23cos(iq23 x)
{
   iq23 temp;
   temp.val = _IQ23cos(x.val);
   return temp;
}

inline	iq22	IQ22cos(iq22 x)
{
   iq22 temp;
   temp.val = _IQ22cos(x.val);
   return temp;
}

inline	iq21	IQ21cos(iq21 x)
{
   iq21 temp;
   temp.val = _IQ21cos(x.val);
   return temp;
}

inline	iq20	IQ20cos(iq20 x)
{
   iq20 temp;
   temp.val = _IQ20cos(x.val);
   return temp;
}

inline	iq19	IQ19cos(iq19 x)
{
   iq19 temp;
   temp.val = _IQ19cos(x.val);
   return temp;
}

inline	iq18	IQ18cos(iq18 x)
{
   iq18 temp;
   temp.val = _IQ18cos(x.val);
   return temp;
}

inline	iq17	IQ17cos(iq17 x)
{
   iq17 temp;
   temp.val = _IQ17cos(x.val);
   return temp;
}

inline	iq16	IQ16cos(iq16 x)
{
   iq16 temp;
   temp.val = _IQ16cos(x.val);
   return temp;
}

inline	iq15	IQ15cos(iq15 x)
{
   iq15 temp;
   temp.val = _IQ15cos(x.val);
   return temp;
}

//---------------------------------------------------------------------------
// Functions: IQcosPU(A), IQNcosPU(A)
//---------------------------------------------------------------------------

inline	iq	IQcosPU(iq x)
{
   iq temp;
   temp.val = _IQcosPU(x.val);
   return temp;
}

inline	iq30	IQ30cosPU(iq30 x)
{
   iq30 temp;
   temp.val = _IQ30cosPU(x.val);
   return temp;
}

inline	iq29	IQ29cosPU(iq29 x)
{
   iq29 temp;
   temp.val = _IQ29cosPU(x.val);
   return temp;
}

inline	iq28	IQ28cosPU(iq28 x)
{
   iq28 temp;
   temp.val = _IQ28cosPU(x.val);
   return temp;
}

inline	iq27	IQ27cosPU(iq27 x)
{
   iq27 temp;
   temp.val = _IQ27cosPU(x.val);
   return temp;
}

inline	iq26	IQ26cosPU(iq26 x)
{
   iq26 temp;
   temp.val = _IQ26cosPU(x.val);
   return temp;
}

inline	iq25	IQ25cosPU(iq25 x)
{
   iq25 temp;
   temp.val = _IQ25cosPU(x.val);
   return temp;
}

inline	iq24	IQ24cosPU(iq24 x)
{
   iq24 temp;
   temp.val = _IQ24cosPU(x.val);
   return temp;
}

inline	iq23	IQ23cosPU(iq23 x)
{
   iq23 temp;
   temp.val = _IQ23cosPU(x.val);
   return temp;
}

inline	iq22	IQ22cosPU(iq22 x)
{
   iq22 temp;
   temp.val = _IQ22cosPU(x.val);
   return temp;
}

inline	iq21	IQ21cosPU(iq21 x)
{
   iq21 temp;
   temp.val = _IQ21cosPU(x.val);
   return temp;
}

inline	iq20	IQ20cosPU(iq20 x)
{
   iq20 temp;
   temp.val = _IQ20cosPU(x.val);
   return temp;
}

inline	iq19	IQ19cosPU(iq19 x)
{
   iq19 temp;
   temp.val = _IQ19cosPU(x.val);
   return temp;
}

inline	iq18	IQ18cosPU(iq18 x)
{
   iq18 temp;
   temp.val = _IQ18cosPU(x.val);
   return temp;
}

inline	iq17	IQ17cosPU(iq17 x)
{
   iq17 temp;
   temp.val = _IQ17cosPU(x.val);
   return temp;
}

inline	iq16	IQ16cosPU(iq16 x)
{
   iq16 temp;
   temp.val = _IQ16cosPU(x.val);
   return temp;
}

inline	iq15	IQ15cosPU(iq15 x)
{
   iq15 temp;
   temp.val = _IQ15cosPU(x.val);
   return temp;
}

//---------------------------------------------------------------------------
// Functions: IQatan2(A,B), IQNatan2(A,B)
//---------------------------------------------------------------------------

inline	iq	IQatan2(iq y, iq x)
{
   iq temp;
   temp.val = _IQatan2(y.val, x.val);
   return temp;
}

inline	iq30	IQ30atan2(iq30 y, iq30 x)
{
   iq30 temp;
   temp.val = _IQ30atan2(y.val, x.val);
   return temp;
}

inline	iq29	IQ29atan2(iq29 y, iq29 x)
{
   iq29 temp;
   temp.val = _IQ29atan2(y.val, x.val);
   return temp;
}

inline	iq28	IQ28atan2(iq28 y, iq28 x)
{
   iq28 temp;
   temp.val = _IQ28atan2(y.val, x.val);
   return temp;
}

inline	iq27	IQ27atan2(iq27 y, iq27 x)
{
   iq27 temp;
   temp.val = _IQ27atan2(y.val, x.val);
   return temp;
}

inline	iq26	IQ26atan2(iq26 y, iq26 x)
{
   iq26 temp;
   temp.val = _IQ26atan2(y.val, x.val);
   return temp;
}

inline	iq25	IQ25atan2(iq25 y, iq25 x)
{
   iq25 temp;
   temp.val = _IQ25atan2(y.val, x.val);
   return temp;
}

inline	iq24	IQ24atan2(iq24 y, iq24 x)
{
   iq24 temp;
   temp.val = _IQ24atan2(y.val, x.val);
   return temp;
}

inline	iq23	IQ23atan2(iq23 y, iq23 x)
{
   iq23 temp;
   temp.val = _IQ23atan2(y.val, x.val);
   return temp;
}

inline	iq22	IQ22atan2(iq22 y, iq22 x)
{
   iq22 temp;
   temp.val = _IQ22atan2(y.val, x.val);
   return temp;
}

inline	iq21	IQ21atan2(iq21 y, iq21 x)
{
   iq21 temp;
   temp.val = _IQ21atan2(y.val, x.val);
   return temp;
}

inline	iq20	IQ20atan2(iq20 y, iq20 x)
{
   iq20 temp;
   temp.val = _IQ20atan2(y.val, x.val);
   return temp;
}

inline	iq19	IQ19atan2(iq19 y, iq19 x)
{
   iq19 temp;
   temp.val = _IQ19atan2(y.val, x.val);
   return temp;
}

inline	iq18	IQ18atan2(iq18 y, iq18 x)
{
   iq18 temp;
   temp.val = _IQ18atan2(y.val, x.val);
   return temp;
}

inline	iq17	IQ17atan2(iq17 y, iq17 x)
{
   iq17 temp;
   temp.val = _IQ17atan2(y.val, x.val);
   return temp;
}

inline	iq16	IQ16atan2(iq16 y, iq16 x)
{
   iq16 temp;
   temp.val = _IQ16atan2(y.val, x.val);
   return temp;
}

inline	iq15	IQ15atan2(iq15 y, iq15 x)
{
   iq15 temp;
   temp.val = _IQ15atan2(y.val, x.val);
   return temp;
}

//---------------------------------------------------------------------------
// Functions: IQatan2PU(A,B), IQNatan2PU(A,B)
//---------------------------------------------------------------------------

inline	iq	IQatan2PU(iq y, iq x)
{
   iq temp;
   temp.val = _IQatan2PU(y.val, x.val);
   return temp;
}

inline	iq30	IQ30atan2PU(iq30 y, iq30 x)
{
   iq30 temp;
   temp.val = _IQ30atan2PU(y.val, x.val);
   return temp;
}

inline	iq29	IQ29atan2PU(iq29 y, iq29 x)
{
   iq29 temp;
   temp.val = _IQ29atan2PU(y.val, x.val);
   return temp;
}

inline	iq28	IQ28atan2PU(iq28 y, iq28 x)
{
   iq28 temp;
   temp.val = _IQ28atan2PU(y.val, x.val);
   return temp;
}

inline	iq27	IQ27atan2PU(iq27 y, iq27 x)
{
   iq27 temp;
   temp.val = _IQ27atan2PU(y.val, x.val);
   return temp;
}

inline	iq26	IQ26atan2PU(iq26 y, iq26 x)
{
   iq26 temp;
   temp.val = _IQ26atan2PU(y.val, x.val);
   return temp;
}

inline	iq25	IQ25atan2PU(iq25 y, iq25 x)
{
   iq25 temp;
   temp.val = _IQ25atan2PU(y.val, x.val);
   return temp;
}

inline	iq24	IQ24atan2PU(iq24 y, iq24 x)
{
   iq24 temp;
   temp.val = _IQ24atan2PU(y.val, x.val);
   return temp;
}

inline	iq23	IQ23atan2PU(iq23 y, iq23 x)
{
   iq23 temp;
   temp.val = _IQ23atan2PU(y.val, x.val);
   return temp;
}

inline	iq22	IQ22atan2PU(iq22 y, iq22 x)
{
   iq22 temp;
   temp.val = _IQ22atan2PU(y.val, x.val);
   return temp;
}

inline	iq21	IQ21atan2PU(iq21 y, iq21 x)
{
   iq21 temp;
   temp.val = _IQ21atan2PU(y.val, x.val);
   return temp;
}

inline	iq20	IQ20atan2PU(iq20 y, iq20 x)
{
   iq20 temp;
   temp.val = _IQ20atan2PU(y.val, x.val);
   return temp;
}

inline	iq19	IQ19atan2PU(iq19 y, iq19 x)
{
   iq19 temp;
   temp.val = _IQ19atan2PU(y.val, x.val);
   return temp;
}

inline	iq18	IQ18atan2PU(iq18 y, iq18 x)
{
   iq18 temp;
   temp.val = _IQ18atan2PU(y.val, x.val);
   return temp;
}

inline	iq17	IQ17atan2PU(iq17 y, iq17 x)
{
   iq17 temp;
   temp.val = _IQ17atan2PU(y.val, x.val);
   return temp;
}

inline	iq16	IQ16atan2PU(iq16 y, iq16 x)
{
   iq16 temp;
   temp.val = _IQ16atan2PU(y.val, x.val);
   return temp;
}

inline	iq15	IQ15atan2PU(iq15 y, iq15 x)
{
   iq15 temp;
   temp.val = _IQ15atan2PU(y.val, x.val);
   return temp;
}

//---------------------------------------------------------------------------
// Functions: IQatan(A), IQNatan(A)
//---------------------------------------------------------------------------

#define  IQatan(A)    IQatan2(A,IQ(1.0))
#define  IQ30atan(A)  IQ30atan2(A,IQ30(1.0))
#define  IQ29atan(A)  IQ29atan2(A,IQ29(1.0))
#define  IQ28atan(A)  IQ28atan2(A,IQ28(1.0))
#define  IQ27atan(A)  IQ27atan2(A,IQ27(1.0))
#define  IQ26atan(A)  IQ26atan2(A,IQ26(1.0))
#define  IQ25atan(A)  IQ25atan2(A,IQ25(1.0))
#define  IQ24atan(A)  IQ24atan2(A,IQ24(1.0))
#define  IQ23atan(A)  IQ23atan2(A,IQ23(1.0))
#define  IQ22atan(A)  IQ22atan2(A,IQ22(1.0))
#define  IQ21atan(A)  IQ21atan2(A,IQ21(1.0))
#define  IQ20atan(A)  IQ20atan2(A,IQ20(1.0))
#define  IQ19atan(A)  IQ19atan2(A,IQ19(1.0))
#define  IQ18atan(A)  IQ18atan2(A,IQ18(1.0))
#define  IQ17atan(A)  IQ17atan2(A,IQ17(1.0))
#define  IQ16atan(A)  IQ16atan2(A,IQ16(1.0))
#define  IQ15atan(A)  IQ15atan2(A,IQ15(1.0))

//---------------------------------------------------------------------------
// Functions: IQsqrt(A), IQNsqrt(A)
//---------------------------------------------------------------------------

inline	iq	IQsqrt(iq x)
{
   iq temp;
   temp.val = _IQsqrt(x.val);
   return temp;
}

inline	iq30	IQ30sqrt(iq30 x)
{
   iq30 temp;
   temp.val = _IQ30sqrt(x.val);
   return temp;
}

inline	iq29	IQ29sqrt(iq29 x)
{
   iq29 temp;
   temp.val = _IQ29sqrt(x.val);
   return temp;
}

inline	iq28	IQ28sqrt(iq28 x)
{
   iq28 temp;
   temp.val = _IQ28sqrt(x.val);
   return temp;
}

inline	iq27	IQ27sqrt(iq27 x)
{
   iq27 temp;
   temp.val = _IQ27sqrt(x.val);
   return temp;
}

inline	iq26	IQ26sqrt(iq26 x)
{
   iq26 temp;
   temp.val = _IQ26sqrt(x.val);
   return temp;
}

inline	iq25	IQ25sqrt(iq25 x)
{
   iq25 temp;
   temp.val = _IQ25sqrt(x.val);
   return temp;
}

inline	iq24	IQ24sqrt(iq24 x)
{
   iq24 temp;
   temp.val = _IQ24sqrt(x.val);
   return temp;
}

inline	iq23	IQ23sqrt(iq23 x)
{
   iq23 temp;
   temp.val = _IQ23sqrt(x.val);
   return temp;
}

inline	iq22	IQ22sqrt(iq22 x)
{
   iq22 temp;
   temp.val = _IQ22sqrt(x.val);
   return temp;
}

inline	iq21	IQ21sqrt(iq21 x)
{
   iq21 temp;
   temp.val = _IQ21sqrt(x.val);
   return temp;
}

inline	iq20	IQ20sqrt(iq20 x)
{
   iq20 temp;
   temp.val = _IQ20sqrt(x.val);
   return temp;
}

inline	iq19	IQ19sqrt(iq19 x)
{
   iq19 temp;
   temp.val = _IQ19sqrt(x.val);
   return temp;
}

inline	iq18	IQ18sqrt(iq18 x)
{
   iq18 temp;
   temp.val = _IQ18sqrt(x.val);
   return temp;
}

inline	iq17	IQ17sqrt(iq17 x)
{
   iq17 temp;
   temp.val = _IQ17sqrt(x.val);
   return temp;
}

inline	iq16	IQ16sqrt(iq16 x)
{
   iq16 temp;
   temp.val = _IQ16sqrt(x.val);
   return temp;
}

inline	iq15	IQ15sqrt(iq15 x)
{
   iq15 temp;
   temp.val = _IQ15sqrt(x.val);
   return temp;
}

//---------------------------------------------------------------------------
// Functions: IQisqrt(A), IQNisqrt(A)
//---------------------------------------------------------------------------

inline	iq	IQisqrt(iq x)
{
   iq temp;
   temp.val = _IQisqrt(x.val);
   return temp;
}

inline	iq30	IQ30isqrt(iq30 x)
{
   iq30 temp;
   temp.val = _IQ30isqrt(x.val);
   return temp;
}

inline	iq29	IQ29isqrt(iq29 x)
{
   iq29 temp;
   temp.val = _IQ29isqrt(x.val);
   return temp;
}

inline	iq28	IQ28isqrt(iq28 x)
{
   iq28 temp;
   temp.val = _IQ28isqrt(x.val);
   return temp;
}

inline	iq27	IQ27isqrt(iq27 x)
{
   iq27 temp;
   temp.val = _IQ27isqrt(x.val);
   return temp;
}

inline	iq26	IQ26isqrt(iq26 x)
{
   iq26 temp;
   temp.val = _IQ26isqrt(x.val);
   return temp;
}

inline	iq25	IQ25isqrt(iq25 x)
{
   iq25 temp;
   temp.val = _IQ25isqrt(x.val);
   return temp;
}

inline	iq24	IQ24isqrt(iq24 x)
{
   iq24 temp;
   temp.val = _IQ24isqrt(x.val);
   return temp;
}

inline	iq23	IQ23isqrt(iq23 x)
{
   iq23 temp;
   temp.val = _IQ23isqrt(x.val);
   return temp;
}

inline	iq22	IQ22isqrt(iq22 x)
{
   iq22 temp;
   temp.val = _IQ22isqrt(x.val);
   return temp;
}

inline	iq21	IQ21isqrt(iq21 x)
{
   iq21 temp;
   temp.val = _IQ21isqrt(x.val);
   return temp;
}

inline	iq20	IQ20isqrt(iq20 x)
{
   iq20 temp;
   temp.val = _IQ20isqrt(x.val);
   return temp;
}

inline	iq19	IQ19isqrt(iq19 x)
{
   iq19 temp;
   temp.val = _IQ19isqrt(x.val);
   return temp;
}

inline	iq18	IQ18isqrt(iq18 x)
{
   iq18 temp;
   temp.val = _IQ18isqrt(x.val);
   return temp;
}

inline	iq17	IQ17isqrt(iq17 x)
{
   iq17 temp;
   temp.val = _IQ17isqrt(x.val);
   return temp;
}

inline	iq16	IQ16isqrt(iq16 x)
{
   iq16 temp;
   temp.val = _IQ16isqrt(x.val);
   return temp;
}

inline	iq15	IQ15isqrt(iq15 x)
{
   iq15 temp;
   temp.val = _IQ15isqrt(x.val);
   return temp;
}

//---------------------------------------------------------------------------
// Functions: IQmag(A,B), IQNmag(A,B)
//---------------------------------------------------------------------------

inline	iq	IQmag(iq y, iq x)
{
   iq temp;
   temp.val = _IQmag(y.val, x.val);
   return temp;
}

inline	iq30	IQ30mag(iq30 y, iq30 x)
{
   iq30 temp;
   temp.val = _IQ30mag(y.val, x.val);
   return temp;
}

inline	iq29	IQ29mag(iq29 y, iq29 x)
{
   iq29 temp;
   temp.val = _IQ29mag(y.val, x.val);
   return temp;
}

inline	iq28	IQ28mag(iq28 y, iq28 x)
{
   iq28 temp;
   temp.val = _IQ28mag(y.val, x.val);
   return temp;
}

inline	iq27	IQ27mag(iq27 y, iq27 x)
{
   iq27 temp;
   temp.val = _IQ27mag(y.val, x.val);
   return temp;
}

inline	iq26	IQ26mag(iq26 y, iq26 x)
{
   iq26 temp;
   temp.val = _IQ26mag(y.val, x.val);
   return temp;
}

inline	iq25	IQ25mag(iq25 y, iq25 x)
{
   iq25 temp;
   temp.val = _IQ25mag(y.val, x.val);
   return temp;
}

inline	iq24	IQ24mag(iq24 y, iq24 x)
{
   iq24 temp;
   temp.val = _IQ24mag(y.val, x.val);
   return temp;
}

inline	iq23	IQ23mag(iq23 y, iq23 x)
{
   iq23 temp;
   temp.val = _IQ23mag(y.val, x.val);
   return temp;
}

inline	iq22	IQ22mag(iq22 y, iq22 x)
{
   iq22 temp;
   temp.val = _IQ22mag(y.val, x.val);
   return temp;
}

inline	iq21	IQ21mag(iq21 y, iq21 x)
{
   iq21 temp;
   temp.val = _IQ21mag(y.val, x.val);
   return temp;
}

inline	iq20	IQ20mag(iq20 y, iq20 x)
{
   iq20 temp;
   temp.val = _IQ20mag(y.val, x.val);
   return temp;
}

inline	iq19	IQ19mag(iq19 y, iq19 x)
{
   iq19 temp;
   temp.val = _IQ19mag(y.val, x.val);
   return temp;
}

inline	iq18	IQ18mag(iq18 y, iq18 x)
{
   iq18 temp;
   temp.val = _IQ18mag(y.val, x.val);
   return temp;
}

inline	iq17	IQ17mag(iq17 y, iq17 x)
{
   iq17 temp;
   temp.val = _IQ17mag(y.val, x.val);
   return temp;
}

inline	iq16	IQ16mag(iq16 y, iq16 x)
{
   iq16 temp;
   temp.val = _IQ16mag(y.val, x.val);
   return temp;
}

inline	iq15	IQ15mag(iq15 y, iq15 x)
{
   iq15 temp;
   temp.val = _IQ15mag(y.val, x.val);
   return temp;
}

//---------------------------------------------------------------------------
// Functions: IQabs(A), IQNabs(A)
//---------------------------------------------------------------------------

inline	iq	IQabs(iq y)
{
   iq temp;
   temp.val = labs(y.val);
   return temp;
}

inline	iq30	IQ30abs(iq30 y)
{
   iq30 temp;
   temp.val = labs(y.val);
   return temp;
}

inline	iq29	IQ29abs(iq29 y)
{
   iq29 temp;
   temp.val = labs(y.val);
   return temp;
}

inline	iq28	IQ28abs(iq28 y)
{
   iq28 temp;
   temp.val = labs(y.val);
   return temp;
}

inline	iq27	IQ27abs(iq27 y)
{
   iq27 temp;
   temp.val = labs(y.val);
   return temp;
}

inline	iq26	IQ26abs(iq26 y)
{
   iq26 temp;
   temp.val = labs(y.val);
   return temp;
}

inline	iq25	IQ25abs(iq25 y)
{
   iq25 temp;
   temp.val = labs(y.val);
   return temp;
}

inline	iq24	IQ24abs(iq24 y)
{
   iq24 temp;
   temp.val = labs(y.val);
   return temp;
}

inline	iq23	IQ23abs(iq23 y)
{
   iq23 temp;
   temp.val = labs(y.val);
   return temp;
}

inline	iq22	IQ22abs(iq22 y)
{
   iq22 temp;
   temp.val = labs(y.val);
   return temp;
}

inline	iq21	IQ21abs(iq21 y)
{
   iq21 temp;
   temp.val = labs(y.val);
   return temp;
}

inline	iq20	IQ20abs(iq20 y)
{
   iq20 temp;
   temp.val = labs(y.val);
   return temp;
}

inline	iq19	IQ19abs(iq19 y)
{
   iq19 temp;
   temp.val = labs(y.val);
   return temp;
}

inline	iq18	IQ18abs(iq18 y)
{
   iq18 temp;
   temp.val = labs(y.val);
   return temp;
}

inline	iq17	IQ17abs(iq17 y)
{
   iq17 temp;
   temp.val = labs(y.val);
   return temp;
}

inline	iq16	IQ16abs(iq16 y)
{
   iq16 temp;
   temp.val = labs(y.val);
   return temp;
}

inline	iq15	IQ15abs(iq15 y)
{
   iq15 temp;
   temp.val = labs(y.val);
   return temp;
}

//---------------------------------------------------------------------------
// Operators: "*", "*="
//---------------------------------------------------------------------------

inline iq operator * (const iq &x, const iq &y)
{
   iq temp;
   temp.val = _IQmpy(x.val, y.val);
   return temp;
}

inline iq & iq :: operator *= (const iq &x)
{
   val = _IQmpy(val, x.val);
   return *this;
}

inline iq30 operator * (const iq30 &x, const iq30 &y)
{
   iq30 temp;
   temp.val = _IQ30mpy(x.val, y.val);
   return temp;
}

inline iq30 & iq30 :: operator *= (const iq30 &x)
{
   val = _IQ30mpy(val, x.val);
   return *this;
}

inline iq29 operator * (const iq29 &x, const iq29 &y)
{
   iq29 temp;
   temp.val = _IQ29mpy(x.val, y.val);
   return temp;
}

inline iq29 & iq29 :: operator *= (const iq29 &x)
{
   val = _IQ29mpy(val, x.val);
   return *this;
}

inline iq28 operator * (const iq28 &x, const iq28 &y)
{
   iq28 temp;
   temp.val = _IQ28mpy(x.val, y.val);
   return temp;
}

inline iq28 & iq28 :: operator *= (const iq28 &x)
{
   val = _IQ28mpy(val, x.val);
   return *this;
}

inline iq27 operator * (const iq27 &x, const iq27 &y)
{
   iq27 temp;
   temp.val = _IQ27mpy(x.val, y.val);
   return temp;
}

inline iq27 & iq27 :: operator *= (const iq27 &x)
{
   val = _IQ27mpy(val, x.val);
   return *this;
}

inline iq26 operator * (const iq26 &x, const iq26 &y)
{
   iq26 temp;
   temp.val = _IQ26mpy(x.val, y.val);
   return temp;
}

inline iq26 & iq26 :: operator *= (const iq26 &x)
{
   val = _IQ26mpy(val, x.val);
   return *this;
}

inline iq25 operator * (const iq25 &x, const iq25 &y)
{
   iq25 temp;
   temp.val = _IQ25mpy(x.val, y.val);
   return temp;
}

inline iq25 & iq25 :: operator *= (const iq25 &x)
{
   val = _IQ25mpy(val, x.val);
   return *this;
}

inline iq24 operator * (const iq24 &x, const iq24 &y)
{
   iq24 temp;
   temp.val = _IQ24mpy(x.val, y.val);
   return temp;
}

inline iq24 & iq24 :: operator *= (const iq24 &x)
{
   val = _IQ24mpy(val, x.val);
   return *this;
}

inline iq23 operator * (const iq23 &x, const iq23 &y)
{
   iq23 temp;
   temp.val = _IQ23mpy(x.val, y.val);
   return temp;
}

inline iq23 & iq23 :: operator *= (const iq23 &x)
{
   val = _IQ23mpy(val, x.val);
   return *this;
}

inline iq22 operator * (const iq22 &x, const iq22 &y)
{
   iq22 temp;
   temp.val = _IQ22mpy(x.val, y.val);
   return temp;
}

inline iq22 & iq22 :: operator *= (const iq22 &x)
{
   val = _IQ22mpy(val, x.val);
   return *this;
}

inline iq21 operator * (const iq21 &x, const iq21 &y)
{
   iq21 temp;
   temp.val = _IQ21mpy(x.val, y.val);
   return temp;
}

inline iq21 & iq21 :: operator *= (const iq21 &x)
{
   val = _IQ21mpy(val, x.val);
   return *this;
}

inline iq20 operator * (const iq20 &x, const iq20 &y)
{
   iq20 temp;
   temp.val = _IQ20mpy(x.val, y.val);
   return temp;
}

inline iq20 & iq20 :: operator *= (const iq20 &x)
{
   val = _IQ20mpy(val, x.val);
   return *this;
}

inline iq19 operator * (const iq19 &x, const iq19 &y)
{
   iq19 temp;
   temp.val = _IQ19mpy(x.val, y.val);
   return temp;
}

inline iq19 & iq19 :: operator *= (const iq19 &x)
{
   val = _IQ19mpy(val, x.val);
   return *this;
}

inline iq18 operator * (const iq18 &x, const iq18 &y)
{
   iq18 temp;
   temp.val = _IQ18mpy(x.val, y.val);
   return temp;
}

inline iq18 & iq18 :: operator *= (const iq18 &x)
{
   val = _IQ18mpy(val, x.val);
   return *this;
}

inline iq17 operator * (const iq17 &x, const iq17 &y)
{
   iq17 temp;
   temp.val = _IQ17mpy(x.val, y.val);
   return temp;
}

inline iq17 & iq17 :: operator *= (const iq17 &x)
{
   val = _IQ17mpy(val, x.val);
   return *this;
}

inline iq16 operator * (const iq16 &x, const iq16 &y)
{
   iq16 temp;
   temp.val = _IQ16mpy(x.val, y.val);
   return temp;
}

inline iq16 & iq16 :: operator *= (const iq16 &x)
{
   val = _IQ16mpy(val, x.val);
   return *this;
}

inline iq15 operator * (const iq15 &x, const iq15 &y)
{
   iq15 temp;
   temp.val = _IQ15mpy(x.val, y.val);
   return temp;
}

inline iq15 & iq15 :: operator *= (const iq15 &x)
{
   val = _IQ15mpy(val, x.val);
   return *this;
}

//---------------------------------------------------------------------------
// Operator: "="
//---------------------------------------------------------------------------

inline iq & iq :: operator = (const iq & x)
{
   val = x.val;
   return *this;
}

inline iq30 & iq30 :: operator = (const iq30 & x)
{
   val = x.val;
   return *this;
}

inline iq29 & iq29 :: operator = (const iq29 & x)
{
   val = x.val;
   return *this;
}

inline iq28 & iq28 :: operator = (const iq28 & x)
{
   val = x.val;
   return *this;
}

inline iq27 & iq27 :: operator = (const iq27 & x)
{
   val = x.val;
   return *this;
}

inline iq26 & iq26 :: operator = (const iq26 & x)
{
   val = x.val;
   return *this;
}

inline iq25 & iq25 :: operator = (const iq25 & x)
{
   val = x.val;
   return *this;
}

inline iq24 & iq24 :: operator = (const iq24 & x)
{
   val = x.val;
   return *this;
}

inline iq23 & iq23 :: operator = (const iq23 & x)
{
   val = x.val;
   return *this;
}

inline iq22 & iq22 :: operator = (const iq22 & x)
{
   val = x.val;
   return *this;
}

inline iq21 & iq21 :: operator = (const iq21 & x)
{
   val = x.val;
   return *this;
}

inline iq20 & iq20 :: operator = (const iq20 & x)
{
   val = x.val;
   return *this;
}

inline iq19 & iq19 :: operator = (const iq19 & x)
{
   val = x.val;
   return *this;
}

inline iq18 & iq18 :: operator = (const iq18 & x)
{
   val = x.val;
   return *this;
}

inline iq17 & iq17 :: operator = (const iq17 & x)
{
   val = x.val;
   return *this;
}

inline iq16 & iq16 :: operator = (const iq16 & x)
{
   val = x.val;
   return *this;
}

inline iq15 & iq15 :: operator = (const iq15 & x)
{
   val = x.val;
   return *this;
}

//---------------------------------------------------------------------------
// Operators: "-", "-=":
//---------------------------------------------------------------------------

inline iq operator - (const iq &x, const iq &y)
{
   iq temp;
   temp.val = x.val - y.val;
   return temp;
}

inline iq operator - (const iq &x)
{
   iq temp;
   temp.val = - x.val;
   return temp;
}

inline iq & iq :: operator -= (const iq &x)
{
   val -= x.val;
   return *this;
}

inline iq30 operator - (const iq30 &x, const iq30 &y)
{
   iq30 temp;
   temp.val = x.val - y.val;
   return temp;
}

inline iq30 operator - (const iq30 &x)
{
   iq30 temp;
   temp.val = - x.val;
   return temp;
}

inline iq30 & iq30 :: operator -= (const iq30 &x)
{
   val -= x.val;
   return *this;
}

inline iq29 operator - (const iq29 &x, const iq29 &y)
{
   iq29 temp;
   temp.val = x.val - y.val;
   return temp;
}

inline iq29 operator - (const iq29 &x)
{
   iq29 temp;
   temp.val = - x.val;
   return temp;
}

inline iq29 & iq29 :: operator -= (const iq29 &x)
{
   val -= x.val;
   return *this;
}

inline iq28 operator - (const iq28 &x, const iq28 &y)
{
   iq28 temp;
   temp.val = x.val - y.val;
   return temp;
}

inline iq28 operator - (const iq28 &x)
{
   iq28 temp;
   temp.val = - x.val;
   return temp;
}

inline iq28 & iq28 :: operator -= (const iq28 &x)
{
   val -= x.val;
   return *this;
}

inline iq27 operator - (const iq27 &x, const iq27 &y)
{
   iq27 temp;
   temp.val = x.val - y.val;
   return temp;
}

inline iq27 operator - (const iq27 &x)
{
   iq27 temp;
   temp.val = - x.val;
   return temp;
}

inline iq27 & iq27 :: operator -= (const iq27 &x)
{
   val -= x.val;
   return *this;
}

inline iq26 operator - (const iq26 &x, const iq26 &y)
{
   iq26 temp;
   temp.val = x.val - y.val;
   return temp;
}

inline iq26 operator - (const iq26 &x)
{
   iq26 temp;
   temp.val = - x.val;
   return temp;
}

inline iq26 & iq26 :: operator -= (const iq26 &x)
{
   val -= x.val;
   return *this;
}

inline iq25 operator - (const iq25 &x, const iq25 &y)
{
   iq25 temp;
   temp.val = x.val - y.val;
   return temp;
}

inline iq25 operator - (const iq25 &x)
{
   iq25 temp;
   temp.val = - x.val;
   return temp;
}

inline iq25 & iq25 :: operator -= (const iq25 &x)
{
   val -= x.val;
   return *this;
}

inline iq24 operator - (const iq24 &x, const iq24 &y)
{
   iq24 temp;
   temp.val = x.val - y.val;
   return temp;
}

inline iq24 operator - (const iq24 &x)
{
   iq24 temp;
   temp.val = - x.val;
   return temp;
}

inline iq24 & iq24 :: operator -= (const iq24 &x)
{
   val -= x.val;
   return *this;
}

inline iq23 operator - (const iq23 &x, const iq23 &y)
{
   iq23 temp;
   temp.val = x.val - y.val;
   return temp;
}

inline iq23 operator - (const iq23 &x)
{
   iq23 temp;
   temp.val = - x.val;
   return temp;
}

inline iq23 & iq23 :: operator -= (const iq23 &x)
{
   val -= x.val;
   return *this;
}

inline iq22 operator - (const iq22 &x, const iq22 &y)
{
   iq22 temp;
   temp.val = x.val - y.val;
   return temp;
}

inline iq22 operator - (const iq22 &x)
{
   iq22 temp;
   temp.val = - x.val;
   return temp;
}

inline iq22 & iq22 :: operator -= (const iq22 &x)
{
   val -= x.val;
   return *this;
}

inline iq21 operator - (const iq21 &x, const iq21 &y)
{
   iq21 temp;
   temp.val = x.val - y.val;
   return temp;
}

inline iq21 operator - (const iq21 &x)
{
   iq21 temp;
   temp.val = - x.val;
   return temp;
}

inline iq21 & iq21 :: operator -= (const iq21 &x)
{
   val -= x.val;
   return *this;
}

inline iq20 operator - (const iq20 &x, const iq20 &y)
{
   iq20 temp;
   temp.val = x.val - y.val;
   return temp;
}

inline iq20 operator - (const iq20 &x)
{
   iq20 temp;
   temp.val = - x.val;
   return temp;
}

inline iq20 & iq20 :: operator -= (const iq20 &x)
{
   val -= x.val;
   return *this;
}

inline iq19 operator - (const iq19 &x, const iq19 &y)
{
   iq19 temp;
   temp.val = x.val - y.val;
   return temp;
}

inline iq19 operator - (const iq19 &x)
{
   iq19 temp;
   temp.val = - x.val;
   return temp;
}

inline iq19 & iq19 :: operator -= (const iq19 &x)
{
   val -= x.val;
   return *this;
}

inline iq18 operator - (const iq18 &x, const iq18 &y)
{
   iq18 temp;
   temp.val = x.val - y.val;
   return temp;
}

inline iq18 operator - (const iq18 &x)
{
   iq18 temp;
   temp.val = - x.val;
   return temp;
}

inline iq18 & iq18 :: operator -= (const iq18 &x)
{
   val -= x.val;
   return *this;
}

inline iq17 operator - (const iq17 &x, const iq17 &y)
{
   iq17 temp;
   temp.val = x.val - y.val;
   return temp;
}

inline iq17 operator - (const iq17 &x)
{
   iq17 temp;
   temp.val = - x.val;
   return temp;
}

inline iq17 & iq17 :: operator -= (const iq17 &x)
{
   val -= x.val;
   return *this;
}

inline iq16 operator - (const iq16 &x, const iq16 &y)
{
   iq16 temp;
   temp.val = x.val - y.val;
   return temp;
}

inline iq16 operator - (const iq16 &x)
{
   iq16 temp;
   temp.val = - x.val;
   return temp;
}

inline iq16 & iq16 :: operator -= (const iq16 &x)
{
   val -= x.val;
   return *this;
}

inline iq15 operator - (const iq15 &x, const iq15 &y)
{
   iq15 temp;
   temp.val = x.val - y.val;
   return temp;
}

inline iq15 operator - (const iq15 &x)
{
   iq15 temp;
   temp.val = - x.val;
   return temp;
}

inline iq15 & iq15 :: operator -= (const iq15 &x)
{
   val -= x.val;
   return *this;
}

//---------------------------------------------------------------------------
// Operators: "+", "+="
//---------------------------------------------------------------------------

inline iq operator + (const iq &x, const iq &y)
{
   iq temp;
   temp.val = x.val + y.val;
   return temp;
}

inline iq & iq :: operator += (const iq &x)
{
   val += x.val;
   return *this;
}

inline iq30 operator + (const iq30 &x, const iq30 &y)
{
   iq30 temp;
   temp.val = x.val + y.val;
   return temp;
}

inline iq30 & iq30 :: operator += (const iq30 &x)
{
   val += x.val;
   return *this;
}

inline iq29 operator + (const iq29 &x, const iq29 &y)
{
   iq29 temp;
   temp.val = x.val + y.val;
   return temp;
}

inline iq29 & iq29 :: operator += (const iq29 &x)
{
   val += x.val;
   return *this;
}

inline iq28 operator + (const iq28 &x, const iq28 &y)
{
   iq28 temp;
   temp.val = x.val + y.val;
   return temp;
}

inline iq28 & iq28 :: operator += (const iq28 &x)
{
   val += x.val;
   return *this;
}

inline iq27 operator + (const iq27 &x, const iq27 &y)
{
   iq27 temp;
   temp.val = x.val + y.val;
   return temp;
}

inline iq27 & iq27 :: operator += (const iq27 &x)
{
   val += x.val;
   return *this;
}

inline iq26 operator + (const iq26 &x, const iq26 &y)
{
   iq26 temp;
   temp.val = x.val + y.val;
   return temp;
}

inline iq26 & iq26 :: operator += (const iq26 &x)
{
   val += x.val;
   return *this;
}

inline iq25 operator + (const iq25 &x, const iq25 &y)
{
   iq25 temp;
   temp.val = x.val + y.val;
   return temp;
}

inline iq25 & iq25 :: operator += (const iq25 &x)
{
   val += x.val;
   return *this;
}

inline iq24 operator + (const iq24 &x, const iq24 &y)
{
   iq24 temp;
   temp.val = x.val + y.val;
   return temp;
}

inline iq24 & iq24 :: operator += (const iq24 &x)
{
   val += x.val;
   return *this;
}

inline iq23 operator + (const iq23 &x, const iq23 &y)
{
   iq23 temp;
   temp.val = x.val + y.val;
   return temp;
}

inline iq23 & iq23 :: operator += (const iq23 &x)
{
   val += x.val;
   return *this;
}

inline iq22 operator + (const iq22 &x, const iq22 &y)
{
   iq22 temp;
   temp.val = x.val + y.val;
   return temp;
}

inline iq22 & iq22 :: operator += (const iq22 &x)
{
   val += x.val;
   return *this;
}

inline iq21 operator + (const iq21 &x, const iq21 &y)
{
   iq21 temp;
   temp.val = x.val + y.val;
   return temp;
}

inline iq21 & iq21 :: operator += (const iq21 &x)
{
   val += x.val;
   return *this;
}

inline iq20 operator + (const iq20 &x, const iq20 &y)
{
   iq20 temp;
   temp.val = x.val + y.val;
   return temp;
}

inline iq20 & iq20 :: operator += (const iq20 &x)
{
   val += x.val;
   return *this;
}

inline iq19 operator + (const iq19 &x, const iq19 &y)
{
   iq19 temp;
   temp.val = x.val + y.val;
   return temp;
}

inline iq19 & iq19 :: operator += (const iq19 &x)
{
   val += x.val;
   return *this;
}

inline iq18 operator + (const iq18 &x, const iq18 &y)
{
   iq18 temp;
   temp.val = x.val + y.val;
   return temp;
}

inline iq18 & iq18 :: operator += (const iq18 &x)
{
   val += x.val;
   return *this;
}

inline iq17 operator + (const iq17 &x, const iq17 &y)
{
   iq17 temp;
   temp.val = x.val + y.val;
   return temp;
}

inline iq17 & iq17 :: operator += (const iq17 &x)
{
   val += x.val;
   return *this;
}

inline iq16 operator + (const iq16 &x, const iq16 &y)
{
   iq16 temp;
   temp.val = x.val + y.val;
   return temp;
}

inline iq16 & iq16 :: operator += (const iq16 &x)
{
   val += x.val;
   return *this;
}

inline iq15 operator + (const iq15 &x, const iq15 &y)
{
   iq15 temp;
   temp.val = x.val + y.val;
   return temp;
}

inline iq15 & iq15 :: operator += (const iq15 &x)
{
   val += x.val;
   return *this;
}

//---------------------------------------------------------------------------
// Operators: "/", "/="
//---------------------------------------------------------------------------

inline iq operator / (const iq &x, const iq &y)
{
   iq temp;
   temp.val = _IQdiv(x.val, y.val);
   return temp;
}

inline iq & iq :: operator /= (const iq &x)
{
   val = _IQdiv(val, x.val);
   return *this;
}

inline iq30 operator / (const iq30 &x, const iq30 &y)
{
   iq30 temp;
   temp.val = _IQ30div(x.val, y.val);
   return temp;
}

inline iq30 & iq30 :: operator /= (const iq30 &x)
{
   val = _IQ30div(val, x.val);
   return *this;
}

inline iq29 operator / (const iq29 &x, const iq29 &y)
{
   iq29 temp;
   temp.val = _IQ29div(x.val, y.val);
   return temp;
}

inline iq29 & iq29 :: operator /= (const iq29 &x)
{
   val = _IQ29div(val, x.val);
   return *this;
}

inline iq28 operator / (const iq28 &x, const iq28 &y)
{
   iq28 temp;
   temp.val = _IQ28div(x.val, y.val);
   return temp;
}

inline iq28 & iq28 :: operator /= (const iq28 &x)
{
   val = _IQ28div(val, x.val);
   return *this;
}

inline iq27 operator / (const iq27 &x, const iq27 &y)
{
   iq27 temp;
   temp.val = _IQ27div(x.val, y.val);
   return temp;
}

inline iq27 & iq27 :: operator /= (const iq27 &x)
{
   val = _IQ27div(val, x.val);
   return *this;
}

inline iq26 operator / (const iq26 &x, const iq26 &y)
{
   iq26 temp;
   temp.val = _IQ26div(x.val, y.val);
   return temp;
}

inline iq26 & iq26 :: operator /= (const iq26 &x)
{
   val = _IQ26div(val, x.val);
   return *this;
}

inline iq25 operator / (const iq25 &x, const iq25 &y)
{
   iq25 temp;
   temp.val = _IQ25div(x.val, y.val);
   return temp;
}

inline iq25 & iq25 :: operator /= (const iq25 &x)
{
   val = _IQ25div(val, x.val);
   return *this;
}

inline iq24 operator / (const iq24 &x, const iq24 &y)
{
   iq24 temp;
   temp.val = _IQ24div(x.val, y.val);
   return temp;
}

inline iq24 & iq24 :: operator /= (const iq24 &x)
{
   val = _IQ24div(val, x.val);
   return *this;
}

inline iq23 operator / (const iq23 &x, const iq23 &y)
{
   iq23 temp;
   temp.val = _IQ23div(x.val, y.val);
   return temp;
}

inline iq23 & iq23 :: operator /= (const iq23 &x)
{
   val = _IQ23div(val, x.val);
   return *this;
}

inline iq22 operator / (const iq22 &x, const iq22 &y)
{
   iq22 temp;
   temp.val = _IQ22div(x.val, y.val);
   return temp;
}

inline iq22 & iq22 :: operator /= (const iq22 &x)
{
   val = _IQ22div(val, x.val);
   return *this;
}

inline iq21 operator / (const iq21 &x, const iq21 &y)
{
   iq21 temp;
   temp.val = _IQ21div(x.val, y.val);
   return temp;
}

inline iq21 & iq21 :: operator /= (const iq21 &x)
{
   val = _IQ21div(val, x.val);
   return *this;
}

inline iq20 operator / (const iq20 &x, const iq20 &y)
{
   iq20 temp;
   temp.val = _IQ20div(x.val, y.val);
   return temp;
}

inline iq20 & iq20 :: operator /= (const iq20 &x)
{
   val = _IQ20div(val, x.val);
   return *this;
}

inline iq19 operator / (const iq19 &x, const iq19 &y)
{
   iq19 temp;
   temp.val = _IQ19div(x.val, y.val);
   return temp;
}

inline iq19 & iq19 :: operator /= (const iq19 &x)
{
   val = _IQ19div(val, x.val);
   return *this;
}

inline iq18 operator / (const iq18 &x, const iq18 &y)
{
   iq18 temp;
   temp.val = _IQ18div(x.val, y.val);
   return temp;
}

inline iq18 & iq18 :: operator /= (const iq18 &x)
{
   val = _IQ18div(val, x.val);
   return *this;
}

inline iq17 operator / (const iq17 &x, const iq17 &y)
{
   iq17 temp;
   temp.val = _IQ17div(x.val, y.val);
   return temp;
}

inline iq17 & iq17 :: operator /= (const iq17 &x)
{
   val = _IQ17div(val, x.val);
   return *this;
}

inline iq16 operator / (const iq16 &x, const iq16 &y)
{
   iq16 temp;
   temp.val = _IQ16div(x.val, y.val);
   return temp;
}

inline iq16 & iq16 :: operator /= (const iq16 &x)
{
   val = _IQ16div(val, x.val);
   return *this;
}

inline iq15 operator / (const iq15 &x, const iq15 &y)
{
   iq15 temp;
   temp.val = _IQ15div(x.val, y.val);
   return temp;
}

inline iq15 & iq15 :: operator /= (const iq15 &x)
{
   val = _IQ15div(val, x.val);
   return *this;
}

//---------------------------------------------------------------------------
// Operators: "==", "!=", "<", ">", "<=", ">=", "&&", "||"
//
inline bool operator == (const iq &x, const iq& y)     { return (x.val == y.val); }
inline bool operator == (const iq30 &x, const iq30& y) { return (x.val == y.val); }
inline bool operator == (const iq29 &x, const iq29& y) { return (x.val == y.val); }
inline bool operator == (const iq28 &x, const iq28& y) { return (x.val == y.val); }
inline bool operator == (const iq27 &x, const iq27& y) { return (x.val == y.val); }
inline bool operator == (const iq26 &x, const iq26& y) { return (x.val == y.val); }
inline bool operator == (const iq25 &x, const iq25& y) { return (x.val == y.val); }
inline bool operator == (const iq24 &x, const iq24& y) { return (x.val == y.val); }
inline bool operator == (const iq23 &x, const iq23& y) { return (x.val == y.val); }
inline bool operator == (const iq22 &x, const iq22& y) { return (x.val == y.val); }
inline bool operator == (const iq21 &x, const iq21& y) { return (x.val == y.val); }
inline bool operator == (const iq20 &x, const iq20& y) { return (x.val == y.val); }
inline bool operator == (const iq19 &x, const iq19& y) { return (x.val == y.val); }
inline bool operator == (const iq18 &x, const iq18& y) { return (x.val == y.val); }
inline bool operator == (const iq17 &x, const iq17& y) { return (x.val == y.val); }
inline bool operator == (const iq16 &x, const iq16& y) { return (x.val == y.val); }
inline bool operator == (const iq15 &x, const iq15& y) { return (x.val == y.val); }

inline bool operator != (const iq &x, const iq& y)     { return (x.val != y.val); }
inline bool operator != (const iq30 &x, const iq30& y) { return (x.val != y.val); }
inline bool operator != (const iq29 &x, const iq29& y) { return (x.val != y.val); }
inline bool operator != (const iq28 &x, const iq28& y) { return (x.val != y.val); }
inline bool operator != (const iq27 &x, const iq27& y) { return (x.val != y.val); }
inline bool operator != (const iq26 &x, const iq26& y) { return (x.val != y.val); }
inline bool operator != (const iq25 &x, const iq25& y) { return (x.val != y.val); }
inline bool operator != (const iq24 &x, const iq24& y) { return (x.val != y.val); }
inline bool operator != (const iq23 &x, const iq23& y) { return (x.val != y.val); }
inline bool operator != (const iq22 &x, const iq22& y) { return (x.val != y.val); }
inline bool operator != (const iq21 &x, const iq21& y) { return (x.val != y.val); }
inline bool operator != (const iq20 &x, const iq20& y) { return (x.val != y.val); }
inline bool operator != (const iq19 &x, const iq19& y) { return (x.val != y.val); }
inline bool operator != (const iq18 &x, const iq18& y) { return (x.val != y.val); }
inline bool operator != (const iq17 &x, const iq17& y) { return (x.val != y.val); }
inline bool operator != (const iq16 &x, const iq16& y) { return (x.val != y.val); }
inline bool operator != (const iq15 &x, const iq15& y) { return (x.val != y.val); }

inline bool operator <  (const iq &x, const iq &y)     { return (x.val <  y.val); }
inline bool operator <  (const iq30 &x, const iq30& y) { return (x.val <  y.val); }
inline bool operator <  (const iq29 &x, const iq29& y) { return (x.val <  y.val); }
inline bool operator <  (const iq28 &x, const iq28& y) { return (x.val <  y.val); }
inline bool operator <  (const iq27 &x, const iq27& y) { return (x.val <  y.val); }
inline bool operator <  (const iq26 &x, const iq26& y) { return (x.val <  y.val); }
inline bool operator <  (const iq25 &x, const iq25& y) { return (x.val <  y.val); }
inline bool operator <  (const iq24 &x, const iq24& y) { return (x.val <  y.val); }
inline bool operator <  (const iq23 &x, const iq23& y) { return (x.val <  y.val); }
inline bool operator <  (const iq22 &x, const iq22& y) { return (x.val <  y.val); }
inline bool operator <  (const iq21 &x, const iq21& y) { return (x.val <  y.val); }
inline bool operator <  (const iq20 &x, const iq20& y) { return (x.val <  y.val); }
inline bool operator <  (const iq19 &x, const iq19& y) { return (x.val <  y.val); }
inline bool operator <  (const iq18 &x, const iq18& y) { return (x.val <  y.val); }
inline bool operator <  (const iq17 &x, const iq17& y) { return (x.val <  y.val); }
inline bool operator <  (const iq16 &x, const iq16& y) { return (x.val <  y.val); }
inline bool operator <  (const iq15 &x, const iq15& y) { return (x.val <  y.val); }

inline bool operator >  (const iq &x, const iq &y)     { return (x.val >  y.val); }
inline bool operator >  (const iq30 &x, const iq30& y) { return (x.val >  y.val); }
inline bool operator >  (const iq29 &x, const iq29& y) { return (x.val >  y.val); }
inline bool operator >  (const iq28 &x, const iq28& y) { return (x.val >  y.val); }
inline bool operator >  (const iq27 &x, const iq27& y) { return (x.val >  y.val); }
inline bool operator >  (const iq26 &x, const iq26& y) { return (x.val >  y.val); }
inline bool operator >  (const iq25 &x, const iq25& y) { return (x.val >  y.val); }
inline bool operator >  (const iq24 &x, const iq24& y) { return (x.val >  y.val); }
inline bool operator >  (const iq23 &x, const iq23& y) { return (x.val >  y.val); }
inline bool operator >  (const iq22 &x, const iq22& y) { return (x.val >  y.val); }
inline bool operator >  (const iq21 &x, const iq21& y) { return (x.val >  y.val); }
inline bool operator >  (const iq20 &x, const iq20& y) { return (x.val >  y.val); }
inline bool operator >  (const iq19 &x, const iq19& y) { return (x.val >  y.val); }
inline bool operator >  (const iq18 &x, const iq18& y) { return (x.val >  y.val); }
inline bool operator >  (const iq17 &x, const iq17& y) { return (x.val >  y.val); }
inline bool operator >  (const iq16 &x, const iq16& y) { return (x.val >  y.val); }
inline bool operator >  (const iq15 &x, const iq15& y) { return (x.val >  y.val); }

inline bool operator <= (const iq &x, const iq &y)     { return (x.val <= y.val); }
inline bool operator <= (const iq30 &x, const iq30& y) { return (x.val <= y.val); }
inline bool operator <= (const iq29 &x, const iq29& y) { return (x.val <= y.val); }
inline bool operator <= (const iq28 &x, const iq28& y) { return (x.val <= y.val); }
inline bool operator <= (const iq27 &x, const iq27& y) { return (x.val <= y.val); }
inline bool operator <= (const iq26 &x, const iq26& y) { return (x.val <= y.val); }
inline bool operator <= (const iq25 &x, const iq25& y) { return (x.val <= y.val); }
inline bool operator <= (const iq24 &x, const iq24& y) { return (x.val <= y.val); }
inline bool operator <= (const iq23 &x, const iq23& y) { return (x.val <= y.val); }
inline bool operator <= (const iq22 &x, const iq22& y) { return (x.val <= y.val); }
inline bool operator <= (const iq21 &x, const iq21& y) { return (x.val <= y.val); }
inline bool operator <= (const iq20 &x, const iq20& y) { return (x.val <= y.val); }
inline bool operator <= (const iq19 &x, const iq19& y) { return (x.val <= y.val); }
inline bool operator <= (const iq18 &x, const iq18& y) { return (x.val <= y.val); }
inline bool operator <= (const iq17 &x, const iq17& y) { return (x.val <= y.val); }
inline bool operator <= (const iq16 &x, const iq16& y) { return (x.val <= y.val); }
inline bool operator <= (const iq15 &x, const iq15& y) { return (x.val <= y.val); }

inline bool operator >= (const iq &x, const iq &y)     { return (x.val >= y.val); }
inline bool operator >= (const iq30 &x, const iq30& y) { return (x.val >= y.val); }
inline bool operator >= (const iq29 &x, const iq29& y) { return (x.val >= y.val); }
inline bool operator >= (const iq28 &x, const iq28& y) { return (x.val >= y.val); }
inline bool operator >= (const iq27 &x, const iq27& y) { return (x.val >= y.val); }
inline bool operator >= (const iq26 &x, const iq26& y) { return (x.val >= y.val); }
inline bool operator >= (const iq25 &x, const iq25& y) { return (x.val >= y.val); }
inline bool operator >= (const iq24 &x, const iq24& y) { return (x.val >= y.val); }
inline bool operator >= (const iq23 &x, const iq23& y) { return (x.val >= y.val); }
inline bool operator >= (const iq22 &x, const iq22& y) { return (x.val >= y.val); }
inline bool operator >= (const iq21 &x, const iq21& y) { return (x.val >= y.val); }
inline bool operator >= (const iq20 &x, const iq20& y) { return (x.val >= y.val); }
inline bool operator >= (const iq19 &x, const iq19& y) { return (x.val >= y.val); }
inline bool operator >= (const iq18 &x, const iq18& y) { return (x.val >= y.val); }
inline bool operator >= (const iq17 &x, const iq17& y) { return (x.val >= y.val); }
inline bool operator >= (const iq16 &x, const iq16& y) { return (x.val >= y.val); }
inline bool operator >= (const iq15 &x, const iq15& y) { return (x.val >= y.val); }

inline bool operator && (const iq &x, const iq &y)     { return (x.val && y.val); }
inline bool operator && (const iq30 &x, const iq30& y) { return (x.val && y.val); }
inline bool operator && (const iq29 &x, const iq29& y) { return (x.val && y.val); }
inline bool operator && (const iq28 &x, const iq28& y) { return (x.val && y.val); }
inline bool operator && (const iq27 &x, const iq27& y) { return (x.val && y.val); }
inline bool operator && (const iq26 &x, const iq26& y) { return (x.val && y.val); }
inline bool operator && (const iq25 &x, const iq25& y) { return (x.val && y.val); }
inline bool operator && (const iq24 &x, const iq24& y) { return (x.val && y.val); }
inline bool operator && (const iq23 &x, const iq23& y) { return (x.val && y.val); }
inline bool operator && (const iq22 &x, const iq22& y) { return (x.val && y.val); }
inline bool operator && (const iq21 &x, const iq21& y) { return (x.val && y.val); }
inline bool operator && (const iq20 &x, const iq20& y) { return (x.val && y.val); }
inline bool operator && (const iq19 &x, const iq19& y) { return (x.val && y.val); }
inline bool operator && (const iq18 &x, const iq18& y) { return (x.val && y.val); }
inline bool operator && (const iq17 &x, const iq17& y) { return (x.val && y.val); }
inline bool operator && (const iq16 &x, const iq16& y) { return (x.val && y.val); }
inline bool operator && (const iq15 &x, const iq15& y) { return (x.val && y.val); }

inline bool operator || (const iq &x, const iq &y)     { return (x.val || y.val); }
inline bool operator || (const iq30 &x, const iq30& y) { return (x.val || y.val); }
inline bool operator || (const iq29 &x, const iq29& y) { return (x.val || y.val); }
inline bool operator || (const iq28 &x, const iq28& y) { return (x.val || y.val); }
inline bool operator || (const iq27 &x, const iq27& y) { return (x.val || y.val); }
inline bool operator || (const iq26 &x, const iq26& y) { return (x.val || y.val); }
inline bool operator || (const iq25 &x, const iq25& y) { return (x.val || y.val); }
inline bool operator || (const iq24 &x, const iq24& y) { return (x.val || y.val); }
inline bool operator || (const iq23 &x, const iq23& y) { return (x.val || y.val); }
inline bool operator || (const iq22 &x, const iq22& y) { return (x.val || y.val); }
inline bool operator || (const iq21 &x, const iq21& y) { return (x.val || y.val); }
inline bool operator || (const iq20 &x, const iq20& y) { return (x.val || y.val); }
inline bool operator || (const iq19 &x, const iq19& y) { return (x.val || y.val); }
inline bool operator || (const iq18 &x, const iq18& y) { return (x.val || y.val); }
inline bool operator || (const iq17 &x, const iq17& y) { return (x.val || y.val); }
inline bool operator || (const iq16 &x, const iq16& y) { return (x.val || y.val); }
inline bool operator || (const iq15 &x, const iq15& y) { return (x.val || y.val); }

//---------------------------------------------------------------------------
// Operators: "&", "&="
//---------------------------------------------------------------------------

inline iq operator & (const iq &x, const long &y)
{
   iq temp;
   temp.val = x.val & y;
   return temp;
}

inline iq & iq :: operator &= (const long &x)
{
   val &= x;
   return *this;
}

inline iq30 operator & (const iq30 &x, const long &y)
{
   iq30 temp;
   temp.val = x.val & y;
   return temp;
}

inline iq30 & iq30 :: operator &= (const long &x)
{
   val &= x;
   return *this;
}

inline iq29 operator & (const iq29 &x, const long &y)
{
   iq29 temp;
   temp.val = x.val & y;
   return temp;
}

inline iq29 & iq29 :: operator &= (const long &x)
{
   val &= x;
   return *this;
}

inline iq28 operator & (const iq28 &x, const long &y)
{
   iq28 temp;
   temp.val = x.val & y;
   return temp;
}

inline iq28 & iq28 :: operator &= (const long &x)
{
   val &= x;
   return *this;
}

inline iq27 operator & (const iq27 &x, const long &y)
{
   iq27 temp;
   temp.val = x.val & y;
   return temp;
}

inline iq27 & iq27 :: operator &= (const long &x)
{
   val &= x;
   return *this;
}

inline iq26 operator & (const iq26 &x, const long &y)
{
   iq26 temp;
   temp.val = x.val & y;
   return temp;
}

inline iq26 & iq26 :: operator &= (const long &x)
{
   val &= x;
   return *this;
}

inline iq25 operator & (const iq25 &x, const long &y)
{
   iq25 temp;
   temp.val = x.val & y;
   return temp;
}

inline iq25 & iq25 :: operator &= (const long &x)
{
   val &= x;
   return *this;
}

inline iq24 operator & (const iq24 &x, const long &y)
{
   iq24 temp;
   temp.val = x.val & y;
   return temp;
}

inline iq24 & iq24 :: operator &= (const long &x)
{
   val &= x;
   return *this;
}

inline iq23 operator & (const iq23 &x, const long &y)
{
   iq23 temp;
   temp.val = x.val & y;
   return temp;
}

inline iq23 & iq23 :: operator &= (const long &x)
{
   val &= x;
   return *this;
}

inline iq22 operator & (const iq22 &x, const long &y)
{
   iq22 temp;
   temp.val = x.val & y;
   return temp;
}

inline iq22 & iq22 :: operator &= (const long &x)
{
   val &= x;
   return *this;
}

inline iq21 operator & (const iq21 &x, const long &y)
{
   iq21 temp;
   temp.val = x.val & y;
   return temp;
}

inline iq21 & iq21 :: operator &= (const long &x)
{
   val &= x;
   return *this;
}

inline iq20 operator & (const iq20 &x, const long &y)
{
   iq20 temp;
   temp.val = x.val & y;
   return temp;
}

inline iq20 & iq20 :: operator &= (const long &x)
{
   val &= x;
   return *this;
}

inline iq19 operator & (const iq19 &x, const long &y)
{
   iq19 temp;
   temp.val = x.val & y;
   return temp;
}

inline iq19 & iq19 :: operator &= (const long &x)
{
   val &= x;
   return *this;
}

inline iq18 operator & (const iq18 &x, const long &y)
{
   iq18 temp;
   temp.val = x.val & y;
   return temp;
}

inline iq18 & iq18 :: operator &= (const long &x)
{
   val &= x;
   return *this;
}

inline iq17 operator & (const iq17 &x, const long &y)
{
   iq17 temp;
   temp.val = x.val & y;
   return temp;
}

inline iq17 & iq17 :: operator &= (const long &x)
{
   val &= x;
   return *this;
}

inline iq16 operator & (const iq16 &x, const long &y)
{
   iq16 temp;
   temp.val = x.val & y;
   return temp;
}

inline iq16 & iq16 :: operator &= (const long &x)
{
   val &= x;
   return *this;
}

inline iq15 operator & (const iq15 &x, const long &y)
{
   iq15 temp;
   temp.val = x.val & y;
   return temp;
}

inline iq15 & iq15 :: operator &= (const long &x)
{
   val &= x;
   return *this;
}

//---------------------------------------------------------------------------
// Operators: "|", "|="
//---------------------------------------------------------------------------

inline iq operator | (const iq &x, const long &y)
{
   iq temp;
   temp.val = x.val | y;
   return temp;
}

inline iq & iq :: operator |= (const long &x)
{
   val |= x;
   return *this;
}

inline iq30 operator | (const iq30 &x, const long &y)
{
   iq30 temp;
   temp.val = x.val | y;
   return temp;
}

inline iq30 & iq30 :: operator |= (const long &x)
{
   val |= x;
   return *this;
}

inline iq29 operator | (const iq29 &x, const long &y)
{
   iq29 temp;
   temp.val = x.val | y;
   return temp;
}

inline iq29 & iq29 :: operator |= (const long &x)
{
   val |= x;
   return *this;
}

inline iq28 operator | (const iq28 &x, const long &y)
{
   iq28 temp;
   temp.val = x.val | y;
   return temp;
}

inline iq28 & iq28 :: operator |= (const long &x)
{
   val |= x;
   return *this;
}

inline iq27 operator | (const iq27 &x, const long &y)
{
   iq27 temp;
   temp.val = x.val | y;
   return temp;
}

inline iq27 & iq27 :: operator |= (const long &x)
{
   val |= x;
   return *this;
}

inline iq26 operator | (const iq26 &x, const long &y)
{
   iq26 temp;
   temp.val = x.val | y;
   return temp;
}

inline iq26 & iq26 :: operator |= (const long &x)
{
   val |= x;
   return *this;
}

inline iq25 operator | (const iq25 &x, const long &y)
{
   iq25 temp;
   temp.val = x.val | y;
   return temp;
}

inline iq25 & iq25 :: operator |= (const long &x)
{
   val |= x;
   return *this;
}

inline iq24 operator | (const iq24 &x, const long &y)
{
   iq24 temp;
   temp.val = x.val | y;
   return temp;
}

inline iq24 & iq24 :: operator |= (const long &x)
{
   val |= x;
   return *this;
}

inline iq23 operator | (const iq23 &x, const long &y)
{
   iq23 temp;
   temp.val = x.val | y;
   return temp;
}

inline iq23 & iq23 :: operator |= (const long &x)
{
   val |= x;
   return *this;
}

inline iq22 operator | (const iq22 &x, const long &y)
{
   iq22 temp;
   temp.val = x.val | y;
   return temp;
}

inline iq22 & iq22 :: operator |= (const long &x)
{
   val |= x;
   return *this;
}

inline iq21 operator | (const iq21 &x, const long &y)
{
   iq21 temp;
   temp.val = x.val | y;
   return temp;
}

inline iq21 & iq21 :: operator |= (const long &x)
{
   val |= x;
   return *this;
}

inline iq20 operator | (const iq20 &x, const long &y)
{
   iq20 temp;
   temp.val = x.val | y;
   return temp;
}

inline iq20 & iq20 :: operator |= (const long &x)
{
   val |= x;
   return *this;
}

inline iq19 operator | (const iq19 &x, const long &y)
{
   iq19 temp;
   temp.val = x.val | y;
   return temp;
}

inline iq19 & iq19 :: operator |= (const long &x)
{
   val |= x;
   return *this;
}

inline iq18 operator | (const iq18 &x, const long &y)
{
   iq18 temp;
   temp.val = x.val | y;
   return temp;
}

inline iq18 & iq18 :: operator |= (const long &x)
{
   val |= x;
   return *this;
}

inline iq17 operator | (const iq17 &x, const long &y)
{
   iq17 temp;
   temp.val = x.val | y;
   return temp;
}

inline iq17 & iq17 :: operator |= (const long &x)
{
   val |= x;
   return *this;
}

inline iq16 operator | (const iq16 &x, const long &y)
{
   iq16 temp;
   temp.val = x.val | y;
   return temp;
}

inline iq16 & iq16 :: operator |= (const long &x)
{
   val |= x;
   return *this;
}

inline iq15 operator | (const iq15 &x, const long &y)
{
   iq15 temp;
   temp.val = x.val | y;
   return temp;
}

inline iq15 & iq15 :: operator |= (const long &x)
{
   val |= x;
   return *this;
}

//---------------------------------------------------------------------------
// Operators: "^", "^="
//---------------------------------------------------------------------------

inline iq operator ^ (const iq &x, const long &y)
{
   iq temp;
   temp.val = x.val ^ y;
   return temp;
}

inline iq & iq :: operator ^= (const long &x)
{
   val ^= x;
   return *this;
}

inline iq30 operator ^ (const iq30 &x, const long &y)
{
   iq30 temp;
   temp.val = x.val ^ y;
   return temp;
}

inline iq30 & iq30 :: operator ^= (const long &x)
{
   val ^= x;
   return *this;
}

inline iq29 operator ^ (const iq29 &x, const long &y)
{
   iq29 temp;
   temp.val = x.val ^ y;
   return temp;
}

inline iq29 & iq29 :: operator ^= (const long &x)
{
   val ^= x;
   return *this;
}

inline iq28 operator ^ (const iq28 &x, const long &y)
{
   iq28 temp;
   temp.val = x.val ^ y;
   return temp;
}

inline iq28 & iq28 :: operator ^= (const long &x)
{
   val ^= x;
   return *this;
}

inline iq27 operator ^ (const iq27 &x, const long &y)
{
   iq27 temp;
   temp.val = x.val ^ y;
   return temp;
}

inline iq27 & iq27 :: operator ^= (const long &x)
{
   val ^= x;
   return *this;
}

inline iq26 operator ^ (const iq26 &x, const long &y)
{
   iq26 temp;
   temp.val = x.val ^ y;
   return temp;
}

inline iq26 & iq26 :: operator ^= (const long &x)
{
   val ^= x;
   return *this;
}

inline iq25 operator ^ (const iq25 &x, const long &y)
{
   iq25 temp;
   temp.val = x.val ^ y;
   return temp;
}

inline iq25 & iq25 :: operator ^= (const long &x)
{
   val ^= x;
   return *this;
}

inline iq24 operator ^ (const iq24 &x, const long &y)
{
   iq24 temp;
   temp.val = x.val ^ y;
   return temp;
}

inline iq24 & iq24 :: operator ^= (const long &x)
{
   val ^= x;
   return *this;
}

inline iq23 operator ^ (const iq23 &x, const long &y)
{
   iq23 temp;
   temp.val = x.val ^ y;
   return temp;
}

inline iq23 & iq23 :: operator ^= (const long &x)
{
   val ^= x;
   return *this;
}

inline iq22 operator ^ (const iq22 &x, const long &y)
{
   iq22 temp;
   temp.val = x.val ^ y;
   return temp;
}

inline iq22 & iq22 :: operator ^= (const long &x)
{
   val ^= x;
   return *this;
}

inline iq21 operator ^ (const iq21 &x, const long &y)
{
   iq21 temp;
   temp.val = x.val ^ y;
   return temp;
}

inline iq21 & iq21 :: operator ^= (const long &x)
{
   val ^= x;
   return *this;
}

inline iq20 operator ^ (const iq20 &x, const long &y)
{
   iq20 temp;
   temp.val = x.val ^ y;
   return temp;
}

inline iq20 & iq20 :: operator ^= (const long &x)
{
   val ^= x;
   return *this;
}

inline iq19 operator ^ (const iq19 &x, const long &y)
{
   iq19 temp;
   temp.val = x.val ^ y;
   return temp;
}

inline iq19 & iq19 :: operator ^= (const long &x)
{
   val ^= x;
   return *this;
}

inline iq18 operator ^ (const iq18 &x, const long &y)
{
   iq18 temp;
   temp.val = x.val ^ y;
   return temp;
}

inline iq18 & iq18 :: operator ^= (const long &x)
{
   val ^= x;
   return *this;
}

inline iq17 operator ^ (const iq17 &x, const long &y)
{
   iq17 temp;
   temp.val = x.val ^ y;
   return temp;
}

inline iq17 & iq17 :: operator ^= (const long &x)
{
   val ^= x;
   return *this;
}

inline iq16 operator ^ (const iq16 &x, const long &y)
{
   iq16 temp;
   temp.val = x.val ^ y;
   return temp;
}

inline iq16 & iq16 :: operator ^= (const long &x)
{
   val ^= x;
   return *this;
}

inline iq15 operator ^ (const iq15 &x, const long &y)
{
   iq15 temp;
   temp.val = x.val ^ y;
   return temp;
}

inline iq15 & iq15 :: operator ^= (const long &x)
{
   val ^= x;
   return *this;
}

//###########################################################################
#else   // MATH_TYPE == FLOAT_MATH
//###########################################################################
// If FLOAT_MATH is used, the IQmath library function are replaced by
// equivalent floating point operations:
//===========================================================================
typedef   float   iq;
typedef   float   iq30;
typedef   float   iq29;
typedef   float   iq28;
typedef   float   iq27;
typedef   float   iq26;
typedef   float   iq25;
typedef   float   iq24;
typedef   float   iq23;
typedef   float   iq22;
typedef   float   iq21;
typedef   float   iq20;
typedef   float   iq19;
typedef   float   iq18;
typedef   float   iq17;
typedef   float   iq16;
typedef   float   iq15;
//---------------------------------------------------------------------------
#define   IQ(A)         (float) A
#define   IQ30(A)       (float) A
#define   IQ29(A)       (float) A
#define   IQ28(A)       (float) A
#define   IQ27(A)       (float) A
#define   IQ26(A)       (float) A
#define   IQ25(A)       (float) A
#define   IQ24(A)       (float) A
#define   IQ23(A)       (float) A
#define   IQ22(A)       (float) A
#define   IQ21(A)       (float) A
#define   IQ20(A)       (float) A
#define   IQ19(A)       (float) A
#define   IQ18(A)       (float) A
#define   IQ17(A)       (float) A
#define   IQ16(A)       (float) A
#define   IQ15(A)       (float) A
//---------------------------------------------------------------------------
#define   IQtoF(A)      A
#define   IQ30toF(A)    A
#define   IQ29toF(A)    A
#define   IQ28toF(A)    A
#define   IQ27toF(A)    A
#define   IQ26toF(A)    A
#define   IQ25toF(A)    A
#define   IQ24toF(A)    A
#define   IQ23toF(A)    A
#define   IQ22toF(A)    A
#define   IQ21toF(A)    A
#define   IQ20toF(A)    A
#define   IQ19toF(A)    A
#define   IQ18toF(A)    A
#define   IQ17toF(A)    A
#define   IQ16toF(A)    A
#define   IQ15toF(A)    A
//---------------------------------------------------------------------------
extern  float _satf(float A, float Pos, float Neg);
#define   IQsat(A, Pos, Neg)    _satf(A, Pos, Neg)
//---------------------------------------------------------------------------
#define   IQtoIQ30(A)   A
#define   IQtoIQ29(A)   A
#define   IQtoIQ28(A)   A
#define   IQtoIQ27(A)   A
#define   IQtoIQ26(A)   A
#define   IQtoIQ25(A)   A
#define   IQtoIQ24(A)   A
#define   IQtoIQ23(A)   A
#define   IQtoIQ22(A)   A
#define   IQtoIQ21(A)   A
#define   IQtoIQ20(A)   A
#define   IQtoIQ19(A)   A
#define   IQtoIQ18(A)   A
#define   IQtoIQ17(A)   A
#define   IQtoIQ16(A)   A
#define   IQtoIQ15(A)   A
//---------------------------------------------------------------------------
#define   IQ30toIQ(A)   A
#define   IQ29toIQ(A)   A
#define   IQ28toIQ(A)   A
#define   IQ27toIQ(A)   A
#define   IQ26toIQ(A)   A
#define   IQ25toIQ(A)   A
#define   IQ24toIQ(A)   A
#define   IQ23toIQ(A)   A
#define   IQ22toIQ(A)   A
#define   IQ21toIQ(A)   A
#define   IQ20toIQ(A)   A
#define   IQ19toIQ(A)   A
#define   IQ18toIQ(A)   A
#define   IQ17toIQ(A)   A
#define   IQ16toIQ(A)   A
#define   IQ15toIQ(A)   A
//---------------------------------------------------------------------------
#define   IQtoQ15(A)    (short) (A##L * 32768.0L)
#define   IQtoQ14(A)    (short) (A##L * 16384.0L)
#define   IQtoQ13(A)    (short) (A##L * 8192.0L)
#define   IQtoQ12(A)    (short) (A##L * 4096.0L)
#define   IQtoQ11(A)    (short) (A##L * 2048.0L)
#define   IQtoQ10(A)    (short) (A##L * 1024.0L)
#define   IQtoQ9(A)     (short) (A##L * 512.0L)
#define   IQtoQ8(A)     (short) (A##L * 256.0L)
#define   IQtoQ7(A)     (short) (A##L * 128.0L)
#define   IQtoQ6(A)     (short) (A##L * 64.0L)
#define   IQtoQ5(A)     (short) (A##L * 32.0L)
#define   IQtoQ4(A)     (short) (A##L * 16.0L)
#define   IQtoQ3(A)     (short) (A##L * 8.0L)
#define   IQtoQ2(A)     (short) (A##L * 4.0L)
#define   IQtoQ1(A)     (short) (A##L * 2.0L)
//---------------------------------------------------------------------------
#define   Q15toIQ(A)    (((float) A) * 0.000030518)
#define   Q14toIQ(A)    (((float) A) * 0.000061035)
#define   Q13toIQ(A)    (((float) A) * 0.000122070)
#define   Q12toIQ(A)    (((float) A) * 0.000244141)
#define   Q11toIQ(A)    (((float) A) * 0.000488281)
#define   Q10toIQ(A)    (((float) A) * 0.000976563)
#define   Q9toIQ(A)     (((float) A) * 0.001953125)
#define   Q8toIQ(A)     (((float) A) * 0.003906250)
#define   Q7toIQ(A)     (((float) A) * 0.007812500)
#define   Q6toIQ(A)     (((float) A) * 0.015625000)
#define   Q5toIQ(A)     (((float) A) * 0.031250000)
#define   Q4toIQ(A)     (((float) A) * 0.062500000)
#define   Q3toIQ(A)     (((float) A) * 0.125000000)
#define   Q2toIQ(A)     (((float) A) * 0.250000000)
#define   Q1toIQ(A)     (((float) A) * 0.500000000)
//---------------------------------------------------------------------------
#define   IQrmpy(A,B)        (A * B)
#define   IQ30rmpy(A,B)      (A * B)
#define   IQ29rmpy(A,B)      (A * B)
#define   IQ28rmpy(A,B)      (A * B)
#define   IQ27rmpy(A,B)      (A * B)
#define   IQ26rmpy(A,B)      (A * B)
#define   IQ25rmpy(A,B)      (A * B)
#define   IQ24rmpy(A,B)      (A * B)
#define   IQ23rmpy(A,B)      (A * B)
#define   IQ22rmpy(A,B)      (A * B)
#define   IQ21rmpy(A,B)      (A * B)
#define   IQ20rmpy(A,B)      (A * B)
#define   IQ19rmpy(A,B)      (A * B)
#define   IQ18rmpy(A,B)      (A * B)
#define   IQ17rmpy(A,B)      (A * B)
#define   IQ16rmpy(A,B)      (A * B)
#define   IQ15rmpy(A,B)      (A * B)
//---------------------------------------------------------------------------
#define   IQrsmpy(A,B)       (A * B)
#define   IQ30rsmpy(A,B)     (A * B)
#define   IQ29rsmpy(A,B)     (A * B)
#define   IQ28rsmpy(A,B)     (A * B)
#define   IQ27rsmpy(A,B)     (A * B)
#define   IQ26rsmpy(A,B)     (A * B)
#define   IQ25rsmpy(A,B)     (A * B)
#define   IQ24rsmpy(A,B)     (A * B)
#define   IQ23rsmpy(A,B)     (A * B)
#define   IQ22rsmpy(A,B)     (A * B)
#define   IQ21rsmpy(A,B)     (A * B)
#define   IQ20rsmpy(A,B)     (A * B)
#define   IQ19rsmpy(A,B)     (A * B)
#define   IQ18rsmpy(A,B)     (A * B)
#define   IQ17rsmpy(A,B)     (A * B)
#define   IQ16rsmpy(A,B)     (A * B)
#define   IQ15rsmpy(A,B)     (A * B)
//---------------------------------------------------------------------------
#define   IQsin(A)           sin(A)
#define   IQ30sin(A)         sin(A)
#define   IQ29sin(A)         sin(A)
#define   IQ28sin(A)         sin(A)
#define   IQ27sin(A)         sin(A)
#define   IQ26sin(A)         sin(A)
#define   IQ25sin(A)         sin(A)
#define   IQ24sin(A)         sin(A)
#define   IQ23sin(A)         sin(A)
#define   IQ22sin(A)         sin(A)
#define   IQ21sin(A)         sin(A)
#define   IQ20sin(A)         sin(A)
#define   IQ19sin(A)         sin(A)
#define   IQ18sin(A)         sin(A)
#define   IQ17sin(A)         sin(A)
#define   IQ16sin(A)         sin(A)
#define   IQ15sin(A)         sin(A)
//---------------------------------------------------------------------------
#define   IQsinPU(A)         sin(A*6.283185307)
#define   IQ30sinPU(A)       sin(A*6.283185307)
#define   IQ29sinPU(A)       sin(A*6.283185307)
#define   IQ28sinPU(A)       sin(A*6.283185307)
#define   IQ27sinPU(A)       sin(A*6.283185307)
#define   IQ26sinPU(A)       sin(A*6.283185307)
#define   IQ25sinPU(A)       sin(A*6.283185307)
#define   IQ24sinPU(A)       sin(A*6.283185307)
#define   IQ23sinPU(A)       sin(A*6.283185307)
#define   IQ22sinPU(A)       sin(A*6.283185307)
#define   IQ21sinPU(A)       sin(A*6.283185307)
#define   IQ20sinPU(A)       sin(A*6.283185307)
#define   IQ19sinPU(A)       sin(A*6.283185307)
#define   IQ18sinPU(A)       sin(A*6.283185307)
#define   IQ17sinPU(A)       sin(A*6.283185307)
#define   IQ16sinPU(A)       sin(A*6.283185307)
#define   IQ15sinPU(A)       sin(A*6.283185307)
//---------------------------------------------------------------------------
#define   IQcos(A)           cos(A)
#define   IQ30cos(A)         cos(A)
#define   IQ29cos(A)         cos(A)
#define   IQ28cos(A)         cos(A)
#define   IQ27cos(A)         cos(A)
#define   IQ26cos(A)         cos(A)
#define   IQ25cos(A)         cos(A)
#define   IQ24cos(A)         cos(A)
#define   IQ23cos(A)         cos(A)
#define   IQ22cos(A)         cos(A)
#define   IQ21cos(A)         cos(A)
#define   IQ20cos(A)         cos(A)
#define   IQ19cos(A)         cos(A)
#define   IQ18cos(A)         cos(A)
#define   IQ17cos(A)         cos(A)
#define   IQ16cos(A)         cos(A)
#define   IQ15cos(A)         cos(A)
//---------------------------------------------------------------------------
#define   IQcosPU(A)         cos(A*6.283185307)
#define   IQ30cosPU(A)       cos(A*6.283185307)
#define   IQ29cosPU(A)       cos(A*6.283185307)
#define   IQ28cosPU(A)       cos(A*6.283185307)
#define   IQ27cosPU(A)       cos(A*6.283185307)
#define   IQ26cosPU(A)       cos(A*6.283185307)
#define   IQ25cosPU(A)       cos(A*6.283185307)
#define   IQ24cosPU(A)       cos(A*6.283185307)
#define   IQ23cosPU(A)       cos(A*6.283185307)
#define   IQ22cosPU(A)       cos(A*6.283185307)
#define   IQ21cosPU(A)       cos(A*6.283185307)
#define   IQ20cosPU(A)       cos(A*6.283185307)
#define   IQ19cosPU(A)       cos(A*6.283185307)
#define   IQ18cosPU(A)       cos(A*6.283185307)
#define   IQ17cosPU(A)       cos(A*6.283185307)
#define   IQ16cosPU(A)       cos(A*6.283185307)
#define   IQ15cosPU(A)       cos(A*6.283185307)
//---------------------------------------------------------------------------
#define   IQatan(A)          atan(A)
#define   IQ30atan(A)        atan(A)
#define   IQ29atan(A)        atan(A)
#define   IQ28atan(A)        atan(A)
#define   IQ27atan(A)        atan(A)
#define   IQ26atan(A)        atan(A)
#define   IQ25atan(A)        atan(A)
#define   IQ24atan(A)        atan(A)
#define   IQ23atan(A)        atan(A)
#define   IQ22atan(A)        atan(A)
#define   IQ21atan(A)        atan(A)
#define   IQ20atan(A)        atan(A)
#define   IQ19atan(A)        atan(A)
#define   IQ18atan(A)        atan(A)
#define   IQ17atan(A)        atan(A)
#define   IQ16atan(A)        atan(A)
#define   IQ15atan(A)        atan(A)
//---------------------------------------------------------------------------
#define   IQatan2(A,B)       atan2(A,B)
#define   IQ30atan2(A,B)     atan2(A,B)
#define   IQ29atan2(A,B)     atan2(A,B)
#define   IQ28atan2(A,B)     atan2(A,B)
#define   IQ27atan2(A,B)     atan2(A,B)
#define   IQ26atan2(A,B)     atan2(A,B)
#define   IQ25atan2(A,B)     atan2(A,B)
#define   IQ24atan2(A,B)     atan2(A,B)
#define   IQ23atan2(A,B)     atan2(A,B)
#define   IQ22atan2(A,B)     atan2(A,B)
#define   IQ21atan2(A,B)     atan2(A,B)
#define   IQ20atan2(A,B)     atan2(A,B)
#define   IQ19atan2(A,B)     atan2(A,B)
#define   IQ18atan2(A,B)     atan2(A,B)
#define   IQ17atan2(A,B)     atan2(A,B)
#define   IQ16atan2(A,B)     atan2(A,B)
#define   IQ15atan2(A,B)     atan2(A,B)
//---------------------------------------------------------------------------
#define   IQatan2PU(A,B)     ((atan2(A,B)/6.283185307) >= 0.0 ? (atan2(A,B)/6.283185307):1.0 + (atan2(A,B)/6.283185307))
#define   IQ30atan2PU(A,B)   ((atan2(A,B)/6.283185307) >= 0.0 ? (atan2(A,B)/6.283185307):1.0 + (atan2(A,B)/6.283185307))
#define   IQ29atan2PU(A,B)   ((atan2(A,B)/6.283185307) >= 0.0 ? (atan2(A,B)/6.283185307):1.0 + (atan2(A,B)/6.283185307))
#define   IQ28atan2PU(A,B)   ((atan2(A,B)/6.283185307) >= 0.0 ? (atan2(A,B)/6.283185307):1.0 + (atan2(A,B)/6.283185307))
#define   IQ27atan2PU(A,B)   ((atan2(A,B)/6.283185307) >= 0.0 ? (atan2(A,B)/6.283185307):1.0 + (atan2(A,B)/6.283185307))
#define   IQ26atan2PU(A,B)   ((atan2(A,B)/6.283185307) >= 0.0 ? (atan2(A,B)/6.283185307):1.0 + (atan2(A,B)/6.283185307))
#define   IQ25atan2PU(A,B)   ((atan2(A,B)/6.283185307) >= 0.0 ? (atan2(A,B)/6.283185307):1.0 + (atan2(A,B)/6.283185307))
#define   IQ24atan2PU(A,B)   ((atan2(A,B)/6.283185307) >= 0.0 ? (atan2(A,B)/6.283185307):1.0 + (atan2(A,B)/6.283185307))
#define   IQ23atan2PU(A,B)   ((atan2(A,B)/6.283185307) >= 0.0 ? (atan2(A,B)/6.283185307):1.0 + (atan2(A,B)/6.283185307))
#define   IQ22atan2PU(A,B)   ((atan2(A,B)/6.283185307) >= 0.0 ? (atan2(A,B)/6.283185307):1.0 + (atan2(A,B)/6.283185307))
#define   IQ21atan2PU(A,B)   ((atan2(A,B)/6.283185307) >= 0.0 ? (atan2(A,B)/6.283185307):1.0 + (atan2(A,B)/6.283185307))
#define   IQ20atan2PU(A,B)   ((atan2(A,B)/6.283185307) >= 0.0 ? (atan2(A,B)/6.283185307):1.0 + (atan2(A,B)/6.283185307))
#define   IQ19atan2PU(A,B)   ((atan2(A,B)/6.283185307) >= 0.0 ? (atan2(A,B)/6.283185307):1.0 + (atan2(A,B)/6.283185307))
#define   IQ18atan2PU(A,B)   ((atan2(A,B)/6.283185307) >= 0.0 ? (atan2(A,B)/6.283185307):1.0 + (atan2(A,B)/6.283185307))
#define   IQ17atan2PU(A,B)   ((atan2(A,B)/6.283185307) >= 0.0 ? (atan2(A,B)/6.283185307):1.0 + (atan2(A,B)/6.283185307))
#define   IQ16atan2PU(A,B)   ((atan2(A,B)/6.283185307) >= 0.0 ? (atan2(A,B)/6.283185307):1.0 + (atan2(A,B)/6.283185307))
#define   IQ15atan2PU(A,B)   ((atan2(A,B)/6.283185307) >= 0.0 ? (atan2(A,B)/6.283185307):1.0 + (atan2(A,B)/6.283185307))
//---------------------------------------------------------------------------
#define   IQsqrt(A)          sqrt(A)
#define   IQ30sqrt(A)        sqrt(A)
#define   IQ29sqrt(A)        sqrt(A)
#define   IQ28sqrt(A)        sqrt(A)
#define   IQ27sqrt(A)        sqrt(A)
#define   IQ26sqrt(A)        sqrt(A)
#define   IQ25sqrt(A)        sqrt(A)
#define   IQ24sqrt(A)        sqrt(A)
#define   IQ23sqrt(A)        sqrt(A)
#define   IQ22sqrt(A)        sqrt(A)
#define   IQ21sqrt(A)        sqrt(A)
#define   IQ20sqrt(A)        sqrt(A)
#define   IQ19sqrt(A)        sqrt(A)
#define   IQ18sqrt(A)        sqrt(A)
#define   IQ17sqrt(A)        sqrt(A)
#define   IQ16sqrt(A)        sqrt(A)
#define   IQ15sqrt(A)        sqrt(A)
//---------------------------------------------------------------------------
#define   IQisqrt(A)         (1.0/sqrt(A))
#define   IQ30isqrt(A)       (1.0/sqrt(A))
#define   IQ29isqrt(A)       (1.0/sqrt(A))
#define   IQ28isqrt(A)       (1.0/sqrt(A))
#define   IQ27isqrt(A)       (1.0/sqrt(A))
#define   IQ26isqrt(A)       (1.0/sqrt(A))
#define   IQ25isqrt(A)       (1.0/sqrt(A))
#define   IQ24isqrt(A)       (1.0/sqrt(A))
#define   IQ23isqrt(A)       (1.0/sqrt(A))
#define   IQ22isqrt(A)       (1.0/sqrt(A))
#define   IQ21isqrt(A)       (1.0/sqrt(A))
#define   IQ20isqrt(A)       (1.0/sqrt(A))
#define   IQ19isqrt(A)       (1.0/sqrt(A))
#define   IQ18isqrt(A)       (1.0/sqrt(A))
#define   IQ17isqrt(A)       (1.0/sqrt(A))
#define   IQ16isqrt(A)       (1.0/sqrt(A))
#define   IQ15isqrt(A)       (1.0/sqrt(A))
//---------------------------------------------------------------------------
#define   IQint(A)           ((long) A)
#define   IQ30int(A)         ((long) A)
#define   IQ29int(A)         ((long) A)
#define   IQ28int(A)         ((long) A)
#define   IQ27int(A)         ((long) A)
#define   IQ26int(A)         ((long) A)
#define   IQ25int(A)         ((long) A)
#define   IQ24int(A)         ((long) A)
#define   IQ23int(A)         ((long) A)
#define   IQ22int(A)         ((long) A)
#define   IQ21int(A)         ((long) A)
#define   IQ20int(A)         ((long) A)
#define   IQ19int(A)         ((long) A)
#define   IQ18int(A)         ((long) A)
#define   IQ17int(A)         ((long) A)
#define   IQ16int(A)         ((long) A)
#define   IQ15int(A)         ((long) A)
//---------------------------------------------------------------------------
#define   IQfrac(A)          (A - (float)((long) A))
#define   IQ30frac(A)        (A - (float)((long) A))
#define   IQ29frac(A)        (A - (float)((long) A))
#define   IQ28frac(A)        (A - (float)((long) A))
#define   IQ27frac(A)        (A - (float)((long) A))
#define   IQ26frac(A)        (A - (float)((long) A))
#define   IQ25frac(A)        (A - (float)((long) A))
#define   IQ24frac(A)        (A - (float)((long) A))
#define   IQ23frac(A)        (A - (float)((long) A))
#define   IQ22frac(A)        (A - (float)((long) A))
#define   IQ21frac(A)        (A - (float)((long) A))
#define   IQ20frac(A)        (A - (float)((long) A))
#define   IQ19frac(A)        (A - (float)((long) A))
#define   IQ18frac(A)        (A - (float)((long) A))
#define   IQ17frac(A)        (A - (float)((long) A))
#define   IQ16frac(A)        (A - (float)((long) A))
#define   IQ15frac(A)        (A - (float)((long) A))
//---------------------------------------------------------------------------
#define   IQmpyIQX(A, IQA, B, IQB)    (A*B)    
#define   IQ30mpyIQX(A, IQA, B, IQB)  (A*B)    
#define   IQ29mpyIQX(A, IQA, B, IQB)  (A*B)    
#define   IQ28mpyIQX(A, IQA, B, IQB)  (A*B)    
#define   IQ27mpyIQX(A, IQA, B, IQB)  (A*B)    
#define   IQ26mpyIQX(A, IQA, B, IQB)  (A*B)    
#define   IQ25mpyIQX(A, IQA, B, IQB)  (A*B)    
#define   IQ24mpyIQX(A, IQA, B, IQB)  (A*B)    
#define   IQ23mpyIQX(A, IQA, B, IQB)  (A*B)    
#define   IQ22mpyIQX(A, IQA, B, IQB)  (A*B)    
#define   IQ21mpyIQX(A, IQA, B, IQB)  (A*B)    
#define   IQ20mpyIQX(A, IQA, B, IQB)  (A*B)    
#define   IQ19mpyIQX(A, IQA, B, IQB)  (A*B)    
#define   IQ18mpyIQX(A, IQA, B, IQB)  (A*B)    
#define   IQ17mpyIQX(A, IQA, B, IQB)  (A*B)    
#define   IQ16mpyIQX(A, IQA, B, IQB)  (A*B)    
#define   IQ15mpyIQX(A, IQA, B, IQB)  (A*B)    
//---------------------------------------------------------------------------
#define   IQmpyI32(A,B)      (A * (float) B)    
#define   IQ30mpyI32(A,B)    (A * (float) B)
#define   IQ29mpyI32(A,B)    (A * (float) B)
#define   IQ28mpyI32(A,B)    (A * (float) B)
#define   IQ27mpyI32(A,B)    (A * (float) B)
#define   IQ26mpyI32(A,B)    (A * (float) B)
#define   IQ25mpyI32(A,B)    (A * (float) B)
#define   IQ24mpyI32(A,B)    (A * (float) B)
#define   IQ23mpyI32(A,B)    (A * (float) B)
#define   IQ22mpyI32(A,B)    (A * (float) B)
#define   IQ21mpyI32(A,B)    (A * (float) B)
#define   IQ20mpyI32(A,B)    (A * (float) B)
#define   IQ19mpyI32(A,B)    (A * (float) B)
#define   IQ18mpyI32(A,B)    (A * (float) B)
#define   IQ17mpyI32(A,B)    (A * (float) B)
#define   IQ16mpyI32(A,B)    (A * (float) B)
#define   IQ15mpyI32(A,B)    (A * (float) B)
//---------------------------------------------------------------------------
#define   IQmpyI32int(A,B)   ((long) (A * (float) B))
#define   IQ30mpyI32int(A,B) ((long) (A * (float) B))
#define   IQ29mpyI32int(A,B) ((long) (A * (float) B))
#define   IQ28mpyI32int(A,B) ((long) (A * (float) B))
#define   IQ27mpyI32int(A,B) ((long) (A * (float) B))
#define   IQ26mpyI32int(A,B) ((long) (A * (float) B))
#define   IQ25mpyI32int(A,B) ((long) (A * (float) B))
#define   IQ24mpyI32int(A,B) ((long) (A * (float) B))
#define   IQ23mpyI32int(A,B) ((long) (A * (float) B))
#define   IQ22mpyI32int(A,B) ((long) (A * (float) B))
#define   IQ21mpyI32int(A,B) ((long) (A * (float) B))
#define   IQ20mpyI32int(A,B) ((long) (A * (float) B))
#define   IQ19mpyI32int(A,B) ((long) (A * (float) B))
#define   IQ18mpyI32int(A,B) ((long) (A * (float) B))
#define   IQ17mpyI32int(A,B) ((long) (A * (float) B))
#define   IQ16mpyI32int(A,B) ((long) (A * (float) B))
#define   IQ15mpyI32int(A,B) ((long) (A * (float) B))
//---------------------------------------------------------------------------
#define   IQmpyI32frac(A,B)    (A - (float)((long) (A * (float) B)))
#define   IQ30mpyI32frac(A,B)  (A - (float)((long) (A * (float) B)))
#define   IQ29mpyI32frac(A,B)  (A - (float)((long) (A * (float) B)))
#define   IQ28mpyI32frac(A,B)  (A - (float)((long) (A * (float) B)))
#define   IQ27mpyI32frac(A,B)  (A - (float)((long) (A * (float) B)))
#define   IQ26mpyI32frac(A,B)  (A - (float)((long) (A * (float) B)))
#define   IQ25mpyI32frac(A,B)  (A - (float)((long) (A * (float) B)))
#define   IQ24mpyI32frac(A,B)  (A - (float)((long) (A * (float) B)))
#define   IQ23mpyI32frac(A,B)  (A - (float)((long) (A * (float) B)))
#define   IQ22mpyI32frac(A,B)  (A - (float)((long) (A * (float) B)))
#define   IQ21mpyI32frac(A,B)  (A - (float)((long) (A * (float) B)))
#define   IQ20mpyI32frac(A,B)  (A - (float)((long) (A * (float) B)))
#define   IQ19mpyI32frac(A,B)  (A - (float)((long) (A * (float) B)))
#define   IQ18mpyI32frac(A,B)  (A - (float)((long) (A * (float) B)))
#define   IQ17mpyI32frac(A,B)  (A - (float)((long) (A * (float) B)))
#define   IQ16mpyI32frac(A,B)  (A - (float)((long) (A * (float) B)))
#define   IQ15mpyI32frac(A,B)  (A - (float)((long) (A * (float) B)))
//---------------------------------------------------------------------------
#define   IQmag(A,B)         sqrt(A*A + B*B)
#define   IQ30mag(A,B)       sqrt(A*A + B*B)
#define   IQ29mag(A,B)       sqrt(A*A + B*B)
#define   IQ28mag(A,B)       sqrt(A*A + B*B)
#define   IQ27mag(A,B)       sqrt(A*A + B*B)
#define   IQ26mag(A,B)       sqrt(A*A + B*B)
#define   IQ25mag(A,B)       sqrt(A*A + B*B)
#define   IQ24mag(A,B)       sqrt(A*A + B*B)
#define   IQ23mag(A,B)       sqrt(A*A + B*B)
#define   IQ22mag(A,B)       sqrt(A*A + B*B)
#define   IQ21mag(A,B)       sqrt(A*A + B*B)
#define   IQ20mag(A,B)       sqrt(A*A + B*B)
#define   IQ19mag(A,B)       sqrt(A*A + B*B)
#define   IQ18mag(A,B)       sqrt(A*A + B*B)
#define   IQ17mag(A,B)       sqrt(A*A + B*B)
#define   IQ16mag(A,B)       sqrt(A*A + B*B)
#define   IQ15mag(A,B)       sqrt(A*A + B*B)
//---------------------------------------------------------------------------
#define   atoIQ(A)           atof(A)
#define   atoIQ30(A)         atof(A)
#define   atoIQ29(A)         atof(A)
#define   atoIQ28(A)         atof(A)
#define   atoIQ27(A)         atof(A)
#define   atoIQ26(A)         atof(A)
#define   atoIQ25(A)         atof(A)
#define   atoIQ24(A)         atof(A)
#define   atoIQ23(A)         atof(A)
#define   atoIQ22(A)         atof(A)
#define   atoIQ21(A)         atof(A)
#define   atoIQ20(A)         atof(A)
#define   atoIQ19(A)         atof(A)
#define   atoIQ18(A)         atof(A)
#define   atoIQ17(A)         atof(A)
#define   atoIQ16(A)         atof(A)
#define   atoIQ15(A)         atof(A)
//###########################################################################
#endif  // No more.
//###########################################################################

#endif /* __IQMATHLIB_H_INCLUDED__ */
