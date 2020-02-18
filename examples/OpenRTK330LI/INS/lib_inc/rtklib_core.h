#ifndef _RTKLIB_CORE_H_
#define _RTKLIB_CORE_H_

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(WIN32) && defined(_WINDOWS)
#define WIN32
#endif

#define ENAGLO
#define ENACMP
#define ENAGAL

#define NFREQ		(2)
#ifndef NFREQ
#define NFREQ       3                   /* number of carrier frequencies */
#endif
#define NFREQGLO    2                   /* number of carrier frequencies of GLONASS */

#ifndef NEXOBS
#define NEXOBS      0                   /* number of extended obs codes */
#endif

#ifdef WIN32
typedef struct {        /* time struct */
	time_t time;        /* time (s) expressed by standard time_t */
	double sec;         /* fraction of second under 1 s */
} gtime_t;
#else
typedef struct {        /* time struct */
	time_t time;        /* time (s) expressed by standard time_t */
	__attribute__((aligned(8)))double sec; /* fraction of second under 1 s */
} gtime_t;
#endif /*WIN32*/

typedef struct {        /* observation data record */
	gtime_t time;       /* receiver sampling time (GPST) */
	gtime_t eventime;   /* time of event (GPST) */
	int timevalid;      /* time is valid (Valid GNSS fix) for time mark */
	unsigned char sat, rcv; /* satellite/receiver number */
	unsigned char SNR[NFREQ + NEXOBS]; /* signal strength (0.25 dBHz) */
	unsigned char LLI[NFREQ + NEXOBS]; /* loss of lock indicator */
	unsigned char code[NFREQ + NEXOBS]; /* code indicator (CODE_???) */
	unsigned char qualL[NFREQ + NEXOBS]; /* quality of carrier phase measurement */
	unsigned char qualP[NFREQ + NEXOBS]; /* quality of pseudorange measurement */
	double L[NFREQ + NEXOBS]; /* observation data carrier-phase (cycle) */
	double P[NFREQ + NEXOBS]; /* observation data pseudorange (m) */
	float  D[NFREQ + NEXOBS]; /* observation data doppler frequency (Hz) */
} obsd_t;

typedef struct {        /* GPS/QZS/GAL broadcast ephemeris type */
	int sat;            /* satellite number */
	int iode, iodc;      /* IODE,IODC */
	int sva;            /* SV accuracy (URA index) */
	int svh;            /* SV health (0:ok) */
	int week;           /* GPS/QZS: gps week, GAL: galileo week */
	int code;           /* GPS/QZS: code on L2 */
						/* GAL: data source defined as rinex 3.03 */
						/* BDS: data source (0:unknown,1:B1I,2:B1Q,3:B2I,4:B2Q,5:B3I,6:B3Q) */
	int flag;           /* GPS/QZS: L2 P data flag */
						/* BDS: nav type (0:unknown,1:IGSO/MEO,2:GEO) */
	gtime_t toe, toc, ttr; /* Toe,Toc,T_trans */
						/* SV orbit parameters */
	double A, e, i0, OMG0, omg, M0, deln, OMGd, idot;
	double crc, crs, cuc, cus, cic, cis;
	double toes;        /* Toe (s) in week */
	double fit;         /* fit interval (h) */
	double f0, f1, f2;    /* SV clock parameters (af0,af1,af2) */
	double tgd[4];      /* group delay parameters */
						/* GPS/QZS:tgd[0]=TGD */
						/* GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1 */
						/* CMP    :tgd[0]=BGD1,tgd[1]=BGD2 */
	double Adot, ndot;   /* Adot,ndot for CNAV */
} eph_t;

typedef struct {        /* GLONASS broadcast ephemeris type */
	int sat;            /* satellite number */
	int iode;           /* IODE (0-6 bit of tb field) */
	int frq;            /* satellite frequency number */
	int svh, sva, age;    /* satellite health, accuracy, age of operation */
	gtime_t toe;        /* epoch of epherides (gpst) */
	gtime_t tof;        /* message frame time (gpst) */
	double pos[3];      /* satellite position (ecef) (m) */
	double vel[3];      /* satellite velocity (ecef) (m/s) */
	double acc[3];      /* satellite acceleration (ecef) (m/s^2) */
	double taun, gamn;   /* SV clock bias (s)/relative freq bias */
	double dtaun;       /* delay between L1 and L2 (s) */
} geph_t;

#ifdef __cplusplus
}
#endif

#endif
