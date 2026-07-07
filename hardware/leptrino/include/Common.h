#ifndef _COMMON_H
#define _COMMON_H

typedef unsigned char	UCHAR;
typedef signed char		SCHAR;
typedef unsigned short	USHORT;
typedef signed short	SSHORT;
typedef unsigned long	ULONG;
typedef signed long		SLONG;

// Constants
#define OK	1
#define NG	-1

#define SERIAL_SIZE		8
#define P_NAME_SIZE		16
#define F_VER_SIZE		4
#define FREQ_SIZE		6
#define MSG_SIZE		128

// Force Numbers
enum ForceNo {
	FN_Fx = 0,
	FN_Fy,
	FN_Fz,
	FN_Mx,
	FN_My,
	FN_Mz,
	FN_Num
};

// Command and Response Codes
#define CMD_GET_INF		0x2A
#define CMD_GET_LIMIT	0x2B
#define CMD_DATA_START	0x32
#define CMD_DATA_STOP	0x33

#define RES_ERR_OK		0x00
#define RES_ERR_LEN		0x01
#define RES_ERR_UNDEF	0x02
#define RES_ERR_VAL		0x03
#define RES_ERR_STATUS	0x04

// Bit Length Definitions
#define BIT_LEN_7       7
#define BIT_LEN_8       8

// Parity Definitions
#define PAR_ODD         1
#define PAR_EVEN        2
#define PAR_NON         0  // No parity

// Control Characters
#define CHR_DLE         0x10  // Data Link Escape
#define CHR_STX         0x02  // Start of Text
#define CHR_ETX         0x03  // End of Text

// Data Structures
struct ST_CMD_HEAD {
    UCHAR ucLen;
    UCHAR ucTermNo;
    UCHAR ucCmd;
    UCHAR ucRsv;
};

struct ST_RES_HEAD {
    UCHAR ucLen;
    UCHAR ucTermNo;
    UCHAR ucCmd;
    UCHAR ucResult;
};

struct ST_R_GET_INF {
    ST_RES_HEAD stHead;
    SCHAR scPName[P_NAME_SIZE];
    SCHAR scSerial[SERIAL_SIZE];
    SCHAR scFVer[F_VER_SIZE];
    SCHAR scFreq[FREQ_SIZE];
};

struct ST_R_DATA_GET_F {
    ST_RES_HEAD stHead;
    SSHORT ssForce[FN_Num];
    SSHORT ssTemp;
    UCHAR ucStatus;
    UCHAR ucRsv;
};

struct ST_R_LEP_GET_LIMIT {
    ST_RES_HEAD stHead;
    float fLimit[FN_Num];
};

#endif // _COMMON_H
