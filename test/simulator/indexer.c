/* Implementation of the indexer
   https://forge.frm2.tum.de/public/doc/plc/master/singlehtml/index.html#devices
*/

#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include "hw_motor.h"
#include "indexer.h"
#include "logerr_info.h"
#include "cmd.h"

//#define XXOLDPERM 1
/* type codes and sizes */
#define TYPECODE_INDEXER               0
/* The lenght of the indexer data, the longest is
   probably netInfoType4_type with 34 byte */
#define WORDS_SIZE_INDEXER_DATA       17
#define HAS_0518
#ifdef HAS_0518
#define TYPECODE_SPECIALDEVICE_0518 0x0518
#endif
#define TYPECODE_DISCRETEINPUT_1202 0x1202
#define TYPECODE_DISCRETEOUTPUT_1604 0x1604
#define TYPECODE_PARAMDEVICE_5008 0x5008

#define HAS_5010
#ifdef HAS_5010
#define TYPECODE_PARAMDEVICE_5010 0x5010
#endif
#define WORDS_SPECIALDEVICE_0518    0x18
#define WORDS_DISCRETEINPUT_1202     0x2
#define WORDS_DISCRETEOUTPUT_1604    0x4
#define WORDS_PARAMDEVICE_5008       0x8
#define WORDS_PARAMDEVICE_5010      0x10


/* Well known unit codes */
#define UNITCODE_NONE                    0
#define UNITCODE_MM                 0xfd04
#define UNITCODE_DEGREE             0x000C

#define AXISNO_NONE         0
/* axis1, axis2 */
#define  NUM_MOTORS5008     2

/* axis3 axis4 */
#define  NUM_MOTORS5010     2

/*
   Devices for the indexer:
   + the indexer itself
   + 1 special 0518
   + 2 motors 5008
   + 2 1202 errorId for the 5008 motors
   + 2 1202 encoderRaw for the 5008 motors
   + 2 motors 5010
*/
#define NUM_INDEXER_5008_1202 7

#ifdef HAS_0518
#define  NUM_0518           1
#else
#define  NUM_0518           0
#endif

#ifdef HAS_5010
#define  NUM_5010           2
#else
#define  NUM_5010           0
#endif
#define  NUM_DEVICES        (NUM_INDEXER_5008_1202 + NUM_0518 + NUM_5010)


typedef enum {
  idxStatusCodeRESET    = 0,
  idxStatusCodeIDLE     = 1,
  idxStatusCodePOWEROFF = 2,
  idxStatusCodeWARN     = 3,
  idxStatusCodeERR4     = 4,
  idxStatusCodeSTART    = 5,
  idxStatusCodeBUSY     = 6,
  idxStatusCodeSTOP     = 7,
  idxStatusCodeERROR    = 8,
  idxStatusCodeERR9     = 9,
  idxStatusCodeERR10    = 10,
  idxStatusCodeERR11    = 11,
  idxStatusCodeERR12    = 12,
  idxStatusCodeERR13    = 13,
  idxStatusCodeERR14    = 14,
  idxStatusCodeERR15    = 15
} idxStatusCodeType;

typedef enum {
              permPNone,
              permPRead,
              permPRDWR
} permPTyp;

#define modePRDWR permPRDWR

#define uminPRDWR permPRDWR
#define umaxPRDWR permPRDWR

#define homePNone permPNone
#define homePRDWR permPRDWR

#define setPoNone permPNone
#define setPoRDWR permPRDWR

#define vmaxPRead permPRDWR
#define umaxPRDWR permPRDWR
#define umaxPRDWR permPRDWR
#define umaxPRDWR permPRDWR
#define umaxPRDWR permPRDWR

#define hystPNone permPNone
#define hystPRead permPRead

#define folleRead permPRead

#define hvelPNone permPNone
#define hvelPRead permPRead

#define hposPRead permPRead
#define urevPRead permPRead
#define srevPRead permPRead


#define hprocRDWR permPRDWR


#define speedRDWR permPRDWR
#define acclPRDWR permPRDWR
#define hposPRDWR permPRDWR
#define mvVelRDWR permPRDWR

#define uminERDWR permPRDWR
#define umaxERDWR permPRDWR
#define hprocRDWR permPRDWR



/* Param interface
   The bit 15..13 are coded like this: */
#define PARAM_IF_CMD_MASKPARAM_IF_CMD_MASK         0xE000
#define PARAM_IF_CMD_MASKPARAM_IF_IDX_MASK         0x1FFF
#define PARAM_IF_CMD_MASKPARAM_DONE                0x8000

#define PARAM_IF_CMD_INVALID                       0x0000
#define PARAM_IF_CMD_DOREAD                        0x2000
#define PARAM_IF_CMD_DOWRITE                       0x4000
#define PARAM_IF_CMD_BUSY                          0x6000
#define PARAM_IF_CMD_DONE                          0x8000
#define PARAM_IF_CMD_ERR_NO_IDX                    0xA000
#define PARAM_IF_CMD_READONLY                      0xC000
#define PARAM_IF_CMD_RETRY_LATER                   0xE000

/* Param index values */
#define PARAM_IDX_OPMODE_AUTO_UINT32            1
#define PARAM_IDX_MICROSTEPS_UINT32             2
#define PARAM_IDX_ABS_MIN_FLOAT32              30
#define PARAM_IDX_ABS_MAX_FLOAT32              31
#define PARAM_IDX_USR_MIN_FLOAT32              32
#define PARAM_IDX_USR_MAX_FLOAT32              33
#define PARAM_IDX_WRN_MIN_FLOAT32              34
#define PARAM_IDX_WRN_MAX_FLOAT32              35
#define PARAM_IDX_FOLLOWING_ERR_WIN_FLOAT32    55
#define PARAM_IDX_HYTERESIS_FLOAT32            56
#define PARAM_IDX_REFSPEED_FLOAT32             58
#define PARAM_IDX_VBAS_FLOAT32                 59
#define PARAM_IDX_SPEED_FLOAT32                60
#define PARAM_IDX_ACCEL_FLOAT32                61
#define PARAM_IDX_IDLE_CURRENT_FLOAT32         62
#define PARAM_IDX_MOVE_CURRENT_FLOAT32         64
#define PARAM_IDX_MICROSTEPS_FLOAT32           67
#define PARAM_IDX_STEPS_PER_UNIT_FLOAT32       68
#define PARAM_IDX_HOME_POSITION_FLOAT32        69
#define PARAM_IDX_FUN_REFERENCE               133
#define PARAM_IDX_FUN_SET_POSITION            137
#define PARAM_IDX_FUN_MOVE_VELOCITY           142
/* Implementation defined, integer */
/* The following 3 are obsolete */
#define PARAM_IDX_USR_MIN_EN_UINT32           192
#define PARAM_IDX_USR_MAX_EN_UINT32           193
#define PARAM_IDX_HOME_PROC_UINT32            194
/* Implementation defined, floating point */
#define PARAM_IDX_USR_MIN_EN_FLOAT32          218
#define PARAM_IDX_USR_MAX_EN_FLOAT32          219
#define PARAM_IDX_HOME_PROC_FLOAT32           220
#define PARAM_IDX_UNITS_PER_REV_FLOAT32       221
#define PARAM_IDX_STEPS_PER_REV_FLOAT32       222
#define PARAM_IDX_MAX_VELO_FLOAT32            223

#if XXOLDPERM
/*  Which parameters are available */
#define PARAM_AVAIL_0_7_OPMODE_AUTO_UINT32             (1 << (1))
#define PARAM_AVAIL_0_7_MICROSTEPS_UINT32              (1 << (2))

#define PARAM_AVAIL_24_31_ABS_MIN_FLOAT32              (1 << (30-24))
#define PARAM_AVAIL_24_31_ABS_MAX_FLOAT32              (1 << (31-24))

#define PARAM_AVAIL_32_39_USR_MIN_FLOAT32              (1 << (32-32))
#define PARAM_AVAIL_32_39_USR_MAX_FLOAT32              (1 << (33-32))
#define PARAM_AVAIL_32_39_WRN_MIN_FLOAT32              (1 << (34-32))
#define PARAM_AVAIL_32_39_WRN_MAX_FLOAT32              (1 << (35-32))

#define PARAM_AVAIL_48_55_FOLLOWING_ERR_WIN_FLOAT32    (1 << (55-48))

#define PARAM_AVAIL_56_63_HYTERESIS_FLOAT32            (1 << (56-56))
#define PARAM_AVAIL_56_63_REFSPEED_FLOAT32             (1 << (58-56))
#define PARAM_AVAIL_56_63_VBAS_FLOAT32                 (1 << (59-56))
#define PARAM_AVAIL_56_63_SPEED_FLOAT32                (1 << (60-56))
#define PARAM_AVAIL_56_63_ACCEL_FLOAT32                (1 << (61-56))
#define PARAM_AVAIL_56_63_IDLE_CURRENT_FLOAT32         (1 << (62-56))

#define PARAM_AVAIL_64_71_MOVE_CURRENT_FLOAT32         (1 << (64-64))
#define PARAM_AVAIL_64_71_MICROSTEPS_FLOAT32           (1 << (67-64))
#define PARAM_AVAIL_64_71_STEPS_PER_UNIT_FLOAT32       (1 << (68-64))
#define PARAM_AVAIL_64_71_HOME_POSITION_FLOAT32        (1 << (69-64))

#define PARAM_AVAIL_128_135_FUN_REFERENCE              (1 << (133-128))
#define PARAM_AVAIL_136_143_FUN_SET_POSITION           (1 << (137-136))
#define PARAM_AVAIL_136_143_FUN_MOVE_VELOCITY          (1 << (142-136))

#define PARAM_AVAIL_192_199_USR_MIN_EN                 (1 << (192-192))
#define PARAM_AVAIL_192_199_USR_MAX_EN                 (1 << (193-192))
#define PARAM_AVAIL_192_199_HOME_PROC                  (1 << (194-192))

#define PARAM_AVAIL_216_223_UNITS_PER_REV              (1 << (221-216))
#define PARAM_AVAIL_216_223_STEPS_PER_REV              (1 << (222-216))
#define PARAM_AVAIL_216_223_MAX_VELO                   (1 << (223-216))
#endif

/* In the memory bytes, the indexer starts at 64 */
  static unsigned offsetIndexer;

/* Info types of the indexer as seen on the network */
typedef struct {
  uint8_t    typeCode[2];
  uint8_t    size[2];
  uint8_t    offset[2];
  uint8_t    unit[2];
  uint8_t    flags[4];
  uint8_t    absMin[4];
  uint8_t    absMax[4];
} netInfoType0_type;

typedef struct {
  uint8_t   size[2];
} netInfoType1_type;

typedef struct {
  uint8_t   unitcode[2];
} netInfoType3_type;

typedef struct {
  char name[33]; /* leave one byte for trailing '\0' */
  char ascii_null;
} netInfoType4_type;

typedef struct {
  uint8_t  parameters[32]; /* counting 0..31 */
} netInfoType15_type;

/* The special device structure. */
typedef struct {
  /* 2 bytes control, 46 payload */
  uint8_t   control[2];
  uint8_t   value[46];
} netDevice0518interface_type;

typedef struct {
  uint8_t   value[4];
} netDevice1202interface_type;

typedef struct {
  uint8_t   actualValue[4];
  uint8_t   targetValue[4];
} netDevice1604interface_type;

/* The paramDevice structure.
   floating point values are 4 bytes long,
   the whole structure uses 16 bytes, 8 words */
typedef struct {
  uint8_t   actualValue[4];
  uint8_t   targetValue[4];
  uint8_t   statusReasonAux16[2];
  uint8_t   paramCtrl[2];
  uint8_t   paramValue[4];
} netDevice5008interface_type;

typedef struct {
  netDevice5008interface_type dev5008;
  netDevice1202interface_type dev1202errorID;
  netDevice1202interface_type dev1202encoderRaw;
#ifdef HAS_1604HOMPROC
  netDevice1604interface_type dev1604homProc;
#endif
} netDevice5008_1202_1604_Interface_type;

/* struct as seen on the network = in memory
 * data must be stored using  uintToNet/doubleToNet
 * and retrieved netToUint/netToDouble */
typedef struct {
  uint8_t   actualValue[8];
  uint8_t   targetValue[8];
  uint8_t   statusReasonAux32[4];
  uint8_t   errorID[2];
  uint8_t   paramCtrl[2];
  uint8_t   paramValue[8];
} netDevice5010interface_type;

/* Info type as seen here locally in memory */
typedef struct {
  uint16_t  typeCode;
  uint16_t  sizeInBytes;
  uint16_t  unitCode;
  uint16_t  axisNo;
#if XXOLDPERM
  uint8_t   paramAvail[32]; /* counting 0..31 */
#else
  permPTyp  permP[256];
#endif
  char      devName[34];
  char      auxName[24][34];
  float     absMin;
  float     absMax;
} indexerDeviceAbsStraction_type;


indexerDeviceAbsStraction_type indexerDeviceAbsStraction[NUM_DEVICES] =
  {
    /* device 0, the indexer itself */
    { TYPECODE_INDEXER, 2*WORDS_SIZE_INDEXER_DATA,
      UNITCODE_NONE, AXISNO_NONE,
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      "indexer",
      { "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", ""},
      0.0, 0.0
    },
#ifdef HAS_0518
    /* special device */
    { TYPECODE_SPECIALDEVICE_0518, 2*WORDS_SPECIALDEVICE_0518,
      UNITCODE_NONE, AXISNO_NONE,
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      "DbgStrToMcu",
      { "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", ""},
      0.0, 0.0
    },
#endif
    { TYPECODE_PARAMDEVICE_5008, 2*WORDS_PARAMDEVICE_5008,
      UNITCODE_MM, 1,
#if XXOLDPERM
      {/*  0..7    */ PARAM_AVAIL_0_7_OPMODE_AUTO_UINT32,
       /*  8..15   */ 0,
       /* 16..23   */ 0,
       /* 24..31   */ 0,
       /* 32..39   */ PARAM_AVAIL_32_39_USR_MIN_FLOAT32 | PARAM_AVAIL_32_39_USR_MAX_FLOAT32,
       /* 40..47   */ 0,
       /* 48..55   */ 0,
       /* 56..63   */ PARAM_AVAIL_56_63_SPEED_FLOAT32 | PARAM_AVAIL_56_63_ACCEL_FLOAT32 | PARAM_AVAIL_56_63_HYTERESIS_FLOAT32,
       /* 64..71   */ PARAM_AVAIL_64_71_HOME_POSITION_FLOAT32,
       /* 72..79   */ 0,
       /* 80..87   */ 0,
       /* 88..95   */ 0,
       /* 96..103  */ 0,
       /* 104..111 */ 0,
       /* 112..119 */ 0,
       /* 120..127 */ 0,
       /* 128..135 */ PARAM_AVAIL_128_135_FUN_REFERENCE,
       /* 136..143 */ PARAM_AVAIL_136_143_FUN_SET_POSITION | PARAM_AVAIL_136_143_FUN_MOVE_VELOCITY,
       /* 144..151 */ 0,
       /* 152..159 */ 0,
       /* 160..167 */ 0,
       /* 168..175 */ 0,
       /* 176..183 */ 0,
       /* 184..191 */ 0,
       /* 192..199 */ PARAM_AVAIL_192_199_USR_MIN_EN | PARAM_AVAIL_192_199_USR_MAX_EN | PARAM_AVAIL_192_199_HOME_PROC,
       /* 200..207 */ 0,
       /* 208..215 */ 0,
       /* 216..223 */ PARAM_AVAIL_216_223_UNITS_PER_REV | PARAM_AVAIL_216_223_STEPS_PER_REV | PARAM_AVAIL_216_223_MAX_VELO,
       /* 224..231 */ 0,
       /* 232..239 */ 0,
       /* 240..247 */ 0,
       /* 248..255 */ 0
      },
#else
    {/*   0..4   */ permPNone, modePRDWR, permPNone, permPNone, permPNone,
     /*   5..9   */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  10..14  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  15..19  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  20..24  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  25..29  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  30..34  */ permPNone, permPNone, uminPRDWR, umaxPRDWR, permPNone,
     /*  35..39  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  40..44  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  45..49  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  50..54  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  55..59  */ permPNone, hystPRead, permPNone, hvelPNone, permPNone,
     /*  60..64  */ speedRDWR, acclPRDWR, permPNone, permPNone, permPNone,
     /*  65..69  */ permPNone, permPNone, permPNone, permPNone, hposPRead,
     /*  70..74  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  75..79  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  80..84  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  85..89  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  90..94  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  95..99  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 100..105 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 105..109 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 110..114 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 115..119 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 120..124 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 125..129 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 130..134 */ permPNone, permPNone, permPNone, homePRDWR, permPNone,
     /* 135..139 */ permPNone, permPNone, setPoRDWR, permPNone, permPNone,
     /* 140..144 */ permPNone, permPNone, mvVelRDWR, permPNone, permPNone,
     /* 145..159 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 150..154 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 155..159 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 160..164 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 165..169 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 170..174 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 175..179 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 180..184 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 185..189 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 190..194 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 195..199 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 200..204 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 205..209 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 210..214 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 215..219 */ permPNone, permPNone, permPNone, uminERDWR, umaxERDWR,
     /* 220..224 */ hprocRDWR, urevPRead, srevPRead, vmaxPRead, permPNone,
     /* 225..229 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 230..234 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 235..239 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 240..204 */ permPNone, permPNone, permPNone, permPNone, permPNone
    },
#endif
      "SimAxis1",
      { "notHomed", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", ""},
      5.0, 175.0
    },
    /* device for errorID */
    { TYPECODE_DISCRETEINPUT_1202, 2*WORDS_DISCRETEINPUT_1202,
      UNITCODE_NONE, 1,
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      "errorID",
      { "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", ""},
      0.0, 0.0
    },
    /* device for encoderRaw */
    { TYPECODE_DISCRETEINPUT_1202, 2*WORDS_DISCRETEINPUT_1202,
      UNITCODE_NONE, 1,
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      "encoderRaw",
      { "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", ""},
      0.0, 0.0
    },
#ifdef HAS_1604HOMPROC
    /* device for homProc */
    { TYPECODE_DISCRETEOUTPUT_1604, 2*WORDS_DISCRETEOUTPUT_1604,
      UNITCODE_NONE, 1,
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      "homProc",
      { "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", ""},
      0.0, 0.0
    },
#endif
    { TYPECODE_PARAMDEVICE_5008, 2*WORDS_PARAMDEVICE_5008,
      UNITCODE_DEGREE, 2,
#if XXOLDPERM
      {/*  0..7    */ PARAM_AVAIL_0_7_OPMODE_AUTO_UINT32,
       /*  8..15   */ 0,
       /* 16..23   */ 0,
       /* 24..31   */ 0,
       /* 32..39   */ PARAM_AVAIL_32_39_USR_MIN_FLOAT32 | PARAM_AVAIL_32_39_USR_MAX_FLOAT32,
       /* 40..47   */ 0,
       /* 48..55   */ 0,
       /* 56..63   */ PARAM_AVAIL_56_63_SPEED_FLOAT32 | PARAM_AVAIL_56_63_ACCEL_FLOAT32 | PARAM_AVAIL_56_63_HYTERESIS_FLOAT32,
       /* 64..71   */ 0,
       /* 72..79   */ 0,
       /* 80..87   */ 0,
       /* 88..95   */ 0,
       /* 96..103  */ 0,
       /* 104..111 */ 0,
       /* 112..119 */ 0,
       /* 120..127 */ 0,
       /* 128..135 */ 0,
       /* 136..143 */ PARAM_AVAIL_136_143_FUN_SET_POSITION | PARAM_AVAIL_136_143_FUN_MOVE_VELOCITY,
       /* 144..151 */ 0,
       /* 152..159 */ 0,
       /* 160..167 */ 0,
       /* 168..175 */ 0,
       /* 176..183 */ 0,
       /* 184..191 */ 0,
       /* 192..199 */ PARAM_AVAIL_192_199_USR_MIN_EN | PARAM_AVAIL_192_199_USR_MAX_EN,
       /* 200..207 */ 0,
       /* 208..215 */ 0,
       /* 216..223 */ 0,
       /* 224..231 */ 0,
       /* 232..239 */ 0,
       /* 240..247 */ 0,
       /* 248..255 */ 0
      },
#else
    {/*   0..4   */ permPNone, modePRDWR, permPNone, permPNone, permPNone,
     /*   5..9   */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  10..14  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  15..19  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  20..24  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  25..29  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  30..34  */ permPNone, permPNone, uminPRDWR, umaxPRDWR, permPNone,
     /*  35..39  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  40..44  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  45..49  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  50..54  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  55..59  */ permPNone, hystPRead, permPNone, hvelPNone, permPNone,
     /*  60..64  */ speedRDWR, acclPRDWR, permPNone, permPNone, permPNone,
     /*  65..69  */ permPNone, permPNone, permPNone, permPNone, hposPRead,
     /*  70..74  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  75..79  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  80..84  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  85..89  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  90..94  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  95..99  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 100..105 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 105..109 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 110..114 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 115..119 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 120..124 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 125..129 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 130..134 */ permPNone, permPNone, permPNone, homePNone, permPNone,
     /* 135..139 */ permPNone, permPNone, setPoRDWR, permPNone, permPNone,
     /* 140..144 */ permPNone, permPNone, mvVelRDWR, permPNone, permPNone,
     /* 145..159 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 150..154 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 155..159 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 160..164 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 165..169 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 170..174 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 175..179 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 180..184 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 185..189 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 190..194 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 195..199 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 200..204 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 205..209 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 210..214 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 215..219 */ permPNone, permPNone, permPNone, uminERDWR, umaxERDWR,
     /* 220..224 */ hprocRDWR, urevPRead, srevPRead, vmaxPRead, permPNone,
     /* 225..229 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 230..234 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 235..239 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 240..204 */ permPNone, permPNone, permPNone, permPNone, permPNone
    },
#endif
      "RotAxis2",
      { "notHomed", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", ""},
      -181.0, +181.0
    },
    /* device for errorID */
    { TYPECODE_DISCRETEINPUT_1202, 2*WORDS_DISCRETEINPUT_1202,
      UNITCODE_NONE, 2,
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      "errorID",
      { "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", ""},
      0.0, 0.0
    },
    /* device for encoderRaw */
    { TYPECODE_DISCRETEINPUT_1202, 2*WORDS_DISCRETEINPUT_1202,
      UNITCODE_NONE, 2,
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      "encoderRaw",
      { "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", ""},
      0.0, 0.0
    },
#ifdef HAS_1604HOMPROC
    /* device for homProc */
    { TYPECODE_DISCRETEOUTPUT_1604, 2*WORDS_DISCRETEOUTPUT_1604,
      UNITCODE_NONE, 2,
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      "homProc",
      { "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", ""},
      0.0, 0.0
    },
#endif
#ifdef HAS_5010
    { TYPECODE_PARAMDEVICE_5010, 2*WORDS_PARAMDEVICE_5010,
      UNITCODE_MM, 3,
#if XXOLDPERM
      {/*  0..7    */ PARAM_AVAIL_0_7_OPMODE_AUTO_UINT32,
       /*  8..15   */ 0,
       /* 16..23   */ 0,
       /* 24..31   */ 0,
       /* 32..39   */ PARAM_AVAIL_32_39_USR_MIN_FLOAT32 | PARAM_AVAIL_32_39_USR_MAX_FLOAT32,
       /* 40..47   */ 0,
       /* 48..55   */ 0,
       /* 56..63   */ PARAM_AVAIL_56_63_SPEED_FLOAT32 | PARAM_AVAIL_56_63_ACCEL_FLOAT32 | PARAM_AVAIL_56_63_HYTERESIS_FLOAT32,
       /* 64..71   */ PARAM_AVAIL_64_71_HOME_POSITION_FLOAT32,
       /* 72..79   */ 0,
       /* 80..87   */ 0,
       /* 88..95   */ 0,
       /* 96..103  */ 0,
       /* 104..111 */ 0,
       /* 112..119 */ 0,
       /* 120..127 */ 0,
       /* 128..135 */ PARAM_AVAIL_128_135_FUN_REFERENCE,
       /* 136..143 */ PARAM_AVAIL_136_143_FUN_MOVE_VELOCITY,
       /* 144..151 */ 0,
       /* 152..159 */ 0,
       /* 160..167 */ 0,
       /* 168..175 */ 0,
       /* 176..183 */ 0,
       /* 184..191 */ 0,
       /* 192..199 */ PARAM_AVAIL_192_199_HOME_PROC,
       /* 200..207 */ 0,
       /* 208..215 */ 0,
       /* 216..223 */ 0,
       /* 224..231 */ 0,
       /* 232..239 */ 0,
       /* 240..247 */ 0,
       /* 248..255 */ 0
      },
#else
    {/*   0..4   */ permPNone, modePRDWR, permPNone, permPNone, permPNone,
     /*   5..9   */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  10..14  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  15..19  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  20..24  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  25..29  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  30..34  */ permPNone, permPNone, uminPRDWR, umaxPRDWR, permPNone,
     /*  35..39  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  40..44  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  45..49  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  50..54  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  55..59  */ permPNone, hystPRead, permPNone, hvelPNone, permPNone,
     /*  60..64  */ speedRDWR, acclPRDWR, permPNone, permPNone, permPNone,
     /*  65..69  */ permPNone, permPNone, permPNone, permPNone, hposPRead,
     /*  70..74  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  75..79  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  80..84  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  85..89  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  90..94  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  95..99  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 100..105 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 105..109 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 110..114 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 115..119 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 120..124 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 125..129 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 130..134 */ permPNone, permPNone, permPNone, homePRDWR, permPNone,
     /* 135..139 */ permPNone, permPNone, setPoNone, permPNone, permPNone,
     /* 140..144 */ permPNone, permPNone, mvVelRDWR, permPNone, permPNone,
     /* 145..159 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 150..154 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 155..159 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 160..164 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 165..169 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 170..174 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 175..179 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 180..184 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 185..189 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 190..194 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 195..199 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 200..204 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 205..209 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 210..214 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 215..219 */ permPNone, permPNone, permPNone, uminERDWR, umaxERDWR,
     /* 220..224 */ hprocRDWR, urevPRead, srevPRead, vmaxPRead, permPNone,
     /* 225..229 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 230..234 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 235..239 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 240..204 */ permPNone, permPNone, permPNone, permPNone, permPNone
    },
#endif
      "Axis5010-3",
      { "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", "notHomed"},
      0, +173.0
    },
    { TYPECODE_PARAMDEVICE_5010, 2*WORDS_PARAMDEVICE_5010,
      UNITCODE_MM, 4,
#if XXOLDPERM
      {/*  0..7    */ PARAM_AVAIL_0_7_OPMODE_AUTO_UINT32,
       /*  8..15   */ 0,
       /* 16..23   */ 0,
       /* 24..31   */ 0,
       /* 32..39   */ PARAM_AVAIL_32_39_USR_MIN_FLOAT32 | PARAM_AVAIL_32_39_USR_MAX_FLOAT32,
       /* 40..47   */ 0,
       /* 48..55   */ 0,
       /* 56..63   */ PARAM_AVAIL_56_63_SPEED_FLOAT32 | PARAM_AVAIL_56_63_ACCEL_FLOAT32 | PARAM_AVAIL_56_63_HYTERESIS_FLOAT32,
       /* 64..71   */ PARAM_AVAIL_64_71_HOME_POSITION_FLOAT32,
       /* 72..79   */ 0,
       /* 80..87   */ 0,
       /* 88..95   */ 0,
       /* 96..103  */ 0,
       /* 104..111 */ 0,
       /* 112..119 */ 0,
       /* 120..127 */ 0,
       /* 128..135 */ 0,
       /* 136..143 */ PARAM_AVAIL_136_143_FUN_MOVE_VELOCITY,
       /* 144..151 */ 0,
       /* 152..159 */ 0,
       /* 160..167 */ 0,
       /* 168..175 */ 0,
       /* 176..183 */ 0,
       /* 184..191 */ 0,
       /* 192..199 */ 0,
       /* 200..207 */ 0,
       /* 208..215 */ 0,
       /* 216..223 */ 0,
       /* 224..231 */ 0,
       /* 232..239 */ 0,
       /* 240..247 */ 0,
       /* 248..255 */ 0
      },
#else
    {/*   0..4   */ permPNone, modePRDWR, permPNone, permPNone, permPNone,
     /*   5..9   */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  10..14  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  15..19  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  20..24  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  25..29  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  30..34  */ permPNone, permPNone, uminPRDWR, umaxPRDWR, permPNone,
     /*  35..39  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  40..44  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  45..49  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  50..54  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  55..59  */ permPNone, hystPRead, permPNone, hvelPNone, permPNone,
     /*  60..64  */ speedRDWR, acclPRDWR, permPNone, permPNone, permPNone,
     /*  65..69  */ permPNone, permPNone, permPNone, permPNone, hposPRead,
     /*  70..74  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  75..79  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  80..84  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  85..89  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  90..94  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /*  95..99  */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 100..105 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 105..109 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 110..114 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 115..119 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 120..124 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 125..129 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 130..134 */ permPNone, permPNone, permPNone, homePNone, permPNone,
     /* 135..139 */ permPNone, permPNone, setPoNone, permPNone, permPNone,
     /* 140..144 */ permPNone, permPNone, mvVelRDWR, permPNone, permPNone,
     /* 145..159 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 150..154 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 155..159 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 160..164 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 165..169 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 170..174 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 175..179 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 180..184 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 185..189 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 190..194 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 195..199 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 200..204 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 205..209 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 210..214 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 215..219 */ permPNone, permPNone, permPNone, uminERDWR, umaxERDWR,
     /* 220..224 */ hprocRDWR, urevPRead, srevPRead, vmaxPRead, permPNone,
     /* 225..229 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 230..234 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 235..239 */ permPNone, permPNone, permPNone, permPNone, permPNone,
     /* 240..204 */ permPNone, permPNone, permPNone, permPNone, permPNone
    },
#endif
      "Axis5010-4",
      { "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", "notHomed"},
      0, +163.0
    }
#endif
  };


typedef struct
{
  double fHysteresis;
  int   nHomProc;
  int   nOldHomProc;
} cmd_Motor_cmd_type;


static union {
  uint8_t  memoryBytes[1024];
  struct {
    uint8_t  magic[4];
    uint8_t offset[2];
    uint8_t indexer_ack[2];
    /* Area for the indexer. union of the different info types */
    union {
      netInfoType0_type  infoType0;
      netInfoType1_type  infoType1;
      netInfoType3_type  infoType3;
      netInfoType4_type  infoType4;
      netInfoType15_type infoType15;
    } indexer;
#ifdef HAS_0518
    netDevice0518interface_type special0518; /* 42 bytes for ASCII to the simulator */
#endif
    /* Remember that motor[0] is defined, but never used */
    netDevice5008_1202_1604_Interface_type motors5008_1202_1604[NUM_MOTORS5008];
    netDevice5010interface_type motors5010[NUM_MOTORS5010];
  } memoryStruct;
} netData;

static int initDone = 0;

/* values commanded to the motor */
static cmd_Motor_cmd_type cmd_Motor_cmd[MAX_AXES];


static unsigned netToUint(void *data, size_t lenInPlc)
{
  const uint8_t *src = (const uint8_t*)data;
  unsigned uRes;
  if (lenInPlc == 2) {
    uRes = (unsigned)src[0] + ((unsigned)src[1] << 8);
    return uRes;
  } else if ((lenInPlc == 4) || (lenInPlc == 8)) {
    /* We don't use the full range of 64 bit integers,
       only values up to 2^31 */
    uRes = (unsigned)src[0] + ((unsigned)src[1] << 8) +
      ((unsigned)src[2] << 16) + ((unsigned)src[3] << 24);
    return uRes;
  }
  return 0;
}

static double netToDouble(void *data, size_t lenInPlc)
{
  const uint8_t *src = (const uint8_t*)data;
  if (lenInPlc == 4) {
    union {
      volatile uint32_t uRes;
      volatile float    fRes;
    } dst;
    dst.uRes = (uint32_t)src[0] + ((uint32_t)src[1] << 8) +
      ((uint32_t)src[2] << 16) + ((uint32_t)src[3] << 24);
    return (double)dst.fRes;
  } else if (lenInPlc == 8) {
    union {
      volatile uint64_t uRes;
      volatile double   fRes;
    } dst;
    dst.uRes = (uint64_t)src[0] + ((uint64_t)src[1] << 8) +
      ((uint64_t)src[2] << 16) + ((uint64_t)src[3] << 24) +
      ((uint64_t)src[4] << 32) + ((uint64_t)src[5] << 40) +
      ((uint64_t)src[6] << 48) + ((uint64_t)src[7] << 56);
    return dst.fRes;
  } else {
    return 0.0;
  }
}

#define NETTODOUBLE(n)   netToDouble(&(n),      sizeof(n))

static void doubleToNet(const double value, void *data, size_t lenInPlc)
{
  uint8_t *dst = (uint8_t*)data;
  if (lenInPlc == 4) {
    union {
      volatile uint32_t uRes;
      volatile float    fRes;
    } src;
    src.fRes = (float)value;
    dst[0] = (uint8_t)src.uRes;
    dst[1] = (uint8_t)(src.uRes >> 8);
    dst[2] = (uint8_t)(src.uRes >> 16);
    dst[3] = (uint8_t)(src.uRes >> 24);
  } else if (lenInPlc == 8) {
    union {
      volatile uint64_t uRes;
      volatile double   fRes;
    } src;
    src.fRes = value;
    dst[0] = (uint8_t)src.uRes;
    dst[1] = (uint8_t)(src.uRes >> 8);
    dst[2] = (uint8_t)(src.uRes >> 16);
    dst[3] = (uint8_t)(src.uRes >> 24);
    dst[4] = (uint8_t)(src.uRes >> 32);
    dst[5] = (uint8_t)(src.uRes >> 40);
    dst[6] = (uint8_t)(src.uRes >> 48);
    dst[7] = (uint8_t)(src.uRes >> 56);
  } else {
    memset(data, 0, lenInPlc);
  }
}

static void uintToNet(const unsigned value, void *data, size_t lenInPlc)
{
  uint8_t *dst = (uint8_t*)data;
  memset(data, 0, lenInPlc);
  if (lenInPlc == 2) {
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
  } else if ((lenInPlc == 4) || (lenInPlc == 8)) {
    /* We don't use the full range of 64 bit integers,
       only values up to 2^31 */
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
    dst[2] = (uint8_t)(value >> 16);
    dst[3] = (uint8_t)(value >> 24);
  }
}

#define NETTOUINT(n)     netToUint(&(n),      sizeof(n))
#define UINTTONET(v,n)   uintToNet(v, &(n),   sizeof(n))
#define DOUBLETONET(v,n) doubleToNet(v, &(n), sizeof(n))

static void init(void)
{
  if (initDone) return;
  memset (&netData, 0, sizeof(netData));
  DOUBLETONET(2015.02, netData.memoryStruct.magic);

  offsetIndexer =
    (unsigned)((void*)&netData.memoryStruct.indexer_ack - (void*)&netData);
  UINTTONET(offsetIndexer, netData.memoryStruct.offset);

  LOGINFO3("%s/%s:%d offsetIndexer=%u\n",
           __FILE__, __FUNCTION__, __LINE__,
           offsetIndexer);
  initDone = 1;
}

static void init_axis(int axis_no)
{
  static char init_done[MAX_AXES];
  const double MRES = 1;
  const double UREV = 60.0; /* mm/revolution */
  const double SREV = 2000.0; /* ticks/revolution */
  const double ERES = UREV / SREV;

  double ReverseMRES = (double)1.0/MRES;

  if (axis_no >= MAX_AXES || axis_no < 0) {
    return;
  }
  if (!init_done[axis_no]) {
    struct motor_init_values motor_init_values;
    double valueLow = -1.0 * ReverseMRES;
    double valueHigh = 186.0 * ReverseMRES;
    memset(&motor_init_values, 0, sizeof(motor_init_values));
    motor_init_values.ReverseERES = MRES/ERES;
    motor_init_values.ParkingPos = (10 + axis_no/10.0);
    motor_init_values.MaxHomeVelocityAbs = 10 * ReverseMRES;
    motor_init_values.lowHardLimitPos = valueLow;
    motor_init_values.highHardLimitPos = valueHigh;
    motor_init_values.hWlowPos = valueLow;
    motor_init_values.hWhighPos = valueHigh;

    hw_motor_init(axis_no,
                  &motor_init_values,
                  sizeof(motor_init_values));

    setMRES_23(axis_no, UREV);
    setMRES_24(axis_no, SREV);
    cmd_Motor_cmd[axis_no].fHysteresis = 0.1;
    if (axis_no == 4)
      setAxisHomed(axis_no, 1);
    cmd_Motor_cmd[axis_no].nHomProc = 1;
    cmd_Motor_cmd[axis_no].nOldHomProc = cmd_Motor_cmd[axis_no].nHomProc;
    setHomePos(axis_no, 0.1);
    setNxtMoveVelocity(axis_no, 50 + axis_no / 10.0);
    setNxtMoveAcceleration(axis_no, 1 + axis_no / 10.0);
    setMaxHomeVelocityAbs(axis_no, 10);
    /* Simulated limit switches, take from indexer table */
    {
      int tmp_axis_no = 1;
      unsigned devNum; /* 0 is the indexer */
      for (devNum = 1; devNum < NUM_DEVICES; devNum++) {
        LOGINFO3("%s/%s:%d axis_no=%d tmp_axis_no=%d devNum=%u typeCode=0x%x\n",
                 __FILE__, __FUNCTION__, __LINE__,
                 axis_no, tmp_axis_no, devNum,
                 indexerDeviceAbsStraction[devNum].typeCode);
        switch (indexerDeviceAbsStraction[devNum].typeCode) {
        case 0x5008:
        case 0x500C:
        case 0x5010:
          {
            if (tmp_axis_no == axis_no) {
              double absMax = indexerDeviceAbsStraction[devNum].absMax;
              double absMin = indexerDeviceAbsStraction[devNum].absMin;
              int hasUserMin = 0;
#if XXOLDPERM
              hasUserMin = indexerDeviceAbsStraction[devNum].paramAvail[4] &
                PARAM_AVAIL_32_39_USR_MIN_FLOAT32;
 #else
              unsigned param_idx = PARAM_IDX_USR_MIN_FLOAT32;
              hasUserMin = indexerDeviceAbsStraction[devNum].permP[param_idx] != permPNone;
 #endif
              setHighHardLimitPos(axis_no, absMax);
              setLowHardLimitPos(axis_no,  absMin);
              LOGINFO3("%s/%s:%d axis_no=%d tmp_axis_no=%d USR_MIN_BIT=%u\n",
                       __FILE__, __FUNCTION__, __LINE__,
                       axis_no, tmp_axis_no,
                       hasUserMin);

              if (hasUserMin) {
                setHighSoftLimitPos(tmp_axis_no, absMax - 1.0);
                setLowSoftLimitPos(tmp_axis_no, absMin + 1.0);
                setEnableHighSoftLimit(tmp_axis_no, 1);
                setEnableLowSoftLimit(tmp_axis_no, 1);
              }
            }
            tmp_axis_no++;
          }
          break;
        default:
          break;
        }
      }
    }
    setAmplifierPercent(axis_no, 1);
    init_done[axis_no] = 1;
  }
}


static void
indexerMotorStatusRead5008(unsigned devNum,
                           unsigned motor_axis_no,
                           unsigned numAuxBits,
                           netDevice5008interface_type *pIndexerDevice5008interface)
{
  unsigned ret = 0;
  unsigned statusReasonAux;
  idxStatusCodeType idxStatusCode;
  /* The following only works on little endian (?)*/
  statusReasonAux = NETTOUINT(pIndexerDevice5008interface->statusReasonAux16);
  idxStatusCode = (idxStatusCodeType)(statusReasonAux >> 12);
  /* The following would be run in an own task in a PLC program.
     For the simulator, we hook the code into the read request
     RESET, START and STOP are commands from IOC.
     RESET is even the "wakeup" state.
  */

  switch (idxStatusCode) {
  case idxStatusCodeRESET:
    init_axis((int)motor_axis_no);
    motorStop(motor_axis_no);
    set_nErrorId(motor_axis_no, 0);
    break;
  case idxStatusCodeSTART:
    movePosition(motor_axis_no,
                 NETTODOUBLE(pIndexerDevice5008interface->targetValue),
                 0, /* int relative, */
                 getNxtMoveVelocity(motor_axis_no),
                 getNxtMoveAcceleration(motor_axis_no));
    break;
  case idxStatusCodeSTOP:
    motorStop(motor_axis_no);
    break;
  default:
    ;
  }
  statusReasonAux = 0;

  /* reason bits */
  if (getPosLimitSwitch(motor_axis_no))
    statusReasonAux |= 0x0800;
  if (getNegLimitSwitch(motor_axis_no))
    statusReasonAux |= 0x0400;
  {
    unsigned auxBitIdx = 0;
    for (auxBitIdx = 0; auxBitIdx < numAuxBits; auxBitIdx++) {
      const char *auxBitName = (const char*)&indexerDeviceAbsStraction[devNum].auxName[auxBitIdx];

      if (!strcmp("homing", auxBitName)) {
        if (isMotorHoming(motor_axis_no)) {
          statusReasonAux |= 1 << auxBitIdx;
        }
      } else if (!strcmp("homed", auxBitName)) {
        int bValue = getAxisHomed(motor_axis_no);
        LOGINFO6("%s/%s:%d motor_axis_no=%u auxBitIdx=%u homed=%d\n",
                 __FILE__, __FUNCTION__, __LINE__,
                 motor_axis_no, auxBitIdx, bValue);
        if (bValue) {
          statusReasonAux |= 1 << auxBitIdx;
        }
      } else if (!strcmp("notHomed", auxBitName)) {
        int bValue = getAxisHomed(motor_axis_no);
        LOGINFO6("%s/%s:%d motor_axis_no=%u auxBitIdx=%u homed=%d\n",
                 __FILE__, __FUNCTION__, __LINE__,
                 motor_axis_no, auxBitIdx, bValue);
        if (!bValue) {
          statusReasonAux |= 1 << auxBitIdx;
        }
      }
    }
  }

  /* the status bits */
  if (get_bError(motor_axis_no))
    idxStatusCode = idxStatusCodeERROR;
  else if (!getAmplifierOn(motor_axis_no))
    idxStatusCode = idxStatusCodePOWEROFF;
  else if (isMotorMoving(motor_axis_no))
    idxStatusCode = idxStatusCodeBUSY;
  else if(statusReasonAux)
    idxStatusCode = idxStatusCodeWARN;
  else
    idxStatusCode = idxStatusCodeIDLE;

  ret = statusReasonAux | (idxStatusCode << 12);;
  UINTTONET(ret, pIndexerDevice5008interface->statusReasonAux16);
}

#ifdef HAS_5010
static void
indexerMotorStatusRead5010(unsigned devNum,
                           unsigned motor_axis_no,
                           unsigned numAuxBits,
                           netDevice5010interface_type *pIndexerDevice5010interface)
{
  unsigned statusReasonAux32;
  idxStatusCodeType idxStatusCode;
  statusReasonAux32 = NETTOUINT(pIndexerDevice5010interface->statusReasonAux32);
  idxStatusCode = (idxStatusCodeType)(statusReasonAux32 >> 28);

  switch (idxStatusCode) {
  case idxStatusCodeRESET:
    init_axis((int)motor_axis_no);
    motorStop(motor_axis_no);
    set_nErrorId(motor_axis_no, 0);
    memset(&pIndexerDevice5010interface->errorID, 0,
           sizeof(pIndexerDevice5010interface->errorID));
    break;
  case idxStatusCodeSTART:
    movePosition(motor_axis_no,
                 NETTODOUBLE(pIndexerDevice5010interface->targetValue),
                 0, /* int relative, */
                 getNxtMoveVelocity(motor_axis_no),
                 getNxtMoveAcceleration(motor_axis_no));
    LOGINFO3("%s/%s:%d motor_axis_no=%u idxStatusCodeSTART isMotorMoving=%d\n",
             __FILE__, __FUNCTION__, __LINE__,
             motor_axis_no,
             isMotorMoving(motor_axis_no));
    break;
  case idxStatusCodeSTOP:
    motorStop(motor_axis_no);
    break;
  default:
    ;
  }
  /* Build a new status word, start with 0 and fill in
     the bits */
  statusReasonAux32 = 0;
  /* reason bits */
  if (getPosLimitSwitch(motor_axis_no))
    statusReasonAux32 |= 0x08000000;
  if (getNegLimitSwitch(motor_axis_no))
    statusReasonAux32 |= 0x04000000;
  {
    unsigned auxBitIdx = 0;
    for (auxBitIdx = 0; auxBitIdx < numAuxBits; auxBitIdx++) {
      const char *auxBitName = (const char*)&indexerDeviceAbsStraction[devNum].auxName[auxBitIdx];
      LOGINFO6("%s/%s:%d motor_axis_no=%u auxBitIdx=%u auxBitName=%s\n",
               __FILE__, __FUNCTION__, __LINE__,
               motor_axis_no, auxBitIdx, auxBitName);

      if (!strcmp("homing", auxBitName)) {
        if (isMotorHoming(motor_axis_no)) {
          statusReasonAux32 |= 1 << auxBitIdx;
        }
      } else if (!strcmp("homed", auxBitName)) {
        int bValue = getAxisHomed(motor_axis_no);
        LOGINFO6("%s/%s:%d motor_axis_no=%u auxBitIdx=%u homed=%d\n",
                 __FILE__, __FUNCTION__, __LINE__,
                 motor_axis_no, auxBitIdx, bValue);
        if (bValue) {
          statusReasonAux32 |= 1 << auxBitIdx;
        }
      } else if (!strcmp("notHomed", auxBitName)) {
        int bValue = getAxisHomed(motor_axis_no);
        LOGINFO6("%s/%s:%d motor_axis_no=%u auxBitIdx=%u homed=%d\n",
                 __FILE__, __FUNCTION__, __LINE__,
                 motor_axis_no, auxBitIdx, bValue);
        if (!bValue) {
          statusReasonAux32 |= 1 << auxBitIdx;
        }
      }
    }
  }

  /* the status bits */
  if (get_bError(motor_axis_no)) {
    idxStatusCode = idxStatusCodeERROR;
    UINTTONET(get_nErrorId(motor_axis_no),
              pIndexerDevice5010interface->errorID);
  } else if (!getAmplifierOn(motor_axis_no))
    idxStatusCode = idxStatusCodePOWEROFF;
  else if (isMotorMoving(motor_axis_no))
    idxStatusCode = idxStatusCodeBUSY;
  else if (statusReasonAux32 & 0x0C000000)
    idxStatusCode = idxStatusCodeWARN;
  else
    idxStatusCode = idxStatusCodeIDLE;

  statusReasonAux32 |= (idxStatusCode << 28);
  UINTTONET(statusReasonAux32,
            pIndexerDevice5010interface->statusReasonAux32);
}
#endif

/* Reads a parameter.
   All return values are returned as double,
   the call will convert into int32 or real32 if needed
*/
static unsigned
indexerMotorParamRead(unsigned motor_axis_no,
                      unsigned paramIndex,
                      double *fRet)

{
  uint16_t ret = PARAM_IF_CMD_DONE | paramIndex;
  if (motor_axis_no >= MAX_AXES) {
    return PARAM_IF_CMD_ERR_NO_IDX;
  }

  init_axis((int)motor_axis_no);

  switch(paramIndex) {
  case PARAM_IDX_USR_MIN_FLOAT32:
    *fRet = getLowSoftLimitPos(motor_axis_no);
    return ret;
  case PARAM_IDX_USR_MAX_FLOAT32:
    *fRet = getHighSoftLimitPos(motor_axis_no);
    return ret;
  case PARAM_IDX_OPMODE_AUTO_UINT32:
    /* We return the value as a double:
       0.0 means power on (normal)
       1.0 means power off (special) */
    *fRet = getAmplifierOn(motor_axis_no) ? 0.0 : 1.0;
    return ret;
  case PARAM_IDX_HYTERESIS_FLOAT32:
    *fRet = cmd_Motor_cmd[motor_axis_no].fHysteresis;
    return ret;
  case PARAM_IDX_SPEED_FLOAT32:
    *fRet = getNxtMoveVelocity(motor_axis_no);
    return ret;
  case PARAM_IDX_ACCEL_FLOAT32:
    *fRet = getNxtMoveAcceleration(motor_axis_no);
    return ret;
  case PARAM_IDX_HOME_POSITION_FLOAT32:
    *fRet = getHomePos(motor_axis_no);
    return ret;
  case PARAM_IDX_FUN_MOVE_VELOCITY:
    /* Use half of the velocity as "JVEL" */
    *fRet = getNxtMoveVelocity(motor_axis_no) / 2.0;
    return ret;
  case PARAM_IDX_USR_MIN_EN_UINT32:
  case PARAM_IDX_USR_MIN_EN_FLOAT32:
    *fRet = getEnableLowSoftLimit(motor_axis_no);
    return ret;
  case PARAM_IDX_USR_MAX_EN_UINT32:
  case PARAM_IDX_USR_MAX_EN_FLOAT32:
    *fRet = getEnableHighSoftLimit(motor_axis_no);
    return ret;
  case PARAM_IDX_HOME_PROC_UINT32:
  case PARAM_IDX_HOME_PROC_FLOAT32:
    *fRet = cmd_Motor_cmd[motor_axis_no].nHomProc;
    return ret;
  case PARAM_IDX_UNITS_PER_REV_FLOAT32:
    *fRet = getMRES_23(motor_axis_no);
    return ret;
  case PARAM_IDX_STEPS_PER_REV_FLOAT32:
    *fRet = getMRES_24(motor_axis_no);
    return ret;
  case PARAM_IDX_MAX_VELO_FLOAT32:
    return ret;

  default:
    break;
  }

  return PARAM_IF_CMD_ERR_NO_IDX | paramIndex;
}

/* Writes a parameter.
   the call will convert into int32 or real32 if needed
*/
static unsigned
indexerMotorParamWrite(unsigned motor_axis_no,
                       unsigned paramIndex,
                       double fValue)

{
  uint16_t ret = PARAM_IF_CMD_DONE | paramIndex;
  if (motor_axis_no >= MAX_AXES) {
    return PARAM_IF_CMD_ERR_NO_IDX;
  }

  init_axis((int)motor_axis_no);
  LOGINFO3("%s/%s:%d motor_axis_no=%u paramIndex=%u, fValue=%f\n",
           __FILE__, __FUNCTION__, __LINE__,
           motor_axis_no, paramIndex, fValue);

  switch(paramIndex) {
  case PARAM_IDX_OPMODE_AUTO_UINT32:
    {
      /* param = 1 means amplifier off.
         param = 0 means "on", then ramp up */
      int setOnNotOff = fValue ? 0 : 1;
      int isOnNotOff  = getAmplifierOn(motor_axis_no);
      if (isOnNotOff != setOnNotOff) {
        setAmplifierPercent(motor_axis_no, setOnNotOff ? 97 : 0);
      }
      return ret;
    }
  case PARAM_IDX_HOME_POSITION_FLOAT32:
    setHomePos(motor_axis_no, fValue);
    return ret;
  case PARAM_IDX_USR_MAX_FLOAT32:
    setHighSoftLimitPos(motor_axis_no, fValue);
    return ret;
  case PARAM_IDX_USR_MIN_FLOAT32:
    setLowSoftLimitPos(motor_axis_no, fValue);
    return ret;
  case PARAM_IDX_SPEED_FLOAT32:
    {
      double maxVelocity = getMaxVelocity(motor_axis_no);
      if (maxVelocity && fValue > maxVelocity)
        fValue = maxVelocity;
      setNxtMoveVelocity(motor_axis_no, fValue);
    }
    return ret;
  case PARAM_IDX_ACCEL_FLOAT32:
    {
      double maxAcceleration = getMaxAcceleration(motor_axis_no);
      if (maxAcceleration && fValue > maxAcceleration)
        fValue = maxAcceleration;
      setNxtMoveAcceleration(motor_axis_no, fValue);
    }
    return ret;
  case PARAM_IDX_USR_MIN_EN_FLOAT32:
    setEnableLowSoftLimit(motor_axis_no, (int)fValue);
    ret = PARAM_IF_CMD_DONE | paramIndex;
    return ret;
  case PARAM_IDX_USR_MAX_EN_FLOAT32:
    setEnableHighSoftLimit(motor_axis_no, (int)fValue);
    ret = PARAM_IF_CMD_DONE | paramIndex;
    return ret;
  case PARAM_IDX_HOME_PROC_FLOAT32:
    cmd_Motor_cmd[motor_axis_no].nHomProc = (int)fValue;
    ret = PARAM_IF_CMD_DONE | paramIndex;
    return ret;
  default:
    break;
  }

  return PARAM_IF_CMD_ERR_NO_IDX | paramIndex;
}

static void
indexerMotorParamInterface(unsigned motor_axis_no,
                           unsigned offset,
                           unsigned lenInPlcPara)
{
  unsigned uValue = netToUint(&netData.memoryBytes[offset], 2);
  unsigned paramCommand = uValue & PARAM_IF_CMD_MASKPARAM_IF_CMD_MASK;
  unsigned paramIndex = uValue & PARAM_IF_CMD_MASKPARAM_IF_IDX_MASK;
  uint16_t ret = (uint16_t)uValue;
  if (paramCommand == PARAM_IF_CMD_INVALID) {
    uValue = PARAM_IF_CMD_DONE;
    paramCommand = uValue & PARAM_IF_CMD_MASKPARAM_IF_CMD_MASK;
    paramIndex = uValue & PARAM_IF_CMD_MASKPARAM_IF_IDX_MASK;
    ret = (uint16_t)uValue;
    /* PILS specification say that the PLC set it to done
       after initialization */
    LOGINFO6("%s/%s:%d motor_axis_no=%u setting to PARAM_IF_CMD_DONE\n",
             __FILE__, __FUNCTION__, __LINE__, motor_axis_no);
    uintToNet(uValue, &netData.memoryBytes[offset], 2);
  }
  if (!(paramCommand & PARAM_IF_CMD_MASKPARAM_DONE)) {
    LOGINFO6("%s/%s:%d NOT done motor_axis_no=%u paramIndex=%u offset=%u paramCommand=0x%4x uValue=0x%x lenInPlcPara=%u\n",
             __FILE__, __FUNCTION__, __LINE__,
             motor_axis_no, paramIndex, offset,
             paramCommand, uValue, lenInPlcPara);
  }
  if (paramCommand == PARAM_IF_CMD_DOWRITE) {
    double fValue;
    int iValue;
    fValue = netToDouble(&netData.memoryBytes[offset + 2], lenInPlcPara);
    iValue = netToUint(&netData.memoryBytes[offset + 2], lenInPlcPara);
    ret = PARAM_IF_CMD_ERR_NO_IDX;
    switch(paramIndex) {
    case PARAM_IDX_OPMODE_AUTO_UINT32:
      /* Comes as an uint via the wire */
      fValue =  (double)netToUint(&netData.memoryBytes[offset + 2], lenInPlcPara);
      /* fall through */
    case PARAM_IDX_HOME_POSITION_FLOAT32:
    case PARAM_IDX_USR_MAX_FLOAT32:
    case PARAM_IDX_USR_MIN_FLOAT32:
    case PARAM_IDX_SPEED_FLOAT32:
    case PARAM_IDX_ACCEL_FLOAT32:
    case PARAM_IDX_USR_MIN_EN_FLOAT32:
    case PARAM_IDX_USR_MAX_EN_FLOAT32:
    case PARAM_IDX_HOME_PROC_FLOAT32:
      ret = indexerMotorParamWrite(motor_axis_no, paramIndex, fValue);
      /* the spec says, that the response must have the "real" value!
         Get it by doing a read */
      paramCommand = PARAM_IF_CMD_DOREAD;
      break;
    case PARAM_IDX_FUN_REFERENCE:
      {
        int direction = 0;
        double max_velocity = 10;
        double acceleration = 3;
#if 0
        moveHome(motor_axis_no,
                 direction,
                 max_velocity,
                 acceleration);
#else
        moveHomeProc(motor_axis_no,
                     direction,
                     cmd_Motor_cmd[motor_axis_no].nHomProc,
                     getHomePos(motor_axis_no),
                     max_velocity,
                     acceleration);

#endif
        ret = PARAM_IF_CMD_DONE | paramIndex;
      }
      break;
    case PARAM_IDX_FUN_MOVE_VELOCITY:
      {
        int direction = fValue >= 0.0;
        double fVelocity = fabs(fValue);
        if (!fVelocity) {
          fVelocity = getNxtMoveVelocity(motor_axis_no);
        } else {
          /* Update Vel_RB in IOC.
             Not sure, if this is what PILS wants,
             but for the moment we need it for TC950 */
          setNxtMoveVelocity(motor_axis_no, fValue);
        }
        moveVelocity(motor_axis_no,
                     direction,
                     fVelocity,
                     getNxtMoveAcceleration(motor_axis_no));
        ret = PARAM_IF_CMD_DONE | paramIndex;
      }
      break;
    case PARAM_IDX_FUN_SET_POSITION:
      setMotorPos(motor_axis_no, fValue);
      ret = PARAM_IF_CMD_DONE | paramIndex;
      break;
    case PARAM_IDX_USR_MIN_EN_UINT32:
      setEnableLowSoftLimit(motor_axis_no, iValue);
      ret = PARAM_IF_CMD_DONE | paramIndex;
      break;
    case PARAM_IDX_USR_MAX_EN_UINT32:
      setEnableHighSoftLimit(motor_axis_no, iValue);
      ret = PARAM_IF_CMD_DONE | paramIndex;
      break;
    case PARAM_IDX_HOME_PROC_UINT32:
      cmd_Motor_cmd[motor_axis_no].nHomProc = iValue;
      ret = PARAM_IF_CMD_DONE | paramIndex;
      break;
    }
    /* put DONE (or ERROR) into the process image */
    uintToNet(ret, &netData.memoryBytes[offset], 2);
  }
  if (paramCommand == PARAM_IF_CMD_DOREAD) {
    double fRet;
    /* do the read */
    ret = indexerMotorParamRead(motor_axis_no,
                                paramIndex,
                                &fRet);
    /* put DONE (or ERROR) into the process image */
    uintToNet(ret, &netData.memoryBytes[offset], 2);
    if ((ret & PARAM_IF_CMD_MASKPARAM_IF_CMD_MASK) == PARAM_IF_CMD_DONE) {
      switch(paramIndex) {
      case PARAM_IDX_OPMODE_AUTO_UINT32:
      case PARAM_IDX_USR_MIN_EN_UINT32:
      case PARAM_IDX_USR_MAX_EN_UINT32:
      case PARAM_IDX_HOME_PROC_UINT32:
        uintToNet((unsigned)fRet, &netData.memoryBytes[offset + 2], lenInPlcPara);
        break;
      default:
        doubleToNet(fRet, &netData.memoryBytes[offset + 2], lenInPlcPara);
        break;
      }
    }
  }
  LOGINFO6("%s/%s:%d indexerMotorParamRead motor_axis_no=%u paramIndex=%u uValue=%x ret=%x\n",
           __FILE__, __FUNCTION__, __LINE__,
           motor_axis_no, paramIndex, uValue, ret);

}

static int indexerHandleIndexerCmd(unsigned offset,
                                   unsigned lenInPlc,
                                   unsigned uValue)
{
  unsigned devNum = uValue & 0xFF;
  unsigned infoType = (uValue >> 8) & 0x7F;
  unsigned maxDevNum = NUM_DEVICES - 1;
  LOGINFO6("%s/%s:%d offset=%u lenInPlc=%u uValue=0x%x devNum=%u maxDevNum=%u infoType=%u\n",
           __FILE__, __FUNCTION__, __LINE__,
           offset, lenInPlc,
           uValue, devNum, maxDevNum, infoType);
  memset(&netData.memoryStruct.indexer, 0, sizeof(netData.memoryStruct.indexer));
  UINTTONET(uValue, netData.memoryStruct.indexer_ack);
  if (devNum >= NUM_DEVICES) {
    netData.memoryStruct.indexer_ack[1] |= 0x80; /* ACK in high byte */
    return 0;
  }
  switch (infoType) {
  case 0:
    /* get values from device table */
    UINTTONET(indexerDeviceAbsStraction[devNum].typeCode,
              netData.memoryStruct.indexer.infoType0.typeCode);
    UINTTONET(indexerDeviceAbsStraction[devNum].sizeInBytes,
              netData.memoryStruct.indexer.infoType0.size);
    UINTTONET(indexerDeviceAbsStraction[devNum].unitCode,
              netData.memoryStruct.indexer.infoType0.unit);
    DOUBLETONET(indexerDeviceAbsStraction[devNum].absMin,
                netData.memoryStruct.indexer.infoType0.absMin);
    DOUBLETONET(indexerDeviceAbsStraction[devNum].absMax,
                netData.memoryStruct.indexer.infoType0.absMax);
    if (!devNum) {
      /* The indexer himself. */
      unsigned flags = 0x80000000; /* extended indexer */
      UINTTONET(offsetIndexer, netData.memoryStruct.indexer.infoType0.offset);
      UINTTONET(flags, netData.memoryStruct.indexer.infoType0.flags);
    } else {
      unsigned auxIdx;
      unsigned flags = 0;
      unsigned maxAuxIdx;
      maxAuxIdx = sizeof(indexerDeviceAbsStraction[devNum].auxName) /
        sizeof(indexerDeviceAbsStraction[devNum].auxName[0]);

      for (auxIdx = 0; auxIdx < maxAuxIdx; auxIdx++) {
        if (strlen(indexerDeviceAbsStraction[devNum].auxName[auxIdx])) {
          flags |= (1 << auxIdx);
        }
        LOGINFO6("%s/%s:%d devNum=%u auxIdx=%u flagsLow=0x%x\n",
                 __FILE__, __FUNCTION__, __LINE__,
                 devNum, auxIdx, flags);
      }
      UINTTONET(flags, netData.memoryStruct.indexer.infoType0.flags);

      /* Calculate offset */
      {
        unsigned offset =
          (unsigned)((void*)&netData.memoryStruct.indexer - (void*)&netData);
        unsigned tmpDevNum = 0;
        LOGINFO6("%s/%s:%d devNum=%u offset=%u\n",
                 __FILE__, __FUNCTION__, __LINE__,
                 devNum, offset);
        while (tmpDevNum < devNum) {
          LOGINFO6("%s/%s:%d devNum=%u tmpDevNum=%u (%s) offsetW=%u (0x%x)\n",
                   __FILE__, __FUNCTION__, __LINE__,
                   devNum, tmpDevNum,
                   indexerDeviceAbsStraction[tmpDevNum].devName,
                   offset,offset);
          offset += indexerDeviceAbsStraction[tmpDevNum].sizeInBytes;
          tmpDevNum++;
        }
        UINTTONET(offset, netData.memoryStruct.indexer.infoType0.offset);
      }
    }
    LOGINFO6("%s/%s:%d devNum=%u netData=%p indexer=%p delta=%u typeCode=%x sizeW=%u offsetW=%u flagsLow=0x%x ack=0x%x\n",
             __FILE__, __FUNCTION__, __LINE__,
             devNum, &netData, &netData.memoryStruct.indexer,
             (unsigned)((void*)&netData.memoryStruct.indexer - (void*)&netData),
             NETTOUINT(netData.memoryStruct.indexer.infoType0.typeCode),
             NETTOUINT(netData.memoryStruct.indexer.infoType0.size),
             NETTOUINT(netData.memoryStruct.indexer.infoType0.offset),
             NETTOUINT(netData.memoryStruct.indexer.infoType0.flags),
             NETTOUINT(netData.memoryStruct.indexer_ack));

    netData.memoryStruct.indexer_ack[1] |= 0x80; /* ACK in high byte */
    return 0;
  case 1:
    UINTTONET(indexerDeviceAbsStraction[devNum].sizeInBytes,
              netData.memoryStruct.indexer.infoType1.size);
    netData.memoryStruct.indexer_ack[1] |= 0x80; /* ACK in high byte */
    return 0;
#if 0
  case 3:
    UINTTONET(indexerDeviceAbsStraction[devNum].unitCode,
              netData.memoryStruct.indexer.infoType3.unitcode);
    netData.memoryStruct.indexer_ack[1] |= 0x80; /* ACK in high byte */
    return 0;
#endif
  case 4:
    /* get values from device table */
    strncpy(&netData.memoryStruct.indexer.infoType4.name[0],
            indexerDeviceAbsStraction[devNum].devName,
            sizeof(netData.memoryStruct.indexer.infoType4.name));
    LOGINFO3("%s/%s:%d devName=%s idxName=%s\n",
             __FILE__, __FUNCTION__, __LINE__,
             indexerDeviceAbsStraction[devNum].devName,
             &netData.memoryStruct.indexer.infoType4.name[0]);
    netData.memoryStruct.indexer_ack[1] |= 0x80; /* ACK in high byte */
    return 0;
  case 5: /* version */
  case 6: /* author 1 */
  case 7: /* author 2 */
    netData.memoryStruct.indexer_ack[1] |= 0x80; /* ACK in high byte */
    return 0;
  case 15:
#if XXOLDPERM
    memcpy(&netData.memoryStruct.indexer.infoType15,
           indexerDeviceAbsStraction[devNum].paramAvail,
           sizeof(netData.memoryStruct.indexer.infoType15));
#else
    {
      unsigned byteIdx;
      memset(&netData.memoryStruct.indexer.infoType15,
             0,
             sizeof(netData.memoryStruct.indexer.infoType15));
      for (byteIdx = 0;
           byteIdx < sizeof(netData.memoryStruct.indexer.infoType15.parameters);
           byteIdx ++) {
        unsigned bitIdx;
        unsigned parameter = 0;
        size_t size_of_param;
        size_of_param = 8 * sizeof(netData.memoryStruct.indexer.infoType15.parameters[0]);
        for (bitIdx = 0; bitIdx < size_of_param; bitIdx++) {
          permPTyp permP;
          unsigned param_idx = byteIdx * size_of_param + bitIdx;
          permP = indexerDeviceAbsStraction[devNum].permP[param_idx];
          LOGINFO6("%s/%s:%d devNum=%u param_idx=%u byteIdx=%02u bitIdx=%u permP=%d\n",
                   __FILE__, __FUNCTION__, __LINE__,
                   devNum, param_idx, byteIdx, bitIdx, (int)permP);
          if (permP != permPNone) parameter |= (1 << bitIdx);
        }
        LOGINFO6("%s/%s:%d devNum=%u byteIdx=%02u parameter=0x%02X\n",
                 __FILE__, __FUNCTION__, __LINE__,
                 devNum, byteIdx, parameter);
        netData.memoryStruct.indexer.infoType15.parameters[byteIdx] = parameter;
      }
    }
#endif
    netData.memoryStruct.indexer_ack[1] |= 0x80; /* ACK in high byte */
    return 0;
  default:
    if (infoType >= 16 && infoType <= 39) {
      /* Support for aux bits 23..0 */
      strncpy(&netData.memoryStruct.indexer.infoType4.name[0],
              indexerDeviceAbsStraction[devNum].auxName[infoType-16],
              sizeof(netData.memoryStruct.indexer.infoType4.name));
      netData.memoryStruct.indexer_ack[1] |= 0x80; /* ACK in high byte */
      return 0;
    }
    RETURN_ERROR_OR_DIE(__LINE__,
                        "%s/%s:%d invalid infoType. infoType=%d",
                        __FILE__, __FUNCTION__, __LINE__,
                        infoType);
    exit(2);
  }
  return __LINE__;
}

/*************************************************************************/
int indexerHandleADS_ADR_getUInt(unsigned adsport,
                                 unsigned offset,
                                 unsigned lenInPlc,
                                 unsigned *uValue)
{
  unsigned ret;
  init();
  if (offset + lenInPlc >= sizeof(netData))
    return __LINE__;
  if (offset & 0x1) /* Must be even */
    return __LINE__;
  ret = netToUint(&netData.memoryBytes[offset], lenInPlc);
  *uValue = ret;
  /*
    LOGINFO3("%s/%s:%d adsport=%u offset=%u lenInPlc=%u mot1=%u ret=%u (0x%x)\n",
    __FILE__, __FUNCTION__, __LINE__,
    adsport,
    offset,
    lenInPlc,
    offsetMotor1StatusReasonAux,
    ret, ret);
  */
  return 0;
}

int indexerHandleADS_ADR_putUInt(unsigned adsport,
                                 unsigned offset,
                                 unsigned lenInPlc,
                                 unsigned uValue)
{
  init();
  LOGINFO6("%s/%s:%d adsport=%u offset=%u lenInPlc=%u uValue=%u (%x)\n",
           __FILE__, __FUNCTION__, __LINE__,
           adsport,
           offset,
           lenInPlc,
           uValue, uValue);
  if (offset == offsetIndexer) {
    return indexerHandleIndexerCmd(offset, lenInPlc, uValue);
  } else if (offset < (sizeof(netData) / sizeof(uint16_t))) {
    uintToNet(uValue, &netData.memoryBytes[offset], lenInPlc);
    return 0;
  }
  LOGERR("%s/%s:%d adsport=%u offset=%u lenInPlc=%u uValue=%u (%x)sizeof=%lu\n",
         __FILE__, __FUNCTION__, __LINE__,
         adsport, offset, lenInPlc, uValue, uValue,
         (unsigned long)(sizeof(netData) / sizeof(uint16_t)));

  return __LINE__;
}

int indexerHandleADS_ADR_getFloat(unsigned adsport,
                                  unsigned offset,
                                  unsigned lenInPlc,
                                  double *fValue)
{
  double fRet;
  init();
  if (offset + lenInPlc >= sizeof(netData))
    return 1;
  if ((lenInPlc == 4) || (lenInPlc == 8)) {
    fRet = netToDouble(&netData.memoryBytes[offset], lenInPlc);
    *fValue = fRet;
    return 0;
  }
  return __LINE__;
}


int indexerHandleADS_ADR_putFloat(unsigned adsport,
                                  unsigned offset,
                                  unsigned lenInPlc,
                                  double fValue)
{
  init();
  LOGINFO3("%s/%s:%d adsport=%u offset=%u lenInPlc=%u fValue=%f\n",
           __FILE__, __FUNCTION__, __LINE__,
           adsport,
           offset,
           lenInPlc,
           fValue);
  if (offset + lenInPlc >= sizeof(netData))
    return 1;
  if ((lenInPlc == 4) || (lenInPlc == 8)) {
    doubleToNet(fValue, &netData.memoryBytes[offset], lenInPlc);
    return 0;
  }
  return __LINE__;
};


int indexerHandleADS_ADR_getString(unsigned adsport,
                                   unsigned offset,
                                   unsigned lenInPlc,
                                   char **sValue)
{
  init();
  if (offset + lenInPlc > sizeof(netData)) {
    RETURN_ERROR_OR_DIE(__LINE__,
                        "%s/%s:%d out of range: offset=%u lenInPlc=%u",
                        __FILE__, __FUNCTION__, __LINE__,
                        offset, lenInPlc);
  }
  *sValue = (char *)&netData.memoryBytes[offset];
  return 0;
};

int indexerHandleADS_ADR_getMemory(unsigned adsport,
                                   unsigned offset,
                                   unsigned lenInPlc,
                                   void *buf)
{
  init();
  if (offset + lenInPlc > sizeof(netData)) {
    RETURN_ERROR_OR_DIE(__LINE__,
                        "%s/%s:%d out of range: offset=%u lenInPlc=%u (0x%x)",
                        __FILE__, __FUNCTION__, __LINE__,
                        offset, lenInPlc, lenInPlc);
  }
  memcpy(buf, &netData.memoryBytes[offset], lenInPlc);
  return 0;
};

int indexerHandleADS_ADR_setMemory(unsigned adsport,
                                   unsigned offset,
                                   unsigned lenInPlc,
                                   void *buf)
{
  init();
  if (offset + lenInPlc > sizeof(netData)) {
    RETURN_ERROR_OR_DIE(__LINE__,
                        "%s/%s:%d out of range: offset=%u lenInPlc=%u (0x%x)",
                        __FILE__, __FUNCTION__, __LINE__,
                        offset, lenInPlc, lenInPlc);
  }
  memcpy(&netData.memoryBytes[offset], buf, lenInPlc);
  return 0;
};

void indexerHandlePLCcycle(void)
{
  static int pLCcycleInitDone = 0;
  unsigned devNum = 0;
  init();
  while (devNum < NUM_DEVICES) {
    LOGINFO6("%s/%s:%d devNum=%u typeCode=0x%x\n",
             __FILE__, __FUNCTION__, __LINE__,
             devNum, indexerDeviceAbsStraction[devNum].typeCode);

    switch (indexerDeviceAbsStraction[devNum].typeCode) {
    case TYPECODE_DISCRETEINPUT_1202:
      {
        unsigned axisNo = indexerDeviceAbsStraction[devNum].axisNo;
        if (axisNo) {
          unsigned motor5008Num = axisNo - 1;
          if (!(strcmp(indexerDeviceAbsStraction[devNum].devName, "errorID"))) {
            unsigned errorID = get_nErrorId(axisNo);
            LOGINFO6("%s/%s:%d devNum=%u axisNo=%u motor5008Num=%u errorID=%u\n",
                     __FILE__, __FUNCTION__, __LINE__,
                     devNum, axisNo, motor5008Num, errorID);
            UINTTONET(errorID,
                      netData.memoryStruct.motors5008_1202_1604[motor5008Num].dev1202errorID.value);
          } else if (!(strcmp(indexerDeviceAbsStraction[devNum].devName, "encoderRaw"))) {
            int encoderRaw = (int)getEncoderPos(axisNo);
            LOGINFO6("%s/%s:%d devNum=%u axisNo=%u motor5008Num=%u encoderRaw=%u\n",
                     __FILE__, __FUNCTION__, __LINE__,
                     devNum, axisNo, motor5008Num, encoderRaw);
            UINTTONET(encoderRaw,
                      netData.memoryStruct.motors5008_1202_1604[motor5008Num].dev1202encoderRaw.value);
          }
        }
      }
      break;
    case TYPECODE_DISCRETEOUTPUT_1604:
      {
        unsigned axisNo = indexerDeviceAbsStraction[devNum].axisNo;
        if (axisNo) {
          unsigned motor5008Num = axisNo - 1;
          (void)motor5008Num;
          (void)pLCcycleInitDone;
#ifdef HAS_1604HOMPROC
          if (!(strcmp(indexerDeviceAbsStraction[devNum].devName, "homProc"))) {
            unsigned homProc = cmd_Motor_cmd[axisNo].nHomProc;
            if (pLCcycleInitDone) {
              homProc = NETTOUINT(netData.memoryStruct.motors5008_1202_1604[motor5008Num].dev1604homProc.targetValue);
              if (homProc != cmd_Motor_cmd[axisNo].nOldHomProc) {
                LOGINFO3("%s/%s:%d devNum=%u axisNo=%u motor5008Num=%u homProc=%i oldHomeProc=%u\n",
                         __FILE__, __FUNCTION__, __LINE__,
                         devNum, axisNo, motor5008Num,
                         homProc, cmd_Motor_cmd[axisNo].nOldHomProc);
                cmd_Motor_cmd[axisNo].nHomProc = homProc;
                cmd_Motor_cmd[axisNo].nOldHomProc = homProc;
              } else {
                UINTTONET(homProc,
                          netData.memoryStruct.motors5008_1202_1604[motor5008Num].dev1604homProc.targetValue);
              }
            }
            LOGINFO6("%s/%s:%d devNum=%u axisNo=%u motor5008Num=%u homProc=%u\n",
                     __FILE__, __FUNCTION__, __LINE__,
                     devNum, axisNo, motor5008Num, homProc);
            UINTTONET(homProc,
                      netData.memoryStruct.motors5008_1202_1604[motor5008Num].dev1604homProc.actualValue);
          }
#endif
        }
      }
      break;
    case TYPECODE_PARAMDEVICE_5008:
      {
        double fRet;
        size_t lenInPlcPara = 4;
        unsigned offset;
        unsigned axisNo = indexerDeviceAbsStraction[devNum].axisNo;
        unsigned motor5008Num = axisNo - 1;
        static const unsigned numAuxBits = 8;
        offset = (unsigned)((void*)&netData.memoryStruct.motors5008_1202_1604[motor5008Num] -
                            (void*)&netData);

        fRet = getMotorPos((int)axisNo);
        LOGINFO6("%s/%s:%d devNum=%u axisNo=%u motor5008Num=%u offset=%u fRet=%f\n",
                 __FILE__, __FUNCTION__, __LINE__,
                 devNum, axisNo, motor5008Num, offset, (double)fRet);
        doubleToNet(fRet, &netData.memoryBytes[offset], lenInPlcPara);
        /* status */
        indexerMotorStatusRead5008(devNum, axisNo, numAuxBits,
                                   &netData.memoryStruct.motors5008_1202_1604[motor5008Num].dev5008);

        /* param interface */
        offset = (unsigned)((void*)&netData.memoryStruct.motors5008_1202_1604[motor5008Num].dev5008.paramCtrl -
                            (void*)&netData);
        LOGINFO6("%s/%s:%d devNum=%u axisNo=%u motor5008Num=%u paramoffset=%u\n",
                 __FILE__, __FUNCTION__, __LINE__,
                 devNum, axisNo, motor5008Num, offset);
        indexerMotorParamInterface(axisNo, offset, lenInPlcPara);
      }
      break;
#ifdef HAS_5010
    case TYPECODE_PARAMDEVICE_5010:
      {
        double fRet;
        size_t lenInPlcPara = 8;
        unsigned offset;
        unsigned axisNo = indexerDeviceAbsStraction[devNum].axisNo;
        unsigned motor5010Num = axisNo - NUM_MOTORS5008 - 1;
        static const unsigned numAuxBits = 24;
        offset = (unsigned)((void*)&netData.memoryStruct.motors5010[motor5010Num].actualValue -
                            (void*)&netData);

        fRet = getMotorPos((int)axisNo);
        LOGINFO6("%s/%s:%d devNum=%u axisNo=%u motor5010Num=%u offset=%u fRet=%f\n",
                 __FILE__, __FUNCTION__, __LINE__,
                 devNum, axisNo, motor5010Num, offset, (double)fRet);
        doubleToNet(fRet, &netData.memoryBytes[offset], lenInPlcPara);
        /* status */
        indexerMotorStatusRead5010(devNum, axisNo, numAuxBits,
                                   &netData.memoryStruct.motors5010[motor5010Num]);

        /* param interface */
        offset = (unsigned)((void*)&netData.memoryStruct.motors5010[motor5010Num].paramCtrl -
                            (void*)&netData);
        LOGINFO6("%s/%s:%d devNum=%u motor5010Num=%u paramoffset=%u\n",
                 __FILE__, __FUNCTION__, __LINE__,
                 devNum, motor5010Num, offset);
        indexerMotorParamInterface(axisNo, offset, lenInPlcPara);
      }
      break;
#endif
    case TYPECODE_INDEXER:
      {
        uint16_t indexer_ack = NETTOUINT(netData.memoryStruct.indexer_ack);
        LOGINFO6("%s/%s:%d devNum=%u indexer_ack=0x%x\n",
                 __FILE__, __FUNCTION__, __LINE__,
                 devNum, indexer_ack);

        if (!(indexer_ack & 0x8000)) {
          unsigned lenInPlc = sizeof(indexer_ack);
          indexerHandleIndexerCmd(offsetIndexer, lenInPlc, indexer_ack);
        }
      }
      break;
#ifdef HAS_0518
    case TYPECODE_SPECIALDEVICE_0518:
      {
        uint16_t ctrl_word = NETTOUINT(netData.memoryStruct.special0518.control);
        unsigned plcNotHostHasWritten = (ctrl_word & 0x8000) ? 1 : 0;
        unsigned numBytes  = ctrl_word & 0x02FF; /* Bit 9..0 */
        LOGINFO6("%s/%s:%d devNum=%u special0518 ctrl0=0x%x crtl1=0x%x "
                 "plcNotHostHasWritten=%u numBytes=%u\n",
                 __FILE__, __FUNCTION__, __LINE__,
                 netData.memoryStruct.special0518.control[0],
                 netData.memoryStruct.special0518.control[1],
                 devNum, plcNotHostHasWritten, numBytes);
        if (!plcNotHostHasWritten && numBytes) {
          char *pNewline;
          pNewline = strchr((char *)&netData.memoryStruct.special0518.value, '\n');
          if (pNewline) {
            size_t line_len = 1 +
              (void*)pNewline -
              (void*)&netData.memoryStruct.special0518.value;
            int had_cr = 0;
            *pNewline = 0; /* Remove '\n' */
            if (line_len > 1) pNewline--;
            if (*pNewline == '\r') {
              had_cr = 1;
              *pNewline = '\0';
            }
            LOGINFO6("%s/%s:%d devNum=%u special0518 value=\"%s\"\n",
                     __FILE__, __FUNCTION__, __LINE__,
                     devNum, (const char*)netData.memoryStruct.special0518.value);
            (void)handle_input_line((const char *)&netData.memoryStruct.special0518.value,
                                    had_cr, 1);
          } else {
            LOGINFO3("%s/%s:%d devNum=%u special0518=\"%s\"\n",
                     __FILE__, __FUNCTION__, __LINE__,
                     devNum, (const char*)netData.memoryStruct.special0518.value);
          }
          memset(&netData.memoryStruct.special0518, 0,
                 sizeof(netData.memoryStruct.special0518));
          ctrl_word = 0;
          UINTTONET(ctrl_word, netData.memoryStruct.special0518.control);
        }
      }
      break;
#endif
    default:
      break;
    }
    devNum++;
  }
  pLCcycleInitDone = 1;
}
