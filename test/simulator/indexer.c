/* Implementation of the indexer
   https://forge.frm2.tum.de/public/doc/plc/v2.0/singlehtml/
*/

#include "indexer.h"

#include <ctype.h>
#include <inttypes.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmd.h"
#include "cmd_buf.h"
#include "hw_motor.h"
#include "logerr_info.h"

#define HAS_1604_OPEN_CLUTCH
#define HAS_1E04_SHUTTER
#define HAS_1E0C_SHUTTER_CAROUSEL

/* type codes and sizes */
#define TYPECODE_INDEXER 0
/* The lenght of the indexer data, the longest is
   probably netInfoType4_type with 34 byte */
#define WORDS_SIZE_INDEXER_DATA 17
#define HAS_0518
#ifdef HAS_0518
#define TYPECODE_SPECIALDEVICE_0518 0x0518
#endif
#define TYPECODE_DISCRETEINPUT_1202 0x1202
#define TYPECODE_STATUSWORD_1802 0x1802
#define TYPECODE_DISCRETEINPUT_1A04 0x1A04
#define TYPECODE_ANALOGINPUT_1B04 0x1B04
#define TYPECODE_DISCRETEOUTPUT_1604 0x1604

#ifdef HAS_1E04_SHUTTER
#define TYPECODE_DISCRETEOTPUT_1E04 0x1E04
#define WORDS_DISCRETEOTPUT_1E04 4
#endif
#ifdef HAS_1E0C_SHUTTER_CAROUSEL
#define TYPECODE_DISCRETEOTPUT_1E0C 0x1E0C
#define WORDS_DISCRETEOTPUT_1E0C 0xC
#endif
#define TYPECODE_PARAMDEVICE_5010 0x5010
#define WORDS_SPECIALDEVICE_0518 0x18
#define WORDS_DISCRETEINPUT_1202 0x2
#define WORDS_DISCRETEOUTPUT_1604 0x4
#define WORDS_STATUSWORD_1802 0x2
#define WORDS_DISCRETEINPUT_1A04 0x4
#define WORDS_ANALOGINPUT_1B04 0x4
#define WORDS_PARAMDEVICE_5010 0x10

/* Well known unit codes */
#define UNITCODE_NONE 0
#define UNITCODE_MM 0xfd04
#define UNITCODE_DEGREE 0x000C

#define AXISNO_NONE 0

/* axis1 .. axis4 */
#define NUM_MOTORS5010 4

#define NUM_1802 1

#ifdef HAS_1604_OPEN_CLUTCH
#define NUM_1604 NUM_MOTORS5010
#else
#define NUM_1604 0
#endif

/*
   Devices for the indexer:
   + the indexer itself
   + 1 SystemHealth#0 1802
   + 1 DISCRETEINPUT#1 1A04
   + 1 DISCRETEOUTPUT#1 1604
   + 1 special 0518
   + 4 motors 5010
   + 4 raw encoders
   + 4 1604 (open clutch)
   + 1 1E04
   + 1 1E0C
   + 1 1B04 (anlog input)
*/
#define NUM_INDEXERS 1

#ifdef HAS_0518
#define NUM_0518 1
#else
#define NUM_0518 0
#endif

#ifdef HAS_1E04_SHUTTER
#define NUM_1E04 1
#else
#define NUM_1E04 0
#endif
#ifdef HAS_1E0C_SHUTTER_CAROUSEL
#define NUM_1E0C 2
#else
#define NUM_1E0C 0
#endif

#define NUM_5010 4
#define NUM_DISCRET_IN 1
#define NUM_DISCRET_OUT 1
#define NUM_ANALOG_IN 1
#define NUM_DEVICES                                                      \
  (NUM_INDEXERS + NUM_0518 + NUM_5010 + NUM_5010 + NUM_1604 + NUM_1802 + \
   NUM_1E04 + NUM_1E0C + NUM_DISCRET_IN + NUM_DISCRET_OUT + NUM_ANALOG_IN)

typedef enum {
  idxStatusCodeRESET = 0,
  idxStatusCodeIDLE = 1,
#ifdef USE_IDXSTATUSCODEPOWEROFF
  idxStatusCodePOWEROFF = 2,
#endif
  idxStatusCodeWARN = 3,
  idxStatusCodeERR4 = 4,
  idxStatusCodeSTART = 5,
  idxStatusCodeBUSY = 6,
  idxStatusCodeSTOP = 7,
  idxStatusCodeERROR = 8,
  idxStatusCodeERR9 = 9,
  idxStatusCodeERR10 = 10,
  idxStatusCodeERR11 = 11,
  idxStatusCodeERR12 = 12,
  idxStatusCodeERR13 = 13,
  idxStatusCodeERR14 = 14,
  idxStatusCodeERR15 = 15
} idxStatusCodeType;

typedef enum { permPNone, permPRead, permPRDWR } permPTyp;

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
#define PARAM_IF_CMD_MASKPARAM_IF_CMD_MASK 0xE000
#define PARAM_IF_CMD_MASKPARAM_IF_IDX_MASK 0x1FFF
#define PARAM_IF_CMD_MASKPARAM_DONE 0x8000

#define PARAM_IF_CMD_INVALID 0x0000
#define PARAM_IF_CMD_DOREAD 0x2000
#define PARAM_IF_CMD_DOWRITE 0x4000
#define PARAM_IF_CMD_BUSY 0x6000
#define PARAM_IF_CMD_DONE 0x8000
#define PARAM_IF_CMD_ERR_NO_IDX 0xA000
#define PARAM_IF_CMD_READONLY 0xC000
#define PARAM_IF_CMD_RETRY_LATER 0xE000

/* Param index values */
#define PARAM_IDX_OPMODE_AUTO_UINT32 1
#define PARAM_IDX_MICROSTEPS_UINT32 2
#define PARAM_IDX_ABS_MIN_FLOAT32 30
#define PARAM_IDX_ABS_MAX_FLOAT32 31
#define PARAM_IDX_USR_MIN_FLOAT32 32
#define PARAM_IDX_USR_MAX_FLOAT32 33
#define PARAM_IDX_WRN_MIN_FLOAT32 34
#define PARAM_IDX_WRN_MAX_FLOAT32 35
#define PARAM_IDX_FOLLOWING_ERR_WIN_FLOAT32 55
#define PARAM_IDX_HYTERESIS_FLOAT32 56
#define PARAM_IDX_REFSPEED_FLOAT32 58
#define PARAM_IDX_VBAS_FLOAT32 59
#define PARAM_IDX_SPEED_FLOAT32 60
#define PARAM_IDX_ACCEL_FLOAT32 61
#define PARAM_IDX_IDLE_CURRENT_FLOAT32 62
#define PARAM_IDX_MOVE_CURRENT_FLOAT32 64
#define PARAM_IDX_MICROSTEPS_FLOAT32 67
#define PARAM_IDX_STEPS_PER_UNIT_FLOAT32 68
#define PARAM_IDX_HOME_POSITION_FLOAT32 69
#define PARAM_IDX_FUN_REFERENCE 133
#define PARAM_IDX_FUN_SET_POSITION 137
#define PARAM_IDX_FUN_MOVE_VELOCITY 142
/* Implementation defined, floating point */
#define PARAM_IDX_USR_MIN_EN_FLOAT32 218
#define PARAM_IDX_USR_MAX_EN_FLOAT32 219
#define PARAM_IDX_HOME_PROC_FLOAT32 220
#define PARAM_IDX_UNITS_PER_REV_FLOAT32 221
#define PARAM_IDX_STEPS_PER_REV_FLOAT32 222
#define PARAM_IDX_MAX_VELO_FLOAT32 223

/* In the memory bytes, the indexer starts at 64 */
static unsigned offsetIndexer;

/* Info types of the indexer as seen on the network */
typedef struct {
  uint8_t typeCode[2];
  uint8_t size[2];
  uint8_t offset[2];
  uint8_t unit[2];
  uint8_t flags[4];
  uint8_t absMin[4];
  uint8_t absMax[4];
} netInfoType0_type;

typedef struct {
  uint8_t size[2];
} netInfoType1_type;

typedef struct {
  uint8_t unitcode[2];
} netInfoType3_type;

typedef struct {
  char name[33]; /* leave one byte for trailing '\0' */
  char ascii_null;
} netInfoType4_type;

typedef struct {
  uint8_t parameters[32]; /* counting 0..31 */
} netInfoType15_type;

/* The special device structure. */
typedef struct {
  /* 2 bytes control, 46 payload */
  uint8_t control[2];
  char value[46];
} netDevice0518interface_type;

typedef struct {
  uint8_t value[4];
} netDevice1202interface_type;

typedef struct {
  uint8_t actualValue[4];
  uint8_t targetValue[4];
} netDevice1604interface_type;

typedef struct {
  uint8_t statusReasonAux32[4];
} netDevice1802interface_type;

typedef struct {
  uint8_t actualValue[4];
  uint8_t statusReasonAux32[4];
} netDevice1A04interface_type;

typedef struct {
  uint8_t actualValue[4];
  uint8_t statusReasonAux32[4];
} netDevice1B04interface_type;

#ifdef HAS_1E04_SHUTTER
typedef struct {
  uint8_t actualValue[2];
  uint8_t targetValue[2];
  uint8_t statusReasonAux32[4];
} netDevice1E04interface_type;
#endif
#ifdef HAS_1E0C_SHUTTER_CAROUSEL
typedef struct {
  uint8_t actualValue[8];
  uint8_t targetValue[8];
  uint8_t statusReasonAux32[4];
  uint8_t errorID[4];  // The spec says 2 byte errorID, 2 bytes reserved
} netDevice1E0Cinterface_type;
#endif

/* struct as seen on the network = in memory
 * data must be stored using  uintToNet/doubleToNet
 * and retrieved netToUint/netToDouble */
typedef struct {
  uint8_t actualValue[8];
  uint8_t targetValue[8];
  uint8_t statusReasonAux32[4];
  uint8_t errorID[2];
  uint8_t paramCtrl[2];
  uint8_t paramValue[8];
} netDevice5010interface_type;

typedef struct {
  netDevice5010interface_type dev5010;
  netDevice1202interface_type dev1202encoderRaw;
#ifdef HAS_1604_OPEN_CLUTCH
  netDevice1604interface_type dev1604openClutch;
#endif
} netDevice5010_1202_Interface_type;

/* Info type as seen here locally in memory */
typedef struct {
  uint16_t typeCode;
  uint16_t sizeInBytes;
  uint16_t unitCode;
  uint16_t axisNo;
  permPTyp permP[256];
  char devName[34];
  char auxName[24][34];
  float absMin;
  float absMax;
} indexerDeviceAbsStraction_type;

indexerDeviceAbsStraction_type indexerDeviceAbsStraction[NUM_DEVICES] = {
    /* device 0, the indexer itself */
    {TYPECODE_INDEXER,
     2 * WORDS_SIZE_INDEXER_DATA,
     UNITCODE_NONE,
     AXISNO_NONE,
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     "indexer",
     {"", "", "", "", "", "", "", "", "", "", "", "",
      "", "", "", "", "", "", "", "", "", "", "", ""},
     0.0,
     0.0},
    /* special device */
    {TYPECODE_STATUSWORD_1802,
     2 * WORDS_STATUSWORD_1802,
     UNITCODE_NONE,
     AXISNO_NONE,
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     "Cabinet#0",
     {"24VPSFailed",
      "48VPSFailed",
      "MCBError",
      "SPDError",
      "DoorOpen",
      "FuseTripped",
      "EStop",
      "TempHigh",
      "ECMasterErr",
      "SlaveNotOP",
      "SlaveMissing",
      "CPULoadHigh",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      ""},
     0.0,
     0.0},
    /* device for discrete input with status word */
    {TYPECODE_DISCRETEINPUT_1A04,
     2 * WORDS_DISCRETEINPUT_1A04,
     UNITCODE_NONE,
     0,
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     "DISCRETEINPUT#1",
     {"", "", "", "", "", "", "", "", "", "", "", "",
      "", "", "", "", "", "", "", "", "", "", "", ""},
     0.0,
     0.0},
    /* device for simple discrete output */
    {TYPECODE_DISCRETEOUTPUT_1604,
     2 * WORDS_DISCRETEOUTPUT_1604,
     UNITCODE_NONE,
     0,
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     "DISCRETEOUTPUT#1",
     {"", "", "", "", "", "", "", "", "", "", "", "",
      "", "", "", "", "", "", "", "", "", "", "", ""},
     0.0,
     0.0},
#ifdef HAS_0518
    /* special device */
    {TYPECODE_SPECIALDEVICE_0518,
     2 * WORDS_SPECIALDEVICE_0518,
     UNITCODE_NONE,
     AXISNO_NONE,
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     "DbgStrToMcu",
     {"", "", "", "", "", "", "", "", "", "", "", "",
      "", "", "", "", "", "", "", "", "", "", "", ""},
     0.0,
     0.0},
#endif
    {TYPECODE_PARAMDEVICE_5010,
     2 * WORDS_PARAMDEVICE_5010,
     UNITCODE_MM,
     1,
     {/*   0..4   */ permPNone,
      modePRDWR,
      permPNone,
      permPNone,
      permPNone,
      /*   5..9   */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  10..14  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  15..19  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  20..24  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  25..29  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  30..34  */ permPNone,
      permPNone,
      uminPRDWR,
      umaxPRDWR,
      permPNone,
      /*  35..39  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  40..44  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  45..49  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  50..54  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  55..59  */ permPNone,
      hystPRead,
      permPNone,
      hvelPRead,
      permPNone,
      /*  60..64  */ speedRDWR,
      acclPRDWR,
      permPNone,
      permPNone,
      permPNone,
      /*  65..69  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      hposPRead,
      /*  70..74  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  75..79  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  80..84  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  85..89  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  90..94  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  95..99  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 100..105 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 105..109 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 110..114 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 115..119 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 120..124 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 125..129 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 130..134 */ permPNone,
      permPNone,
      permPNone,
      homePRDWR,
      permPNone,
      /* 135..139 */ permPNone,
      permPNone,
      setPoRDWR,
      permPNone,
      permPNone,
      /* 140..144 */ permPNone,
      permPNone,
      mvVelRDWR,
      permPNone,
      permPNone,
      /* 145..159 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 150..154 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 155..159 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 160..164 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 165..169 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 170..174 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 175..179 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 180..184 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 185..189 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 190..194 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 195..199 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 200..204 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 205..209 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 210..214 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 215..219 */ permPNone,
      permPNone,
      permPNone,
      uminERDWR,
      umaxERDWR,
      /* 220..224 */ hprocRDWR,
      urevPRead,
      srevPRead,
      vmaxPRead,
      permPNone,
      /* 225..229 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 230..234 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 235..239 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 240..204 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone},
     "SimAxis1",
     {"",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "InterlockFwd",
      "InterlockBwd",
      "localMode",
      "",
      "enabled",
      "notHomed"},
     5.0,
     175.0},
    /* device for encoderRaw */
    {TYPECODE_DISCRETEINPUT_1202,
     2 * WORDS_DISCRETEINPUT_1202,
     UNITCODE_NONE,
     1,
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     "encoderRaw",
     {"", "", "", "", "", "", "", "", "", "", "", "",
      "", "", "", "", "", "", "", "", "", "", "", ""},
     0.0,
     0.0},
#ifdef HAS_1604_OPEN_CLUTCH
    /* device for openClutch */
    {TYPECODE_DISCRETEOUTPUT_1604,
     2 * WORDS_DISCRETEOUTPUT_1604,
     UNITCODE_NONE,
     1,
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     "openClutch#1",
     {"", "", "", "", "", "", "", "", "", "", "", "",
      "", "", "", "", "", "", "", "", "", "", "", ""},
     0.0,
     1.0},
#endif
    {TYPECODE_PARAMDEVICE_5010,
     2 * WORDS_PARAMDEVICE_5010,
     UNITCODE_DEGREE,
     2,
     {/*   0..4   */ permPNone,
      modePRDWR,
      permPNone,
      permPNone,
      permPNone,
      /*   5..9   */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  10..14  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  15..19  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  20..24  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  25..29  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  30..34  */ permPNone,
      permPNone,
      uminPRDWR,
      umaxPRDWR,
      permPNone,
      /*  35..39  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  40..44  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  45..49  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  50..54  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  55..59  */ permPNone,
      hystPRead,
      permPNone,
      hvelPNone,
      permPNone,
      /*  60..64  */ speedRDWR,
      acclPRDWR,
      permPNone,
      permPNone,
      permPNone,
      /*  65..69  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      hposPRead,
      /*  70..74  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  75..79  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  80..84  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  85..89  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  90..94  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  95..99  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 100..105 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 105..109 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 110..114 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 115..119 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 120..124 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 125..129 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 130..134 */ permPNone,
      permPNone,
      permPNone,
      homePNone,
      permPNone,
      /* 135..139 */ permPNone,
      permPNone,
      setPoRDWR,
      permPNone,
      permPNone,
      /* 140..144 */ permPNone,
      permPNone,
      mvVelRDWR,
      permPNone,
      permPNone,
      /* 145..159 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 150..154 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 155..159 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 160..164 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 165..169 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 170..174 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 175..179 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 180..184 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 185..189 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 190..194 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 195..199 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 200..204 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 205..209 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 210..214 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 215..219 */ permPNone,
      permPNone,
      permPNone,
      uminERDWR,
      umaxERDWR,
      /* 220..224 */ hprocRDWR,
      urevPRead,
      srevPRead,
      vmaxPRead,
      permPNone,
      /* 225..229 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 230..234 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 235..239 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 240..204 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone},
     "RotAxis2",
     {"",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "InerlockFwd",
      "InerlockBwd",
      "localMode",
      "",
      "enabled",
      "notHomed"},
     -181.0,
     +181.0},
    /* device for encoderRaw */
    {TYPECODE_DISCRETEINPUT_1202,
     2 * WORDS_DISCRETEINPUT_1202,
     UNITCODE_NONE,
     2,
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     "encoderRaw",
     {"", "", "", "", "", "", "", "", "", "", "", "",
      "", "", "", "", "", "", "", "", "", "", "", ""},
     0.0,
     0.0},
#ifdef HAS_1604_OPEN_CLUTCH
    /* device for openClutch */
    {TYPECODE_DISCRETEOUTPUT_1604,
     2 * WORDS_DISCRETEOUTPUT_1604,
     UNITCODE_NONE,
     2,
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     "openClutch#2",
     {"", "", "", "", "", "", "", "", "", "", "", "",
      "", "", "", "", "", "", "", "", "", "", "", ""},
     0.0,
     1.0},
#endif
    {TYPECODE_PARAMDEVICE_5010,
     2 * WORDS_PARAMDEVICE_5010,
     UNITCODE_MM,
     3,
     {/*   0..4   */ permPNone,
      modePRDWR,
      permPNone,
      permPNone,
      permPNone,
      /*   5..9   */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  10..14  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  15..19  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  20..24  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  25..29  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  30..34  */ permPNone,
      permPNone,
      uminPRDWR,
      umaxPRDWR,
      permPNone,
      /*  35..39  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  40..44  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  45..49  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  50..54  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  55..59  */ permPNone,
      hystPRead,
      permPNone,
      hvelPNone,
      permPNone,
      /*  60..64  */ speedRDWR,
      acclPRDWR,
      permPNone,
      permPNone,
      permPNone,
      /*  65..69  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      hposPRead,
      /*  70..74  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  75..79  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  80..84  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  85..89  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  90..94  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  95..99  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 100..105 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 105..109 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 110..114 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 115..119 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 120..124 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 125..129 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 130..134 */ permPNone,
      permPNone,
      permPNone,
      homePRDWR,
      permPNone,
      /* 135..139 */ permPNone,
      permPNone,
      setPoNone,
      permPNone,
      permPNone,
      /* 140..144 */ permPNone,
      permPNone,
      mvVelRDWR,
      permPNone,
      permPNone,
      /* 145..159 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 150..154 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 155..159 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 160..164 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 165..169 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 170..174 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 175..179 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 180..184 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 185..189 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 190..194 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 195..199 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 200..204 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 205..209 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 210..214 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 215..219 */ permPNone,
      permPNone,
      permPNone,
      uminERDWR,
      umaxERDWR,
      /* 220..224 */ hprocRDWR,
      urevPRead,
      srevPRead,
      vmaxPRead,
      permPNone,
      /* 225..229 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 230..234 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 235..239 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 240..204 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone},
     "Axis5010-3",
     {"", "", "", "", "", "", "", "", "", "", "",        "",
      "", "", "", "", "", "", "", "", "", "", "enabled", "notHomed"},
     0,
     +173.0},
    /* device for encoderRaw */
    {TYPECODE_DISCRETEINPUT_1202,
     2 * WORDS_DISCRETEINPUT_1202,
     UNITCODE_NONE,
     3,
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     "encoderRaw",
     {"", "", "", "", "", "", "", "", "", "", "", "",
      "", "", "", "", "", "", "", "", "", "", "", ""},
     0.0,
     0.0},
#ifdef HAS_1604_OPEN_CLUTCH
    /* device for openClutch */
    {TYPECODE_DISCRETEOUTPUT_1604,
     2 * WORDS_DISCRETEOUTPUT_1604,
     UNITCODE_NONE,
     3,
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     "openClutch#3",
     {"", "", "", "", "", "", "", "", "", "", "", "",
      "", "", "", "", "", "", "", "", "", "", "", ""},
     0.0,
     1.0},
#endif
    {TYPECODE_PARAMDEVICE_5010,
     2 * WORDS_PARAMDEVICE_5010,
     UNITCODE_MM,
     4,
     {/*   0..4   */ permPNone,
      modePRDWR,
      permPNone,
      permPNone,
      permPNone,
      /*   5..9   */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  10..14  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  15..19  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  20..24  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  25..29  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  30..34  */ permPNone,
      permPNone,
      uminPRDWR,
      umaxPRDWR,
      permPNone,
      /*  35..39  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  40..44  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  45..49  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  50..54  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  55..59  */ permPNone,
      hystPRead,
      permPNone,
      hvelPNone,
      permPNone,
      /*  60..64  */ speedRDWR,
      acclPRDWR,
      permPNone,
      permPNone,
      permPNone,
      /*  65..69  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      hposPRead,
      /*  70..74  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  75..79  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  80..84  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  85..89  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  90..94  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /*  95..99  */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 100..105 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 105..109 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 110..114 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 115..119 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 120..124 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 125..129 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 130..134 */ permPNone,
      permPNone,
      permPNone,
      homePNone,
      permPNone,
      /* 135..139 */ permPNone,
      permPNone,
      setPoNone,
      permPNone,
      permPNone,
      /* 140..144 */ permPNone,
      permPNone,
      mvVelRDWR,
      permPNone,
      permPNone,
      /* 145..159 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 150..154 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 155..159 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 160..164 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 165..169 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 170..174 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 175..179 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 180..184 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 185..189 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 190..194 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 195..199 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 200..204 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 205..209 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 210..214 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 215..219 */ permPNone,
      permPNone,
      permPNone,
      uminERDWR,
      umaxERDWR,
      /* 220..224 */ hprocRDWR,
      urevPRead,
      srevPRead,
      vmaxPRead,
      permPNone,
      /* 225..229 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 230..234 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 235..239 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone,
      /* 240..204 */ permPNone,
      permPNone,
      permPNone,
      permPNone,
      permPNone},
     "Axis5010-4",
     {"", "", "", "", "", "", "", "", "", "", "",        "",
      "", "", "", "", "", "", "", "", "", "", "enabled", "notHomed"},
     0,
     +163.0},
    /* device for encoderRaw */
    {TYPECODE_DISCRETEINPUT_1202,
     2 * WORDS_DISCRETEINPUT_1202,
     UNITCODE_NONE,
     4,
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     "encoderRaw",
     {"", "", "", "", "", "", "", "", "", "", "", "",
      "", "", "", "", "", "", "", "", "", "", "", ""},
     0.0,
     0.0},
#ifdef HAS_1604_OPEN_CLUTCH
    /* device for openClutch */
    {TYPECODE_DISCRETEOUTPUT_1604,
     2 * WORDS_DISCRETEOUTPUT_1604,
     UNITCODE_NONE,
     4,
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     "openClutch#4",
     {"", "", "", "", "", "", "", "", "", "", "", "",
      "", "", "", "", "", "", "", "", "", "", "", ""},
     0.0,
     1.0}
#endif
#ifdef HAS_1E04_SHUTTER
    /* device for shutter, motor 5 */
    ,
    {TYPECODE_DISCRETEOTPUT_1E04,
     2 * WORDS_DISCRETEOTPUT_1E04,
     UNITCODE_NONE,
     5,
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     "Shutter",
     {"Closed",  "Closing", "InTheMiddle",
      "Opening", "Opened",  "",
      "",        "",        "",
      "",        "",        "",
      "",        "",        "",
      "",        "",        "",
      "",        "",        "",
      "",        "",        ""},
     1.0,
     5.0}
#endif
#ifdef HAS_1E0C_SHUTTER_CAROUSEL
    /* device for shutter, motor 6 */
    ,
    {TYPECODE_DISCRETEOTPUT_1E0C,
     2 * WORDS_DISCRETEOTPUT_1E0C,
     UNITCODE_NONE,
     6,
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     "Pneu",
     {"Closed",  "Closing", "InTheMiddle",
      "Opening", "Opened",  "",
      "",        "",        "",
      "",        "",        "",
      "",        "",        "",
      "",        "",        "",
      "",        "",        "",
      "",        "",        ""},
     1.0,
     5.0}
#endif
#ifdef HAS_1E0C_SHUTTER_CAROUSEL
    /* device for carousel, motor 7 */
    ,
    {TYPECODE_DISCRETEOTPUT_1E0C,
     2 * WORDS_DISCRETEOTPUT_1E0C,
     UNITCODE_NONE,
     7,
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     "Carousel$inbits=0..10",
     {"OutOffCarousel",
      "Position1",
      "Position2",
      "Position3",
      "Position4",
      "Position5",
      "Position6",
      "Position7",
      "Position8",
      "Position9",
      "Position10",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      ""},
     0.0,
     10.0}
#endif
    /* device for analog input with status word */
    ,
    {TYPECODE_ANALOGINPUT_1B04,
     2 * WORDS_ANALOGINPUT_1B04,
     UNITCODE_NONE,
     0,
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     //"ANALOGINPUT#1#AnalogInWithStatus",
     "ANALOGINPUT#1",
     {"", "", "", "", "", "", "", "", "", "", "", "",
      "", "", "", "", "", "", "", "", "", "", "", ""},
     180.0,
     -1.0}
#if 0
    ,{ 0, 0,
       UNITCODE_NONE, 0,
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       "",
       { "", "", "", "", "", "", "", "",
         "", "", "", "", "", "", "", "",
         "", "", "", "", "", "", "", "" },
       0.0, 0.0
    }
#endif
};

typedef struct {
  double fHysteresis;
  double fRefSpeed;
  int nHomProc;
  int nOldHomProc;
  /*
    TODO (about paramIfBusyMoveVelocity):
    Add code in cmd_Sim.c to allo to change this variabe
    Add code in ../pytests36/950... to add a new test case:
       - change it to 1.
       - move to a low position
       - start a moveVel: self.axisCom.put("-MoveVel", jvel)
       - wait for started
       - change velocity: self.axisCom.put("-MoveVel", jvel), see below
       - check that the previous put failed (!)
       - check that the previos put did not spend more than 500msec
  */
  int paramIfBusyMoveVelocity;
} cmd_Motor_cmd_type;

static union {
  uint8_t memoryBytes[1024];
  struct {
    uint8_t magic[4];
    uint8_t offset[2];
    uint8_t indexer_ack[2];
    /* Area for the indexer. union of the different info types */
    union {
      netInfoType0_type infoType0;
      netInfoType1_type infoType1;
      netInfoType3_type infoType3;
      netInfoType4_type infoType4;
      netInfoType15_type infoType15;
    } indexer_info;
    netDevice1802interface_type statusWord1802[NUM_1802];
    netDevice1A04interface_type discreteInput1A04[1];
    netDevice1604interface_type discreteOutput1604[1];
#ifdef HAS_0518
    netDevice0518interface_type
        special0518; /* 42 bytes for ASCII to the simulator */
#endif
    /* Remember that motor[0] is defined, but never used */
    netDevice5010_1202_Interface_type motors5010_1202[NUM_MOTORS5010];
    netDevice1E04interface_type motors1E04[NUM_1E04];
    netDevice1E0Cinterface_type motors1E0C[NUM_1E0C];
    netDevice1B04interface_type analogInput1B04[1];
  } memoryStruct;
} netData;

static int initDone = 0;

/* values commanded to the motor */
static cmd_Motor_cmd_type cmd_Motor_cmd[MAX_AXES];

static unsigned netToUint(void *data, size_t lenInPlc) {
  const uint8_t *src = (const uint8_t *)data;
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

static double netToDouble(void *data, size_t lenInPlc) {
  const uint8_t *src = (const uint8_t *)data;
  if (lenInPlc == 4) {
    union {
      volatile uint32_t uRes;
      volatile float fRes;
    } dst;
    dst.uRes = (uint32_t)src[0] + ((uint32_t)src[1] << 8) +
               ((uint32_t)src[2] << 16) + ((uint32_t)src[3] << 24);
    return (double)dst.fRes;
  } else if (lenInPlc == 8) {
    union {
      volatile uint64_t uRes;
      volatile double fRes;
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

#define NETTODOUBLE(n) netToDouble(&(n), sizeof(n))

static void doubleToNet(const double value, void *data, size_t lenInPlc) {
  uint8_t *dst = (uint8_t *)data;
  if (lenInPlc == 4) {
    union {
      volatile uint32_t uRes;
      volatile float fRes;
    } src;
    src.fRes = (float)value;
    dst[0] = (uint8_t)src.uRes;
    dst[1] = (uint8_t)(src.uRes >> 8);
    dst[2] = (uint8_t)(src.uRes >> 16);
    dst[3] = (uint8_t)(src.uRes >> 24);
  } else if (lenInPlc == 8) {
    union {
      volatile uint64_t uRes;
      volatile double fRes;
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

static void uintToNet(const unsigned value, void *data, size_t lenInPlc) {
  uint8_t *dst = (uint8_t *)data;
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

#define NETTOUINT(n) netToUint(&(n), sizeof(n))
#define UINTTONET(v, n) uintToNet(v, &(n), sizeof(n))
#define DOUBLETONET(v, n) doubleToNet(v, &(n), sizeof(n))

static void init(void) {
  if (initDone) return;
  memset(&netData, 0, sizeof(netData));
  DOUBLETONET(2015.02, netData.memoryStruct.magic);

  offsetIndexer =
      (unsigned)((void *)&netData.memoryStruct.indexer_ack - (void *)&netData);
  UINTTONET(offsetIndexer, netData.memoryStruct.offset);

  LOGTIME3("%s/%s:%d offsetIndexer=%u\n", __FILE__, __FUNCTION__, __LINE__,
           offsetIndexer);
  setCabinetStatus(idxStatusCodeIDLE << 28);
  initDone = 1;
}

static void init_axis(int axis_no) {
  static char init_done[MAX_AXES];
  const double MRES = 1;
  const double UREV = 60.0;   /* mm/revolution */
  const double SREV = 2000.0; /* ticks/revolution */
  const double ERES = UREV / SREV;

  double ReverseMRES = (double)1.0 / MRES;

  if (axis_no >= MAX_AXES || axis_no < 0) {
    return;
  }
  if (!init_done[axis_no]) {
    struct motor_init_values motor_init_values;
    double valueLow = -1.0 * ReverseMRES;
    double valueHigh = 186.0 * ReverseMRES;
    memset(&motor_init_values, 0, sizeof(motor_init_values));
    motor_init_values.ReverseERES = MRES / ERES;
    motor_init_values.ParkingPos = (10 + axis_no / 10.0);
    motor_init_values.MaxHomeVelocityAbs = 10 * ReverseMRES;
    motor_init_values.lowHardLimitPos = valueLow;
    motor_init_values.highHardLimitPos = valueHigh;
    motor_init_values.hWlowPos = valueLow;
    motor_init_values.hWhighPos = valueHigh;

    hw_motor_init(axis_no, &motor_init_values, sizeof(motor_init_values));

    setMRES_23(axis_no, UREV);
    setMRES_24(axis_no, SREV);
    cmd_Motor_cmd[axis_no].fHysteresis = 0.1;
    cmd_Motor_cmd[axis_no].fRefSpeed = 1.5;
    if (axis_no == 4) setAxisHomed(axis_no, 1);
    cmd_Motor_cmd[axis_no].nHomProc = 1;
    cmd_Motor_cmd[axis_no].nOldHomProc = cmd_Motor_cmd[axis_no].nHomProc;
    setHomePos(axis_no, 0.1);
    setNxtMoveVelocity(axis_no, 50 + axis_no / 10.0);
    setNxtMoveAcceleration(axis_no, 1 + axis_no / 10.0);
    setMaxHomeVelocityAbs(axis_no, 10);
    /* Simulated limit switches, take from indexer table */
    {
      unsigned devNum; /* 0 is the indexer */
      for (devNum = 1; devNum < NUM_DEVICES; devNum++) {
        int tmp_axis_no = indexerDeviceAbsStraction[devNum].axisNo;
        LOGTIME3("%s/%s:%d axis_no=%d tmp_axis_no=%d devNum=%u typeCode=0x%x\n",
                 __FILE__, __FUNCTION__, __LINE__, axis_no, tmp_axis_no, devNum,
                 indexerDeviceAbsStraction[devNum].typeCode);
        if (tmp_axis_no == axis_no) {
          double absMax = indexerDeviceAbsStraction[devNum].absMax;
          double absMin = indexerDeviceAbsStraction[devNum].absMin;
          switch (indexerDeviceAbsStraction[devNum].typeCode) {
            case 0x1E04:
            case 0x1E0C: {
              setMotorParkingPosition(axis_no, absMin);
              setHighHardLimitPos(axis_no, absMax);
              setLowHardLimitPos(axis_no, absMin);
              setHighSoftLimitPos(axis_no, absMax);
              setLowSoftLimitPos(axis_no, absMin);
              setEnableHighSoftLimit(axis_no, 1);
              setEnableLowSoftLimit(axis_no, 1);
              setMRES_23(axis_no,
                         0.0); /* avoid rounding to a step in hw_motor.c */
              setMRES_24(axis_no, 0.0);

              LOGTIME3(
                  "%s/%s:%d init1E0x axis_no=%d absMax=%f "
                  "absMin=%f\n",
                  __FILE__, __FUNCTION__, __LINE__, axis_no, absMax, absMin);
            } break;
            case 0x5010: {
              int hasUserMin = 0;
              unsigned param_idx = PARAM_IDX_USR_MIN_FLOAT32;
              hasUserMin = indexerDeviceAbsStraction[devNum].permP[param_idx] !=
                           permPNone;
              setHighHardLimitPos(axis_no, absMax + 1.0);
              setLowHardLimitPos(axis_no, absMin - 1.0);
              LOGTIME3("%s/%s:%d axis_no=%d axis_no=%d USR_MIN_BIT=%u\n",
                       __FILE__, __FUNCTION__, __LINE__, axis_no, axis_no,
                       hasUserMin);

              if (hasUserMin) {
                setHighSoftLimitPos(axis_no, absMax);
                setLowSoftLimitPos(axis_no, absMin);
                setEnableHighSoftLimit(axis_no, 1);
                setEnableLowSoftLimit(axis_no, 1);
              }
            } break;
            default:
              break;
          }
        }
      }
    }
    setAmplifierPercent(axis_no, 1);
    init_done[axis_no] = 1;
  }
}

#ifdef HAS_1E04_SHUTTER
static void indexerMotorStatusRead1E04(
    unsigned devNum, unsigned motor_axis_no, unsigned numAuxBits,
    netDevice1E04interface_type *pIndexerDevice1E04interface) {
  unsigned statusReasonAux32;
  idxStatusCodeType idxStatusCode;
  statusReasonAux32 = NETTOUINT(pIndexerDevice1E04interface->statusReasonAux32);
  idxStatusCode = (idxStatusCodeType)(statusReasonAux32 >> 28);

  switch (idxStatusCode) {
    case idxStatusCodeRESET:
      init_axis((int)motor_axis_no);
      motorStop(motor_axis_no);
      set_bError(motor_axis_no, 0);
      set_nErrorId(motor_axis_no, 0);
      break;
    case idxStatusCodeSTART:
      movePosition(motor_axis_no,
                   /* Note: We can only move inside the positive range */
                   (double)NETTOUINT(pIndexerDevice1E04interface->targetValue),
                   0,    /* int relative, */
                   1.0,  // getNxtMoveVelocity(motor_axis_no),
                   getNxtMoveAcceleration(motor_axis_no));
      LOGTIME3(
          "%s/%s:%d motor_axis_no=%u idxStatusCodeSTART isMotorMoving=%d\n",
          __FILE__, __FUNCTION__, __LINE__, motor_axis_no,
          isMotorMoving(motor_axis_no));
      break;
    case idxStatusCodeSTOP:
      motorStop(motor_axis_no);
      break;
    default:;
  }
  /* Build a new status word, start with 0 and fill in
     the bits */
  statusReasonAux32 = 0;
  /*
    if (getPosLimitSwitch(motor_axis_no))
      statusReasonAux32 |= 0x08000000;
    if (getNegLimitSwitch(motor_axis_no))
      statusReasonAux32 |= 0x04000000;
  */

  /* reason bits */
  {
    double motorVelocity = getMotorVelocity(motor_axis_no);
    int posLimitSwitch = getPosLimitSwitch(motor_axis_no);
    int negLimitSwitch = getNegLimitSwitch(motor_axis_no);

    unsigned auxBitIdx = 0;
    if (motorVelocity) {
      /* While moving: Ignore the limits */
      posLimitSwitch = 0;
      negLimitSwitch = 0;
    }
    for (auxBitIdx = 0; auxBitIdx < numAuxBits; auxBitIdx++) {
      const char *auxBitName =
          (const char *)&indexerDeviceAbsStraction[devNum].auxName[auxBitIdx];
      LOGTIME6("%s/%s:%d motor_axis_no=%u auxBitIdx=%u auxBitName=%s\n",
               __FILE__, __FUNCTION__, __LINE__, motor_axis_no, auxBitIdx,
               auxBitName);

      if (!strcmp("enabled", auxBitName)) {
        int bValue = getAmplifierOn(motor_axis_no);
        LOGTIME6("%s/%s:%d motor_axis_no=%u auxBitIdx=%u enabled=%d\n",
                 __FILE__, __FUNCTION__, __LINE__, motor_axis_no, auxBitIdx,
                 bValue);
        if (bValue) {
          statusReasonAux32 |= 1 << auxBitIdx;
        }
      } else if (!strcmp("Closed", auxBitName)) {
        if (negLimitSwitch) statusReasonAux32 |= 1 << auxBitIdx;
      } else if (!strcmp("Opened", auxBitName)) {
        if (posLimitSwitch) statusReasonAux32 |= 1 << auxBitIdx;
      } else if (!strcmp("Opening", auxBitName)) {
        if (motorVelocity > 0.0) statusReasonAux32 |= 1 << auxBitIdx;
      } else if (!strcmp("Closing", auxBitName)) {
        if (motorVelocity < 0.0) statusReasonAux32 |= 1 << auxBitIdx;
      } else if (!strcmp("InTheMiddle", auxBitName)) {
        if (!motorVelocity && !posLimitSwitch && !negLimitSwitch)
          statusReasonAux32 |= 1 << auxBitIdx;
      }
    }
  }

  /* the status bits */
  if (idxStatusCode == idxStatusCodeSTART) {
    idxStatusCode = idxStatusCodeBUSY;
  } else if (get_bError(motor_axis_no)) {
    idxStatusCode = idxStatusCodeERROR;
  }
#ifdef USE_IDXSTATUSCODEPOWEROFF
  else if (!getAmplifierOn(motor_axis_no))
    idxStatusCode = idxStatusCodePOWEROFF;
#endif
  else if (isMotorMoving(motor_axis_no))
    idxStatusCode = idxStatusCodeBUSY;
  else if (statusReasonAux32 & 0x0C000000)
    idxStatusCode = idxStatusCodeWARN;
  else
    idxStatusCode = idxStatusCodeIDLE;

  statusReasonAux32 &= 0x0FFFFFFF;
  statusReasonAux32 |= ((unsigned)idxStatusCode << 28);
  UINTTONET(statusReasonAux32, pIndexerDevice1E04interface->statusReasonAux32);
  UINTTONET((int)getMotorPos(motor_axis_no),
            pIndexerDevice1E04interface->actualValue);
}
#endif
#ifdef HAS_1E0C_SHUTTER_CAROUSEL
static void indexerMotorStatusRead1E0C(
    unsigned devNum, unsigned motor_axis_no, unsigned numAuxBits,
    netDevice1E0Cinterface_type *pIndexerDevice1E0Cinterface,
    const char *pDevName) {
  unsigned statusReasonAux32;
  idxStatusCodeType idxStatusCode;
  statusReasonAux32 = NETTOUINT(pIndexerDevice1E0Cinterface->statusReasonAux32);
  idxStatusCode = (idxStatusCodeType)(statusReasonAux32 >> 28);

  switch (idxStatusCode) {
    case idxStatusCodeRESET:
      LOGTIME("%s/%s:%d motor_axis_no=%u idxStatusCodeRESET\n", __FILE__,
              __FUNCTION__, __LINE__, motor_axis_no);

      init_axis((int)motor_axis_no);
      motorStop(motor_axis_no);
      set_bError(motor_axis_no, 0);
      set_nErrorId(motor_axis_no, 0);
      break;
    case idxStatusCodeSTART:
      movePosition(motor_axis_no,
                   /* Note: We can only move inside the positive range */
                   (double)NETTOUINT(pIndexerDevice1E0Cinterface->targetValue),
                   0,    /* int relative, */
                   1.0,  // getNxtMoveVelocity(motor_axis_no),
                   getNxtMoveAcceleration(motor_axis_no));
      LOGTIME3(
          "%s/%s:%d motor_axis_no=%u idxStatusCodeSTART isMotorMoving=%d\n",
          __FILE__, __FUNCTION__, __LINE__, motor_axis_no,
          isMotorMoving(motor_axis_no));
      break;
    case idxStatusCodeSTOP:
      motorStop(motor_axis_no);
      break;
    default:;
  }
  if (getManualSimulatorMode(motor_axis_no)) {
    /* The simulator may overwrite the whole statusReasonAux32 */
    statusReasonAux32 = get_nStatReasAUX(motor_axis_no);
    LOGTIME3("%s/%s:%d motor_axis_no=%u simulated statusReasonAux32=0x%08x\n",
             __FILE__, __FUNCTION__, __LINE__, motor_axis_no,
             statusReasonAux32);
  } else {
    /* Build a new status word, start with 0 and fill in
       the bits */
    statusReasonAux32 = 0;
    if (!strcmp("Carousel$inbits=0..10", pDevName)) {
      int iRet = (int)(0.5 + getMotorPos((int)motor_axis_no)); /* NINT */
      statusReasonAux32 = 1 << iRet;
    }
    /*
      if (getPosLimitSwitch(motor_axis_no))
      statusReasonAux32 |= 0x08000000;
      if (getNegLimitSwitch(motor_axis_no))
      statusReasonAux32 |= 0x04000000;
    */

    /* reason bits */
    {
      double motorVelocity = getMotorVelocity(motor_axis_no);
      int posLimitSwitch = getPosLimitSwitch(motor_axis_no);
      int negLimitSwitch = getNegLimitSwitch(motor_axis_no);

      unsigned auxBitIdx = 0;
      if (motorVelocity) {
        /* While moving: Ignore the limits */
        posLimitSwitch = 0;
        negLimitSwitch = 0;
      }
      for (auxBitIdx = 0; auxBitIdx < numAuxBits; auxBitIdx++) {
        const char *auxBitName =
            (const char *)&indexerDeviceAbsStraction[devNum].auxName[auxBitIdx];
        LOGTIME6("%s/%s:%d motor_axis_no=%u auxBitIdx=%u auxBitName=%s\n",
                 __FILE__, __FUNCTION__, __LINE__, motor_axis_no, auxBitIdx,
                 auxBitName);

        if (!strcmp("enabled", auxBitName)) {
          int bValue = getAmplifierOn(motor_axis_no);
          LOGTIME6("%s/%s:%d motor_axis_no=%u auxBitIdx=%u enabled=%d\n",
                   __FILE__, __FUNCTION__, __LINE__, motor_axis_no, auxBitIdx,
                   bValue);
          if (bValue) {
            statusReasonAux32 |= 1 << auxBitIdx;
          }
        } else if (!strcmp("Closed", auxBitName)) {
          if (negLimitSwitch) statusReasonAux32 |= 1 << auxBitIdx;
        } else if (!strcmp("Opened", auxBitName)) {
          if (posLimitSwitch) statusReasonAux32 |= 1 << auxBitIdx;
        } else if (!strcmp("Opening", auxBitName)) {
          if (motorVelocity > 0.0) statusReasonAux32 |= 1 << auxBitIdx;
        } else if (!strcmp("Closing", auxBitName)) {
          if (motorVelocity < 0.0) statusReasonAux32 |= 1 << auxBitIdx;
        } else if (!strcmp("InTheMiddle", auxBitName)) {
          if (!motorVelocity && !posLimitSwitch && !negLimitSwitch)
            statusReasonAux32 |= 1 << auxBitIdx;
        }
      }
    }

    /* the status bits */
    if (idxStatusCode == idxStatusCodeSTART) {
      idxStatusCode = idxStatusCodeBUSY;
    } else if (get_bError(motor_axis_no)) {
      idxStatusCode = idxStatusCodeERROR;
    }
#ifdef USE_IDXSTATUSCODEPOWEROFF
    else if (!getAmplifierOn(motor_axis_no))
      idxStatusCode = idxStatusCodePOWEROFF;
#endif
    else if (isMotorMoving(motor_axis_no))
      idxStatusCode = idxStatusCodeBUSY;
    else if (statusReasonAux32 & 0x0C000000)
      idxStatusCode = idxStatusCodeWARN;
    else
      idxStatusCode = idxStatusCodeIDLE;

    statusReasonAux32 &= 0x0FFFFFFF;
    statusReasonAux32 |= ((unsigned)idxStatusCode << 28);
  }
  UINTTONET(statusReasonAux32, pIndexerDevice1E0Cinterface->statusReasonAux32);
  UINTTONET(get_nErrorId(motor_axis_no), pIndexerDevice1E0Cinterface->errorID);
  UINTTONET((int)getMotorPos(motor_axis_no),
            pIndexerDevice1E0Cinterface->actualValue);
}
#endif

static void indexerMotorStatusRead5010(
    unsigned devNum, unsigned motor_axis_no, unsigned numAuxBits,
    netDevice5010interface_type *pIndexerDevice5010interface) {
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
      LOGTIME3(
          "%s/%s:%d motor_axis_no=%u idxStatusCodeSTART isMotorMoving=%d\n",
          __FILE__, __FUNCTION__, __LINE__, motor_axis_no,
          isMotorMoving(motor_axis_no));
      break;
    case idxStatusCodeSTOP:
      motorStop(motor_axis_no);
      break;
    default:;
  }
  /* Build a new status word, start with 0 and fill in
     the bits */
  if (getManualSimulatorMode(motor_axis_no)) {
    /* The simulator may overwrite the whole statusReasonAux32 */
    statusReasonAux32 = get_nStatReasAUX(motor_axis_no);
    LOGTIME3("%s/%s:%d motor_axis_no=%u simulated statusReasonAux32=0x%08x\n",
             __FILE__, __FUNCTION__, __LINE__, motor_axis_no,
             statusReasonAux32);
  } else {
    statusReasonAux32 = 0;  // Reset bits
    /* reason bits */
    if (getPosLimitSwitch(motor_axis_no)) statusReasonAux32 |= 0x08000000;
    if (getNegLimitSwitch(motor_axis_no)) statusReasonAux32 |= 0x04000000;
    {
      unsigned auxBitIdx = 0;
      for (auxBitIdx = 0; auxBitIdx < numAuxBits; auxBitIdx++) {
        const char *auxBitName =
            (const char *)&indexerDeviceAbsStraction[devNum].auxName[auxBitIdx];
        LOGTIME6("%s/%s:%d motor_axis_no=%u auxBitIdx=%u auxBitName=%s\n",
                 __FILE__, __FUNCTION__, __LINE__, motor_axis_no, auxBitIdx,
                 auxBitName);
        if (!strcmp("homing", auxBitName)) {
          if (isMotorHoming(motor_axis_no)) {
            statusReasonAux32 |= 1 << auxBitIdx;
          }
        } else if (!strcmp("homed", auxBitName)) {
          int bValue = getAxisHomed(motor_axis_no);
          LOGTIME6("%s/%s:%d motor_axis_no=%u auxBitIdx=%u homed=%d\n",
                   __FILE__, __FUNCTION__, __LINE__, motor_axis_no, auxBitIdx,
                   bValue);
          if (bValue) {
            statusReasonAux32 |= 1 << auxBitIdx;
          }
        } else if (!strcmp("notHomed", auxBitName)) {
          int bValue = getAxisHomed(motor_axis_no);
          LOGTIME6("%s/%s:%d motor_axis_no=%u auxBitIdx=%u homed=%d\n",
                   __FILE__, __FUNCTION__, __LINE__, motor_axis_no, auxBitIdx,
                   bValue);
          if (!bValue) {
            statusReasonAux32 |= 1 << auxBitIdx;
          }
        } else if (!strcmp("enabled", auxBitName)) {
          int bValue = getAmplifierOn(motor_axis_no);
          LOGTIME6("%s/%s:%d motor_axis_no=%u auxBitIdx=%u enabled=%d\n",
                   __FILE__, __FUNCTION__, __LINE__, motor_axis_no, auxBitIdx,
                   bValue);
          if (bValue) {
            statusReasonAux32 |= 1 << auxBitIdx;
          }
        } else if (!strcmp("localMode", auxBitName)) {
          int bValue = getLocalmode(motor_axis_no);
          LOGTIME6("%s/%s:%d motor_axis_no=%u auxBitIdx=%u localMode=%d\n",
                   __FILE__, __FUNCTION__, __LINE__, motor_axis_no, auxBitIdx,
                   bValue);
          if (bValue) {
            statusReasonAux32 |= 1 << auxBitIdx;
          }
        } else if (!strcmp("InterlockBwd", auxBitName)) {
          int bValue = getInterlockBwd(motor_axis_no);
          LOGTIME6("%s/%s:%d motor_axis_no=%u auxBitIdx=%u interlockBwd=%d\n",
                   __FILE__, __FUNCTION__, __LINE__, motor_axis_no, auxBitIdx,
                   bValue);
          if (bValue) {
            statusReasonAux32 |= 1 << auxBitIdx;
          }
        } else if (!strcmp("InterlockFwd", auxBitName)) {
          int bValue = getInterlockFwd(motor_axis_no);
          LOGTIME6("%s/%s:%d motor_axis_no=%u auxBitIdx=%u interlockFwd=%d\n",
                   __FILE__, __FUNCTION__, __LINE__, motor_axis_no, auxBitIdx,
                   bValue);
          if (bValue) {
            statusReasonAux32 |= 1 << auxBitIdx;
          }
        }
      }
    }
    /* the status bits */
    if (idxStatusCode == idxStatusCodeSTART) {
      idxStatusCode = idxStatusCodeBUSY;
    } else if (get_bError(motor_axis_no)) {
      idxStatusCode = idxStatusCodeERROR;
    }
#ifdef USE_IDXSTATUSCODEPOWEROFF
    else if (!getAmplifierOn(motor_axis_no))
      idxStatusCode = idxStatusCodePOWEROFF;
#endif
    else if (isMotorMoving(motor_axis_no))
      idxStatusCode = idxStatusCodeBUSY;
    // else if (statusReasonAux32 & 0x02000000)
    // idxStatusCode = idxStatusCodeERROR;
    else if (statusReasonAux32 & 0x0C000000)
      idxStatusCode = idxStatusCodeWARN;
    else
      idxStatusCode = idxStatusCodeIDLE;

    statusReasonAux32 |= ((unsigned)idxStatusCode << 28);
    LOGTIME6(
        "%s/%s:%d motor_axis_no=%u "
        "statusReasonAux32=0x%08x "
        "idxStatusCode=0x%x\n",
        __FILE__, __FUNCTION__, __LINE__, motor_axis_no, statusReasonAux32,
        idxStatusCode);
  };

  UINTTONET(statusReasonAux32, pIndexerDevice5010interface->statusReasonAux32);
  UINTTONET(get_nErrorId(motor_axis_no), pIndexerDevice5010interface->errorID);
}

/* Reads a parameter.
   All return values are returned as double,
   the call will convert into int32 or real32 if needed
*/
static unsigned indexerMotorParamRead(unsigned motor_axis_no,
                                      unsigned paramIndex, double *fRet)

{
  uint16_t ret = PARAM_IF_CMD_DONE | paramIndex;
  if (motor_axis_no >= MAX_AXES) {
    return PARAM_IF_CMD_ERR_NO_IDX;
  }

  init_axis((int)motor_axis_no);

  switch (paramIndex) {
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
    case PARAM_IDX_REFSPEED_FLOAT32:
      *fRet = cmd_Motor_cmd[motor_axis_no].fRefSpeed;
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
    case PARAM_IDX_USR_MIN_EN_FLOAT32:
      *fRet = getEnableLowSoftLimit(motor_axis_no);
      return ret;
    case PARAM_IDX_USR_MAX_EN_FLOAT32:
      *fRet = getEnableHighSoftLimit(motor_axis_no);
      return ret;
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
      *fRet = (double)0.0;
      return ret;

    default:
      break;
  }

  return PARAM_IF_CMD_ERR_NO_IDX | paramIndex;
}

/* Writes a parameter.
   the call will convert into int32 or real32 if needed
*/
static unsigned indexerMotorParamWrite(unsigned motor_axis_no,
                                       unsigned paramIndex, double fValue)

{
  uint16_t ret = PARAM_IF_CMD_DONE | paramIndex;
  if (motor_axis_no >= MAX_AXES) {
    return PARAM_IF_CMD_ERR_NO_IDX;
  }

  init_axis((int)motor_axis_no);
  LOGTIME3("%s/%s:%d motor_axis_no=%u paramIndex=%u, fValue=%f\n", __FILE__,
           __FUNCTION__, __LINE__, motor_axis_no, paramIndex, fValue);

  switch (paramIndex) {
    case PARAM_IDX_OPMODE_AUTO_UINT32: {
      /* param = 1 means amplifier off.
         param = 0 means "on", then ramp up */
      int newOnNotOff = fValue ? 0 : 1;
      int oldOnNotOff = getAmplifierOn(motor_axis_no);
      if (oldOnNotOff != newOnNotOff) {
        if (newOnNotOff) {
          /* Ramp up */
          setAmplifierPercent(motor_axis_no, 97);
        } else {
          /* Stop; then power off */
          motorStop(motor_axis_no);
          setAmplifierPercent(motor_axis_no, 0);
        }
      }
    }
      return ret;
    case PARAM_IDX_HOME_POSITION_FLOAT32:
      setHomePos(motor_axis_no, fValue);
      return ret;
    case PARAM_IDX_USR_MAX_FLOAT32:
      setHighSoftLimitPos(motor_axis_no, fValue);
      return ret;
    case PARAM_IDX_USR_MIN_FLOAT32:
      setLowSoftLimitPos(motor_axis_no, fValue);
      return ret;
    case PARAM_IDX_SPEED_FLOAT32: {
      double maxVelocity = getMaxVelocity(motor_axis_no);
      if (maxVelocity && fValue > maxVelocity) fValue = maxVelocity;
      setNxtMoveVelocity(motor_axis_no, fValue);
    }
      return ret;
    case PARAM_IDX_ACCEL_FLOAT32: {
      double maxAcceleration = getMaxAcceleration(motor_axis_no);
      if (maxAcceleration && fValue > maxAcceleration) fValue = maxAcceleration;
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

static void indexerMotorParamInterface(unsigned motor_axis_no, unsigned offset,
                                       unsigned lenInPlcPara) {
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
    LOGTIME6("%s/%s:%d motor_axis_no=%u setting to PARAM_IF_CMD_DONE\n",
             __FILE__, __FUNCTION__, __LINE__, motor_axis_no);
    uintToNet(uValue, &netData.memoryBytes[offset], 2);
  }
  if (!(paramCommand & PARAM_IF_CMD_MASKPARAM_DONE)) {
    LOGTIME6(
        "%s/%s:%d NOT done motor_axis_no=%u paramIndex=%u offset=%u "
        "paramCommand=0x%4x uValue=0x%x lenInPlcPara=%u\n",
        __FILE__, __FUNCTION__, __LINE__, motor_axis_no, paramIndex, offset,
        paramCommand, uValue, lenInPlcPara);
  }
  if (paramCommand == PARAM_IF_CMD_BUSY) {
    switch (paramIndex) {
      case PARAM_IDX_FUN_MOVE_VELOCITY:
        if (!cmd_Motor_cmd[motor_axis_no].paramIfBusyMoveVelocity) {
          ret = PARAM_IF_CMD_DONE | paramIndex;
        }
        if (!isMotorMoving(motor_axis_no)) {
          ret = PARAM_IF_CMD_DONE | paramIndex;
        }
        break;
      case PARAM_IDX_FUN_REFERENCE:
        if (!isMotorMoving(motor_axis_no)) {
          ret = PARAM_IF_CMD_DONE | paramIndex;
        }
        break;
      default:
        ret = PARAM_IF_CMD_DONE | paramIndex;
    }
    /* put DONE into the process image */
    uintToNet(ret, &netData.memoryBytes[offset], 2);
  }
  if (paramCommand == PARAM_IF_CMD_DOWRITE) {
    double fValue;
    fValue = netToDouble(&netData.memoryBytes[offset + 2], lenInPlcPara);
    ret = PARAM_IF_CMD_ERR_NO_IDX;
    switch (paramIndex) {
      case PARAM_IDX_OPMODE_AUTO_UINT32:
        /* Comes as an uint via the wire */
        fValue =
            (double)netToUint(&netData.memoryBytes[offset + 2], lenInPlcPara);
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
      case PARAM_IDX_FUN_REFERENCE: {
        int direction = 0;
        double max_velocity = 10;
        double acceleration = 3;
        if (getAmplifierOn(motor_axis_no)) {
          moveHomeProc(motor_axis_no, direction,
                       cmd_Motor_cmd[motor_axis_no].nHomProc,
                       getHomePos(motor_axis_no), max_velocity, acceleration);
        } else {
          set_nErrorId(motor_axis_no, 0x4260);
        }
        ret = PARAM_IF_CMD_BUSY | paramIndex;
      } break;
      case PARAM_IDX_FUN_MOVE_VELOCITY: {
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
        if (getAmplifierOn(motor_axis_no)) {
          moveVelocity(motor_axis_no, direction, fVelocity,
                       getNxtMoveAcceleration(motor_axis_no));
        } else {
          set_nErrorId(motor_axis_no, 0x4260);
        }
        ret = PARAM_IF_CMD_BUSY | paramIndex;
      } break;
      case PARAM_IDX_FUN_SET_POSITION: {
        setPosHome(motor_axis_no, fValue);
      }
        ret = PARAM_IF_CMD_DONE | paramIndex;
        break;
    }
    /* put DONE (or ERROR) into the process image */
    uintToNet(ret, &netData.memoryBytes[offset], 2);
  }
  if (paramCommand == PARAM_IF_CMD_DOREAD) {
    double fRet;
    /* do the read */
    ret = indexerMotorParamRead(motor_axis_no, paramIndex, &fRet);
    /* put DONE (or ERROR) into the process image */
    uintToNet(ret, &netData.memoryBytes[offset], 2);
    if ((ret & PARAM_IF_CMD_MASKPARAM_IF_CMD_MASK) == PARAM_IF_CMD_DONE) {
      switch (paramIndex) {
        case PARAM_IDX_OPMODE_AUTO_UINT32:
          uintToNet((unsigned)fRet, &netData.memoryBytes[offset + 2],
                    lenInPlcPara);
          break;
        default:
          doubleToNet(fRet, &netData.memoryBytes[offset + 2], lenInPlcPara);
          break;
      }
    }
  }
  LOGTIME6(
      "%s/%s:%d indexerMotorParamRead motor_axis_no=%u paramIndex=%u "
      "uValue=%x "
      "ret=%x\n",
      __FILE__, __FUNCTION__, __LINE__, motor_axis_no, paramIndex, uValue, ret);
}

static int indexerHandleIndexerCmd(unsigned offset, unsigned lenInPlc,
                                   unsigned uValue) {
  unsigned devNum = uValue & 0xFF;
  unsigned infoType = (uValue >> 8) & 0x7F;
  unsigned maxDevNum = NUM_DEVICES - 1;
  unsigned axisNo = indexerDeviceAbsStraction[devNum].axisNo;
  LOGTIME6(
      "%s/%s:%d offset=%u lenInPlc=%u uValue=0x%x devNum=%u maxDevNum=%u "
      "infoType=%u\n",
      __FILE__, __FUNCTION__, __LINE__, offset, lenInPlc, uValue, devNum,
      maxDevNum, infoType);
  memset(&netData.memoryStruct.indexer_info, 0,
         sizeof(netData.memoryStruct.indexer_info));
  UINTTONET(uValue, netData.memoryStruct.indexer_ack);
  if (devNum >= NUM_DEVICES) {
    netData.memoryStruct.indexer_ack[1] |= 0x80; /* ACK in high byte */
    return 0;
  }
  switch (infoType) {
    case 0:
      /* get values from device table */
      UINTTONET(indexerDeviceAbsStraction[devNum].typeCode,
                netData.memoryStruct.indexer_info.infoType0.typeCode);
      UINTTONET(indexerDeviceAbsStraction[devNum].sizeInBytes,
                netData.memoryStruct.indexer_info.infoType0.size);
      UINTTONET(indexerDeviceAbsStraction[devNum].unitCode,
                netData.memoryStruct.indexer_info.infoType0.unit);
      DOUBLETONET(indexerDeviceAbsStraction[devNum].absMin,
                  netData.memoryStruct.indexer_info.infoType0.absMin);
      DOUBLETONET(indexerDeviceAbsStraction[devNum].absMax,
                  netData.memoryStruct.indexer_info.infoType0.absMax);
      if (!devNum) {
        /* The indexer himself. */
        unsigned flags = 0x80000000; /* extended indexer */
        UINTTONET(offsetIndexer,
                  netData.memoryStruct.indexer_info.infoType0.offset);
        UINTTONET(flags, netData.memoryStruct.indexer_info.infoType0.flags);
      } else {
        unsigned auxIdx;
        unsigned flags = 0;
        unsigned maxAuxIdx;
        if (axisNo) {
          init_axis((int)axisNo);
          if (indexerDeviceAbsStraction[devNum].typeCode == 0x5010) {
            DOUBLETONET(getLowHardLimitPos((int)axisNo),
                        netData.memoryStruct.indexer_info.infoType0.absMin);
            DOUBLETONET(getHighHardLimitPos((int)axisNo),
                        netData.memoryStruct.indexer_info.infoType0.absMax);
          }
        }
        maxAuxIdx = sizeof(indexerDeviceAbsStraction[devNum].auxName) /
                    sizeof(indexerDeviceAbsStraction[devNum].auxName[0]);

        for (auxIdx = 0; auxIdx < maxAuxIdx; auxIdx++) {
          if (strlen(indexerDeviceAbsStraction[devNum].auxName[auxIdx])) {
            flags |= (1 << auxIdx);
          }
          LOGTIME6("%s/%s:%d devNum=%u auxIdx=%u flagsLow=0x%x\n", __FILE__,
                   __FUNCTION__, __LINE__, devNum, auxIdx, flags);
        }
        UINTTONET(flags, netData.memoryStruct.indexer_info.infoType0.flags);

        /* Calculate offset */
        {
          unsigned offset =
              (unsigned)((void *)&netData.memoryStruct.indexer_info -
                         (void *)&netData);
          unsigned tmpDevNum = 0;
          LOGTIME6("%s/%s:%d devNum=%u offset=%u\n", __FILE__, __FUNCTION__,
                   __LINE__, devNum, offset);
          while (tmpDevNum < devNum) {
            LOGTIME6("%s/%s:%d devNum=%u tmpDevNum=%u (%s) offsetW=%u (0x%x)\n",
                     __FILE__, __FUNCTION__, __LINE__, devNum, tmpDevNum,
                     indexerDeviceAbsStraction[tmpDevNum].devName, offset,
                     offset);
            offset += indexerDeviceAbsStraction[tmpDevNum].sizeInBytes;
            tmpDevNum++;
          }
          UINTTONET(offset, netData.memoryStruct.indexer_info.infoType0.offset);
        }
      }
      LOGTIME6(
          "%s/%s:%d devNum=%u axisNo=%u netData=%p indexer=%p delta=%u "
          "typeCode=%x sizeW=%u offsetW=%u flagsLow=0x%x ack=0x%x\n",
          __FILE__, __FUNCTION__, __LINE__, devNum, axisNo, &netData,
          &netData.memoryStruct.indexer_info,
          (unsigned)((void *)&netData.memoryStruct.indexer_info -
                     (void *)&netData),
          NETTOUINT(netData.memoryStruct.indexer_info.infoType0.typeCode),
          NETTOUINT(netData.memoryStruct.indexer_info.infoType0.size),
          NETTOUINT(netData.memoryStruct.indexer_info.infoType0.offset),
          NETTOUINT(netData.memoryStruct.indexer_info.infoType0.flags),
          NETTOUINT(netData.memoryStruct.indexer_ack));

      netData.memoryStruct.indexer_ack[1] |= 0x80; /* ACK in high byte */
      return 0;
    case 1:
      UINTTONET(indexerDeviceAbsStraction[devNum].sizeInBytes,
                netData.memoryStruct.indexer_info.infoType1.size);
      netData.memoryStruct.indexer_ack[1] |= 0x80; /* ACK in high byte */
      return 0;
#if 0
  case 3:
    UINTTONET(indexerDeviceAbsStraction[devNum].unitCode,
              netData.memoryStruct.indexer_info.infoType3.unitcode);
    netData.memoryStruct.indexer_ack[1] |= 0x80; /* ACK in high byte */
    return 0;
#endif
    case 4:
      /* get values from device table */
      strncpy(&netData.memoryStruct.indexer_info.infoType4.name[0],
              indexerDeviceAbsStraction[devNum].devName,
              sizeof(netData.memoryStruct.indexer_info.infoType4.name));
      LOGTIME3("%s/%s:%d devName=%s idxName=%s\n", __FILE__, __FUNCTION__,
               __LINE__, indexerDeviceAbsStraction[devNum].devName,
               &netData.memoryStruct.indexer_info.infoType4.name[0]);
      netData.memoryStruct.indexer_ack[1] |= 0x80; /* ACK in high byte */
      return 0;
    case 5:                                        /* version */
    case 6:                                        /* author 1 */
    case 7:                                        /* author 2 */
      netData.memoryStruct.indexer_ack[1] |= 0x80; /* ACK in high byte */
      return 0;
    case 15: {
      unsigned byteIdx;
      memset(&netData.memoryStruct.indexer_info.infoType15, 0,
             sizeof(netData.memoryStruct.indexer_info.infoType15));
      for (byteIdx = 0;
           byteIdx <
           sizeof(netData.memoryStruct.indexer_info.infoType15.parameters);
           byteIdx++) {
        unsigned bitIdx;
        unsigned parameter = 0;
        size_t size_of_param;
        size_of_param =
            8 *
            sizeof(netData.memoryStruct.indexer_info.infoType15.parameters[0]);
        for (bitIdx = 0; bitIdx < size_of_param; bitIdx++) {
          permPTyp permP;
          unsigned param_idx = byteIdx * size_of_param + bitIdx;
          permP = indexerDeviceAbsStraction[devNum].permP[param_idx];
          LOGTIME6(
              "%s/%s:%d devNum=%u param_idx=%u byteIdx=%02u bitIdx=%u "
              "permP=%d\n",
              __FILE__, __FUNCTION__, __LINE__, devNum, param_idx, byteIdx,
              bitIdx, (int)permP);
          if (permP != permPNone) parameter |= (1 << bitIdx);
        }
        LOGTIME6("%s/%s:%d devNum=%u byteIdx=%02u parameter=0x%02X\n", __FILE__,
                 __FUNCTION__, __LINE__, devNum, byteIdx, parameter);
        netData.memoryStruct.indexer_info.infoType15.parameters[byteIdx] =
            parameter;
      }
    }
      netData.memoryStruct.indexer_ack[1] |= 0x80; /* ACK in high byte */
      return 0;
    default:
      if (infoType >= 16 && infoType <= 39) {
        /* Support for aux bits 23..0 */
        strncpy(&netData.memoryStruct.indexer_info.infoType4.name[0],
                indexerDeviceAbsStraction[devNum].auxName[infoType - 16],
                sizeof(netData.memoryStruct.indexer_info.infoType4.name));
        netData.memoryStruct.indexer_ack[1] |= 0x80; /* ACK in high byte */
        return 0;
      } else if (infoType >= 40 && infoType < 72) {
        /* custom parameter name */
        memset(&netData.memoryStruct.indexer_info.infoType4.name, 0,
               sizeof(netData.memoryStruct.indexer_info.infoType4.name));
        netData.memoryStruct.indexer_ack[1] |= 0x80; /* ACK in high byte */
        return 0;
      }

      RETURN_ERROR_OR_DIE(__LINE__, "%s/%s:%d invalid infoType. infoType=%d",
                          __FILE__, __FUNCTION__, __LINE__, infoType);
  }
  return __LINE__;
}

/*************************************************************************/
int indexerHandleADS_ADR_getUInt(unsigned adsport, unsigned offset,
                                 unsigned lenInPlc, unsigned *uValue) {
  unsigned ret;
  init();
  if (offset + lenInPlc >= sizeof(netData)) return __LINE__;
  if (offset & 0x1) /* Must be even */
    return __LINE__;
  ret = netToUint(&netData.memoryBytes[offset], lenInPlc);
  *uValue = ret;
  /*
    LOGTIME3("%s/%s:%d adsport=%u offset=%u lenInPlc=%u mot1=%u ret=%u
    (0x%x)\n",
    __FILE__, __FUNCTION__, __LINE__,
    adsport,
    offset,
    lenInPlc,
    offsetMotor1StatusReasonAux,
    ret, ret);
  */
  return 0;
}

int indexerHandleADS_ADR_putUInt(unsigned adsport, unsigned offset,
                                 unsigned lenInPlc, unsigned uValue) {
  init();
  LOGTIME6("%s/%s:%d adsport=%u offset=%u lenInPlc=%u uValue=%u (%x)\n",
           __FILE__, __FUNCTION__, __LINE__, adsport, offset, lenInPlc, uValue,
           uValue);
  if (offset == offsetIndexer) {
    return indexerHandleIndexerCmd(offset, lenInPlc, uValue);
  } else if (offset < (sizeof(netData) / sizeof(uint16_t))) {
    uintToNet(uValue, &netData.memoryBytes[offset], lenInPlc);
    return 0;
  }
  LOGERR("%s/%s:%d adsport=%u offset=%u lenInPlc=%u uValue=%u (%x)sizeof=%lu\n",
         __FILE__, __FUNCTION__, __LINE__, adsport, offset, lenInPlc, uValue,
         uValue, (unsigned long)(sizeof(netData) / sizeof(uint16_t)));

  return __LINE__;
}

int indexerHandleADS_ADR_getFloat(unsigned adsport, unsigned offset,
                                  unsigned lenInPlc, double *fValue) {
  double fRet;
  init();
  if (offset + lenInPlc >= sizeof(netData)) return 1;
  if ((lenInPlc == 4) || (lenInPlc == 8)) {
    fRet = netToDouble(&netData.memoryBytes[offset], lenInPlc);
    *fValue = fRet;
    return 0;
  }
  return __LINE__;
}

int indexerHandleADS_ADR_putFloat(unsigned adsport, unsigned offset,
                                  unsigned lenInPlc, double fValue) {
  init();
  LOGTIME3("%s/%s:%d adsport=%u offset=%u lenInPlc=%u fValue=%f\n", __FILE__,
           __FUNCTION__, __LINE__, adsport, offset, lenInPlc, fValue);
  if (offset + lenInPlc >= sizeof(netData)) return 1;
  if ((lenInPlc == 4) || (lenInPlc == 8)) {
    doubleToNet(fValue, &netData.memoryBytes[offset], lenInPlc);
    return 0;
  }
  return __LINE__;
};

int indexerHandleADS_ADR_getString(unsigned adsport, unsigned offset,
                                   unsigned lenInPlc, char **sValue) {
  init();
  if (offset + lenInPlc > sizeof(netData)) {
    RETURN_ERROR_OR_DIE(__LINE__,
                        "%s/%s:%d out of range: offset=%u lenInPlc=%u",
                        __FILE__, __FUNCTION__, __LINE__, offset, lenInPlc);
  }
  *sValue = (char *)&netData.memoryBytes[offset];
  return 0;
};

int indexerHandleADS_ADR_getMemory(unsigned adsport, unsigned offset,
                                   unsigned lenInPlc, void *buf) {
  init();
  if (offset + lenInPlc > sizeof(netData)) {
    RETURN_ERROR_OR_DIE(
        __LINE__, "%s/%s:%d out of range: offset=%u lenInPlc=%u (0x%x)",
        __FILE__, __FUNCTION__, __LINE__, offset, lenInPlc, lenInPlc);
  }
  memcpy(buf, &netData.memoryBytes[offset], lenInPlc);
  return 0;
};

int indexerHandleADS_ADR_setMemory(unsigned adsport, unsigned offset,
                                   unsigned lenInPlc, void *buf) {
  init();
  if (offset + lenInPlc > sizeof(netData)) {
    RETURN_ERROR_OR_DIE(
        __LINE__, "%s/%s:%d out of range: offset=%u lenInPlc=%u (0x%x)",
        __FILE__, __FUNCTION__, __LINE__, offset, lenInPlc, lenInPlc);
  }
  memcpy(&netData.memoryBytes[offset], buf, lenInPlc);
  return 0;
};

void indexerHandlePLCcycle(void) {
  static int pLCcycleInitDone = 0;
  unsigned devNum = 0;
  init();
  while (devNum < NUM_DEVICES) {
    LOGTIME6("%s/%s:%d devNum=%u typeCode=0x%x\n", __FILE__, __FUNCTION__,
             __LINE__, devNum, indexerDeviceAbsStraction[devNum].typeCode);

    switch (indexerDeviceAbsStraction[devNum].typeCode) {
      case TYPECODE_DISCRETEINPUT_1202: {
        unsigned axisNo = indexerDeviceAbsStraction[devNum].axisNo;
        if (axisNo) {
          unsigned motor5010Num = axisNo - 1;
          if (!(strcmp(indexerDeviceAbsStraction[devNum].devName,
                       "encoderRaw"))) {
            int encoderRaw = (int)getEncoderPos(axisNo);
            LOGTIME6(
                "%s/%s:%d devNum=%u axisNo=%u motor5010Num=%u "
                "encoderRaw=%u\n",
                __FILE__, __FUNCTION__, __LINE__, devNum, axisNo, motor5010Num,
                encoderRaw);
            UINTTONET(encoderRaw,
                      netData.memoryStruct.motors5010_1202[motor5010Num]
                          .dev1202encoderRaw.value);
          }
        }
      } break;
      case TYPECODE_DISCRETEOUTPUT_1604: {
        unsigned axisNo = indexerDeviceAbsStraction[devNum].axisNo;
        if (axisNo) {
          unsigned motor5010Num = axisNo - 1;
          (void)motor5010Num;
          (void)pLCcycleInitDone;
        } else if (!strcmp("DISCRETEOUTPUT#1",
                           indexerDeviceAbsStraction[devNum].devName)) {
          unsigned value =
              NETTOUINT(netData.memoryStruct.discreteOutput1604[0].targetValue);
          /* Mirror the value back */
          UINTTONET(value,
                    netData.memoryStruct.discreteOutput1604[0].actualValue);
          /* And copy it into our input device (this is for testing only) */
          UINTTONET(value,
                    netData.memoryStruct.discreteInput1A04[0].actualValue);
          /* And into the status word. More testing : EPICS Record Alarms */
          UINTTONET(
              value,
              netData.memoryStruct.discreteInput1A04[0].statusReasonAux32);
        } else {
          LOGTIME("%s/%s:%d devNum=%u '%s' not handled\n", __FILE__,
                  __FUNCTION__, __LINE__, devNum,
                  indexerDeviceAbsStraction[devNum].devName);
        }
      } break;
      case TYPECODE_STATUSWORD_1802: {
        if (!strcmp("Cabinet#0", indexerDeviceAbsStraction[devNum].devName)) {
          unsigned value = getCabinetStatus();
          UINTTONET(value,
                    netData.memoryStruct.statusWord1802[0].statusReasonAux32);
        } else {
          LOGTIME("%s/%s:%d devNum=%u '%s' '0x%04X' not handled\n", __FILE__,
                  __FUNCTION__, __LINE__, devNum,
                  indexerDeviceAbsStraction[devNum].devName,
                  indexerDeviceAbsStraction[devNum].typeCode);
        }
      } break;

      case TYPECODE_DISCRETEINPUT_1A04: {
        if (!strcmp("DISCRETEINPUT#1",
                    indexerDeviceAbsStraction[devNum].devName)) {
          ; /* Done in DISCRETEOUTPUT#0 */
        } else {
          LOGTIME("%s/%s:%d devNum=%u '%s' '0x%04X' not handled\n", __FILE__,
                  __FUNCTION__, __LINE__, devNum,
                  indexerDeviceAbsStraction[devNum].devName,
                  indexerDeviceAbsStraction[devNum].typeCode);
        }
      } break;
      case TYPECODE_ANALOGINPUT_1B04: {
        // if (!strcmp("ANALOGINPUT#1#AnalogInWithStatus",
        if (!strcmp("ANALOGINPUT#1",
                    indexerDeviceAbsStraction[devNum].devName)) {
#if 0
          /* Copy the readback of motor 1 */
          int axisNo = 1;
          double fRet = getMotorPos(axisNo);
          unsigned nValue = 0;
          DOUBLETONET(fRet, netData.memoryStruct.analogInput1B04[0].actualValue);
          UINTTONET(nValue, netData.memoryStruct.analogInput1B04[0].statusReasonAux32);
#else
          ;
#endif
        } else {
          LOGTIME("%s/%s:%d devNum=%u '%s' '0x%04X' not handled\n", __FILE__,
                  __FUNCTION__, __LINE__, devNum,
                  indexerDeviceAbsStraction[devNum].devName,
                  indexerDeviceAbsStraction[devNum].typeCode);
        }
      } break;
      case TYPECODE_DISCRETEOTPUT_1E04: {
        unsigned axisNo = indexerDeviceAbsStraction[devNum].axisNo;
        if (axisNo) {
#ifdef HAS_1E04_SHUTTER
          /*
           * motor1E04Num starts at 0
           * all hw_motor axes start at 1, and we need to jump over
           * the 5010 axes
           */
          unsigned motor1E04Num = axisNo - NUM_MOTORS5010 - 1;
          static const unsigned numAuxBits = 24;
          int iRet = (int)(0.5 + getMotorPos((int)axisNo)); /* NINT */
          UINTTONET(iRet,
                    netData.memoryStruct.motors1E04[motor1E04Num].actualValue);
          /* status */
          indexerMotorStatusRead1E04(
              devNum, axisNo, numAuxBits,
              &netData.memoryStruct.motors1E04[motor1E04Num]);
#else
          LOGTIME("%s/%s:%d devNum=%u '%s' not handled\n", __FILE__,
                  __FUNCTION__, __LINE__, devNum,
                  indexerDeviceAbsStraction[devNum].devName);

#endif
        } else {
          LOGTIME("%s/%s:%d devNum=%u '%s' not handled\n", __FILE__,
                  __FUNCTION__, __LINE__, devNum,
                  indexerDeviceAbsStraction[devNum].devName);
        }
      } break;
      case TYPECODE_DISCRETEOTPUT_1E0C: {
        unsigned axisNo = indexerDeviceAbsStraction[devNum].axisNo;
        if (axisNo) {
#ifdef HAS_1E0C_SHUTTER_CAROUSEL
          /*
           * motor1E0CNum starts at 0
           * all hw_motor axes start at 1, and we need to jump over
           * the 5010 and 1E04axes
           */
          unsigned motor1E0CNum = axisNo - NUM_MOTORS5010 - NUM_1E04 - 1;
          static const unsigned numAuxBits = 24;
          int iRet = (int)(0.5 + getMotorPos((int)axisNo)); /* NINT */
          UINTTONET(iRet,
                    netData.memoryStruct.motors1E0C[motor1E0CNum].actualValue);
          /* status */
          indexerMotorStatusRead1E0C(
              devNum, axisNo, numAuxBits,
              &netData.memoryStruct.motors1E0C[motor1E0CNum],
              indexerDeviceAbsStraction[devNum].devName);
#else
          LOGTIME("%s/%s:%d devNum=%u '%s' not handled\n", __FILE__,
                  __FUNCTION__, __LINE__, devNum,
                  indexerDeviceAbsStraction[devNum].devName);

#endif
        } else {
          LOGTIME("%s/%s:%d devNum=%u '%s' not handled\n", __FILE__,
                  __FUNCTION__, __LINE__, devNum,
                  indexerDeviceAbsStraction[devNum].devName);
        }
      } break;
      case TYPECODE_PARAMDEVICE_5010: {
        double fRet;
        size_t lenInPlcPara = 8;
        unsigned offset;
        unsigned axisNo = indexerDeviceAbsStraction[devNum].axisNo;
        unsigned motor5010Num = axisNo - 1;
        static const unsigned numAuxBits = 24;
        offset = (unsigned)((void *)&netData.memoryStruct
                                .motors5010_1202[motor5010Num]
                                .dev5010.actualValue -
                            (void *)&netData);

        fRet = getMotorPos((int)axisNo);
        LOGTIME6(
            "%s/%s:%d devNum=%u axisNo=%u motor5010Num=%u offset=%u "
            "fRet=%f\n",
            __FILE__, __FUNCTION__, __LINE__, devNum, axisNo, motor5010Num,
            offset, (double)fRet);
        if (sim_usleep[axisNo]) {
          LOGTIME3("%s/%s:%d axis_no=%d usleep=%lu\n", __FILE__, __FUNCTION__,
                   __LINE__, axisNo, (unsigned long)sim_usleep[axisNo]);
          (void)usleep(sim_usleep[axisNo]);
        }

        doubleToNet(fRet, &netData.memoryBytes[offset], lenInPlcPara);
        /* status */
        indexerMotorStatusRead5010(
            devNum, axisNo, numAuxBits,
            &netData.memoryStruct.motors5010_1202[motor5010Num].dev5010);

        /* param interface */
        offset = (unsigned)((void *)&netData.memoryStruct
                                .motors5010_1202[motor5010Num]
                                .dev5010.paramCtrl -
                            (void *)&netData);
        LOGTIME6("%s/%s:%d devNum=%u motor5010Num=%u paramoffset=%u\n",
                 __FILE__, __FUNCTION__, __LINE__, devNum, motor5010Num,
                 offset);
        indexerMotorParamInterface(axisNo, offset, lenInPlcPara);
      } break;
      case TYPECODE_INDEXER: {
        uint16_t indexer_ack = NETTOUINT(netData.memoryStruct.indexer_ack);
        LOGTIME6("%s/%s:%d devNum=%u indexer_ack=0x%x\n", __FILE__,
                 __FUNCTION__, __LINE__, devNum, indexer_ack);

        if (!(indexer_ack & 0x8000)) {
          unsigned lenInPlc = sizeof(indexer_ack);
          indexerHandleIndexerCmd(offsetIndexer, lenInPlc, indexer_ack);
        }
      } break;
#ifdef HAS_0518
      case TYPECODE_SPECIALDEVICE_0518: {
        uint16_t ctrl_word =
            NETTOUINT(netData.memoryStruct.special0518.control);
        unsigned plcNotHostHasWritten = (ctrl_word & 0x8000) ? 1 : 0;
        unsigned numBytes = ctrl_word & 0x02FF; /* Bit 9..0 */
        int retval = 0;
        LOGTIME6(
            "%s/%s:%d devNum=%u special0518 ctrl0=0x%x crtl1=0x%x "
            "plcNotHostHasWritten=%u numBytes=%u\n",
            __FILE__, __FUNCTION__, __LINE__,
            netData.memoryStruct.special0518.control[0],
            netData.memoryStruct.special0518.control[1], devNum,
            plcNotHostHasWritten, numBytes);
        if (!ctrl_word) {
          /* Init phase */
          ctrl_word = 0x8000;
          UINTTONET(ctrl_word, netData.memoryStruct.special0518.control);
        } else if (!plcNotHostHasWritten && numBytes) {
          char *pNewline;
          pNewline =
              strchr((char *)&netData.memoryStruct.special0518.value, '\n');
          if (pNewline) {
            size_t line_len = 1 + (void *)pNewline -
                              (void *)&netData.memoryStruct.special0518.value;
            int had_cr = 0;
            *pNewline = 0; /* Remove '\n' */
            if (line_len > 1) pNewline--;
            if (*pNewline == '\r') {
              had_cr = 1;
              *pNewline = '\0';
            }
            LOGTIME3("%s/%s:%d devNum=%u special0518 value=\"%s\"\n", __FILE__,
                     __FUNCTION__, __LINE__, devNum,
                     (const char *)netData.memoryStruct.special0518.value);
            retval = handle_input_line(
                (const char *)&netData.memoryStruct.special0518.value, had_cr,
                1);
            LOGTIME6("%s/%s:%d devNum=%u special0518 value=\"%s\" retval=%d\n",
                     __FILE__, __FUNCTION__, __LINE__, devNum,
                     (const char *)netData.memoryStruct.special0518.value,
                     retval);
          } else {
            LOGTIME3("%s/%s:%d devNum=%u special0518=\"%s\"\n", __FILE__,
                     __FUNCTION__, __LINE__, devNum,
                     (const char *)netData.memoryStruct.special0518.value);
          }
          ctrl_word = snprintf(&netData.memoryStruct.special0518.value[0],
                               sizeof(netData.memoryStruct.special0518.value),
                               "%s", retval ? "Error" : "OK");
          LOGTIME3(
              "%s/%s:%d devNum=%u special0518=\"%s\" retval=%d "
              "ctrl_word_len=%u\n",
              __FILE__, __FUNCTION__, __LINE__, devNum,
              (const char *)netData.memoryStruct.special0518.value, retval,
              ctrl_word);
          ctrl_word |= 0x8000;
          UINTTONET(ctrl_word, netData.memoryStruct.special0518.control);
        }
      } break;
#endif
      default:
        LOGTIME("%s/%s:%d devNum=%u '%s' '0x%04X' not handled\n", __FILE__,
                __FUNCTION__, __LINE__, devNum,
                indexerDeviceAbsStraction[devNum].devName,
                indexerDeviceAbsStraction[devNum].typeCode);
        break;
    }
    devNum++;
  }
  pLCcycleInitDone = 1;
}
