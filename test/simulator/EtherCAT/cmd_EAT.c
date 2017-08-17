#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>
#include "sock-util.h"
#include "logerr_info.h"
#include "cmd_buf.h"
#include "hw_motor.h"
#include "cmd_EAT.h"

typedef struct
{
  int    command_no;
  unsigned nCmdData;
  int    bExecute;
  int    bReset;
  double fPosition;
  double fVelocity;
  double fAcceleration;
  double fDeceleration;
  double homeVeloTowardsHomeSensor;
  double homeVeloFromHomeSensor;
  double manualVelocitySlow;
  double manualVelocityFast;
  double inTargetPositionMonitorWindow;
  double inTargetPositionMonitorTime;
  int    inTargetPositionMonitorEnabled;
  double maximumVelocity;
  double referenceVelocity;
  double positionLagMonitoringValue;
  double positionLagFilterTime;
  double deadTimeCompensation;
  int    positionLagMonitorEnable;
  unsigned nErrorId;
} cmd_Motor_cmd_type;

typedef struct
{
  double fActPostion;
  int bEnable;
  int bReset;
  int bExecute;
  unsigned nCommand;
  unsigned nCmdData;
  double fVelocity;
  double fPosition;
  //double fAcceleration;
  //double fDecceleration;
  int bLimitFwd;
  int bLimitBwd;
  int bHomeSensor;
  int bEnabled;
  int bError;
  int nErrorId;
  double fActVelocity;
  double fActDiff;
  int bHomed;
  int bBusy;
} cmd_Motor_status_type;

/* values commanded to the motor */
static cmd_Motor_cmd_type cmd_Motor_cmd[MAX_AXES];

/* values reported back from the motor */
static cmd_Motor_status_type cmd_Motor_status[MAX_AXES];

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
    double valueLow = -1.0 * ReverseMRES;
    double valueHigh = 186.0 * ReverseMRES;
    hw_motor_init(axis_no,
                  MRES/ERES,              /* ReverseERES */
                  (100 + axis_no/10.0),   /* ParkingPOS */
                  5 * ReverseMRES,        /* maxHomeVelocityAbs */
                  valueLow,               /* lowHardLimitPos */
                  valueHigh,              /* highHardLimitPos */
                  valueLow,               /* hWlowPos */
                  valueHigh,              /* hWhighPos */
                  0);                     /* homeSwitchPos */

    cmd_Motor_cmd[axis_no].maximumVelocity = 50;
    cmd_Motor_cmd[axis_no].homeVeloTowardsHomeSensor = 10;
    cmd_Motor_cmd[axis_no].homeVeloFromHomeSensor = 5;
    cmd_Motor_cmd[axis_no].fPosition = getMotorPos(axis_no);
    cmd_Motor_cmd[axis_no].referenceVelocity = 600;
    cmd_Motor_cmd[axis_no].inTargetPositionMonitorWindow = 0.1;
    cmd_Motor_cmd[axis_no].inTargetPositionMonitorTime = 0.02;
    cmd_Motor_cmd[axis_no].inTargetPositionMonitorEnabled = 1;
    setMRES_23(axis_no, UREV);
    setMRES_24(axis_no, SREV);

    init_done[axis_no] = 1;
  }
}

static const char * const ADSPORT_equals_str = "ADSPORT=";
static const char * const Main_dot_str = "Main.";
static const char * const getAxisDebugInfoData_str = "getAxisDebugInfoData";

static const char *seperator_seperator = ";";

static int motorHandleADS_ADR_getInt(unsigned adsport,
                                     unsigned indexGroup,
                                     unsigned indexOffset,
                                     int *iValue)
{
  if (indexGroup >= 0x4000 && indexGroup < 0x5000) {
    int motor_axis_no = (int)indexGroup - 0x4000;
    switch(indexOffset)
      case 0x15:
      *iValue = cmd_Motor_cmd[motor_axis_no].inTargetPositionMonitorEnabled;
      return 0; /* Monitor */
  } else if (indexGroup >= 0x5000 && indexGroup < 0x6000) {
    int motor_axis_no = (int)indexGroup - 0x5000;
    switch(indexOffset) {
      case 0x8:
        /* Encoder direction axis1: Negative; axis2: positive */
        *iValue = motor_axis_no & 1 ? 1 : 0;
        return 0;
      case 0xB:
        *iValue = getEnableLowSoftLimit(motor_axis_no);
        return 0;
      case 0xC:
        *iValue = getEnableHighSoftLimit(motor_axis_no);
        return 0;
    }
  } else if (indexGroup >= 0x6000 && indexGroup < 0x7000) {
    int motor_axis_no = (int)indexGroup - 0x6000;
    switch(indexOffset) {
      case 0x10:
        *iValue = cmd_Motor_cmd[motor_axis_no].positionLagMonitorEnable;
        return 0;
    }
  } else if (indexGroup >= 0x7000 && indexGroup < 0x8000) {
    int motor_axis_no = (int)indexGroup - 0x7000;
    switch(indexOffset) {
      case 0x6:
        /* Motorr direction axis1: Positive; axis2: negative */
        *iValue = motor_axis_no & 1 ? 0 : 1;
        return 0;
    }
  } else if (indexGroup == 0x3040010 && indexOffset == 0x80000049) {
    *iValue = (int)getEncoderPos(1);
    return 0;
  } else if (indexGroup == 0x3040010 && indexOffset == 0x8000004F) {
    *iValue = (int)getEncoderPos(2);
    return 0;
  }
  RETURN_ERROR_OR_DIE(__LINE__, "%s/%s:%d indexGroup=0x%x indexOffset=0x%x",
                      __FILE__, __FUNCTION__, __LINE__,
                      indexGroup,
                      indexOffset);
}

/*
  ADSPORT=501/.ADR.16#5001,16#B,2,2=1; #enable low Softlimit
  ADSPORT=501/.ADR.16#5001,16#C,2,2=1; #enable high Softlimit
*/
static int motorHandleADS_ADR_putInt(unsigned adsport,
                                     unsigned indexGroup,
                                     unsigned indexOffset,
                                     int iValue)
{
  if (indexGroup >= 0x5000 && indexGroup < 0x6000) {
    int motor_axis_no = (int)indexGroup - 0x5000;
    if (indexOffset == 0xB) {
      setEnableLowSoftLimit(motor_axis_no, iValue);
      return 0;
    }
    if (indexOffset == 0xC) {
      setEnableHighSoftLimit(motor_axis_no, iValue);
      return 0;
    }
  }
  if (indexGroup >= 0x4000 && indexGroup < 0x5000) {
    int motor_axis_no = (int)indexGroup - 0x4000;
    if (indexOffset == 0x15) {
      cmd_Motor_cmd[motor_axis_no].inTargetPositionMonitorEnabled = iValue;
      return 0;
    }
  } else if (indexGroup >= 0x6000 && indexGroup < 0x7000) {
    int motor_axis_no = (int)indexGroup - 0x6000;
    switch(indexOffset) {
      case 0x10:
        cmd_Motor_cmd[motor_axis_no].positionLagMonitorEnable = iValue;
        return 0;
    }
  }

  RETURN_ERROR_OR_DIE(__LINE__, "%s/%s:%d indexGroup=0x%x indexOffset=0x%x",
               __FILE__, __FUNCTION__, __LINE__,
               indexGroup,
               indexOffset);
}


static int motorHandleADS_ADR_getFloat(unsigned adsport,
                                       unsigned indexGroup,
                                       unsigned indexOffset,
                                       double *fValue)
{
  if (indexGroup >= 0x4000 && indexGroup < 0x5000) {
    int motor_axis_no = (int)indexGroup - 0x4000;
    switch(indexOffset) {
    case 0x6:
      *fValue = cmd_Motor_cmd[motor_axis_no].homeVeloTowardsHomeSensor;
      return 0;
    case 0x7:
      *fValue = cmd_Motor_cmd[motor_axis_no].homeVeloFromHomeSensor;
      return 0;
    case 0x8:
      *fValue = cmd_Motor_cmd[motor_axis_no].manualVelocitySlow;
      return 0;
    case 0x9:
      *fValue = cmd_Motor_cmd[motor_axis_no].manualVelocityFast;
      return 0;
    case 0x16:
      *fValue = cmd_Motor_cmd[motor_axis_no].inTargetPositionMonitorWindow;
      return 0;
    case 0x17:
      *fValue = cmd_Motor_cmd[motor_axis_no].inTargetPositionMonitorTime;
      return 0;
    case 0x27:
      *fValue = cmd_Motor_cmd[motor_axis_no].maximumVelocity;
      return 0;
    case 0x104:
      *fValue = cmd_Motor_cmd[motor_axis_no].deadTimeCompensation;
      return 0;
    }
  } else if (indexGroup >= 0x5000 && indexGroup < 0x6000) {
    int motor_axis_no = (int)indexGroup - 0x5000;
    switch(indexOffset) {
      case 0xD:
        *fValue = getLowSoftLimitPos(motor_axis_no);
        return 0;
      case 0xE:
        *fValue = getHighSoftLimitPos(motor_axis_no);
        return 0;
      case 0x23:
        *fValue = getMRES_23(motor_axis_no);
        return 0;
      case 0x24:
        *fValue = getMRES_24(motor_axis_no);
        return 0;
    }
  } else if (indexGroup >= 0x6000 && indexGroup < 0x7000) {
    int motor_axis_no = (int)indexGroup - 0x6000;
    switch(indexOffset) {
      case 0x12:
        *fValue = cmd_Motor_cmd[motor_axis_no].positionLagMonitoringValue;
        return 0;
      case 0x13:
        *fValue = cmd_Motor_cmd[motor_axis_no].positionLagFilterTime;
        return 0;
    }
  } else if (indexGroup >= 0x7000 && indexGroup < 0x8000) {
    int motor_axis_no = (int)indexGroup - 0x7000;
    switch(indexOffset) {
    case 0x101:
      *fValue = cmd_Motor_cmd[motor_axis_no].referenceVelocity;
      return 0;
    }

  }
  return -1;
}

/*
  ADSPORT=501/.ADR.16#5001,16#D,8,5=-13.5; #low Softlimit
  ADSPORT=501/.ADR.16#5001,16#E,8,5=140.0; #high Softlimit
*/
static int motorHandleADS_ADR_putFloat(unsigned adsport,
                                       unsigned indexGroup,
                                       unsigned indexOffset,
                                       double fValue)
{
  if (indexGroup >= 0x4000 && indexGroup < 0x5000) {
    int motor_axis_no = (int)indexGroup - 0x4000;
    switch(indexOffset) {
    case 0x6:
      cmd_Motor_cmd[motor_axis_no].homeVeloTowardsHomeSensor = fValue;
      return 0;
    case 0x7:
      cmd_Motor_cmd[motor_axis_no].homeVeloFromHomeSensor = fValue;
      return 0;
    case 0x8:
      cmd_Motor_cmd[motor_axis_no].manualVelocitySlow = fValue;
      return 0;
    case 0x9:
      cmd_Motor_cmd[motor_axis_no].manualVelocityFast = fValue;
      return 0;
    case 0x27:
      cmd_Motor_cmd[motor_axis_no].maximumVelocity = fValue;
      return 0;
    case 0x104:
      cmd_Motor_cmd[motor_axis_no].deadTimeCompensation = fValue;
      return 0;
    }
  } else if (indexGroup >= 0x5000 && indexGroup < 0x6000) {
    int motor_axis_no = (int)indexGroup - 0x5000;
    switch(indexOffset) {
      case 0xD:
        setLowSoftLimitPos(motor_axis_no, fValue);
        return 0;
      case 0xE:
        setHighSoftLimitPos(motor_axis_no, fValue);
        return 0;
      case 0x23:
        return setMRES_23(motor_axis_no, fValue);
      case 0x24:
        return setMRES_24(motor_axis_no, fValue);
    }
  } else if (indexGroup >= 0x6000 && indexGroup < 0x7000) {
    int motor_axis_no = (int)indexGroup - 0x6000;
    (void)motor_axis_no;
    switch(indexOffset) {
      case 0x12:
        cmd_Motor_cmd[motor_axis_no].positionLagMonitoringValue = fValue;
        return 0;
      case 0x13:
        cmd_Motor_cmd[motor_axis_no].positionLagFilterTime = fValue;
        return 0;
    }
  } else if (indexGroup >= 0x7000 && indexGroup < 0x8000) {
    int motor_axis_no = (int)indexGroup - 0x7000;
    switch(indexOffset) {
    (void)motor_axis_no;
    case 0x101:
      cmd_Motor_cmd[motor_axis_no].referenceVelocity = fValue ;
      return 0;
    }
  }
  return __LINE__;
}


/*
  ADSPORT=501/.ADR.16#5001,16#B,2,2=1;
*/
static int motorHandleADS_ADR(const char *arg)
{
  const char *myarg_1 = NULL;
  unsigned adsport = 0;
  unsigned indexGroup = 0;
  unsigned indexOffset = 0;
  unsigned len_in_PLC = 0;
  unsigned type_in_PLC = 0;
  int nvals;
  nvals = sscanf(arg, "%u/.ADR.16#%x,16#%x,%u,%u=",
                 &adsport,
                 &indexGroup,
                 &indexOffset,
                 &len_in_PLC,
                 &type_in_PLC);
  LOGINFO6("%s/%s:%d "
           "nvals=%d adsport=%u indexGroup=0x%x indexOffset=0x%x len_in_PLC=%u type_in_PLC=%u\n",
           __FILE__, __FUNCTION__, __LINE__,
           nvals,
           adsport,
           indexGroup,
           indexOffset,
           len_in_PLC,
           type_in_PLC);

  if (nvals != 5) return __LINE__;
  if (adsport != 501) return __LINE__;

  myarg_1 = strchr(arg, '=');
  if (myarg_1) {
    myarg_1++; /* Jump over '=' */
    switch (type_in_PLC) {
      case 5: {
        double fValue;
        if (len_in_PLC != 8) return __LINE__;
        nvals = sscanf(myarg_1, "%lf", &fValue);
        if (nvals != 1) return __LINE__;
        return motorHandleADS_ADR_putFloat(adsport,
                                        indexGroup,
                                        indexOffset,
                                        fValue);
      }
        break;
      case 2: {
        int iValue;
        if (len_in_PLC != 2) return __LINE__;
        nvals = sscanf(myarg_1, "%d", &iValue);
        if (nvals != 1) return __LINE__;
        return motorHandleADS_ADR_putInt(adsport,
                                      indexGroup,
                                      indexOffset,
                                      iValue);
      }
        break;
      default:
        return __LINE__;
    }
  }
  myarg_1 = strchr(arg, '?');
  if (myarg_1) {
    int res;
    myarg_1++; /* Jump over '?' */
    switch (type_in_PLC) {
      case 5: {
        double fValue;
        if (len_in_PLC != 8) return __LINE__;
        res = motorHandleADS_ADR_getFloat(adsport,
                                          indexGroup,
                                          indexOffset,
                                          &fValue);
        if (res) return res;
        cmd_buf_printf("%g", fValue);
        return -1;
      }
        break;
      case 2: {
        int res;
        int iValue = -1;
        if (len_in_PLC != 2) return __LINE__;
        res = motorHandleADS_ADR_getInt(adsport,
                                        indexGroup,
                                        indexOffset,
                                        &iValue);
        if (res) return res;
        cmd_buf_printf("%d", iValue);
        return -1;
      }
        break;
      default:
        return __LINE__;
    }
  }
  return __LINE__;
}


static void motorHandleOneArg(const char *myarg_1)
{
  static const char * const ADSPORT_sFeaturesQ_str = "ADSPORT=852/.THIS.sFeatures?";
  const char *myarg = myarg_1;
  int iValue = 0;
  double fValue = 0;
  int motor_axis_no = 0;
  int nvals = 0;

  /* ADSPORT=852/.THIS.sFeatures? */
  if (0 == strcmp(myarg_1, ADSPORT_sFeaturesQ_str)) {
    cmd_buf_printf("%s", "sim");
    return;
  }

  /* ADSPORT= */
  if (!strncmp(myarg_1, ADSPORT_equals_str, strlen(ADSPORT_equals_str))) {
    int err_code;
    myarg_1 += strlen(ADSPORT_equals_str);
    err_code = motorHandleADS_ADR(myarg_1);
    if (err_code == -1) return;
    if (err_code == 0) {
      cmd_buf_printf("OK");
      return;
    }
    RETURN_OR_DIE("%s/%s:%d myarg_1=%s err_code=%d",
                  __FILE__, __FUNCTION__, __LINE__,
                  myarg_1,
                  err_code);
  }
  /* getAxisDebugInfoData(1) */
  if (!strncmp(myarg_1, getAxisDebugInfoData_str, strlen(getAxisDebugInfoData_str))) {
    myarg_1 += strlen(getAxisDebugInfoData_str);
    nvals = sscanf(myarg_1, "(%d)", &motor_axis_no);
    if (nvals == 1) {
      char buf[80];
      getAxisDebugInfoData(motor_axis_no, buf, sizeof(buf));
      cmd_buf_printf("%s", buf);
      return;
    } else {
      RETURN_OR_DIE("%s/%s:%d line=%s nvals=%d",
                    __FILE__, __FUNCTION__, __LINE__,
                    myarg, nvals);
    }
  }

  /* Main.*/
  if (!strncmp(myarg_1, Main_dot_str, strlen(Main_dot_str))) {
    myarg_1 += strlen(Main_dot_str);
  }

  /* From here on, only M1. commands */
  /* e.g. M1.nCommand=3 */
  nvals = sscanf(myarg_1, "M%d.", &motor_axis_no);
  if (nvals != 1) {
    RETURN_OR_DIE("%s/%s:%d line=%s nvals=%d",
                  __FILE__, __FUNCTION__, __LINE__,
                  myarg, nvals);
  }
  AXIS_CHECK_RETURN(motor_axis_no);
  myarg_1 = strchr(myarg_1, '.');
  if (!myarg_1) {
    RETURN_OR_DIE("%s/%s:%d line=%s missing '.'",
                  __FILE__, __FUNCTION__, __LINE__,
                  myarg);
  }
  myarg_1++; /* Jump over '.' */
  if (0 == strcmp(myarg_1, "bBusy?")) {
    cmd_buf_printf("%d", isMotorMoving(motor_axis_no));
    return;
  }
  /* bError?  */
  if (!strcmp(myarg_1, "bError?")) {
    cmd_buf_printf("%d", get_bError(motor_axis_no));
    return;
  }

  /* bEnable? bEnabled? Both are the same in the simulator */
  if (!strcmp(myarg_1, "bEnable?") || !strcmp(myarg_1, "bEnabled?")) {
    cmd_buf_printf("%d",getAmplifierOn(motor_axis_no));
    return;
  }
  /* bExecute? */
  if (!strcmp(myarg_1, "bExecute?")) {
    cmd_buf_printf("%d", cmd_Motor_cmd[motor_axis_no].bExecute);
    return;
  }
  /* bHomeSensor? */
  if (0 == strcmp(myarg_1, "bHomeSensor?")) {
    cmd_buf_printf("%d", getAxisHome(motor_axis_no));
    return;
  }
  /* bLimitBwd? */
  if (0 == strcmp(myarg_1, "bLimitBwd?")) {
    cmd_buf_printf("%d", getNegLimitSwitch(motor_axis_no) ? 0 : 1);
    return;
  }
  /* bLimitFwd? */
  if (0 == strcmp(myarg_1, "bLimitFwd?")) {
    cmd_buf_printf("%d", getPosLimitSwitch(motor_axis_no) ? 0 : 1);
    return;
  }
  /* bHomed? */
  if (0 == strcmp(myarg_1, "bHomed?")) {
    cmd_buf_printf("%d", getAxisHomed(motor_axis_no) ? 1 : 0);
    return;
  }
  /* bReset? */
  if (!strcmp(myarg_1, "bReset?")) {
    cmd_buf_printf("%d",cmd_Motor_cmd[motor_axis_no].bReset);
    return;
  }
  /* fAcceleration? */
  if (0 == strcmp(myarg_1, "fAcceleration?")) {
    cmd_buf_printf("%g", cmd_Motor_cmd[motor_axis_no].fAcceleration);
    return;
  }
  /* fActPosition? */
  if (0 == strcmp(myarg_1, "fActPosition?")) {
    cmd_buf_printf("%g", getMotorPos(motor_axis_no));
    return;
  }
  /* fActVelocity? */
  if (0 == strcmp(myarg_1, "fActVelocity?")) {
    cmd_buf_printf("%g", getMotorVelocity(motor_axis_no));
    return;
  }
  /* fPosition? */
  if (0 == strcmp(myarg_1, "fPosition?")) {
    /* The "set" value */
    cmd_buf_printf("%g", cmd_Motor_cmd[motor_axis_no].fPosition);
    return;
  }
  /* nCommand? */
  if (0 == strcmp(myarg_1, "nCommand?")) {
    cmd_buf_printf("%d", cmd_Motor_cmd[motor_axis_no].command_no);
    return;
  }
  /* nMotionAxisID? */
  if (0 == strcmp(myarg_1, "nMotionAxisID?")) {
    /* The NC axis id is the same as motion axis id */
    printf("%s/%s:%d %s(%d)\n",  __FILE__, __FUNCTION__, __LINE__,
           myarg_1, motor_axis_no);
    cmd_buf_printf("%d", motor_axis_no);
    return;
  }
  /* stAxisStatus? */
  if (0 == strcmp(myarg_1, "stAxisStatus?")) {
    int bJogFwd = 0;
    int bJogBwd = 0;
    double fOverride = 0;
    /* getMotorPos must be first, it calls simulateMotion() */
    cmd_Motor_status[motor_axis_no].fActPostion = getMotorPos(motor_axis_no);
    cmd_Motor_status[motor_axis_no].bEnable = getAmplifierOn(motor_axis_no);
    cmd_Motor_status[motor_axis_no].bEnabled = getAmplifierOn(motor_axis_no);
    cmd_Motor_status[motor_axis_no].bLimitFwd = getPosLimitSwitch(motor_axis_no) ? 0 : 1;
    cmd_Motor_status[motor_axis_no].bLimitBwd = getNegLimitSwitch(motor_axis_no) ? 0 : 1;
    cmd_Motor_status[motor_axis_no].bHomeSensor = getAxisHome(motor_axis_no);
    cmd_Motor_status[motor_axis_no].bError = get_bError(motor_axis_no);
    cmd_Motor_status[motor_axis_no].nErrorId = get_nErrorId(motor_axis_no);
    cmd_Motor_status[motor_axis_no].fActVelocity = getMotorVelocity(motor_axis_no);
    cmd_Motor_status[motor_axis_no].bHomed = getAxisHomed(motor_axis_no);
    cmd_Motor_status[motor_axis_no].bBusy = isMotorMoving(motor_axis_no);

    cmd_buf_printf("Main.M%d.stAxisStatus="
                   "%d,%d,%d,%u,%u,%g,%g,%g,%g,%d,"
                   "%d,%d,%d,%g,%d,%d,%d,%u,%g,%g,%g,%d,%d",
                   motor_axis_no,
                   cmd_Motor_status[motor_axis_no].bEnable,        /*  1 */
                   cmd_Motor_status[motor_axis_no].bReset,         /*  2 */
                   cmd_Motor_cmd[motor_axis_no].bExecute,          /*  3 */
                   cmd_Motor_status[motor_axis_no].nCommand,       /*  4 */
                   cmd_Motor_cmd[motor_axis_no].nCmdData,          /*  5 */
                   cmd_Motor_status[motor_axis_no].fVelocity,      /*  6 */
                   cmd_Motor_status[motor_axis_no].fPosition,      /*  7 */
                   cmd_Motor_cmd[motor_axis_no].fAcceleration,     /*  8 */
                   cmd_Motor_cmd[motor_axis_no].fDeceleration,     /*  9 */
                   bJogFwd,        /* 10 */
                   bJogBwd,        /* 11 */
                   cmd_Motor_status[motor_axis_no].bLimitFwd,      /* 12 */
                   cmd_Motor_status[motor_axis_no].bLimitBwd,      /* 13 */
                   fOverride,      /* 14 */
                   cmd_Motor_status[motor_axis_no].bHomeSensor,    /* 15 */
                   cmd_Motor_status[motor_axis_no].bEnabled,       /* 16 */
                   cmd_Motor_status[motor_axis_no].bError,         /* 17 */
                   cmd_Motor_status[motor_axis_no].nErrorId,       /* 18 */
                   cmd_Motor_status[motor_axis_no].fActVelocity,   /* 19 */
                   cmd_Motor_status[motor_axis_no].fActPostion,    /* 20 */
                   cmd_Motor_status[motor_axis_no].fActDiff,       /* 21 */
                   cmd_Motor_status[motor_axis_no].bHomed,         /* 22 */
                   cmd_Motor_status[motor_axis_no].bBusy           /* 23 */
                   );
    return;
  }
  /* sErrorMessage?  */
  if (!strcmp(myarg_1, "sErrorMessage?")) {
    char buf[32]; /* 9 should be OK */
    int nErrorId = get_nErrorId(motor_axis_no);
    snprintf(buf, sizeof(buf), "%x", nErrorId);
    cmd_buf_printf("%s", buf);
    return;
  }

  /* End of "get" commands, from here, set commands */

  /* nCommand=3 */
  nvals = sscanf(myarg_1, "nCommand=%d", &iValue);
  if (nvals == 1) {
    cmd_Motor_cmd[motor_axis_no].command_no = iValue;
    cmd_buf_printf("OK");
    return;
  }
  /* nCmdData=1 */
  nvals = sscanf(myarg_1, "nCmdData=%d", &iValue);
  if (nvals == 1) {
    cmd_Motor_cmd[motor_axis_no].nCmdData = iValue;
    cmd_buf_printf("OK");
    return;
  }
  /* fPosition=100 */
  nvals = sscanf(myarg_1, "fPosition=%lf", &fValue);
  if (nvals == 1) {
    cmd_Motor_cmd[motor_axis_no].fPosition = fValue;
    cmd_buf_printf("OK");
    return;
  }
  /* fHomePosition */
  nvals = sscanf(myarg_1, "fHomePosition=%lf", &fValue);
  if (nvals == 1) {
    /* Do noting */
    cmd_buf_printf("OK");
    return;
  }


  /* fVelocity=20 */
  nvals = sscanf(myarg_1, "fVelocity=%lf", &fValue);
  if (nvals == 1) {
    cmd_Motor_cmd[motor_axis_no].fVelocity = fValue;
    cmd_buf_printf("OK");
    return;
  }
  /* fAcceleration=1000 */
  nvals = sscanf(myarg_1, "fAcceleration=%lf", &fValue);
  if (nvals == 1) {
    cmd_Motor_cmd[motor_axis_no].fAcceleration = fValue;
    cmd_buf_printf("OK");
    return;
  }
  /* fDeceleration=1000 */
  nvals = sscanf(myarg_1, "fDeceleration=%lf", &fValue);
  if (nvals == 1) {
    cmd_Motor_cmd[motor_axis_no].fDeceleration = fValue;
    cmd_buf_printf("OK");
    return;
  }
  /* bEnable= */
  nvals = sscanf(myarg_1, "bEnable=%d", &iValue);
  if (nvals == 1) {
    int amplifierLockedToBeOff = getAmplifierLockedToBeOff(motor_axis_no);
    if (amplifierLockedToBeOff) {
      setAmplifierPercent(motor_axis_no, 0);
      if (amplifierLockedToBeOff == AMPLIFIER_LOCKED_TO_BE_OFF_LOUD) {
        cmd_buf_printf("Amplifier locked");
        return;
      }
      iValue = 0;
    }
    setAmplifierPercent(motor_axis_no, iValue ? 100 : 0);
    cmd_buf_printf("OK");
    return;
  }
  /* bExecute= */
  nvals = sscanf(myarg_1, "bExecute=%d", &iValue);
  if (nvals == 1) {
    cmd_Motor_cmd[motor_axis_no].bExecute = iValue;
    if (!iValue) {
      /* bExecute=0 is always allowed, regardless the command */
      motorStop(motor_axis_no);
      cmd_buf_printf("OK");
      return;
    } else if (iValue == 1) {
      if (cmd_Motor_cmd[motor_axis_no].fVelocity >
          cmd_Motor_cmd[motor_axis_no].maximumVelocity) {
        fprintf(stdlog, "%s/%s:%d axis_no=%d velocity=%g maximumVelocity=%g\n",
                __FILE__, __FUNCTION__, __LINE__,
                motor_axis_no,
                cmd_Motor_cmd[motor_axis_no].fVelocity,
                cmd_Motor_cmd[motor_axis_no].maximumVelocity);
        set_nErrorId(motor_axis_no, 0x512);
        cmd_buf_printf("OK");
        return;
      }
      if (isMotorMoving(motor_axis_no)) {
        int nErrorId = 0x1431C;
        cmd_buf_printf("Error: %d", nErrorId);
        set_nErrorId(motor_axis_no, nErrorId);
        return;
      }
      switch (cmd_Motor_cmd[motor_axis_no].command_no) {
        case 1:
        {
          int direction = 1;
          if (cmd_Motor_cmd[motor_axis_no].fVelocity < 0) {
            direction = 0;
            cmd_Motor_cmd[motor_axis_no].fVelocity = -cmd_Motor_cmd[motor_axis_no].fVelocity;
          }
          (void)moveVelocity(motor_axis_no,
                             direction,
                             cmd_Motor_cmd[motor_axis_no].fVelocity,
                             cmd_Motor_cmd[motor_axis_no].fAcceleration);
          cmd_buf_printf("OK");
        }
        break;
        case 2:
          (void)movePosition(motor_axis_no,
                             cmd_Motor_cmd[motor_axis_no].fPosition,
                             1, /* int relative, */
                             cmd_Motor_cmd[motor_axis_no].fVelocity,
                             cmd_Motor_cmd[motor_axis_no].fAcceleration);
          cmd_buf_printf("OK");
          break;
        case 3:
          (void)movePosition(motor_axis_no,
                             cmd_Motor_cmd[motor_axis_no].fPosition,
                             0, /* int relative, */
                             cmd_Motor_cmd[motor_axis_no].fVelocity,
                             cmd_Motor_cmd[motor_axis_no].fAcceleration);
          cmd_buf_printf("OK");
          break;
        case 10:
        {
          if (cmd_Motor_cmd[motor_axis_no].homeVeloTowardsHomeSensor &&
              cmd_Motor_cmd[motor_axis_no].homeVeloFromHomeSensor) {
            (void)moveHomeProc(motor_axis_no,
                               0, /* direction, */
                               cmd_Motor_cmd[motor_axis_no].nCmdData,
                               cmd_Motor_cmd[motor_axis_no].homeVeloTowardsHomeSensor,
                               cmd_Motor_cmd[motor_axis_no].fAcceleration);
            cmd_buf_printf("OK");
          } else {
            cmd_buf_printf("Error : %d %g %g",
                           70000,
                           cmd_Motor_cmd[motor_axis_no].homeVeloTowardsHomeSensor,
                           cmd_Motor_cmd[motor_axis_no].homeVeloFromHomeSensor);
          }
        }
        break;
        default:
          RETURN_OR_DIE("%s/%s:%d line=%s command_no=%u",
                        __FILE__, __FUNCTION__, __LINE__,
                        myarg, cmd_Motor_cmd[motor_axis_no].command_no);
      }
      return;
    }
    RETURN_OR_DIE("%s/%s:%d line=%s invalid_iValue=%u '.'",
                  __FILE__, __FUNCTION__, __LINE__,
                  myarg,  iValue);
  }
  /* bReset= */
  nvals = sscanf(myarg_1, "bReset=%d", &iValue);
  if (nvals == 1) {
    cmd_Motor_cmd[motor_axis_no].bReset = iValue;
    if (iValue) {
      motorStop(motor_axis_no);
      set_nErrorId(motor_axis_no, 0);
    }
    cmd_buf_printf("OK");
    return;
  }
  /* if we come here, we do not understand the command */
  RETURN_OR_DIE("%s/%s:%d line=%s",
                __FILE__, __FUNCTION__, __LINE__,
                myarg);
}

void cmd_EAT(int argc, const char *argv[])
{
  const char *myargline = (argc > 0) ? argv[0] : "";
  if (PRINT_STDOUT_BIT6())
  {
    const char *myarg[5];
    myarg[0] = myargline;
    myarg[1] = (argc > 1) ? argv[1] : "";
    myarg[2] = (argc > 2) ? argv[2] : "";
    myarg[3] = (argc > 3) ? argv[3] : "";
    myarg[4] = (argc > 4) ? argv[4] : "";
    LOGINFO6("%s/%s:%d argc=%d "
             "myargline=\"%s\" myarg[1]=\"%s\" myarg[2]=\"%s\" myarg[3]=\"%s\" myarg[4]=\"%s\"\n",
             __FILE__, __FUNCTION__, __LINE__,
             argc,  myargline,
             myarg[1], myarg[2],
             myarg[3], myarg[4]);
  }

  while (argc > 1) {
    motorHandleOneArg(argv[1]);
    cmd_buf_printf("%s", seperator_seperator);
    argc--;
    argv++;
  } /* while argc > 0 */
  cmd_buf_printf("%s", "\n");
}
/******************************************************************************/
