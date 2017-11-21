#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>
#include "sock-util.h"
#include "logerr_info.h"
#include "cmd_buf.h"
#include "hw_motor.h"
#include "cmd_IcePAP.h"

#define ICEPAP_SEND_NEWLINE  1
#define ICEPAP_SEND_OK 2
#define ICEPAP_CMD_NOT_IMPLEMENTED 3

typedef struct
{
  double homevel;
  int    velocity;
} cmd_Motor_cmd_type;

static cmd_Motor_cmd_type cmd_Motor_cmd[MAX_AXES];


static void init_axis(int axis_no)
{
  static char init_done[MAX_AXES];
  const double MRES = 0.03;
  const double ERES = MRES;
  double ReverseMRES = (double)1.0/MRES;

  if (axis_no >= MAX_AXES || axis_no < 0) {
    return;
  }
  if (!init_done[axis_no]) {
    double valueLow = -1.0;
    double valueHigh = 180.0;

    hw_motor_init(axis_no,
                  MRES/ERES,               /* ReverseERES */
                  (1 + axis_no/10.0),      /* ParkingPOS, steps */
                  66 * ReverseMRES,        /* maxHomeVelocityAbs */
                  valueLow * ReverseMRES,  /* lowHardLimitPos */
                  valueHigh * ReverseMRES, /* highHardLimitPos */
                  valueLow * ReverseMRES,  /* hWlowPos */
                  valueHigh * ReverseMRES, /* hWhighPos */
                  0);                      /* homeSwitchPos */
    setAmplifierPercent(axis_no,100);
  init_done[axis_no] = 1;
  }
}


static int handle_IcePAP_cmd(const char *myarg_1)
{
  int motor_axis_no = 0;
  int nvals = 0;
  int ret = ICEPAP_SEND_NEWLINE;
  if (myarg_1 && *myarg_1 == '#') {
    ret = ICEPAP_SEND_OK; /* Jump over '#' */
    myarg_1++;
  }
  nvals = sscanf(myarg_1, "%d:", &motor_axis_no);
  if (nvals != 1) {
    return 0; /* Not IcePAP */
  }
  AXIS_CHECK_RETURN_ZERO(motor_axis_no);
  myarg_1 = strchr(myarg_1, ':');
  if (!myarg_1) {
    RETURN_ERROR_OR_DIE(0, "%s/%s:%d line=%s missing ':'",
                        __FILE__, __FUNCTION__, __LINE__,
                        myarg_1);
  }
  myarg_1++; /* Jump over ':' */
  if (0 == strcmp(myarg_1, "STOP")) {
    motorStop(motor_axis_no);
    return ret;
  }
  if (0 == strcmp(myarg_1, "?STATUS")) {
    /* Page 21 */
    #define STATUS_BIT_READY     (1<<9)
    #define STATUS_BIT_MOVING    (1<<10)
    #define STATUS_BIT_LIMIT_POS (1<<18)
    #define STATUS_BIT_LIMIT_NEG (1<<19)
    #define STATUS_BIT_HSIGNAL   (1<<20)
    #define STATUS_BIT_POWERON   (1<<23)
    int status = 0;

    if (isMotorMoving(motor_axis_no))     status |= STATUS_BIT_MOVING;
    if (getAxisHome(motor_axis_no))       status |= STATUS_BIT_HSIGNAL;
    if (getPosLimitSwitch(motor_axis_no)) status |= STATUS_BIT_LIMIT_POS;
    if (getNegLimitSwitch(motor_axis_no)) status |= STATUS_BIT_LIMIT_NEG;
    if (1)                                status |= STATUS_BIT_POWERON; /* Power always on for now */
    if (!(status & STATUS_BIT_MOVING))    status |= STATUS_BIT_READY;
    cmd_buf_printf("%d:?STATUS %x", motor_axis_no, status);
    return ICEPAP_SEND_NEWLINE;
  }
  if (0 == strcmp(myarg_1, "?POS")) {
    int value = (int)getMotorPos(motor_axis_no);
    cmd_buf_printf("%d:%s %d", motor_axis_no, myarg_1, value);
    return ICEPAP_SEND_NEWLINE;
  }
  if (0 == strcmp(myarg_1, "?HOMESTAT")) {
    const char *status_txt = "";
    if (getAxisHomed(motor_axis_no)) {
      status_txt = "FOUND -1";
    } else {
      status_txt = "NOTFOUND";
    }
    cmd_buf_printf("%d:%s %s", motor_axis_no, myarg_1, status_txt);
    return ICEPAP_SEND_NEWLINE;
  }
  /* 1:?POWER */
  if (0 == strcmp(myarg_1, "?POWER")) {
    /* Power always on for now */
    cmd_buf_printf("%d:%s %s", motor_axis_no, myarg_1 ,"ON");
    return ICEPAP_SEND_NEWLINE;
  }
  return 0;
}


static int handle_IcePAP_cmd3(const char *myarg_1, const char *myarg_2)
{
  int motor_axis_no = 0;
  int value = 0;
  int nvals = 0;
  int ret = ICEPAP_SEND_NEWLINE;
  if (myarg_1 && *myarg_1 == '#') {
    ret = ICEPAP_SEND_OK; /* Jump over '#' */
    myarg_1++;
  }
  (void)ret;
  /* ?FPOS 1 */
  if (0 == strcmp(myarg_1, "?FPOS")) {
    nvals = sscanf(myarg_2, "%d", &motor_axis_no);
    LOGINFO4("%s/%s:%d nvals=%d motor_axis_no=%d\n",
             __FILE__, __FUNCTION__, __LINE__,
             nvals, motor_axis_no);
    if (nvals == 1) {
      cmd_buf_printf("%s %d", myarg_1, (int)getMotorPos(motor_axis_no));
      return ICEPAP_SEND_NEWLINE;
    }
  }

  nvals = sscanf(myarg_1, "%d:", &motor_axis_no);
  if (nvals != 1) {
    return 0; /* Not IcePAP */
  }
  AXIS_CHECK_RETURN_ZERO(motor_axis_no);
  myarg_1 = strchr(myarg_1, ':');
  if (!myarg_1) {
    RETURN_ERROR_OR_DIE(0, "%s/%s:%d line=%s missing ':'",
                        __FILE__, __FUNCTION__, __LINE__,
                        myarg_1);
  }
  myarg_1++; /* Jump over ':' */
  if (0 == strcmp(myarg_1, "JOG")) {
    nvals = sscanf(myarg_2, "%d", &value);
    LOGINFO4("%s/%s:%d nvals=%d\n",
             __FILE__, __FUNCTION__, __LINE__,
             nvals);
    if (nvals == 1) {
      double velocity = (double)value;
      if (value < 0) velocity = 0 - velocity;
      moveVelocity(motor_axis_no,
                   value > 0, /* int direction */
                   velocity, /* double max_velocity, */
                   1.0 /*double acceleration */ );
      return ret;
    }
  }
  if (0 == strcmp(myarg_1, "HOME")) {
    nvals = sscanf(myarg_2, "%d", &value);
    LOGINFO4("%s/%s:%d nvals=%d\n",
             __FILE__, __FUNCTION__, __LINE__,
             nvals);
    if (nvals == 1) {
      moveHome(motor_axis_no,
               value, /* int direction */
               cmd_Motor_cmd[motor_axis_no].homevel,
               1.0 /*double acceleration */ );
      return ret;
    }
  }
  if (0 == strcmp(myarg_1, "MOVE")) {
    nvals = sscanf(myarg_2, "%d", &value);
    LOGINFO5("%s/%s:%d res=%d\n",
             __FILE__, __FUNCTION__, __LINE__,
             nvals);
    if (nvals == 1) {
      movePosition(motor_axis_no,
                   (double)value,
                   0, /* int relative, */
                   cmd_Motor_cmd[motor_axis_no].velocity,
                   1.0 /*double acceleration */ );
      return ret; /* MOVE does respond */
    }
  }
  if (0 == strcmp(myarg_1, "VELOCITY")) {
    nvals = sscanf(myarg_2, "%d", &value);
    LOGINFO5("%s/%s:%d res=%d\n",
             __FILE__, __FUNCTION__, __LINE__,
             nvals);
    if (nvals == 1) {
      cmd_Motor_cmd[motor_axis_no].velocity = value;
      return ret;
    }
  }
  /* #1:POWER ON */
  if (0 == strcmp(myarg_1, "POWER")) {
    if (0 == strcmp(myarg_2, "ON")) {
      /* Power always on for now */
      return ICEPAP_SEND_OK;
    }
    if (0 == strcmp(myarg_2, "OFF")) {
      /* Power always on for now */
      return ICEPAP_SEND_OK;
    }
    return ICEPAP_CMD_NOT_IMPLEMENTED;
  }

#if NOTTESTED
  if (0 == strcmp(myarg_1, "RMOVE")) {
    nvals = sscanf(myarg_2, "%d", &value);
    LOGINFO5("%s/%s:%d res=%d\n",
             __FILE__, __FUNCTION__, __LINE__,
             nvals);
    if (nvals == 1) {
      movePosition(motor_axis_no,
                   (double)value,
                   1, /* int relative, */
                   1000.0, /* double max_velocity, */
                   1.0 /*double acceleration */ );
      return ICEPAP_SEND_NEWLINE; /* MOVE doesn't respond */
    }
  }
#endif
  return 0;
}

static int handle_IcePAP_cmd4(const char *myarg_1,
                              const char *myarg_2,
                              const char *myarg_3)
{
  int motor_axis_no = 0;
  int value = 0;
  int nvals = 0;
  int ret = ICEPAP_SEND_NEWLINE;
  if (myarg_1 && *myarg_1 == '#') {
    ret = ICEPAP_SEND_OK; /* Jump over '#' */
    myarg_1++;
  }
  (void)ret;
  (void)value;
  /* Commands with '?' must be first, they don't have a board number
     (or axis number) and don't have a ':' */
  if (0 == strcmp(myarg_1, "?FPOS")) {
    if (0 == strcmp(myarg_2, "MEASURE")) {
      nvals = sscanf(myarg_3, "%d", &motor_axis_no);
      LOGINFO4("%s/%s:%d nvals=%d motor_axis_no=%d\n",
               __FILE__, __FUNCTION__, __LINE__,
               nvals, motor_axis_no);
      if (nvals == 1) {
        int value = (int)getEncoderPos(motor_axis_no);
        cmd_buf_printf("%s %d", myarg_1, value);
        LOGINFO3("%s/%s:%d encoderpos=%d\n",
                 __FILE__, __FUNCTION__, __LINE__, value);
        return ICEPAP_SEND_NEWLINE;
      }
    }
  }
  nvals = sscanf(myarg_1, "%d:", &motor_axis_no);
  if (nvals != 1) {
    return 0; /* Not IcePAP */
  }
  AXIS_CHECK_RETURN_ZERO(motor_axis_no);
  myarg_1 = strchr(myarg_1, ':');
  if (!myarg_1) {
    RETURN_ERROR_OR_DIE(0, "%s/%s:%d line=%s missing ':'",
                        __FILE__, __FUNCTION__, __LINE__,
                        myarg_1);
  }
  myarg_1++; /* Jump over ':' */

  if (0 == strcmp(myarg_1, "CFG")) {
    if (0 == strcmp(myarg_2, "HOMEVEL")) {
      double fValue = 0;
      nvals = sscanf(myarg_3, "%lf", &fValue);
      LOGINFO4("%s/%s:%d nvals=%d fValue=%f\n",
               __FILE__, __FUNCTION__, __LINE__,
               nvals, fValue);
      if (nvals == 1) {
        cmd_Motor_cmd[motor_axis_no].homevel = fValue;
        return ret;
      }
    }
  }
  /* #1: POWER ON */
  if (0 == strcmp(myarg_1, "")) {
    if (0 == strcmp(myarg_2, "POWER")) {
      if (0 == strcmp(myarg_3, "ON")) {
        /* Power always on for now */
        cmd_buf_printf("POWER OK");
        return ICEPAP_SEND_NEWLINE;
      }
      if (0 == strcmp(myarg_3, "OFF")) {
        /* Power always on for now */
        cmd_buf_printf("POWER OK");
        return ICEPAP_SEND_NEWLINE;
      }
      return ICEPAP_CMD_NOT_IMPLEMENTED;
    }
    LOGERR("%s/%s:%d myarg_1=%s myarg_2=%s\n",
           __FILE__, __FUNCTION__, __LINE__,
           myarg_1, myarg_2);
  }
  return 0;
}

int cmd_IcePAP(int argc, const char *argv[])
{
  int ret = 0;
  LOGINFO5("%s/%s:%d argc=%d argv[0]=%s\n",
           __FILE__, __FUNCTION__, __LINE__,
           argc, argv[0]);
  /* We use a UNIX like counting:
     argc == 3
     argv[0]  "1:MOVE 2011" (non-UNIX: the whole command line)
     argv[1]  "1:MOVE"
     argv[2]  "2011"
  */
  if (argc == 4) {
    LOGINFO5("%s/%s:%d argv[1]=%s argv[2]=%s\n",
             __FILE__, __FUNCTION__, __LINE__,
             argv[1], argv[2]);
    ret = handle_IcePAP_cmd4(argv[1], argv[2], argv[3]);
  } else if (argc == 3) {
    LOGINFO5("%s/%s:%d argv[1]=%s argv[2]=%s\n",
             __FILE__, __FUNCTION__, __LINE__,
             argv[1], argv[2]);
    ret = handle_IcePAP_cmd3(argv[1], argv[2]);
  } else if  (argc == 2) {
    LOGINFO5("%s/%s:%d argv[1]=%s\n",
             __FILE__, __FUNCTION__, __LINE__,
             argv[1]);
    ret = handle_IcePAP_cmd(argv[1]);
  }
  switch (ret) {
    case ICEPAP_SEND_OK:
      cmd_buf_printf("%s OK\n", &argv[1][1]); /* Don't echo '#' */
      break;
    case ICEPAP_SEND_NEWLINE:
      cmd_buf_printf("\n");
      break;
    case ICEPAP_CMD_NOT_IMPLEMENTED:
      LOGERR("%s/%s:%d argv[0]=%s\n",
             __FILE__, __FUNCTION__, __LINE__,
             argv[0]);
      /* We should have handled */
      break;
    default:
      LOGERR("%s/%s:%d argv[0]=%s\n",
             __FILE__, __FUNCTION__, __LINE__,
             argv[0]);
      ;
  }
  return ret;
}
/******************************************************************************/
/* TODO:
#1:CFG HOMEVEL 200.87
1:CFG OK
*/
