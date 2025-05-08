#include "hw_motor.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "logerr_info.h"
#include "sock-util.h" /* stdlog */

#define NINT(f) (long)((f) > 0 ? (f) + 0.5 : (f)-0.5) /* Nearest integer. */

#define MOTOR_POS_HOME 0
#define MOTOR_REV_ERES (-57)
#define MOTOR_PARK_POS (-64)

/* Homing procdures LS=Limit switch, HS=Home switch */
#define HomProc_LOW_LS 1
#define HomProc_HIGH_LS 2
#define HomProc_LOW_HS 3
#define HomProc_HIGH_HS 4

#define MOTOR_VEL_HOME_MAX 5.0

#define RAMPDOWNONLIMIT 3

typedef struct {
  struct timeval lastPollTime;
  struct timeval lastJitterTime;
  struct timeval powerOnTime;

  int amplifierPercent;
  /* What the (simulated) hardware has physically.
     When homing against the high limit switch is done,
     all logical values will be re-calculated.
  */
  double HWlowPos;
  double HWhighPos;
  double HWhomeSwitchpos;
  /*
     What the (simulated) hardware has logically.
  */
  double HomeSwitchPos; /* home switch */
  double HomePos;       /* Position after homing */
  int HomeProc;
  double highHardLimitPos;
  double lowHardLimitPos;

  /* What EPICS sends us */
  double MRES_23;
  double MRES_24;
  double highSoftLimitPos;
  double lowSoftLimitPos;

  int definedLowHardLimitPos;
  int definedHighHardLimitPos;
  int enabledLowSoftLimitPos;
  int enabledHighSoftLimitPos;
  double MotorPosSetPosOffset;
  double MotorPosNow;
  double MotorPosReported;
  double positionJitter;
  int positionJitterPlus;
  double MotorPosWanted;
  double HomeVelocityAbsWanted;
  double MaxHomeVelocityAbs;
  double MaxVelocity;
  double MaxAcceleration;
  struct {
    struct {
      double HomeVelocity;
      double PosVelocity;
      double JogVelocity;
      int stillMoving;
    } velo;
    int hitPosLimitSwitch;
    int hitNegLimitSwitch;
    unsigned int rampDownOnLimit;
    unsigned int rampUpAfterStart;
    int clipped;
  } moving;
  double EncoderPos;
  double ParkingPos;
  double ReverseERES;
  int homed;
  int bError;
  int nErrorId;
  unsigned nStatReasAUX;
  FILE *logFile;
  int bManualSimulatorMode;
  int bLocalMode;
  int bInterlockBwd;
  int bInterlockFwd;
  int amplifierLockedToBeOff;
  int defRampUpAfterStart;
  double nxtMoveAcceleration;
  double nxtMoveVelocity;
} motor_axis_type;

static unsigned cabinetStatus;
static char init_done[MAX_AXES];
static motor_axis_type motor_axis[MAX_AXES];
static motor_axis_type motor_axis_last[MAX_AXES];
static motor_axis_type motor_axis_reported[MAX_AXES];
static double calcMotorPosReported(int axis_no, double jitter);

static double calcMotorPosHw(int axis_no) {
  return motor_axis[axis_no].MotorPosNow +
         motor_axis[axis_no].MotorPosSetPosOffset;
}

static double getEncoderPosFromMotorPos(int axis_no, double MotorPosNow) {
  (void)axis_no;
  return ((calcMotorPosHw(axis_no) - motor_axis[axis_no].ParkingPos)) *
         motor_axis[axis_no].ReverseERES;
}

#if 0
static double getMotorPosFromEncoderPos(int axis_no, double EncoderPos)
{
  (void)axis_no;
  return (double)(int)((EncoderPos / motor_axis[axis_no].ReverseERES) + motor_axis[axis_no].ParkingPos);
}
#endif

void hw_motor_init_fl(const char *file, int line_no, int axis_no,
                      const struct motor_init_values *pMotor_init_values,
                      size_t motor_init_len) {
  if (axis_no >= MAX_AXES || axis_no < 0) {
    return;
  }
  if (init_done[axis_no]) {
    return;
  }

  if (motor_init_len != sizeof(struct motor_init_values)) {
    fprintf(stderr, "%s/%s:%d axis_no=%d motor_init_len=%u sizeof=%u\n",
            __FILE__, __FUNCTION__, __LINE__, axis_no, (unsigned)motor_init_len,
            (unsigned)sizeof(struct motor_init_values));
    return;
  } else {
    double ReverseERES = pMotor_init_values->ReverseERES;
    double ParkingPos = pMotor_init_values->ParkingPos;
    double MaxHomeVelocityAbs = pMotor_init_values->MaxHomeVelocityAbs;
    double lowHardLimitPos = pMotor_init_values->lowHardLimitPos;
    double highHardLimitPos = pMotor_init_values->highHardLimitPos;
    double hWlowPos = pMotor_init_values->hWlowPos;
    double hWhighPos = pMotor_init_values->hWhighPos;
    double homeSwitchPos = pMotor_init_values->homeSwitchPos;
    int defRampUpAfterStart = pMotor_init_values->defRampUpAfterStart;

    fprintf(
        stdlog,
        "%s:%d %s axis_no=%d ReverseERES=%f ParkingPos=%f MaxHomeVelocityAbs=%f"
        "\n  lowHardLimitPos=%f highHardLimitPos=%f hWlowPos=%f hWhighPos=%f "
        "homeSwitchPos=%f\n",
        file, line_no, __FUNCTION__, axis_no, ReverseERES, ParkingPos,
        MaxHomeVelocityAbs, lowHardLimitPos, highHardLimitPos, hWlowPos,
        hWhighPos, homeSwitchPos);

    memset(&motor_axis[axis_no], 0, sizeof(motor_axis[axis_no]));
    memset(&motor_axis_last[axis_no], 0, sizeof(motor_axis_last[axis_no]));
    memset(&motor_axis_reported[axis_no], 0,
           sizeof(motor_axis_reported[axis_no]));

    motor_axis[axis_no].ReverseERES = ReverseERES;
    motor_axis[axis_no].ParkingPos = ParkingPos;
    motor_axis[axis_no].MotorPosNow = ParkingPos;
    motor_axis[axis_no].MaxHomeVelocityAbs = MaxHomeVelocityAbs;

    motor_axis[axis_no].MaxVelocity = 100;    /* May be overwritten later */
    motor_axis[axis_no].MaxAcceleration = 95; /* May be overwritten later */
    motor_axis[axis_no].lowHardLimitPos = lowHardLimitPos;
    motor_axis[axis_no].definedLowHardLimitPos = 1;
    motor_axis[axis_no].highHardLimitPos = highHardLimitPos;
    motor_axis[axis_no].definedHighHardLimitPos = 1;

    motor_axis[axis_no].HWlowPos = hWlowPos;
    motor_axis[axis_no].HWhighPos = hWhighPos;

    motor_axis[axis_no].HomeSwitchPos = homeSwitchPos;
    motor_axis[axis_no].defRampUpAfterStart = defRampUpAfterStart;
    // motor_axis[axis_no].amplifierPercent = 100;
    //  setMotorParkingPosition(axis_no, MOTOR_PARK_POS);
    //  motor_axis[axis_no].ReverseERES = MOTOR_REV_ERES;
    motor_axis[axis_no].EncoderPos =
        getEncoderPosFromMotorPos(axis_no, motor_axis[axis_no].MotorPosNow);
    motor_axis_last[axis_no].EncoderPos = motor_axis[axis_no].EncoderPos;
    motor_axis_last[axis_no].MotorPosNow = motor_axis[axis_no].MotorPosNow;
    /* Do not re-initalize again */
    init_done[axis_no] = 1;
  }
}

static void init_axis(int axis_no) {
  struct motor_init_values motor_init_values;
  const double MRES = 1;
  const double UREV = 60.0;   /* mm/revolution */
  const double SREV = 2000.0; /* ticks/revolution */
  const double ERES = UREV / SREV;
  double ReverseMRES = (double)1.0 / MRES;
  double valueLow = -1.0 * ReverseMRES;
  double valueHigh = 186.0 * ReverseMRES;

  memset(&motor_init_values, 0, sizeof(motor_init_values));
  motor_init_values.ReverseERES = MRES / ERES;
  motor_init_values.ParkingPos = (100 + axis_no / 10.0);
  motor_init_values.MaxHomeVelocityAbs = 5 * ReverseMRES;
  motor_init_values.lowHardLimitPos = valueLow;
  motor_init_values.highHardLimitPos = valueHigh;
  motor_init_values.hWlowPos = valueLow;
  motor_init_values.hWhighPos = valueHigh;

  if (!init_done[axis_no]) {
    hw_motor_init(axis_no, &motor_init_values, sizeof(motor_init_values));
    init_done[axis_no] = 1;
  }
}

unsigned getCabinetStatus(void) { return cabinetStatus; }
void setCabinetStatus(unsigned value) {
  LOGTIME3("%s old=0x%08x value=0x%08x\n", __FUNCTION__, cabinetStatus, value);
  cabinetStatus = value;
}

double getMaxVelocity(int axis_no) {
  double velocity;
  AXIS_CHECK_RETURN_ZERO(axis_no);
  if (motor_axis[axis_no].moving.rampUpAfterStart) {
    return 0;
  }
  velocity = motor_axis[axis_no].MaxVelocity;
  return velocity;
}

void setMaxVelocity(int axis_no, double value) {
  LOGTIME3("%s(%d) value=%g\n", __FUNCTION__, axis_no, value);
  if (((axis_no) <= 0) || ((axis_no) >= MAX_AXES)) {
    return;
  }
  motor_axis[axis_no].MaxVelocity = value;
}

double getMaxAcceleration(int axis_no) {
  double velocity;
  AXIS_CHECK_RETURN_ZERO(axis_no);
  if (motor_axis[axis_no].moving.rampUpAfterStart) {
    return 0;
  }
  velocity = motor_axis[axis_no].MaxAcceleration;
  return velocity;
}

void setMaxAcceleration(int axis_no, double value) {
  LOGTIME3("%s(%d) value=%g\n", __FUNCTION__, axis_no, value);
  if (((axis_no) <= 0) || ((axis_no) >= MAX_AXES)) {
    return;
  }
  motor_axis[axis_no].MaxAcceleration = value;
}

double getNxtMoveAcceleration(int axis_no) {
  double value = 0.0;
  if (((axis_no) >= 0) && ((axis_no) < MAX_AXES)) {
    value = motor_axis[axis_no].nxtMoveAcceleration;
  }
#if 0
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%f\n",
          __FUNCTION__, axis_no, value);
#endif
  return value;
}

void setNxtMoveAcceleration(int axis_no, double value) {
  LOGTIME3("%s(%d) value=%g\n", __FUNCTION__, axis_no, value);
  if (((axis_no) <= 0) || ((axis_no) >= MAX_AXES)) {
    return;
  }
  motor_axis[axis_no].nxtMoveAcceleration = value;
}

double getNxtMoveVelocity(int axis_no) {
  double value = 0.0;
  if (((axis_no) >= 0) && ((axis_no) < MAX_AXES)) {
    value = motor_axis[axis_no].nxtMoveVelocity;
  }
#if 0
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%f\n",
          __FUNCTION__, axis_no, value);
#endif
  return value;
}

void setNxtMoveVelocity(int axis_no, double value) {
  LOGTIME3("%s(%d) value=%g\n", __FUNCTION__, axis_no, value);
  if (((axis_no) <= 0) || ((axis_no) >= MAX_AXES)) {
    return;
  }
  motor_axis[axis_no].nxtMoveVelocity = value;
}

void setMotorParkingPosition(int axis_no, double value) {
  LOGTIME3("%s(%d) value=%g\n", __FUNCTION__, axis_no, value);
  if (((axis_no) <= 0) || ((axis_no) >= MAX_AXES)) {
    return;
  }
  if (motor_axis[axis_no].ParkingPos != value) {
    motor_axis[axis_no].ParkingPos = value;
    motor_axis[axis_no].MotorPosNow = value;
    motor_axis[axis_no].EncoderPos =
        getEncoderPosFromMotorPos(axis_no, motor_axis[axis_no].MotorPosNow);
  }
}

void setMotorReverseERES(int axis_no, double value) {
  LOGTIME3("%s(%d) value=%g\n", __FUNCTION__, axis_no, value);
  if (((axis_no) <= 0) || ((axis_no) >= MAX_AXES)) {
    return;
  }
  motor_axis[axis_no].ReverseERES = value;
}

double getHomePos(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].HomeSwitchPos;
}

void setHomePos(int axis_no, double value) {
  LOGTIME3("%s(%d) value=%g\n", __FUNCTION__, axis_no, value);
  if (((axis_no) <= 0) || ((axis_no) >= MAX_AXES)) {
    return;
  }
  motor_axis[axis_no].HomeSwitchPos = value;
}

void setMaxHomeVelocityAbs(int axis_no, double value) {
  LOGTIME3("%s(%d) value=%g\n", __FUNCTION__, axis_no, value);
  if (((axis_no) <= 0) || ((axis_no) >= MAX_AXES)) {
    return;
  }
  motor_axis[axis_no].MaxHomeVelocityAbs = value;
}

static double getMotorVelocityInt(int axis_no) {
  if (motor_axis[axis_no].moving.velo.JogVelocity)
    return motor_axis[axis_no].moving.velo.JogVelocity;
  if (motor_axis[axis_no].moving.velo.PosVelocity)
    return motor_axis[axis_no].moving.velo.PosVelocity;
  if (motor_axis[axis_no].moving.velo.HomeVelocity)
    return motor_axis[axis_no].moving.velo.HomeVelocity;
  return 0;
}

double getMotorVelocity(int axis_no) {
  double velocity;
  AXIS_CHECK_RETURN_ZERO(axis_no);
  if (motor_axis[axis_no].moving.rampUpAfterStart) {
    return 0;
  }
  velocity = getMotorVelocityInt(axis_no);
  return velocity;
}

int isMotorHoming(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  if (motor_axis[axis_no].moving.velo.HomeVelocity) {
    return 1;
  }
  return 0;
}

int isMotorMoving(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  if (motor_axis[axis_no].bManualSimulatorMode) {
    return 0;
  }
  if (motor_axis[axis_no].moving.rampDownOnLimit) {
    motor_axis[axis_no].moving.rampDownOnLimit--;
    return 1;
  }
  if (motor_axis[axis_no].moving.rampUpAfterStart) {
    return 0;
  }
  if (motor_axis[axis_no].moving.velo.stillMoving) {
    return 1;
  }
  return getMotorVelocityInt(axis_no) ? 1 : 0;
}

int getAxisDone(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  int ret;
  ret = !isMotorMoving(axis_no);
  return ret;
}

int getAxisHome(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  int ret;
  ret = (motor_axis[axis_no].MotorPosNow == motor_axis[axis_no].HomePos);
  return ret;
}

int getAxisHomed(int axis_no) {
  int ret;
  AXIS_CHECK_RETURN_ZERO(axis_no);
  ret = motor_axis[axis_no].homed;
  return ret;
}

void setAxisHomed_fl(int axis_no, int value, const char *file, int line_no) {
  LOGTIME3("%s(%d) %s:%d value=%d\n", "setAxisHomed", axis_no, file, line_no,
           value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].homed = value;
  if (value) {
    motor_axis[axis_no].MotorPosSetPosOffset = 0;
    motor_axis[axis_no].moving.velo.HomeVelocity = 0;
  }
}

double getLowSoftLimitPos(int axis_no) {
  double value = 0;
  AXIS_CHECK_RETURN_ZERO(axis_no);
  value = motor_axis[axis_no].lowSoftLimitPos;
  return value;
}

void setLowSoftLimitPos(int axis_no, double value) {
  LOGTIME3("%s(%d) value=%g\n", __FUNCTION__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].lowSoftLimitPos = value;
}

int getEnableLowSoftLimit(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].enabledLowSoftLimitPos;
}

void setEnableLowSoftLimit(int axis_no, int value) {
  LOGTIME3("%s(%d) value=%d\n", __FUNCTION__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].enabledLowSoftLimitPos = value;
}

double getLowHardLimitPos(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].lowHardLimitPos;
}

void setLowHardLimitPos(int axis_no, double value) {
  LOGTIME3("%s(%d) value=%g\n", __FUNCTION__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].lowHardLimitPos = value;
  motor_axis[axis_no].definedLowHardLimitPos = 1;
}

double getHighSoftLimitPos(int axis_no) {
  double value = 0;
  AXIS_CHECK_RETURN_ZERO(axis_no);
  value = motor_axis[axis_no].highSoftLimitPos;
  return value;
}

void setHighSoftLimitPos(int axis_no, double value) {
  LOGTIME3("%s(%d) value=%g\n", __FUNCTION__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].highSoftLimitPos = value;
}

int getEnableHighSoftLimit(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].enabledHighSoftLimitPos;
}

void setEnableHighSoftLimit(int axis_no, int value) {
  LOGTIME3("%s(%d) value=%d\n", __FUNCTION__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].enabledHighSoftLimitPos = value;
}

double getHighHardLimitPos(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].highHardLimitPos;
}

void setHighHardLimitPos(int axis_no, double value) {
  LOGTIME3("%s(%d) value=%g\n", __FUNCTION__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].highHardLimitPos = value;
  motor_axis[axis_no].definedHighHardLimitPos = 1;
}

double getMRES_23(int axis_no) {
  double value = 0;
  AXIS_CHECK_RETURN_ZERO(axis_no);
  value = motor_axis[axis_no].MRES_23;
  return value;
}

int setMRES_23(int axis_no, double value) {
  LOGTIME3("%s(%d) value=%g\n", __FUNCTION__, axis_no, value);
  AXIS_CHECK_RETURN_ERROR(axis_no);
  if (!value) {
    /*
     * Normally the scaling can not be changed when the power is on.
     * But: The scaling can be switched off, even when power is on
     * Setting it to 0.0 will not work in a real controller,
     * but is needed in the simulator & TC 142
     */
    motor_axis[axis_no].MRES_23 = value;
    return 0;
  }
  if (motor_axis[axis_no].MRES_23 == value) return 0;
  if (getAmplifierOn(axis_no)) {
    LOGTIME3("%s(%d) error: amplifier is on MRES_23=%f\n", __FUNCTION__,
             axis_no, motor_axis[axis_no].MRES_24);
    return 1;
  }
  motor_axis[axis_no].MRES_23 = value;
  return 0;
}

double getMRES_24(int axis_no) {
  double value = 0;
  AXIS_CHECK_RETURN_ZERO(axis_no);
  value = motor_axis[axis_no].MRES_24;
  return value;
}

int setMRES_24(int axis_no, double value) {
  LOGTIME3("%s(%d) value=%g\n", __FUNCTION__, axis_no, value);
  AXIS_CHECK_RETURN_ERROR(axis_no);
  if (motor_axis[axis_no].MRES_24 == value) return 0;
  if (getAmplifierOn(axis_no)) {
    LOGTIME3("%s(%d) error: amplifier is on MRES_24=%f\n", __FUNCTION__,
             axis_no, motor_axis[axis_no].MRES_24);
    return 1;
  }
  motor_axis[axis_no].MRES_24 = value;
  return 0;
}

static int soft_limits_clip(int axis_no, double velocity) {
  int clipped = 0;
  /* Soft limits defined: Clip the value  */
  if (motor_axis[axis_no].enabledHighSoftLimitPos && velocity > 0 &&
      motor_axis[axis_no].MotorPosNow > motor_axis[axis_no].highSoftLimitPos) {
    LOGTIME3("%s(%d) CLIP soft high motorPosNow=%g highSoftLimitPos=%g\n",
             __FUNCTION__, axis_no, motor_axis[axis_no].MotorPosNow,
             motor_axis[axis_no].highSoftLimitPos);
    motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].highSoftLimitPos;
    clipped = 1;
  }
  if (motor_axis[axis_no].enabledLowSoftLimitPos && velocity < 0 &&
      motor_axis[axis_no].MotorPosNow < motor_axis[axis_no].lowSoftLimitPos) {
    LOGTIME3("%s(%d) CLIP soft low motorPosNow=%g lowSoftLimitPos=%g\n",
             __FUNCTION__, axis_no, motor_axis[axis_no].MotorPosNow,
             motor_axis[axis_no].lowSoftLimitPos);
    motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].lowSoftLimitPos;
    clipped = 1;
  }
  if (clipped) {
    motor_axis[axis_no].moving.rampDownOnLimit = RAMPDOWNONLIMIT;
  }
  return clipped;
} /* Soft limits */

static int hard_limits_clip(int axis_no, double velocity) {
  int clipped = 0;

  if (motor_axis[axis_no].highHardLimitPos >
      motor_axis[axis_no].lowHardLimitPos) {
    /* Hard limits defined: Clip the value  */
    if (motor_axis[axis_no].definedHighHardLimitPos && velocity > 0 &&
        calcMotorPosHw(axis_no) > motor_axis[axis_no].highHardLimitPos) {
      LOGTIME3(
          "%s(%d)CLIP LLS motorPosNow=%g calcMotorPosHw=%g "
          "velocity=%g jvel=%g velo=%g hvel=%g HomeVelocity=%g HomePos=%g "
          "highHardLimitPos=%g "
          "enabledHighSoftLimitPos=%d highSoftLimitPos=%g stillMoving=%d\n",
          __FUNCTION__, axis_no, motor_axis[axis_no].MotorPosNow,
          calcMotorPosHw(axis_no), velocity,
          motor_axis[axis_no].moving.velo.JogVelocity,
          motor_axis[axis_no].moving.velo.PosVelocity,
          motor_axis[axis_no].moving.velo.HomeVelocity,
          motor_axis[axis_no].HomePos,
          motor_axis[axis_no].moving.velo.HomeVelocity,
          motor_axis[axis_no].highHardLimitPos,
          motor_axis[axis_no].enabledHighSoftLimitPos,
          motor_axis[axis_no].highSoftLimitPos,
          motor_axis[axis_no].moving.velo.stillMoving);
      clipped = 1;
      if (motor_axis[axis_no].moving.velo.HomeVelocity) {
        /* setAxisHomed will set MotorPosSetPosOffset to 0 */
        setAxisHomed(axis_no, 1);
      }
      motor_axis[axis_no].MotorPosNow =
          motor_axis[axis_no].highHardLimitPos -
          motor_axis[axis_no].MotorPosSetPosOffset;
    }
    if (motor_axis[axis_no].definedLowHardLimitPos && velocity < 0 &&
        calcMotorPosHw(axis_no) < motor_axis[axis_no].lowHardLimitPos) {
      LOGTIME3(
          "%s(%d) CLIP LLS motorPosNow=%g calcMotorPosHw=%g "
          "velocity=%g jvel=%g velo=%g hvel=%g HomeVelocity=%g HomePos=%g "
          "lowHardLimitPos=%g "
          "enabledLowSoftLimitPos=%d lowSoftLimitPos=%g stillMoving=%d\n",
          __FUNCTION__, axis_no, motor_axis[axis_no].MotorPosNow,
          calcMotorPosHw(axis_no), velocity,
          motor_axis[axis_no].moving.velo.JogVelocity,
          motor_axis[axis_no].moving.velo.PosVelocity,
          motor_axis[axis_no].moving.velo.HomeVelocity,
          motor_axis[axis_no].HomePos,
          motor_axis[axis_no].moving.velo.HomeVelocity,
          motor_axis[axis_no].lowHardLimitPos,
          motor_axis[axis_no].enabledLowSoftLimitPos,
          motor_axis[axis_no].lowSoftLimitPos,
          motor_axis[axis_no].moving.velo.stillMoving);
      clipped = 1;
      if (motor_axis[axis_no].moving.velo.HomeVelocity) {
        /* setAxisHomed will set MotorPosSetPosOffset to 0 */
        setAxisHomed(axis_no, 1);
      }
      motor_axis[axis_no].MotorPosNow =
          motor_axis[axis_no].lowHardLimitPos -
          motor_axis[axis_no].MotorPosSetPosOffset;
    }
  }
  if (clipped) {
    motor_axis[axis_no].moving.rampDownOnLimit = RAMPDOWNONLIMIT;
  }
  return clipped;
} /* Hard limits */

void setHWlowPos(int axis_no, double value) {
  LOGTIME3("%s(%d) value=%g\n", __FUNCTION__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].HWlowPos = value;
}

void setHWhighPos(int axis_no, double value) {
  LOGTIME3("%s(%d) value=%g\n", __FUNCTION__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].HWhighPos = value;
}

void setHWhomeSwitchpos(int axis_no, double value) {
  LOGTIME3("%s(%d) value=%g\n", __FUNCTION__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].HWhomeSwitchpos = value;
}

static void simulateMotion(int axis_no) {
  struct timeval timeNow;
  double velocity;
  double jitter = 0.0;
  int clipped = 0;

  AXIS_CHECK_RETURN(axis_no);
  if (getManualSimulatorMode(axis_no)) return;

  if (motor_axis[axis_no].moving.rampUpAfterStart) {
    LOGTIME3("%s(%d) rampUpAfterStart=%d\n", __FUNCTION__, axis_no,
             motor_axis[axis_no].moving.rampUpAfterStart);
    motor_axis[axis_no].moving.rampUpAfterStart--;
    return;
  }
  velocity = getMotorVelocity(axis_no);

  if (!getAmplifierOn(axis_no)) {
    if (velocity) {
      /* Amplifier off, while moving */
      set_bError(axis_no, 1);
      set_nErrorId(axis_no, 16992);
      StopInternal(axis_no);
    }
  }

  gettimeofday(&timeNow, NULL);

  if (motor_axis[axis_no].moving.velo.JogVelocity) {
    /* Simulate jogging  */
    motor_axis[axis_no].MotorPosNow +=
        motor_axis[axis_no].moving.velo.JogVelocity *
        (timeNow.tv_sec - motor_axis[axis_no].lastPollTime.tv_sec);
    clipped = soft_limits_clip(axis_no, velocity);
  }

  if (motor_axis[axis_no].moving.velo.PosVelocity) {
    clipped = soft_limits_clip(axis_no, velocity);
    if (!clipped) {
      /* Simulate a move to postion */
      motor_axis[axis_no].MotorPosNow +=
          motor_axis[axis_no].moving.velo.PosVelocity *
          (timeNow.tv_sec - motor_axis[axis_no].lastPollTime.tv_sec);
      if (((motor_axis[axis_no].moving.velo.PosVelocity > 0) &&
           (motor_axis[axis_no].MotorPosNow >
            motor_axis[axis_no].MotorPosWanted)) ||
          ((motor_axis[axis_no].moving.velo.PosVelocity < 0) &&
           (motor_axis[axis_no].MotorPosNow <
            motor_axis[axis_no].MotorPosWanted))) {
        /* overshoot or undershoot. We are at the target position */
        motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].MotorPosWanted;
        motor_axis[axis_no].moving.velo.PosVelocity = 0;
      }
    }
  }
  if (motor_axis[axis_no].moving.velo.HomeVelocity) {
    double motorPosHw = calcMotorPosHw(axis_no);
    /* Simulate move to home */
    motor_axis[axis_no].MotorPosNow +=
        motor_axis[axis_no].moving.velo.HomeVelocity *
        (timeNow.tv_sec - motor_axis[axis_no].lastPollTime.tv_sec);
    if (((motor_axis[axis_no].moving.velo.HomeVelocity > 0) &&
         (motorPosHw > motor_axis[axis_no].HomePos)) ||
        ((motor_axis[axis_no].moving.velo.HomeVelocity < 0) &&
         (motorPosHw < motor_axis[axis_no].HomePos))) {
      /* overshoot or undershoot. We are at home */
      motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].HomePos;
    }
    if (calcMotorPosHw(axis_no) == motor_axis[axis_no].HomePos) {
      setAxisHomed(axis_no, 1);
    }
  }

  motor_axis[axis_no].lastPollTime = timeNow;
  clipped |= hard_limits_clip(axis_no, velocity);

  /* Jitter only if the motor is not moving */
  if (!clipped && !velocity && motor_axis[axis_no].positionJitterPlus &&
      timeNow.tv_sec != motor_axis[axis_no].lastJitterTime.tv_sec) {
    motor_axis[axis_no].lastJitterTime.tv_sec = timeNow.tv_sec;
    motor_axis[axis_no].positionJitterPlus =
        -motor_axis[axis_no].positionJitterPlus;
    jitter = motor_axis[axis_no].positionJitterPlus *
             motor_axis[axis_no].positionJitter;
  }

  motor_axis[axis_no].MotorPosReported = calcMotorPosReported(axis_no, jitter);
  /* Compare moving to see if there is anything new */
  if (memcmp(&motor_axis_last[axis_no].moving, &motor_axis[axis_no].moving,
             sizeof(motor_axis[axis_no].moving)) ||
      motor_axis_last[axis_no].MotorPosNow != motor_axis[axis_no].MotorPosNow ||
      motor_axis_last[axis_no].MotorPosWanted !=
          motor_axis[axis_no].MotorPosWanted ||
      clipped) {
    LOGTIME3(
        "%s(%d) vel=%g MotorPosWanted=%g JogVel=%g PosVel=%g "
        "HomeVel=%g "
        "RampDown=%d home=%d motorPosNow=%g (%g) calcMotorPosHw=%g\n",
        __FUNCTION__, axis_no, velocity, motor_axis[axis_no].MotorPosWanted,
        motor_axis[axis_no].moving.velo.JogVelocity,
        motor_axis[axis_no].moving.velo.PosVelocity,
        motor_axis[axis_no].moving.velo.HomeVelocity,
        motor_axis[axis_no].moving.rampDownOnLimit, getAxisHome(axis_no),
        motor_axis[axis_no].MotorPosNow, motor_axis[axis_no].MotorPosReported,
        calcMotorPosHw(axis_no));
    memcpy(&motor_axis_last[axis_no], &motor_axis[axis_no],
           sizeof(motor_axis[axis_no]));
  }
  /*
    homing against a limit switch does not clip,
    jogging and positioning does, and cause a
    rampdown, which needs to be handled correctly
    by the driver and the motor/axisRecord
  */
  if (clipped) {
    StopInternal(axis_no);
  }
  motor_axis[axis_no].moving.clipped = clipped;
}

double calcMotorPosReported(int axis_no, double jitter) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  double MotorPosNow = motor_axis[axis_no].MotorPosNow + jitter;
  if (motor_axis[axis_no].MRES_23 && motor_axis[axis_no].MRES_24) {
    /* If we have a scaling, round the position to a step */
    double srev = motor_axis[axis_no].MRES_24;
    double urev = motor_axis[axis_no].MRES_23;
    if (urev > 0.0) {
      long step = NINT(MotorPosNow * srev / urev);
      double ret = (double)step * urev / srev;
      return ret;
    }
  }
  return MotorPosNow;
}

double getMotorPos(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  simulateMotion(axis_no);
  /* simulate EncoderPos */
  motor_axis[axis_no].EncoderPos =
      getEncoderPosFromMotorPos(axis_no, motor_axis[axis_no].MotorPosNow);
  return motor_axis[axis_no].MotorPosReported;
}

void setMotorPos_fl(int axis_no, double value, int flags, const char *file,
                    int line_no) {
  LOGTIME3("setMotorPos(%d) (%s:%d) value=%g flags=0x%x\n", axis_no, file,
           line_no, value, flags);
  AXIS_CHECK_RETURN(axis_no);
  int stillMoving = isMotorMoving(axis_no);
  if (stillMoving) {
    StopInternal(axis_no);
  }
  motor_axis[axis_no].homed = 1;
  if (flags & SET_MOTOR_POS_FLAGS_KEEP_MOVING) {
    motor_axis[axis_no].moving.velo.stillMoving = stillMoving;
  }

  if (flags & SET_MOTOR_POS_FLAGS_KEEP_LIMITS_FORCE) {
    /*
      The new position is pushed into the axis.
       This is a "fast move", the offset is set to 0
    */
    motor_axis[axis_no].MotorPosSetPosOffset = 0;
  } else {
    /* The new position is pushed into the axis.
       This corresponds to setPosition() in EPICS model 3,
       used to home the motor to a certain position.
       If the motor is at 30.0 and pushed into 50.0 (which is wrong)
       we may run into soft- or hard limits at unexpected positions.
       For that: save the offset so that
       the hardware (limit switches, encoder) stay where they are
    */
    /* Save the old MotorPosSetPosOffset for printing below */
    double oldMotorPosSetPosOffset = motor_axis[axis_no].MotorPosSetPosOffset;
    /* Calculate the new MotorPosSetPosOffset */
    motor_axis[axis_no].MotorPosSetPosOffset = calcMotorPosHw(axis_no) - value;
    LOGTIME3(
        "setMotorPos(%d) value=%g oldMotorPosSetPosOffset=%g "
        "newMotorPosSetPosOffset=%g\n",
        axis_no, value, oldMotorPosSetPosOffset,
        motor_axis[axis_no].MotorPosSetPosOffset);
  }
  /* simulate EncoderPos */
  motor_axis[axis_no].moving.velo.HomeVelocity = 0;
  motor_axis[axis_no].MotorPosNow = value;
  {
    double jitter = 0.0;
    motor_axis[axis_no].MotorPosReported =
        calcMotorPosReported(axis_no, jitter);
  }
  motor_axis[axis_no].EncoderPos =
      getEncoderPosFromMotorPos(axis_no, motor_axis[axis_no].MotorPosNow);
}

void setPositionJitter(int axis_no, double value) {
  AXIS_CHECK_RETURN(axis_no);
  LOGTIME3("setPositionJitter(%d) value=%g\n", axis_no, value);
  motor_axis[axis_no].positionJitter = value;
  motor_axis[axis_no].positionJitterPlus = 1;
}

double getEncoderPos(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  (void)getMotorPos(axis_no);
  if (motor_axis_reported[axis_no].EncoderPos !=
      motor_axis[axis_no].EncoderPos) {
    LOGTIME3("%s(%d) EncoderPos=%g\n", __FUNCTION__, axis_no,
             motor_axis[axis_no].EncoderPos);
    motor_axis_reported[axis_no].EncoderPos = motor_axis[axis_no].EncoderPos;
  }
  return motor_axis[axis_no].EncoderPos;
}

/* Stop the ongoing motion (like JOG),
   to be able to start a new one (like HOME)
*/
void StopInternal_fl(int axis_no, const char *file, int line_no) {
  unsigned int rampDownOnLimit;
  rampDownOnLimit = motor_axis[axis_no].moving.rampDownOnLimit;

  LOGTIME3("%s(%d) rampDownOnLimit=%d file=%s line_no=%d\n", __FUNCTION__,
           axis_no, rampDownOnLimit, file, line_no);
  AXIS_CHECK_RETURN(axis_no);
  memset(&motor_axis[axis_no].moving.velo, 0,
         sizeof(motor_axis[axis_no].moving.velo));
  /* Restore the ramp down */
  motor_axis[axis_no].moving.rampDownOnLimit = rampDownOnLimit;
}

/* caput pv.VAL */
int movePosition(int axis_no, double position, int relative,
                 double max_velocity, double acceleration) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  if (motor_axis[axis_no].logFile) {
    if (relative) {
      fprintf(motor_axis[axis_no].logFile,
              "move relative delta=%g max_velocity=%g acceleration=%g "
              "motorPosNow=%g\n",
              position, max_velocity, acceleration,
              motor_axis[axis_no].MotorPosNow);
    } else {
      fprintf(motor_axis[axis_no].logFile,
              "move absolute position=%g max_velocity=%g acceleration=%g "
              "motorPosNow=%g\n",
              position, max_velocity, acceleration,
              motor_axis[axis_no].MotorPosNow);
    }
    fflush(motor_axis[axis_no].logFile);
  }

  LOGTIME3(
      "%s%s(%d) relative=%d position=%g max_velocity=%g "
      "acceleration=%g motorPosNow=%g\n",
      motor_axis[axis_no].logFile ? "LLLL " : "", __FUNCTION__, axis_no,
      relative, position, max_velocity, acceleration,
      motor_axis[axis_no].MotorPosNow);
  StopInternal(axis_no);
  gettimeofday(&motor_axis[axis_no].lastPollTime, NULL);

  if (relative) {
    position += motor_axis[axis_no].MotorPosNow;
  }
  if (motor_axis[axis_no].enabledLowSoftLimitPos &&
      position < motor_axis[axis_no].lowSoftLimitPos) {
    set_nErrorId(axis_no, 0x4460);
    StopInternal(axis_no);
    return 0;
  } else if (motor_axis[axis_no].enabledHighSoftLimitPos &&
             position > motor_axis[axis_no].highSoftLimitPos) {
    set_nErrorId(axis_no, 0x4461);
    StopInternal(axis_no);
    return 0;
  }
  motor_axis[axis_no].MotorPosWanted = position;

  if (position > motor_axis[axis_no].MotorPosNow) {
    motor_axis[axis_no].moving.velo.PosVelocity = max_velocity;
    motor_axis[axis_no].moving.rampUpAfterStart =
        motor_axis[axis_no].defRampUpAfterStart;
  } else if (position < motor_axis[axis_no].MotorPosNow) {
    motor_axis[axis_no].moving.velo.PosVelocity = -max_velocity;
    motor_axis[axis_no].moving.rampUpAfterStart =
        motor_axis[axis_no].defRampUpAfterStart;
  } else {
    motor_axis[axis_no].moving.velo.PosVelocity = 0;
  }

  return 0;
}

int moveRelative(int axis_no, double position) {
  int relative = 1;
  AXIS_CHECK_RETURN_ZERO(axis_no);
  fprintf(stdlog, "moveRelative axis_no=%d position=%g\n", axis_no, position);
  return movePosition(axis_no, position, relative,
                      motor_axis[axis_no].nxtMoveVelocity,
                      motor_axis[axis_no].nxtMoveAcceleration);
}

int moveHomeProc(int axis_no, int direction, int nCmdData, double position,
                 double max_velocity, double acceleration) {
  double velocity =
      max_velocity ? max_velocity : motor_axis[axis_no].MaxHomeVelocityAbs;
  velocity = fabs(velocity);
  if (motor_axis[axis_no].logFile) {
    fprintf(motor_axis[axis_no].logFile,
            "moveHomeProc axis_no=%d nCmdData=%d position=%g max_velocity=%g "
            "velocity=%g acceleration=%g motorPosNow=%g\n",
            axis_no, nCmdData, position, max_velocity, velocity, acceleration,
            motor_axis[axis_no].MotorPosNow);
    fflush(motor_axis[axis_no].logFile);
  }
  LOGTIME3(
      "%s%s(%d) nCmdData=%d position=%g max_velocity=%g "
      "velocity=%g acceleration=%g\n",
      motor_axis[axis_no].logFile ? "LLLL " : "", __FUNCTION__, axis_no,
      nCmdData, position, max_velocity, velocity, acceleration);

  if (nCmdData == 15) {
    int flags = 0;
    setMotorPos(axis_no, position, flags);
    return 0;
  }
  motor_axis[axis_no].HomePos = position;
  motor_axis[axis_no].MotorPosWanted = position;
  motor_axis[axis_no].HomeProc = nCmdData;

  if (motor_axis[axis_no].MaxHomeVelocityAbs &&
      (fabs(velocity) > motor_axis[axis_no].MaxHomeVelocityAbs)) {
    velocity = motor_axis[axis_no].MaxHomeVelocityAbs;
  }
  motor_axis[axis_no].HomeVelocityAbsWanted = velocity;
  LOGTIME3(
      "%s(%d)direction=%d max_velocity=%g velocity=%g "
      "acceleration=%g\n",
      __FUNCTION__, axis_no, direction, max_velocity, velocity, acceleration);
  StopInternal(axis_no);
  setAxisHomed(axis_no, 0); /* Not homed any more */
  gettimeofday(&motor_axis[axis_no].lastPollTime, NULL);

  if (position > motor_axis[axis_no].MotorPosNow) {
    motor_axis[axis_no].moving.velo.HomeVelocity = velocity;
    motor_axis[axis_no].moving.rampUpAfterStart =
        motor_axis[axis_no].defRampUpAfterStart;
  } else if (position < motor_axis[axis_no].MotorPosNow) {
    motor_axis[axis_no].moving.velo.HomeVelocity = -velocity;
    motor_axis[axis_no].moving.rampUpAfterStart =
        motor_axis[axis_no].defRampUpAfterStart;
  } else {
    setAxisHomed(axis_no, 1); /* homed again */
  }

  return 0;
};

/* caput pv.HOMF, caput pv.HOMR */
int moveHome(int axis_no, int direction, double max_velocity,
             double acceleration) {
  return moveHomeProc(axis_no, direction, HomProc_LOW_HS, /* int nCmdData, */
                      motor_axis[axis_no].HomePos, max_velocity, acceleration);
}

/* caput pv.JOGF, caput pv.JOGR */
int moveVelocity(int axis_no, int direction, double max_velocity,
                 double acceleration) {
  double velocity = max_velocity;
  if (!direction) {
    velocity = -velocity;
  }

  StopInternal(axis_no);

  if (motor_axis[axis_no].logFile) {
    fprintf(motor_axis[axis_no].logFile,
            "move velocity direction=%d max_velocity=%g "
            "acceleration=%g motorPosNow=%g\n",
            direction, max_velocity, acceleration,
            motor_axis[axis_no].MotorPosNow);
    fflush(motor_axis[axis_no].logFile);
  }
  LOGTIME3("%s%s(%d) direction=%d max_velocity=%g acceleration=%g\n",
           motor_axis[axis_no].logFile ? "LLLL " : "", __FUNCTION__, axis_no,
           direction, max_velocity, acceleration);
  if (direction < 0) {
    velocity = -velocity;
  }
  motor_axis[axis_no].moving.velo.JogVelocity = velocity;
  motor_axis[axis_no].moving.rampUpAfterStart =
      motor_axis[axis_no].defRampUpAfterStart;
  return 0;
};

int setAmplifierPercent(int axis_no, int percent) {
  LOGTIME3("%s(%d) percent=%d\n", __FUNCTION__, axis_no, percent);
  AXIS_CHECK_RETURN_ERROR(axis_no);
  if (percent < 0 || percent > 100) return -1;
  if (getAmplifierLockedToBeOff(axis_no)) {
    percent = 0;
    LOGTIME3("%s(%d) AmplifierLockedToBeOff=1 percent=%d\n", __FUNCTION__,
             axis_no, percent);
  }
  motor_axis[axis_no].amplifierPercent = percent;
  if (percent > 0) {
    gettimeofday(&motor_axis[axis_no].powerOnTime, NULL);
  }

  return 0;
}

int getAmplifierOn(int axis_no) {
  if (motor_axis[axis_no].amplifierPercent == 100) return 1;
  if (getAmplifierLockedToBeOff(axis_no)) return 0;
  if (motor_axis[axis_no].amplifierPercent > 0) {
    struct timeval timeNow;
    gettimeofday(&timeNow, NULL);
    while ((motor_axis[axis_no].amplifierPercent < 100) &&
           (timeNow.tv_sec > motor_axis[axis_no].powerOnTime.tv_sec)) {
      timeNow.tv_sec--;
      motor_axis[axis_no].amplifierPercent++;
    }
  }
  if (motor_axis[axis_no].amplifierPercent == 100) {
    LOGTIME3("%s(%d) amplifierPercent=%d\n", __FUNCTION__, axis_no,
             motor_axis[axis_no].amplifierPercent);
    return 1;
  } else
    return 0;
}

void getAxisDebugInfoData(int axis_no, char *buf, size_t maxlen) {
  snprintf(buf, maxlen,
           "rvel=%g VAL=%g JVEL=%g VELO=%g HVEL=%g athome=%d RBV=%g",
           getMotorVelocity(axis_no), motor_axis[axis_no].MotorPosWanted,
           motor_axis[axis_no].moving.velo.JogVelocity,
           motor_axis[axis_no].moving.velo.PosVelocity,
           motor_axis[axis_no].moving.velo.HomeVelocity, getAxisHome(axis_no),
           motor_axis[axis_no].MotorPosNow);
}

int getNegLimitSwitch(int axis_no) {
  double calcedMotorPosHw = calcMotorPosHw(axis_no);
  int clipped = motor_axis[axis_no].definedLowHardLimitPos &&
                (calcedMotorPosHw <= motor_axis[axis_no].lowHardLimitPos);

  motor_axis[axis_no].moving.hitNegLimitSwitch = clipped;
  if (motor_axis_reported[axis_no].moving.hitNegLimitSwitch !=
      motor_axis[axis_no].moving.hitNegLimitSwitch) {
    LOGTIME3(
        "%s(%d) definedLowHardLimitPos=%d motorPosNow=%g "
        "calcedMotorPosHw=%g "
        "lowHardLimitPos=%g hitNegLimitSwitch=%d\n",
        __FUNCTION__, axis_no, motor_axis[axis_no].definedLowHardLimitPos,
        motor_axis[axis_no].MotorPosNow, calcedMotorPosHw,
        motor_axis[axis_no].lowHardLimitPos,
        motor_axis[axis_no].moving.hitNegLimitSwitch);
    motor_axis_reported[axis_no].moving.hitNegLimitSwitch =
        motor_axis[axis_no].moving.hitNegLimitSwitch;
    if (clipped) {
      motor_axis[axis_no].moving.rampDownOnLimit = RAMPDOWNONLIMIT;
    }
  }
  return clipped;
}

int getPosLimitSwitch(int axis_no) {
  double calcedMotorPosHw = calcMotorPosHw(axis_no);
  int clipped = (calcedMotorPosHw >= motor_axis[axis_no].highHardLimitPos);

  motor_axis[axis_no].definedHighHardLimitPos = clipped;
  if (motor_axis_reported[axis_no].moving.hitPosLimitSwitch !=
      motor_axis[axis_no].moving.hitPosLimitSwitch) {
    LOGTIME3(
        "%s(%d) definedHighHardLimitPos=%d motorPosNow=%g "
        "calcedMotorPosHw=%g "
        "highHardLimitPos=%g hitPosLimitSwitch=%d\n",
        __FUNCTION__, axis_no, motor_axis[axis_no].definedHighHardLimitPos,
        motor_axis[axis_no].MotorPosNow, calcedMotorPosHw,
        motor_axis[axis_no].highHardLimitPos,
        motor_axis[axis_no].moving.hitPosLimitSwitch);
    motor_axis_reported[axis_no].moving.hitPosLimitSwitch =
        motor_axis[axis_no].moving.hitPosLimitSwitch;
    if (clipped) {
      motor_axis[axis_no].moving.rampDownOnLimit = RAMPDOWNONLIMIT;
    }
  }
  return clipped;
}

int get_bError(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].bError;
}

int set_bError(int axis_no, int value) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  motor_axis[axis_no].bError = value;
  return 0;
}

int get_nErrorId(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].nErrorId;
}

int set_nErrorId(int axis_no, int value) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  motor_axis[axis_no].nErrorId = value;
  return 0;
}

unsigned get_nStatReasAUX(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].nStatReasAUX;
}

int set_nStatReasAUX(int axis_no, unsigned value) {
  LOGTIME3("%s(%d) value=0x%08x\n", __FUNCTION__, axis_no, value);
  AXIS_CHECK_RETURN_ZERO(axis_no);
  motor_axis[axis_no].nStatReasAUX = value;
  return 0;
}

/*
 *  Debug logfile.
 */
int openLogFile(int axis_no, const char *filename) {
  AXIS_CHECK_RETURN_EINVAL(axis_no);
  LOGTIME3("LLLL %s(%d) filename=%s\n", __FUNCTION__, axis_no, filename);
  motor_axis[axis_no].logFile = fopen(filename, "w+");
  if (!motor_axis[axis_no].logFile) return errno;

  return 0;
}

int closeLogFile(int axis_no) {
  const static char *EOF_LF_str = "EOF\n";
  int strlen_EOF_LF_str = (int)strlen(EOF_LF_str);
  int res1 = 0;
  int res2 = -1;
  AXIS_CHECK_RETURN_ERROR(axis_no);
  if (motor_axis[axis_no].logFile) {
    res1 = fprintf(motor_axis[axis_no].logFile, "%s", EOF_LF_str);
    res2 = fclose(motor_axis[axis_no].logFile);
    motor_axis[axis_no].logFile = NULL;
  }
  LOGTIME3("LLLL %s(%d) res1=%d strlen_EOF_LF_str=%d res2=%d\n", __FUNCTION__,
           axis_no, res1, strlen_EOF_LF_str, res2);

  if ((res1 == strlen_EOF_LF_str) && (res2 == 0))
    return 0;
  else
    return 1;
}

int getManualSimulatorMode(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].bManualSimulatorMode;
}

void setManualSimulatorMode(int axis_no, int manualMode) {
  AXIS_CHECK_RETURN(axis_no);
  LOGTIME3("%s(%d) manualMode=%d\n", __FUNCTION__, axis_no, manualMode);
  if (motor_axis[axis_no].bManualSimulatorMode && !manualMode) {
    /* Manual mode switched off, stop to prevent the motor to
       start moving */
    StopInternal(axis_no);
  }
  motor_axis[axis_no].bManualSimulatorMode = manualMode;
}

int getLocalmode(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].bLocalMode;
}

void setLocalmode(int axis_no, int localMode) {
  AXIS_CHECK_RETURN(axis_no);
  LOGTIME3("setLocalmode(%d) %d\n", axis_no, localMode);
  motor_axis[axis_no].bLocalMode = localMode;
}

int getInterlockBwd(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].bInterlockBwd;
}

void setInterlockBwd(int axis_no, int interlockBwd) {
  AXIS_CHECK_RETURN(axis_no);
  LOGTIME3("interlockBwd(%d) %d\n", axis_no, interlockBwd);
  motor_axis[axis_no].bInterlockBwd = interlockBwd;
}

int getInterlockFwd(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].bInterlockFwd;
}

void setInterlockFwd(int axis_no, int interlockFwd) {
  AXIS_CHECK_RETURN(axis_no);
  LOGTIME3("interlockFwd(%d) %d\n", axis_no, interlockFwd);
  motor_axis[axis_no].bInterlockFwd = interlockFwd;
}

int getAmplifierLockedToBeOff(int axis_no) {
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].amplifierLockedToBeOff;
}

void setAmplifierLockedToBeOff(int axis_no, int value) {
  AXIS_CHECK_RETURN(axis_no);
  LOGTIME3("%s%s(%d) value=%d\n", motor_axis[axis_no].logFile ? "LLLL " : "",
           __FUNCTION__, axis_no, value);
  motor_axis[axis_no].amplifierLockedToBeOff = value;
}
