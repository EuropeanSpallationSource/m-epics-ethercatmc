#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <math.h>
#include "hw_motor.h"
#include "sock-util.h" /* stdlog */

#define MOTOR_POS_HOME 0
#define MOTOR_REV_ERES (-57)
#define MOTOR_PARK_POS (-64)

/* Homing procdures LS=Limit switch, HS=Home switch */
#define ProcHom_LOW_LS  1
#define ProcHom_HIGH_LS 2
#define ProcHom_LOW_HS  3
#define ProcHom_HIGH_HS 4

#define MOTOR_VEL_HOME_MAX 5.0

#define RAMPDOWNONLIMIT 3

typedef struct
{
  struct timeval lastPollTime;

  double amplifierPercent;
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
  double HomeSwitchPos;     /* home switch */
  double HomeProcPos;       /* Position of used home switch */
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
  double MotorPosNow;
  double MotorPosWanted;
  double HomeVelocityAbsWanted;
  double MaxHomeVelocityAbs;
  struct {
    struct {
      double HomeVelocity;
      double PosVelocity;
      double JogVelocity;
    } velo;
    int hitPosLimitSwitch;
    int hitNegLimitSwitch;
    unsigned int rampDownOnLimit;
    int clipped;
  } moving;
  double EncoderPos;
  double ParkingPos;
  double ReverseERES;
  int homed;
  int nErrorId;
  FILE *logFile;
  int bManualSimulatorMode;
  int amplifierLockedToBeOff;
} motor_axis_type;


static motor_axis_type motor_axis[MAX_AXES];
static motor_axis_type motor_axis_last[MAX_AXES];
static motor_axis_type motor_axis_reported[MAX_AXES];

static void recalculate_pos(int axis_no, int nCmdData)
{
  double HWlowPos = motor_axis[axis_no].HWlowPos;
  double HWhomeSwitchpos = motor_axis[axis_no].HWhomeSwitchpos;
  double HWhighPos = motor_axis[axis_no].HWhighPos;
  double oldLowHardLimitPos = motor_axis[axis_no].lowHardLimitPos;
  switch (nCmdData) {
    case ProcHom_LOW_LS:
      motor_axis[axis_no].lowHardLimitPos = 0;
      motor_axis[axis_no].HomeSwitchPos = HWhomeSwitchpos - HWlowPos;
      motor_axis[axis_no].highHardLimitPos = HWhighPos - HWlowPos;
      break;
    case ProcHom_HIGH_LS:
      motor_axis[axis_no].lowHardLimitPos = HWlowPos - HWhighPos;
      motor_axis[axis_no].HomeSwitchPos = HWhomeSwitchpos - HWhighPos;
      motor_axis[axis_no].highHardLimitPos = 0;
      break;
    case ProcHom_LOW_HS:
    case ProcHom_HIGH_HS:
      motor_axis[axis_no].lowHardLimitPos = HWlowPos;
      motor_axis[axis_no].HomeSwitchPos = 0;
      motor_axis[axis_no].highHardLimitPos = HWhighPos;
      break;
  }
  motor_axis[axis_no].HomeProcPos = 0; /* in any case */
  motor_axis[axis_no].MotorPosWanted = 0;
  /* adjust position to "force a simulated movement" */
  motor_axis[axis_no].MotorPosNow += motor_axis[axis_no].lowHardLimitPos - oldLowHardLimitPos;

  fprintf(stdlog,
          "%s/%s:%d axis_no=%d motorPosNow=%g lowHardLimitPos=%g HomeSwitchPos=%g higHardLimitPos=%g\n",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no,
          motor_axis[axis_no].MotorPosNow,
          motor_axis[axis_no].lowHardLimitPos,
          motor_axis[axis_no].HomeSwitchPos,
          motor_axis[axis_no].highHardLimitPos);
}

static double getEncoderPosFromMotorPos(int axis_no, double MotorPosNow)
{
  (void)axis_no;
  return ((MotorPosNow - motor_axis[axis_no].ParkingPos)) * motor_axis[axis_no].ReverseERES;
}

#if 0
static double getMotorPosFromEncoderPos(int axis_no, double EncoderPos)
{
  (void)axis_no;
  return (double)(int)((EncoderPos / motor_axis[axis_no].ReverseERES) + motor_axis[axis_no].ParkingPos);
}
#endif

void hw_motor_init(int axis_no,
                   double ReverseERES,
                   double ParkingPos,
                   double MaxHomeVelocityAbs,
                   double lowHardLimitPos,
                   double highHardLimitPos,
                   double hWlowPos,
                   double hWhighPos,
                   double homeSwitchPos)
{
  static char init_done[MAX_AXES];


  fprintf(stdlog,
          "%s/%s:%d axis_no=%d ReverseERES=%f ParkingPos=%f MaxHomeVelocityAbs=%f"
          "\n  lowHardLimitPos=%f highHardLimitPos=%f hWlowPos=%f hWhighPos=%f homeSwitchPos=%f\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no,
          ReverseERES,
          ParkingPos,
          MaxHomeVelocityAbs,
          lowHardLimitPos,
          highHardLimitPos,
          hWlowPos,
          hWhighPos,
          homeSwitchPos);

  if (axis_no >= MAX_AXES || axis_no < 0) {
    return;
  }
  if (!init_done[axis_no]) {
    memset(&motor_axis[axis_no], 0, sizeof(motor_axis[axis_no]));
    memset(&motor_axis_last[axis_no], 0, sizeof(motor_axis_last[axis_no]));
    memset(&motor_axis_reported[axis_no], 0, sizeof(motor_axis_reported[axis_no]));

    motor_axis[axis_no].ReverseERES = ReverseERES;
    motor_axis[axis_no].ParkingPos = ParkingPos;
    motor_axis[axis_no].MotorPosNow = ParkingPos;
    motor_axis[axis_no].MaxHomeVelocityAbs = MaxHomeVelocityAbs;


    motor_axis[axis_no].lowHardLimitPos = lowHardLimitPos;
    motor_axis[axis_no].definedLowHardLimitPos = 1;
    motor_axis[axis_no].highHardLimitPos = highHardLimitPos;
    motor_axis[axis_no].definedHighHardLimitPos = 1;

    motor_axis[axis_no].HWlowPos = hWlowPos;
    motor_axis[axis_no].HWhighPos = hWhighPos;

    motor_axis[axis_no].HomeSwitchPos = homeSwitchPos;
    //motor_axis[axis_no].amplifierPercent = 100;
    // setMotorParkingPosition(axis_no, MOTOR_PARK_POS);
    // motor_axis[axis_no].ReverseERES = MOTOR_REV_ERES;
    motor_axis[axis_no].EncoderPos = getEncoderPosFromMotorPos(axis_no, motor_axis[axis_no].MotorPosNow);
    motor_axis_last[axis_no].EncoderPos  = motor_axis[axis_no].EncoderPos;
    motor_axis_last[axis_no].MotorPosNow = motor_axis[axis_no].MotorPosNow;
    init_done[axis_no] = 1;
  }
}


static void init_axis(int axis_no)
{
  (void)axis_no;
  // hw_motor_init(axis_no);
}


void setMotorParkingPosition(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%g\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  if (((axis_no) <= 0) || ((axis_no) >=MAX_AXES)) {
    return;
  }
  motor_axis[axis_no].ParkingPos = value;
  motor_axis[axis_no].MotorPosNow = value;
  motor_axis[axis_no].EncoderPos =
    getEncoderPosFromMotorPos(axis_no, motor_axis[axis_no].MotorPosNow);
}

void setMotorReverseERES(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%g\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  if (((axis_no) <= 0) || ((axis_no) >=MAX_AXES)) {
    return;
  }
  motor_axis[axis_no].ReverseERES = value;
}


void setHomePos(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%g\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  if (((axis_no) <= 0) || ((axis_no) >=MAX_AXES)) {
    return;
  }
  motor_axis[axis_no].HomeSwitchPos = value;
}

void setMaxHomeVelocityAbs(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%g\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  if (((axis_no) <= 0) || ((axis_no) >=MAX_AXES)) {
    return;
  }
  motor_axis[axis_no].MaxHomeVelocityAbs = value;
}

static double getMotorVelocityInt(int axis_no)
{
  if (motor_axis[axis_no].moving.velo.JogVelocity) return motor_axis[axis_no].moving.velo.JogVelocity;
  if (motor_axis[axis_no].moving.velo.PosVelocity) return motor_axis[axis_no].moving.velo.PosVelocity;
  if (motor_axis[axis_no].moving.velo.HomeVelocity) return motor_axis[axis_no].moving.velo.HomeVelocity;
  return 0;
}

double getMotorVelocity(int axis_no)
{
  double velocity;
  AXIS_CHECK_RETURN_ZERO(axis_no);
  velocity = getMotorVelocityInt(axis_no);
  return velocity;
}

int isMotorMoving(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  if (motor_axis[axis_no].bManualSimulatorMode) {
    return 0;
  }
  if (motor_axis[axis_no].moving.rampDownOnLimit) {
    motor_axis[axis_no].moving.rampDownOnLimit--;
    return 1;
  }
  return getMotorVelocityInt(axis_no) ? 1 : 0;
}

int getAxisDone(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  int ret;
  ret = !isMotorMoving(axis_no);
  return ret;
}

int getAxisHome(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  int ret;
  ret = (motor_axis[axis_no].MotorPosNow == motor_axis[axis_no].HomeProcPos);
  return ret;
}

int getAxisHomed(int axis_no)
{
  int ret;
  AXIS_CHECK_RETURN_ZERO(axis_no);
  ret = motor_axis[axis_no].homed;
  return ret;
}

void setAxisHomed(int axis_no, int value)
{
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].homed = value;
}


double getLowSoftLimitPos(int axis_no)
{
  double value = 0;
  AXIS_CHECK_RETURN_ZERO(axis_no);
  value = motor_axis[axis_no].lowSoftLimitPos;
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%g\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  return value;
}

void setLowSoftLimitPos(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%g\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no,
          value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].lowSoftLimitPos = value;
}

void enableLowSoftLimit(int axis_no, int value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%d\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].enabledLowSoftLimitPos = value;
}

void setLowHardLimitPos(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%g\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].lowHardLimitPos = value;
  motor_axis[axis_no].definedLowHardLimitPos = 1;
}

double getHighSoftLimitPos(int axis_no)
{
  double value = 0;
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%g\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN_ZERO(axis_no);
  value = motor_axis[axis_no].highSoftLimitPos;
  return value;
}

void setHighSoftLimitPos(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%g\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no,
          value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].highSoftLimitPos = value;
}

void enableHighSoftLimit(int axis_no, int value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%d\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].enabledHighSoftLimitPos = value;
}

void setHighHardLimitPos(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%g\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].highHardLimitPos = value;
  motor_axis[axis_no].definedHighHardLimitPos = 1;
}

double getMRES_23(int axis_no)
{
  double value = 0;
  AXIS_CHECK_RETURN_ZERO(axis_no);
  value = motor_axis[axis_no].MRES_23;
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%g\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  return value;
}

int setMRES_23(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%g\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no,
          value);
  AXIS_CHECK_RETURN_ERROR(axis_no);
  if (getAmplifierOn(axis_no))
    return 1;
  motor_axis[axis_no].MRES_23 = value;
  return 0;
}

double getMRES_24(int axis_no)
{
  double value = 0;
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%g\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN_ZERO(axis_no);
  value = motor_axis[axis_no].MRES_24;
  return value;
}

int setMRES_24(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%g\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no,
          value);
  AXIS_CHECK_RETURN_ERROR(axis_no);
  if (getAmplifierOn(axis_no))
    return 1;
  motor_axis[axis_no].MRES_24 = value;
  return 0;
}

static int soft_limits_clip(int axis_no, double velocity)
{
  int clipped = 0;
  /* Soft limits defined: Clip the value  */
  if (motor_axis[axis_no].enabledHighSoftLimitPos &&
      velocity > 0 &&
      motor_axis[axis_no].MotorPosNow > motor_axis[axis_no].highSoftLimitPos) {
    fprintf(stdlog,
            "%s/%s:%d axis_no=%d CLIP soft high motorPosNow=%g highSoftLimitPos=%g\n",
            __FILE__, __FUNCTION__, __LINE__,
            axis_no,
            motor_axis[axis_no].MotorPosNow,
            motor_axis[axis_no].highSoftLimitPos);
    motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].highSoftLimitPos;
    clipped = 1;
  }
  if (motor_axis[axis_no].enabledLowSoftLimitPos &&
      velocity < 0 &&
      motor_axis[axis_no].MotorPosNow < motor_axis[axis_no].lowSoftLimitPos) {
    fprintf(stdlog,
            "%s/%s:%d axis_no=%d CLIP soft low motorPosNow=%g lowSoftLimitPos=%g\n",
            __FILE__, __FUNCTION__, __LINE__,
            axis_no,
            motor_axis[axis_no].MotorPosNow,
            motor_axis[axis_no].lowSoftLimitPos);
    motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].lowSoftLimitPos;
    clipped = 1;
  }
  if (clipped) {
    motor_axis[axis_no].moving.rampDownOnLimit = RAMPDOWNONLIMIT;
  }
  return clipped;
} /* Soft limits */


static int hard_limits_clip(int axis_no, double velocity)
{
  int clipped = 0;

  if (motor_axis[axis_no].highHardLimitPos > motor_axis[axis_no].lowHardLimitPos) {
    /* Hard limits defined: Clip the value  */
    if (motor_axis[axis_no].definedHighHardLimitPos &&
        velocity > 0 &&
        motor_axis[axis_no].MotorPosNow > motor_axis[axis_no].highHardLimitPos) {
      fprintf(stdlog,
              "%s/%s:%d axis_no=%d CLIP HLS motorPosNow=%g highHardLimitPos=%g\n",
              __FILE__, __FUNCTION__, __LINE__,
              axis_no,
              motor_axis[axis_no].MotorPosNow,
              motor_axis[axis_no].highHardLimitPos);
      motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].highHardLimitPos;
      clipped = 1;
    }
    if (motor_axis[axis_no].definedLowHardLimitPos &&
        velocity < 0 &&
        motor_axis[axis_no].MotorPosNow < motor_axis[axis_no].lowHardLimitPos) {
      fprintf(stdlog,
              "%s/%s:%d axis_no=%d CLIP LLS motorPosNow=%g lowHardLimitPos=%g\n",
              __FILE__, __FUNCTION__, __LINE__,
              axis_no,
              motor_axis[axis_no].MotorPosNow,
              motor_axis[axis_no].lowHardLimitPos);
      motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].lowHardLimitPos;
      clipped = 1;
    }
  }
  if (clipped) {
    motor_axis[axis_no].moving.rampDownOnLimit = RAMPDOWNONLIMIT;
  }
  return clipped;
}  /* Hard limits */


void setHWlowPos (int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%g\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].HWlowPos = value;
}

void setHWhighPos(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%g\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].HWhighPos = value;
}

void setHWhomeSwitchpos(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%g\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].HWhomeSwitchpos = value;
}


static void simulateMotion(int axis_no)
{
  struct timeval timeNow;
  double velocity;
  int clipped = 0;

  AXIS_CHECK_RETURN(axis_no);
  if (getManualSimulatorMode(axis_no)) return;

  velocity = getMotorVelocity(axis_no);

  if (motor_axis[axis_no].amplifierPercent < 100) {
    if (velocity) {
      /* Amplifier off, while moving */
      set_nErrorId(axis_no, 16992);
      StopInternal(axis_no);
    }
  }

  gettimeofday(&timeNow, NULL);

  if (motor_axis[axis_no].moving.velo.JogVelocity) {
    clipped = soft_limits_clip(axis_no, velocity);
    if (!clipped) {
      /* Simulate jogging  */
      motor_axis[axis_no].MotorPosNow += motor_axis[axis_no].moving.velo.JogVelocity *
        (timeNow.tv_sec - motor_axis[axis_no].lastPollTime.tv_sec);
    }
  }

  if (motor_axis[axis_no].moving.velo.PosVelocity) {
    clipped = soft_limits_clip(axis_no, velocity);
    if (!clipped) {
      /* Simulate a move to postion */
      motor_axis[axis_no].MotorPosNow += motor_axis[axis_no].moving.velo.PosVelocity *
        (timeNow.tv_sec - motor_axis[axis_no].lastPollTime.tv_sec);
      if (((motor_axis[axis_no].moving.velo.PosVelocity > 0) &&
           (motor_axis[axis_no].MotorPosNow > motor_axis[axis_no].MotorPosWanted)) ||
          ((motor_axis[axis_no].moving.velo.PosVelocity < 0) &&
           (motor_axis[axis_no].MotorPosNow < motor_axis[axis_no].MotorPosWanted))) {
        /* overshoot or undershoot. We are at the target position */
        motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].MotorPosWanted;
        motor_axis[axis_no].moving.velo.PosVelocity = 0;
      }
    }
  }
  if (motor_axis[axis_no].moving.velo.HomeVelocity) {
    /* Simulate move to home */
    motor_axis[axis_no].MotorPosNow += motor_axis[axis_no].moving.velo.HomeVelocity *
      (timeNow.tv_sec - motor_axis[axis_no].lastPollTime.tv_sec);

    if (((motor_axis[axis_no].moving.velo.HomeVelocity > 0) &&
         (motor_axis[axis_no].MotorPosNow > motor_axis[axis_no].HomeProcPos)) ||
        ((motor_axis[axis_no].moving.velo.HomeVelocity < 0) &&
         (motor_axis[axis_no].MotorPosNow < motor_axis[axis_no].HomeProcPos))) {
      /* overshoot or undershoot. We are at home */
      motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].HomeProcPos;
    }
  }
  if (motor_axis[axis_no].MotorPosNow == motor_axis[axis_no].HomeProcPos) {
    motor_axis[axis_no].moving.velo.HomeVelocity = 0;
    motor_axis[axis_no].homed = 1;
  }

  motor_axis[axis_no].lastPollTime = timeNow;
  clipped |= hard_limits_clip(axis_no, velocity);

  /* Compare moving to see if there is anything new */
  if (memcmp(&motor_axis_last[axis_no].moving, &motor_axis[axis_no].moving, sizeof(motor_axis[axis_no].moving)) ||
      motor_axis_last[axis_no].MotorPosNow     != motor_axis[axis_no].MotorPosNow ||
      motor_axis_last[axis_no].MotorPosWanted  != motor_axis[axis_no].MotorPosWanted ||
      clipped) {
    fprintf(stdlog,
            "%s/%s:%d axis_no=%d vel=%g MotorPosWanted=%g JogVel=%g PosVel=%g HomeVel=%g RampDown=%d home=%d motorPosNow=%g\n",
            __FILE__, __FUNCTION__, __LINE__,
            axis_no,
            velocity,
            motor_axis[axis_no].MotorPosWanted,
            motor_axis[axis_no].moving.velo.JogVelocity,
            motor_axis[axis_no].moving.velo.PosVelocity,
            motor_axis[axis_no].moving.velo.HomeVelocity,
            motor_axis[axis_no].moving.rampDownOnLimit,
            getAxisHome(axis_no),
            motor_axis[axis_no].MotorPosNow);
    memcpy(&motor_axis_last[axis_no], &motor_axis[axis_no], sizeof(motor_axis[axis_no]));
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

double getMotorPos(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  simulateMotion(axis_no);
  /* simulate EncoderPos */
  motor_axis[axis_no].EncoderPos = getEncoderPosFromMotorPos(axis_no, motor_axis[axis_no].MotorPosNow);
  return motor_axis[axis_no].MotorPosNow;
}

void setMotorPos(int axis_no, double value)
{
  AXIS_CHECK_RETURN(axis_no);
  StopInternal(axis_no);
  fprintf(stdlog, "%s/%s:%d axis_no=%d value=%g\n",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no, value);
  /* simulate EncoderPos */
  motor_axis[axis_no].MotorPosNow = value;
  motor_axis[axis_no].EncoderPos = getEncoderPosFromMotorPos(axis_no, motor_axis[axis_no].MotorPosNow);
}

double getEncoderPos(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  (void)getMotorPos(axis_no);
  if (motor_axis_reported[axis_no].EncoderPos != motor_axis[axis_no].EncoderPos) {
    fprintf(stdlog, "%s/%s:%d axis_no=%d EncoderPos=%g\n",
            __FILE__, __FUNCTION__, __LINE__,
            axis_no,
            motor_axis[axis_no].EncoderPos);
    motor_axis_reported[axis_no].EncoderPos = motor_axis[axis_no].EncoderPos;
  }
  return motor_axis[axis_no].EncoderPos;
}

/* Stop the ongoing motion (like JOG),
   to be able to start a new one (like HOME)
*/
void StopInternal_fl(int axis_no, const char *file, int line_no)
{
  unsigned int rampDownOnLimit;
  rampDownOnLimit = motor_axis[axis_no].moving.rampDownOnLimit;

  fprintf(stdlog, "%s/%s:%d axis_no=%d rampDownOnLimit=%d file=%s line_no=%d\n",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no, rampDownOnLimit, file, line_no);
  AXIS_CHECK_RETURN(axis_no);
  memset(&motor_axis[axis_no].moving.velo, 0,
         sizeof(motor_axis[axis_no].moving.velo));
  /* Restore the ramp down */
  motor_axis[axis_no].moving.rampDownOnLimit = rampDownOnLimit;
}


/* caput pv.VAL */
int movePosition(int axis_no,
                 double position,
                 int relative,
                 double max_velocity,
                 double acceleration)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  if (motor_axis[axis_no].logFile) {
    if (relative) {
      fprintf(motor_axis[axis_no].logFile,
              "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g\n",
              position, max_velocity, acceleration,
              motor_axis[axis_no].MotorPosNow);
    } else {
      fprintf(motor_axis[axis_no].logFile,
              "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g\n",
              position, max_velocity, acceleration,
              motor_axis[axis_no].MotorPosNow);
    }
    fflush(motor_axis[axis_no].logFile);
  }

  fprintf(stdlog, "%s%s/%s:%d axis_no=%d relative=%d position=%g max_velocity=%g acceleration=%g motorPosNow=%g\n",
          motor_axis[axis_no].logFile ? "LLLL " : "",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no,
          relative,
          position,
          max_velocity,
          acceleration,
          motor_axis[axis_no].MotorPosNow);
  StopInternal(axis_no);
  gettimeofday(&motor_axis[axis_no].lastPollTime, NULL);

  if (relative) {
    position += motor_axis[axis_no].MotorPosNow;
  }
  motor_axis[axis_no].MotorPosWanted = position;

  if (position > motor_axis[axis_no].MotorPosNow) {
    motor_axis[axis_no].moving.velo.PosVelocity = max_velocity;
  } else if (position < motor_axis[axis_no].MotorPosNow) {
    motor_axis[axis_no].moving.velo.PosVelocity = -max_velocity;
  } else {
    motor_axis[axis_no].moving.velo.PosVelocity = 0;
  }

  return 0;
}


int moveHomeProc(int axis_no,
                 int direction,
                 int nCmdData,
                 double max_velocity,
                 double acceleration)
{
  double position;
  double velocity = max_velocity ? max_velocity : motor_axis[axis_no].MaxHomeVelocityAbs;
  velocity = fabs(velocity);
  if (motor_axis[axis_no].logFile) {
    fprintf(motor_axis[axis_no].logFile,
            "moveHomeProc axis_no=%d nCmdData=%d max_velocity=%g "
            "velocity=%g acceleration=%g motorPosNow=%g\n",
            axis_no,
            nCmdData,
            max_velocity,
            velocity,
            acceleration,
            motor_axis[axis_no].MotorPosNow);
    fflush(motor_axis[axis_no].logFile);
  }
  fprintf(stdlog, "%s%s/%s:%d axis_no=%d nCmdData=%d max_velocity=%g velocity=%g acceleration=%g\n",
          motor_axis[axis_no].logFile ? "LLLL " : "",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no,
          nCmdData,
          max_velocity,
          velocity,
          acceleration);

  recalculate_pos(axis_no, nCmdData);
  position = motor_axis[axis_no].HomeProcPos;
  switch (nCmdData) {
    case ProcHom_LOW_LS:
      if (!motor_axis[axis_no].definedLowHardLimitPos)
        return -1;
      motor_axis[axis_no].HomeProcPos = motor_axis[axis_no].lowHardLimitPos;
      break;
    case ProcHom_HIGH_LS:
      if (!motor_axis[axis_no].definedHighHardLimitPos)
        return -1;
      motor_axis[axis_no].HomeProcPos =
        motor_axis[axis_no].highHardLimitPos;
      break;
    case ProcHom_LOW_HS:
    case ProcHom_HIGH_HS:
      motor_axis[axis_no].HomeProcPos = motor_axis[axis_no].HomeSwitchPos;
      break;
    default:
      return -1;
  }
  position = motor_axis[axis_no].HomeProcPos;

  if (motor_axis[axis_no].MaxHomeVelocityAbs &&
      (fabs(velocity) > motor_axis[axis_no].MaxHomeVelocityAbs)) {
    velocity = motor_axis[axis_no].MaxHomeVelocityAbs;
  }
  motor_axis[axis_no].HomeVelocityAbsWanted = velocity;
  fprintf(stdlog, "%s/%s:%d axis_no=%d direction=%d max_velocity=%g velocity=%g acceleration=%g\n",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no,
          direction,
          max_velocity,
          velocity,
          acceleration);
  StopInternal(axis_no);
  motor_axis[axis_no].homed = 0; /* Not homed any more */
  gettimeofday(&motor_axis[axis_no].lastPollTime, NULL);

  if (position > motor_axis[axis_no].MotorPosNow) {
    motor_axis[axis_no].moving.velo.HomeVelocity = velocity;
  } else if (position < motor_axis[axis_no].MotorPosNow) {
    motor_axis[axis_no].moving.velo.HomeVelocity = -velocity;
  } else {
    motor_axis[axis_no].moving.velo.HomeVelocity = 0;
    motor_axis[axis_no].homed = 1; /* homed again */
  }

  return 0;
};

 /* caput pv.HOMF, caput pv.HOMR */
int moveHome(int axis_no,
             int direction,
             double max_velocity,
             double acceleration)
{
  return moveHomeProc(axis_no, direction,
                      ProcHom_LOW_HS, /* int nCmdData, */
                      max_velocity,acceleration);
}


/* caput pv.JOGF, caput pv.JOGR */
int moveVelocity(int axis_no,
                 int direction,
                 double max_velocity,
                 double acceleration)
{
  double velocity = max_velocity;
  if (!direction) {
    velocity = - velocity;
  }

  StopInternal(axis_no);

  if (motor_axis[axis_no].logFile) {
    fprintf(motor_axis[axis_no].logFile,
            "move velocity axis_no=%d direction=%d max_velocity=%g "
            "acceleration=%g motorPosNow=%g\n",
            axis_no,
            direction,
            max_velocity,
            acceleration,
            motor_axis[axis_no].MotorPosNow);
    fflush(motor_axis[axis_no].logFile);
  }
  fprintf(stdlog, "%s%s/%s:%d axis_no=%d direction=%d max_velocity=%g acceleration=%g\n",
          motor_axis[axis_no].logFile ? "LLLL " : "",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no,
          direction,
          max_velocity,
          acceleration);
  if (direction < 0) {
    velocity = -velocity;
  }
  motor_axis[axis_no].moving.velo.JogVelocity = velocity;
  return 0;
};



int setAmplifierPercent(int axis_no, int percent)
{
  fprintf(stdlog, "%s/%s:%d axis_no=%d percent=%d\n",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no, percent);
  AXIS_CHECK_RETURN_ERROR(axis_no);
  if (percent < 0 || percent > 100) return -1;
  motor_axis[axis_no].amplifierPercent = percent;
  return 0;
}

int getAmplifierOn(int axis_no)
{
  if (motor_axis[axis_no].amplifierPercent == 100) return 1;
  return 0;
}


void getAxisDebugInfoData(int axis_no, char *buf, size_t maxlen)
{
  snprintf(buf, maxlen,
           "rvel=%g VAL=%g JVEL=%g VELO=%g HVEL=%g athome=%d RBV=%g",
           getMotorVelocity(axis_no),
           motor_axis[axis_no].MotorPosWanted,
           motor_axis[axis_no].moving.velo.JogVelocity,
           motor_axis[axis_no].moving.velo.PosVelocity,
           motor_axis[axis_no].moving.velo.HomeVelocity,
           getAxisHome(axis_no),
           motor_axis[axis_no].MotorPosNow);
}

int getNegLimitSwitch(int axis_no)
{
  int clipped =
    motor_axis[axis_no].definedLowHardLimitPos &&
    (motor_axis[axis_no].MotorPosNow <= motor_axis[axis_no].lowHardLimitPos);

  if (motor_axis_reported[axis_no].moving.hitNegLimitSwitch != motor_axis[axis_no].moving.hitNegLimitSwitch) {
    fprintf(stdlog, "%s/%s:%d axis_no=%d definedLowHardLimitPos=%d motorPosNow=%g lowHardLimitPos=%g hitNegLimitSwitch=%d\n",
            __FILE__, __FUNCTION__, __LINE__,
            axis_no,
            motor_axis[axis_no].definedLowHardLimitPos,
            motor_axis[axis_no].MotorPosNow,
            motor_axis[axis_no].lowHardLimitPos,
            motor_axis[axis_no].moving.hitNegLimitSwitch);
    motor_axis_reported[axis_no].moving.hitNegLimitSwitch = motor_axis[axis_no].moving.hitNegLimitSwitch;
    if (clipped) {
      motor_axis[axis_no].moving.rampDownOnLimit = RAMPDOWNONLIMIT;
    }
  }
  motor_axis[axis_no].moving.hitNegLimitSwitch = clipped;
  return clipped;
}

int getPosLimitSwitch(int axis_no)
{
  int clipped =
    (motor_axis[axis_no].MotorPosNow >= motor_axis[axis_no].highHardLimitPos);

  if (motor_axis_reported[axis_no].moving.hitPosLimitSwitch != motor_axis[axis_no].moving.hitPosLimitSwitch) {
    fprintf(stdlog, "%s/%s:%d axis_no=%d definedHighHardLimitPos=%d motorPosNow=%g highHardLimitPos=%g hitPosLimitSwitch=%d\n",
            __FILE__, __FUNCTION__, __LINE__,
            axis_no,
            motor_axis[axis_no].definedHighHardLimitPos,
            motor_axis[axis_no].MotorPosNow,
            motor_axis[axis_no].highHardLimitPos,
            motor_axis[axis_no].moving.hitPosLimitSwitch);
    motor_axis_reported[axis_no].moving.hitPosLimitSwitch = motor_axis[axis_no].moving.hitPosLimitSwitch;
    if (clipped) {
      motor_axis[axis_no].moving.rampDownOnLimit = RAMPDOWNONLIMIT;
    }
  }
  motor_axis[axis_no].definedHighHardLimitPos = clipped;
  return clipped;
}

int get_bError(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].nErrorId ? 1 : 0;
}

int get_nErrorId(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].nErrorId;
}

int set_nErrorId(int axis_no, int value)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  motor_axis[axis_no].nErrorId = value;
  return 0;
}

/*
 *  Debug logfile.
 */
int openLogFile(int axis_no, const char *filename)
{
  AXIS_CHECK_RETURN_EINVAL(axis_no);
  fprintf(stdlog, "LLLL %s/%s:%d axis_no=%d filename=%s\n",
            __FILE__, __FUNCTION__, __LINE__,
          axis_no, filename);
  motor_axis[axis_no].logFile = fopen(filename, "w+");
  if (!motor_axis[axis_no].logFile) return errno;

  return 0;
}

void closeLogFile(int axis_no)
{
  fprintf(stdlog, "LLLL %s/%s:%d axis_no=%d\n",
            __FILE__, __FUNCTION__, __LINE__,
          axis_no);

  AXIS_CHECK_RETURN(axis_no);
  if (motor_axis[axis_no].logFile) {
    fclose(motor_axis[axis_no].logFile);
    motor_axis[axis_no].logFile = NULL;
  }
}

int getManualSimulatorMode(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].bManualSimulatorMode;
}

void setManualSimulatorMode(int axis_no, int manualMode)
{
  AXIS_CHECK_RETURN(axis_no);
  fprintf(stdlog, "%s/%s:%d axis_no=%d manualMode=%d\n",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no, manualMode);
  if (motor_axis[axis_no].bManualSimulatorMode && !manualMode) {
    /* Manual mode switched off, stop to prevent the motor to
       start moving */
    StopInternal(axis_no);
  }
  motor_axis[axis_no].bManualSimulatorMode = manualMode;
}


int getAmplifierLockedToBeOff(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].amplifierLockedToBeOff;
}

void setAmplifierLockedToBeOff(int axis_no, int value)
{
  AXIS_CHECK_RETURN(axis_no);
  fprintf(stdlog, "%s%s/%s:%d axis_no=%d value=%d\n",
          motor_axis[axis_no].logFile ? "LLLL " : "",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no, value);
  motor_axis[axis_no].amplifierLockedToBeOff = value;
}


