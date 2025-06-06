#ifndef MOTOR_H
#define MOTOR_H

#include <errno.h>
#include <max_axes.h>
#include <stddef.h>
#define AXIS_CHECK_RETURN(_axis)                         \
  {                                                      \
    init_axis(_axis);                                    \
    if (((_axis) <= 0) || ((_axis) >= MAX_AXES)) return; \
  }
#define AXIS_CHECK_RETURN_ZERO(_axis)                      \
  {                                                        \
    init_axis(_axis);                                      \
    if (((_axis) <= 0) || ((_axis) >= MAX_AXES)) return 0; \
  }
#define AXIS_CHECK_RETURN_ERROR(_axis)                        \
  {                                                           \
    init_axis(_axis);                                         \
    if (((_axis) <= 0) || ((_axis) >= MAX_AXES)) return (-1); \
  }
#define AXIS_CHECK_RETURN_EINVAL(_axis)                           \
  {                                                               \
    init_axis(_axis);                                             \
    if (((_axis) <= 0) || ((_axis) >= MAX_AXES)) return (EINVAL); \
  }

typedef struct motor_init_values {
  double ReverseERES;
  double ParkingPos;
  double MaxHomeVelocityAbs;
  double lowHardLimitPos;
  double highHardLimitPos;
  double hWlowPos;
  double hWhighPos;
  double homeSwitchPos;
  int defRampUpAfterStart;
} motor_init_values;

int getAxisDone(int axis_no);
int getAxisHome(int axis_no);
int getAxisHomed(int axis_no);

void setAxisHomed_fl(int axis_no, int value, const char *file, int line_no);
#define setAxisHomed(a, b) setAxisHomed_fl(a, b, __FILE__, __LINE__);

static void init_axis(int);
void hw_motor_init_fl(const char *file, int line_no, int axis_no,
                      const struct motor_init_values *pMotor_init_values,
                      size_t motor_init_len);
#define hw_motor_init(a, b, c) hw_motor_init_fl(__FILE__, __LINE__, a, b, c)

unsigned getCabinetStatus(void);
void setCabinetStatus(unsigned);

double getMaxVelocity(int axis_no);
void setMaxVelocity(int axis_no, double value);
double getMaxAcceleration(int axis_no);
void setMaxAcceleration(int axis_no, double value);
double getNxtMoveAcceleration(int axis_no);
void setNxtMoveAcceleration(int axis_no, double value);
double getNxtMoveVelocity(int axis_no);
void setNxtMoveVelocity(int axis_no, double value);
/* Where does the motor wake up after power-on */
void setMotorParkingPosition(int axis_no, double value);
double getHomePos(int axis_no);
void setHomePos(int axis_no, double value);
void setMaxHomeVelocityAbs(int axis_no, double value);
void setMotorReverseERES(int axis_no, double value);

double getMotorVelocity(int axis_no);
int isMotorHoming(int axis_no);
int isMotorMoving(int axis_no);

void setHWlowPos(int axis_no, double value);
void setHWhighPos(int axis_no, double value);
void setHWhomeSwitchpos(int axis_no, double value);

double getLowSoftLimitPos(int axis_no);
void setLowSoftLimitPos(int axis_no, double value);
int getEnableLowSoftLimit(int axis_no);
void setEnableLowSoftLimit(int axis_no, int value);
double getLowHardLimitPos(int axis_no);
void setLowHardLimitPos(int axis_no, double value);

double getHighSoftLimitPos(int axis_no);
void setHighSoftLimitPos(int axis_no, double value);
int getEnableHighSoftLimit(int axis_no);
void setEnableHighSoftLimit(int axis_no, int value);
double getHighHardLimitPos(int axis_no);
void setHighHardLimitPos(int axis_no, double value);

double getMRES_23(int axis_no);
int setMRES_23(int axis_no, double value);
double getMRES_24(int axis_no);
int setMRES_24(int axis_no, double value);

double getMotorPos(int axis_no);
void setPosHome_fl(int axis_no, double value, const char *file, int line_no);
#define setPosHome(a, b) setPosHome_fl(a, b, __FILE__, __LINE__)
void simFastMove_fl(int axis_no, double value, const char *file, int line_no);
#define simFastMove(a, b) simFastMove_fl(a, b, __FILE__, __LINE__)

void setPositionJitter(int axis_no, double value);

double getEncoderPos(int axis_no);

int getNegLimitSwitch(int axis_no);
int getPosLimitSwitch(int axis_no);
int get_bError(int axis_no);
int set_bError(int axis_no, int value);
int get_nErrorId(int axis_no);
int set_nErrorId(int axis_no, int value);
unsigned get_nStatReasAUX(int axis_no);
int set_nStatReasAUX(int axis_no, unsigned value);

/*
 * Movements
 */

int movePosition(int axis_no, double position, int relative,
                 double max_velocity, double acceleration);

/*
 *  moveHome
 *  move until the home switch is hit
 *
 *  direction:    either <0 or >=0, from which direction the switch
 *                should be reached (Which means that we may run over
 *                the switch and return from the other side with min_velocity
 *  nCmdData      The homig procedure as described in a separate document
 *  max_velocity: >0 velocity after acceleration has been done
 *  acceleration: time in seconds to reach max_velocity
 *
 *  return value: 0 == OK,
 *                error codes and error handling needs to be defined
 */
int moveHomeProc(int axis_no, int direction, int nCmdData, double position,
                 double max_velocity, double acceleration);

/*
 *  moveHome
 *  move until the home switch is hit
 *
 *  direction:    either <0 or >=0, from which direction the switch
 *                should be reached (Which means that we may run over
 *                the switch and return from the other side with min_velocity
 *  max_velocity: >0 velocity after acceleration has been done
 *  acceleration: time in seconds to reach max_velocity
 *
 *  return value: 0 == OK,
 *                error codes and error handling needs to be defined
 */
int moveHome(int axis_no, int direction, double max_velocity,
             double acceleration);

/*
 *  moveAbsolute: Move to absolute position
 *  limit switches shoud be obeyed (I think)
 *
 *  direction:    either <0 or >=0
 *  max_velocity: >0 velocity after acceleration has been done
 *  acceleration: time in seconds to reach max_velocity
 *
 *  return value: 0 == OK,
 *                error codes and error handling needs to be defined
 */
int moveAbsolute(int axis_no, double position, double max_velocity,
                 double acceleration);

/*
 *  moveRelative: Move to relative position
 *  limit switches shoud be obeyed (I think)
 *
 *  postitin      relative position
 *
 *  return value: 0 == OK,
 *                error codes and error handling needs to be defined
 */
int moveRelative(int axis_no, double position);

/*
 *  moveVelocity: Same as JOG
 *  limit switches shoud be obeyed (I think)
 *
 *  axis_no       1..max
 *  direction:    either <0 or >=0
 *  max_velocity: >0 velocity after acceleration has been done
 *  acceleration: time in seconds to reach max_velocity
 *
 *  return value: 0 == OK,
 *                error codes and error handling needs to be defined
 */
int moveVelocity(int axis_no, int direction, double max_velocity,
                 double acceleration);

/*
 *  stop
 *  axis_no       1..max
 *
 *  return value: 0 == OK,
 *                error codes and error handling needs to be defined
 */
void StopInternal_fl(int axis_no, const char *file, int line_no);
#define StopInternal(a) StopInternal_fl((a), __FILE__, __LINE__);
#define motorStop(a) StopInternal_fl((a), __FILE__, __LINE__);

/*
 *  amplifier power in percent 0..100
 *  axis_no       1..max
 *
 *  return value: 0 == OK,
 *                error codes and error handling needs to be defined
 */
int setAmplifierPercent(int axis_no, int percent);

/*
 *  amplifier     on
 *  axis_no       1..max
 *
 *  return value: 1 == ON, 100%
 *                0 otherwise
 */
int getAmplifierOn(int axis_no);

/*
 *  Debug info.
 */
void getAxisDebugInfoData(int axis_no, char *buf, size_t maxlen);

/*
 *  Debug logfile.
 */
int openLogFile(int axis_no, const char *filename);
int closeLogFile(int axis_no);

/*
 * Make it possible to disable the simulator
 * All data must be feed in by hand in e.g. cmd_EAT.c
 */
int getManualSimulatorMode(int axis_no);
void setManualSimulatorMode(int axis_no, int manualMode);

/*
 * localmode can be set in TwinCAT to disable all writing from EPICS
 */
int getLocalmode(int axis_no);
void setLocalmode(int axis_no, int manualMode);

/* Interlocks */
int getInterlockBwd(int axis_no);
void setInterlockBwd(int axis_no, int manualMode);
int getInterlockFwd(int axis_no);
void setInterlockFwd(int axis_no, int manualMode);

/*
 * Lock the amplifier to off (even if it should be on)
 */
#define AMPLIFIER_LOCKED_TO_BE_OFF_SILENT 1
#define AMPLIFIER_LOCKED_TO_BE_OFF_LOUD 2

int getAmplifierLockedToBeOff(int axis_no);
void setAmplifierLockedToBeOff(int axis_no, int value);

#endif /* MOTOR_H */
