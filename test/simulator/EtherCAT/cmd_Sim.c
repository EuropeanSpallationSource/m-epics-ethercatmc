#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>
#include "sock-util.h"
#include "logerr_info.h"
#include "cmd_buf.h"
#include "hw_motor.h"
#include "cmd_Sim.h"

static const char * const Sim_dot_str = "Sim.";
static const char * const log_equals_str = "log=";
static const char * const dbgCloseLogFile_str = "dbgCloseLogFile";

static const char *seperator_seperator = ";";

static void init_axis(int axis_no)
{
  (void)axis_no;
}
static void motorHandleOneArg(const char *myarg_1)
{
  const char *myarg = myarg_1;
  int iValue = 0;
  double fValue = 0;
  int motor_axis_no = 0;
  int nvals = 0;
  (void)iValue;
  (void)fValue;
  (void)iValue;


  /* Sim.*/
  if (!strncmp(myarg_1, Sim_dot_str, strlen(Sim_dot_str))) {
    myarg_1 += strlen(Sim_dot_str);
  }

  /* From here on, only M1. commands */
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

  /* log= */
  if (!strncmp(myarg_1, log_equals_str, strlen(log_equals_str))) {
    int ret;
    myarg_1 += strlen(log_equals_str);
    ret = openLogFile(motor_axis_no, myarg_1);
    if (!ret)
      cmd_buf_printf("OK");
    else
      cmd_buf_printf("Error %s(%d)",
                     strerror(ret), ret);
    return;
  }
  /* dbgCloseLogFile */
  if (!strncmp(myarg_1, dbgCloseLogFile_str, strlen(dbgCloseLogFile_str))) {
    closeLogFile(motor_axis_no);
    cmd_buf_printf("OK");
    return;
  }

  /* bAmplifierLockedToBeOff? */
  if (!strcmp(myarg_1, "bAmplifierLockedToBeOff?")) {
    cmd_buf_printf("%d", getAmplifierLockedToBeOff(motor_axis_no));
    return;
  }

  /******************************************************/
  /* bAmplifierLockedToBeOff= */
  nvals = sscanf(myarg_1, "bAmplifierLockedToBeOff=%d", &iValue);
  if (nvals == 1) {
    setAmplifierLockedToBeOff(motor_axis_no, iValue);
    if (iValue) {
      setAmplifierPercent(motor_axis_no, 0);
    }
    cmd_buf_printf("OK");
    return;
  }

  /* fMotorParkingPosition=100 */
  nvals = sscanf(myarg_1, "fMotorParkingPosition=%lf", &fValue);
  if (nvals == 1) {
    setMotorParkingPosition(motor_axis_no, fValue);
    cmd_buf_printf("OK");
    return;
  }
  /* fLowHardLimitPos=15 */
  nvals = sscanf(myarg_1, "fLowHardLimitPos=%lf", &fValue);
  if (nvals == 1) {
    setLowHardLimitPos(motor_axis_no, fValue);
    cmd_buf_printf("OK");
    return;
  }
  /* fHighHardLimitPos=165 */
  nvals = sscanf(myarg_1, "fHighHardLimitPos=%lf", &fValue);
  if (nvals == 1) {
    setHighHardLimitPos(motor_axis_no, fValue);
    cmd_buf_printf("OK");
    return;
  }
  /* fHWhomeSwitchpos=30 */
  nvals = sscanf(myarg_1, "fHWhomeSwitchpos=%lf", &fValue);
  if (nvals == 1) {
    setHWhomeSwitchpos(motor_axis_no, fValue);
    cmd_buf_printf("OK");
    return;
  }

  /* fActPosition=30 */
  nvals = sscanf(myarg_1, "fActPosition=%lf", &fValue);
  if (nvals == 1) {
    //setActPosition(motor_axis_no, fValue);
    setMotorPos(motor_axis_no, fValue);
    cmd_buf_printf("OK");
    return;
  }

  /* bManualSimulatorMode=1 */
  nvals = sscanf(myarg_1, "bManualSimulatorMode=%d", &iValue);
  if (nvals == 1) {
    setManualSimulatorMode(motor_axis_no, iValue);
    cmd_buf_printf("OK");
    return;
  }
  /* bAxisHomede=1 */
  nvals = sscanf(myarg_1, "bAxisHomed=%d", &iValue);
  if (nvals == 1) {
    setAxisHomed(motor_axis_no, iValue);
    cmd_buf_printf("OK");
    return;
  }
  /* nAmplifierPercent=1 */
  nvals = sscanf(myarg_1, "nAmplifierPercent=%d", &iValue);
  if (nvals == 1) {
    setAmplifierPercent(motor_axis_no, iValue);
    cmd_buf_printf("OK");
    return;
  }
  /* if we come here, we do not understand the command */
  RETURN_OR_DIE("%s/%s:%d line=%s myarg_1=%s",
                __FILE__, __FUNCTION__, __LINE__,
                myarg, myarg_1);
}

void cmd_Sim(int argc, const char *argv[])
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
