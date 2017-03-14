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
static const char * const dbgOpenLogFile_equals_str = "dbgOpenLogFile=";
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

  /* dbgOpenLogFile= */
  if (!strncmp(myarg_1, dbgOpenLogFile_equals_str, strlen(dbgOpenLogFile_equals_str))) {
    int ret;
    myarg_1 += strlen(dbgOpenLogFile_equals_str);
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


  /* if we come here, we do not understand the command */
  RETURN_OR_DIE("%s/%s:%d line=%s",
                __FILE__, __FUNCTION__, __LINE__,
                myarg);
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
