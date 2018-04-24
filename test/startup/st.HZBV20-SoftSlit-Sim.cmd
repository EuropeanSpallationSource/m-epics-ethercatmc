require asyn,4.31
require motor,USER
require EthercatMC,USER

epicsEnvSet("MOTOR_PORT",    "$(SM_MOTOR_PORT=MCU1)")

epicsEnvSet("IPADDR",        "$(SM_IPADDR=127.0.0.1)")
epicsEnvSet("IPPORT",        "$(SM_IPPORT=5000)")
epicsEnvSet("ASYN_PORT",     "$(SM_ASYN_PORT=MC_CPU1)")
#epicsEnvSet("P",             "$(SM_PREFIX=HZB-V20:MC-SLT-01:)")
epicsEnvSet("PREFIX",        "$(SM_PREFIX=HZB-V20:MC-MCU-01:)")
epicsEnvSet("P",             "$(SM_PREFIX=HZB-V20:MC-MCU-01:)")
< EthercatMCController.cmd

# Common to all axes
epicsEnvSet("EGU",           "$(SM_EGU=mm)")
epicsEnvSet("PREC",          "$(SM_PREC=3)")

# Real motors and slit systems
#########################
epicsEnvSet("AXISCONFIG",    "cfgFile=./SlitAxisMCU010-1.cfg")
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m1)")
epicsEnvSet("M",             "$(SM_M=m1)")
epicsEnvSet("R",             "$(SM_R=m1-)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=1)")
epicsEnvSet("DESC",          "$(SM_DESC=V low blade)")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "cfgFile=./SlitAxisMCU010-2.cfg")
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m2)")
epicsEnvSet("M",             "$(SM_M=m2)")
epicsEnvSet("R",             "$(SM_R=m2-)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=2)")
epicsEnvSet("DESC",          "$(SM_DESC=V high Blade)")
< EthercatMCAxis.cmd

## Logical axes and slit
epicsEnvSet("SLIT",          "$(SM_SLIT=SltH-)")
epicsEnvSet("mXp",           "$(SM_mXp=m2)")
epicsEnvSet("mXn",           "$(SM_mXp=m1)")

## Slit
< EthercatMC2slit.cmd
#########################
