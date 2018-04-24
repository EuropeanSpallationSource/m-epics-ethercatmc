require asyn,4.31
require motor,USER
require EthercatMC,USER

epicsEnvSet("MOTOR_PORT",    "$(SM_MOTOR_PORT=MCU1)")

epicsEnvSet("IPADDR",        "$(SM_IPADDR=127.0.0.1)")
epicsEnvSet("IPPORT",        "$(SM_IPPORT=5000)")
epicsEnvSet("ASYN_PORT",     "$(SM_ASYN_PORT=MC_CPU1)")
epicsEnvSet("PREFIX",        "$(SM_PREFIX=HZB-V20:)")
epicsEnvSet("P",             "$(SM_PREFIX=HZB-V20:)")
< EthercatMCController.cmd

# Common to all axes
epicsEnvSet("EGU",           "$(SM_EGU=mm)")
epicsEnvSet("PREC",          "$(SM_PREC=3)")

# Real motors and slit systems
#########################
epicsEnvSet("AXISCONFIG",    "cfgFile=./SlitAxisMCU010-1.cfg")
epicsEnvSet("MOTOR_NAME",    "MC-MCU-01:m1")
epicsEnvSet("R",             "MC-MCU-01:m1-")
epicsEnvSet("AXIS_NO",       "1")
epicsEnvSet("DESC",          "V low blade")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "cfgFile=./SlitAxisMCU010-2.cfg")
epicsEnvSet("MOTOR_NAME",    "MC-MCU-01:m2")
epicsEnvSet("R",             "MC-MCU-01:m2-")
epicsEnvSet("AXIS_NO",       "2")
epicsEnvSet("DESC",          "V high Blade")
< EthercatMCAxis.cmd

## Logical axes and slit
epicsEnvSet("SLIT",          "MC-SLT-01:SltH-")
epicsEnvSet("mXp",           "MC-MCU-01:m2")
epicsEnvSet("mXn",           "MC-MCU-01:m1")

## Slit
< EthercatMC2slit.cmd
#########################
