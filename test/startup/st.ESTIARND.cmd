require asyn,4.31
require EthercatMC,USER

epicsEnvSet("ECM_NUMAXES",   "5")
epicsEnvSet("MOTOR_PORT",    "$(SM_MOTOR_PORT=MCU1)")

epicsEnvSet("IPADDR",        "$(SM_IPADDR=127.0.0.1)")
epicsEnvSet("IPPORT",        "$(SM_IPPORT=5000)")
epicsEnvSet("ASYN_PORT",     "$(SM_ASYN_PORT=MC_CPU1)")
epicsEnvSet("PREFIX",        "$(SM_PREFIX=PSI-ESTIARND:MC-MCU-01:)")
< EthercatMCController.cmd


# Common to all axes
epicsEnvSet("EGU",           "$(SM_EGU=mm)")
epicsEnvSet("PREC",          "$(SM_PREC=3)")
###############################################
epicsEnvSet("AXISCONFIG",    "HomProc=1")
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m1)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=1)")
epicsEnvSet("DESC",          "$(SM_DESC=Planetary)")

< EthercatMCAxis.cmd
#< EthercatMCAxisdebug.cmd

###############################################
epicsEnvSet("AXISCONFIG",    "HomProc=15")
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m2)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=2)")
epicsEnvSet("DESC",          "$(SM_DESC=Resolver)")
< EthercatMCAxis.cmd
#< EthercatMCAxisdebug.cmd


###############################################
epicsEnvSet("AXISCONFIG",    "HomProc=15")
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m3)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=3)")
epicsEnvSet("DESC",          "$(SM_DESC=EKSMA)")
< EthercatMCAxis.cmd
#< EthercatMCAxisdebug.cmd

###############################################
epicsEnvSet("AXISCONFIG",    "HomProc=15")
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m4)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=4)")
epicsEnvSet("DESC",          "$(SM_DESC=IGUS)")
< EthercatMCAxis.cmd
#< EthercatMCAxisdebug.cmd


###############################################
epicsEnvSet("AXISCONFIG",    "HomProc=15")
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m5)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=5)")
epicsEnvSet("DESC",          "$(SM_DESC=Phytron)")
< EthercatMCAxis.cmd
#< EthercatMCAxisdebug.cmd


