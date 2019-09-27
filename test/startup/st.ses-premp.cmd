require asyn,4.33
require EthercatMC,2.1.0

epicsEnvSet("MOTOR_PORT",    "$(SM_MOTOR_PORT=MCU1)")

epicsEnvSet("IPADDR",        "$(SM_IPADDR=172.16.0.2)")
epicsEnvSet("IPPORT",        "$(SM_IPPORT=5000)")
epicsEnvSet("ASYN_PORT",     "$(SM_ASYN_PORT=MC_CPU1)")
epicsEnvSet("PREFIX",        "$(SM_PREFIX=SES-PREMP:MC-MCU-01:)")
epicsEnvSet("EGU",           "$(SM_EGU=mm)")
epicsEnvSet("PREC",          "$(SM_PREC=3)")
< EthercatMCController.iocsh

epicsEnvSet("AXISCONFIG",    "HomProc=1;HomPos=0")
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m1)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=1)")
epicsEnvSet("DESC",          "$(SM_DESC=motor1)")
< EthercatMCAxis.iocsh
< EthercatMCAxisdebug.iocsh

epicsEnvSet("AXISCONFIG",    "HomProc=1;HomPos=-16.75")
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m2)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=2)")
epicsEnvSet("DESC",          "$(SM_DESC=motor2)")
< EthercatMCAxis.iocsh
< EthercatMCAxisdebug.iocsh

epicsEnvSet("AXISCONFIG",    "HomProc=1;HomPos=0")
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m3)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=3)")
epicsEnvSet("DESC",          "$(SM_DESC=motor3)")
< EthercatMCAxis.iocsh
< EthercatMCAxisdebug.iocsh

