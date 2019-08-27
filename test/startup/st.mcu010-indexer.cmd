require EthercatMC,USER

epicsEnvSet("MOTOR_PORT",    "$(SM_MOTOR_PORT=MCU1)")

epicsEnvSet("IPADDR",        "$(SM_IPADDR=127.0.0.1)")
epicsEnvSet("IPPORT",        "$(SM_IPPORT=5000)")
epicsEnvSet("ASYN_PORT",     "$(SM_ASYN_PORT=MC_CPU1)")
epicsEnvSet("PREFIX",        "$(SM_PREFIX=IOC:)")
epicsEnvSet("PREC",          "$(SM_PREC=3)")
epicsEnvSet("SM_NOAXES",     "3")
epicsEnvSet("ADSPORT",       "$(ECM_ADSPORT=851)")
epicsEnvSet("ECM_OPTIONS",   "adsPort=$(ADSPORT);amsNetIdRemote=5.40.216.206.1.1;amsNetIdLocal=192.168.209.71.1.1"

< EthercatMCController.cmd


#
# Axis 1
#
epicsEnvSet("AXISCONFIG",    "")
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m1)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=1)")
epicsEnvSet("DESC",          "$(SM_DESC=DESC)")
epicsEnvSet("EGU",           "$(SM_EGU=EGU)")
< EthercatMCIndexerAxis.cmd
< EthercatMCAxisdebug.cmd


#
# Axis 2
#
epicsEnvSet("AXISCONFIG",    "")
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m2)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=2)")
< EthercatMCIndexerAxis.cmd
< EthercatMCAxisdebug.cmd
