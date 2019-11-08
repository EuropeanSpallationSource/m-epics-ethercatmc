require EthercatMC,USER

epicsEnvSet("ECM_NUMAXES",   "2")
epicsEnvSet("MOTOR_PORT",    "$(SM_MOTOR_PORT=MCU1)")

epicsEnvSet("IPADDR",        "$(SM_IPADDR=172.30.242.18)")
epicsEnvSet("IPPORT",        "$(SM_IPPORT=5000)")
epicsEnvSet("ASYN_PORT",     "$(SM_ASYN_PORT=MC_CPU1)")
epicsEnvSet("PREFIX",        "$(SM_PREFIX=LabS-ESSIIP:)")
epicsEnvSet("P",             "$(SM_PREFIX=LabS-ESSIIP:)")
epicsEnvSet("EGU",           "$(SM_EGU=mm)")
epicsEnvSet("PREC",          "$(SM_PREC=3)")

epicsEnvSet("ECM_OPTIONS",          "")
# Controller
< EthercatMCController.iocsh


# Axis 1
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=MC-MCU-02:m1)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=1)")
epicsEnvSet("DESC",          "$(SM_DESC=Lower=Right)")
epicsEnvSet("AXISCONFIG",    "HomProc=1;HomPos=-63;encoder=ADSPORT=501/.ADR.16#3040010,16#80000049,2,2")


< EthercatMCAxis.iocsh
< EthercatMCAxisdebug.iocsh
< EthercatMCAxishome.iocsh


epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=MC-MCU-02:m2)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=2)")
epicsEnvSet("DESC",          "$(SM_DESC=Upper=Left)")
epicsEnvSet("AXISCONFIG",    "HomProc=2;HomPos=64;encoder=ADSPORT=501/.ADR.16#3040010,16#8000004F,2,2")

< EthercatMCAxis.iocsh
< EthercatMCAxisdebug.iocsh
< EthercatMCAxishome.iocsh

## Logical axes and slit
epicsEnvSet("SLIT",          "$(SM_SLIT=MC-SLT-01:SltH-)")
epicsEnvSet("mXp",           "$(SM_mXp=MC-MCU-02:m2)")
epicsEnvSet("mXn",           "$(SM_mXp=MC-MCU-02:m1)")

## Slit
< EthercatMC2slit.iocsh
#########################
