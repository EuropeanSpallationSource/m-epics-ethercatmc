require asyn,4.31
require EthercatMC,USER

epicsEnvSet("MOTOR_PORT",    "$(SM_MOTOR_PORT=MCU1)")

epicsEnvSet("IPADDR",        "$(SM_IPADDR=127.0.0.1)")
epicsEnvSet("IPPORT",        "$(SM_IPPORT=5000)")
epicsEnvSet("ASYN_PORT",     "$(SM_ASYN_PORT=MC_CPU1)")
epicsEnvSet("PREFIX",        "$(SM_PREFIX=HZB-V20:MC-)")
epicsEnvSet("P",             "$(PREFIX)")
epicsEnvSet("EGU",           "$(SM_EGU=mm)")
epicsEnvSet("PREC",          "$(SM_PREC=3)")
< EthercatMCController.cmd

# 2 physical axes
epicsEnvSet("M1",           "$(SM_mXp=MCU-01:m1)")
epicsEnvSet("R1",           "$(SM_mXp=MCU-01:m1-)")
epicsEnvSet("M2",           "$(SM_mXp=MCU-01:m2)")
epicsEnvSet("R2",           "$(SM_mXp=MCU-01:m1-)")

# Logical axes and slit
epicsEnvSet("SLIT",          "$(SM_SLIT=SltH-)")
epicsEnvSet("mXp",           "$(SM_mXp=MCU-01:m1)")
epicsEnvSet("mXn",           "$(SM_mXp=m2)")

epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m2)")
epicsEnvSet("M",             "$(M2)")
epicsEnvSet("R",             "$(R2)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=1)")
epicsEnvSet("DESC",          "$(SM_DESC=Lower=Right)")
# And this reads the input of an incremental encoder terminal
# on the EtherCAT bus. Works with the simulator.
# For real terminals the adresses must be adapted
epicsEnvSet("AXISCONFIG",    "stepSize=1.0;cfgFile=./SlitAxisMCU010-1.cfg;encoder=ADSPORT=501/.ADR.16#3040010,16#80000049,2,2;HomProc=1;HomPos=0.0")

< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "stepSize=1.0;cfgFile=./SlitAxisMCU010-2.cfg;encoder=ADSPORT=501/.ADR.16#3040010,16#8000004F,2,2;HomProc=2;HomPos=172")

epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m1)")
epicsEnvSet("M",             "$(M1)")
epicsEnvSet("R",             "$(R1)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=2)")
epicsEnvSet("DESC",          "$(SM_DESC=Upper=Left)")
< EthercatMCAxis.cmd


< EthercatMC2slit.cmd
#########################
