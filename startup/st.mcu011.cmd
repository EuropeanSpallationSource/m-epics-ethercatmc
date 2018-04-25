require asyn,4.31
require EthercatMC,USER

epicsEnvSet("MOTOR_PORT",    "$(SM_MOTOR_PORT=MCU1)")

epicsEnvSet("IPADDR",        "$(SM_IPADDR=127.0.0.1)")
#epicsEnvSet("IPADDR",        "$(SM_IPADDR=192.168.88.61")
epicsEnvSet("IPPORT",        "$(SM_IPPORT=5000)")
epicsEnvSet("ASYN_PORT",     "$(SM_ASYN_PORT=MC_CPU1)")
epicsEnvSet("PREFIX",        "$(SM_PREFIX=IOC:)")
epicsEnvSet("EGU",           "$(SM_EGU=mm)")
epicsEnvSet("PREC",          "$(SM_PREC=3)")

# And this reads the input of an incremental encoder terminal
# on the EtherCAT bus. Works with the simulator.
# For real terminals the adresses must be adapted
#epicsEnvSet("AXISCONFIG",    "encoder=ADSPORT=501/.ADR.16#3040010,16#80000049,2,2;HomProc=0")
epicsEnvSet("AXISCONFIG",    "HomProc=0")

< EthercatMCController.cmd


epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m1)")
epicsEnvSet("M",             "$(SM_M=m1)")
epicsEnvSet("R",             "$(SM_R=m1-)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=1)")
epicsEnvSet("DESC",          "$(SM_DESC=Y_NEG)")
< EthercatMCAxis.cmd

epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m2)")
epicsEnvSet("M",             "$(SM_M=m2)")
epicsEnvSet("R",             "$(SM_R=m2-)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=2)")
epicsEnvSet("DESC",          "$(SM_DESC=Y_POS)")
< EthercatMCAxis.cmd

epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m3)")
epicsEnvSet("M",             "$(SM_M=m3)")
epicsEnvSet("R",             "$(SM_R=m3-)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=3)")
epicsEnvSet("DESC",          "$(SM_DESC=Y_CENTER)")
< EthercatMCAxis.cmd

epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m4)")
epicsEnvSet("M",             "$(SM_M=m4)")
epicsEnvSet("R",             "$(SM_R=m4-)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=4)")
epicsEnvSet("DESC",          "$(SM_DESC=Y_GAP)")
< EthercatMCAxis.cmd



