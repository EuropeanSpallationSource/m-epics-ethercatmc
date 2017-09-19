require asyn,4.31
require axis,USER

epicsEnvSet("MOTOR_PORT",    "$(SM_MOTOR_PORT=MCU1)")

epicsEnvSet("IPADDR",        "$(SM_IPADDR=127.0.0.1)")
epicsEnvSet("IPPORT",        "$(SM_IPPORT=5000)")
epicsEnvSet("ASYN_PORT",     "$(SM_ASYN_PORT=MC_CPU1)")
epicsEnvSet("PREFIX",        "$(SM_PREFIX=IOC:)")
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m1)")
epicsEnvSet("M",             "$(SM_M=m1)")
epicsEnvSet("R",             "$(SM_R=m1-)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=1)")
epicsEnvSet("DESC",          "$(SM_DESC=motor1)")
epicsEnvSet("EGU",           "$(SM_EGU=mm)")
epicsEnvSet("PREC",          "$(SM_PREC=3)")
epicsEnvSet("VELO",          "$(SM_VELO=0.0)")
epicsEnvSet("JVEL",          "$(SM_JVEL=0)")
epicsEnvSet("JAR",           "$(SM_JAR=0)")
epicsEnvSet("ACCL",          "$(SM_ACCL=0)")
epicsEnvSet("MRES",          "$(SM_MRES=1)")
epicsEnvSet("SDBD",          "$(SM_SDBD=0)")
epicsEnvSet("RDBD",          "$(SM_RDBD=0)")
epicsEnvSet("DLLM",          "$(SM_DLLM=15)")
epicsEnvSet("DHLM",          "$(SM_DHLM=170)")

# And this reads the input of an incremental encoder terminal
# on the EtherCAT bus. Works with the simulator.
# For real terminals the adresses must be adapted
epicsEnvSet("AXISCONFIG",    "encoder=ADSPORT=501/.ADR.16#3040010,16#80000049,2,2")
epicsEnvSet("ERES",          "$(SM_ERES=-0.03)")

epicsEnvSet("HOMEPROC",      "$(SM_HOMEPROC=1)")
epicsEnvSet("HOMEPOS",       "$(SM_HOMEPOS=0.0)")
epicsEnvSet("HVELTO",        "$(SM_HVELTP=9)")
epicsEnvSet("HVELFRM",       "$(SM_HVELTP=4)")
epicsEnvSet("HOMEACC",       "$(SM_HOMEACC=24)")
epicsEnvSet("HOMEDEC",       "$(SM_HOMEDEC=26)")

< EthercatMCController.cmd
< EthercatMCAxis.cmd
