require asyn,4.27
require axis,USER

epicsEnvSet("MOTOR_PORT",    "$(SM_MOTOR_PORT=MCU1)")

epicsEnvSet("IPADDR",        "$(SM_IPADDR=127.0.0.1)")
epicsEnvSet("IPPORT",        "$(SM_IPPORT=5000)")
epicsEnvSet("ASYN_PORT",     "$(SM_ASYN_PORT=MC_CPU1)")
#epicsEnvSet("AXISCONFIG",    "cfgFile=../iocBoot/iocEthercatMC/SolAxis-48-1.cfg")
epicsEnvSet("AXISCONFIG",    "")
epicsEnvSet("PREFIX",        "$(SM_PREFIX=IOC:)")
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m1)")
epicsEnvSet("R",             "$(SM_R=m1-)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=1)")
epicsEnvSet("DESC",          "$(SM_DESC=motor1)")
epicsEnvSet("EGU",           "$(SM_EGU=mm)")
epicsEnvSet("PREC",          "$(SM_PREC=3)")
epicsEnvSet("VELO",          "$(SM_VELO=24.9)")
epicsEnvSet("JVEL",          "$(SM_JVEL=10)")
epicsEnvSet("JAR",           "$(SM_JAR=10.2)")
epicsEnvSet("ACCL",          "$(SM_ACCL=1)")
epicsEnvSet("MRES",          "$(SM_MRES=0.0046875)")
epicsEnvSet("DLLM",          "$(SM_DLLM=15)")
epicsEnvSet("DHLM",          "$(SM_DHLM=170)")
epicsEnvSet("HOMEPROC",      "$(SM_HOMEPROC=1)")


< EthercatMCController.cmd
< EthercatMCAxis.cmd
