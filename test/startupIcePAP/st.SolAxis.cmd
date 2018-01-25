require asyn,4.31
require axis,USER

epicsEnvSet("MOTOR_PORT",    "$(SM_MOTOR_PORT=MCU1)")

epicsEnvSet("IPADDR",        "$(SM_IPADDR=127.0.0.1)")
epicsEnvSet("IPPORT",        "$(SM_IPPORT=5000)")
epicsEnvSet("ASYN_PORT",     "$(SM_ASYN_PORT=MC_CPU1)")
epicsEnvSet("PREFIX",        "$(SM_PREFIX=IOC:)")
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m1)")
epicsEnvSet("R",             "$(SM_R=m1-)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=1)")
epicsEnvSet("DESC",          "$(SM_DESC=motor1)")
epicsEnvSet("EGU",           "$(SM_EGU=mm)")
epicsEnvSet("PREC",          "$(SM_PREC=3)")
epicsEnvSet("VELO",          "$(SM_VELO=10.0)")
epicsEnvSet("JVEL",          "$(SM_JVEL=5.0)")
epicsEnvSet("JAR",           "$(SM_JAR=21)")
epicsEnvSet("ACCL",          "$(SM_ACCL=0.4)")
epicsEnvSet("MRES",          "$(SM_MRES=0.001)")
epicsEnvSet("RDBD",          "$(SM_RDBD=0.1)")
epicsEnvSet("NTMF",          "$(SM_NTMF=3)")
epicsEnvSet("MRES",          "$(SM_MRES=0.03)")
epicsEnvSet("DLLM",          "$(SM_DLLM=15)")
epicsEnvSet("DHLM",          "$(SM_DHLM=170)")

epicsEnvSet("HOMEPROC",      "$(SM_HOMEPROC=1)")
epicsEnvSet("HOMEPOS",       "$(SM_HOMEPOS=0.0)")
epicsEnvSet("HVELTO",        "$(SM_HVELTP=9)")
epicsEnvSet("HVELFRM",       "$(SM_HVELTP=4)")
epicsEnvSet("HOMEACC",       "$(SM_HOMEACC=24)")
epicsEnvSet("HOMEDEC",       "$(SM_HOMEDEC=26)")

#Controller
drvAsynIPPortConfigure("$(ASYN_PORT)","$(IPADDR):$(IPPORT)",0,0,0)
asynOctetSetOutputEos("$(ASYN_PORT)", -1, "\n")
asynOctetSetInputEos("$(ASYN_PORT)", -1, "\n")
IcePAPCreateController("$(MOTOR_PORT)", "$(ASYN_PORT)" "32", "200", "1000")
asynSetTraceMask("$(ASYN_PORT)", -1, 0x41)
asynSetTraceIOMask("$(ASYN_PORT)", -1, 2)
asynSetTraceInfoMask("$(ASYN_PORT)", -1, 15)

# Axis
< IcePAPAxis.cmd
