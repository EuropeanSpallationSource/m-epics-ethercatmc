require asyn,4.31
require ethercatmc,USER

epicsEnvSet("ECM_NUMAXES",   "10")
epicsEnvSet("MOTOR_PORT",    "$(SM_MOTOR_PORT=MCU1)")

epicsEnvSet("IPADDR",        "$(SM_IPADDR=127.0.0.1)")
epicsEnvSet("IPPORT",        "$(SM_IPPORT=5000)")
epicsEnvSet("ASYN_PORT",     "$(SM_ASYN_PORT=MC_CPU1)")
epicsEnvSet("PREFIX",        "$(SM_PREFIX=IOC:)")
epicsEnvSet("MOTOR_NAME",    "$(SM_MOTOR_NAME=m10)")
epicsEnvSet("AXIS_NO",       "$(SM_AXIS_NO=10)")
epicsEnvSet("DESC",          "$(SM_DESC=motor10)")
epicsEnvSet("EGU",           "$(SM_EGU=mm)")
epicsEnvSet("PREC",          "$(SM_PREC=3)")

# And this reads the input of an incremental encoder terminal
# on the EtherCAT bus. Works with the simulator.
# For real terminals the adresses must be adapted
#epicsEnvSet("AXISCONFIG",    "stepSize=1.0;cfgFile=./SimAxis.cfg;encoder=ADSPORT=501/.ADR.16#3040010,16#80000049,2,2;HomProc=1")
epicsEnvSet("AXISCONFIG",    "")
epicsEnvSet("ECAXISFIELDINIT", ",DVAL=11")

< ethercatmcController.iocsh
##< ethercatmcAxis.iocsh
ethercatmcCreateAxis("$(MOTOR_PORT)", "$(AXIS_NO)", "6", "$(AXISCONFIG)")
dbLoadRecords("ethercatmc.template", "PREFIX=$(PREFIX), MOTOR_NAME=$(MOTOR_NAME), R=$(MOTOR_NAME)-, MOTOR_PORT=$(MOTOR_PORT), ASYN_PORT=$(ASYN_PORT), AXIS_NO=$(AXIS_NO), DESC=$(DESC), PREC=$(PREC) $(ECAXISFIELDINIT)")
dbLoadRecords("ethercatmcreadback.template", "PREFIX=$(PREFIX), MOTOR_NAME=$(MOTOR_NAME), R=$(MOTOR_NAME)-, MOTOR_PORT=$(MOTOR_PORT), ASYN_PORT=$(ASYN_PORT), AXIS_NO=$(AXIS_NO), DESC=$(DESC), PREC=$(PREC) ")



< ethercatmcAxisdebug.iocsh

