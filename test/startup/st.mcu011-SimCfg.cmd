require EthercatMC,USER

epicsEnvSet("ECM_NUMAXES",   "4")
epicsEnvSet("MOTOR_PORT",    "MCU1")

epicsEnvSet("IPADDR",        "127.0.0.1")
#epicsEnvSet("IPADDR",       "192.168.88.61")
epicsEnvSet("IPPORT",        "5000")
epicsEnvSet("ASYN_PORT",     "MC_CPU1")
epicsEnvSet("PREFIX",        "IOC:")
epicsEnvSet("EGU",           "mm")
epicsEnvSet("PREC",          "3")
< EthercatMCController.cmd

epicsEnvSet("SLIT",          "MC-SltH-01:")

epicsEnvSet("AXISCONFIG",    "HomProc=0;cfgFile=./mcu011-sim-1.cfg")
epicsEnvSet("MOTOR_NAME",    "$(SLIT)xn")
#epicsEnvSet("M",             "$(SLIT)m1")
#epicsEnvSet("R",             "$(SLIT)m1-")
epicsEnvSet("AXIS_NO",       "1")
epicsEnvSet("DESC",          "H_NEG")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "HomProc=0;cfgFile=./mcu011-sim-2.cfg")
epicsEnvSet("MOTOR_NAME",    "$(SLIT)xp")
#epicsEnvSet("M",             "$(SLIT)m2")
#epicsEnvSet("R",             "$(SLIT)m2-")
epicsEnvSet("AXIS_NO",       "2")
epicsEnvSet("DESC",          "H_POS")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "HomProc=0")
epicsEnvSet("MOTOR_NAME",    "$(SLIT)Center")
#epicsEnvSet("M",             "$(SLIT)m3")
#epicsEnvSet("R",             "$(SLIT)m3-")
epicsEnvSet("AXIS_NO",       "3")
epicsEnvSet("DESC",          "H_CENTER")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "HomProc=0")
epicsEnvSet("MOTOR_NAME",    "$(SLIT)Gap")
#epicsEnvSet("M",             "$(SLIT)m4")
#epicsEnvSet("R",             "$(SLIT)m4-")
epicsEnvSet("AXIS_NO",       "4")
epicsEnvSet("DESC",          "H_GAP")
< EthercatMCAxis.cmd

## Logical axes and slit
epicsEnvSet("mXp",           "$(SLIT)xp")
epicsEnvSet("mXn",           "$(SLIT)xn")
## Soft limits for the Slit
< EthercatMCslitAvoidCollSoftlimits.cmd
#########################
