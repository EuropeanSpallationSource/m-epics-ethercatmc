require asyn,4.31
require motor,6.9.3-ESS
require EthercatMC,USER

epicsEnvSet("MOTOR_PORT",    "$(SM_MOTOR_PORT=MCU1)")

epicsEnvSet("IPADDR",        "$(SM_IPADDR=127.0.0.1)")
epicsEnvSet("IPPORT",        "$(SM_IPPORT=5000)")
epicsEnvSet("ASYN_PORT",     "$(SM_ASYN_PORT=MC_CPU1)")
epicsEnvSet("PREFIX",        "$(SM_PREFIX=HZB-V20:)")
epicsEnvSet("P",             "$(SM_PREFIX=HZB-V20:)")
< EthercatMCController.cmd

# Common to all axes
epicsEnvSet("EGU",           "$(SM_EGU=mm)")
epicsEnvSet("PREC",          "$(SM_PREC=3)")

# Real motors and slit systems
#########################
epicsEnvSet("AXISCONFIG",    "stepSize=1.0;HomProc=0")
epicsEnvSet("MOTOR_NAME",    "MC-SLT-01:SltH-Center")
epicsEnvSet("AXIS_NO",       "1")
epicsEnvSet("DESC",          "H Center")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "stepSize=1.0;HomProc=0")
epicsEnvSet("MOTOR_NAME",    "MC-SLT-01:SltH-Gap")
epicsEnvSet("AXIS_NO",       "2")
epicsEnvSet("DESC",          "H Gap")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "stepSize=1.0;HomProc=0")
epicsEnvSet("MOTOR_NAME",    "MC-SLT-01:SltH-xp")
epicsEnvSet("AXIS_NO",       "3")
epicsEnvSet("DESC",          "H Pos Blade")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "stepSize=1.0;HomProc=0")
epicsEnvSet("MOTOR_NAME",    "MC-SLT-01:SltH-xn")
epicsEnvSet("AXIS_NO",       "4")
epicsEnvSet("DESC",          "H Neg Blade")
< EthercatMCAxis.cmd

#########################
## Logical axes and slit
epicsEnvSet("SLIT",          "MC-SLT-01:SltH-")
epicsEnvSet("mXc",           "Center")
epicsEnvSet("mXg",           "Gap")
epicsEnvSet("mXp",           "MC-SLT-01:SltH-xp")
epicsEnvSet("mXn",           "MC-SLT-01:SltH-xn")

## Slit
< EthercatMCslitAvoidCollSoftlimits.cmd
#########################

# Real motors and slit systems
#########################
epicsEnvSet("AXISCONFIG",    "stepSize=1.0;HomProc=0")
epicsEnvSet("MOTOR_NAME",    "MC-SLT-01:SltV-Center")
epicsEnvSet("AXIS_NO",       "5")
epicsEnvSet("DESC",          "V Center")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "stepSize=1.0;HomProc=0")
epicsEnvSet("MOTOR_NAME",    "MC-SLT-01:SltV-Gap")
epicsEnvSet("AXIS_NO",       "6")
epicsEnvSet("DESC",          "V Gap")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "stepSize=1.0;HomProc=0")
epicsEnvSet("MOTOR_NAME",    "MC-SLT-01:SltV-xp")
epicsEnvSet("AXIS_NO",       "7")
epicsEnvSet("DESC",          "V Pos Blade")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "stepSize=1.0;HomProc=0")
epicsEnvSet("MOTOR_NAME",    "MC-SLT-01:SltV-xn")
epicsEnvSet("AXIS_NO",       "8")
epicsEnvSet("DESC",          "V Neg Blade")
< EthercatMCAxis.cmd

#########################
## Logical axes and slit
epicsEnvSet("SLIT",          "MC-SLT-01:SltV-")
epicsEnvSet("mXc",           "Center")
epicsEnvSet("mXg",           "Gap")
epicsEnvSet("mXp",           "MC-SLT-01:SltV-xp")
epicsEnvSet("mXn",           "MC-SLT-01:SltV-xn")

## Slit
< EthercatMCslitAvoidCollSoftlimits.cmd
#########################


epicsEnvSet("AXISCONFIG",    "stepSize=1.0;HomProc=1")
epicsEnvSet("MOTOR_NAME",    "MC-MCU-01:m9")
epicsEnvSet("AXIS_NO",       "9")
epicsEnvSet("DESC",          "Height 1")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "stepSize=1.0;HomProc=15")
epicsEnvSet("MOTOR_NAME",    "MC-MCU-01:m10")
epicsEnvSet("AXIS_NO",       "10")
epicsEnvSet("DESC",          "Omega 2")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "stepSize=1.0;HomProc=15")
epicsEnvSet("MOTOR_NAME",    "MC-MCU-01:m11")
epicsEnvSet("AXIS_NO",       "11")
epicsEnvSet("DESC",          "Kappa")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "stepSize=1.0;HomProc=1")
epicsEnvSet("MOTOR_NAME",    "MC-MCU-01:m12")
epicsEnvSet("AXIS_NO",       "12")
epicsEnvSet("DESC",          "Lin3")
< EthercatMCAxis.cmd
