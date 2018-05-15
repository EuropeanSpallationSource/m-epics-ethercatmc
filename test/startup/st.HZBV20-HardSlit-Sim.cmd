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
epicsEnvSet("AXISCONFIG",    "cfgFile=./HZBV20-SoftSlit-Sim-1.cfg")
epicsEnvSet("MOTOR_NAME",    "MC-MCU-01:m1")
epicsEnvSet("R",             "MC-MCU-01:m1-")
epicsEnvSet("AXIS_NO",       "1")
epicsEnvSet("DESC",          "H Center")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "cfgFile=./HZBV20-SoftSlit-Sim-2.cfg")
epicsEnvSet("MOTOR_NAME",    "MC-MCU-01:m2")
epicsEnvSet("R",             "MC-MCU-01:m2-")
epicsEnvSet("AXIS_NO",       "2")
epicsEnvSet("DESC",          "H Gap")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "cfgFile=./HZBV20-SoftSlit-Sim-3.cfg")
epicsEnvSet("MOTOR_NAME",    "MC-MCU-01:m3")
epicsEnvSet("R",             "MC-MCU-01:m3-")
epicsEnvSet("AXIS_NO",       "3")
epicsEnvSet("DESC",          "H high Blade")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "cfgFile=./HZBV20-SoftSlit-Sim-4.cfg")
epicsEnvSet("MOTOR_NAME",    "MC-MCU-01:m4")
epicsEnvSet("R",             "MC-MCU-01:m4-")
epicsEnvSet("AXIS_NO",       "4")
epicsEnvSet("DESC",          "H low Blade")
< EthercatMCAxis.cmd

#########################
## Logical axes and slit
epicsEnvSet("P",             "$(PREFIX)")
epicsEnvSet("SLIT",          "MC-SLT-01:SltH-")
epicsEnvSet("mXc",           "m1")
epicsEnvSet("mXg",           "m2")
epicsEnvSet("mXp",           "m3")
epicsEnvSet("mXn",           "m4")

## Slit
< EthercatMC2slit_hard.cmd
#########################

# Slit Vertical
#########################
#epicsEnvSet("AXISCONFIG",    "cfgFile=./HZBV20-SoftSlit-Sim-3.cfg")
#epicsEnvSet("MOTOR_NAME",    "MC-MCU-01:m3")
#epicsEnvSet("R",             "MC-MCU-01:m3-")
#epicsEnvSet("AXIS_NO",       "3")
#epicsEnvSet("DESC",          "V low blade")
#< EthercatMCAxis.cmd
#
#epicsEnvSet("AXISCONFIG",    "cfgFile=./HZBV20-SoftSlit-Sim-4.cfg")
#epicsEnvSet("MOTOR_NAME",    "MC-MCU-01:m4")
#epicsEnvSet("R",             "MC-MCU-01:m4-")
#epicsEnvSet("AXIS_NO",       "4")
#epicsEnvSet("DESC",          "V high Blade")
#< EthercatMCAxis.cmd
#
### Logical axes and slit
#epicsEnvSet("SLIT",          "MC-SLT-01:SltV-")
#epicsEnvSet("mXp",           "MC-MCU-01:m4")
#epicsEnvSet("mXn",           "MC-MCU-01:m3")
#
### Slit
#< EthercatMC2slit.cmd
#########################


epicsEnvSet("AXISCONFIG",    "cfgFile=./HZBV20-SoftSlit-Sim-1.cfg")
epicsEnvSet("MOTOR_NAME",    "MC-MCU-01:m5")
epicsEnvSet("R",             "MC-MCU-01:m5-")
epicsEnvSet("AXIS_NO",       "5")
epicsEnvSet("DESC",          "Height 1")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "cfgFile=./SlitAxisMCU010-1.cfg")
epicsEnvSet("MOTOR_NAME",    "MC-MCU-01:m6")
epicsEnvSet("R",             "MC-MCU-01:m6-")
epicsEnvSet("AXIS_NO",       "6")
epicsEnvSet("DESC",          "Omega 2")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "cfgFile=./SlitAxisMCU010-1.cfg")
epicsEnvSet("MOTOR_NAME",    "MC-MCU-01:m7")
epicsEnvSet("R",             "MC-MCU-01:m7-")
epicsEnvSet("AXIS_NO",       "7")
epicsEnvSet("DESC",          "Kappa")
< EthercatMCAxis.cmd

epicsEnvSet("AXISCONFIG",    "cfgFile=./SlitAxisMCU010-1.cfg")
epicsEnvSet("MOTOR_NAME",    "MC-MCU-01:m8")
epicsEnvSet("R",             "MC-MCU-01:m8-")
epicsEnvSet("AXIS_NO",       "8")
epicsEnvSet("DESC",          "Lin3")
< EthercatMCAxis.cmd
