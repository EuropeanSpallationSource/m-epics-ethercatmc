# @field AXISCONFIG
# @type  STRING
# File name for axis configuration

# @field PREFIX
# @type  STRING

# @field MOTOR_NAME
# @type  STRING
# m1, m2, m3, X, Y, Z

# @field MOTOR_PORT
# @type  STRING
# @uniqueness IOC
# MCU1, MCU2

# @field AXIS_NO
# @type  STRING
# 1,2,3,4

# @field DESC
# @type  STRING
# Description shown in UI

# @field PREC
# @type  INTEGER
# 3

# @field VELO
# @type  FLOAT
# moving velocity. 10 means 10mm/sec

# @field JVEL
# @type  FLOAT
# jogging velocity 5 means 5mm/sec

# @field JAR
# @type  FLOAT
# jogging acceleration 10mm/sec2

# @field ACCL
# @type  FLOAT
# acceleration to velocity in seconds

# @field MRES
# @type  FLOAT
# Size of a motor step

# @field ERES
# @type  FLOAT
# Size of an encoder step

# @field RDBD
# @type  FLOAT
# Retry deadband

# @field NTMF
# @type  INTEGER
# New Target Monitor Deadband Factor

# @field DLLM
# @type  FLOAT
# low soft limit in dial coordinates

# @field DHLM
# @type  FLOAT
# high soft limit in dial coordinates

# @field HOMEPROC
# @type  INTEGER
# Homing procedure

EthercatMCCreateAxis("$(MOTOR_PORT)", "$(AXIS_NO)", "4", "$(AXISCONFIG)")

dbLoadRecords("EthercatMC.template", "PREFIX=$(PREFIX), MOTOR_NAME=$(MOTOR_NAME), R=$(R), MOTOR_PORT=$(MOTOR_PORT), ASYN_PORT=$(ASYN_PORT), AXIS_NO=$(AXIS_NO), DESC=$(DESC), PREC=$(PREC), VELO=$(VELO), JVEL=$(JVEL), JAR=$(JAR), ACCL=$(ACCL), MRES=$(MRES), ERES=$(ERES), RDBD=$(RDBD), NTMF=$(NTMF), DLLM=$(DLLM), DHLM=$(DHLM), HOMEPROC=$(HOMEPROC)")


dbLoadRecords("EthercatMChome.template", "PREFIX=$(PREFIX), MOTOR_NAME=$(MOTOR_NAME), R=$(R), MOTOR_PORT=$(MOTOR_PORT), ASYN_PORT=$(ASYN_PORT), AXIS_NO=$(AXIS_NO),  HOMEPROC=$(HOMEPROC), HOMEPOS=$(HOMEPOS), HVELTO=$(HVELTO), HVELFRM=$(HVELFRM), HOMEACC=$(HOMEACC), HOMEDEC=$(HOMEDEC)")
