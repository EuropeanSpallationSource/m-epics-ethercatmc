# @field AXISCONFIG
# @type  STRING
# File name for axis configuration, leave empty

# @field PREFIX
# @type  STRING

# @field MOTOR_NAME
# @type  STRING
# m1, m2, m3, X, Y, Z

# @field MOTOR_PORT
# @type  STRING
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

# @field ERES
# @type  FLOAT
# Size of an encoder step, set to 1

# @field RDBD
# @type  FLOAT
# Retry deadband

# @field SDBD
# @type  FLOAT
# Setpoint deadband

# @field DLLM
# @type  FLOAT
# low soft limit in dial coordinates

# @field DHLM
# @type  FLOAT
# high soft limit in dial coordinates

EthercatMCCreateAxis("$(MOTOR_PORT)", "$(AXIS_NO)", "2", "$(AXISCONFIG)")

dbLoadRecords("EthercatMC.template", "PREFIX=$(PREFIX), MOTOR_NAME=$(MOTOR_NAME), R=$(R), MOTOR_PORT=$(MOTOR_PORT), ASYN_PORT=$(ASYN_PORT), AXIS_NO=$(AXIS_NO), DESC=$(DESC), PREC=$(PREC), VELO=$(VELO), JVEL=$(JVEL), JAR=$(JAR), ACCL=$(ACCL), ERES=$(ERES), SDBD=$(SDBD), RDBD=$(RDBD), DLLM=$(DLLM), DHLM=$(DHLM)")


dbLoadRecords("EthercatMChome.template", "PREFIX=$(PREFIX), MOTOR_NAME=$(MOTOR_NAME), R=$(R), MOTOR_PORT=$(MOTOR_PORT), ASYN_PORT=$(ASYN_PORT), AXIS_NO=$(AXIS_NO),  HOMEPROC=$(HOMEPROC), HOMEPOS=$(HOMEPOS), HVELTO=$(HVELTO), HVELFRM=$(HVELFRM), HOMEACC=$(HOMEACC), HOMEDEC=$(HOMEDEC)")
