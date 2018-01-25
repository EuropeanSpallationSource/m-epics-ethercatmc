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

EthercatMCCreateAxis("$(MOTOR_PORT)", "$(AXIS_NO)", "6", "$(AXISCONFIG)")

dbLoadRecords("EthercatMC.template", "PREFIX=$(PREFIX), MOTOR_NAME=$(MOTOR_NAME), R=$(R), MOTOR_PORT=$(MOTOR_PORT), ASYN_PORT=$(ASYN_PORT), AXIS_NO=$(AXIS_NO), DESC=$(DESC), PREC=$(PREC) ")


dbLoadRecords("EthercatMChome.template", "PREFIX=$(PREFIX), MOTOR_NAME=$(MOTOR_NAME), R=$(R), MOTOR_PORT=$(MOTOR_PORT), ASYN_PORT=$(ASYN_PORT), AXIS_NO=$(AXIS_NO),  HOMEPROC=$(HOMEPROC), HOMEPOS=$(HOMEPOS), HVELTO=$(HVELTO), HVELFRM=$(HVELFRM), HOMEACC=$(HOMEACC), HOMEDEC=$(HOMEDEC)")
