# @field ASYN_PORT
# @type  STRING
# MC_CPU1

# @field MOTOR_PORT
# @type  STRING
# MCU1

# @field IPADDR
# @type  STRING

# @field IPPORT
# @type  INTEGER

drvAsynIPPortConfigure("$(ASYN_PORT)","$(IPADDR):$(IPPORT)",0,0,0)
asynOctetSetOutputEos("$(ASYN_PORT)", -1, ";\n")
asynOctetSetInputEos("$(ASYN_PORT)", -1, ";\n")
EthercatMCCreateController("$(MOTOR_PORT)", "$(ASYN_PORT)", "32", "200", "1000")


asynSetTraceMask("$(ASYN_PORT)", -1, 0x41)
asynSetTraceIOMask("$(ASYN_PORT)", -1, 2)

# Bit 2: file/line
# Bit 3: thread
#asynSetTraceInfoMask("$(ASYN_PORT)", -1, 15)
# Bit 0: Time
# Bit 1: Port
asynSetTraceInfoMask("$(ASYN_PORT)", -1, 3)
