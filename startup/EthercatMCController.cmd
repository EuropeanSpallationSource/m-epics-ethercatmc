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

# Additional fields can e initialized in Axis
epicsEnvSet("ECAXISFIELDINIT",    "")


## One of the 2 needs to be done, either drvAsynIPPortConfigure+Eos
## Or
## adsAsynPortDriverConfigure (without Eos.
## Note that ./run-ADS-ioc.sh will patch this file as needed
drvAsynIPPortConfigure("$(ASYN_PORT)","$(IPADDR):$(IPPORT)",0,0,0)
asynOctetSetOutputEos("$(ASYN_PORT)", -1, ";\n")
asynOctetSetInputEos("$(ASYN_PORT)", -1, ";\n")

#adsAsynPortDriverConfigure("$(ASYN_PORT)","$(IPADDR)","$(AMSID)",852,1000,0,0,50,100,1000,0)

EthercatMCCreateController("$(MOTOR_PORT)", "$(ASYN_PORT)", "32", "200", "1000")

#/* traceMask definitions*/
#define ASYN_TRACE_ERROR     0x0001
#define ASYN_TRACEIO_DEVICE  0x0002
#define ASYN_TRACEIO_FILTER  0x0004
#define ASYN_TRACEIO_DRIVER  0x0008
#define ASYN_TRACE_FLOW      0x0010
#define ASYN_TRACE_WARNING   0x0020
#define ASYN_TRACE_INFO      0x0040
asynSetTraceMask("$(ASYN_PORT)", -1, 0x41)



#/* traceIO mask definitions*/
#define ASYN_TRACEIO_NODATA 0x0000
#define ASYN_TRACEIO_ASCII  0x0001
#define ASYN_TRACEIO_ESCAPE 0x0002
#define ASYN_TRACEIO_HEX    0x0004
asynSetTraceIOMask("$(ASYN_PORT)", -1, 2)


#/* traceInfo mask definitions*/
#define ASYN_TRACEINFO_TIME 0x0001
#define ASYN_TRACEINFO_PORT 0x0002
#define ASYN_TRACEINFO_SOURCE 0x0004
#define ASYN_TRACEINFO_THREAD 0x0008
asynSetTraceInfoMask("$(ASYN_PORT)", -1, 5)
