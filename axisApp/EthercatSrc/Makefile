TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC = EthercatMCSupport

# EthercatMCSupport.dbd will be created and installed
DBD += EthercatMCSupport.dbd
DBD += EthercatMC.dbd

# EthercatMCSupport.dbd will be made up from these files:
EthercatMCSupport_DBD += devEthercatMC.dbd

# Add source files
EthercatMCSupport_SRCS += EthercatMCController.cpp
EthercatMCSupport_SRCS += EthercatMCAxis.cpp
EthercatMCSupport_SRCS += EthercatMCHelper.cpp

# EthercatMCSupport_registerRecordDeviceDriver.cpp derives from EthercatMCSupport.dbd
EthercatMCSupport_SRCS += EthercatMCSupport_registerRecordDeviceDriver.cpp

#=============================
# Build the IOC application

PROD_IOC = EthercatMC

EthercatMC_DBD += base.dbd
EthercatMC_DBD += axisSupport.dbd
EthercatMC_DBD += asyn.dbd
EthercatMC_DBD += drvAsynIPPort.dbd
EthercatMC_DBD += devEthercatMC.dbd

# Build the main IOC entry point on workstation OSs.
EthercatMC_SRCS += EthercatMCMain.cpp
EthercatMC_SRCS += EthercatMC_registerRecordDeviceDriver.cpp

EthercatMC_LIBS += asyn
EthercatMC_LIBS += axis
EthercatMC_LIBS += EthercatMCSupport
EthercatMC_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

