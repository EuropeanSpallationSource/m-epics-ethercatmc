TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = EthercatMC
# EthercatMC.dbd will be created and installed
DBD += EthercatMC.dbd

# EthercatMC.dbd will be made up from these files:
EthercatMC_DBD += base.dbd
EthercatMC_DBD += axisSupport.dbd

# Include dbd files from all support applications:
#EthercatMC_DBD += xxx.dbd
EthercatMC_DBD += asyn.dbd
EthercatMC_DBD += drvAsynIPPort.dbd
EthercatMC_DBD += EthercatMCController.dbd

# Add source files
EthercatMC_SRCS += EthercatMCController.cpp
EthercatMC_SRCS += EthercatMCAxis.cpp

# Add all the support libraries needed by this IOC
EthercatMC_LIBS += asyn
EthercatMC_LIBS += axis

# EthercatMC_registerRecordDeviceDriver.cpp derives from EthercatMC.dbd
EthercatMC_SRCS += EthercatMC_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
EthercatMC_SRCS_DEFAULT += EthercatMCMain.cpp
EthercatMC_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#EthercatMC_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
EthercatMC_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

