TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library

LIBRARY_IOC += EthercatMcSupport

DBD += EthercatMcSupport.dbd

# The following are compiled and added to the Support library

EthercatMcSupport_SRCS += EthercatMCController.cpp
EthercatMcSupport_SRCS += EthercatMCAxis.cpp
EthercatMcSupport_SRCS += EthercatMCHelper.cpp


EthercatMcSupport_LIBS += asyn

ifneq (,$(findstring /,$(ADS)))
EthercatMcSupport_LIBS += ads
endif
ifneq (,$(findstring /,$(AXIS)))
EthercatMcSupport_LIBS += axis
endif
ifneq (,$(findstring /,$(CALC)))
EthercatMcSupport_LIBS += calc
endif
ifneq (,$(findstring /,$(MOTOR)))
EthercatMcSupport_LIBS += motor
EthercatMcSupport_LIBS += softMotor
endif


EthercatMcSupport_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
