TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library

LIBRARY_IOC += ethercatmcSupport

DBD += ethercatmcSupport.dbd

# The following are compiled and added to the Support library

ethercatmcSupport_SRCS += ethercatmcADS.cpp
ethercatmcSupport_SRCS += ethercatmcController.cpp
ethercatmcSupport_SRCS += ethercatmcIndexer.cpp
ethercatmcSupport_SRCS += ethercatmcIndexerV2.cpp
ethercatmcSupport_SRCS += ethercatmcIndexerAxis.cpp


ETHERCATMC_TCBSD   := $(shell sh -c 'test -f /usr/local/lib/libTcAdsDll.so && echo YES')
ifeq ($(ETHERCATMC_TCBSD),YES)
USR_CXXFLAGS += -DETHERCATMC_TCBSD
endif

ethercatmcSupport_LIBS += asyn

ifneq (,$(findstring /,$(ADS)))
ethercatmcSupport_LIBS += ads
endif
ifneq (,$(findstring /,$(CALC)))
ethercatmcSupport_LIBS += calc
endif
ifneq (,$(findstring /,$(MOTOR)))
ethercatmcSupport_LIBS += motor
ethercatmcSupport_LIBS += softMotor
endif

ifneq (,$(findstring /,$(STREAM)))
ethercatmcSupport_LIBS += stream
endif


ethercatmcSupport_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
