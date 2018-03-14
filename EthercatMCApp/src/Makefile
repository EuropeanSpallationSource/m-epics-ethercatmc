TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# build an ioc application

PROD_IOC = EthercatMC

# <name>.dbd will be created from <name>Include.dbd
DBD += EthercatMC.dbd
EthercatMC_DBD += base.dbd
EthercatMC_DBD += asyn.dbd
EthercatMC_DBD += drvAsynIPPort.dbd

ifneq (,$(findstring /,$(AXIS)))
  EthercatMC_DBD += axisSupport.dbd
  EthercatMC_LIBS += axis
  USR_CPPFLAGS += -DAXISNOTMOTOR=1
endif
ifneq (,$(findstring /,$(CALC)))
  EthercatMC_DBD += calcSupport.dbd
  EthercatMC_LIBS += calc
endif
ifneq (,$(findstring /,$(MOTOR)))
  #EthercatMC_DBD += devSoftMotor.dbd
  EthercatMC_DBD += motorSupport.dbd
  USR_CPPFLAGS += -DMOTORNOTAXIS=1
  EthercatMC_LIBS += motor
  EthercatMC_LIBS += softMotor
endif

EthercatMC_DBD += EthercatMcSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
EthercatMC_SRCS += EthercatMC_registerRecordDeviceDriver.cpp
EthercatMC_SRCS_DEFAULT += EthercatMCMain.cpp

#add a definition for each support application used by this application
EthercatMC_LIBS += asyn

ifneq (,$(findstring /,$(AXIS)))
EthercatMC_LIBS += axis
endif
ifneq (,$(findstring /,$(CALC)))
EthercatMC_LIBS += calc

endif
ifneq (,$(findstring /,$(MOTOR)))
EthercatMC_LIBS += motor
EthercatMC_LIBS += softMotor
endif


EthercatMC_LIBS += EthercatMcSupport
EthercatMC_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
