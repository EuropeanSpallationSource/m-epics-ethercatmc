TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# build an ioc application

PROD_IOC = ethercatmc

# <name>.dbd will be created from <name>Include.dbd
DBD += ethercatmc.dbd
ethercatmc_DBD += base.dbd
ethercatmc_DBD += asyn.dbd
ethercatmc_DBD += drvAsynIPPort.dbd

ifneq (,$(findstring /,$(ADS)))
  ethercatmc_DBD  +=  ads.dbd
  ethercatmc_LIBS += ads
endif
ifneq (,$(findstring /,$(CALC)))
  ethercatmc_DBD += calcSupport.dbd
  ethercatmc_LIBS += calc
endif
ifneq (,$(findstring /,$(MOTOR)))
  ethercatmc_DBD += motorSupport.dbd
  ethercatmc_DBD += devSoftMotor.dbd
  ethercatmc_LIBS += motor
  ethercatmc_LIBS += softMotor
endif

ifneq (,$(findstring /,$(STREAM)))
  ethercatmc_DBD += stream.dbd
  ethercatmc_LIBS += stream
endif

ETHERCATMC_TCBSD   := $(shell sh -c 'test -f /usr/local/lib/libTcAdsDll.so && echo YES')
ifeq ($(ETHERCATMC_TCBSD),YES)
  ethercatmc_SYS_LIBS += TcAdsDll
endif

ifdef BASE_7_0
 ethercatmc_DBD += PVAServerRegister.dbd
 ethercatmc_DBD += PVACommonRegister.dbd
 ethercatmc_DBD += softIocPVA.dbd
 ethercatmc_DBD += registerChannelProviderLocal.dbd
endif

ethercatmc_DBD += ethercatmcSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
ethercatmc_SRCS += ethercatmc_registerRecordDeviceDriver.cpp
ethercatmc_SRCS_DEFAULT += ethercatmcMain.cpp

#add a definition for each support application used by this application
ethercatmc_LIBS += asyn

ifneq (,$(findstring /,$(CALC)))
ethercatmc_LIBS += calc
endif

ifneq (,$(findstring /,$(MOTOR)))
ethercatmc_LIBS += motor
ethercatmc_LIBS += softMotor
endif

ifneq (,$(findstring /,$(PVXS)))
ethercatmc_LIBS += pvxsIoc
ethercatmc_LIBS += pvxs
ethercatmc_DBD  += pvxsIoc.dbd
endif

ifneq (,$(findstring /,$(TWINCATADS)))
ethercatmc_LIBS += ads
ethercatmc_DBD  += ads.dbd
endif

ifdef BASE_7_0
ethercatmc_LIBS += qsrv
ethercatmc_LIBS += pvDatabase
ethercatmc_LIBS += $(EPICS_BASE_PVA_CORE_LIBS)
endif

ethercatmc_LIBS += ethercatmcSupport
ethercatmc_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
