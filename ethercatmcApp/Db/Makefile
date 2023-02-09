TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install) dbd files
# <name>.dbd will be created from <name>Include.dbd
#DBD += axis.dbd

#----------------------------------------------------
# Create and install (or just install)
# databases, templates, substitutions like this
DB += ethercatmc.template
DB += ethercatmcAuxBitsStatus.template
DB += ethercatmcClutch.template
DB += ethercatmcEL1252-OneChannel.template
DB += ethercatmcEL3162-OneChannel.template
DB += ethercatmcRawMtrEnc.template
DB += ethercatmcPLCopenStateMachine.template
DB += ethercatmcEL6688-PTP.template
DB += ethercatmcEL6688-EL1252-PTP.template
DB += ethercatmcMotorTemp.template
DB += ethercatmcOpenClutch.template
DB += ethercatmcPlcName.template
DB += ethercatmcShutter.template
DB += ethercatmcdebug.template
DB += ethercatmcindexer.template
DB += ethercatmcreadback.template
DB += ethercatmcDefPos.template
DB += ethercatmcslit_soft.template
DB += ethercatmcslit_hard.template
DB += ethercatmcslitAvoidCollSoftlimits.template
DB += ethercatmcTestInputRecordAlarm.template



#----------------------------------------------------
# Declare template files which do not show up in DB
#USES_TEMPLATE += axis.template

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
