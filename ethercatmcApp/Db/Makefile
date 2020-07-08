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
DB += sDctEL1252.proto
DB += UTCtime.proto
DB += ethercatmc.template
DB += ethercatmcPTP.template
DB += ethercatmcDctEL1252-PILS.template
DB += ethercatmcDctEL1252-Stream.template
DB += ethercatmcdebug.template
DB += ethercatmchome.template
DB += ethercatmcindexer.template
DB += ethercatmcmcUTCtime.template
DB += ethercatmcreadback.template
DB += ethercatmcslit_soft.template
DB += ethercatmcslit_hard.template
DB += ethercatmcslitAvoidCollSoftlimits.template



#----------------------------------------------------
# Declare template files which do not show up in DB
#USES_TEMPLATE += axis.template

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
