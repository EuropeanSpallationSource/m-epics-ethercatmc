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
DB += EthercatMC.template
DB += EthercatMCdebug.template
DB += EthercatMChome.template
DB += EthercatMCindexer.template
DB += EthercatMCreadback.template
DB += EthercatMCslit_soft.template
DB += EthercatMCslit_hard.template
DB += EthercatMCslitAvoidCollSoftlimits.template



#----------------------------------------------------
# Declare template files which do not show up in DB
#USES_TEMPLATE += axis.template

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
