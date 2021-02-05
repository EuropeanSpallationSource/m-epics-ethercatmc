#!/bin/sh

if ! type pvmonitor >/dev/null ; then
  . ~/MCAG_setupMotionDemo.191003-base-7.0.3/.epics.ics-vm-mc-ioc-01.cslab.esss.lu.se.Linux.x86_64
fi


#pvmonitor LabS-MCAG:MC-MCU-07:PTPState  LabS-MCAG:MC-MCU-07:sDctEL1252P LabS-MCAG:MC-MCU-07:sDctEL1252N LabS-MCAG:MC-MCU-07:sSystemTAIclock LabS-MCAG:MC-MCU-07:nSystemTAIclock

#pvmonitor LabS-MCAG:MC-MCU-07:PTPState  LabS-MCAG:MC-MCU-07:sSystemTAIclock LabS-MCAG:MC-MCU-07:nSystemTAIclock LabS-MCAG:MC-MCU-07:TAIEL1252N0
pvmonitor LabS-MCAG:MC-MCU-07:PTPState  LabS-MCAG:MC-MCU-07:TAIEL1252P0 | ./tai2string.py

