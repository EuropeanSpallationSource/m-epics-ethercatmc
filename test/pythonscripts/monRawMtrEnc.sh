#!/bin/sh

if ! type pvmonitor >/dev/null ; then
  . ~/MCAG_setupMotionDemo.191003-base-7.0.3/.epics.ics-vm-mc-ioc-01.cslab.esss.lu.se.Linux.x86_64
fi
export P=LabS-MCAG:MC-MCU-07:
export M=m1
LOGFILE=monRawMtrEnc.txt

if test -f $LOGFILE; then
  timestamp=$(date "+%y-%m-%d-%H.%M.%S")
  mkdir -p logs
  mv $LOGFILE ./logs/$timestamp-$LOGFILE || exit 1
fi

#  ${P}${M}-StatusBits \

camonitor \
  ${P}${M}-StatusCode \
  ${P}${M}-RawEncCounter-UTC \
  ${P}${M}-RawMtrStep-UTC \
  ${P}${M}-RawMtrVelocity-UTC \
  | tee $LOGFILE
#camonitor   ${P}${M}-EL5101CounterValue  ${P}${M}-EL7037STM_CounterValue ${P}${M}-EL7037STM_Velocity


