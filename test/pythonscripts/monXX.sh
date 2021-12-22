#!/bin/sh

# Our motor to work against
export P=LabS-MCAG:MC-MCU-07:
export M=m1


if test -x ../../checkws.sh; then
  ../../checkws.sh || exit 1
fi

# Do we have pvmonitor in the PATH ?
if ! type pvmonitor >/dev/null 2>&1 ; then
  uname_s=$(uname -s 2>/dev/null || echo unknown)
  uname_m=$(uname -m 2>/dev/null || echo unknown)
  INSTALLED_EPICS=../../../../../.epics.$(hostname).$uname_s.$uname_m
  if ! test -r "$INSTALLED_EPICS"; then
    echo >&2 #can not find pvmonitor"
    echo >&2 #can not find $INSTALLED_EPICS"
    exit 1
  fi
  . "$INSTALLED_EPICS"
fi


# Do we need EPICS_CA_ADDR_LIST
if ! caget "${P}${M}" >/dev/null; then
  IOCIP=172.30.244.38
  export EPICS_CA_ADDR_LIST="$IOCIP $EPICS_CA_ADDR_LIST"
  export EPICS_PVA_ADDR_LIST="$IOCIP $EPICS_PVA_ADDR_LIST"
fi
if ! caget "${P}${M}" >/dev/null; then
  echo >&2 "pvget ${P}${M} failed"
  exit 1
fi

# The log file name is dependent on the name of this script
mewithoutdir="${0##*/}"
basename="${mewithoutdir%.*}"
#echo mewithoutdir=$mewithoutdir
#echo basname=$basename

LOGFILE=$basename.txt
LOGFILE2=${basename}X.txt
#echo LOGFILE=$LOGFILE ; exit

if test -f $LOGFILE; then
  timestamp=$(date "+%y-%m-%d-%H.%M.%S")
  mkdir -p logs
  test -f $LOGFILE && mv $LOGFILE ./logs/$timestamp-$LOGFILE || exit 1
  test -f $LOGFILE2 && mv $LOGFILE2 ./logs/$timestamp-$LOGFILE2 || exit 1
fi

#

TSE="-TSE"
#TSE=""
export TSE
camonitor \
  ${P}${M}-NamAuxBit0 \
  ${P}${M}-NamAuxBit1 \
  ${P}${M}-NamAuxBit2 \
  ${P}${M}-NamAuxBit3 \
  ${P}${M}-NamAuxBit4 \
  ${P}${M}-NamAuxBit5 \
  ${P}${M}-NamAuxBit6 \
  ${P}${M}-NamAuxBit7 \
  ${P}${M}-NamAuxBit8 \
  ${P}${M}-NamAuxBit9 \
  ${P}${M}-NamAuxBit10 \
  ${P}${M}-NamAuxBit11 \
  ${P}${M}-NamAuxBit12 \
  ${P}${M}-NamAuxBit13 \
  ${P}${M}-NamAuxBit14 \
  ${P}${M}-NamAuxBit15 \
  ${P}${M}-NamAuxBit16 \
  ${P}${M}-NamAuxBit17 \
  ${P}${M}-NamAuxBit18 \
  ${P}${M}-NamAuxBit19 \
  ${P}${M}-NamAuxBit20 \
  ${P}${M}-NamAuxBit21 \
  ${P}${M}-NamAuxBit22 \
  ${P}${M}-NamAuxBit23 \
  ${P}${M}-CfgSREV-RB \
  ${P}${M}-CfgUREV-RB \
  ${P}${M}-PosAct${TSE} \
  ${P}${M}-StatusCode${TSE} \
  ${P}${M}-StatusBits${TSE} \
  ${P}${M}-RawEncStep${TSE} \
  ${P}${M}-RawMtrStep${TSE} \
  ${P}${M}-RawMtrVelo${TSE} \
 | tee $LOGFILE | ./monXXX.py | tee $LOGFILE2
#  | tee $LOGFILE



