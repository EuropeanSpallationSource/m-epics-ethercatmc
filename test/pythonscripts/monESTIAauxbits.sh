#!/bin/sh
# shellcheck disable=SC1090
# shellcheck disable=SC2086

#  Script to monitor MotorPositions
#  The positions are timestamped inside the controller:
#  see the "-TSE" pvs
#  Monitor the values from the stepper terminal and the encoder
#  terminal as well.

# Our motor to work against
if test -z "$P"; then
  export P=PSI-ESTIARND:MC-MCU-01:
fi
if test -z "$M"; then
  export M=Mtr8
fi

if test -x ../../checkws.sh; then
  ../../checkws.sh || exit 1
fi

# Do we have pvmonitor in the PATH ?
if ! type pvmonitor >/dev/null 2>&1; then
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
P_M_NO_COLON=$(echo $P$M | sed -e "s/:/_/g")
LOGFILEBASENAME=log-$P_M_NO_COLON-$basename
export LOGFILEBASENAME
echo LOGFILEBASENAME=$LOGFILEBASENAME

#Move old logfiles out of the way
mkdir -p logs
timestamp=$(date "+%y-%m-%d-%H.%M.%S")
for ext in .txt -processed.txt; do
  test -f $LOGFILEBASENAME$ext && mv $LOGFILEBASENAME$ext logs/log-$P_M_NO_COLON-$timestamp$ext
done

#TSE="-TSE"
TSE=""
export TSE
pvmonitor \
  ${P}${M}-NamBit0 \
  ${P}${M}-NamBit1 \
  ${P}${M}-NamBit2 \
  ${P}${M}-NamBit3 \
  ${P}${M}-NamBit4 \
  ${P}${M}-NamBit5 \
  ${P}${M}-NamBit6 \
  ${P}${M}-NamBit7 \
  ${P}${M}-NamBit8 \
  ${P}${M}-NamBit9 \
  ${P}${M}-NamBit10 \
  ${P}${M}-NamBit11 \
  ${P}${M}-NamBit12 \
  ${P}${M}-NamBit13 \
  ${P}${M}-NamBit14 \
  ${P}${M}-NamBit15 \
  ${P}${M}-NamBit16 \
  ${P}${M}-NamBit17 \
  ${P}${M}-NamBit18 \
  ${P}${M}-NamBit19 \
  ${P}${M}-NamBit20 \
  ${P}${M}-NamBit21 \
  ${P}${M}-NamBit22 \
  ${P}${M}-NamBit23 \
  ${P}${M}.VAL \
  ${P}${M}.RBV \
  ${P}${M}.MISS \
  ${P}${M}-StatusBits \
  ${P}${M}-HexScrew |
  tee $LOGFILEBASENAME.txt | ./RawMtrEncPostprocess.py | tee $LOGFILEBASENAME-processed.txt

#PSI-ESTIARND:MC-MCU-01:Mtr8-HexScrew
