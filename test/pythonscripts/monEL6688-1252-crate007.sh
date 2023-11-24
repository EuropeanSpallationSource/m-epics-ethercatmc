#!/bin/sh
# shellcheck disable=SC1090
# shellcheck disable=SC2086
P="LabS-MCAG:MC-MCU-07:"
if ! type pvmonitor >/dev/null 2>&1; then
  . ~/MCAG_SetupMotionDemo/.epics.ymir-vm-ioc-01.cn.nin.ess.eu.Linux.x86_64
fi

if ! type python3 >/dev/null 2>&1; then
  echo >&2 python3 not found. Please run
  echo >&2 conda activate pyepicsPytestPVApy
  exit 1
fi

# The log file name is dependent on the name of this script
mewithoutdir="${0##*/}"
basename="${mewithoutdir%.*}"
LOGFILE=${basename}.txt
echo "##MYBASENAME=$MYBASENAME"
echo "##LOGFILE=$LOGFILE"

if test -f $LOGFILE; then
  timestamp=$(date "+%y-%m-%d-%H.%M.%S")
  mkdir -p logs
  mv $LOGFILE ./logs/$timestamp-$LOGFILE || exit 1
fi

PVS=""
PVS="$PVS ${P}PTPOffset"
#PVS="$PVS ${P}PTPOffset.STAT"
#PVS="$PVS ${P}PTPOffset.SEVR"
PVS="$PVS ${P}PTPOffset1"
PVS="$PVS ${P}PTPOffset2"
#PVS="$PVS ${P}PTPOffset2.STAT"
#PVS="$PVS ${P}PTPOffset2.SEVR"
PVS="$PVS ${P}PTPState"
PVS="$PVS ${P}PTPallGood"
#PVS="$PVS ${P}UTCEL1252P0"
PVS="$PVS ${P}TimeOffsetDiffEL6688"
PVS="$PVS ${P}PTPdiffTimeIOC_MCU"
PVS="$PVS ${P}PTPErrorStatus"
PVS="$PVS ${P}oTS_NS"

for PV in $PVS; do
  pvget $PV || {
    echo >&2 $PV not found
    exit 1
  }
done

pvmonitor $PVS | ./tai2string.py 2>&1 | tee $LOGFILE
