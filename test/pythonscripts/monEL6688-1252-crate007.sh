#!/bin/sh
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

pvmonitor ${P}PTPState ${P}PTPOffset \
${P}DcToExtTimeOffsetEL6688 \
${P}DcToExtTimeOffsetSystem  \
${P}WriteDoneExtTimeOffset  \
${P}TimeOffsetDiffEL6688 \
${P}PTPdiffTimeIOC_MCU \
${P}UTCEL1252P0 | ./tai2string.py  2>&1 | tee $LOGFILE
