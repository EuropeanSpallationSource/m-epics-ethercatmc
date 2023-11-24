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
PVS="$PVS ${P}PTPState"
PVS="$PVS ${P}PTPallGood"
PVS="$PVS ${P}UTCEL1252P0"
PVS="$PVS ${P}PTPdiffTimeIOC_MCU"
PVS="$PVS ${P}PTPErrorStatus"
PVS="$PVS ${P}TS_NS"
PVS="$PVS YMIR-SpScn:MC-MCU-001:PTPOffset"
PVS="$PVS YMIR-SpScn:MC-MCU-001:PTPState"
PVS="$PVS YMIR-SpScn:MC-MCU-001:PTPallGood"
PVS="$PVS YMIR-SETS:SE-BADC-001:TS_NS"
PVS="$PVS YMIR-SETS:SE-BPTP-001:SYNCHRONIZED"
PVS="$PVS YMIR-SETS:SE-BPTP-001:TTL_EPOCH_SYNCH"
PVS="$PVS YMIR-SETS:SE-BPTP-001:TTL_NS_OFSET_OK"
PVS="$PVS YMIR-SETS:SE-BPTP-001:PTP_OFFSET_MASTER"

export EPICS_PVA_ADDR_LIST="idmz-ro-epics-gw-tn.esss.lu.se 172.30.38.12"

PVSEGU=""
for PV in $PVS; do
  pvget $PV || {
    echo >&2 $PV not found
    exit 1
  }
  PVSEGU="$PVSEGU $PV.EGU"
done

pvmonitor $PVS $PVSEGU 2>&1 | ./tai2string.py | tee $LOGFILE
