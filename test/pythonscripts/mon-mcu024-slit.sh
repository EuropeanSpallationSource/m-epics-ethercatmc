#!/bin/sh
# shellcheck disable=SC1090
# shellcheck disable=SC2086
P="LabS-MCAG:MC-MCU-24:"
if ! type pvmonitor >/dev/null 2>&1; then
  echo >&2 "pvmonitor not found"
  exit 1
fi

for n in 1 2 3 4; do
  PVS="$PVS ${P}m${n}-PLCopenState"
  for j in 0 1 2 3 4 5 6 7test/pythonscripts/mon-mcu024-slit.sh; do
    PVS="$PVS ${P}m${n}-PLCopenState-NamBit${j}"
  done
done

#export EPICS_PVA_ADDR_LIST="idmz-ro-epics-gw-tn.esss.lu.se 172.30.38.12"

PVSEGU=""
for PV in $PVS; do
  pvget $PV || {
    echo >&2 $PV not found
    exit 1
  }
done

pvmonitor $PVS
