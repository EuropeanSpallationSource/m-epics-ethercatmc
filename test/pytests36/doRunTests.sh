#!/bin/sh

set -x

run_pytest ()
{
  echo CONDA_PREFIX="$CONDA_PREFIX"
  if type pytest; then
    pytest "$@" || exit 1
  elif test "$CONDA_PREFIX"; then
      echo pytest "$@"
      pytest "$@"
  else
    echo $PYTHON $VIRTUALENVDIR/bin/pytest "$@"
    $PYTHON $VIRTUALENVDIR/bin/pytest "$@" || exit 1
  fi
}

echo "$0" "$@"

if test -n "$1"; then
   TESTEDMOTORAXIS=$1
   PREFIX=${1%:*}
   TESTEDMOTORADDR=${1##*:m}
   TESTEDMCUASYN=$PREFIX:MCU1:asyn
   echo TESTEDMOTORAXIS=$TESTEDMOTORAXIS
   echo TESTEDMOTORADDR=$TESTEDMOTORADDR
   echo TESTEDMCUASYN=$TESTEDMCUASYN
   shift 1
else
  echo >&2 "$0 <PV> [numruns] [testfile.py]"
  exit 1
fi


files=""
numruns=1
while test -n "$1" && test -f "$1"; do
    files="$files $1"
    shift 1
done

if test -n "$1" && test "$1" -ne 0; then
    numruns=$1
    shift 1
else
    numruns=1
fi


#if test -z "$EPICS_CA_ADDR_LIST" && test -z "$EPICS_CA_AUTO_ADDR_LIST"; then
#  if EPICS_CA_ADDR_LIST=127.0.1 EPICS_CA_AUTO_ADDR_LIST=NO caget $TESTEDMOTORAXIS.RBV >/dev/null 2>&1; then
#    EPICS_CA_ADDR_LIST=127.0.1
#    EPICS_CA_AUTO_ADDR_LIST=NO
#    export EPICS_CA_ADDR_LIST EPICS_CA_AUTO_ADDR_LIST
#  fi
#fi

while test $numruns -gt 0; do
  #if ! caget $TESTEDMOTORAXIS.RBV >/dev/null 2>/dev/null; then
  #  echo >&2 caget $TESTEDMOTORAXIS failed
  #  exit 1
  #fi
  export TESTEDMOTORADDR TESTEDMOTORAXIS TESTEDMCUASYN
  if test -n "$files"; then
    files=$(echo $files | sort)
    echo files=$files
    for file in $files; do
      echo file=$file
      run_pytest "$@" $file || exit 1
    done
  else
    py=$(echo *.py | sort)
    echo py=$py
    for p in $py
    do
      run_pytest "$@" $p || exit 1
    done
  fi
  echo Runs left=$numruns
  numruns=$(($numruns - 1))
done
