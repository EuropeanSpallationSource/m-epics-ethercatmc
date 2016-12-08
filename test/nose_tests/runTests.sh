#!/bin/sh

uname_s=$(uname -s 2>/dev/null || echo unknown)
uname_m=$(uname -m 2>/dev/null || echo unknown)


INSTALLED_EPICS=../../../.epics.$(hostname).$uname_s.$uname_m
if test -r $INSTALLED_EPICS; then
  echo INSTALLED_EPICS=$INSTALLED_EPICS
. $INSTALLED_EPICS
fi

if test -z "$PYEPICS_LIBCA"; then
    MYLIB=$EPICS_BASE/lib/$EPICS_HOST_ARCH/libca.so
    if test -r "$MYLIB"; then
      PYEPICS_LIBCA=$MYLIB
      export PYEPICS_LIBCA
    fi
fi &&
if ! python -c 'import epics' >/dev/null 2>&1; then
  if which easy_install >/dev/null 2>&1; then
    sudo easy_install -U pyepics
  else
    if ! which pip >/dev/null 2>&1; then
      if which yum >/dev/null 2>&1; then
        sudo yum install python-pip
      fi
      if which apt-get >/dev/null 2>&1; then
        sudo apt-get install python-pip
      fi
    fi
    if which pip >/dev/null 2>&1; then
      sudo pip install pyepics
    else
      false
    fi
  fi
fi &&
if ! which nosetests >/dev/null 2>&1; then
  if which easy_install >/dev/null 2>&1; then
    sudo easy_install nose
  else
    if ! which pip >/dev/null 2>&1; then
      if which yum >/dev/null 2>&1; then
        sudo yum install python-pip
      fi
      if which apt-get >/dev/null 2>&1; then
        sudo apt-get install python-pip
      fi
    fi
    if which pip >/dev/null 2>&1; then
      sudo pip install nose
    else
      false
    fi
  fi
fi || {
  echo >&2 Installation problem:
  echo >&2 pip not found
  echo >&2 easy_install not found
  exit 1
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
  echo >&2 "doit.sh <PV> [numruns] [testfile.py]"
  exit 1
fi


if test -n "$1" && test -f "$1"; then
    file=$1
    shift 1
else
    numruns=1
fi

if test -n "$1" && test "$1" -ne 0; then
    numruns=$1
    shift 1
else
    numruns=1
fi

run_nosetests ()
{
  echo nosetests $TESTEDMOTORAXIS "$@"
  nosetests "$@" || exit 1
}

while test $numruns -gt 0; do
  if ! caget $TESTEDMOTORAXIS.RBV >/dev/null; then
    continue
  fi
  export TESTEDMOTORADDR TESTEDMOTORAXIS TESTEDMCUASYN
  if test -n "$file"; then
    run_nosetests "$@" $file || exit 1
  else
    py=$(echo *.py | sort)
    echo py=$py
    for p in $py
    do
      run_nosetests "$@" $p || exit 1
    done
  fi
  echo Runs left=$numruns
  numruns=$(($numruns - 1))
done
