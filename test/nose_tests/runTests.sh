#!/bin/sh
#
# Wrapper scipt to run tests
# Usage:
# ./runTests.sh <PV> [test.py]
#
# Examples:
#
# run all tests on a motor PV
# ./runtests.sh IOC:m1
#
# run some tests on a motor PV
# ./runtests.sh IOC:m1 100*.py 12*.py
#
# run specif test on a motor PV a couple of times
# ./runTests.sh IOC:m1 100_Record-HOMF.py 4


# First of all, check for whitespace damage (TAB, trailing WS
../checkws.sh || {
  echo >&2   ../checkws.sh failed
  exit 1
}

#
PYTEST=pytest
if which python3.7 >/dev/null 2>&1; then
  PYTHON=python3.7
elif which python36 >/dev/null 2>&1; then
  PYTHON=python37
elif which python3.6 >/dev/null 2>&1; then
  PYTHON=python3.6
elif which python36 >/dev/null 2>&1; then
  PYTHON=python36
elif which python3.4 >/dev/null 2>&1; then
  PYTHON=python3.4
  # need $ pip install "pytest<5"
  PYTEST="pytest<5"
else
  echo >&2 "No pyton 3.7, 3.6, 36 or 3.4 found"
  exit 1
fi

##############################################################################
# functions
#
#
checkAndInstallSystemPackage()
{
  BINARYNAME=$1
  PACKAGENAME=$2
  if ! which $BINARYNAME; then
    if which yum >/dev/null 2>&1; then
      sudo yum install $PACKAGENAME || {
        echo >&2 failed: sudo yum install $PACKAGENAME
        exit 1
      }
    fi
    if which apt-get >/dev/null 2>&1; then
      sudo apt-get install $PACKAGENAME || {
        echo >&2 failed: sudo apt-get install $PACKAGENAME
        exit 1
      }
    fi
  fi
}

########################################
checkAndInstallPythonPackage()
{
  IMPORTNAME=$1
  PACKAGENAME=$2

  if ! $PYTHON -c "import $IMPORTNAME" >/dev/null 2>&1; then
    $PYTHON -c "import $IMPORTNAME"
    echo failed: $PYTHON -c "import $IMPORTNAME"
    if which pip3 >/dev/null 2>&1; then
      pip3 install $PACKAGENAME || sudo pip3 install $PACKAGENAME || {
          echo >&1 pip3 install $PACKAGENAME failed
          exit 1
        }
      return
    fi
    #checkAndInstallSystemPackage pip python-pip
    #if which pip >/dev/null 2>&1; then
    #  sudo pip install $PACKAGENAME
    #fi
    #return 0
    #if which easy_install >/dev/null 2>&1; then
    #  sudo easy_install -U $PACKAGENAME
    #else
    #  echo >&2 "neither 'easy_install' nor 'pip' are found"
    #  exit 1
    #fi
  fi
}
##############################################################################

# Set up a virtual environment with python 3.x
VIRTUALENVDIR=virtual$PYTHON
if ! test -d $VIRTUALENVDIR; then
    checkAndInstallSystemPackage virtualenv python-virtualenv
    if ! type virtualenv >/dev/null 2>&1; then
        if ! which yum >/dev/null 2>&1; then
            # centos has yum, but no virtualenv
            echo >&2 virtualenv not found.
            exit 1
        fi
    fi
    if type virtualenv >/dev/null 2>&1; then
      virtualenv --python=$PYTHON $VIRTUALENVDIR || {
        echo >&2 virtualenv failed
        exit 1
      }
    fi
fi
if test -r $VIRTUALENVDIR/bin/activate; then
  .  $VIRTUALENVDIR/bin/activate
fi

# See if we have a local EPICS installation
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
    else
      MYLIB=$EPICS_BASE/lib/$EPICS_HOST_ARCH/libca.dylib
      if test -r "$MYLIB"; then
        PYEPICS_LIBCA=$MYLIB
        export PYEPICS_LIBCA
      fi
    fi
fi &&

checkAndInstallSystemPackage virtualenv python-virtualenv


# See if we have pyepics
checkAndInstallPythonPackage epics pyepics &&
checkAndInstallPythonPackage pytest $PYTEST || {
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

run_pytest ()
{
  echo $PYTHON $VIRTUALENVDIR/bin/pytest $TESTEDMOTORAXIS "$@"
  $PYTHON $VIRTUALENVDIR/bin/pytest "$@" || exit 1
}

if test -z "$EPICS_CA_ADDR_LIST" && test -z "$EPICS_CA_AUTO_ADDR_LIST"; then
  if EPICS_CA_ADDR_LIST=127.0.1 EPICS_CA_AUTO_ADDR_LIST=NO caget $TESTEDMOTORAXIS.RBV >/dev/null 2>&1; then
    EPICS_CA_ADDR_LIST=127.0.1
    EPICS_CA_AUTO_ADDR_LIST=NO
    export EPICS_CA_ADDR_LIST EPICS_CA_AUTO_ADDR_LIST
  fi
fi

while test $numruns -gt 0; do
  if ! caget $TESTEDMOTORAXIS.RBV >/dev/null 2>/dev/null; then
    continue
  fi
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
