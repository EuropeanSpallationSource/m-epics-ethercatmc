#!/bin/bash
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

if test -e $HOME/.bash_profile; then
   . $HOME/.bash_profile
fi

# First of all, check for whitespace damage (TAB, trailing WS
../checkws.sh || {
  echo >&2   ../checkws.sh failed
  exit 1
}

# Those values should work as default
# They may be overwrtitten further down
PYTEST=pytest
PYTHON=python3

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
        return 1
      }
    fi
    if which apt-get >/dev/null 2>&1; then
      sudo apt-get install $PACKAGENAME || {
        echo >&2 failed: sudo apt-get install $PACKAGENAME
        return 1
      }
    fi
    return 1
  fi
}

########################################
checkAndInstallPythonPackage()
{
  IMPORTNAME=$1

  if ! python -c "import $IMPORTNAME" >/dev/null 2>&1; then
    while test $# -gt 1; do
      shift
      PACKAGEINSTALL=$1
      echo failed: $PYTHON -c "import $IMPORTNAME"
      $PACKAGEINSTALL && return 0
    done
    echo >&1  $PACKAGEINSTALL failed
    exit 1
  fi
}
##############################################################################
if ! which conda >/dev/null 2>&1; then
  # This is Mac OS
  #URL=https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-x86_64.sh
  #wget $URL || {
    #echo >&2 wget $URL failed
    #exit 1
  #}
  # conda is preferred over virtualenv
  # But: if we have virtualenv, use it
  if ! which virtualenv; then
    checkAndInstallSystemPackage conda anaconda || {
      # conda installation failed, fall back to virtualenv
      checkAndInstallSystemPackage virtualenv python-virtualenv || {
        echo >2 "could not install virtualenv"
        exit 1
      }
    }
  fi
fi


##############################################################################
if which conda >/dev/null 2>&1; then
  conda activate pyepicsPytestPVApy || {
    echo >&2 conda activate pyepicsPytestPVApy failed
    conda create -n  pyepicsPytestPVApy
    exit 1
  }
  checkAndInstallPythonPackage pytest "conda install -c conda-forge pyTest"
  checkAndInstallPythonPackage epics  "conda install -c https://conda.anaconda.org/GSECARS pyepics" "conda install pyepics"
else
  if ! type virtualenv >/dev/null 2>&1; then
    echo >&2 virtualenv not found.
    exit 1
  fi
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
  VIRTUALENVDIR=virtual$PYTHON
  if test -r $VIRTUALENVDIR/bin/activate; then
    .  $VIRTUALENVDIR/bin/activate
  else
    virtualenv --python=$PYTHON $VIRTUALENVDIR || {
      echo >&2 virtualenv failed
      exit 1
    }
  fi
  if test -r $VIRTUALENVDIR/bin/activate; then
    .  $VIRTUALENVDIR/bin/activate
  fi
  checkAndInstallPythonPackage epics "pip3 install pyepics" "pip install pyepics" &&
    checkAndInstallPythonPackage pytest "pip3 install $PYTEST" "pip install $PYTEST" || {
      echo >&2 Installation problem:
      echo >&2 pip not found
      echo >&2 easy_install not found
      exit 1
    }
fi

checkAndInstallPythonPackage p4p "pip3 install p4p" "pip install p4p"


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
  echo CONDA_PREFIX="$CONDA_PREFIX"
  if test "$CONDA_PREFIX"; then
      echo pytest "$@"
      pytest "$@"
  else
    echo $PYTHON $VIRTUALENVDIR/bin/pytest "$@"
    $PYTHON $VIRTUALENVDIR/bin/pytest "$@" || exit 1
  fi
}

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
