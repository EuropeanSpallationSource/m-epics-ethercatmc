#!/bin/bash
# shellcheck disable=SC1090
# shellcheck disable=SC1091
# shellcheck disable=SC2086
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
  echo >&2 ../checkws.sh failed
  exit 1
}

if ! test "$1"; then
  # get the help from doRunTests.sh
  ./doRunTests.sh
  exit 1
fi

##############################################################################
# functions
#
#
checkAndInstallSystemPackage() {
  while test $# -gt 0; do
    PACKAGENAME=$1
    shift
    if which yum >/dev/null 2>&1; then
      sudo yum install $PACKAGENAME && return 0
    fi
    if which apt-get >/dev/null 2>&1; then
      sudo apt-get install -y $PACKAGENAME && return 0
    fi
    if which port >/dev/null 2>&1; then
      sudo port install $PACKAGENAME && return 0
    fi
  done
  echo >&1 install $PACKAGENAME failed
  return 1
}

########################################
checkAndInstallPythonPackage() {
  IMPORTNAME=$1

  if ! python -c "import $IMPORTNAME" >/dev/null 2>&1; then
    while test $# -gt 1; do
      shift
      PACKAGEINSTALL=$1
      echo failed: $PYTHON -c "import $IMPORTNAME"
      $PACKAGEINSTALL && return 0
    done
    echo >&1 $PACKAGEINSTALL failed
    exit 1
  fi
}
########################################

if which virtualenv-3.9 >/dev/null 2>&1; then
  MYVIRTUALENV=virtualenv-3.9
elif which virtualenv-3.8 >/dev/null 2>&1; then
  MYVIRTUALENV=virtualenv-3.8
elif which virtualenv-3.7 >/dev/null 2>&1; then
  MYVIRTUALENV=virtualenv-3.7
elif which virtualenv-3.6 >/dev/null 2>&1; then
  MYVIRTUALENV=virtualenv-3.6
elif which pyenv-virtualenv >/dev/null 2>&1; then
  # brew has pyenv-virtualenv
  # and a bug, "pyenv-root" should be written as "pyenv root"
  PYENV_ROOT="$(pyenv root)"
  export PYENV_ROOT
  MYVIRTUALENV="pyenv virtualenv"
  if test -e /usr/local/opt/python@3.7/bin; then
    PATH=/usr/local/opt/python@3.7/bin:$PATH
    export PATH
  fi
elif type virtualenv >/dev/null 2>&1; then
  MYVIRTUALENV=virtualenv
fi

echo MYVIRTUALENV=$MYVIRTUALENV
export MYVIRTUALENV

# There must be a better way to do this
# shellcheck disable=SC2154
if test "$ImageOS" = ubuntu20; then
  ./doRunTests.sh "$@"
  exit
fi

########################################
if test -z "$MYVIRTUALENV"; then
  echo no VIRTUALENV found, trying conda
  if which conda >/dev/null 2>&1; then
    if test -n "$CONDA_PROMPT_MODIFIER"; then
      #echo "We use activated $CONDA_PYTHON_EXE"
      echo "We use activated $CONDA_PROMPT_MODIFIER"
      checkAndInstallPythonPackage pytest "conda install pyTest"
      checkAndInstallPythonPackage epics "conda install pyepics"
    else
      echo >&2 "run:"
      if test -d ~/.conda/envs/pyepicsPytestPVApy; then
        echo >&2 "conda activate pyepicsPytestPVApy"
      else
        echo >&2 "conda create -n pyepicsPytestPVApy"
      fi
      exit 1
    fi
  fi
fi
########################################
PYTEST=pytest
PYTHON=python3
if ! type pytest >/dev/null 2>&1; then
  # more things to do, either conda or virtualenv is our friend
  if test -e $HOME/.bash_profile; then
    . $HOME/.bash_profile
  fi

  # Those values should work as default
  # They may be overwrtitten further down
fi

##############################################################################
if test -n "$MYVIRTUALENV" && type $MYVIRTUALENV >/dev/null 2>&1; then
  if which python3.9 >/dev/null 2>&1; then
    PYTHON=python3.9
  elif which python3.8 >/dev/null 2>&1; then
    PYTHON=python3.8
  elif which python3.7 >/dev/null 2>&1; then
    PYTHON=python3.7
  elif which python36 >/dev/null 2>&1; then
    PYTHON=python36
  elif which python3.6 >/dev/null 2>&1; then
    PYTHON=python3.6
  elif which python3.5 >/dev/null 2>&1; then
    PYTHON=python3.5
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
  VIRTUALENVDIR=venv$PYTHON
  if test -d $HOME/.pyenv/versions/$VIRTUALENVDIR/bin/; then
    VIRTUALENVDIR=$HOME/.pyenv/versions/$VIRTUALENVDIR
  fi
  if test -r $VIRTUALENVDIR/bin/activate; then
    . $VIRTUALENVDIR/bin/activate
  elif test -z "$MYVIRTUALENV"; then
    checkAndInstallSystemPackage py37-virtualenv virtualenv python-virtualenv || {
      echo >&2 "could not install virtualenv"
    }
    echo >&2 "virtualenv has been installed"
    echo >&2 "Re-run the script"
    exit 1
  else
    $MYVIRTUALENV --python=$PYTHON $VIRTUALENVDIR || {
      echo >&2 $MYVIRTUALENV failed
      exit 1
    }
  fi
  if test -r $VIRTUALENVDIR/bin/activate; then
    . $VIRTUALENVDIR/bin/activate
  fi
else
  if which conda >/dev/null 2>&1; then
    checkAndInstallPythonPackage pytest "conda install -c conda-forge pyTest"
    checkAndInstallPythonPackage epics "conda install -c https://conda.anaconda.org/GSECARS pyepics" "conda install pyepics"
  fi
fi

if ! which pip >/dev/null; then
  checkAndInstallSystemPackage pip
fi

checkAndInstallPythonPackage epics "pip3 install pyepics" "pip install pyepics" &&
  checkAndInstallPythonPackage p4p "pip3 install p4p" "pip install p4p"
checkAndInstallPythonPackage pytest "pip3 install $PYTEST" "pip install $PYTEST" || {
  echo >&2 Installation problem:
  echo >&2 pip not found
  echo >&2 easy_install not found
  exit 1
}

checkAndInstallPythonPackage p4p "pip3 install p4p" "pip install p4p"

#Check black, the python formatter
BLACK_VERSION=23.9.1
if ! black --version | grep -q "[^0-9]${BLACK_VERSION}[^0-9]"; then
  pip install git+https://github.com/psf/black@$BLACK_VERSION
fi
if ! black --version | grep -q "[^0-9]${BLACK_VERSION}[^0-9]"; then
  echo >&2 black not found or wrong version
  exit 1
fi
# shellcheck disable=SC2035
black *.py

#Check ruff, the fast Python linter and code formatter
RUFF_VERSION=0.1.7
if ! ruff --version | grep -q "[^0-9]${RUFF_VERSION}$"; then
  echo >&2 ruff not found or wrong version
  exit 1
fi
# shellcheck disable=SC2035
ruff *.py

# See if we have a local EPICS installation
uname_s=$(uname -s 2>/dev/null || echo unknown)
uname_m=$(uname -m 2>/dev/null || echo unknown)
INSTALLED_EPICS=../../../.epics.$(hostname).$uname_s.$uname_m
if test -r $INSTALLED_EPICS; then
  echo INSTALLED_EPICS=$INSTALLED_EPICS
  . $INSTALLED_EPICS
fi

export VIRTUALENVDIR
export CONDA_SYSPFX

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
  echo ./doRunTests.sh "$@"
./doRunTests.sh "$@"
