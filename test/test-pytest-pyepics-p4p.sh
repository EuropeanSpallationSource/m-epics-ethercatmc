#!/bin/sh

set -x

checkPythonPackage()
{
  IMPORTNAME=$1
  python -c "import $IMPORTNAME"
}

printVersionIfPossible()
{
  PROGRAM=$1
  if type "$PROGRAM"; then
    "$PROGRAM" --version || :
  fi
}

printVersionIfPossible python
printVersionIfPossible python2
printVersionIfPossible python3
printVersionIfPossible pip
printVersionIfPossible pip3
printVersionIfPossible pytest


checkPythonPackage pytest &&
checkPythonPackage epics &&
checkPythonPackage p4p
