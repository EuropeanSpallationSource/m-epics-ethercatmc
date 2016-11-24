#!/bin/sh
if test -z "$1" ; then
  echo >&2 "$0" "<PV>"
  exit 1
fi
./checkws.sh &&
(
  cd nose_tests/ &&
  ./runTests.sh "$@"
)
