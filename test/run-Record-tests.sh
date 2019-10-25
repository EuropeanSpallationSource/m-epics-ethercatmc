#!/bin/sh
if test -z "$1" ; then
  echo >&2 "$0" "<PV>"
  exit 1
fi
./checkws.sh &&
(
  cd pyTests/ &&
    PV="$1"
  shift
    for f in $( echo *Record*.py); do
      ./runTests.sh "$PV" "$f" "$@"
    done
)
