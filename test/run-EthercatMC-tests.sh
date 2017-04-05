#!/bin/sh
if test -z "$1" ; then
  echo >&2 "$0" "<PV>"
  exit 1
fi
./checkws.sh &&
(
  OLDPWD=$PWD
  cd ../axisApp/EthercatSrc/ &&
  $OLDPWD/checkws.sh
) || {
  echo >&2   $OLDPWD/checkws.sh failed
  exit 1
}
(
  cd nose_tests/ &&
    PV="$1"
    shift
    TESTS=$(ls -1  *Record*.py *Ethercat*.py | sort -n)
    for f in $TESTS; do
      ./runTests.sh "$PV" "$f" "$@"
    done
)
