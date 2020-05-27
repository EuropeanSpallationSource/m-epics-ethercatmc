#!/bin/sh
if test -z "$1" ; then
  echo >&2 "$0" "<PV>"
  exit 1
fi
./checkws.sh || {
  echo >&2   $OLDPWD/checkws.sh failed
  exit 1
}
(
  cd pytests36/ &&
    PV="$1"
    shift
    ./runTests.sh "$PV" "$f" "$@"
)
