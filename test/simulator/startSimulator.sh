#!/bin/sh
# shellcheck disable=SC2086
uname_S=$(uname -s 2>/dev/null || echo unknown)
uname_M=$(uname -m 2>/dev/null || echo unknown)
uname_R=$(uname -r 2>/dev/null | sed -e "s/[()/]/-/g" || echo unknown)

binary=simMotor
valg=

if test "$1" = "--valgrind"; then
  if which valgrind >/dev/null 2>/dev/null; then
    valg='valgrind  --leak-check=full   --show-reachable=yes'
  fi
  shift
fi
make && $valg ${uname_S}_${uname_M}_${uname_R}/$binary "$@"
