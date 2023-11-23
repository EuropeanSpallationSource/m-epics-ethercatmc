#!/bin/sh
find test -name "*.sh" |
  grep -v checkws.sh |
  xargs shellcheck --format=gcc -x
