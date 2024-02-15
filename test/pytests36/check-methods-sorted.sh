#!/bin/sh

#
# Shell script to check if all methods inside a class are
# sorted alphabetically
#

files=$(git ls-files 'A*.py')
if test -n "$files"; then
  for file in $files; do
    if ! test -s $file; then
      continue
    fi
    rm -f /tmp/$$sorted /tmp/$$unsorted
    if grep -q "class Test:" $file; then
      continue
    fi
    grep "^    def" $file >/tmp/$$unsorted
    LC_ALL=C sort </tmp/$$unsorted >/tmp/$$sorted
    diff /tmp/$$sorted /tmp/$$unsorted
    res=$?
    if test $res != 0; then
      echo >&2 $file is not sorted
      exit 1
    fi
    rm /tmp/$$sorted /tmp/$$unsorted
  done
fi
