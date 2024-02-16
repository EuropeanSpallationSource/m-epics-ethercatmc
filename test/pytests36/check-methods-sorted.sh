#!/bin/sh

#
# Shell script to check if all methods inside a class are
# sorted alphabetically
#
LC_ALL=C
export LC_ALL

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
    grep "^    def" $file |
      sed -e "s/__\([a-zA-Z]*\)__/@@\1@@/g" >/tmp/$$unsorted
    sed -e "s/__\([a-zA-Z]*\)__/@@\1@@/g" </tmp/$$unsorted |
      sort -f  >/tmp/$$sorted
    diff /tmp/$$sorted /tmp/$$unsorted
    res=$?
    if test $res != 0; then
      echo >&2 $file is not sorted
      cat /tmp/$$sorted | sed -e "s/@@\([a-zA-Z]*\)@@/__\1__/g"
      rm /tmp/$$sorted /tmp/$$unsorted
      exit 1
    fi
    rm /tmp/$$sorted /tmp/$$unsorted
  done
fi
