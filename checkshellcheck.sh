#!/bin/sh
if type git >/dev/null 2>&1; then
  git ls-files "*.sh" |
    grep -v "ethercatmcExApp/op" |
    xargs shellcheck --format=gcc -x
else
  find . -name "*.sh" |
    grep -v -E "ethercatmcExApp/op|.git" |
    xargs shellcheck --format=gcc -x
fi
