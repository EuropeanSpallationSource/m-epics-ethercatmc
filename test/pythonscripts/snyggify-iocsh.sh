#!/bin/sh

#
# Wrapper script to run snyggify-iocsh.py
#

if [ $# -lt 1 ]; then
  echo "usage $0 filename [filename]" >&2
  exit 1
fi

# debug to show how the extension .sh can be stripped further down
#  twohash=${1##*.}
#  onehash=${1#*.}
#  twoper=${1%%.*}
#  oneper=${1%.*}
#  echo onehash="$onehash"
#  echo oneper="$oneper"
#  echo twohash="$twohash"
#  echo twoper="$twoper"

SNYGGIFYPY=${0%.*}.py
for f in "$@"; do
  cp -fv "$f" /tmp/$$
  $SNYGGIFYPY </tmp/$$ >"$f"
  rm -f tmp/$$
done
