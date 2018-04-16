#!/bin/sh

# Number of motors in Y direction
Y=1
OPIS=..

if test -z "$1"; then
  echo >&2 "$0 <numberOfMotorsX>"
  exit 1
fi
if test "$1" -eq 0; then
  echo >&2 "$0 <numberOfMotorsX>"
  exit 1
fi
X=$1
shift
FILE=motor${X}x.opi

#Do we have e.g. 4 x 3
if test "$1" = x; then
  shift
  if test -n "$1"; then
    if test "$1" -eq 0; then
      echo >&2 "$0 <numberOfMotorsY>"
      exit 1
    fi
    Y=$1
		shift
    FILE=motor${X}x${Y}.opi
  fi
fi

cp motorx.start $OPIS/$$ &&
iy=0
im=0
while test $iy -lt $Y; do
  ix=0
  y=$(($iy * 173))
  while test $ix -lt $X; do
    x=$(($ix * 120))
    cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
    echo cmd=$cmd
    eval $cmd <motorx.mid >>$OPIS/$$
    im=$(($im + 1))
    ix=$(($ix + 1))
  done &&
  iy=$(($iy + 1))
done &&
	cat motorx.end  >>$OPIS/$$ &&
	mv -f $OPIS/$$ $OPIS/$FILE



  

