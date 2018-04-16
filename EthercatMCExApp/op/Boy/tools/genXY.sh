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
X="$1"
FILE=motor${X}x.opi

if test -n "$2"; then
	if test "$2" -eq 0; then
		echo >&2 "$0 <numberOfMotorsY>"
		exit 1
	fi
	Y=$2
	FILE=motor${X}x${Y}.opi
fi

cp $OPIS/motorx.start $OPIS/$FILE &&
iy=0
while test $iy -lt $Y; do
	ix=0
	y=$(($iy * 173))
	while test $ix -lt $X; do
		x=$(($ix * 120))
		im=$(($ix + ($iy * X)))
		cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
		echo cmd=$cmd
		eval $cmd <$OPIS/motorx.mid >>$OPIS/$FILE
		ix=$(($ix + 1))
	done &&
  iy=$(($iy + 1))
done &&
	cat $OPIS/motorx.end  >>$OPIS/$FILE


	

