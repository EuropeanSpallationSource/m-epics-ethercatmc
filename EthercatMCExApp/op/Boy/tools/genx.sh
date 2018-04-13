#!/bin/sh

if test -z "$1"; then
	echo >&2 "$0 <numberOfMotors>"
	exit 1
fi
if test "$1" -eq 0; then
	echo >&2 "$0 <numberOfMotors>"
	exit 1
fi
N="$1"
OPIS=..
FILE=motor${N}x.opi

cp $OPIS/motorx.start $OPIS/$FILE &&
i=0
while test $i -lt $N; do
	x=$(($i * 120))
	./shiftopi.py --shiftx $x --shiftm $i <$OPIS/motorx.mid >>$OPIS/$FILE
	i=$(($i + 1))
done &&
cat $OPIS/motorx.end  >>$OPIS/$FILE


	

