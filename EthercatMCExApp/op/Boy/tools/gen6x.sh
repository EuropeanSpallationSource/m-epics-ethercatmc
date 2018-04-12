#!/bin/sh

OPIS=..
FILE=motor6x.opi

cp $OPIS/motorx.start $OPIS/$FILE &&
	i=0
while test "$i" -lt 6; do
	x=$(($i * 120))
	./shiftopi.py --shiftx $x --shiftm $i <$OPIS/motorx.mid >>$OPIS/$FILE
	i=$(($i + 1))
done &&
cat $OPIS/motorx.end  >>$OPIS/$FILE


	

