#!/bin/sh

# Where to find the resulting opi files
OPIS=..
# File name extension
EXT=opi

export OPIS TITLEH WIDTH HIGHT


BASENAME=$1
shift

FILE=$BASENAME.$EXT
echo "Creating $OPIS/$FILE" &&
  cat $BASENAME.start >$OPIS/$$ &&
  cat ptp.mid  >>$OPIS/$$ &&
  im=0
  x=0
  y=$TITLEH
  cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
  echo cmd=$cmd
    eval $cmd <$BASENAME.mid >>$OPIS/$$
  cat $BASENAME.end  >>$OPIS/$$ &&
  mv -f $OPIS/$$ $OPIS/$FILE

