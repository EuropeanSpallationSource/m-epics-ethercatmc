#!/bin/sh
FILE=$1
shift
BASENAME=$1
shift

MCUOPIS=$(ls -1 $BASENAME-MCU*.mid | sort)
OTHEROPIS=$(ls -1 $BASENAME-*.mid | grep -v $BASENAME-MCU[0-9] | sort)
echo $0: MCUOPIS=$MCUOPIS
echo $0: OTHEROPIS=$OTHEROPIS
x=0
y=0
echo "Creating $FILE" &&
  cat $BASENAME.start >/tmp/$$ &&
  for f in $MCUOPIS; do
    cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y)
    echo $0: cmd=$cmd
    eval $cmd <$f >>/tmp/$$
    y=$(($y + 36))
  done &&
  y=0 &&
  x=182 &&
  for f in $OTHEROPIS; do
    cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y)
    echo $0: cmd=$cmd
    eval $cmd <$f >>/tmp/$$
    y=$(($y + 36))
  done &&
  cat $BASENAME.end >>/tmp/$$ &&
  mv -f /tmp/$$ $FILE &&
  chmod -w $FILE
