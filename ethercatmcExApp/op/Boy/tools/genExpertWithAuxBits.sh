#!/bin/sh

export OPIS

# Name of the destination file
FILE=$1
shift

# Name of the source file file
BASENAME=$1
shift

# ptp version or not
if test "$1" = "ptp"; then
  shift
  HAS_PTP="-ptp"
else
  HAS_PTP=""
fi
export HAS_PTP


echo "Creating $FILE" &&
cat $BASENAME.start >$$ &&
if test "$HAS_PTP" != ""; then
  cat ptp.mid  >>$$
fi &&
im=0
x=0
y=$TITLEH

echo $0: FILE=$FILE BASENAME=$BASENAME rest=$@

cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
echo cmd=$cmd
eval $cmd <$BASENAME.mid >>$$
  y=416 #start here
  for n in $@; do
      y=$(($y + 20))
      cmd=$(echo ./genExpertWithAuxBits.py --shiftn $n --shifty $y)
      echo cmd=$cmd
      eval $cmd <ethercatmcaxisAuxBit.mid >>$$
  done
  cat $BASENAME.end  >>$$ &&
  mv -f $$ $FILE

