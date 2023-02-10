#!/bin/sh

STATUSBITS=StatusBits
NAMAUXBIT=NamAuxBit
# Name of the destination file
FILE=$1
shift

# Name of the source file file
BASENAME=$1
shift

case $BASENAME in
  ethercatmcShutter)
    yaux=298
    ;;
  ethercatmcaxisExpert)
    yaux=416
    ;;
  ethercatmcPTPErrBits)
    yaux=18
    STATUSBITS=PTPErrorStatus
    NAMAUXBIT=PTPErrBitNam
    ;;
  ethercatmcStatusWord1802)
    yaux=18
    ;;
  *)
    echo >&2 "invalid: $BASENAME"
    exit 1
    ;;
esac



# open ptp aux
if test "$1" = "openPTPErrBits"; then
  shift
  PTPSHIFTX=20
else
  PTPSHIFTX=0
fi
export PTPSHIFTX


# ptp version or not
if test "$1" = "ptp"; then
  shift
  HAS_PTP="-ptp"
else
  HAS_PTP=""
fi
export HAS_PTP

if test "$1" = "ptprwoffset"; then
  shift
  PTPRWOFFSET="y"
else
  PTPRWOFFSET=""
fi
export PTPRWOFFSET


im=0
x=0
y=$TITLEH

echo "Creating $FILE" &&
cat $BASENAME.start >$$ &&
cat plcName.mid  >>$$ &&
if test $PTPSHIFTX != 0; then
  cat openPTPErrBits.mid >>$$
fi
if test "$HAS_PTP" != ""; then
  cmd=$(echo ./shiftopi.py --shiftx $PTPSHIFTX --shifty $yaux)
  echo HAS_PTP cmd=$cmd
  eval $cmd <ptp.mid >>$$
  yaux=$(($yaux + 16))
fi &&

if test "$PTPRWOFFSET" = "y"; then
  yaux=$(($yaux + 6))
  cmd=$(echo ./shiftopi.py --shiftx 15 --shifty $yaux)
  echo PTPRWOFFSET cmd=$cmd
  eval $cmd <ptp_rw_offset_do_synch_check.mid >>$$
  yaux=$(($yaux + 34))
fi &&

echo $0: FILE=$FILE BASENAME=$BASENAME rest=$@

cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
echo cmd=$cmd
eval $cmd <$BASENAME.mid >>$$
  for n in $@; do
      yaux=$(($yaux + 20))
      cmd=$(echo ./genExpertWithAuxBits.py --shiftn $n --shifty $yaux)
      echo cmd=$cmd
      eval $cmd <ethercatmcaxisAuxBit.mid | sed -e "s/StatusBits/$STATUSBITS/g" -e "s/NamAuxBit/$NAMAUXBIT/g">>$$
  done
  cat $BASENAME.end  >>$$ &&
  mv -f $$ $FILE &&
  chmod -w $FILE

