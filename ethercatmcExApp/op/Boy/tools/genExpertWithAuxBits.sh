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



# pick up all arguments
HAS_PTP=""
PTPRWOFFSET=""
PTPOPENERRBITS=0
PARAM="$1"
while test "$PARAM" != ""; do
  case $1 in
  ptp)
    HAS_PTP="-ptp"
    shift
    PARAM="$1"
    ;;
  openPTPErrBits)
    PTPOPENERRBITS=20
    shift
    PARAM="$1"
    ;;
  ptprwoffset)
    PTPRWOFFSET="y"
    shift
    PARAM="$1"
    ;;
  [0-9]*)
    # stop the loop
    PARAM=""
    ;;
  *)
    echo >&2 "illegal option: $1"
    exit 1
    ;;
  esac
done
export HAS_PTP
export PTPRWOFFSET
export PTPOPENERRBITS

im=0
x=0
y=0

echo "Creating $FILE" &&
cat $BASENAME.start >$$ &&
cat plcName.mid  >>$$ &&
if test $PTPOPENERRBITS != 0; then
  cat openPTPErrBits.mid >>$$
fi
if test "$HAS_PTP" != ""; then
  cmd=$(echo ./shiftopi.py --shiftx $PTPOPENERRBITS --shifty $y)
  echo HAS_PTP cmd=$cmd
  eval $cmd <ptp.mid >>$$
  yaux=$(($yaux + 16))
  y=$(($y + 16))
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

