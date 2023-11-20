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
    yaux=436
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
HAS_ECMC=""
HAS_PTPHIGH=""
HAS_PTPLOW=""
HAS_PTP=""
HAS_TC=""
PTPOPENERRBITS=0
PARAM="$1"
while test "$PARAM" != ""; do
  case $1 in
    ecmc)
      HAS_ECMC="y"
      shift
      PARAM="$1"
      ;;
    ptphigh)
      HAS_PTPHIGH="y"
      shift
      ;;
    ptplow)
      HAS_PTPLOW="y"
      shift
      ;;
    ptp)
      HAS_PTP="y"
      shift
      ;;
    openPTPErrBits)
      PTPOPENERRBITS=20
      shift
      PARAM="$1"
      ;;
    tc)
      HAS_TC="y"
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
export HAS_ECMC
export HAS_PTP
export HAS_PTPHIGH
export HAS_PTPLOW
export HAS_TC
export PTPOPENERRBITS

im=0
x=0
y=0

echo "Creating $FILE" &&
  cat $BASENAME.start >$$ &&
  cat plcName.mid >>$$ &&
  y=$(($y + 16)) &&
  if test $PTPOPENERRBITS != 0; then
    cat openPTPErrBits.mid >>$$
  fi &&
  if test "$HAS_PTPHIGH" = "y"; then
    cmd=$(echo ./shiftopi.py --shiftx $PTPOPENERRBITS --shifty $y)
    echo HAS_PTPHIGH cmd=$cmd
    eval $cmd <ptp-high.mid >>$$
    yaux=$(($yaux + 16))
    y=$(($y + 16))
  fi &&
  if test "$HAS_PTP" = "y"; then
    cmd=$(echo ./shiftopi.py --shiftx $PTPOPENERRBITS --shifty $y)
    echo HAS_PTP cmd=$cmd
    eval $cmd <ptp.mid >>$$
    yaux=$(($yaux + 16))
    y=$(($y + 16))
  fi &&
  if test "$HAS_PTPLOW" = "y"; then
    cmd=$(echo ./shiftopi.py --shiftx $PTPOPENERRBITS --shifty $y)
    echo HAS_PTPLOW cmd=$cmd
    eval $cmd <ptp-low.mid >>$$
    yaux=$(($yaux + 16))
    y=$(($y + 16))
  fi

echo $0: FILE=$FILE BASENAME=$BASENAME rest=$@

cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im) &&
  echo $0: $BASENAME cmd=$cmd &&
  eval $cmd <$BASENAME.mid >>$$ &&
  if test "$HAS_TC" = "y"; then
    cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
    echo $0: HAS_TC cmd=$cmd
    eval $cmd <tc.mid >>$$
  fi &&
  if test "$HAS_ECMC" = "y"; then
    cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
    echo $0: HAS_TC cmd=$cmd
    eval $cmd <ecmc.mid >>$$
  fi &&
  for n in $@; do
    yaux=$(($yaux + 20))
    cmd=$(echo ./genExpertWithAuxBits.py --shiftn $n --shifty $yaux)
    echo cmd=$cmd
    eval $cmd <ethercatmcaxisAuxBit.mid | sed -e "s/StatusBits/$STATUSBITS/g" -e "s/NamAuxBit/$NAMAUXBIT/g" >>$$
  done
cat $BASENAME.end >>$$ &&
  mv -f $$ $FILE &&
  chmod -w $FILE
