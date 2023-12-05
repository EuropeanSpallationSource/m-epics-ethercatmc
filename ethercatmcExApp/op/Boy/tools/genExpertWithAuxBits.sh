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
HAS_PTPdiffTimeIOC_MCUHIGHLOW=""
HAS_PTP=""
HAS_PTP_POS_NEG=""
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
    ptpdifftimeioc_mcuhighlow)
      HAS_PTPdiffTimeIOC_MCUHIGHLOW="y"
      shift
      ;;
    ptp)
      HAS_PTP="y"
      shift
      ;;
    ptpposneg)
      HAS_PTP_POS_NEG="y"
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
export HAS_PTPdiffTimeIOC_MCUHIGHLOW
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
  if test "$HAS_PTPdiffTimeIOC_MCUHIGHLOW" = "y"; then
    WIDTH_PTPHIGH=78
    cmd=$(echo ./shiftopi.py --shiftx 0 --shifty $y)
    echo HAS_PTPdiffTimeIOC_MCUHIGHLOW cmd=$cmd
    eval $cmd <PTPdiffTimeIOC_MCU-HIGH-LOW.mid >>$$
    cmd=$(echo ./shiftopi.py --shiftx $WIDTH_PTPHIGH --shifty $y)
    eval $cmd <PTPdiffTimeIOC_MCU-HIGH-LOW.mid |
      sed -e "s/PTPdiffTimeIOC_MCU/PTPdiffNTtime_MCU/g" >>$$
    if test "$HAS_PTP_POS_NEG" = "y"; then
      echo HAS_PTP_POS_NEG cmd=$cmd "<ptp-ts-ns-pos-neg.mid"
      eval $cmd <ptp-ts-ns-pos-neg.mid >>$$
    fi
    yaux=$(($yaux + 16))
    y=$(($y + 16))
  fi &&
  if test "$HAS_PTP" = "y"; then
    cmd=$(echo ./shiftopi.py --shiftx $PTPOPENERRBITS --shifty $y)
    echo HAS_PTP cmd=$cmd "<ptp.mid"
    eval $cmd <ptp.mid >>$$
    cmd=$(echo ./shiftopi.py --shiftx $PTPOPENERRBITS --shifty $y)
    echo HAS_PTP cmd=$cmd "<ptp-ts-ns.mid"
    eval $cmd <ptp-ts-ns.mid >>$$
    yaux=$(($yaux + 16))
    y=$(($y + 16))
  fi &&

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
