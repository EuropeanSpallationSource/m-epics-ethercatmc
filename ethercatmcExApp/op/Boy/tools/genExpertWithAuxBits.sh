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
HAS_PTPdiffTimeIOC_MCU=""
HAS_PTPNTdiffTime_MCU=""
HAS_PTP=""
HAS_PTP_TS_NS_POS_NEG=""
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
    ptpdifftimeioc_mcu)
      HAS_PTPdiffTimeIOC_MCU="y"
      shift
      ;;
    ptpNTdifftime_mcu)
    HAS_PTPNTdiffTime_MCU="y"
      shift
      ;;
    ptp)
      HAS_PTP="y"
      shift
      ;;
    ptptsnsposneg)
      HAS_PTP_TS_NS_POS_NEG="y"
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
export HAS_PTPdiffTimeIOC_MCU
export HAS_PTPNTdiffTime_MCU
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
  WIDTH_PTP=78
  SHIFTX=$(($PTPOPENERRBITS + 282)) # PTP fields are from left to right
  SHIFTY=0 # Set to 16 further down, if any PTP is here
  if test "$HAS_PTPdiffTimeIOC_MCU" = "y"; then
    ./shiftopi.py --shiftx $SHIFTX --shifty $y <PTPdiffTimeIOC_MCU-HIGH-LOW.mid >>$$
    ./shiftopi.py --shiftx $SHIFTX --shifty $((y + 16)) <PTPdiffTimeIOC_MCU.mid >>$$
    SHIFTX=$(($SHIFTX + $WIDTH_PTP))
    SHIFTY=16
  fi &&
  if test "$HAS_PTPNTdiffTime_MCU" = "y"; then
    ./shiftopi.py --shiftx $SHIFTX --shifty $y <PTPdiffTimeIOC_MCU-HIGH-LOW.mid |
      sed -e "s/PTPdiffTimeIOC_MCU/PTPdiffNTtime_MCU/g" >>$$
    ./shiftopi.py --shiftx $SHIFTX --shifty $((y + 16)) <PTPdiffTimeIOC_MCU.mid |
      sed -e "s/PTPdiffTimeIOC_MCU/PTPdiffNTtime_MCU/g" >>$$
    SHIFTX=$(($SHIFTX + $WIDTH_PTP))
    SHIFTY=16
  fi &&
  if test "$HAS_PTP_TS_NS_POS_NEG" = "y"; then
    ./shiftopi.py --shiftx $SHIFTX --shifty $y <ptp-ts-ns-pos-neg.mid >>$$
    ./shiftopi.py --shiftx $SHIFTX --shifty $((y + 16)) <ptp-ts-ns.mid >>$$
    SHIFTX=$(($SHIFTX + $WIDTH_PTP))
    SHIFTY=16
  fi &&
  yaux=$(($yaux + $SHIFTY)) # shift one line from high to have PTP in the middle
  y=$(($y + $SHIFTY))       # shift one line from high to have PTP in the middle
  if test "$HAS_PTP" = "y"; then
    ./shiftopi.py --shiftx $PTPOPENERRBITS --shifty $y <ptp.mid >>$$
    SHIFTY=16
  fi &&
  yaux=$(($yaux + $SHIFTY)) # shift one line from low to have PTP in the middle
  y=$(($y + $SHIFTY))       # shift one line from low to have PTP in the middle

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
