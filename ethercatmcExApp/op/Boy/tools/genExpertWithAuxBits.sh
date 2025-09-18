#!/bin/sh

STATUSBITS=StatusBits
NAMAUXBIT=NamBit
# Name of the destination file
FILE=$1
shift

# Name of the source file file
BASENAME=$1
shift

# Default for the majority
STATUSCODE='$(M)-StatusCode'
STATUSBITS='$(M)-StatusBits'
NAMAUXBIT='$(M)-NamAuxBit'
case $BASENAME in
  ethercatmcCabinet)
    NAMAUXBIT='$(M)-CabinetNamBit'
    yaux=18
    ;;
  ethercatmcShutter)
    STATUSCODE='$(M)StatusCode'
    STATUSBITS='$(M)StatusBits'
    NAMAUXBIT='$(M)NamAuxBit'
    yaux=298
    ;;
  ethercatmcaxisExpert)
    yaux=396
    ;;
  ethercatmcPTPErrBits)
    yaux=18
    STATUSCODE=StatusCode
    STATUSBITS=PTPErrorStatus
    NAMAUXBIT=PTPErrNamBit
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
HAS_PTPdiffNTTime_MCU=""
HAS_PTPdiffTcNTPExttime_mcu=""
HAS_PTP=""
HAS_PTP_TS_NS_POS_NEG=""
HAS_PIEZO=""
HAS_PILS=""
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
    PTPdiffNTtime_mcu)
      HAS_PTPdiffNTTime_MCU="y"
      shift
      ;;
    PTPdiffTcNTPExttime_mcu)
      HAS_PTPdiffTcNTPExttime_mcu="HAS_PTPdiffTcNTPExttime_mcu_yes"
      shift
      ;;
    ptpTCdifftime_mcu)
      HAS_PTPTCdiffTime_MCU="y"
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
    pils)
      HAS_PILS="y"
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
export HAS_PTPdiffNTTime_MCU
export HAS_PTPdiffTcNTPExttime_mcu
export HAS_PTPTCdiffTime_MCU
export HAS_PIEZO
export HAS_PILS
export PTPOPENERRBITS

im=0
x=0
y=0

echo "Creating $FILE" &&
  cat $BASENAME.start >/tmp/$$ &&
  if test "$HAS_PILS" = "y"; then
    cat plcName.mid >>/tmp/$$ &&
      y=$(($y + 16))
  fi &&
  if test $PTPOPENERRBITS != 0; then
    cat openPTPErrBits.mid >>/tmp/$$
  fi &&
  WIDTH_PTP=78
SHIFTX=$(($PTPOPENERRBITS + 282)) # PTP fields are from left to right
SHIFTY=0                          # Set to 16 further down, if any PTP is here
if test "$HAS_PTPdiffTimeIOC_MCU" = "y"; then
  ./shiftopi.py --shiftx $SHIFTX --shifty $y <PTPdiffTimeIOC_MCU-HIGH-LOW.mid >>/tmp/$$
  ./shiftopi.py --shiftx $SHIFTX --shifty $((y + 16)) <PTPdiffTimeIOC_MCU.mid >>/tmp/$$
  SHIFTX=$(($SHIFTX + $WIDTH_PTP))
  SHIFTY=16
fi &&
  if test "$HAS_PTPdiffNTTime_MCU" = "y"; then
    ./shiftopi.py --shiftx $SHIFTX --shifty $y <PTPdiffTimeIOC_MCU-HIGH-LOW.mid |
      sed -e "s/PTPdiffTimeIOC_MCU/PTPdiffNTtime_MCU/g" >>/tmp/$$
    ./shiftopi.py --shiftx $SHIFTX --shifty $((y + 16)) <PTPdiffTimeIOC_MCU.mid |
      sed -e "s/PTPdiffTimeIOC_MCU/PTPdiffNTtime_MCU/g" >>/tmp/$$
    SHIFTX=$(($SHIFTX + $WIDTH_PTP))
    SHIFTY=16
  fi &&
  if test "$HAS_PTPdiffTcNTPExttime_mcu" = "HAS_PTPdiffTcNTPExttime_mcu_yes"; then
    ./shiftopi.py --shiftx $SHIFTX --shifty $y <PTPdiffTimeIOC_MCU-HIGH-LOW.mid |
      sed -e "s/PTPdiffTimeIOC_MCU/PTPdiffTcNTPExttime_MCU/g" >>/tmp/$$
    ./shiftopi.py --shiftx $SHIFTX --shifty $((y + 16)) <PTPdiffTimeIOC_MCU.mid |
      sed -e "s/PTPdiffTimeIOC_MCU/PTPdiffTcNTPExttime_MCU/g" >>/tmp/$$
    SHIFTX=$(($SHIFTX + $WIDTH_PTP))
    SHIFTY=16
  fi &&
  if test "$HAS_PTPTCdiffTime_MCU" = "y"; then
    ./shiftopi.py --shiftx $SHIFTX --shifty $y <PTPdiffTimeIOC_MCU-HIGH-LOW.mid |
      sed -e "s/PTPdiffTimeIOC_MCU/PTPdiffTCtime_MCU/g" >>/tmp/$$
    ./shiftopi.py --shiftx $SHIFTX --shifty $((y + 16)) <PTPdiffTimeIOC_MCU.mid |
      sed -e "s/PTPdiffTimeIOC_MCU/PTPdiffTCtime_MCU/g" >>/tmp/$$
    SHIFTX=$(($SHIFTX + $WIDTH_PTP))
    SHIFTY=16
  fi &&
  if test "$HAS_PTP_TS_NS_POS_NEG" = "y"; then
    ./shiftopi.py --shiftx $SHIFTX --shifty $y <ptp-ts-ns-pos-neg.mid >>/tmp/$$
    ./shiftopi.py --shiftx $SHIFTX --shifty $((y + 16)) <ptp-ts-ns.mid >>/tmp/$$
    SHIFTX=$(($SHIFTX + $WIDTH_PTP))
    SHIFTY=16
  fi &&
  yaux=$(($yaux + $SHIFTY)) # shift one line from high to have PTP in the middle
y=$(($y + $SHIFTY))         # shift one line from high to have PTP in the middle
if test "$HAS_PTP" = "y"; then
  ./shiftopi.py --shiftx $PTPOPENERRBITS --shifty $y <ptp.mid >>/tmp/$$
  SHIFTY=16
fi &&
  yaux=$(($yaux + $SHIFTY)) # shift one line from low to have PTP in the middle
y=$(($y + $SHIFTY))         # shift one line from low to have PTP in the middle

echo $0: FILE=$FILE BASENAME=$BASENAME rest=$@
ETHERCATMCAXISCONFIG_OPI=ethercatmcaxisConfig-pils.opi
cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im) &&
  echo $0: $BASENAME cmd=$cmd &&
  eval $cmd <$BASENAME.mid >>/tmp/$$ &&
  if test "$HAS_PIEZO" = "y"; then
    cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
    echo $0: HAS_PIEZO cmd=$cmd
    eval $cmd <e-cnen.mid >>/tmp/$$
    eval $cmd <e-foff.mid >>/tmp/$$
    eval $cmd <e-homf-homr.mid >>/tmp/$$
    eval $cmd <e-retry.mid >>/tmp/$$
    eval $cmd <e-URIP.mid >>/tmp/$$
    # Create the status bits
    n=0
    for NAMEPSTATBIT in $@; do
      echo NAMEPSTATBIT=$NAMEPSTATBIT
      yaux=$(($yaux + 20))
      cmd=$(echo ./genExpertWithAuxBits.py --shiftn $n --shifty $yaux)
      echo cmd=$cmd
      eval $cmd <ethercatmcaxisPiezoPSTAT.mid | sed -e "s/StatusBits/$STATUSBITS/g" -e "s/NamPstatBitXXX/$NAMEPSTATBIT/g" >>/tmp/$$
      shift
      n=$(($n + 1))
    done
  elif test "$HAS_PILS" = "y"; then
    cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
    echo $0: HAS_PILS cmd=$cmd
    eval $cmd <e-cnen-vis.mid >>/tmp/$$
    eval $cmd <e-foff-vis.mid >>/tmp/$$
    eval $cmd <e-homf-homr-vis.mid >>/tmp/$$
    eval $cmd <e-homproc-rb.mid >>/tmp/$$
    eval $cmd <e-inhibitf-inhibitr.mid >>/tmp/$$
    eval $cmd <e-pils-cfgdxlm-en.mid >>/tmp/$$
    eval $cmd <e-pils-status-bit24-25.mid >>/tmp/$$
    yaux=$((yaux + 40))
    eval $cmd <e-pils-statuscode.mid >>/tmp/$$
    eval $cmd <e-errrst.mid >>/tmp/$$
    eval $cmd <e-homproc-rb.mid >>/tmp/$$
    eval $cmd <e-vel-acc-RB-act.mid >>/tmp/$$
  elif test "$HAS_ECMC" = "y"; then
    cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
    echo $0: HAS_ECMC cmd=$cmd
    eval $cmd <e-ecmc.mid >>/tmp/$$
    eval $cmd <e-errrst.mid >>/tmp/$$
    eval $cmd <e-homproc-rb.mid >>/tmp/$$
    eval $cmd <e-vel-acc-RB-act.mid >>/tmp/$$
  elif test "$BASENAME" = "ethercatmcaxisExpert"; then
    # The old etthercatmc (no pils)
    ETHERCATMCAXISCONFIG_OPI=ethercatmcaxisConfig.opi
    cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
    echo $0: Neither_PILS_NOR_ECMC cmd=$cmd
    eval $cmd <e-cnen-vis.mid >>/tmp/$$
    eval $cmd <e-foff-vis.mid >>/tmp/$$
    eval $cmd <e-homf-homr-vis.mid >>/tmp/$$
  fi &&
  for n in $@; do
    yaux=$(($yaux + 20))
    cmd=$(echo ./genExpertWithAuxBits.py --shiftn $n --shifty $yaux)
    echo cmd=$cmd
    eval $cmd <ethercatmcaxisAuxBit.mid | sed -e "s/StatusBits/$STATUSBITS/g" -e "s/NamBit/$NAMAUXBIT/g" -e "s/StatusCode/$STATUSCODE/g" >>/tmp/$$
  done
touch "$FILE" &&
  chmod +w "$FILE" &&
  cat $BASENAME.end >>/tmp/$$ &&
  sed -e "s!ethercatmcaxisConfig-pils.opi!$ETHERCATMCAXISCONFIG_OPI!" </tmp/$$ >"$FILE"
chmod -w "$FILE"
