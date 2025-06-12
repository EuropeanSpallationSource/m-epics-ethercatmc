#!/bin/sh

echo genMotorShutter.sh "$@"
# Number of motors in Y direction
Y=1
# Hight of one "motor widget"
MOTORHIGHT=204
# Width of one "motor widget"
WIDTH=120
# hight of a temperature wdget
TEMPSENSORHIGHT=36
# File name extension
EXT=opi

HAS_ECMC=""
HAS_HXPD=""
HAS_PIEZO=""
HAS_PILS=""
HAS_PTP=""
OPIMID_MOT_SHT=motorx-pils.mid
OPIMID_EGU_OR_TEMP_AND_RBV=m-egu-rbv.mid
y0=16
export y0 WIDTH MOTORHIGHT TEMPSENSORHIGHT

genMatrix() {
  echo $0::genMatrix "$@"
  numparameaten=0
  cntx=0
  cnty=0
  y=$y0
  im=0 # motor number, start at 0
  it=0 # tempsensor number, start at 0
  XCNTMAX=1
  YCNTMAX=1
  haveseenY=0
  while test -n "$1"; do
    echo genMatrix "numparameaten=$numparameaten param=$1"
    case "$1" in
      [123456789])
        if test $haveseenY -eq 0; then
          XCNTMAX=$1
          echo genMatrix XCNTMAX=$XCNTMAX
        elif test $haveseenY -eq 1; then
          cntx=0
          YCNTMAX=$1
          echo genMatrix YCNTMAX=$YCNTMAX
          # loop x times y
          cntx=0
          echo genMatrix y=$y cnty=$cnty YCNTMAX=$YCNTMAX cntx=$cntx XCNTMAX=$XCNTMAX OPIMID_EGU_OR_TEMP_AND_RBV=$OPIMID_EGU_OR_TEMP_AND_RBV
          while test $cnty -lt $YCNTMAX; do
            while test $cntx -lt $XCNTMAX; do
              x=$(($cntx * $WIDTH))
              cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
              echo xcmd=$cmd "<$OPIMID_MOT_SHT"
              eval $cmd <$OPIMID_MOT_SHT >>/tmp/$$
              if test "$OPIMID_EGU_OR_TEMP_AND_RBV"; then
                eval $cmd <$OPIMID_EGU_OR_TEMP_AND_RBV >>/tmp/$$
              fi
              im=$(($im + 1))
              cntx=$(($cntx + 1))
            done
            cntx=0
            cnty=$(($cnty + 1))
            y=$(($y + $MOTORHIGHT))
          done
        else
          echo >&2 "Illegale numbers"
          echo >&2 "numparameaten=$numparameaten haveseenY=$haveseenY"
          exit 1
        fi
        ;;
      motor)
        OPIMID_MOT_SHT=motorx-pils.mid
        ;;
      m)
        HIGHT=$MOTORHIGHT
        x=$(($cntx * $WIDTH))
        cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
        echo xcmd=$cmd "<$OPIMID_MOT_SHT"
        eval $cmd <$OPIMID_MOT_SHT >>/tmp/$$
        if test "$OPIMID_EGU_OR_TEMP_AND_RBV"; then
          eval $cmd <$OPIMID_EGU_OR_TEMP_AND_RBV >>/tmp/$$
        fi
        im=$(($im + 1))
        cntx=$(($cntx + 1))
        ;;
      n)
        cntx=0
        cnty=$(($cnty + 1)) # newline
        y=$(($y + $HIGHT))
        ;;
      ptp)
        y=$(($y + 16))
        ;;
      shutter)
        OPIMID_MOT_SHT=shutterx.mid
        ;;
      s)
        OPIMID_MOT_SHT=shutterx.mid
        HIGHT=$MOTORHIGHT
        x=$(($cntx * $WIDTH))
        cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
        echo xcmd=$cmd "<$OPIMID_MOT_SHT"
        eval $cmd <$OPIMID_MOT_SHT >>/tmp/$$
        im=$(($im + 1))
        cntx=$(($cntx + 1))
        ;;
      temp)
        OPIMID_EGU_OR_TEMP_AND_RBV=m-temp-rbv.mid
        ;;
      t)
        OPIMID_MOT_SHT=tempsensor.mid
        HIGHT=$TEMPSENSORHIGHT
        x=$(($cntx * $WIDTH))
        cmd=$(echo OPIMID_MOT_SHT=$OPIMID_MOT_SHT ./shiftopi.py --shiftx $x --shifty $y --shiftt $it)
        echo xcmd=$cmd "<$OPIMID_MOT_SHT"
        eval $cmd <$OPIMID_MOT_SHT >>/tmp/$$
        it=$(($it + 1))
        cntx=$(($cntx + 1))
        ;;
      x)
        haveseenY=$(($haveseenY + 1))
        ;;
      *)
        echo >&2 "invalid: parameter $1"
        echo >&2 "allowed: m s n"
        exit 1
        ;;
    esac
    shift
    numparameaten=$(($numparameaten + 1))
  done
}

########################

FILE=$1
shift
BASENAMEF=${FILE##*/}

if test "$1" = "pils"; then
  shift
  HAS_PILS=y
  export HAS_PILS
elif test "$1" = "ecmc"; then
  shift
  HAS_ECMC=y
  export HAS_ECMC
elif test "$1" = "hxpd"; then
  shift
  OPIMID_MOT_SHT=motorx-hxpd.mid
  HAS_HXPD=y
  export HAS_HXPD
fi &&
  if test "$1" = "ptp"; then
    #shift keep it for genMatrix below
    HAS_PTP="-ptp"
    export HAS_PTP
  fi &&
  sed -e "s!<name>motorx</name>!<name>$BASENAMEF</name>!" <motorx.start >/tmp/$$ &&
  echo "Creating $FILE" &&
  if test "$HAS_PILS" = "y"; then
    cat plcName.mid >>/tmp/$$ &&
      cat Cabinet.mid >>/tmp/$$ &&
      cat plcIPADDR_PORT.mid >>/tmp/$$
  fi &&
  if test "$HAS_HXPD" = "y"; then
    cat hxpd_status_err_desc.mid >>/tmp/$$
  fi &&
  if test "$HAS_PTP" != ""; then
    cmd=$(echo ./shiftopi.py --shiftx 0 --shifty 16 --shiftm 0)
    echo cmd=$cmd "<openPTPErrBits.mid"
    eval $cmd <openPTPErrBits.mid >>/tmp/$$
    cmd=$(echo ./shiftopi.py --shiftx 100 --shifty 16 --shiftm 0)
    echo cmd=$cmd "<ptp.mid"
    eval $cmd <ptp.mid >>/tmp/$$
    cmd=$(echo ./shiftopi.py --shiftx $((282 + 100)) --shifty 16)
    echo cmd=$cmd "ptp-ts-ns.mid"
    eval $cmd <ptp-ts-ns.mid >>/tmp/$$
  fi &&
  genMatrix "$@" &&
  cat motorx.end >>/tmp/$$ &&
  if test "$HAS_ECMC" = "y"; then
    touch $FILE &&
      chmod +w $FILE &&
      sed -e "s!ethercatmcaxisExpert-pils.opi!ethercatmcaxisExpert-ecmc.opi!" </tmp/$$ >$FILE &&
      rm /tmp/$$ &&
      chmod -w $FILE
  elif test "$HAS_PTP" != ""; then
    touch $FILE &&
      chmod +w $FILE &&
      sed -e "s!ethercatmcaxisExpert-pils.opi!ethercatmcaxisExpert-pils-ptp.opi!" </tmp/$$ >$FILE &&
      rm /tmp/$$ &&
      chmod -w $FILE
  elif test "$HAS_HXPD" != ""; then
    touch $FILE &&
      chmod +w $FILE &&
      sed -e "s!ethercatmcaxisExpert-hxpd.opi!ethercatmcaxisExpert.opi!" </tmp/$$ >$FILE &&
      rm /tmp/$$ &&
      chmod -w $FILE
  elif test "$HAS_PILS" = ""; then
    touch $FILE &&
      chmod +w $FILE &&
      sed -e "s!ethercatmcaxisExpert-pils.opi!ethercatmcaxisExpert.opi!" </tmp/$$ >$FILE &&
      rm /tmp/$$ &&
      chmod -w $FILE
  else
    mv -f /tmp/$$ $FILE &&
      chmod -w $FILE
  fi
