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
HAS_PTP=""
OPIMID_EGU_TEMP=motorx-egu-rbv.mid
y0=16
export y0 WIDTH MOTORHIGHT TEMPSENSORHIGHT

genMatrix() {
  echo genMatrix "$@"
  OPIMID=motorx.mid
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
          echo genMatrix y=$y cnty=$cnty YCNTMAX=$YCNTMAX cntx=$cntx XCNTMAX=$XCNTMAX OPIMID_EGU_TEMP=$OPIMID_EGU_TEMP
          while test $cnty -lt $YCNTMAX; do
            while test $cntx -lt $XCNTMAX; do
              x=$(($cntx * $WIDTH))
              cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
              echo xcmd=$cmd "<$OPIMID"
              eval $cmd <$OPIMID >>$$
              if test "$OPIMID_EGU_TEMP"; then
                eval $cmd <$OPIMID_EGU_TEMP >>$$
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
        OPIMID=motorx.mid
        ;;
      m)
        HIGHT=$MOTORHIGHT
        x=$(($cntx * $WIDTH))
        cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
        echo xcmd=$cmd "<$OPIMID"
        eval $cmd <$OPIMID >>$$
        if test "$OPIMID_EGU_TEMP"; then
          eval $cmd <$OPIMID_EGU_TEMP >>$$
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
        OPIMID=shutterx.mid
        ;;
      temp)
        OPIMID_EGU_TEMP=motorx-temp-rbv.mid
        ;;
      t)
        OPIMID=tempsensor.mid
        HIGHT=$TEMPSENSORHIGHT
        x=$(($cntx * $WIDTH))
        cmd=$(echo OPIMID=$OPIMID ./shiftopi.py --shiftx $x --shifty $y --shiftt $it)
        echo xcmd=$cmd "<$OPIMID"
        eval $cmd <$OPIMID >>$$
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

if test "$1" = "ecmc"; then
  shift
  HAS_ECMC=y
  export HAS_ECMC
fi &&
  if test "$1" = "ptp"; then
    #shift keep it for genMatrix below
    HAS_PTP="-ptp"
    export HAS_PTP
  fi &&
  sed -e "s!<name>motorx</name>!<name>$BASENAMEF</name>!" <motorx.start >$$ &&
  echo "Creating $FILE" &&
  cat plcName.mid >>$$ &&
  cat plcHealthStatus.mid >>$$ &&
  cat plcIPADDR_PORT.mid >>$$ &&
  if test "$HAS_PTP" != ""; then
    cmd=$(echo ./shiftopi.py --shiftx 0 --shifty 16 --shiftm 0)
    echo cmd=$cmd "<openPTPErrBits.mid"
    eval $cmd <openPTPErrBits.mid >>$$
    cmd=$(echo ./shiftopi.py --shiftx 100 --shifty 16 --shiftm 0)
    echo cmd=$cmd "<ptp.mid"
    eval $cmd <ptp.mid >>$$
    cmd=$(echo ./shiftopi.py --shiftx 100 --shifty 16)
    echo cmd=$cmd "ptp-ts-ns.mid"
    eval $cmd <ptp-ts-ns.mid >>$$
  fi &&
  genMatrix "$@" &&
  cat motorx.end >>$$ &&
  if test "$HAS_ECMC" = "y"; then
    touch $FILE &&
      chmod +w $FILE &&
      sed -e "s!ethercatmcaxisExpert-tc.opi!ethercatmcaxisExpert-ecmc.opi!" <$$ >$FILE &&
      rm $$ &&
      chmod -w $FILE
  elif test "$HAS_PTP" != ""; then
    touch $FILE &&
      chmod +w $FILE &&
      sed -e "s!ethercatmcaxisExpert-tc.opi!ethercatmcaxisExpert-tc-ptp.opi!" <$$ >$FILE &&
      rm $$ &&
      chmod -w $FILE
  else
    mv -f $$ $FILE &&
      chmod -w $FILE
  fi
