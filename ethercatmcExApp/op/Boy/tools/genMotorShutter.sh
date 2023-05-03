#!/bin/sh

echo genMotorShutter.sh "$@"
# Number of motors in Y direction
Y=1
# Hight of one "motor widget"
HIGHT=204
# Width of one "motor widget"
WIDTH=120
# File name extension
EXT=opi

HAS_ECMC=""
HAS_PTP=""
OPIMID_EGU_TEMP=motorx-egu-rbv.mid
y0=16
export y0 WIDTH HIGHT

genMatrix() {
  echo genMatrix "$@"
  OPIMID=motorx.mid
  numparameaten=0
  ix=0
  iy=0
  im=0
  XCNTMAX=1
  YCNTMAX=1
  haveseenX=0
  while test -n "$1"; do
    echo genMatrix "numparameaten=$numparameaten param=$1"
    case "$1" in
    [123456789])
      if test $haveseenX -eq 0; then
        XCNTMAX=$1
        echo genMatrix XCNTMAX=$XCNTMAX
      elif test $haveseenX -eq 1; then
        ix=0
        YCNTMAX=$1
        echo genMatrix YCNTMAX=$YCNTMAX
      else
        echo >&2 "Illegale numbers"
        echo >&2 "numparameaten=$numparameaten haveseenX=$haveseenX"
        exit 1
      fi
      ;;
    m)
      OPIMID=motorx.mid
      ;;
    n)
      ix=0
      iy=$(($iy + 1)) # newline
      ;;
    ptp)
      y0=$(($y0 + 16))
      shift
      continue
      ;;
    s)
      OPIMID=shutterx.mid
      ;;
    temp)
      OPIMID_EGU_TEMP=motorx-temp-rbv.mid
      ;;
    x)
      haveseenX=$(($haveseenX + 1))
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
  echo genMatrix y0=$y0 iy=$iy YCNTMAX=$YCNTMAX ix=$ix XCNTMAX=$XCNTMAX OPIMID_EGU_TEMP=$OPIMID_EGU_TEMP
  while test $iy -lt $YCNTMAX; do
    while test $ix -lt $XCNTMAX; do
      y=$(($y0 + $iy * $HIGHT))
      x=$(($ix * $WIDTH))
      cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
      echo xcmd=$cmd "<$OPIMID"
      eval $cmd <$OPIMID >>$$
      if test "$OPIMID_EGU_TEMP"; then
        eval $cmd <$OPIMID_EGU_TEMP >>$$
      fi
      im=$(($im + 1))
      ix=$(($ix + 1))
    done
    ix=0
    iy=$(($iy + 1))
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

sed -e "s!<name>motorx</name>!<name>$BASENAMEF</name>!"  <motorx.start >$$ &&
  echo "Creating $FILE" &&
  cat plcName.mid  >>$$ &&
  cat plcHealthStatus.mid >>$$ &&
  cat plcIPADDR_PORT.mid >>$$ &&
  if test "$HAS_PTP" != ""; then
    cmd=$(echo ./shiftopi.py --shiftx 0 --shifty 16 --shiftm 0)
    echo cmd=$cmd "<openPTPErrBits.mid"
    eval $cmd <openPTPErrBits.mid >>$$
    cmd=$(echo ./shiftopi.py --shiftx 100 --shifty 16 --shiftm 0)
    echo cmd=$cmd "<ptp.mid"
    eval $cmd <ptp.mid >>$$
  fi &&
  genMatrix "$@" &&
  cat motorx.end  >>$$ &&
  if test "$HAS_ECMC" = "y"; then
    touch $FILE &&
    chmod +w $FILE &&
    sed -e "s!ethercatmcaxisExpert-tc.opi!ethercatmcaxisExpert-ecmc.opi!"  <$$ >$FILE &&
    rm $$ &&
    chmod -w $FILE
  elif test "$HAS_PTP" != ""; then
    touch $FILE &&
    chmod +w $FILE &&
    sed -e "s!ethercatmcaxisExpert-tc.opi!ethercatmcaxisExpert-tc-ptp.opi!"  <$$ >$FILE &&
    rm $$ &&
    chmod -w $FILE
  else
    mv -f $$ $FILE &&
    chmod -w $FILE
  fi
