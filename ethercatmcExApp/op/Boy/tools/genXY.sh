#!/bin/sh

# Number of motors in Y direction
Y=1
# Hight of one "motor widget"
HIGHT=204
# Width of one "motor widget"
WIDTH=120
# File name extension
EXT=opi

HAS_PTP=""

export TITLEH WIDTH HIGHT


########################
genXY() {
  echo genXY "$@"
  iy=0
  im=0
  while test $iy -lt $Y; do
    ix=0
    y=$(($TITLEH + $iy * $HIGHT))
    while test $ix -lt $X; do
      x=$(($ix * $WIDTH))
      cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
      echo cmd=$cmd
      eval $cmd <motorx.mid >>$$
      im=$(($im + 1))
      ix=$(($ix + 1))
    done &&
      iy=$(($iy + 1))
  done
}
########################

genXX() {
  echo genXX "$@"
  iy=0
  im=0
  while test -n "$X"; do
    ix=0
    y=$(($TITLEH + $iy * $HIGHT))
    while test $ix -lt $X; do
      x=$(($ix * $WIDTH))
      cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
      echo cmd=$cmd
      eval $cmd <motorx.mid >>$$
      im=$(($im + 1))
      ix=$(($ix + 1))
    done
    iy=$(($iy + 1))
    if test $# -gt 0; then
      X=$1
      shift
    else
      X=
    fi
  done
}

########################

FILE=$1
shift
BASENAMEF=${FILE##*/}

if test "$1" = "ptp"; then
  shift
  HAS_PTP="-ptp"
  export HAS_PTP
fi &&
if test -z "$1"; then
  echo >&2 "$0 <numberOfMotorsX>"
  exit 1
fi
if test "$1" -eq 0; then
  echo >&2 "$0 <numberOfMotorsX>"
  exit 1
fi
X=$1
# Get the right name inside the opi, like motor-4x3.opi
XXYY=$(echo "$@" | sed -e "s/ /-/g")
shift

#Do we have e.g. 4 x 3
if test "$1" = x; then
  shift
  if test -n "$1"; then
    if test "$1" -eq 0; then
      echo >&2 "$0 <numberOfMotorsY>"
      exit 1
    fi
    Y=$1
    shift
  fi
fi

sed -e "s!<name>motorx</name>!<name>$BASENAMEF</name>!"  <motorx.start >$$ &&
  echo "Creating $FILE" &&
  cat plcName.mid  >>$$ &&
  if test "$HAS_PTP" != ""; then
    cat ptp.mid  >>$$
  fi &&
  if test "$Y" = 1; then
    genXX "$@"
  else
    genXY "$@"
  fi &&
  cat motorx.end  >>$$ &&
  if test "$HAS_PTP" != ""; then
    chmod +w $FILE &&
    sed -e "s!ethercatmcaxisExpert.opi!ethercatmcaxisExpert-ptp.opi!"  <$$ >$FILE &&
    rm $$ &&
    chmod -w $FILE
  else
    mv -f $$ $FILE &&
    chmod -w $FILE
  fi
