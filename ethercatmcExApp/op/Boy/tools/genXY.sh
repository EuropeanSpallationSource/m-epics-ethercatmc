#!/bin/sh

# Number of motors in Y direction
Y=1
# Where to find the OPI files
OPIS=..
# Hight of one "motor widget"
HIGHT=204
# Width of one "motor widget"
WIDTH=120
# File name extension
EXT=opi

HAS_PTP=""

export OPIS TITLEH WIDTH HIGHT


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
      eval $cmd <motorx.mid >>$OPIS/$$
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
  FILE=motor
  while test -n "$X"; do
    FILE=$FILE-$X
    ix=0
    y=$(($TITLEH + $iy * $HIGHT))
    while test $ix -lt $X; do
      x=$(($ix * $WIDTH))
      cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
      echo cmd=$cmd
      eval $cmd <motorx.mid >>$OPIS/$$
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
  FILE=$FILE${HAS_PTP}.$EXT
}

########################

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
FILE=motor-${XXYY}${HAS_PTP}.$EXT

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
    FILE=motor-${X}x${Y}${HAS_PTP}.$EXT
  fi
fi

sed -e "s!<name>motorx</name>!<name>$FILE</name>!"  <motorx.start >$OPIS/$$ &&
  echo "Creating $OPIS/$FILE" &&
  cat plcName.mid  >>$OPIS/$$ &&
  if test "$HAS_PTP" != ""; then
    cat ptp.mid  >>$OPIS/$$
  fi &&
  if test "$Y" = 1; then
    genXX "$@"
  else
    genXY "$@"
  fi &&
  cat motorx.end  >>$OPIS/$$ &&
  mv -f $OPIS/$$ $OPIS/$FILE


