#!/bin/sh

# Number of motors in Y direction
Y=1
OPIS=..
HIGHT=204
WIDTH=120
export OPIS WIDTH HIGHT


########################
genXY() {
  iy=0
  im=0
  while test $iy -lt $Y; do
    ix=0
    y=$(($iy * $HIGHT))
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
  iy=0
  im=0
  FILE=motor
  while test -n "$X"; do
    FILE=$FILE-$X
    ix=0
    y=$(($iy * $HIGHT))
    while test $ix -lt $X; do
      x=$(($ix * $WIDTH))
      cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
      echo cmd=$cmd
      eval $cmd <motorx.mid >>$OPIS/$$
      im=$(($im + 1))
      ix=$(($ix + 1))
    done
    iy=$(($iy + 1))
    X=$1
    shift
  done
  FILE=$FILE.opi
}

########################

if test -z "$1"; then
  echo >&2 "$0 <numberOfMotorsX>"
  exit 1
fi
if test "$1" -eq 0; then
  echo >&2 "$0 <numberOfMotorsX>"
  exit 1
fi
X=$1
shift
FILE=motor-${X}.opi

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
    FILE=motor-${X}x${Y}.opi
  fi
fi

sed -e "s!<name>motorx</name>!<name>$FILE</name>!"  <motorx.start >$OPIS/$$ &&
  echo "Creating $OPIS/$FILE" &&
  if test "$Y" = 1; then
    genXX "$@"
  else
    genXY "$@"
  fi &&
  cat motorx.end  >>$OPIS/$$ &&
  mv -f $OPIS/$$ $OPIS/$FILE


