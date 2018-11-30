#!/bin/sh

# Number of motors in Y direction
Y=1
OPIS=..
HIGHT=204
WIDTH=120
maxX=1
maxY=1
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
  done
  FILE=$FILE.adl
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
FILE=motor-${X}.adl

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
    FILE=motor-${X}x${Y}.adl
  fi
fi

maxX=$X
maxY=$Y
echo maxX=$maxX maxY=$maxY
# Need to calculate the values of the display
x=$(($maxX * $WIDTH))
y=$(($maxY * $HIGHT))
cmd=$(echo ./shiftopi.py --shiftw $x --shifth $y)
echo cmd=" <motorx.start $cmd >$OPIS/$$"
eval $cmd <motorx.start >$OPIS/$$ &&
  echo "Creating $OPIS/$FILE" &&
  if test "$Y" = 1; then
    genXX "$@"
  else
    genXY "$@"
  fi &&
  cat motorx.end  >>$OPIS/$$ &&
  mv -f $OPIS/$$ $OPIS/$FILE


