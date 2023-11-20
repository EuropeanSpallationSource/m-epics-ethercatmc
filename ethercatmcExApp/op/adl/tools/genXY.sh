#!/bin/sh

# Number of motors in Y direction
Y=1
OPIS=..
HIGHT=204
WIDTH=120
EXT=adl
maxX=1
maxY=1
export OPIS WIDTH HIGHT

########################
genXY() {
  echo genXY "$@"
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
  echo genXX "$@"
  iy=0
  im=0
  #FILE=motor
  while test -n "$1"; do
    X=$1
    #FILE=$FILE-$X
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
    if test -z "$1"; then
      return
    fi
    shift
  done
  #FILE=$FILE.$EXT
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

HASX=0
#Do we have e.g. 4 x 3
if test "$1" = x; then
  HASX=1
  shift
  if test -n "$1"; then
    if test "$1" -eq 0; then
      echo >&2 "$0 <numberOfMotorsY>"
      exit 1
    fi
    Y=$1
    shift
    FILE=motor-${X}x${Y}.$EXT
  fi
elif test -n "$1"; then
  FILE=$(echo motor-$X-"$@".$EXT | sed -e "s/ /-/g")
  Y=$(($# + 1))
  echo Y=$Y
else
  FILE=motor-${X}.$EXT
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
  if test "$HASX" = 0; then
    genXX "$X" "$@"
  else
    genXY "$@"
  fi &&
  cat motorx.end >>$OPIS/$$ &&
  mv -f $OPIS/$$ $OPIS/$FILE
