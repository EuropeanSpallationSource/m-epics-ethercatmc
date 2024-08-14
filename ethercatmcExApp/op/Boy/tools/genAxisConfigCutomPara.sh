#!/bin/sh

# Name of the destination file on disk
FILE=$1
shift

cat ethercatmcaxisConfig.start ethercatmcaxisConfig-pils.mid >/tmp/$$ &&
  y=420
# pick up all arguments
PARAM="$1"
while test "$PARAM" != ""; do
  cmd=$(echo ./shiftopi.py --shifty $y)
  echo $0: cmd=$cmd
  eval $cmd <ethercatmcaxisConfigCustomPara.mid |
    sed -e "s!CfgXXYYZZ!$PARAM!" >>/tmp/$$
  shift
  PARAM="$1"
  y=$(($y + 20))
done

touch "$FILE" &&
  chmod +w "$FILE" &&
  cat ethercatmcaxisConfig.end >>/tmp/$$ &&
  sed -e "s!ethercatmcaxisConfig-pils.opi!$FILE!" </tmp/$$ >"$FILE" &&
  chmod -w "$FILE" &&
  rm /tmp/$$
