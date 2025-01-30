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
  echo $0: PARAM=$PARAM cmd=$cmd
  case $PARAM in
    CfgMoveCurrent | CfgIdleCurrent)
      eval $cmd <ethercatmcaxisConfigCustomPara.mid |
        sed -e "s!CfgXXYYZZ!$PARAM!" >>/tmp/$$
      ;;
    para[12]*)
      eval $cmd <ethercatmcaxisConfigParaDesc.mid |
        sed -e "s!PARAXXYYZZ!$PARAM!" >>/tmp/$$
      ;;
    *)
      echo >&2 "Not supported: $PARAM"
      exit 1
      ;;
  esac
  shift
  PARAM="$1"
  y=$(($y + 20))
done

touch "$FILE" &&
  chmod +w "$FILE" &&
  cat ethercatmcaxisConfig.end >>/tmp/$$ &&
  sed -e "s!ethercatmcaxisExpert-pils.opi!$FILE!" -e "s!<path>../!<path>!" -e "s!<path>ethercatmcaxisConfig!<path>ethercatmcaxisExpert!" </tmp/$$ >"$FILE" &&
  chmod -w "$FILE" &&
  rm /tmp/$$
