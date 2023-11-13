#!/bin/sh

#
# Shell script to generate (and run) an "st.cmd" for an IOC
# Assembles start scripts for both "classic EPICS" and e3
# While classic EPICS can be used under MacOs, Debian Linux or other OS,
# e3 today is Centos only
#
APPXX=ethercatmc
TOP=$(echo $PWD/.. | sed -e "s%/test/\.\.$%%")
export APPXX
EPICS_EEE_E3=classic
DOLOG=
HOST=""
MOTORPORT=""
REMOTEAMSNETID=""
LOCALAMSNETID=""

## functions
help_and_exit()
{
  cd startup
  CMDS=$(echo st.*.iocsh | sed -e "s/st\.//g" -e "s/\.iocsh//g" | sort)
  #echo CMDS=$CMDS
  test -n "$1" && echo >&2 "not found st.${1}.iocsh"
  echo >&2 $0  "[--no-make][--no-run][--epics-twincat-ads]"
  echo >&2 "try one of these:"
  for cmd in $CMDS; do
    case $cmd in
      *sim-indexer)
        echo >&2 $0 " $cmd"
        ;;
      *-indexer)
        echo >&2 $0 " $cmd" " <ip>:48898"
        ;;
      *)
        echo >&2 $0 " $cmd" " <ip>[:port]"
        ;;
    esac
  done
  exit 1
}

help_ads_and_exit()
{
  if which ifconfig >/dev/null 2>&1; then
    LOCALIPS=$(ifconfig | grep "inet [0-9]" | grep -v 127.0.0.1 | sed -e "s/.*inet //g" -e "s/ netmask.*//")
  elif which ip >/dev/null 2>&1; then
     LOCALIPS=$(ip addr | grep "inet [0-9]" | grep -v 127.0.0.1 | sed -e "s/.*inet //g"  -e "s%/.*%%g")
  fi
  #echo LOCALIP=$LOCALIP
  echo >&2         $0 "${MOTORCFG} " $MOTORIP:$MOTORPORT "<REMOTEAMSNETID> <LOCALAMSNETID>"
  for LOCALIP in $LOCALIPS; do
    REMOTEAMSNETID=$MOTORIP.1.1
    case $MOTORCFG in
    mcu0[0-9][0-9]*)
      CRATENO=$(echo $MOTORCFG | sed -e "s/mcu0//")
      IP_DIGIT=$((50 + $CRATENO))
      REMOTEAMSNETID=192.168.88.$IP_DIGIT.1.1
      ;;
    esac
    echo >&2 Example $0 "${MOTORCFG} " $MOTORIP:$MOTORPORT " $REMOTEAMSNETID  $LOCALIP.1.1"
  done
  exit 1
}


generate_st_cmd_classic() {
  # classic EPICS, non EEE
  # We need to patch the cmd files to adjust dbLoadRecords
  for src in  ../../iocsh/*iocsh ../../test/startup/*iocsh; do
    dst=${src##*/}
    #echo sed PWD=$PWD src=$src dst=$dst
    sed <"$src" >"$dst" \
        -e "s%dbLoadRecords(\"%dbLoadRecords(\"./db/%" \
        -e "s%< %< ${TOP}/iocBoot/ioc${APPXX}/%"    \
        -e "s!/c/Users/!c:/Users/!" \
        -e "s%$ASYNPORTCONFIGUREDONTUSE%#$ASYNPORTCONFIGUREDONTUSE%" \
        -e "s%# *$ASYNPORTCONFIGUREUSE%$ASYNPORTCONFIGUREUSE%"
  done &&
    rm -f $stcmddst &&
    cat >$stcmddst <<-EOF &&
#!../../bin/$EPICS_HOST_ARCH/${APPXX}
#This file is autogenerated by run-ethercatmc-ioc.sh - do not edit
epicsEnvSet("ARCH","$EPICS_HOST_ARCH")
epicsEnvSet("IOC","ioc${APPXX}")
epicsEnvSet("TOP","$TOP")
epicsEnvSet("EPICS_BASE","$EPICS_BASE")
epicsEnvSet("PVXS_QSRV_ENABLE", "YES")

cd ${TOP}
dbLoadDatabase "dbd/${APPXX}.dbd"
${APPXX}_registerRecordDeviceDriver pdbbase
EOF
  # Side note: st.${MOTORCFG}.iocsh needs extra patching
  #echo sed PWD=$PWD "<../../startup/st.${MOTORCFG}.iocsh >>$stcmddst"
  sed <../../test/startup/st.${MOTORCFG}.iocsh  \
      -e "s/__EPICS_HOST_ARCH/$EPICS_HOST_ARCH/" \
      -e "s%cfgFile=./%cfgFile=./test/startup/%"    \
      -e "s%< %< ${TOP}/iocBoot/ioc${APPXX}/%"    \
      -e "s%require%#require%" \
      -e "s!/c/Users/!c:/Users/!" \
    | grep -v '^  *#' >>$stcmddst &&
    cat >>$stcmddst <<-EOF &&
        iocInit
EOF
  postprocess_stcmddst &&
  # Create an xx file to be used under gdb
  chmod +x $stcmddst && egrep -v "^ *#" $stcmddst >xx
}


#
# generate an st.cmd for e3
# Add require at the start of the file
# rplace
#< ethercatmcController.iocsh
#with
#iocshLoad("$(ethercatmc_DIR)/ethercatmcController.iocsh")
#
#
generate_st_cmd_e3() {
  if test -z "$1"; then
     echo >&2 generate_st_cmd_e3: Parameter MOTORCFG is empty
     exit 1
  fi
  local stcmddst
  stcmddst="st.$1.cmd"
  > $stcmddst &&
    cat >>$stcmddst <<-EOF &&
require essioc
require ethercatmc

EOF
  sed <../../test/startup/st.${MOTORCFG}.iocsh  \
      -e "s/^cd /#cd /" \
      -e 's! *< *\([^ ]*\)!iocshLoad("\$(ethercatmc_DIR)/\1")!' |
    grep -v '^  *#' >>$stcmddst || {
      echo >&2 can not create stcmddst $stcmddst
      exit 1
  }
  cat >>$stcmddst <<-EOF &&

iocinit()
EOF

  postprocess_stcmddst
}


postprocess_stcmddst() {
  # Post-process of stcmddst
  if test -n "$HOST" ; then
    sed < $stcmddst \
        -e "s/172\.[0-9]*\.[0-9]*.[0-9]*/$MOTORIP/" \
        -e "s/127.0.0.1/$MOTORIP/" >/tmp/$$ &&
      mv -f /tmp/$$ $stcmddst
  fi
  if test -n "$MOTORPORT" ; then
    sed < $stcmddst -e "s/5000/$MOTORPORT/" >/tmp/$$ &&
      mv -f /tmp/$$ $stcmddst
  fi
  if test -n "$REMOTEAMSNETID" ; then
    sed < $stcmddst \
        -e "s/REMOTEAMSNETIDXX/$REMOTEAMSNETID/" \
        -e "s/amsNetIdRemote=[0-9]*\.[0-9]*\.[0-9]*\.[0-9]*\.[0-9]*\.[0-9]*:[0-9]*/amsNetIdRemote=$REMOTEAMSNETID/" >/tmp/$$ &&
      mv -f /tmp/$$ $stcmddst
  fi
  if test -n "$LOCALAMSNETID" ; then
    sed < $stcmddst \
        -e "s/LOCALAMSNETIDXX/$LOCALAMSNETID/"  >/tmp/$$ &&
      mv -f /tmp/$$ $stcmddst
  fi
  chmod +x $stcmddst
}


uname_s=$(uname -s 2>/dev/null || echo unknown)
uname_m=$(uname -m 2>/dev/null || echo unknown)

INSTALLED_EPICS=../../../../.epics.$(hostname).$uname_s.$uname_m

IOCDIR_CLASSIC=../iocBoot/ioc${APPXX} &&
mkdir -p  $IOCDIR_CLASSIC/ &&
IOCDIR_E3=../iocBoot/e3 &&
mkdir -p  $IOCDIR_E3/ &&
if test -r $INSTALLED_EPICS; then
  echo INSTALLED_EPICS=$INSTALLED_EPICS
 . $INSTALLED_EPICS
elif test "$EPICS_DRIVER_PATH" ; then
  EPICS_EEE_E3=e3
else
  echo not found: INSTALLED_EPICS=$INSTALLED_EPICS
fi


export EPICS_EEE_E3
echo EPICS_EEE_E3=$EPICS_EEE_E3

if test -z "$EPICS_HOST_ARCH"; then
  RELEASELOCAL=../configure/RELEASE.local
  if test -r "$RELEASELOCAL"; then
    # Code stolen from .ci/travis/prepare.sh
    eval $(grep "EPICS_BASE=" $RELEASELOCAL)
    export EPICS_BASE
    echo "EPICS_BASE=$EPICS_BASE"
    if test -z "$EPICS_BASE"; then
      echo >&2 "EPICS_BASE" is not set
      exit 1
    fi
    [ -z "$EPICS_HOST_ARCH" -a -f $EPICS_BASE/src/tools/EpicsHostArch.pl ] && EPICS_HOST_ARCH=$(perl $EPICS_BASE/src/tools/EpicsHostArch.pl)
    [ -z "$EPICS_HOST_ARCH" -a -f $EPICS_BASE/startup/EpicsHostArch.pl ] && EPICS_HOST_ARCH=$(perl $EPICS_BASE/startup/EpicsHostArch.pl)
    export EPICS_HOST_ARCH
    echo "EPICS_HOST_ARCH=$EPICS_HOST_ARCH"
  fi
fi

if test -z "$EPICS_HOST_ARCH"; then
  echo >&2 "EPICS_HOST_ARCH" is not set
  exit 1
fi

NOMAKE=""
NORUN=""
DOLOG=""
ASYNPORTCONFIGUREDONTUSE="adsAsynPortDriverConfigure"
ASYNPORTCONFIGUREUSE="drvAsynIPPortConfigure"

# pick up some arguments
PARAM="$1"
while test "$PARAM" != ""; do
  case $1 in
  --no-make)
    NOMAKE=y
    shift
  ;;
  --no-run)
    NORUN=y
    shift
    ;;
  --epics-twincat-ads)
    ASYNPORTCONFIGUREUSE=adsAsynPortDriverConfigure
    ASYNPORTCONFIGUREDONTUSE=drvAsynIPPortConfigure
    shift
    ;;
  -h|--help)
    help_and_exit
    ;;
  -l)
    DOLOG=y
    shift
    ;;
  *)
    PARAM="" # end the loop
    ;;
  esac
done

export NOMAKE
export NORUN
export ASYNPORTCONFIGUREDONTUSE

MOTORCFG="$1"
export MOTORCFG
echo MOTORCFG=$MOTORCFG
if ! test -f startup/st.${MOTORCFG}.iocsh; then
  help_and_exit
fi
shift

# motor port is different for indexer
case $MOTORCFG in
  *sim-indexer)
    ;;
  *indexer)
    MOTORPORT=48898
    ;;
  *)
    MOTORPORT=5000
    ;;
esac

PARAM="$1"
while test "$PARAM" != ""; do
  case $1 in
  -h|--help)
    help_ads_and_exit
    ;;
  -l)
    DOLOG=y
    shift
    ;;
  *)
    PARAM="" # end the loop
    ;;
  esac
done

if test -n "$1"; then
  # allow doit.sh host:port
  PORT=${1##*:}
  HOST=${1%:*}
  echo HOST=$HOST PORT=$PORT
  if test "$PORT" != "$HOST"; then
    MOTORPORT=$PORT
  else
    MOTORPORT=5000
  fi
  echo HOST=$HOST MOTORPORT=$MOTORPORT
  MOTORIP=$HOST
  echo MOTORIP=$MOTORIP
  shift
fi
export MOTORIP MOTORPORT
if test "$MOTORPORT" = 48898; then
  if test -z "$2"; then
    help_ads_and_exit
  fi
  REMOTEAMSNETID=$1
  shift
  LOCALAMSNETID=$1
  shift
fi
export LOCALAMSNETID REMOTEAMSNETID

PARAM="$1"
while test "$PARAM" != ""; do
  case $1 in
  -h|--help)
    help_ads_and_exit
    ;;
  -l)
    DOLOG=y
    shift
    ;;
  *)
    PARAM="" # end the loop
    ;;
  esac
done

# log/tee to file
if test "$DOLOG" = y; then
  if test -n "$SM_PREFIX"; then
    LOG_TXT=log-$(echo $SM_PREFIX | sed -e "s/:$//g" | tr ":" "-" ).txt
  else
    LOG_TXT=log-$MOTORCFG.txt
  fi
  export LOG_TXT
	if test -f $LOG_TXT; then
    timestamp=$(date "+%y-%m-%d-%H.%M.%S")
    mkdir -p ../logs/ &&
    mv $LOG_TXT ../logs/$timestamp-$MOTORCFG.txt || exit 1
  fi
  DOLOG=" 2>&1 | tee $PWD/$LOG_TXT"
  shift
fi
export DOLOG

if test -n "$1"; then
  echo >&2 unsupported additional parameters: $@
  exit 1
fi

if test "$NOMAKE" != "y"; then
(
  if test -x ../checkws.sh; then
  (
    cd .. && ./checkws.sh
  ) || exit
  fi
  stcmddst=./st.iocsh.$EPICS_HOST_ARCH &&
  case $EPICS_EEE_E3 in
    classic)
      if test -d ../motor; then
        (cd ../motor && make install) && (cd .. && make install) || {
          echo >&2 make ../motor install failed
          exit 1
        }
      fi &&
      if test -d ../../motor; then
        (cd ../../motor &&
            make install) || {
            echo >&2 make ../.. motor install failed
            exit 1
        }
      fi
      if test -d ../../ads; then
        (cd ../../ads &&
            make ) || {
            echo >&2 make ../.. ads failed
            exit 1
        }
      fi
      (cd .. &&
        make install) || {
        echo >&2 make classic EPICS install failed
        exit 1
      }
      ;;
    e3)
      #( cd ../.. && make devinstall)
      ;;
    *)
      echo >&2 invalid1 EPICS_EEE_E3 $EPICS_EEE_E3
      exit 1
      ;;
  esac &&
  case $EPICS_EEE_E3 in
    classic)
      # Do it in a subshell to undo the `cd $IOCDIR_E3`
      ( cd $IOCDIR_E3 && generate_st_cmd_e3 $MOTORCFG)
      cd $IOCDIR_CLASSIC && generate_st_cmd_classic
      ;;
    e3)
      cd $IOCDIR_E3 && generate_st_cmd_e3 $MOTORCFG
      ;;
    *)
      echo >&2 invalid2 EPICS_EEE_E3 $EPICS_EEE_E3
      exit 1
      ;;
  esac
) || exit
fi
if test "$NORUN" != "y"; then
  stcmddst=./st.iocsh.$EPICS_HOST_ARCH &&
  cd $IOCDIR_CLASSIC/ &&
  case $EPICS_EEE_E3 in
      classic)
          # classic EPICS, non EEE
          # We need to patch the cmd files to adjust dbLoadRecords
          # All patched files are under IOCDIR_CLASSIC=../iocBoot/ioc${APPXX}
          echo PWD=$PWD $stcmddst DOLOG=$DOLOG
          eval $stcmddst $DOLOG
          ;;
      e3)
          stcmddst=./st.iocsh.e3.$EPICS_HOST_ARCH &&
          cmd=$(echo iocsh.bash $stcmddst) &&
          echo PWD=$PWD cmd=$cmd &&
          eval $cmd
          ;;
      *)
          echo >&2 invalid2 EPICS_EEE_E3 $EPICS_EEE_E3
          exit 1
          ;;
  esac
fi
