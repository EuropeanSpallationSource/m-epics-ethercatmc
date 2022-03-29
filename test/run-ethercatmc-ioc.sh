#!/bin/sh
APPXX=ethercatmc
TOP=$(echo $PWD/.. | sed -e "s%/test/\.\.$%%")
export APPXX
EPICS_EEE_E3=n
DOLOG=
HOST=""
MOTORPORT=""
REMOTEAMSNETID=""
LOCALAMSNETID=""

uname_s=$(uname -s 2>/dev/null || echo unknown)
uname_m=$(uname -m 2>/dev/null || echo unknown)

INSTALLED_EPICS=../../../../.epics.$(hostname).$uname_s.$uname_m

if test -r $INSTALLED_EPICS; then
  echo INSTALLED_EPICS=$INSTALLED_EPICS
 . $INSTALLED_EPICS
elif test "$EPICS_DRIVER_PATH" ; then
  EPICS_EEE_E3=e3
elif test "$EPICS_ENV_PATH" &&
    test "$EPICS_MODULES_PATH" &&
    test "$EPICS_BASES_PATH"; then
  EPICS_EEE_E3=y
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

if test "$1" = "--no-make"; then
  NOMAKE=y
  shift
fi
export NOMAKE
if test "$1" = "--no-run"; then
  NORUN=y
  shift
fi
export NORUN

MOTORCFG="$1"
export MOTORCFG
echo MOTORCFG=$MOTORCFG
(
  cd startup &&
  if ! test -f st.${MOTORCFG}.iocsh; then
    CMDS=$(echo st.*.iocsh | sed -e "s/st\.//g" -e "s/\.iocsh//g" | sort)
    #echo CMDS=$CMDS
    test -n "$1" && echo >&2 "not found st.${1}.iocsh"
    echo >&2 $0  "[--no-make][--no-run]"
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
  fi
) || exit 1

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


if test -n "$1" && test "$1" != "-l"; then
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
  fi
  REMOTEAMSNETID=$1
  shift
  LOCALAMSNETID=$1
  shift
fi
export LOCALAMSNETID REMOTEAMSNETID

# log/tee to file
if test "$1" = "-l"; then
    if test -n "$SM_PREFIX"; then
        XX_TXT=xx-$(echo $SM_PREFIX | sed -e "s/:$//g" | tr ":" "-" ).txt
    else
        XX_TXT=xx-$MOTORCFG.txt
    fi
    export XX_TXT
  if test -f $XX_TXT; then
    timestamp=$(date "+%y-%m-%d-%H.%M.%S")
    mkdir -p ../logs/ &&
    mv $XX_TXT ../logs/$timestamp-$MOTORCFG.txt || exit 1
  fi
  DOLOG=" 2>&1 | tee $PWD/$XX_TXT"
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
  IOCDIR=../iocBoot/ioc${APPXX}
  DBMOTOR=db
  envPathsdst=./envPaths.$EPICS_HOST_ARCH &&
  stcmddst=./st.iocsh.$EPICS_HOST_ARCH &&
  mkdir -p  $IOCDIR/ &&
  case $EPICS_EEE_E3 in
    n)
      if test -d ../motor; then
        DBMOTOR=dbmotor
        #motor
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
      (cd .. &&
        make install) || {
        echo >&2 make classic EPICS install failed
        exit 1
      }
      ;;
    y)
      #EEE
      if sed -e "s/#.*//" -e "s/-ESS\$//"  <startup/st.${MOTORCFG}.iocsh |
          grep "require *motor,.*[A-Za-z]"; then
        (cd ../../motor &&
            rm -rfv ./dbd ./include ./doc ./db &&
            make install) || {
          echo >&2 make EEE motor install failed
          exit 1
        }
      fi &&
        if sed -e "s/#.*//" <startup/st.${MOTORCFG}.iocsh |
            grep "require *ethercatmc,.*[A-Za-z]"; then
          (cd .. &&
              rm -rfv ./dbd ./include ./doc ./db &&
              make install) || {
            echo >&2 make EEE install failed
            exit 1
          }
        fi
      ;;
    e3)
      #( cd ../.. && make devinstall)
      ;;
    *)
      echo >&2 invalid1 EPICS_EEE_E3 $EPICS_EEE_E3
      exit 1
      ;;
  esac &&
  cd $IOCDIR/ &&
  case $EPICS_EEE_E3 in
      y)
          #EEE
          stcmddst=./st.iocsh.EEE.$EPICS_HOST_ARCH &&
          # We need to patch the cmd files to adjust "<"
          # All patched files are under IOCDIR=../iocBoot/ioc${APPXX}
          for src in  ../../iocsh/*iocsh ../../test/startup/*cfg ../../test/startup/*iocsh; do
              dst=${src##*/}
              echo cp PWD=$PWD src=$src dst=$dst
              cp "$src" "$dst"
          done &&
          rm -f $stcmddst &&
          sed  <st.${MOTORCFG}.iocsh  \
              -e "s/require motor,USER/require motor,$USER/" \
              -e "s/require ethercatmc,USER/require ethercatmc,$USER/" \
              -e "s/^cd /#cd /" \
          grep -v '^  *#' >$stcmddst || {
              echo >&2 can not create stcmddst $stcmddst
              exit 1
          }
          chmod +x $stcmddst
          ;;
      n)
          # classic EPICS, non EEE
          # We need to patch the cmd files to adjust dbLoadRecords
          # All patched files are under IOCDIR=../iocBoot/ioc${APPXX}
          for src in ../../test/startup/*iocsh  ../../iocsh/*iocsh; do
              dst=${src##*/}
              #echo sed PWD=$PWD src=$src dst=$dst
              sed <"$src" >"$dst" \
                  -e "s%dbLoadRecords(\"%dbLoadRecords(\"./$DBMOTOR/%" \
                  -e "s%< %< ${TOP}/iocBoot/ioc${APPXX}/%"    \
                  -e "s%adsAsynPortDriverConfigure%#adsAsynPortDriverConfigure%"
          done &&
          rm -f $stcmddst &&
          cat >$stcmddst <<-EOF &&
#!../../bin/$EPICS_HOST_ARCH/${APPXX}
#This file is autogenerated by run-ethercatmc-ioc.sh - do not edit
epicsEnvSet("ARCH","$EPICS_HOST_ARCH")
epicsEnvSet("IOC","ioc${APPXX}")
epicsEnvSet("TOP","$TOP")
epicsEnvSet("EPICS_BASE","$EPICS_BASE")

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
              | grep -v '^  *#' >>$stcmddst &&
          cat >>$stcmddst <<-EOF &&
    iocInit
EOF
          chmod +x $stcmddst &&
          egrep -v "^ *#" $stcmddst >xx
          ;;
      e3)
          #e3
          stcmddst=./st.iocsh.EEE.$EPICS_HOST_ARCH &&
          # We need to patch the cmd files to adjust "<"
          # All patched files are under IOCDIR=../iocBoot/ioc${APPXX}
          for src in  ../../iocsh/*iocsh ../../test/startup/*cfg ../../test/startup/*iocsh; do
              dst=${src##*/}
              echo cp PWD=$PWD src=$src dst=$dst
              cp "$src" "$dst"
          done &&
          rm -f $stcmddst &&
          sed  <st.${MOTORCFG}.iocsh  \
              -e "s/require motor,USER/require motor,develop/" \
              -e "s/require ethercatmc,USER/require ethercatmc,3.0.2/" \
              -e "s%require asyn%#require assyn%" \
              -e "s/^cd /#cd /" \
          grep -v '^  *#' >$stcmddst || {
              echo >&2 can not create stcmddst $stcmddst
              exit 1
          }
          ;;
      *)
          echo >&2 invalid2 EPICS_EEE_E3 $EPICS_EEE_E3
          exit 1
          ;;
  esac
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
) || exit
fi
if test "$NORUN" != "y"; then
  IOCDIR=../iocBoot/ioc${APPXX}
  DBMOTOR=db
  envPathsdst=./envPaths.$EPICS_HOST_ARCH &&
  stcmddst=./st.iocsh.$EPICS_HOST_ARCH &&
  cd $IOCDIR/ &&
  case $EPICS_EEE_E3 in
      y)
          rm -fv  require.lock* &&
          cmd=$(echo iocsh $stcmddst) &&
          echo PWD=$PWD cmd=$cmd &&
          eval $cmd
          ;;
      n)
          # classic EPICS, non EEE
          # We need to patch the cmd files to adjust dbLoadRecords
          # All patched files are under IOCDIR=../iocBoot/ioc${APPXX}
          echo PWD=$PWD $stcmddst DOLOG=$DOLOG
          eval $stcmddst $DOLOG
          ;;
      e3)
          #e3
          stcmddst=./st.iocsh.EEE.$EPICS_HOST_ARCH &&
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
