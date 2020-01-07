#!/bin/sh
APPXX=EthercatMC
TOP=$(echo $PWD/.. | sed -e "s%/test/\.\.$%%")
export APPXX
EPICS_EEE_E3=n

uname_s=$(uname -s 2>/dev/null || echo unknown)
uname_m=$(uname -m 2>/dev/null || echo unknown)

INSTALLED_EPICS=../../../../.epics.$(hostname).$uname_s.$uname_m

if test "$EPICS_DRIVER_PATH" ; then
  EPICS_EEE_E3=e3
elif test "$EPICS_ENV_PATH" &&
    test "$EPICS_MODULES_PATH" &&
    test "$EPICS_BASES_PATH"; then
  EPICS_EEE_E3=y
elif test -r $INSTALLED_EPICS; then
  echo INSTALLED_EPICS=$INSTALLED_EPICS
 . $INSTALLED_EPICS
else
  echo not found: INSTALLED_EPICS=$INSTALLED_EPICS
fi
export EPICS_EEE_E3
echo EPICS_EEE_E3=$EPICS_EEE_E3

if test -z "$EPICS_BASE";then
  echo >&2 "EPICS_BASE" is not set
  exit 1
fi

MOTORCFG="$1"
export MOTORCFG
echo MOTORCFG=$MOTORCFG
(
  cd startup &&
  if ! test -f st.${MOTORCFG}.iocsh; then
    CMDS=$(echo st.*.iocsh | sed -e "s/st\.//g" -e "s/\.iocsh//g" | sort)
    #echo CMDS=$CMDS
    test -n "$1" && echo >&2 "not found st.${1}.iocsh"
    echo >&2 "try one of these:"
    for cmd in $CMDS; do
      case $cmd in
        *sim-indexer)
          echo >&2 $0 " $cmd" " 127.0.0.1:48898"
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

MOTORIP=127.0.0.1
MOTORPORT=5000

if test -n "$1"; then
  # allow doit.sh host:port
  PORT=${1##*:}
  HOST=${1%:*}
  echo HOST=$HOST PORT=$PORT
  if test "$PORT" != "$HOST"; then
    MOTORPORT=$PORT
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
    else
       LOCALIPS=$(ip addr | grep "inet [0-9]" | grep -v 127.0.0.1 | sed -e "s/.*inet //g"  -e "s%/.*%%g")
    fi
    #echo LOCALIP=$LOCALIP
    echo >&2         $0 "${MOTORCFG} " $MOTORIP:$MOTORPORT "<REMOTEAMSNETID> <LOCALAMSNETID>"
    for LOCALIP in $LOCALIPS; do
      echo >&2 Example $0 "${MOTORCFG} " $MOTORIP:$MOTORPORT "  $MOTORIP.1.1     $LOCALIP.1.1"
    done
    exit 1
  fi
  REMOTEAMSNETID=$1
  shift
  LOCALAMSNETID=$1
  shift
fi
export LOCALAMSNETID REMOTEAMSNETID
(
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
          echo >&2 make install failed
          exit 1
        }
        (cd .. &&
            mkdir -p dbmotor &&
            for src in db/*template; do
              dst=dbmotor/${src##*/}
              echo sed PWD=$PWD src=$src dst=$dst
              sed <"$src" >"$dst" \
                  -e "s%record(axis%record(motor%" \
                  -e "s%asynAxis%asynMotor%"
            done
        )
      fi &&
      if test -d ../../motor; then
        (cd ../../motor &&
            make install) || {
            echo >&2 make install failed
            exit 1
        }
      fi
      (cd .. &&
        make install) || {
        echo >&2 make install failed
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
          echo >&2 make install failed
          exit 1
        }
      fi &&
        if sed -e "s/#.*//" <startup/st.${MOTORCFG}.iocsh |
            grep "require *EthercatMC,.*[A-Za-z]"; then
          (cd .. &&
              rm -rfv ./dbd ./include ./doc ./db &&
              make install) || {
            echo >&2 make install failed
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
              -e "s/require EthercatMC,USER/require EthercatMC,$USER/" \
              -e "s/^cd /#cd /" \
              -e "s/REMOTEAMSNETIDXX/$REMOTEAMSNETID/" \
              -e "s/LOCALAMSNETIDXX/$LOCALAMSNETID/" \
              -e "s/127.0.0.1/$MOTORIP/" \
              -e "s/5000/$MOTORPORT/" |
          grep -v '^  *#' >$stcmddst || {
              echo >&2 can not create stcmddst $stcmddst
              exit 1
          }
          rm -fv  require.lock* &&
          chmod +x $stcmddst &&
          cmd=$(echo iocsh $stcmddst) &&
          echo PWD=$PWD cmd=$cmd &&
          eval $cmd
          ;;
      n)
          # classic EPICS, non EEE
          # We need to patch the cmd files to adjust dbLoadRecords
          # All patched files are under IOCDIR=../iocBoot/ioc${APPXX}
          for src in ../../test/startup/*iocsh  ../../iocsh/*iocsh; do
              dst=${src##*/}
              echo sed PWD=$PWD src=$src dst=$dst
              sed <"$src" >"$dst" \
                  -e "s%dbLoadRecords(\"%dbLoadRecords(\"./$DBMOTOR/%" \
                  -e "s%adsAsynPortDriverConfigure%#adsAsynPortDriverConfigure%"
          done &&
          rm -f $stcmddst &&
          cat >$stcmddst <<-EOF &&
#!../../bin/$EPICS_HOST_ARCH/${APPXX}
#This file is autogenerated by run-EthercatMC-ioc.sh - do not edit
epicsEnvSet("ARCH","$EPICS_HOST_ARCH")
epicsEnvSet("IOC","ioc${APPXX}")
epicsEnvSet("TOP","$TOP")
epicsEnvSet("EPICS_BASE","$EPICS_BASE")

cd ${TOP}
dbLoadDatabase "dbd/${APPXX}.dbd"
${APPXX}_registerRecordDeviceDriver pdbbase
EOF
   # Side note: st.${MOTORCFG}.iocsh needs extra patching
          echo sed PWD=$PWD "<../../startup/st.${MOTORCFG}.iocsh >>$stcmddst"
          sed <../../test/startup/st.${MOTORCFG}.iocsh  \
              -e "s/__EPICS_HOST_ARCH/$EPICS_HOST_ARCH/" \
              -e "s/5000/$MOTORPORT/" \
              -e "s/127.0.0.1/$MOTORIP/" \
              -e "s/REMOTEAMSNETIDXX/$REMOTEAMSNETID/" \
              -e "s/LOCALAMSNETIDXX/$LOCALAMSNETID/" \
              -e "s%cfgFile=./%cfgFile=./test/startup/%"    \
              -e "s%< %< ${TOP}/iocBoot/ioc${APPXX}/%"    \
              -e "s%require%#require%" \
              | grep -v '^  *#' >>$stcmddst &&
          cat >>$stcmddst <<-EOF &&
    iocInit
EOF
          chmod +x $stcmddst &&
          egrep -v "^ *#" $stcmddst >xx
          echo PWD=$PWD $stcmddst
          $stcmddst
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
              -e "s/require EthercatMC,USER/require EthercatMC,3.0.2/" \
              -e "s%require asyn%#require assyn%" \
              -e "s/^cd /#cd /" \
              -e "s/REMOTEAMSNETIDXX/$REMOTEAMSNETID/" \
              -e "s/LOCALAMSNETIDXX/$LOCALAMSNETID/" \
              -e "s/127.0.0.1/$MOTORIP/" \
              -e "s/5000/$MOTORPORT/" |
          grep -v '^  *#' >$stcmddst || {
              echo >&2 can not create stcmddst $stcmddst
              exit 1
          }
          chmod +x $stcmddst &&
          cmd=$(echo iocsh.bash $stcmddst) &&
          echo PWD=$PWD cmd=$cmd &&
          eval $cmd
          ;;
      *)
          echo >&2 invalid2 EPICS_EEE_E3 $EPICS_EEE_E3
          exit 1
          ;;
  esac
)
