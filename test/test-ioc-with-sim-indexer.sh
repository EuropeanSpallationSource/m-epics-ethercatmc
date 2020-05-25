#!/bin/sh

set -x

# Need to make sure that we have netcat
if ! type nc; then
  echo >&2 "`nc` is not found"
  exit 1
fi

# Need to make sure that we have caget in the PATH
if ! type caget >/dev/null 2>&1; then
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
      EPICS_BASE_BIN=${EPICS_BASE}/bin/$EPICS_HOST_ARCH
      export PATH=$PATH:$EPICS_BASE_BIN
    fi
  fi
fi

# Re-check that we have caget
if ! type caget; then
  echo >&2 "`caget` is not found"
  exit 1
fi



# hard-coded values
# Disadvantage: We can only run one instance on the same machine
# Advantage:    We can kill old instances, because the port number is known
PVNAME=ca://IOC:m1
SIM_NC_PORT=5000
IOC_NC_PORT=5001

killExitIocSimulator()
{
  # exit IOC
  echo "exit" | nc localhost ${IOC_NC_PORT} || :
  # terminate simulator
  echo "kill" | nc localhost ${SIM_NC_PORT} || :
}

# stop all programness, which may be still running
# if the script was aborted with ^C
killExitIocSimulator
echo env
env
echo =====
echo set
set
echo =====
SIM_NC_PORT=5000
IOC_NC_PORT=5001

# start simulator
./run-ethercatmc-simulator.sh &
sleep 5

# start ioc
nc -l  ${IOC_NC_PORT} | /bin/sh -e -x ./run-ethercatmc-ioc.sh sim-indexer 127.0.0.1:48898 127.0.0.1.1.1 128.0.0.1.1.1 &
sleep 10

# run test cases
/bin/sh -e -x ./run-ethercatmc-tests.sh ca://IOC:m1
status_m1=$?

# run more test cases
/bin/sh -e -x ./run-ethercatmc-tests.sh ca://IOC:m3
status_m3=$?

killExitIocSimulator

echo status_m1=$status_m1 status_m3=$status_m3


if test $status_m1 -ne 0; then
  exit 11
fi
if test $status_m3 -ne 0; then
  exit 13
fi

exit 0
