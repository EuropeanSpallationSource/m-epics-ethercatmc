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
    RELEASELOCAL=../../configure/RELEASE.local
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

SIM_NC_PORT=5000
IOC_NC_PORT=5001


killExitIocSimulator()
{
  # exit IOC
  echo "exit" | nc localhost ${IOC_NC_PORT} || :
  sleep 2
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

# compile simulator
(cd ../simulator && make) &&
# start simulator
(cd .. && ./run-ethercatmc-simulator.sh ) &
SIMULATOR_PID=$!

XXPVNAME=$(echo $1 | sed -e 's!.*://\(.*\)!\1!')
echo XXPVNAME=$XXPVNAME
if caget $XXPVNAME >/dev/null; then
  echo >&2 "Process variable $XXPVNAME online before starting the IOC, aborting"
  exitCode=2
  exit $exitCode
fi
sleep 5

#build ioc
( cd .. && ./run-ethercatmc-ioc.sh --no-run sim-indexer 127.0.0.1:48898 127.0.0.1.1.1 128.0.0.1.1.1)

# start ioc
date
( cd .. && nc -l  ${IOC_NC_PORT} | /bin/sh -e -x ./run-ethercatmc-ioc.sh --no-make sim-indexer 127.0.0.1:48898 127.0.0.1.1.1 128.0.0.1.1.1 ) &
IOC_PID=$!
sleep 10
date

./runTests.sh "$@"
exitCode=$?

kill -9 $IOC_PID || :
kill -9 $SIMULATOR_PID || :
killExitIocSimulator || :

echo exitCode=$exitCode
exit $exitCode
