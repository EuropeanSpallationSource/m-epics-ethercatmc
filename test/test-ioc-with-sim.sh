#!/bin/sh

set -x

if ! type nc; then
  echo >&2 "`nc` is not found"
  exit 1
fi

killExitIocSimulator()
{
  # exit IOC
  echo "exit" | nc localhost ${IOC_NC_PORT} || :
  # terminate simulator
  echo "kill" | nc localhost ${SIM_NC_PORT} || :
}

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
sleep 3

# start ioc
nc -l  ${IOC_NC_PORT} | /bin/sh -e -x ./run-ethercatmc-ioc.sh  SolAxis-SimCfgDbg &
sleep 3

# run test cases
/bin/sh -e -x ./run-ethercatmc-tests.sh IOC:m1
status=$?

killExitIocSimulator

exit $status
