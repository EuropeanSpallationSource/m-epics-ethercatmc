#!/bin/sh

#PVNAME=ca://IOC:m1
PVNAME=$1
shift
TESTSCRIPTNAME=$1
shift
LOGFILENAME=$1
shift
export PVNAME
export TESTSCRIPTNAME
export LOGFILENAME

# delete old file(s), if they exists
rm -f $LOGFILENAME $LOGFILENAME.failed
# This should go to localhost only
EPICS_CA_ADDR_LIST=127.0.0.1
EPICS_CA_AUTO_ADDR_LIST=NO
export EPICS_CA_ADDR_LIST EPICS_CA_AUTO_ADDR_LIST

# run test case
./test-ioc-with-sim-indexer-one-TC.sh $PVNAME $TESTSCRIPTNAME >$$.txt 2>&1
exitCode=$?
if test $exitCode -eq 0; then
  mv $$.txt $LOGFILENAME
else
  mv $$.txt $LOGFILENAME.failed
  cat $LOGFILENAME.failed
fi

echo TESTSCRIPTNAME=$TESTSCRIPTNAME exitCode=$exitCode
exit $exitCode
