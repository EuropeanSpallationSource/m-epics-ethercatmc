#!/bin/sh

ADLFILE=adl/IOC-m1-m2.adl
MEDM=medm
if ! type $MEDM; then
    FILE=../../../../extensions/src/medm/medm/O.$EPICS_HOST_ARCH/medm
    if test -x $FILE; then
        MEDM=$FILE
    fi
fi
export MEDM
rm -f $ADLFILE &&
( cd adl/tools && make ) &&
cp -u adl/motor-2.adl $ADLFILE

$MEDM -x -macro "P=IOC:,M1=m1,M2=m2" $ADLFILE
