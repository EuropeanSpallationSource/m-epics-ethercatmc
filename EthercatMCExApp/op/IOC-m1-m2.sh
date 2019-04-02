#!/bin/sh

ADLFILE=adl/IOC-m1-m2.adl

rm -f $ADLFILE &&
( cd adl/tools && make ) &&
cp -u adl/motor-2.adl $ADLFILE

medm -x -macro "P=IOC:,M1=m1,M2=m2" $ADLFILE
