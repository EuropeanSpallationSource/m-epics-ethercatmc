#!/usr/bin/env python
#

import epics
import unittest
import os
import sys
import time
from motor_lib import motor_lib
lib = motor_lib()
###



# Values to be used for backlash test
# Note: Make sure to use different values to hae a good
# test coverage
myVELO = 10.0   # positioning velocity
myACCL =  1.0   # Time to VELO, seconds
myAR   = myVELO / myACCL # acceleration, mm/sec^2

myJVEL = 5.0    # Jogging velocity
myJAR  = 6.0    # Jogging acceleration, mm/sec^2

myBVEL = 2.0    # backlash velocity
myBACC = 1.5    # backlash acceleration, seconds
myBAR  = myBVEL / myBACC  # backlash acceleration, mm/sec^2
myRTRY   = 3
myDLY    =  0.0
myBDST   = 24.0 # backlash destination, mm
myRMOD   = 0    # Default
myFRAC   = 1.0  #
myPOSlow = 48   #
myPOSmid = 72   # low + BDST
myPOShig = 96   # low + 2*BDST

motorRMOD_D = 0 # "Default"
motorRMOD_A = 1 # "Arithmetic"
motorRMOD_G = 2 # "Geometric"
motorRMOD_I = 3 # "In-Position"

#How we move: Absolute (without encoder) or relative (with encode via UEIP)
use_abs = 0
use_rel = 1

def motorInitTC(tself, motor, tc_no, encRel):
    epics.caput(motor + '.UEIP', encRel)


def setMotorStartPos(tself, motor, tc_no, startpos):
    lib.setValueOnSimulator(motor, tc_no, "fActPosition", startpos)
    # Run a status update and a sync
    epics.caput(motor + '.STUP', 1)
    epics.caput(motor + '.SYNC', 1)


def jogAndBacklash(tself, motor, tc_no, encRel, motorStartPos, motorEndPos, myJOGX):
    # expected and actual
    fileName = "/tmp/" + motor.replace(':', '-') + "-" + str(tc_no)
    expFileName = fileName + ".exp"
    actFileName = fileName + ".act"

    motorInitTC(tself, motor, tc_no, encRel)
    setMotorStartPos(tself, motor, tc_no, motorStartPos)
    lib.setValueOnSimulator(motor, tc_no, "log", actFileName)
    if myJOGX == 'JOGF':
        myDirection = 1
    elif myJOGX == 'JOGR':
        myDirection = 0
    else:
        assert(0)
    #
    epics.caput(motor + '.' + myJOGX, 1)
    time.sleep(3)
    lib.setValueOnSimulator(motor, tc_no, "fActPosition", motorEndPos)
    epics.caput(motor + '.' + myJOGX, 0)
    time.sleep(12)
    lib.setValueOnSimulator(motor, tc_no, "dbgCloseLogFile", "1")

    # Create a "expected" file
    expFile=open(expFileName, 'w')

    # The jogging command
    line1 = "move velocity axis_no=1 direction=%d max_velocity=%g acceleration=%g motorPosNow=%g" % \
            (myDirection, myJVEL, myJAR, motorStartPos)
    if encRel:
        # Move back in relative mode
        line2 = "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                (0 - myBDST, myVELO, myAR, motorEndPos)
        # Move relative forward with backlash parameters
        line3 = "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
            (myBDST, myBVEL, myBAR, motorEndPos - myBDST)
    else:
        # Move back in positioning mode
        line2 = "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                (motorEndPos - myBDST, myVELO, myAR, motorEndPos)
        # Move forward with backlash parameters
        line3 = "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
            (motorEndPos, myBVEL, myBAR, motorEndPos - myBDST)

    expFile.write('%s\n%s\n%s\n' % (line1, line2, line3))
    expFile.close()

    lib.cmpUnlinkExpectedActualFile(None, expFileName, actFileName)


class Test(unittest.TestCase):
    motor = os.getenv("TESTEDMOTORAXIS")

    myPOSlow = lib.myPOSlow
    myPOSmid = lib.myPOSmid
    myPOShig = lib.myPOShig

    def test_TC_14100(self):
        lib.motorInitAllForBDST(self.motor, 14100)

    # JOG forward & backlash compensation, absolute
    def test_TC_14111(self):
        jogAndBacklash(self, self.motor, 14111, use_abs, self.myPOSlow, self.myPOSmid, 'JOGF')

    # JOG forward & backlash compensation, relative
    def test_TC_14112(self):
        jogAndBacklash(self, self.motor, 14112, use_rel, self.myPOSmid, self.myPOSlow, 'JOGF')

    # JOG backward & backlash compensation, absolute
    def test_TC_14121(self):
        jogAndBacklash(self, self.motor, 14121, use_abs, self.myPOSlow, self.myPOSmid, 'JOGR')

    # JOG backward & backlash compensation, relative
    def test_TC_14122(self):
        jogAndBacklash(self, self.motor, 14122, use_rel, self.myPOSmid, self.myPOSlow, 'JOGR')

