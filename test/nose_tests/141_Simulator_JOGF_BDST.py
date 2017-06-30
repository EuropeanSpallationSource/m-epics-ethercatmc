#!/usr/bin/env python
#
# https://nose.readthedocs.org/en/latest/
# https://nose.readthedocs.org/en/latest/testing.html

import epics
import unittest
import os
import sys
import time
import filecmp
from motor_lib import motor_lib
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
myBDST = 15.0  # backlash destination, mm



def setValueOnSimulator(self, motor, tc_no, var, value):
    var = str(var)
    value = str(value)
    outStr = 'Sim.this.' + var + '=' + value
    print '%s: DbgStrToMCU motor=%s var=%s value=%s outStr=%s' % \
          (tc_no, motor, var, value, outStr)
    assert(len(outStr) < 40)
    epics.caput(motor + '-DbgStrToMCU', outStr)
    err = int(epics.caget(motor + '-Err', use_monitor=False))
    print '%s: DbgStrToMCU motor=%s var=%s value=%s err=%d' % \
          (tc_no, motor, var, value, err)
    assert (not err)


def motorInit(tself, motor, tc_no, encRel):
    setValueOnSimulator(tself, motor, tc_no, "nAmplifierPercent", 100)
    setValueOnSimulator(tself, motor, tc_no, "bAxisHomed",          1)
    setValueOnSimulator(tself, motor, tc_no, "fLowHardLimitPos",   15)
    setValueOnSimulator(tself, motor, tc_no, "fHighHardLimitPos", 165)

    # Prepare parameters for jogging and backlash
    epics.caput(motor + '.VELO', myVELO)
    epics.caput(motor + '.ACCL', myACCL)

    epics.caput(motor + '.JVEL', myJVEL)
    epics.caput(motor + '.JAR', myJAR)

    epics.caput(motor + '.BVEL', myBVEL)
    epics.caput(motor + '.BACC', myBACC)
    epics.caput(motor + '.BDST', myBDST)
    epics.caput(motor + '.UEIP', encRel)


def setMotorStartPos(tself, motor, tc_no, startpos):
    setValueOnSimulator(tself, motor, tc_no, "fActPosition", startpos)
    # Run a status update and a sync
    epics.caput(motor + '.STUP', 1)
    # The SYNC is not "synched" with STUP in the record.
    # Just delay
    #time.sleep(1)
    epics.caput(motor + '.SYNC', 1)

def compareExpectedActual(tself, expFileName, actFileName):
    # compare actual and expFile
    sameContent= filecmp.cmp(expFileName, actFileName, shallow=False)
    if not sameContent:
        file = open(expFileName, 'r')
        for line in file:
            if line[-1] == '\n':
                line = line[0:-1]
            print ("%s: %s" % (expFileName, str(line)));
        file.close();
        file = open(actFileName, 'r')
        for line in file:
            if line[-1] == '\n':
                line = line[0:-1]
            print ("%s: %s" % (actFileName, str(line)));
        file.close();
        assert(sameContent)


def jogAndBacklash(tself, motor, tc_no, encRel, motorStartPos, motorEndPos, myJOGX):
        # expected and actual
        fileName = "/tmp/" + motor + "-" + str(tc_no)
        fileName.replace(':', '-')
        expFileName = fileName + ".exp"
        actFileName = fileName + ".act"

        motorInit(tself, motor, tc_no, encRel)
        setMotorStartPos(tself, motor, tc_no, motorStartPos)
        setValueOnSimulator(tself, motor, tc_no, "bManualSimulatorMode", 1)
        setValueOnSimulator(tself, motor, tc_no, "log", actFileName)
        if myJOGX == 'JOGF':
            myDirection = 1
        elif myJOGX == 'JOGR':
            myDirection = 0
        else:
            assert(0)
        #
        epics.caput(motor + '.' + myJOGX, 1)
        time.sleep(3)
        setValueOnSimulator(tself, motor, tc_no, "fActPosition", motorEndPos)
        epics.caput(motor + '.' + myJOGX, 0)
        time.sleep(12)
        setValueOnSimulator(tself, motor, tc_no, "dbgCloseLogFile", "1")
        setValueOnSimulator(tself, motor, tc_no, "bManualSimulatorMode", "0")


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

        compareExpectedActual(tself, expFileName, actFileName)

def positionAndBacklash(tself, motor, tc_no, encRel, motorStartPos, motorEndPos):
    ###########
    # expected and actual
    fileName = "/tmp/" + motor + "-" + str(tc_no)
    fileName.replace(':', '-')
    expFileName = fileName + ".exp"
    actFileName = fileName + ".act"

    motorInit(tself, motor, tc_no, encRel)
    setMotorStartPos(tself, motor, tc_no, motorStartPos)
    setValueOnSimulator(tself, motor, tc_no, "bManualSimulatorMode", 0)
    setValueOnSimulator(tself, motor, tc_no, "log", actFileName)
    #
    epics.caput(motor + '.VAL', motorEndPos, wait=True)
    # Create a "expected" file
    expFile=open(expFileName, 'w')

    # Positioning
    # 2 different ways to move:
    # - Within the backlash distance and into the backlash direction:
    #   single move with back lash parameters
    # - against the backlash direction -or- bigger than the backlash distance:
    #   two moves, first with moving, second with backlash parameters
    if motorEndPos - motorStartPos > 0:
        directionOfMove = 1
    else:
        directionOfMove = -1
    if myBDST > 0:
        directionOfBL = 1
    else:
        directionOfBL = -1

    if abs(motorEndPos - motorStartPos) < abs(myBDST) and directionOfMove == directionOfBL:
        if encRel:
            line1 = "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                    (motorEndPos - motorStartPos, myBVEL, myBAR, motorStartPos)
        else:
            line1 = "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                    (motorEndPos, myBVEL, myBAR, motorStartPos)
        expFile.write('%s\n' % (line1))
    else:
        if encRel:
            line1 = "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                    (motorEndPos - motorStartPos - myBDST, myVELO, myAR, motorStartPos)
            # Move forward with backlash parameters
            line2 = "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                    (myBDST, myBVEL, myBAR, motorEndPos - myBDST)
        else:
            line1 = "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                    (motorEndPos - myBDST, myVELO, myAR, motorStartPos)
            # Move forward with backlash parameters
            line2 = "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                    (motorEndPos, myBVEL, myBAR, motorEndPos - myBDST)
        expFile.write('%s\n%s\n' % (line1, line2))
    expFile.close()

    compareExpectedActual(tself, expFileName, actFileName)



class Test(unittest.TestCase):
    lib = motor_lib()
    motor = os.getenv("TESTEDMOTORAXIS")

    # JOG forward & backlash compensation, absolute
    def test_TC_1411(self):
        jogAndBacklash(self, self.motor, 1411, 0, 40, 60, 'JOGF')

    # JOG forward & backlash compensation, relative
    def test_TC_1412(self):
        jogAndBacklash(self, self.motor, 1412, 1, 40, 60, 'JOGF')


    # JOG backward & backlash compensation, absolute
    def test_TC_1421(self):
        jogAndBacklash(self, self.motor, 1421, 0, 60, 40, 'JOGR')

    # JOG backward & backlash compensation, relative
    def test_TC_1422(self):
        jogAndBacklash(self, self.motor, 1422, 1, 60, 40, 'JOGR')

    # position forward & backlash compensation, absolute
    def test_TC_1431(self):
        positionAndBacklash(self, self.motor, 1431, 0, 60, 80)

    # position forward & backlash compensation, relative
    def test_TC_1432(self):
        positionAndBacklash(self, self.motor, 1432, 1, 60, 80)


    # position backward & backlash compensation, absolute
    def test_TC_1441(self):
        positionAndBacklash(self, self.motor, 1441, 0, 80, 60)

    # position backward & backlash compensation, relative
    def test_TC_1442(self):
        positionAndBacklash(self, self.motor, 1442, 1, 80, 60)

    # position forward inside backlash range, absolute
    def test_TC_1431(self):
        positionAndBacklash(self, self.motor, 1451, 0, 60, 70)

    # position forward inside backlash range, relative
    def test_TC_1432(self):
        positionAndBacklash(self, self.motor, 1452, 1, 60, 70)

