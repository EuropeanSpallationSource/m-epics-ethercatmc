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


def motorInit(tself, motor, tc_no):
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
    same = filecmp.cmp(expFileName, actFileName, shallow=False)
    if not same:
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
        assert(same)


def jogAndBacklash(tself, motor, tc_no, motorStartPos, motorEndPos, myJOGX):
        # expected and actual
        expFileName = "/tmp/" + motor + tc_no + ".exp"
        actFileName = "/tmp/" + motor + tc_no + ".act"

        motorInit(tself, motor, tc_no)
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
        setValueOnSimulator(tself, motor, tc_no, "bManualSimulatorMode", "0")
        time.sleep(12)
        setValueOnSimulator(tself, motor, tc_no, "dbgCloseLogFile", "1")


        # Create a "expected" file
        expFile=open(expFileName, 'w')

        # The jogging command
        line1 = "move velocity axis_no=1 direction=%d max_velocity=%g acceleration=%g motorPosNow=%g" % \
                (myDirection, myJVEL, myJAR, motorStartPos)
        # Move back in positioning mode
        line2 = "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                (motorEndPos - myBDST, myVELO, myAR, motorEndPos)
        # Move forward with backlash parameters
        line3 = "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                (motorEndPos, myBVEL, myBAR, motorEndPos - myBDST)
        expFile.write('%s\n%s\n%s\n' % (line1, line2, line3))
        expFile.close()

        compareExpectedActual(tself, expFileName, actFileName)


class Test(unittest.TestCase):
    lib = motor_lib()
    motor = os.getenv("TESTEDMOTORAXIS")

    # JOG forward & backlash compensation
    def test_TC_1411(self):
        tc_no = "1411"
        motor = self.motor
        motorStartPos = 60 # That's where we start the jogging
        motorEndPos   = 80 # That's where we stop the jogging
        myJOGX = 'JOGF'
        jogAndBacklash(self, motor, tc_no, motorStartPos, motorEndPos, myJOGX)


    # JOG backward & backlash compensation
    def test_TC_1412(self):
        tc_no = "1412"
        motor = self.motor
        motorStartPos = 60 # That's where we start the jogging
        motorEndPos   = 40 # That's where we stop the jogging
        myJOGX = 'JOGR'
        jogAndBacklash(self, motor, tc_no, motorStartPos, motorEndPos, myJOGX)

    # position forward & backlash compensation
    def test_TC_1413(self):
        tc_no = "1413"
        motor = self.motor
        motorStartPos = 60 # That's where we start the jogging
        motorEndPos   = 80 # That's where we stop the jogging

        ###########
        tself = self
        # expected and actual
        expFileName = "/tmp/" + motor + tc_no + ".exp"
        actFileName = "/tmp/" + motor + tc_no + ".act"

        motorInit(tself, motor, tc_no)
        setMotorStartPos(tself, motor, tc_no, motorStartPos)
        setValueOnSimulator(tself, motor, tc_no, "bManualSimulatorMode", 0)
        setValueOnSimulator(tself, motor, tc_no, "log", actFileName)
        #
        epics.caput(motor + '.VAL', motorEndPos, wait=True)
        # Create a "expected" file
        expFile=open(expFileName, 'w')

        # Positioning
        line1 = "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                (motorEndPos - myBDST, myVELO, myAR, motorStartPos)
        # Move forward with backlash parameters
        line2 = "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                (motorEndPos, myBVEL, myBAR, motorEndPos - myBDST)
        expFile.write('%s\n%s\n' % (line1, line2))
        expFile.close()

        compareExpectedActual(tself, expFileName, actFileName)

    # position backward & backlash compensation
    def test_TC_1414(self):
        tc_no = "1414"
        motor = self.motor
        motorStartPos = 80 # That's where we start the jogging
        motorEndPos   = 60 # That's where we stop the jogging

        ###########
        tself = self
        # expected and actual
        expFileName = "/tmp/" + motor + tc_no + ".exp"
        actFileName = "/tmp/" + motor + tc_no + ".act"

        motorInit(tself, motor, tc_no)
        setMotorStartPos(tself, motor, tc_no, motorStartPos)
        setValueOnSimulator(tself, motor, tc_no, "bManualSimulatorMode", 0)
        setValueOnSimulator(tself, motor, tc_no, "log", actFileName)
        #
        epics.caput(motor + '.VAL', motorEndPos, wait=True)
        # Create a "expected" file
        expFile=open(expFileName, 'w')

        # Positioning
        line1 = "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                (motorEndPos - myBDST, myVELO, myAR, motorStartPos)
        # Move forward with backlash parameters
        line2 = "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                (motorEndPos, myBVEL, myBAR, motorEndPos - myBDST)
        expFile.write('%s\n%s\n' % (line1, line2))
        expFile.close()

        compareExpectedActual(tself, expFileName, actFileName)

