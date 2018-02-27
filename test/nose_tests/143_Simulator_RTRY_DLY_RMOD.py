#!/usr/bin/env python
#
import epics
import unittest
import os
import sys
import time
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
myRTRY   = 3
myDLY    = 1
myBDST   = 24.0 # backlash destination, mm
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
# Note: motorRMOD_I is always absolute !


def setValueOnSimulator(self, motor, tc_no, var, value):
    var = str(var)
    value = str(value)
    outStr = 'Sim.this.' + var + '=' + value
    print '%s: DbgStrToMCU motor=%s var=%s value=%s outStr=%s' % \
          (tc_no, motor, var, value, outStr)
    assert(len(outStr) < 40)
    epics.caput(motor + '-DbgStrToMCU', outStr, wait=True)
    err = int(epics.caget(motor + '-Err', use_monitor=False))
    print '%s: DbgStrToMCU motor=%s var=%s value=%s err=%d' % \
          (tc_no, motor, var, value, err)
    assert (not err)


def motorInitAll(tself, motor, tc_no):
    setValueOnSimulator(tself, motor, tc_no, "nAmplifierPercent", 100)
    setValueOnSimulator(tself, motor, tc_no, "bAxisHomed",          1)
    setValueOnSimulator(tself, motor, tc_no, "fLowHardLimitPos",   15)
    setValueOnSimulator(tself, motor, tc_no, "fHighHardLimitPos", 165)
    setValueOnSimulator(tself, motor, tc_no, "setMRES_23", 0)
    setValueOnSimulator(tself, motor, tc_no, "setMRES_24", 0)

    # Prepare parameters for jogging and backlash
    epics.caput(motor + '.VELO', myVELO)
    epics.caput(motor + '.ACCL', myACCL)

    epics.caput(motor + '.JVEL', myJVEL)
    epics.caput(motor + '.JAR',  myJAR)

    epics.caput(motor + '.BVEL', myBVEL)
    epics.caput(motor + '.BACC', myBACC)
    epics.caput(motor + '.BDST', 0.0)
    epics.caput(motor + '.RTRY', myRTRY)
    epics.caput(motor + '.DLY',  myDLY)

def motorInitTC(tself, motor, tc_no, rmod, encRel):
    epics.caput(motor + '.UEIP', encRel)
    epics.caput(motor + '.RMOD', rmod)


def setMotorStartPos(tself, motor, tc_no, startpos):
    setValueOnSimulator(tself, motor, tc_no, "fActPosition", startpos)
    # Run a status update and a sync
    epics.caput(motor + '.STUP', 1)
    epics.caput(motor + '.SYNC', 1)


def writeExpFileRMOD_X(tself, motor, tc_no, rmod, dbgFile, expFile, maxcnt, encRel, motorStartPos, motorEndPos):
    cnt = 0
    if motorEndPos - motorStartPos > 0:
        directionOfMove = 1
    else:
        directionOfMove = -1
    if myBDST > 0:
        directionOfBL = 1
    else:
        directionOfBL = -1

    print '%s: writeExpFileRMOD_X motor=%s encRel=%d motorStartPos=%f motorEndPos=%f directionOfMove=%d directionOfBL=%d' % \
          (tc_no, motor, encRel, motorStartPos, motorEndPos, directionOfMove, directionOfBL)

    if dbgFile != None:
        dbgFile.write('#%s: writeExpFileRMOD_X motor=%s rmod=%d encRel=%d motorStartPos=%f motorEndPos=%f directionOfMove=%d directionOfBL=%d\n' % \
          (tc_no, motor, rmod, encRel, motorStartPos, motorEndPos, directionOfMove, directionOfBL))

    if rmod == motorRMOD_I:
        maxcnt = 1 # motorRMOD_I means effecttivly "no retry"
        encRel = 0

    if abs(motorEndPos - motorStartPos) <= abs(myBDST) and directionOfMove == directionOfBL:
        while cnt < maxcnt:
            # calculate the delta to move
            # The calculated delta is the scaled, and used for both absolute and relative
            # movements
            delta = motorEndPos - motorStartPos
            if cnt > 1:
                if rmod == motorRMOD_A:
                    # From motorRecord.cc:
                    #factor = (pmr->rtry - pmr->rcnt + 1.0) / pmr->rtry;
                    factor = 1.0 * (myRTRY -  cnt + 1.0) / myRTRY
                    delta = delta * factor
                elif rmod == motorRMOD_G:
                    #factor = 1 / pow(2.0, (pmr->rcnt - 1));
                    rcnt_1 = cnt - 1
                    factor = 1.0
                    while rcnt_1 > 0:
                        factor = factor / 2.0
                        rcnt_1 -= 1
                    delta = delta * factor

            if encRel:
                line1 = "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                        (delta, myBVEL, myBAR, motorStartPos)
            else:
                line1 = "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                        (motorStartPos + delta, myBVEL, myBAR, motorStartPos)
            expFile.write('%s\n' % (line1))
            cnt += 1
    else:
        # As we don't move the motor (it is simulated, we both times start at motorStartPos
        while cnt < maxcnt:
            # calculate the delta to move
            # The calculated delta is the scaled, and used for both absolute and relative
            # movements
            delta = motorEndPos - motorStartPos - myBDST
            if cnt > 1:
                if rmod == motorRMOD_A:
                    # From motorRecord.cc:
                    #factor = (pmr->rtry - pmr->rcnt + 1.0) / pmr->rtry;
                    factor = 1.0 * (myRTRY -  cnt + 1.0) / myRTRY
                    delta = delta * factor
                elif rmod == motorRMOD_G:
                    #factor = 1 / pow(2.0, (pmr->rcnt - 1));
                    rcnt_1 = cnt - 1
                    factor = 1.0
                    while rcnt_1 > 0:
                        factor = factor / 2.0
                        rcnt_1 -= 1
                    delta = delta * factor

            if encRel:
                line1 = "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                        (delta, myVELO, myAR, motorStartPos)
                # Move forward with backlash parameters
                # Note: This should be myBDST, but since we don't move the motor AND
                # the record uses the readback value, use "motorEndPos - motorStartPos"
                delta = motorEndPos - motorStartPos
                line2 = "move relative delta=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                        (delta, myBVEL, myBAR, motorStartPos)
            else:
                line1 = "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                        (motorStartPos + delta, myVELO, myAR, motorStartPos)
                # Move forward with backlash parameters
                line2 = "move absolute position=%g max_velocity=%g acceleration=%g motorPosNow=%g" % \
                        (motorEndPos, myBVEL, myBAR, motorStartPos)

            expFile.write('%s\n%s\n' % (line1, line2))
            cnt += 1




def positionAndDelay(tself, motor, tc_no, rmod, encRel, motorStartPos, motorEndPos):
    lib = motor_lib()
    ###########
    # expected and actual
    fileName = "/tmp/" + motor.replace(':', '-') + "-" + str(tc_no)
    expFileName = fileName + ".exp"
    actFileName = fileName + ".act"
    dbgFileName = fileName + ".dbg"

    motorInitTC(tself, motor, tc_no, rmod, encRel)
    setMotorStartPos(tself, motor, tc_no, motorStartPos)
    setValueOnSimulator(tself, motor, tc_no, "bManualSimulatorMode", 1)
    time.sleep(2)
    setValueOnSimulator(tself, motor, tc_no, "log", actFileName)
    time.sleep(2)
    #
    epics.caput(motor + '.VAL', motorEndPos, wait=True)
    setValueOnSimulator(tself, motor, tc_no, "dbgCloseLogFile", "1")
    time.sleep(2)
    setValueOnSimulator(tself, motor, tc_no, "bManualSimulatorMode", 0)

    # Create a "expected" file
    expFile=open(expFileName, 'w')
    if dbgFileName != None:
        dbgFile=open(dbgFileName, 'w')
    else:
        dbgFile = None
    # Positioning
    # 2 different ways to move:
    # - Within the backlash distance and into the backlash direction:
    #   single move with back lash parameters
    # - against the backlash direction -or- bigger than the backlash distance:
    #   two moves, first with moving, second with backlash parameters

    cnt = 1 + int(epics.caget(motor + '.RTRY'))
    writeExpFileRMOD_X(tself, motor, tc_no, rmod, dbgFile, expFile, cnt, encRel, motorStartPos, motorEndPos)

    expFile.close()
    if dbgFileName != None:
        dbgFile.close()
    setValueOnSimulator(tself, motor, tc_no, "dbgCloseLogFile", "1")

    lib.cmpUnlinkExpectedActualFile(dbgFileName, expFileName, actFileName)



class Test(unittest.TestCase):
    lib = motor_lib()
    motor = os.getenv("TESTEDMOTORAXIS")

    # init the record and controller
    def test_TC_14300(self):
        motorInitAll(self, self.motor, 14300)

#    # motorRMOD_D = 0 # "Default"
#    # position forward & backlash compensation, absolute
#    def test_TC_14301(self):
#        positionAndDelay(self, self.motor, 14301, motorRMOD_D, use_abs, myPOSlow, myPOShig)
#
#    # position forward & backlash compensation, relative
#    def test_TC_14302(self):
#        positionAndDelay(self, self.motor, 14302, motorRMOD_D, use_rel, myPOSlow, myPOShig)
#
#    # position backward & backlash compensation, absolute
#    def test_TC_14303(self):
#        positionAndDelay(self, self.motor, 14303, motorRMOD_D, use_abs, myPOSlow, myPOShig)
#
#    # position backward & backlash compensation, relative
#    def test_TC_14304(self):
#        positionAndDelay(self, self.motor, 14304, motorRMOD_D, use_rel, myPOSlow, myPOShig)
#
#    # position forward inside backlash range, absolute
#    def test_TC_14305(self):
#        positionAndDelay(self, self.motor, 14305, motorRMOD_D, use_abs, myPOSmid, myPOSlow)
#
#    # position forward inside backlash range, relative
#    def test_TC_14306(self):
#        positionAndDelay(self, self.motor, 14306, motorRMOD_D, use_rel, myPOSmid, myPOSlow)
#
#    # position forward inside backlash range, absolute
#    def test_TC_14307(self):
#        positionAndDelay(self, self.motor, 14307, motorRMOD_D, use_abs, myPOSlow, myPOSmid)
#
#    # position forward inside backlash range, relative
#    def test_TC_14308(self):
#        positionAndDelay(self, self.motor, 14308, motorRMOD_D, use_rel, myPOSlow, myPOSmid)
#
#    ###############################################################################
#    # motorRMOD_A
#    # position forward & backlash compensation, absolute
#    def test_TC_14311(self):
#        positionAndDelay(self, self.motor, 14311, motorRMOD_A, use_abs, myPOSlow, myPOShig)
#
#    # position forward & backlash compensation, relative
#    def test_TC_14312(self):
#        positionAndDelay(self, self.motor, 14312, motorRMOD_A, use_rel, myPOSlow, myPOShig)
#
#    # position backward & backlash compensation, absolute
#    def test_TC_14313(self):
#        positionAndDelay(self, self.motor, 14313, motorRMOD_A, use_abs, myPOShig, myPOSlow)
#
#    # position backward & backlash compensation, relative
#    def test_TC_14314(self):
#        positionAndDelay(self, self.motor, 14314, motorRMOD_A, use_rel, myPOShig, myPOSlow)
#
#    # position forward inside backlash range, absolute
#    def test_TC_14315(self):
#        positionAndDelay(self, self.motor, 14315, motorRMOD_A, use_abs, myPOSmid, myPOSlow)
#
#    # position forward inside backlash range, relative
#    def test_TC_14316(self):
#        positionAndDelay(self, self.motor, 14316, motorRMOD_A, use_rel, myPOSmid, myPOSlow)
#
#    # position forward inside backlash range, absolute
#    def test_TC_14317(self):
#        positionAndDelay(self, self.motor, 14317, motorRMOD_A, use_abs, myPOSlow, myPOSmid)
#
#    # position forward inside backlash range, relative
#    def test_TC_14318(self):
#        positionAndDelay(self, self.motor, 14318, motorRMOD_A, use_rel, myPOSlow, myPOSmid)
#
#    ###############################################################################
#    # motorRMOD_G
#    # position forward & backlash compensation, absolute
#    def test_TC_14321(self):
#        positionAndDelay(self, self.motor, 14321, motorRMOD_G, use_abs, myPOSlow, myPOShig)
#
#    # position forward & backlash compensation, relative
#    def test_TC_14322(self):
#        positionAndDelay(self, self.motor, 14322, motorRMOD_G, use_rel, myPOSlow, myPOShig)
#
#    # position backward & backlash compensation, absolute
#    def test_TC_14323(self):
#        positionAndDelay(self, self.motor, 14323, motorRMOD_G, use_abs, myPOShig, myPOSlow)
#
#    # position backward & backlash compensation, relative
#    def test_TC_14324(self):
#        positionAndDelay(self, self.motor, 14324, motorRMOD_G, use_rel, myPOShig, myPOSlow)
#
#    # position forward inside backlash range, absolute
#    def test_TC_14325(self):
#        positionAndDelay(self, self.motor, 14325, motorRMOD_G, use_abs, myPOSmid, myPOSlow)
#
#    # position forward inside backlash range, relative
#    def test_TC_14326(self):
#        positionAndDelay(self, self.motor, 14326, motorRMOD_G, use_rel, myPOSmid, myPOSlow)
#
#    # position forward inside backlash range, absolute
#    def test_TC_14327(self):
#        positionAndDelay(self, self.motor, 14327, motorRMOD_G, use_abs, myPOSlow, myPOSmid)
#
#    # position forward inside backlash range, relative
#    def test_TC_14328(self):
#        positionAndDelay(self, self.motor, 14328, motorRMOD_G, use_rel, myPOSlow, myPOSmid)
#

    ###############################################################################
    # motorRMOD_I
    # position forward & backlash compensation, absolute
    def test_TC_14331(self):
        positionAndDelay(self, self.motor, 14331, motorRMOD_I, use_abs, myPOSlow, myPOShig)

    # position forward & backlash compensation, relative
    def test_TC_14332(self):
        positionAndDelay(self, self.motor, 14332, motorRMOD_I, use_rel, myPOSlow, myPOShig)

    # position backward & backlash compensation, absolute
    def test_TC_14333(self):
        positionAndDelay(self, self.motor, 14333, motorRMOD_I, use_abs, myPOShig, myPOSlow)

    # position backward & backlash compensation, relative
    def test_TC_14334(self):
        positionAndDelay(self, self.motor, 14334, motorRMOD_I, use_rel, myPOShig, myPOSlow)

    # position forward inside backlash range, absolute
    def test_TC_14335(self):
        positionAndDelay(self, self.motor, 14335, motorRMOD_I, use_abs, myPOSmid, myPOSlow)

    # position forward inside backlash range, relative
    def test_TC_14336(self):
        positionAndDelay(self, self.motor, 14336, motorRMOD_I, use_rel, myPOSmid, myPOSlow)

    # position forward inside backlash range, absolute
    def test_TC_14337(self):
        positionAndDelay(self, self.motor, 14337, motorRMOD_I, use_abs, myPOSlow, myPOSmid)

    # position forward inside backlash range, relative
    def test_TC_14338(self):
        positionAndDelay(self, self.motor, 14338, motorRMOD_I, use_rel, myPOSlow, myPOSmid)
