#!/usr/bin/env python
#

import unittest
import os
import sys
import time
from motor_lib import motor_lib
lib = motor_lib()
import capv_lib
import inspect
###

def lineno():
    return inspect.currentframe().f_back.f_lineno


#How we move: Absolute (without encoder) or relative (with encode via UEIP)
use_abs = 0
use_rel = 1

noFRAC   = 1.0
withFRAC = 1.5


def motorInitTC(tself, motor, tc_no, frac, encRel):
    capv_lib.capvput(motor + '.FRAC', frac)
    capv_lib.capvput(motor + '.UEIP', encRel)
    msta = int(capv_lib.capvget(motor + '.MSTA', use_monitor=False))
    print('%s:%d motorInitTC msta=%s' % (tc_no, lineno(), lib.getMSTAtext(msta)))


def setMotorStartPos(tself, motor, tc_no, startpos):
    lib.setValueOnSimulator(motor, tc_no, "fActPosition", startpos)
    # Run a status update and a sync
    lib.doSTUPandSYNC(motor, tc_no)


def jogAndBacklash(tself, motor, tc_no, frac, encRel, motorStartPos, motorEndPos, myJOGX):
    # expected and actual
    if motor.startswith('pva://'):
        mot = motor[6:]
    else:
        mot = motor
    fileName = "/tmp/" + mot.replace(':', '-') + "-" + str(tc_no)
    expFileName = fileName + ".exp"
    actFileName = fileName + ".act"

    motorInitTC(tself, motor, tc_no, frac, encRel)
    setMotorStartPos(tself, motor, tc_no, motorStartPos)
    lib.setValueOnSimulator(motor, tc_no, "log", actFileName)
    if myJOGX == 'JOGF':
        myDirection = 1
    elif myJOGX == 'JOGR':
        myDirection = 0
    else:
        assert(0)
    #
    capv_lib.capvput(motor + '.' + myJOGX, 1)
    time.sleep(1)
    lib.setValueOnSimulator(motor, tc_no, "fActPosition", motorEndPos)
    capv_lib.capvput(motor + '.' + myJOGX, 0)
    resW = lib.waitForMipZero(motor, tc_no, 12)
    lib.setValueOnSimulator(motor, tc_no, "dbgCloseLogFile", "1")

    dbgFileName = None
    lib.writeExpFileJOG_BDST(motor, tc_no, dbgFileName, expFileName, myDirection, frac, encRel, motorStartPos, motorEndPos)
    time_to_wait = 100
    lib.waitForStop(motor, tc_no, time_to_wait)
    lib.cmpUnlinkExpectedActualFile(None, expFileName, actFileName)

class Test(unittest.TestCase):
    motor = os.getenv("TESTEDMOTORAXIS")
    capv_lib.capvput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])

    myPOSlow = lib.myPOSlow
    myPOSmid = lib.myPOSmid
    myPOShig = lib.myPOShig

    def test_TC_14100(self):
        motor = self.motor
        tc_no = "TC-14100"
        lib.initializeMotorRecordSimulatorAxis(motor, tc_no)
        lib.motorInitAllForBDST(self.motor, tc_no)

    # JOG forward & backlash compensation, absolute
    def test_TC_14111(self):
        jogAndBacklash(self, self.motor, 14111, noFRAC, use_abs, self.myPOSlow, self.myPOSmid, 'JOGF')

    # JOG forward & backlash compensation, relative
    def test_TC_14112(self):
        jogAndBacklash(self, self.motor, 14112, noFRAC, use_rel, self.myPOSmid, self.myPOSlow, 'JOGF')

    # JOG backward & backlash compensation, absolute
    def test_TC_14121(self):
        jogAndBacklash(self, self.motor, 14121, noFRAC, use_abs, self.myPOSlow, self.myPOSmid, 'JOGR')

    # JOG backward & backlash compensation, relative
    def test_TC_14122(self):
        jogAndBacklash(self, self.motor, 14122, noFRAC, use_rel, self.myPOSmid, self.myPOSlow, 'JOGR')

    # JOG forward & backlash compensation, absolute
    def test_TC_14131(self):
        jogAndBacklash(self, self.motor, 14131, withFRAC, use_abs, self.myPOSlow, self.myPOSmid, 'JOGF')

    # JOG forward & backlash compensation, relative
    def test_TC_14132(self):
        jogAndBacklash(self, self.motor, 14132, withFRAC, use_rel, self.myPOSmid, self.myPOSlow, 'JOGF')

    # JOG backward & backlash compensation, absolute
    def test_TC_14141(self):
        jogAndBacklash(self, self.motor, 14141, withFRAC, use_abs, self.myPOSlow, self.myPOSmid, 'JOGR')

    # JOG backward & backlash compensation, relative
    def test_TC_14142(self):
        jogAndBacklash(self, self.motor, 14142, withFRAC, use_rel, self.myPOSmid, self.myPOSlow, 'JOGR')
