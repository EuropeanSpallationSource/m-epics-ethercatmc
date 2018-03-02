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

myFRAC   = 1.5

motorRMOD_D = 0 # "Default"
motorRMOD_A = 1 # "Arithmetic"
motorRMOD_G = 2 # "Geometric"
motorRMOD_I = 3 # "In-Position"

#
#How we move: Absolute (without encoder) or relative (with encode via UEIP)
use_abs = 0
use_rel = 1
# Note: motorRMOD_I is always absolute !




def motorInitTC(tself, motor, tc_no, rmod, encRel):
    epics.caput(motor + '.RMOD', rmod)
    epics.caput(motor + '.UEIP', encRel)


def setMotorStartPos(tself, motor, tc_no, startpos):
    lib.setValueOnSimulator(motor, tc_no, "fActPosition", startpos)
    # Run a status update and a sync
    epics.caput(motor + '.STUP', 1)
    epics.caput(motor + '.SYNC', 1)




def positionAndBacklash(tself, motor, tc_no, rmod, encRel, motorStartPos, motorEndPos):
    ###########
    # expected and actual
    fileName = "/tmp/" + motor.replace(':', '-') + "-" + str(tc_no)
    expFileName = fileName + ".exp"
    actFileName = fileName + ".act"
    dbgFileName = fileName + ".dbg"

    motorInitTC(tself, motor, tc_no, rmod, encRel)
    setMotorStartPos(tself, motor, tc_no, motorStartPos)
    lib.setValueOnSimulator(motor, tc_no, "bManualSimulatorMode", 1)
    time.sleep(2)
    lib.setValueOnSimulator(motor, tc_no, "log", actFileName)
    time.sleep(2)
    #
    epics.caput(motor + '.VAL', motorEndPos, wait=True)
    lib.setValueOnSimulator(motor, tc_no, "dbgCloseLogFile", "1")
    time.sleep(2)
    lib.setValueOnSimulator(motor, tc_no, "bManualSimulatorMode", 0)

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
    lib.writeExpFileRMOD_X(motor, tc_no, rmod, dbgFile, expFile, cnt, encRel, motorStartPos, motorEndPos)

    expFile.close()
    if dbgFileName != None:
        dbgFile.close()
    lib.setValueOnSimulator(motor, tc_no, "dbgCloseLogFile", "1")

    lib.cmpUnlinkExpectedActualFile(dbgFileName, expFileName, actFileName)



class Test(unittest.TestCase):
    motor = os.getenv("TESTEDMOTORAXIS")
    myPOSlow = lib.myPOSlow
    myPOSmid = lib.myPOSmid
    myPOShig = lib.myPOShig

    def test_TC_14400(self):
        lib.motorInitAllForBDST(self.motor, 14400)
        epics.caput(self.motor + '.FRAC', myFRAC)


    # motorRMOD_D = 0 # "Default"
    # position forward & backlash compensation, absolute
    def test_TC_14401(self):
        positionAndBacklash(self, self.motor, 14401, motorRMOD_D, use_abs, self.myPOSlow, self.myPOShig)

    # position forward & backlash compensation, relative
    def test_TC_14402(self):
        positionAndBacklash(self, self.motor, 14402, motorRMOD_D, use_rel, self.myPOSlow, self.myPOShig)

    # position backward & backlash compensation, absolute
    def test_TC_14403(self):
        positionAndBacklash(self, self.motor, 14403, motorRMOD_D, use_abs, self.myPOShig, self.myPOSlow)

    # position backward & backlash compensation, relative
    def test_TC_14404(self):
        positionAndBacklash(self, self.motor, 14404, motorRMOD_D, use_rel, self.myPOShig, self.myPOSlow)

    # position forward inside backlash range, absolute
    def test_TC_14405(self):
        positionAndBacklash(self, self.motor, 14405, motorRMOD_D, use_abs, self.myPOSmid, self.myPOSlow)

    # position forward inside backlash range, relative
    def test_TC_14406(self):
        positionAndBacklash(self, self.motor, 14406, motorRMOD_D, use_rel, self.myPOSmid, self.myPOSlow)

    # position forward inside backlash range, absolute
    def test_TC_14407(self):
        positionAndBacklash(self, self.motor, 14407, motorRMOD_D, use_abs, self.myPOSlow, self.myPOSmid)

    # position forward inside backlash range, relative
    def test_TC_14408(self):
        positionAndBacklash(self, self.motor, 14408, motorRMOD_D, use_rel, self.myPOSlow, self.myPOSmid)

    ###############################################################################
    # motorRMOD_A
    # position forward & backlash compensation, absolute
    def test_TC_14411(self):
        positionAndBacklash(self, self.motor, 14411, motorRMOD_A, use_abs, self.myPOSlow, self.myPOShig)

    # position forward & backlash compensation, relative
    def test_TC_14412(self):
        positionAndBacklash(self, self.motor, 14412, motorRMOD_A, use_rel, self.myPOSlow, self.myPOShig)

    # position backward & backlash compensation, absolute
    def test_TC_14413(self):
        positionAndBacklash(self, self.motor, 14413, motorRMOD_A, use_abs, self.myPOShig, self.myPOSlow)

    # position backward & backlash compensation, relative
    def test_TC_14414(self):
        positionAndBacklash(self, self.motor, 14414, motorRMOD_A, use_rel, self.myPOShig, self.myPOSlow)

    # position forward inside backlash range, absolute
    def test_TC_14415(self):
        positionAndBacklash(self, self.motor, 14415, motorRMOD_A, use_abs, self.myPOSmid, self.myPOSlow)

    # position forward inside backlash range, relative
    def test_TC_14416(self):
        positionAndBacklash(self, self.motor, 14416, motorRMOD_A, use_rel, self.myPOSmid, self.myPOSlow)

    # position forward inside backlash range, absolute
    def test_TC_14417(self):
        positionAndBacklash(self, self.motor, 14417, motorRMOD_A, use_abs, self.myPOSlow, self.myPOSmid)

    # position forward inside backlash range, relative
    def test_TC_14418(self):
        positionAndBacklash(self, self.motor, 14418, motorRMOD_A, use_rel, self.myPOSlow, self.myPOSmid)

    ###############################################################################
    # motorRMOD_G
    # position forward & backlash compensation, absolute
    def test_TC_14421(self):
        positionAndBacklash(self, self.motor, 14421, motorRMOD_G, use_abs, self.myPOSlow, self.myPOShig)

    # position forward & backlash compensation, relative
    def test_TC_14422(self):
        positionAndBacklash(self, self.motor, 14422, motorRMOD_G, use_rel, self.myPOSlow, self.myPOShig)

    # position backward & backlash compensation, absolute
    def test_TC_14423(self):
        positionAndBacklash(self, self.motor, 14423, motorRMOD_G, use_abs, self.myPOShig, self.myPOSlow)

    # position backward & backlash compensation, relative
    def test_TC_14424(self):
        positionAndBacklash(self, self.motor, 14424, motorRMOD_G, use_rel, self.myPOShig, self.myPOSlow)

    # position forward inside backlash range, absolute
    def test_TC_14425(self):
        positionAndBacklash(self, self.motor, 14425, motorRMOD_G, use_abs, self.myPOSmid, self.myPOSlow)

    # position forward inside backlash range, relative
    def test_TC_14426(self):
        positionAndBacklash(self, self.motor, 14426, motorRMOD_G, use_rel, self.myPOSmid, self.myPOSlow)

    # position forward inside backlash range, absolute
    def test_TC_14427(self):
        positionAndBacklash(self, self.motor, 14427, motorRMOD_G, use_abs, self.myPOSlow, self.myPOSmid)

    # position forward inside backlash range, relative
    def test_TC_14428(self):
        positionAndBacklash(self, self.motor, 14428, motorRMOD_G, use_rel, self.myPOSlow, self.myPOSmid)


    ###############################################################################
    # motorRMOD_I
    # position forward & backlash compensation, absolute
    def test_TC_14431(self):
        positionAndBacklash(self, self.motor, 14431, motorRMOD_I, use_abs, self.myPOSlow, self.myPOShig)

    # position forward & backlash compensation, relative
    def test_TC_14432(self):
        positionAndBacklash(self, self.motor, 14432, motorRMOD_I, use_rel, self.myPOSlow, self.myPOShig)

    # position backward & backlash compensation, absolute
    def test_TC_14433(self):
        positionAndBacklash(self, self.motor, 14433, motorRMOD_I, use_abs, self.myPOShig, self.myPOSlow)

    # position backward & backlash compensation, relative
    def test_TC_14434(self):
        positionAndBacklash(self, self.motor, 14434, motorRMOD_I, use_rel, self.myPOShig, self.myPOSlow)

    # position forward inside backlash range, absolute
    def test_TC_14435(self):
        positionAndBacklash(self, self.motor, 14435, motorRMOD_I, use_abs, self.myPOSmid, self.myPOSlow)

    # position forward inside backlash range, relative
    def test_TC_14436(self):
        positionAndBacklash(self, self.motor, 14436, motorRMOD_I, use_rel, self.myPOSmid, self.myPOSlow)

    # position forward inside backlash range, absolute
    def test_TC_14437(self):
        positionAndBacklash(self, self.motor, 14437, motorRMOD_I, use_abs, self.myPOSlow, self.myPOSmid)

    # position forward inside backlash range, relative
    def test_TC_14438(self):
        positionAndBacklash(self, self.motor, 14438, motorRMOD_I, use_rel, self.myPOSlow, self.myPOSmid)

